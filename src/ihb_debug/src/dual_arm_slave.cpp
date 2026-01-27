#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// --- 引入自定义消息 ---
#include "ihb_msgs/msg/ihb_status.hpp"
#include "ihb_msgs/msg/ihb_target.hpp"
// 引入子消息类型 (通常包含在上述头文件中，但为了明确字段结构，需确认你的 msg生成情况)
// 假设自动生成了如下结构:
// ihb_msgs::msg::ServoMotorJointStatus
// ihb_msgs::msg::MuscleJointStatus

// --- 引入底层驱动库 ---
#include "hall_sensors.h"
#include "servo_motorJoint.h"
#include "muscle_joint.h"
#include "CanDriver.hpp"

using namespace std::chrono_literals;

class DualArmSlave : public rclcpp::Node {
public:
    DualArmSlave()
    : Node("ihb_slave_dual_arm_controller"), g_running_(true), 
      ser1_(1), ser2_(2), ser3_(3), ser4_(4) 
    {
        RCLCPP_INFO(this->get_logger(), ">>> IHB 双臂底层控制器启动 [适配新Msg定义] <<<");

        try {
            // 1. 初始化日志
            init_csv_logger();

            // 2. 硬件初始化
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);
            setup_hardware();

            // 3. 订阅指令 (IhbTarget)
            subscriber_ = this->create_subscription<ihb_msgs::msg::IhbTarget>(
                "arm_target", 10, std::bind(&DualArmSlave::target_callback, this, std::placeholders::_1));
            
            // 4. 发布状态 (IhbStatus)
            publisher_ = this->create_publisher<ihb_msgs::msg::IhbStatus>("slave_arm_status", 10);

            // 5. 线程与定时器
            receive_thread_ = std::thread(&DualArmSlave::receive_can_frames, this);
            pid_thread_ = std::thread(&DualArmSlave::pid_control_loop, this);

            // 30ms 轮询电机位置, 50ms 发布状态
            query_timer_ = this->create_wall_timer(30ms, std::bind(&DualArmSlave::poll_motor_callback, this));
            publish_timer_ = this->create_wall_timer(50ms, std::bind(&DualArmSlave::publish_status_callback, this));

            // 电机初始限位 (0-360度)
            ser1_.setPositonWithSpeedLimit(0.0, 360.0);
            ser2_.setPositonWithSpeedLimit(0.0, 360.0);
            ser3_.setPositonWithSpeedLimit(0.0, 360.0);
            ser4_.setPositonWithSpeedLimit(0.0, 360.0);

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "❌ 初始化失败: %s", e.what());
        }
    }

    ~DualArmSlave() {
        g_running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (pid_thread_.joinable()) pid_thread_.join();
        if (log_file_.is_open()) log_file_.close(); 

        for (auto & mj : muscle_joints_) mj.setPressure(0, 0, 0, 0);
        hall_sensors_.stop();
        ser1_.close(); ser2_.close(); ser3_.close(); ser4_.close();
        RCLCPP_INFO(this->get_logger(), "✅ 系统安全退出。");
    }

private:
    // --- 日志记录 (CSV) ---
    void init_csv_logger() {
        system("mkdir -p log");
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream ss;
        ss << "log/dual_full_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S") << ".csv";
        
        log_file_.open(ss.str(), std::ios::out);
        if (log_file_.is_open()) {
            log_file_ << std::fixed << std::setprecision(2);
            log_file_ << "Time_ms";
            // 记录8个肌肉关节的目标与实际
            for(int i=1; i<=8; i++) log_file_ << ",T_A" << i << "_1,T_A" << i << "_2";
            for(int i=1; i<=8; i++) log_file_ << ",Act_A" << i << "_1,Act_A" << i << "_2";
            // 记录4个电机
            log_file_ << ",M1,M2,M3,M4" << std::endl;
        }
    }

    // --- 状态发布回调 (IhbStatus) ---
    void publish_status_callback() {
        auto msg = ihb_msgs::msg::IhbStatus();
        msg.header.stamp = this->now();
        // frame_id 可根据需要填写
        msg.header.frame_id = "base_link"; 

        // 1. 填充步进电机 (目前底层代码无步进电机驱动，填默认值)
        // msg.stepper_motor.angle = 0.0;
        // msg.stepper_motor.speed = 0.0;

        // 2. 填充力传感器 (32个，默认0)
        for(int i=0; i<32; i++) {
            msg.force_sensor[i].id = i + 1;
            msg.force_sensor[i].force = 0.0;
        }

        std::vector<float> halls = hall_sensors_.getAllFilteredAngles();

        // 3. 填充 8 个肌肉关节 (MuscleJointStatus[8])
        // Msg定义中是固定数组，直接遍历下标 0-7
        for (int i = 0; i < 8; i++) {
            msg.muscle_joint[i].id = i + 1;
            
            if (halls.size() >= 16) {
                if (i < 4) {
                    // --- 右臂 (Joint 1-4) ---
                    // 映射: Hall [i] 和 [7-i]
                    msg.muscle_joint[i].angle[0] = -static_cast<int16_t>(halls[i]);
                    msg.muscle_joint[i].angle[1] = static_cast<int16_t>(halls[7-i]);
                } else {
                    // --- 左臂 (Joint 5-8) ---
                    // 映射: Hall [8 + (i-4)] 和 [15 - (i-4)]
                    int offset = i - 4;
                    msg.muscle_joint[i].angle[0] = -static_cast<int16_t>(halls[8 + offset]);
                    msg.muscle_joint[i].angle[1] = static_cast<int16_t>(halls[15 - offset]);
                }
            } else {
                msg.muscle_joint[i].angle[0] = 0;
                msg.muscle_joint[i].angle[1] = 0;
            }
            
            // 可以选择性回传当前气压值 (如果有读取功能)
            // msg.muscle_joint[i].pressure[0] = ...
        }

        // 4. 填充 4 个伺服电机 (ServoMotorJointStatus[4])
        // 获取角度并填充
        float m_angles[4];
        m_angles[0] = -ser1_.MAngle();
        m_angles[1] = -ser2_.MAngle();
        m_angles[2] = -ser3_.MAngle();
        m_angles[3] = -ser4_.MAngle();

        for(int i=0; i<4; i++) {
            msg.servo_motor[i].id = i + 1;
            msg.servo_motor[i].angle = m_angles[i];
            // msg.servo_motor[i].speed = ...; // 如果有读速度
            // msg.servo_motor[i].current = ...; // 如果有读电流
        }

        publisher_->publish(msg);

        // --- 写入 CSV ---
        if (log_file_.is_open()) {
            std::lock_guard<std::mutex> lock(mtx_);
            uint64_t current_ms = (static_cast<uint64_t>(msg.header.stamp.sec) * 1000ULL) + 
                                  (static_cast<uint64_t>(msg.header.stamp.nanosec) / 1000000ULL);
            if (start_time_ms_ == 0) start_time_ms_ = current_ms;
            
            log_file_ << (current_ms - start_time_ms_);

            // 目标值
            for(int i=0; i<8; i++) {
                log_file_ << "," << muscle_joints_[i].getTargetAngle1() 
                          << "," << muscle_joints_[i].getTargetAngle2();
            }
            
            // 实际值
            for(int i=0; i<8; i++) {
                log_file_ << "," << msg.muscle_joint[i].angle[0] 
                          << "," << msg.muscle_joint[i].angle[1];
            }
            
            // 电机
            for(int i=0; i<4; i++) {
                log_file_ << "," << m_angles[i];
            }
            log_file_ << std::endl;
        }
    }

    // --- 目标回调 (IhbTarget) ---
    void target_callback(const ihb_msgs::msg::IhbTarget::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        // 1. 伺服电机控制 (ServoMotorJointTarget[4])
        // Msg中是固定数组，直接访问下标 0-3
        // 注意：IhbTarget 的 ServoMotorJointTarget 结构体里应该有 angle 和 speed_limit
        ser1_.setPositonWithSpeedLimit(msg->servo_joints[0].angle, msg->servo_joints[0].speed_limit);
        ser2_.setPositonWithSpeedLimit(msg->servo_joints[1].angle, msg->servo_joints[1].speed_limit);
        ser3_.setPositonWithSpeedLimit(msg->servo_joints[2].angle, msg->servo_joints[2].speed_limit);
        ser4_.setPositonWithSpeedLimit(msg->servo_joints[3].angle, msg->servo_joints[3].speed_limit);

        // 2. 肌肉关节控制 (MuscleJointTarget[8])
        for (int i = 0; i < 8; i++) {
            muscle_joints_[i].setTargetAngle(msg->muscle_joints[i].angle[0], msg->muscle_joints[i].angle[1]);
            
            // 如果指令是 PID_CONTROL (2)，更新 PID 参数
            if (msg->cmd == ihb_msgs::msg::IhbTarget::CMD_PID_CONTROL) {
                muscle_joints_[i].setPidParameters(
                    msg->muscle_joints[i].kp[0], msg->muscle_joints[i].ki[0], msg->muscle_joints[i].kd[0],
                    msg->muscle_joints[i].kp[1], msg->muscle_joints[i].ki[1], msg->muscle_joints[i].kd[1]
                );
            }
        }
    }

    // --- PID 控制循环 ---
    void pid_control_loop() {
        while (g_running_ && rclcpp::ok()) {
            std::vector<float> halls = hall_sensors_.getAllFilteredAngles();
            // 必须等待双臂共16个传感器数据就绪
            if (halls.size() >= 16) {
                std::lock_guard<std::mutex> lock(mtx_); 
                
                // --- 右臂 (Muscle 0-3) ---
                for (int i = 0; i < 4; i++) {
                    muscle_joints_[i].processAngle(-halls[i], halls[7-i]);
                    muscle_joints_[i].update(); 
                }

                // --- 左臂 (Muscle 4-7) ---
                for (int i = 0; i < 4; i++) {
                    int idx = 4 + i; // 肌肉关节索引 4,5,6,7
                    // Hall索引: 
                    // i=0(Muscle4) -> Hall 8, 15
                    // i=1(Muscle5) -> Hall 9, 14
                    muscle_joints_[idx].processAngle(-halls[8+i], halls[15-i]);
                    muscle_joints_[idx].update();
                }
            }
            std::this_thread::sleep_for(20ms);
        }
    }

    // --- CAN 接收线程 ---
    void receive_can_frames() {
        int lb = 0; 
        setsockopt(shared_can_driver_->getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &lb, sizeof(lb));
        uint8_t data[8];
        while (g_running_ && rclcpp::ok()) {
            if (shared_can_driver_->waitForMessages(2ms)) {
                try {
                    auto msg = shared_can_driver_->readMessage();
                    uint32_t id = msg.getCanId();
                    std::memcpy(data, msg.getFrameData().data(), 8);
                    
                    // 伺服电机 ID: 0x141 - 0x144
                    if (id == 0x141) ser1_.processCanMessage(data);
                    else if (id == 0x142) ser2_.processCanMessage(data);
                    else if (id == 0x143) ser3_.processCanMessage(data);
                    else if (id == 0x144) ser4_.processCanMessage(data);
                    
                    // 肌肉关节阀岛反馈 (假设 ID 为 0x180 + JointID)
                    // JointID 0-7 => CAN ID 0x180 - 0x187
                    else if ((id >> 4) == 0x18) {
                        uint8_t idx = id & 0x0F;
                        if(idx < 8) muscle_joints_[idx].processCanMessage(data);
                    }
                } catch (...) {}
            }
        }
    }

    // --- 电机轮询 ---
    void poll_motor_callback() {
        ser1_.readAngle(); std::this_thread::sleep_for(1ms);
        ser2_.readAngle(); std::this_thread::sleep_for(1ms);
        ser3_.readAngle(); std::this_thread::sleep_for(1ms);
        ser4_.readAngle();
    }

    // --- 辅助函数 ---
    void init_can_bus() {
        system("sudo ip link set can1 down");
        system("sudo ip link set can1 type can bitrate 1000000");
        system("sudo ip link set can1 up");
    }

    void setup_hardware() {
        for(int i=0; i<8; i++) {
            muscle_joints_[i].setId(i);
            muscle_joints_[i].setCanDriver(shared_can_driver_);
            // 初始PID归零，等待主端下发
            muscle_joints_[i].setPidParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        
        ser1_.setCanDriver(shared_can_driver_);
        ser2_.setCanDriver(shared_can_driver_);
        ser3_.setCanDriver(shared_can_driver_);
        ser4_.setCanDriver(shared_can_driver_);
        
        ser1_.start(); ser2_.start(); ser3_.start(); ser4_.start();
        hall_sensors_.start(0);
    }

    std::atomic<bool> g_running_;
    std::thread receive_thread_, pid_thread_;
    std::mutex mtx_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;
    std::ofstream log_file_; 
    uint64_t start_time_ms_ = 0;
    
    // 硬件对象
    ihb::HallSensors hall_sensors_;
    ihb::MuscleJoint muscle_joints_[8]; // 0-7
    ihb::ServoMotorJoint ser1_, ser2_, ser3_, ser4_;

    // 适配新的消息类型
    rclcpp::Publisher<ihb_msgs::msg::IhbStatus>::SharedPtr publisher_;
    rclcpp::Subscription<ihb_msgs::msg::IhbTarget>::SharedPtr subscriber_;
    
    rclcpp::TimerBase::SharedPtr query_timer_, publish_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualArmSlave>());
    rclcpp::shutdown();
    return 0;
}