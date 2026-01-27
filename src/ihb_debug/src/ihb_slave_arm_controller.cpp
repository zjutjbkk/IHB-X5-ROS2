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

#include "ihb_msgs/msg/ihb_arm_status.hpp"
#include "ihb_msgs/msg/ihb_target.hpp"
#include "hall_sensors.h"
#include "servo_motorJoint.h"
#include "muscle_joint.h"
#include "CanDriver.hpp"

using namespace std::chrono_literals;

class IhbSlaveArmController : public rclcpp::Node
{
public:
    IhbSlaveArmController()
        : Node("ihb_slave_arm_controller"), g_running_(true)
    {
        RCLCPP_INFO(this->get_logger(), ">>> IHB 从臂双臂控制器启动 [16路传感器模式] <<<");

        try {
            init_csv_logger();
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);
            setup_hardware();

            subscriber_ = this->create_subscription<ihb_msgs::msg::IhbTarget>(
                "arm_target", 10, std::bind(&IhbSlaveArmController::target_callback, this, std::placeholders::_1));
            
            // 发布从臂状态 (建议根据需要检查消息定义是否支持 8 肌肉关节数据)
            publisher_ = this->create_publisher<ihb_msgs::msg::IhbArmStatus>("slave_arm_status", 10);

            receive_thread_ = std::thread(&IhbSlaveArmController::receive_can_frames, this);
            pid_thread_ = std::thread(&IhbSlaveArmController::pid_control_loop, this);

            query_timer_ = this->create_wall_timer(40ms, std::bind(&IhbSlaveArmController::poll_motor_callback, this));
            publish_timer_ = this->create_wall_timer(60ms, std::bind(&IhbSlaveArmController::publish_status_callback, this));

            // 电机初始位置归零
            ser1_.setPositonWithSpeedLimit(0.0, 100.0);
            ser2_.setPositonWithSpeedLimit(0.0, 100.0);
            ser3_.setPositonWithSpeedLimit(0.0, 100.0);
            ser4_.setPositonWithSpeedLimit(0.0, 100.0);
        }
        catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "❌ 初始化失败: %s", e.what());
        }
    }

    ~IhbSlaveArmController() {
        g_running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (pid_thread_.joinable()) pid_thread_.join();
        if (log_file_.is_open()) log_file_.close();
        for (auto &mj : muscle_joints_) mj.setPressure(0, 0, 0, 0);
        hall_sensors_.stop();
        ser1_.close(); ser2_.close(); ser3_.close(); ser4_.close();
    }

private:
    void init_csv_logger() {
        system("mkdir -p log");
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream ss;
        ss << "log/slave_dual_arm_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S") << ".csv";
        log_file_.open(ss.str(), std::ios::out);
        if (log_file_.is_open()) {
            log_file_ << std::fixed << std::setprecision(2);
            log_file_ << "Time_ms,M_Joint_1_T,M_Joint_2_T,M_Joint_3_T,M_Joint_4_T,M_Joint_5_T,M_Joint_6_T,M_Joint_7_T,M_Joint_8_T,"
                      << "H0,H1,H2,H3,H4,H5,H6,H7,H8,H9,H10,H11,H12,H13,H14,H15,"
                      << "S1,S2,S3,S4" << std::endl;
        }
    }

    void target_callback(const ihb_msgs::msg::IhbTarget::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        // 1. 电机动态分发 (适配主端 ID 1-4)
        for (const auto & servo : msg->servo_joint) {
            if (servo.id == 1) ser1_.setPositonWithSpeedLimit(servo.angle, servo.speed_limit);
            else if (servo.id == 2) ser2_.setPositonWithSpeedLimit(servo.angle, servo.speed_limit);
            else if (servo.id == 3) ser3_.setPositonWithSpeedLimit(servo.angle, servo.speed_limit);
            else if (servo.id == 4) ser4_.setPositonWithSpeedLimit(servo.angle, servo.speed_limit);
        }

        // 2. 肌肉关节动态分发 (适配 8 个肌肉关节)
        for (const auto & mj_target : msg->muscle_joint) {
            uint8_t id = mj_target.id;
            if (id >= 1 && id <= 8) {
                muscle_joints_[id-1].setTargetAngle(mj_target.angle[0], mj_target.angle[1]);
                if (msg->cmd == 2) { // 仅在命令为 2 时更新 PID
                    muscle_joints_[id-1].setPidParameters(
                        mj_target.kp[0], mj_target.ki[0], mj_target.kd[0],
                        mj_target.kp[1], mj_target.ki[1], mj_target.kd[1]);
                }
            }
        }
    }
    void pid_control_loop() {
    while (g_running_ && rclcpp::ok()) {
        // 从端 16 路传感器数据
        std::vector<float> halls = hall_sensors_.getAllAngles();
        
        if (halls.size() >= 16) {
            std::lock_guard<std::mutex> lock(mtx_);
            
            for (int i = 0; i < 8; i++) {
                if (i < 4) {
                    muscle_joints_[i].processAngle(-halls[i], halls[7 - i]);
                } 
                else {
                    int offset = i - 4;
                    muscle_joints_[i].processAngle(-halls[8 + offset], halls[15 - offset]);
                }
                
                // 执行 PID 计算并输出 CAN 控制指令
                muscle_joints_[i].update();
            }
        }
        // 50Hz 的控制频率 (20ms) 对气动/肌肉控制通常是比较理想的
        std::this_thread::sleep_for(20ms);
    }
}

void publish_status_callback() {
    auto msg = ihb_msgs::msg::IhbArmStatus();
    msg.header.stamp = this->now();
    std::vector<float> halls = hall_sensors_.getAllAngles();

    // --- 1. 填充 8 个肌肉关节数据 (对应从端 16 个传感器) ---
    if (halls.size() >= 16) {
        for (int i = 0; i < 8; i++) {
            msg.muscle_joint[i].id = i + 1;
            if (i < 4) {
                // 右臂肌肉关节 (Joint 1-4): 映射从端传感器 0-7
                // 对应逻辑: i 和 7-i
                msg.muscle_joint[i].angle[0] = -static_cast<int16_t>(halls[i]);
                msg.muscle_joint[i].angle[1] = static_cast<int16_t>(halls[7 - i]);
            } else {
                // 左臂肌肉关节 (Joint 5-8): 映射从端传感器 8-15
                // 对应逻辑: 8+offset 和 15-offset
                int offset = i - 4;
                msg.muscle_joint[i].angle[0] = -static_cast<int16_t>(halls[8 + offset]);
                msg.muscle_joint[i].angle[1] = static_cast<int16_t>(halls[15 - offset]);
            }
        }
    }
        msg.servo_motor[0].angle = -ser1_.MAngle();
        msg.servo_motor[1].angle = -ser2_.MAngle();
        msg.servo_motor[2].angle = -ser3_.MAngle();
        msg.servo_motor[3].angle = -ser4_.MAngle();
        publisher_->publish(msg);

        // --- CSV 日志记录所有 16 个传感器和 4 个电机 ---
        if (log_file_.is_open()) {
            std::lock_guard<std::mutex> lock(mtx_);
            uint64_t current_ms = (static_cast<uint64_t>(msg.header.stamp.sec) * 1000ULL) +
                                  (static_cast<uint64_t>(msg.header.stamp.nanosec) / 1000000ULL);
            if (start_time_ms_ == 0) start_time_ms_ = current_ms;
            
            log_file_ << (current_ms - start_time_ms_);
            // 8个肌肉目标
            for (int i = 0; i < 8; i++) log_file_ << "," << muscle_joints_[i].getTargetAngle1();
            // 16个传感器反馈
            for (int i = 0; i < 16; i++) log_file_ << "," << (halls.size() > i ? halls[i] : 0.0);
            // 4个电机角度反馈
            log_file_ << "," << ser1_.MAngle() << "," << ser2_.MAngle() 
                      << "," << ser3_.MAngle() << "," << ser4_.MAngle() << std::endl;
        }
    }

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
                    
                    if (id == 0x141) ser1_.processCanMessage(data);
                    else if (id == 0x142) ser2_.processCanMessage(data);
                    else if (id == 0x143) ser3_.processCanMessage(data);
                    else if (id == 0x144) ser4_.processCanMessage(data);
                    else if ((id >> 4) == 0x18) {
                        uint8_t idx = id & 0x0F;
                        if(idx >= 1 && idx <= 8) muscle_joints_[idx-1].processCanMessage(data);
                    }
                } catch (...) {}
            }
        }
    }

    void poll_motor_callback() {
        ser1_.readAngle(); std::this_thread::sleep_for(1ms);
        ser2_.readAngle(); std::this_thread::sleep_for(1ms);
        ser3_.readAngle(); std::this_thread::sleep_for(1ms);
        ser4_.readAngle();
    }

    void init_can_bus() {
        system("sudo ip link set can1 down");
        system("sudo ip link set can1 type can bitrate 1000000");
        system("sudo ip link set can1 up");
    }

    void setup_hardware() {
        for (int i = 0; i < 8; i++) {
            muscle_joints_[i].setId(i + 1);
            muscle_joints_[i].setCanDriver(shared_can_driver_);
            muscle_joints_[i].setPidParameters(0,0,0,0,0,0);
        }
        ser1_.setCanDriver(shared_can_driver_); ser2_.setCanDriver(shared_can_driver_);
        ser3_.setCanDriver(shared_can_driver_); ser4_.setCanDriver(shared_can_driver_);
        ser1_.start(); ser2_.start(); ser3_.start(); ser4_.start();
        
        // 关键：启动传感器采集，确保支持 16 路输入
        hall_sensors_.start(0); 
    }

    std::atomic<bool> g_running_;
    std::thread receive_thread_, pid_thread_;
    std::mutex mtx_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;
    std::ofstream log_file_;
    uint64_t start_time_ms_ = 0;

    ihb::HallSensors hall_sensors_;
    ihb::MuscleJoint muscle_joints_[8];
    ihb::ServoMotorJoint ser1_{1}, ser2_{2}, ser3_{3}, ser4_{4};

    rclcpp::Publisher<ihb_msgs::msg::IhbArmStatus>::SharedPtr publisher_;
    rclcpp::Subscription<ihb_msgs::msg::IhbTarget>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr query_timer_, publish_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IhbSlaveArmController>());
    rclcpp::shutdown();
    return 0;
}