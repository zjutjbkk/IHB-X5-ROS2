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

#include "ihb_msgs/msg/single_arm_status.hpp"
#include "ihb_msgs/msg/arm_target.hpp"
#include "hall_sensors.h"
#include "servo_motorJoint.h"
#include "muscle_joint.h"
#include "CanDriver.hpp"

using namespace std::chrono_literals;

class IhbSlaveArmController : public rclcpp::Node {
public:
    IhbSlaveArmController()
    : Node("ihb_slave_arm_controller"), g_running_(true), ser1_(1), ser2_(2) {
        RCLCPP_INFO(this->get_logger(), ">>> IHB 从臂控制器启动 [日志模式已开启] <<<");

        try {
            // 1. 初始化日志文件
            init_csv_logger();

            // 2. 硬件与通信初始化
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);
            setup_hardware();

            subscriber_ = this->create_subscription<ihb_msgs::msg::ArmTarget>(
                "arm_target", 10, std::bind(&IhbSlaveArmController::target_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>("slave_arm_status", 10);

            // 3. 开启线程与定时器
            receive_thread_ = std::thread(&IhbSlaveArmController::receive_can_frames, this);
            pid_thread_ = std::thread(&IhbSlaveArmController::pid_control_loop, this);

            query_timer_ = this->create_wall_timer(30ms, std::bind(&IhbSlaveArmController::poll_motor_callback, this));
            publish_timer_ = this->create_wall_timer(50ms, std::bind(&IhbSlaveArmController::publish_status_callback, this));

            ser1_.setPositonWithSpeedLimit(0.0, 360.0);   
            ser2_.setPositonWithSpeedLimit(0.0, 360.0);


        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "❌ 初始化失败: %s", e.what());
        }
    }

    ~IhbSlaveArmController() {
        g_running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (pid_thread_.joinable()) pid_thread_.join();
        if (log_file_.is_open()) log_file_.close(); 

        for (auto & mj : muscle_joints_) mj.setPressure(0, 0, 0, 0);
        hall_sensors_.stop();
        ser1_.close(); ser2_.close();
        RCLCPP_INFO(this->get_logger(), "✅ 系统安全退出。");
    }

private:
    void init_csv_logger() {
        system("mkdir -p log");
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream ss;
        ss << "log/pid_record_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S") << ".csv";
        
        log_file_.open(ss.str(), std::ios::out);
        if (log_file_.is_open()) {
            // 设置全局不使用科学计数法
            log_file_ << std::fixed << std::setprecision(2);
            log_file_ << "Timestamp_ms,Target_A1,Target_A2,Target_A3,Target_A4,Target_A5,Target_A6,Target_A7,Target_A8,"
                      << "Actual_A1,Actual_A2,Actual_A3,Actual_A4,Actual_A5,Actual_A6,Actual_A7,Actual_A8,"
                      << "Motor1,Motor2" << std::endl;
        }
    }

    void publish_status_callback() {
        auto msg = ihb_msgs::msg::SingleArmStatus();
        msg.header.stamp = this->now();
        std::vector<float> halls = hall_sensors_.getAllAngles();
        
        for (int i = 0; i < 4; i++) {
            msg.muscle_joint[i].id = i + 1;
            if (halls.size() >= 8) {
                msg.muscle_joint[i].angle[0] = -static_cast<int16_t>(halls[i]);
                msg.muscle_joint[i].angle[1] = static_cast<int16_t>(halls[7-i]);
            }
        }
        msg.servo_motor[0].angle = -ser1_.MAngle();
        msg.servo_motor[1].angle = -ser2_.MAngle();
        publisher_->publish(msg);

        // --- 核心修改：写入 CSV ---
if (log_file_.is_open()) {
    std::lock_guard<std::mutex> lock(mtx_);

    // 1. 计算当前总毫秒数 (绝对时间戳)
    uint64_t current_ms = (static_cast<uint64_t>(msg.header.stamp.sec) * 1000ULL) + 
                          (static_cast<uint64_t>(msg.header.stamp.nanosec) / 1000000ULL);

    // 2. 如果是第一次运行，记录初始时间
    if (start_time_ms_ == 0) {
        start_time_ms_ = current_ms;
    }

    // 3. 计算相对时间戳（从0开始递增，非常短）
    uint64_t relative_ms = current_ms - start_time_ms_;

    // 4. 写入文件 (假设你使用的是 fprintf 或 log_file_ <<)
    // 此时只打印 relative_ms，去掉了之前的 time_str
    log_file_ << relative_ms << "";

            // 记录目标值
            for(int i=0; i<4; i++) {
                log_file_ << "," << muscle_joints_[i].getTargetAngle1() 
                          << "," << muscle_joints_[i].getTargetAngle2();
            }
            
            // 记录实际值
            if (halls.size() >= 8) {
                for(int i=0; i<4; i++) {
                    log_file_ << "," << -halls[i] << "," << halls[7-i];
                }
            } else {
                for(int i=0; i<8; i++) log_file_ << ",0.00";
            }
            
            // 记录电机及换行，并强制刷新缓冲确保数据不丢失（适合农用复杂环境）
            log_file_ << "," << msg.servo_motor[0].angle 
                      << "," << msg.servo_motor[1].angle << std::endl;
        }
    }

    // ... 其余逻辑保持不变 ...
    void target_callback(const ihb_msgs::msg::ArmTarget::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (msg->servo_joints.size() >= 2) {
            ser1_.setPositonWithSpeedLimit(msg->servo_joints[0].angle, msg->servo_joints[0].speed_limit);
            ser2_.setPositonWithSpeedLimit(msg->servo_joints[1].angle, msg->servo_joints[1].speed_limit);
        }
        for (size_t i = 0; i < msg->muscle_joints.size() && i < 4; i++) {
            muscle_joints_[i].setTargetAngle(msg->muscle_joints[i].angle[0], msg->muscle_joints[i].angle[1]);
            if (msg->cmd == 2) {
                muscle_joints_[i].setPidParameters(msg->muscle_joints[i].kp, msg->muscle_joints[i].ki, msg->muscle_joints[i].kd);
            }
        }
    }

    void pid_control_loop() {
        while (g_running_ && rclcpp::ok()) {
            std::vector<float> halls = hall_sensors_.getAllAngles();
            if (halls.size() >= 8) {
                std::lock_guard<std::mutex> lock(mtx_); 
                for (int i = 0; i < 4; i++) {
                    muscle_joints_[i].processAngle(-halls[i], halls[7-i]);
                    muscle_joints_[i].update(); 
                }
            }
            std::this_thread::sleep_for(20ms);
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
                    else if ((id >> 4) == 0x18) {
                        uint8_t idx = id & 0x0F;
                        if(idx < 4) muscle_joints_[idx].processCanMessage(data);
                    }
                } catch (...) {}
            }
        }
    }

    void poll_motor_callback() {
        ser1_.readAngle();
        std::this_thread::sleep_for(1ms);
        ser2_.readAngle();
    }

    void init_can_bus() {
        system("sudo ip link set can1 down");
        system("sudo ip link set can1 type can bitrate 1000000");
        system("sudo ip link set can1 up");
    }

    void setup_hardware() {
        for(int i=0; i<4; i++) {
            muscle_joints_[i].setId(i);
            muscle_joints_[i].setCanDriver(shared_can_driver_);
            // muscle_joints_[i].setPidParameters(30.0, 1.5, 100.0);
            muscle_joints_[i].setPidParameters(0.0, 0.0, 0.0);
        }
        ser1_.setCanDriver(shared_can_driver_);
        ser2_.setCanDriver(shared_can_driver_);
        ser1_.start(); ser2_.start();
        hall_sensors_.start(0);
    }

    std::atomic<bool> g_running_;
    std::thread receive_thread_, pid_thread_;
    std::mutex mtx_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;
    std::ofstream log_file_; 
    uint64_t start_time_ms_ = 0;
    
    ihb::HallSensors hall_sensors_;
    ihb::MuscleJoint muscle_joints_[4];
    ihb::ServoMotorJoint ser1_, ser2_;

    rclcpp::Publisher<ihb_msgs::msg::SingleArmStatus>::SharedPtr publisher_;
    rclcpp::Subscription<ihb_msgs::msg::ArmTarget>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr query_timer_, publish_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IhbSlaveArmController>());
    rclcpp::shutdown();
    return 0;
}