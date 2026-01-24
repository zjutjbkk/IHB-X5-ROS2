#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

// 消息定义
#include "ihb_msgs/msg/single_arm_status.hpp"
#include "ihb_msgs/msg/arm_target.hpp"

// 底层库头文件 (请确保路径正确)
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/servo_motorJoint.h"
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/muscle_joint.h"
#include "CanDriver.hpp"

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std::chrono_literals;

class IhbSlaveArmController : public rclcpp::Node
{
public:
    IhbSlaveArmController()
    : Node("ihb_slave_arm_controller"), 
      g_running_(true),
      ser1_(1), ser2_(2)
    {
        RCLCPP_INFO(this->get_logger(), ">>> 正在初始化 IHB [从臂跟随控制] 节点 (对齐 JointState 版本)...");

        try {
            // 1. 硬件初始化 (CAN1)
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);

            // 2. 配置关节对象
            setup_hardware();

            // 3. ROS 通信逻辑
            publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>("slave_arm_status", 10);
            subscriber_ = this->create_subscription<ihb_msgs::msg::ArmTarget>(
                "slave_arm_target", 10, std::bind(&IhbSlaveArmController::target_callback, this, std::placeholders::_1));

            // 4. 启动核心工作线程 (接收与 PID)
            receive_thread_ = std::thread(&IhbSlaveArmController::receive_can_frames, this);
            pid_thread_ = std::thread(&IhbSlaveArmController::pid_control_loop, this);

            // 5. 定时器
            query_timer_ = this->create_wall_timer(50ms, std::bind(&IhbSlaveArmController::poll_motor_callback, this));
            publish_timer_ = this->create_wall_timer(100ms, std::bind(&IhbSlaveArmController::publish_status_callback, this));

            RCLCPP_INFO(this->get_logger(), "🚀 对齐完成：数据映射已匹配 JointStatePublisher。");

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "❌ 初始化失败: %s", e.what());
        }
    }

    ~IhbSlaveArmController() {
        g_running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (pid_thread_.joinable()) pid_thread_.join();
        hall_sensors_.stop();
        ser1_.close(); ser2_.close();
        RCLCPP_INFO(this->get_logger(), "从臂系统已安全关闭");
    }

private:
    void init_can_bus() {
        system("sudo ip link set can1 down");
        system("sudo ip link set can1 type can bitrate 1000000");
        system("sudo ip link set can1 up");
        std::this_thread::sleep_for(500ms);
    }

    void setup_hardware() {
        for(int i=0; i<4; i++) {
            muscle_joints_[i].setId(i);
            muscle_joints_[i].setCanDriver(shared_can_driver_);
            // muscle_joints_[i].setPidParameters(30.0, 1.8, 120.0); 
            muscle_joints_[i].setPidParameters(0.0, 0.0, 0.0,0.0, 0.0, 0.0);
        }
        ser1_.setCanDriver(shared_can_driver_);
        ser2_.setCanDriver(shared_can_driver_);
        ser1_.start(); ser2_.start();
        hall_sensors_.start(0);
    }

    void receive_can_frames() {
        int lb = 0; setsockopt(shared_can_driver_->getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &lb, sizeof(lb));
        uint8_t data[8];
        while (g_running_ && rclcpp::ok()) {
            if (shared_can_driver_->waitForMessages(2ms)) {
                try {
                    auto msg = shared_can_driver_->readMessage();
                    uint32_t id = msg.getCanId();
                    std::memcpy(data, msg.getFrameData().data(), 8);

                    if      (id == 0x141) ser1_.processCanMessage(data);
                    else if (id == 0x142) ser2_.processCanMessage(data);
                    else if ((id >> 4) == 0x18) {
                        uint8_t idx = id & 0x0F;
                        if(idx < 4) muscle_joints_[idx].processCanMessage(data);
                    }
                } catch (...) {}
            }
        }
    }

    void pid_control_loop() {
        while (g_running_ && rclcpp::ok()) {
            std::vector<float> halls = hall_sensors_.getAllAngles();
            if (halls.size() >= 8) {
                std::lock_guard<std::mutex> lock(mtx_);
                for (int i = 0; i < 4; i++) {
                    // 硬件 PID 计算保持原样
                    muscle_joints_[i].processAngle(halls[i], halls[7-i]);
                    muscle_joints_[i].update(); 
                }
            }
            std::this_thread::sleep_for(25ms);
        }
    }

    void target_callback(const ihb_msgs::msg::ArmTarget::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (msg->servo_joints.size() >= 2) {
            ser1_.setPositonWithSpeedLimit(msg->servo_joints[0].angle, msg->servo_joints[0].speed_limit);
            ser2_.setPositonWithSpeedLimit(msg->servo_joints[1].angle, msg->servo_joints[1].speed_limit);
        }
        for (size_t i = 0; i < msg->muscle_joints.size() && i < 4; i++) {
            muscle_joints_[i].setTargetAngle(msg->muscle_joints[i].angle[0], msg->muscle_joints[i].angle[1]);
        }
    }

    void poll_motor_callback() {
        ser1_.readAngle();
        std::this_thread::sleep_for(2ms);
        ser2_.readAngle();
    }

    // --- 核心修改部分：对齐上位机 JointStatePublisher ---
    void publish_status_callback() {
        auto msg = ihb_msgs::msg::SingleArmStatus();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link"; // 统一 frame_id

        // 1. 获取霍尔角度传感器数据
        std::vector<float> halls = hall_sensors_.getAllAngles();
        
        // 确保数组大小满足消息定义 (通常为 4 个肌肉关节)
        // 在 JointStatePublisher 中，它是遍历 0~3 的索引
        for (int i = 0; i < 4; i++) {
            msg.muscle_joint[i].id = i; 
            if (halls.size() >= 8) {
                // 注意：上位机代码中使用了 -msg->muscle_joint[i].angle[0]，
                // 这里我们发送原始霍尔值，上位机负责负号和弧度转换。
                msg.muscle_joint[i].angle[0] = static_cast<int16_t>(halls[i]);
                msg.muscle_joint[i].angle[1] = static_cast<int16_t>(halls[7-i]);
            }
        }

        // 2. 获取伺服电机角度 (需对应 ID 和顺序)
        // 上位机逻辑：
        // joint_state_msg->position[21] (Right Shoulder 1) = servo_motor[1].angle
        // joint_state_msg->position[22] (Right Shoulder 2) = servo_motor[0].angle
        
        // 我们需要确保电机 ID 顺序正确。假设 ser1 是 shoulder2，ser2 是 shoulder1
        msg.servo_motor[0].id = 1;
        msg.servo_motor[0].angle = -ser1_.MAngle(); // 电机反馈通常取反以匹配机械安装方向
        
        msg.servo_motor[1].id = 2;
        msg.servo_motor[1].angle = -ser2_.MAngle();

        // 3. 发布话题
        publisher_->publish(msg);

        // 4. 终端监控
        static int print_count = 0;
        if (print_count++ % 10 == 0) { 
            printf("\033[2J\033[H"); 
            RCLCPP_INFO(this->get_logger(), 
                "\n==================== [ 从臂硬件状态 -> ROS2 映射 ] ====================\n"
                "伺服电机 (Servo -> Joint 21,22): \n"
                "  [ID:1 -> Motor1] 映射至 Servo[0]: %6.2f° \n"
                "  [ID:2 -> Motor2] 映射至 Servo[1]: %6.2f° \n"
                "----------------------------------------------------------------------\n"
                "气动肌肉 (Muscle -> Joint 23-30): \n"
                "  关节 1 (ID:0) -> A: %6.1f | B: %6.1f \n"
                "  关节 2 (ID:1) -> A: %6.1f | B: %6.1f \n"
                "  关节 3 (ID:2) -> A: %6.1f | B: %6.1f \n"
                "  关节 4 (ID:3) -> A: %6.1f | B: %6.1f \n"
                "======================================================================",
                msg.servo_motor[0].angle, msg.servo_motor[1].angle,
                halls[0], halls[7], halls[1], halls[6], halls[2], halls[5], halls[3], halls[4]);
        }
    }

    std::atomic<bool> g_running_;
    std::thread receive_thread_, pid_thread_;
    std::mutex mtx_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;
    
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