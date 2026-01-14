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

// 底层库头文件
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/servo_motorJoint.h"

#include "CanDriver.hpp"
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std::chrono_literals;

class IhbMasterController : public rclcpp::Node
{
public:
    IhbMasterController()
    : Node("ihb_master_controller"), 
      g_running_(true),
      ser3_(3), ser4_(4) 
    {
        RCLCPP_INFO(this->get_logger(), ">>> [主臂端] 正在启动同步控制系统 (适配 ArmTarget.msg)...");

        try {
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);

            ser3_.setCanDriver(shared_can_driver_);
            ser4_.setCanDriver(shared_can_driver_);
            
            hall_sensors_.start(0); 
            ser3_.start(); ser4_.start(); 

            // 订阅和发布的配置
            target_publisher_ = this->create_publisher<ihb_msgs::msg::ArmTarget>("arm_target", 10);
            
            receive_thread_ = std::thread(&IhbMasterController::receive_can_frames, this);
            query_timer_ = this->create_wall_timer(50ms, std::bind(&IhbMasterController::poll_motor_callback, this));
            sync_timer_ = this->create_wall_timer(33ms, std::bind(&IhbMasterController::sync_control_callback, this));
            
            RCLCPP_INFO(this->get_logger(), "系统就绪。发送模式: cmd=2 (PID控制)");

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "启动失败: %s", e.what());
            throw;
        }
    }

    ~IhbMasterController() {
        g_running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        hall_sensors_.stop();
        ser3_.close(); ser4_.close(); 
    }

private:
    void init_can_bus() {
        system("sudo ip link set can1 down");
        system("sudo ip link set can1 type can bitrate 1000000");
        system("sudo ip link set can1 up");
        std::this_thread::sleep_for(100ms);
    }

    void receive_can_frames() {
        int loopback = 0;
        setsockopt(shared_can_driver_->getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
        while (g_running_ && rclcpp::ok()) {
            if (shared_can_driver_->waitForMessages(5ms)) {
                auto msg = shared_can_driver_->readMessage();
                uint8_t data[8];
                for (size_t i = 0; i < 8; i++) data[i] = msg.getFrameData()[i];
                uint32_t canId = msg.getCanId();
                if (canId == 0x143) ser3_.processCanMessage(data);
                else if (canId == 0x144) ser4_.processCanMessage(data);
            }
        }
    }

    void poll_motor_callback() {
        ser3_.readAngle();
        std::this_thread::sleep_for(2ms);
        ser4_.readAngle();
    }

    // --- 核心修改：适配固定数组格式的同步回调 ---
    void sync_control_callback() {
        auto target_msg = ihb_msgs::msg::ArmTarget();
        
        // 1. 设置控制模式 (根据你的 .msg 定义: 2 = PID控制)
        target_msg.cmd = 2; 

        // 2. 伺服电机目标映射 (主 3/4 -> 从 1/2)
        // 注意：由于是固定数组 [2]，直接使用索引 0 和 1
        target_msg.servo_joints[0].id = 1; 
        target_msg.servo_joints[0].angle = static_cast<float>(ser3_.MAngle());
        target_msg.servo_joints[0].speed_limit = 120.0; 

        target_msg.servo_joints[1].id = 2; 
        target_msg.servo_joints[1].angle = static_cast<float>(ser4_.MAngle());
        target_msg.servo_joints[1].speed_limit = 120.0;

        // 3. 肌肉关节目标映射 (主 0-7 -> 从 1-4)
        // 注意：由于是固定数组 [4]，直接循环索引 0 到 3
        std::vector<float> hall_angles = hall_sensors_.getAllAngles();
        if (hall_angles.size() >= 8) {
            for (int i = 0; i < 4; i++) {
                target_msg.muscle_joints[i].id = i + 1;
                target_msg.muscle_joints[i].angle[0] = static_cast<int16_t>(hall_angles[i]);
                target_msg.muscle_joints[i].angle[1] = static_cast<int16_t>(hall_angles[7-i]);
                
                // 注意：如果你的 MuscleJointTarget 有 kp, ki, kd 字段，
                // 且 cmd 为 2，你可能需要在这里给它们赋值，或者在从臂内部管理参数
            }
        }

        // 4. 发布
        target_publisher_->publish(target_msg);
    }

private:
    std::atomic<bool> g_running_;
    std::thread receive_thread_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;
    
    ihb::HallSensors hall_sensors_;
    ihb::ServoMotorJoint ser3_, ser4_;

    rclcpp::Publisher<ihb_msgs::msg::ArmTarget>::SharedPtr target_publisher_;
    rclcpp::TimerBase::SharedPtr query_timer_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IhbMasterController>());
    rclcpp::shutdown();
    return 0;
}