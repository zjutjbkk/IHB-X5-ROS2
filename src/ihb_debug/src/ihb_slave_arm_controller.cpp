#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

// 新增：用于处理参数更新结果的消息
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "ihb_msgs/msg/single_arm_status.hpp"
#include "ihb_msgs/msg/arm_target.hpp"

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
        RCLCPP_INFO(this->get_logger(), ">>> 正在初始化 IHB [从臂跟随控制] 节点...");

        // --- 1. 参数声明与初始读取 ---
        this->declare_parameter("muscle_p", 30.0);
        this->declare_parameter("muscle_i", 1.8);
        this->declare_parameter("muscle_d", 120.0);

        double p = this->get_parameter("muscle_p").as_double();
        double i = this->get_parameter("muscle_i").as_double();
        double d = this->get_parameter("muscle_d").as_double();

        try {
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);

            // --- 2. 配置硬件 (传入初始参数) ---
            setup_hardware(p, i, d);

            // --- 3. 注册参数动态更新回调 ---
            // 当你通过命令行或 YAML 更新参数时，这个回调会自动触发
            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&IhbSlaveArmController::on_params_changed_callback, this, std::placeholders::_1));

            publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>("slave_arm_status", 10);
            subscriber_ = this->create_subscription<ihb_msgs::msg::ArmTarget>(
                "arm_target", 10, std::bind(&IhbSlaveArmController::target_callback, this, std::placeholders::_1));

            receive_thread_ = std::thread(&IhbSlaveArmController::receive_can_frames, this);
            pid_thread_ = std::thread(&IhbSlaveArmController::pid_control_loop, this);

            query_timer_ = this->create_wall_timer(50ms, std::bind(&IhbSlaveArmController::poll_motor_callback, this));
            publish_timer_ = this->create_wall_timer(100ms, std::bind(&IhbSlaveArmController::publish_status_callback, this));

            RCLCPP_INFO(this->get_logger(), "🚀 系统就绪。当前 PID: P=%.1f, I=%.1f, D=%.1f", p, i, d);

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
    // 参数更新回调函数
    rcl_interfaces::msg::SetParametersResult on_params_changed_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "muscle_p" || param.get_name() == "muscle_i" || param.get_name() == "muscle_d") {
                // 获取最新值，如果参数列表里没有就用当前的
                double p = (param.get_name() == "muscle_p") ? param.as_double() : this->get_parameter("muscle_p").as_double();
                double i = (param.get_name() == "muscle_i") ? param.as_double() : this->get_parameter("muscle_i").as_double();
                double d = (param.get_name() == "muscle_d") ? param.as_double() : this->get_parameter("muscle_d").as_double();

                // 实时更新到 4 个肌肉关节对象中
                std::lock_guard<std::mutex> lock(mtx_);
                for(int j=0; j<4; j++) {
                    muscle_joints_[j].setPidParameters(p, i, d);
                }
                RCLCPP_INFO(this->get_logger(), "♻️ PID 参数已实时更新: P=%.2f, I=%.2f, D=%.2f", p, i, d);
            }
        }
        return result;
    }

    void init_can_bus() {
        system("sudo ip link set can1 down");
        system("sudo ip link set can1 type can bitrate 1000000");
        system("sudo ip link set can1 up");
        std::this_thread::sleep_for(500ms);
    }

    void setup_hardware(double p, double i, double d) {
        for(int idx=0; idx<4; idx++) {
            muscle_joints_[idx].setId(idx);
            muscle_joints_[idx].setCanDriver(shared_can_driver_);
            muscle_joints_[idx].setPidParameters(p, i, d); 
        }
        ser1_.setCanDriver(shared_can_driver_);
        ser2_.setCanDriver(shared_can_driver_);
        ser1_.start(); ser2_.start();
        hall_sensors_.start(0);
    }

    // ... (receive_can_frames, pid_control_loop, target_callback 等函数保持不变) ...
    // 注意：请确保 pid_control_loop 中对 muscle_joints_ 的操作在 mtx_ 的保护下

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
                std::lock_guard<std::mutex> lock(mtx_); // 保护肌肉对象
                for (int i = 0; i < 4; i++) {
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

    void publish_status_callback() {
        auto msg = ihb_msgs::msg::SingleArmStatus();
        msg.header.stamp = this->now();
        msg.header.frame_id = "slave_link";
        std::vector<float> halls = hall_sensors_.getAllAngles();
        for (int i = 0; i < 4; i++) {
            msg.muscle_joint[i].id = i + 1;
            if (halls.size() >= 8) {
                msg.muscle_joint[i].angle[0] = static_cast<int16_t>(halls[i]);
                msg.muscle_joint[i].angle[1] = static_cast<int16_t>(halls[7-i]);
            }
        }
        msg.servo_motor[0].id = 1;
        msg.servo_motor[0].angle = -ser1_.MAngle();
        msg.servo_motor[1].id = 2;
        msg.servo_motor[1].angle = -ser2_.MAngle();
        publisher_->publish(msg);

        static int print_count = 0;
        if (print_count++ % 5 == 0) {
            printf("\033[2J\033[H");
            RCLCPP_INFO(this->get_logger(), 
                "\n==================== [ 从臂硬件状态监控 ] ====================\n"
                "伺服电机反馈: [M1] %.2f° | [M2] %.2f° \n"
                "肌肉关节 PID: P=%.1f, I=%.1f, D=%.1f\n"
                "==============================================================",
                msg.servo_motor[0].angle, msg.servo_motor[1].angle,
                this->get_parameter("muscle_p").as_double(),
                this->get_parameter("muscle_i").as_double(),
                this->get_parameter("muscle_d").as_double());
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

    // 新增：参数回调句柄
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IhbSlaveArmController>());
    rclcpp::shutdown();
    return 0;
}