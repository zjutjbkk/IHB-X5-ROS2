#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

// --- 修复点 1: 增加必要的系统头文件以支持 setsockopt 和 CAN 宏 ---
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/inotify.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "ihb_msgs/msg/single_arm_status.hpp"
#include "ihb_msgs/msg/arm_target.hpp"

#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/servo_motorJoint.h"
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/muscle_joint.h"
#include "CanDriver.hpp"

using namespace std::chrono_literals;

class IhbSlaveArmController : public rclcpp::Node
{
public:
    IhbSlaveArmController()
    : Node("ihb_slave_arm_controller"), 
      // --- 修复点 2: 严格按照类私有变量声明的顺序初始化，解决 -Wreorder 警告 ---
      yaml_path_("/root/jbkk/IHB-X5-ROS2/src/ihb_debug/config/pid_params.yaml"),
      g_running_(true),
      ser1_(1), ser2_(2)
    {
        RCLCPP_INFO(this->get_logger(), ">>> 正在初始化 IHB [热加载 PID 控制] 节点...");

        for (int i = 0; i < 4; i++) {
            std::string prefix = "joint" + std::to_string(i) + ".";
            this->declare_parameter(prefix + "p", 30.0);
            this->declare_parameter(prefix + "i", 1.8);
            this->declare_parameter(prefix + "d", 120.0);
        }

        try {
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);
            setup_hardware();

            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&IhbSlaveArmController::on_params_changed_callback, this, std::placeholders::_1));

            publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>("slave_arm_status", 10);
            subscriber_ = this->create_subscription<ihb_msgs::msg::ArmTarget>(
                "arm_target", 10, std::bind(&IhbSlaveArmController::target_callback, this, std::placeholders::_1));

            receive_thread_ = std::thread(&IhbSlaveArmController::receive_can_frames, this);
            pid_thread_ = std::thread(&IhbSlaveArmController::pid_control_loop, this);
            watch_thread_ = std::thread(&IhbSlaveArmController::watch_yaml_file, this);

            query_timer_ = this->create_wall_timer(50ms, std::bind(&IhbSlaveArmController::poll_motor_callback, this));
            publish_timer_ = this->create_wall_timer(100ms, std::bind(&IhbSlaveArmController::publish_status_callback, this));

            RCLCPP_INFO(this->get_logger(), "🚀 系统就绪。当前监控文件: %s", yaml_path_.c_str());

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "❌ 初始化失败: %s", e.what());
        }
    }

    ~IhbSlaveArmController() {
        g_running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (pid_thread_.joinable()) pid_thread_.join();
        if (watch_thread_.joinable()) watch_thread_.join();
        hall_sensors_.stop();
        ser1_.close(); ser2_.close();
    }

private:
    void watch_yaml_file() {
        int fd = inotify_init();
        if (fd < 0) return;
        int wd = inotify_add_watch(fd, yaml_path_.c_str(), IN_CLOSE_WRITE);
        if (wd < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法监控 YAML 文件: %s", yaml_path_.c_str());
            close(fd);
            return;
        }

        char buffer[1024];
        while (g_running_ && rclcpp::ok()) {
            fd_set descriptors;
            FD_ZERO(&descriptors);
            FD_SET(fd, &descriptors);
            struct timeval tv;
            tv.tv_sec = 1; tv.tv_usec = 0;

            int retval = select(fd + 1, &descriptors, NULL, NULL, &tv);
            if (retval > 0 && FD_ISSET(fd, &descriptors)) {
                int length = read(fd, buffer, 1024);
                if (length > 0) {
                    RCLCPP_INFO(this->get_logger(), "📂 检测到 YAML 保存，自动加载参数...");
                    std::string cmd = "ros2 param load /ihb_slave_arm_controller " + yaml_path_ + " &";
                    int res = system(cmd.c_str());
                    (void)res;
                }
            }
        }
        inotify_rm_watch(fd, wd);
        close(fd);
    }

    rcl_interfaces::msg::SetParametersResult on_params_changed_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto &param : parameters) {
            std::string name = param.get_name();
            if (name.find("joint") == 0) {
                try {
                    int idx = std::stoi(name.substr(5, 1)); 
                    if (idx >= 0 && idx < 4) {
                        std::string prefix = "joint" + std::to_string(idx);
                        double p = (name == prefix + ".p") ? param.as_double() : this->get_parameter(prefix + ".p").as_double();
                        double i = (name == prefix + ".i") ? param.as_double() : this->get_parameter(prefix + ".i").as_double();
                        double d = (name == prefix + ".d") ? param.as_double() : this->get_parameter(prefix + ".d").as_double();
                        muscle_joints_[idx].setPidParameters(p, i, d);
                        RCLCPP_INFO(this->get_logger(), "♻️ 关节 %d PID 更新: P=%.2f, I=%.2f, D=%.2f", idx, p, i, d);
                    }
                } catch (...) {}
            }
        }
        return result;
    }

    void init_can_bus() {
        int res = system("sudo ip link set can1 down");
        res = system("sudo ip link set can1 type can bitrate 1000000");
        res = system("sudo ip link set can1 up");
        (void)res;
        std::this_thread::sleep_for(500ms);
    }

    void setup_hardware() {
        for(int idx=0; idx<4; idx++) {
            std::string prefix = "joint" + std::to_string(idx) + ".";
            muscle_joints_[idx].setId(idx);
            muscle_joints_[idx].setCanDriver(shared_can_driver_);
            muscle_joints_[idx].setPidParameters(
                this->get_parameter(prefix+"p").as_double(),
                this->get_parameter(prefix+"i").as_double(),
                this->get_parameter(prefix+"d").as_double());
        }
        ser1_.setCanDriver(shared_can_driver_);
        ser2_.setCanDriver(shared_can_driver_);
        ser1_.start(); ser2_.start();
        hall_sensors_.start(0);
    }

    void receive_can_frames() {
        int lb = 0; 
        // --- 修复点 3: 这里现在有了头文件，宏定义可以正常识别 ---
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

    void pid_control_loop() {
        while (g_running_ && rclcpp::ok()) {
            std::vector<float> halls = hall_sensors_.getAllAngles();
            if (halls.size() >= 8) {
                std::lock_guard<std::mutex> lock(mtx_); 
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

    void publish_status_callback() 
    {
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
    }

    // --- 修复点 2 相关: 类成员变量的声明顺序 ---
    std::string yaml_path_;
    std::atomic<bool> g_running_;
    std::thread receive_thread_, pid_thread_, watch_thread_;
    std::mutex mtx_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;
    
    ihb::HallSensors hall_sensors_;
    ihb::MuscleJoint muscle_joints_[4];
    ihb::ServoMotorJoint ser1_, ser2_;

    rclcpp::Publisher<ihb_msgs::msg::SingleArmStatus>::SharedPtr publisher_;
    rclcpp::Subscription<ihb_msgs::msg::ArmTarget>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr query_timer_, publish_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IhbSlaveArmController>());
    rclcpp::shutdown();
    return 0;
}