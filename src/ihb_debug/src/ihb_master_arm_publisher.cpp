#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <vector>
#include <atomic>
#include <fstream>
#include <iostream>
#include <string>

// 消息定义 (请确保你的 .msg 文件已经重新编译并包含新的 bool 字段)
#include "ihb_msgs/msg/ihb_target.hpp"

// 底层库头文件
// 请确保 gripper_switch.h 和 hall_sensors.h 在同一个目录下，或者修改为正确的绝对路径
#include "hall_sensors.h"
#include "gripper_switch.h" 

using namespace std::chrono_literals;

// ================== 配置路径 ==================
#define PID_CONFIG_PATH "/root/jbkk/IHB-X5-ROS2/pid_config.txt"

// ================== GPIO 配置 (基于之前分析) ==================
static constexpr const char *GPIO_CHIP_NAME = "gpiochip4";
// 物理 Pin 13 (GPIO 27) -> Offset 0
static constexpr int LEFT_SWITCH_OFFSET = 0; 
// 物理 Pin 11 (GPIO 17) -> Offset 1
static constexpr int RIGHT_SWITCH_OFFSET = 1;

class IhbMasterController : public rclcpp::Node
{
public:
    IhbMasterController()
        : Node("ihb_master_controller")
    {
        RCLCPP_INFO(this->get_logger(), ">>> [主臂端] 纯传感器采集模式启动 (Hall + GPIO)...");

        // 1. 初始化 PID 参数
        init_default_pids();
        load_pids_from_file();

        try
        {
            // 3. 启动传感器
            // (A) 启动 Hall 角度读取
            hall_sensors_.start(0);

            // (B) 启动 GPIO 夹爪开关 (使用 unique_ptr 延迟初始化以捕获异常)
            // 这里的逻辑对应之前的 active-low (接地触发)
            left_gripper_switch_ = std::make_unique<ihb::GripperSwitch>(GPIO_CHIP_NAME, LEFT_SWITCH_OFFSET);
            right_gripper_switch_ = std::make_unique<ihb::GripperSwitch>(GPIO_CHIP_NAME, RIGHT_SWITCH_OFFSET);

            // 4. 发布的配置
            target_publisher_ = this->create_publisher<ihb_msgs::msg::IhbTarget>("arm_target", 10);

            // 5. 定时同步回调 (33ms 约等于 30Hz)
            sync_timer_ = this->create_wall_timer(33ms, std::bind(&IhbMasterController::sync_control_callback, this));

            RCLCPP_INFO(this->get_logger(), "传感器系统 (Hall x16 + Gripper x2) 就绪。");
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "硬件初始化失败: %s", e.what());
            // 如果是 GPIO 初始化失败（比如权限问题），这里会直接抛出，避免程序带病运行
            throw; 
        }
    }

    ~IhbMasterController()
    {
        hall_sensors_.stop();
        // unique_ptr 会自动释放 GPIO 资源
    }

private:
    // PID 表格成员变量
    float kp_table[4][2];
    float ki_table[4][2];
    float kd_table[4][2];

    void init_default_pids()
    {
        for (int i = 0; i < 4; i++) {
            kp_table[i][0] = 0.0f; kp_table[i][1] = 0.0f;
            ki_table[i][0] = 0.0f; ki_table[i][1] = 0.0f;
            kd_table[i][0] = 0.0f; kd_table[i][1] = 0.0f;
        }
    }

    void load_pids_from_file()
    {
        std::ifstream file(PID_CONFIG_PATH);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "!!!! 无法打开配置文件: %s !!!! 使用默认(0)参数。", PID_CONFIG_PATH);
            return;
        }
        std::string line;
        int row_idx = 0;
        while (std::getline(file, line) && row_idx < 4) {
            if (line.empty() || line[0] == '#') continue;
            float p1, i1, d1, p2, i2, d2;
            int n = sscanf(line.c_str(), "%f %f %f %f %f %f", &p1, &i1, &d1, &p2, &i2, &d2);
            if (n == 6) {
                kp_table[row_idx][0] = p1; ki_table[row_idx][0] = i1; kd_table[row_idx][0] = d1;
                kp_table[row_idx][1] = p2; ki_table[row_idx][1] = i2; kd_table[row_idx][1] = d2;
                row_idx++;
            }
        }
        file.close();
    }

    void sync_control_callback()
    {
        auto target_msg = ihb_msgs::msg::IhbTarget();
        
        // 1. 填充 Header
        target_msg.header.stamp = this->now();
        target_msg.header.frame_id = "master_arm_link";
        
        target_msg.cmd = 2; // PID 控制模式

        // 2. 读取 Hall 角度
        std::vector<float> hall_angles = hall_sensors_.getAllFilteredAngles();

        // 3. 读取 GPIO 夹爪状态 (新增功能)
        // isTriggered() 返回 true 表示按下/闭合
        bool left_triggered = left_gripper_switch_->isTriggered();
        bool right_triggered = right_gripper_switch_->isTriggered();

        target_msg.left_gripper_touched = left_triggered;
        target_msg.right_gripper_touched = right_triggered;

        if (hall_angles.size() >= 16)
        {
            // --- 电机映射逻辑 (保持不变) ---
            target_msg.servo_joint[0].id = 1; target_msg.servo_joint[0].angle = hall_angles[2]; target_msg.servo_joint[0].speed_limit = 120.0;
            target_msg.servo_joint[1].id = 2; target_msg.servo_joint[1].angle = hall_angles[3]; target_msg.servo_joint[1].speed_limit = 120.0;
            target_msg.servo_joint[2].id = 3; target_msg.servo_joint[2].angle = hall_angles[10]; target_msg.servo_joint[2].speed_limit = 120.0;
            target_msg.servo_joint[3].id = 4; target_msg.servo_joint[3].angle = hall_angles[11]; target_msg.servo_joint[3].speed_limit = 120.0;

            // --- 肌肉关节映射逻辑 (保持不变) ---
            float R_master_a0 = hall_angles[0]/3;
            float R_master_a1 = hall_angles[1]/3;
            float L_master_a0 = hall_angles[8]/3;
            float L_master_a1 = hall_angles[9]/3;

            for (int i = 0; i < 8; i++)
            {
                target_msg.muscle_joint[i].id = i + 1;
                if (i < 4) {
                    target_msg.muscle_joint[i].angle[0] = R_master_a0;
                    target_msg.muscle_joint[i].angle[1] = R_master_a1;
                    target_msg.muscle_joint[i].kp[0] = kp_table[i][0]; target_msg.muscle_joint[i].ki[0] = ki_table[i][0]; target_msg.muscle_joint[i].kd[0] = kd_table[i][0];
                    target_msg.muscle_joint[i].kp[1] = kp_table[i][1]; target_msg.muscle_joint[i].ki[1] = ki_table[i][1]; target_msg.muscle_joint[i].kd[1] = kd_table[i][1];
                } else {
                    target_msg.muscle_joint[i].angle[0] = L_master_a0;
                    target_msg.muscle_joint[i].angle[1] = L_master_a1;
                    target_msg.muscle_joint[i].kp[0] = kp_table[i - 4][0]; target_msg.muscle_joint[i].ki[0] = ki_table[i - 4][0]; target_msg.muscle_joint[i].kd[0] = kd_table[i - 4][0];
                    target_msg.muscle_joint[i].kp[1] = kp_table[i - 4][1]; target_msg.muscle_joint[i].ki[1] = ki_table[i - 4][1]; target_msg.muscle_joint[i].kd[1] = kd_table[i - 4][1];
                }
            }

            // 4. 发布消息
            target_publisher_->publish(target_msg);

            // 5. 日志打印 (增加夹爪状态显示)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "\n[Master State]\n"
                                 "--- Grippers ---\n"
                                 "Left (Pin 13/GPIO27): %s\n"
                                 "Right (Pin 11/GPIO17): %s\n"
                                 "--- Right Arm ---\n"
                                 "Motor: [%.2f, %.2f] | Muscle: [%.2f, %.2f]\n"
                                 "--- Left Arm ---\n"
                                 "Motor: [%.2f, %.2f] | Muscle: [%.2f, %.2f]\n"
                                 "-------------------------------------------",
                                 left_triggered ? "CLOSED (Triggered)" : "OPEN",
                                 right_triggered ? "CLOSED (Triggered)" : "OPEN",
                                 hall_angles[2], hall_angles[3], R_master_a0, R_master_a1,
                                 hall_angles[10], hall_angles[11], L_master_a0, L_master_a1);
        }
    }

    // 硬件对象
    ihb::HallSensors hall_sensors_;
    // 使用 unique_ptr 管理 GPIO 对象，因为它们的构造函数可能抛出异常
    std::unique_ptr<ihb::GripperSwitch> left_gripper_switch_;
    std::unique_ptr<ihb::GripperSwitch> right_gripper_switch_;

    rclcpp::Publisher<ihb_msgs::msg::IhbTarget>::SharedPtr target_publisher_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IhbMasterController>());
    rclcpp::shutdown();
    return 0;
}