#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <vector>
#include <atomic>
#include <fstream>
#include <iostream>
#include <string>

// 消息定义
#include "ihb_msgs/msg/ihb_target.hpp"

// 底层库头文件
#include "hall_sensors.h"
#include "gripper_switch.h"

using namespace std::chrono_literals;

#define PID_CONFIG_PATH "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/config/pid_config.txt"

// GPIO 配置
static constexpr const char *GPIO_CHIP_NAME = "gpiochip4";
static constexpr int LEFT_SWITCH_OFFSET = 0;
static constexpr int RIGHT_SWITCH_OFFSET = 1;

class IhbMasterController : public rclcpp::Node
{
public:
    IhbMasterController()
        : Node("ihb_master_controller")
    {
        RCLCPP_INFO(this->get_logger(), ">>> [主臂端] 纯传感器采集模式启动 (Hall + GPIO)...");

        // 1. 初始化为您亲自测试的最佳 PID 参数！
        init_default_pids();

        // 可选：如果 txt 文件存在，会覆盖上面硬编码的参数；如果不存在，就用硬编码的安全参数
        load_pids_from_file();

        try
        {
            // 2. 启动传感器
            hall_sensors_.start(0);

            left_gripper_switch_ = std::make_unique<ihb::GripperSwitch>(GPIO_CHIP_NAME, LEFT_SWITCH_OFFSET);
            right_gripper_switch_ = std::make_unique<ihb::GripperSwitch>(GPIO_CHIP_NAME, RIGHT_SWITCH_OFFSET);

            // 3. 发布者与定时器
            target_publisher_ = this->create_publisher<ihb_msgs::msg::IhbTarget>("arm_target", 10);
            sync_timer_ = this->create_wall_timer(33ms, std::bind(&IhbMasterController::sync_control_callback, this));

            RCLCPP_INFO(this->get_logger(), "✅ 传感器系统 (Hall x16 + Gripper x2) 就绪。开始遥操作！");
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "❌ 硬件初始化失败: %s", e.what());
            throw;
        }
    }

    ~IhbMasterController()
    {
        hall_sensors_.stop();
    }

private:
    // 将数组扩展为 8行2列，分别对应左臂 4 个关节和右臂 4 个关节
    float kp_table[8][2];
    float ki_table[8][2];
    float kd_table[8][2];

    void init_default_pids()
    {
        // ================= 【 左臂 Left Arm (索引 0~3) 】 =================
        // Joint 1
        // kp_table[0][0] = 60.0; ki_table[0][0] = 1.6; kd_table[0][0] = 0.0;
        // kp_table[0][1] = 45.0; ki_table[0][1] = 1.6; kd_table[0][1] = 0.0;
        // // Joint 2
        // kp_table[1][0] = 40.0; ki_table[1][0] = 1.3; kd_table[1][0] = 0.0;
        // kp_table[1][1] = 30.0; ki_table[1][1] = 1.4; kd_table[1][1] = 0.0;
        // // Joint 3
        // kp_table[2][0] = 60.0; ki_table[2][0] = 1.3; kd_table[2][0] = 0.0;
        // kp_table[2][1] = 60.0; ki_table[2][1] = 1.1; kd_table[2][1] = 0.0;
        // // Joint 4
        // kp_table[3][0] = 40.0; ki_table[3][0] = 1.0; kd_table[3][0] = 0.0;
        // kp_table[3][1] = 40.0; ki_table[3][1] = 1.0; kd_table[3][1] = 0.0;
        kp_table[0][0] = 0.0;
        ki_table[0][0] = 0.0;
        kd_table[0][0] = 0.0;
        kp_table[0][1] = 0.0;
        ki_table[0][1] = 0.0;
        kd_table[0][1] = 0.0;
        kp_table[1][0] = 0.0;
        ki_table[1][0] = 0.0;
        kd_table[1][0] = 0.0;
        kp_table[1][1] = 0.0;
        ki_table[1][1] = 0.0;
        kd_table[1][1] = 0.0;
        kp_table[2][0] = 0.0;
        ki_table[2][0] = 0.0;
        kd_table[2][0] = 0.0;
        kp_table[2][1] = 0.0;
        ki_table[2][1] = 0.0;
        kd_table[2][1] = 0.0;
        kp_table[3][0] = 0.0;
        ki_table[3][0] = 0.0;
        kd_table[3][0] = 0.0;
        kp_table[3][1] = 0.0;
        ki_table[3][1] = 0.0;
        kd_table[3][1] = 0.0;

        // ================= 【 右臂 Right Arm (索引 4~7) 】 =================
        // Joint 1
        // kp_table[4][0] = 25.0; ki_table[4][0] = 1.0;  kd_table[4][0] = 0.0;
        // kp_table[4][1] = 28.0; ki_table[4][1] = 1.0;  kd_table[4][1] = 0.0;
        // // Joint 2
        // kp_table[5][0] = 32.0; ki_table[5][0] = 0.55; kd_table[5][0] = 0.0;
        // kp_table[5][1] = 35.0; ki_table[5][1] = 0.55; kd_table[5][1] = 0.0;
        // // Joint 3
        // kp_table[6][0] = 35.0; ki_table[6][0] = 0.7;  kd_table[6][0] = 0.0;
        // kp_table[6][1] = 20.0; ki_table[6][1] = 0.5;  kd_table[6][1] = 0.0;
        // // Joint 4
        // kp_table[7][0] = 28.0; ki_table[7][0] = 1.0;  kd_table[7][0] = 0.0;
        // kp_table[7][1] = 28.0; ki_table[7][1] = 1.0;  kd_table[7][1] = 0.0;
        kp_table[4][0] = 0.0;
        ki_table[4][0] = 0.0;
        kd_table[4][0] = 0.0;
        kp_table[4][1] = 0.0;
        ki_table[4][1] = 0.0;
        kd_table[4][1] = 0.0;
        kp_table[5][0] = 0.0;
        ki_table[5][0] = 0.0;
        kd_table[5][0] = 0.0;
        kp_table[5][1] = 0.0;
        ki_table[5][1] = 0.0;
        kd_table[5][1] = 0.0;
        kp_table[6][0] = 0.0;
        ki_table[6][0] = 0.0;
        kd_table[6][0] = 0.0;
        kp_table[6][1] = 0.0;
        ki_table[6][1] = 0.0;
        kd_table[6][1] = 0.0;
        kp_table[7][0] = 0.0;
        ki_table[7][0] = 0.0;
        kd_table[7][0] = 0.0;
        kp_table[7][1] = 0.0;
        ki_table[7][1] = 0.0;
        kd_table[7][1] = 0.0;
    }

    void load_pids_from_file()
    {
        std::ifstream file(PID_CONFIG_PATH);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "未找到 pid_config.txt，已自动应用内置的最佳 PID 参数！");
            return;
        }
        std::string line;
        int row_idx = 0;
        // 扩展为读取 8 行
        while (std::getline(file, line) && row_idx < 8)
        {
            if (line.empty() || line[0] == '#')
                continue;
            float p1, i1, d1, p2, i2, d2;
            int n = sscanf(line.c_str(), "%f %f %f %f %f %f", &p1, &i1, &d1, &p2, &i2, &d2);
            if (n == 6)
            {
                kp_table[row_idx][0] = p1;
                ki_table[row_idx][0] = i1;
                kd_table[row_idx][0] = d1;
                kp_table[row_idx][1] = p2;
                ki_table[row_idx][1] = i2;
                kd_table[row_idx][1] = d2;
                row_idx++;
            }
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "已从 txt 文件加载覆盖了 %d 行 PID 参数。", row_idx);
    }

    void sync_control_callback()
    {
        auto target_msg = ihb_msgs::msg::IhbTarget();
        target_msg.header.stamp = this->now();
        target_msg.header.frame_id = "master_arm_link";
        target_msg.cmd = 2; // PID 控制模式

        std::vector<float> hall_angles = hall_sensors_.getAllFilteredAngles();

        bool left_triggered = left_gripper_switch_->isTriggered();
        bool right_triggered = right_gripper_switch_->isTriggered();

        target_msg.left_gripper_touched = left_triggered;
        target_msg.right_gripper_touched = right_triggered;

        if (hall_angles.size() >= 16)
        {
            // 伺服电机 (注意这里 120 必须是整数)
            target_msg.servo_joint[0].id = 1;
            target_msg.servo_joint[0].angle = hall_angles[2];
            target_msg.servo_joint[0].speed_limit = 120;
            target_msg.servo_joint[1].id = 2;
            target_msg.servo_joint[1].angle = hall_angles[3];
            target_msg.servo_joint[1].speed_limit = 120;
            target_msg.servo_joint[2].id = 3;
            target_msg.servo_joint[2].angle = hall_angles[10];
            target_msg.servo_joint[2].speed_limit = 120;
            target_msg.servo_joint[3].id = 4;
            target_msg.servo_joint[3].angle = hall_angles[11];
            target_msg.servo_joint[3].speed_limit = 120;

            // 主端角度读取 (根据代码推断：[0][1]是右臂，[8][9]是左臂)
            float R_master_a0 = hall_angles[0] / 3.0f;
            float R_master_a1 = hall_angles[1] / 3.0f;
            float L_master_a0 = hall_angles[8] / 3.0f;
            float L_master_a1 = hall_angles[9] / 3.0f;

            for (int i = 0; i < 8; i++)
            {
                target_msg.muscle_joint[i].id = i + 1;

                // 给 16个 PID 赋值，直接从 [8][2] 的数组里取
                target_msg.muscle_joint[i].kp[0] = kp_table[i][0];
                target_msg.muscle_joint[i].ki[0] = ki_table[i][0];
                target_msg.muscle_joint[i].kd[0] = kd_table[i][0];
                target_msg.muscle_joint[i].kp[1] = kp_table[i][1];
                target_msg.muscle_joint[i].ki[1] = ki_table[i][1];
                target_msg.muscle_joint[i].kd[1] = kd_table[i][1];

                if (i < 4)
                {
                    // 左臂从动端 (0~3)，接收主端左臂传感器数据 (L_master)
                    target_msg.muscle_joint[i].angle[0] = L_master_a0;
                    target_msg.muscle_joint[i].angle[1] = L_master_a1;
                }
                else
                {
                    // 右臂从动端 (4~7)，接收主端右臂传感器数据 (R_master)
                    target_msg.muscle_joint[i].angle[0] = R_master_a0;
                    target_msg.muscle_joint[i].angle[1] = R_master_a1;
                }
            }

            target_publisher_->publish(target_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "\n[Master State]\n"
                                 "--- Grippers ---\n"
                                 "Left: %s | Right: %s\n"
                                 "--- Left Arm (Master -> Slave 0~3) ---\n"
                                 "Muscle Cmd: [%.2f, %.2f]\n"
                                 "--- Right Arm (Master -> Slave 4~7) ---\n"
                                 "Muscle Cmd: [%.2f, %.2f]\n"
                                 "-------------------------------------------",
                                 left_triggered ? "CLOSED" : "OPEN",
                                 right_triggered ? "CLOSED" : "OPEN",
                                 L_master_a0, L_master_a1,
                                 R_master_a0, R_master_a1);
        }
    }

    ihb::HallSensors hall_sensors_;
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