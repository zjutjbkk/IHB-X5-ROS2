#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <vector>
#include <atomic>

// 消息定义
#include "ihb_msgs/msg/ihb_target.hpp"

// 底层库头文件 (仅保留传感器)
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"

using namespace std::chrono_literals;

class IhbMasterController : public rclcpp::Node
{
public:
    IhbMasterController()
        : Node("ihb_master_controller")
    {
        RCLCPP_INFO(this->get_logger(), ">>> [主臂端] 纯传感器采集模式启动...");

        try
        {
            // 1. 仅启动传感器读取 (0号设备)
            hall_sensors_.start(0);

            // 2. 发布的配置
            target_publisher_ = this->create_publisher<ihb_msgs::msg::IhbTarget>("arm_target", 10);

            // 3. 定时同步回调 (33ms 约等于 30Hz)
            sync_timer_ = this->create_wall_timer(33ms, std::bind(&IhbMasterController::sync_control_callback, this));

            RCLCPP_INFO(this->get_logger(), "传感器系统就绪。");
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "启动失败: %s", e.what());
            throw;
        }
    }

    ~IhbMasterController()
    {
        hall_sensors_.stop();
    }

private:
    void sync_control_callback()
    {
        auto target_msg = ihb_msgs::msg::IhbTarget();
        target_msg.cmd = 2; // PID 控制模式

        // 获取主臂的所有角度传感器数据
        std::vector<float> hall_angles = hall_sensors_.getAllAngles();

        if (hall_angles.size() >= 8)
        {
            // ================= 1. 电机目标映射 (主臂传感器 -> 从臂电机) =================
            // 这里的逻辑是：主臂的 4 个传感器读数，直接传给从臂的 4 个电机目标

            // 从臂左电机 1 & 2 (由主臂 hall 0 和 1 决定)
            target_msg.servo_joints[0].id = 1;
            target_msg.servo_joints[0].angle = hall_angles[2];
            target_msg.servo_joints[0].speed_limit = 120.0;

            target_msg.servo_joints[1].id = 2;
            target_msg.servo_joints[1].angle = hall_angles[3];
            target_msg.servo_joints[1].speed_limit = 120.0;

            // 【左臂电机】: 主端 Hall 4, 5 -> 从端 ID 1, 2
            target_msg.servo_joints[2].id = 3;
            target_msg.servo_joints[2].angle = hall_angles[6];
            target_msg.servo_joints[2].speed_limit = 120.0;

            target_msg.servo_joints[3].id = 4;
            target_msg.servo_joints[3].angle = hall_angles[7];
            target_msg.servo_joints[3].speed_limit = 120.0;

            // ================= 2. 肌肉关节 1对4 映射逻辑 =================
            static const float kp_table[4][2] = {{250.0, 180.0}, {180.0, 160.0}, {180.0, 240.0}, {240.0, 220.0}};
            static const float ki_table[4][2] = {{0.8, 1.0}, {1.8, 1.5}, {1.0, 1.0}, {1.0, 1.2}};
            static const float kd_table[4][2] = {{1.5, 1.0}, {0.0, 0.0}, {0.5, 1.0}, {0.1, 0.1}};

            // 获取左右主臂基准关节数据
            float R_master_a0 = hall_angles[0];
            float R_master_a1 = hall_angles[1];
            // 左臂关节 (6, 7)
            float L_master_a0 = hall_angles[4];
            float L_master_a1 = hall_angles[5];

            for (int i = 0; i < 8; i++)
            {
                target_msg.muscle_joints[i].id = i + 1;

                if (i < 4) // 前 4 个从臂关节映射给右侧主臂传感器
                {
                    target_msg.muscle_joints[i].angle[0] = R_master_a0;
                    target_msg.muscle_joints[i].angle[1] = R_master_a1;
                    target_msg.muscle_joints[i].kp[0] = kp_table[i][0];
                    target_msg.muscle_joints[i].ki[0] = ki_table[i][0];
                    target_msg.muscle_joints[i].kd[0] = kd_table[i][0];
                    target_msg.muscle_joints[i].kp[1] = kp_table[i][1];
                    target_msg.muscle_joints[i].ki[1] = ki_table[i][1];
                    target_msg.muscle_joints[i].kd[1] = kd_table[i][1];
                }
                else // 后 4 个从臂关节映射给左侧主臂传感器
                {
                    target_msg.muscle_joints[i].angle[0] = L_master_a0;
                    target_msg.muscle_joints[i].angle[1] = L_master_a1;
                    target_msg.muscle_joints[i].kp[0] = kp_table[i - 4][0];
                    target_msg.muscle_joints[i].ki[0] = ki_table[i - 4][0];
                    target_msg.muscle_joints[i].kd[0] = kd_table[i - 4][0];
                    target_msg.muscle_joints[i].kp[1] = kp_table[i - 4][1];
                    target_msg.muscle_joints[i].ki[1] = ki_table[i - 4][1];
                    target_msg.muscle_joints[i].kd[1] = kd_table[i - 4][1];
                }
            }

            // 3. 发布数据给从臂
            target_publisher_->publish(target_msg);
           RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "\n[New Mapping Applied]\n"
                                 "--- Right Arm (Slave ID 3,4 | MJ 1-4) ---\n"
                                 "Motor Target (Hall 0,1): [%.2f, %.2f]\n"
                                 "Muscle Target (Hall 2,3): [%.2f, %.2f]\n"
                                 "--- Left Arm (Slave ID 1,2 | MJ 5-8) ---\n"
                                 "Motor Target (Hall 4,5): [%.2f, %.2f]\n"
                                 "Muscle Target (Hall 6,7): [%.2f, %.2f]\n"
                                 "-------------------------------------------",
                                 hall_angles[2], hall_angles[3], R_master_a0, R_master_a1,
                                 hall_angles[6], hall_angles[7], L_master_a0, L_master_a1);
        }
    }

    ihb::HallSensors hall_sensors_;
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