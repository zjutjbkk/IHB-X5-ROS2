#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <cmath>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// --- 引入消息定义 ---
#include "ihb_msgs/msg/ihb_status.hpp"

// --- 引入底层驱动库 ---
#include "hall_sensors.h"
#include "servo_motorJoint.h"
#include "CanDriver.hpp"

using namespace std::chrono_literals;

class DualArmSlave : public rclcpp::Node
{
public:
    DualArmSlave()
        : Node("ihb_slave_dual_arm_controller"), g_running_(true),
          // 初始化电机对象 (对应 CAN ID 1->0x141, 2->0x142, 3->0x143, 4->0x144)
          ser1_(1), ser2_(2), ser3_(3), ser4_(4)
    {
        RCLCPP_INFO(this->get_logger(), ">>> IHB 传感器状态发布节点启动 (浮点数版本) <<<");

        try
        {
            // 1. 初始化 CAN 总线
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);

            // 2. 硬件启动
            setup_hardware();

            // 3. 创建发布者 (IhbStatus)
            publisher_ = this->create_publisher<ihb_msgs::msg::IhbStatus>("slave_arm_status", 10);

            // 4. 启动 CAN 接收线程 (负责更新底层数据)
            receive_thread_ = std::thread(&DualArmSlave::receive_can_frames, this);

            // 5. 定时器配置
            // Timer 1: 30ms 轮询电机 (发送 "读取当前角度" 指令)
            query_timer_ = this->create_wall_timer(10ms, std::bind(&DualArmSlave::poll_motor_callback, this));

            // Timer 2: 100ms 打包并发布 ROS 消息 (同时也用于打印日志)
            publish_timer_ = this->create_wall_timer(30ms, std::bind(&DualArmSlave::publish_status_callback, this));
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "❌ 初始化失败: %s", e.what());
        }
    }

    ~DualArmSlave()
    {
        g_running_ = false;
        if (receive_thread_.joinable())
            receive_thread_.join();

        hall_sensors_.stop();
        // 关闭电机连接
        ser1_.close();
        ser2_.close();
        ser3_.close();
        ser4_.close();
        RCLCPP_INFO(this->get_logger(), "✅ 节点已退出");
    }

private:
    // --- 核心功能：填充并发布消息 ---
    void publish_status_callback()
    {
        auto msg = ihb_msgs::msg::IhbStatus();
        msg.header.stamp = this->now();
        msg.header.frame_id = "ihb_slave";

        // =========================================================
        // 1. 填充伺服电机 (ServoMotorJointStatus[2])
        // =========================================================
        // 消息定义: float32 angle

        // 电机 1 (0x141)
        msg.servo_motor[0].id = 1;
        msg.servo_motor[0].angle = static_cast<float>(ser1_.MAngle()); // 直接保留浮点数
        msg.servo_motor[0].is_online = true;                           // 简单标记在线

        // 电机 2 (0x142)
        msg.servo_motor[1].id = 2;
        msg.servo_motor[1].angle = static_cast<float>(ser2_.MAngle());
        msg.servo_motor[1].is_online = true;

        // =========================================================
        // 2. 填充肌肉关节 (MuscleJointStatus[4])
        // =========================================================
        // 消息定义: float32[2] angle
        std::vector<float> halls = hall_sensors_.getAllFilteredAngles();

        // 遍历 4 个关节结构体
        // 逻辑：i=0(Joint4), i=1(Joint3), i=2(Joint2), i=3(Joint1) 按照您之前的 3-i 逻辑
        for (int i = 0; i < 4; i++)
        {
            // 计算目标索引：i=0 -> idx=3; i=3 -> idx=0
            int target_idx = 3 - i;
            // int target_idx = i ;

            // 设置 ID
            msg.muscle_joint[target_idx].id = target_idx + 1;

            if (halls.size() >= 16)
            {
                // 读取原始浮点数
                // 映射逻辑保持不变: i 与 8+i 配对
                //  float angle1 = -halls[i];
                // float angle2 =-halls[4 + i];
                float angle1 = -halls[i];
                float angle2 = halls[8 + i];

                // 直接赋值浮点数，不再强转 int16
                msg.muscle_joint[target_idx].angle[0] = angle1;
                msg.muscle_joint[target_idx].angle[1] = angle2;
            }
            else
            {
                // 数据不足时的安全填充
                msg.muscle_joint[target_idx].angle[0] = 0.0f;
                msg.muscle_joint[target_idx].angle[1] = 0.0f;
            }
        }

        // =========================================================
        // [调试] 终端打印 (使用 %.1f 保留一位小数显示)
        // =========================================================
        RCLCPP_INFO(this->get_logger(),
                    "\n--------------- 状态预览 (Float32) ---------------\n"
                    "电机 M1: %6.1f | M2: %6.1f\n"
                    "肌肉 J1: [%6.1f, %6.1f]\n"
                    "肌肉 J2: [%6.1f, %6.1f]\n"
                    "肌肉 J3: [%6.1f, %6.1f]\n"
                    "肌肉 J4: [%6.1f, %6.1f]\n"
                    "--------------------------------------------------",
                    msg.servo_motor[0].angle, msg.servo_motor[1].angle,
                    msg.muscle_joint[0].angle[0], msg.muscle_joint[0].angle[1],
                    msg.muscle_joint[1].angle[0], msg.muscle_joint[1].angle[1],
                    msg.muscle_joint[2].angle[0], msg.muscle_joint[2].angle[1],
                    msg.muscle_joint[3].angle[0], msg.muscle_joint[3].angle[1]);

        // 发布消息
        publisher_->publish(msg);
    }

    // --- CAN 接收线程 ---
    void receive_can_frames()
    {
        int lb = 0;
        setsockopt(shared_can_driver_->getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &lb, sizeof(lb));
        uint8_t data[8];
        while (g_running_ && rclcpp::ok())
        {
            if (shared_can_driver_->waitForMessages(2ms))
            {
                try
                {
                    auto msg = shared_can_driver_->readMessage();
                    uint32_t id = msg.getCanId();
                    std::memcpy(data, msg.getFrameData().data(), 8);

                    if (id == 0x141)
                        ser1_.processCanMessage(data);
                    else if (id == 0x142)
                        ser2_.processCanMessage(data);
                    else if (id == 0x143)
                        ser3_.processCanMessage(data);
                    else if (id == 0x144)
                        ser4_.processCanMessage(data);
                }
                catch (...)
                {
                }
            }
        }
    }

    // --- 电机轮询 ---
    void poll_motor_callback()
    {
        // 轮询4个电机，保持活跃
        ser1_.readAngle();
        std::this_thread::sleep_for(1ms);
        ser2_.readAngle();
        std::this_thread::sleep_for(1ms);
        ser3_.readAngle();
        std::this_thread::sleep_for(1ms);
        ser4_.readAngle();
    }

    // --- 硬件初始化 ---
    void init_can_bus()
    {
        system("sudo ip link set can1 down");
        system("sudo ip link set can1 type can bitrate 1000000");
        system("sudo ip link set can1 up");
    }

    void setup_hardware()
    {
        ser1_.setCanDriver(shared_can_driver_);
        ser2_.setCanDriver(shared_can_driver_);
        ser3_.setCanDriver(shared_can_driver_);
        ser4_.setCanDriver(shared_can_driver_);

        ser1_.start();
        ser2_.start();
        ser3_.start();
        ser4_.start();
        hall_sensors_.start(0);
    }

    // 成员变量
    std::atomic<bool> g_running_;
    std::thread receive_thread_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;

    ihb::HallSensors hall_sensors_;
    ihb::ServoMotorJoint ser1_, ser2_, ser3_, ser4_;

    rclcpp::Publisher<ihb_msgs::msg::IhbStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr query_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualArmSlave>());
    rclcpp::shutdown();
    return 0;
}