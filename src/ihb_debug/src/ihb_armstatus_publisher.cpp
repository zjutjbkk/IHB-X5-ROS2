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
#include "ihb_msgs/msg/muscle_joint_status.hpp"
#include "ihb_msgs/msg/servo_motor_joint_status.hpp"

// 底层库头文件
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"
#include "/root/jbkk/IHB-X5-ROS2/src/ihb_debug/lib/ihb_cpp/ihb_cpp/include/servo_motorJoint.h"


#include "CanDriver.hpp"
#include <sys/socket.h>      // 为 setsockopt 提供支持
#include <linux/can.h>       // 为 SOL_CAN_RAW 提供支持
#include <linux/can/raw.h>   // 为 CAN_RAW_LOOPBACK 提供支持

using namespace std::chrono_literals;

class IhbArmStatusPublisher : public rclcpp::Node
{
public:
    IhbArmStatusPublisher()
    : Node("ihb_arm_status_publisher"), 
      g_running_(true),
      ser3_(3), ser4_(4) // 初始化电机对象
    {
        RCLCPP_INFO(this->get_logger(), ">>> 正在初始化 IHB 系统整合节点...");

        try {
            // 1. 初始化 CAN1 硬件接口
            init_can_bus();
            shared_can_driver_ = std::make_shared<sockcanpp::CanDriver>("can1", CAN_RAW);
            RCLCPP_INFO(this->get_logger(), "1. CAN1 驱动加载成功");

            // 2. 注入驱动并启动硬件
            ser3_.setCanDriver(shared_can_driver_); // 解决代码 2 局部变量无法发送指令的问题
            ser4_.setCanDriver(shared_can_driver_);
            
            hall_sensors_.start(0); 
            ser3_.start(); ser4_.start(); 
            RCLCPP_INFO(this->get_logger(), "2. 硬件设备(电机/传感器)启动成功");

            // 3. 创建发布器
            publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>("arm_status", 10);
            RCLCPP_INFO(this->get_logger(), "3. ROS 2 发布器创建成功");

            // 4. 启动后台高频接收线程 (保留代码 1 的高性能接收)
            receive_thread_ = std::thread(&IhbArmStatusPublisher::receive_can_frames, this);
            RCLCPP_INFO(this->get_logger(), "4. CAN 接收线程已进入后台");

            // 5. 使用定时器进行轮询与发布 (引入代码 2 的规范调度)
            // 查询定时器：负责发送 0x92 指令
            query_timer_ = this->create_wall_timer(50ms, std::bind(&IhbArmStatusPublisher::poll_motor_callback, this));
            
            // 发布定时器：负责打包数据 (15Hz)
            publish_timer_ = this->create_wall_timer(150ms, std::bind(&IhbArmStatusPublisher::publish_status_callback, this));
            
            RCLCPP_INFO(this->get_logger(), "5. 系统定时任务已就绪，开始监控...");

        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "系统启动失败: %s", e.what());
            throw; // 抛出异常以阻止节点运行
        }
    }

    ~IhbArmStatusPublisher()
    {
        g_running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        
        hall_sensors_.stop();
        ser3_.close(); ser4_.close(); 
        RCLCPP_INFO(this->get_logger(), "IHB 系统已安全关闭");
    }

private:
    void init_can_bus() {
        system("sudo ip link set can1 type can bitrate 1000000");
        std::this_thread::sleep_for(100ms);
        system("sudo ip link set can1 up");
        std::this_thread::sleep_for(500ms);
    }

    // 接收线程：保持代码 1 的高频解析逻辑
    void receive_can_frames() {
        int loopback = 0;
        setsockopt(shared_can_driver_->getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

        while (g_running_ && rclcpp::ok()) {
            if (shared_can_driver_->waitForMessages(5ms)) {
                auto msg = shared_can_driver_->readMessage();
                uint8_t data[8];
                for (size_t i = 0; i < 8; i++) data[i] = msg.getFrameData()[i];

                uint32_t canId = msg.getCanId();
                if      (canId == 0x143) ser3_.processCanMessage(data);
                else if (canId == 0x144) ser4_.processCanMessage(data);
            }
        }
    }

    // 查询定时器：引入代码 2 的定时查询机制，避免死循环
    void poll_motor_callback() {
        ser3_.readAngle();
        std::this_thread::sleep_for(2ms);
        ser4_.readAngle();
    }

    // 发布定时器：整合数据并发布
    void publish_status_callback() {
        auto message = ihb_msgs::msg::SingleArmStatus();
        message.header.stamp = this->now();
        message.header.frame_id = "base_link";

        // A. 霍尔角度填充
        std::vector<float> hall_angles = hall_sensors_.getAllFilteredAngles();
        for (int i = 0; i < 4; i++) {
            message.muscle_joint[i].id = i + 1;
            message.muscle_joint[i].angle[0] = static_cast<int16_t>(hall_angles[i]);
            message.muscle_joint[i].angle[1] = static_cast<int16_t>(hall_angles[7-i]);
        }

        // B. 伺服电机多圈角度填充
        message.servo_motor[0].id = 3;
        message.servo_motor[0].angle = static_cast<float>(ser3_.MAngle());

        message.servo_motor[1].id = 4;
        message.servo_motor[1].angle = static_cast<float>(ser4_.MAngle());

        publisher_->publish(message);
        static int print_counter = 0;
        if (print_counter++ % 15 == 0) { // 每 15 次循环（约 1 秒）打印一次
            RCLCPP_INFO(this->get_logger(), 
                "\n================ [ 臂状态监控: ID 3&4 ] ================\n"
                "伺服电机 -> M3: %6.1f° | M4: %6.1f°\n"
                "肌肉关节 1 (CH 0-7) -> A: %6.1f | B: %6.1f\n"
                "肌肉关节 2 (CH 1-6) -> A: %6.1f | B: %6.1f\n"
                "肌肉关节 3 (CH 2-5) -> A: %6.1f | B: %6.1f\n"
                "肌肉关节 4 (CH 3-4) -> A: %6.1f | B: %6.1f\n"
                "======================================================",
                message.servo_motor[0].angle, 
                message.servo_motor[1].angle,
                hall_angles[0], hall_angles[7],
                hall_angles[1], hall_angles[6],
                hall_angles[2], hall_angles[5],
                hall_angles[3], hall_angles[4]);
        }
    }

    std::atomic<bool> g_running_;
    std::thread receive_thread_;
    std::shared_ptr<sockcanpp::CanDriver> shared_can_driver_;
    
    ihb::HallSensors hall_sensors_;
    ihb::ServoMotorJoint ser3_, ser4_;

    rclcpp::Publisher<ihb_msgs::msg::SingleArmStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr query_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<IhbArmStatusPublisher>());
    } catch (...) {}
    rclcpp::shutdown();
    return 0;
}