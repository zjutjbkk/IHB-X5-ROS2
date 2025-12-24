#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

// ROS 消息头文件
#include "ihb_msgs/msg/single_arm_status.hpp"
#include "ihb_msgs/msg/muscle_joint_status.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_status.hpp"
#include "ihb_msgs/msg/servo_motor_joint_status.hpp"
#include "ihb_msgs/msg/force_sensor_status.hpp"

// 硬件驱动头文件
#include "../lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"
#include "../lib/ihb_cpp/ihb_cpp/include/servo_motorJoint.h"

#include "CanDriver.hpp"      // 包含 CAN 驱动
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>

using namespace std::chrono_literals;

class ArmStatusNode : public rclcpp::Node
{
public:
  ArmStatusNode()
  : Node("arm_status_publisher"), 
    ser1_(1), ser2_(2),  // 初始化电机对象
    running_(true)
  {
    RCLCPP_INFO(this->get_logger(), "构造函数开始执行");
        
    try {
        // 1. 创建发布器
        publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>(
            "arm_status", 10);
        RCLCPP_INFO(this->get_logger(), "发布器创建成功");
        
        // 2. 初始化霍尔传感器
        hall_sensors_.start(0);
        RCLCPP_INFO(this->get_logger(), "霍尔传感器启动成功");
        
        // 3. 启动 CAN 接收线程
        can_thread_ = std::thread(&ArmStatusNode::receive_can_loop, this);
        RCLCPP_INFO(this->get_logger(), "CAN线程已启动");
        
        // 4. 创建定时器
        query_timer_ = this->create_wall_timer(
            20ms, std::bind(&ArmStatusNode::query_motor_status, this));
        RCLCPP_INFO(this->get_logger(), "查询定时器已创建");
        
        publish_timer_ = this->create_wall_timer(
            500ms, std::bind(&ArmStatusNode::publish_data, this));
        RCLCPP_INFO(this->get_logger(), "发布定时器已创建");
        
        RCLCPP_INFO(this->get_logger(), "ArmStatusNode 已成功启动");
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "构造函数异常: %s", e.what());
        throw; // 重新抛出异常
    }
  }
  
  ~ArmStatusNode()
  {
    // 1. 停止线程标志位
    running_ = false;

    // 2. 等待 CAN 线程安全退出
    if (can_thread_.joinable()) {
      can_thread_.join();
    }

    // 3. 停止硬件
    hall_sensors_.stop();
    // ser1_.close(); // 根据需要决定退出时是否关闭电机
    
    RCLCPP_INFO(this->get_logger(), "ArmStatusNode 已关闭");
  }

private:
  // === 硬件交互线程：CAN 接收循环 ===
  void receive_can_loop()
  {
      RCLCPP_INFO(this->get_logger(), "CAN 接收线程已启动");
      
      try {
          // 初始化 CAN 驱动 (局部变量即可，只要 socket 保持打开)
          sockcanpp::CanDriver canDriver("can1", CAN_RAW);
          
          // 关闭回环
          int loopback = 0;
          setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

          uint8_t data[8] = {0};

          while (running_ && rclcpp::ok())
          {
              // 等待消息，超时 5ms，避免死循环占用 100% CPU
              if (canDriver.waitForMessages(std::chrono::milliseconds(5)))
              {
                  sockcanpp::CanMessage receivedMessage = canDriver.readMessage();
                  
                  // 提取数据
                  const auto& frameData = receivedMessage.getFrameData();
                  for (size_t i = 0; i < frameData.size() && i < 8; i++) {
                      data[i] = frameData[i];
                  }

                  uint16_t canId = (uint16_t)receivedMessage.getCanId();

                  // 分发数据给对应的电机对象处理
                  // processCanMessage 内部有互斥锁，是线程安全的
                  if(canId == 0x141) ser1_.processCanMessage(data);
                  else if (canId == 0x142) ser2_.processCanMessage(data);
              }
              else
              {
                  // 没有消息时短暂休眠
                  std::this_thread::sleep_for(std::chrono::milliseconds(1));
              }
          }
      } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "CAN 驱动初始化失败或运行错误: %s", e.what());
      }
      
      RCLCPP_INFO(this->get_logger(), "CAN 接收线程已退出");
  }

  // === 辅助任务：定期查询电机状态 ===
  void query_motor_status()
  {
      // 轮询发送读取角度命令
      // 简单错峰发送
      // 注意：发送频率不要太高以免堵塞 CAN 总线
      ser1_.readAngle();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      ser2_.readAngle();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }


  // === 主任务：发布 ROS 消息 ===
  void publish_data()
  {
    auto message = ihb_msgs::msg::SingleArmStatus();

    // 1. 设置 Header
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";

    // 2. 填充 霍尔传感器 / 肌肉关节 数据
    // -------------------------------------------------
    std::vector<float> hall_angles = hall_sensors_.getAllAngles();

    for (int i = 0; i < 4; i++) {
      message.muscle_joint[i].id = i + 1;     

      // 填充霍尔角度
      message.muscle_joint[i].angle[0] = static_cast<float>(hall_angles[i]);
      message.muscle_joint[i].angle[1] = static_cast<float>(hall_angles[7-i]); 

    }


      message.servo_motor[0].id = 1;
      message.servo_motor[0].angle = static_cast<float>(ser1_.MAngle());

      message.servo_motor[1].id = 2; 
      message.servo_motor[1].angle = static_cast<float>(ser2_.MAngle());

    // 4. 发布消息
    publisher_->publish(message);
    
    // 降低日志频率 (INFO级别)
    RCLCPP_INFO(this->get_logger(), "Data published: Servo1 Angle: %ld, Servo2 Angle: %ld", ser1_.MAngle(), ser2_.MAngle());
  }

  void start_motors() {
      ser1_.start(); ser2_.start();
  }

  // ROS 通信对象
  rclcpp::Publisher<ihb_msgs::msg::SingleArmStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr query_timer_;

  // 硬件对象
  ihb::HallSensors hall_sensors_;
  ihb::ServoMotorJoint ser1_;
  ihb::ServoMotorJoint ser2_;

  // 线程控制
  std::thread can_thread_;
  std::atomic<bool> running_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmStatusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
