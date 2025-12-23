#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "ihb_msgs/msg/single_arm_status.hpp"
#include "ihb_msgs/msg/muscle_joint_status.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_status.hpp"
#include "ihb_msgs/msg/servo_motor_joint_status.hpp"
#include "ihb_msgs/msg/force_sensor_status.hpp"

// 包含霍尔传感器的头文件
#include "../lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"

using namespace std::chrono_literals;

class HallSensorPublisher : public rclcpp::Node
{
public:
  HallSensorPublisher()
  : Node("hall_sensor_publisher")
  {
    // 创建发布器，发布SingleArmStatus消息
    publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>(
      "arm_status", 10);
    
    // 初始化并启动霍尔传感器
    hall_sensors_.start(0); // 启动但不显示数据
    
    // 创建定时器，定期发布数据
    timer_ = this->create_wall_timer(
      60ms, std::bind(&HallSensorPublisher::publish_hall_data, this));
    
    RCLCPP_INFO(this->get_logger(), "霍尔传感器发布节点已启动, 将在arm_status话题上发布数据");
  }
  
  ~HallSensorPublisher()
  {
    // 停止霍尔传感器
    hall_sensors_.stop();
    RCLCPP_INFO(this->get_logger(), "霍尔传感器发布节点已关闭");
  }

private:
  void publish_hall_data()
  {
    // 创建SingleArmStatus消息
    auto message = ihb_msgs::msg::SingleArmStatus();

    // 设置消息头
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";

    // 优化：使用线程安全的批量读取接口
    std::vector<float> hall_angles = hall_sensors_.getAllAngles();

    // 从霍尔传感器读取8个角度值，分配到4个肌肉关节
    for (int i = 0; i < 4; i++) {
      // 设置肌肉关节ID (从1开始)
      message.muscle_joint[i].id = i + 1;

      // 设置压力值（默认为0）
      for (int j = 0; j < 4; j++) {
        message.muscle_joint[i].pressure[j] = 0;
      }

      // 设置刚度值（默认为0）
      for (int j = 0; j < 2; j++) {
        message.muscle_joint[i].stiffness[j] = 0;
      }

      // 优化：从批量读取的数据中获取角度值
      float angle1 = hall_angles[i];     // 第一个角度
      float angle2 = hall_angles[i+4];   // 第二个角度

      // 设置角度值（转为int16）
      message.muscle_joint[i].angle[0] = static_cast<int16_t>(angle1);
      // message.muscle_joint[i].angle[1] = static_cast<int16_t>(angle2);
      // RCLCPP_DEBUG(this->get_logger(), "肌肉%d: 角度1=%d, 角度2=%d", i+1, message.muscle_joint[i].angle[0], message.muscle_joint[i].angle[1]);

    }
    
    // 创建空的步进电机状态
    // message.stepper_motor = ihb_msgs::msg::StepperMotorJointStatus();
    
    // 创建空的力传感器状态数组
    // 注意：servo_motor和force_sensor是固定大小的数组，不需要调用resize()
    
    // 发布消息
    publisher_->publish(message);
    
    // 打印调试信息
    RCLCPP_DEBUG(this->get_logger(), "发布了新的霍尔传感器数据");
 
  }

  rclcpp::Publisher<ihb_msgs::msg::SingleArmStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  ihb::HallSensors hall_sensors_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HallSensorPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 