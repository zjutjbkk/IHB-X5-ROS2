#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm> // 用于 std::nth_element (中值计算)
#include <deque>     // 用于存储滑动窗口数据

#include "ihb_msgs/msg/single_arm_status.hpp"
#include "ihb_msgs/msg/muscle_joint_status.hpp"
#include "../lib/ihb_cpp/ihb_cpp/include/hall_sensors.h"

using namespace std::chrono_literals;

class HallSensorPublisher : public rclcpp::Node
{
public:
  HallSensorPublisher()
  : Node("hall_sensor_publisher")
  {
    // 初始化
    filtered_angles_.assign(16, 0.0f);
    // 初始化中值滤波窗口，每个传感器保存最近 5 个值
    median_windows_.assign(16, std::deque<float>(WINDOW_SIZE, 0.0f));
    is_first_run_ = true;

    publisher_ = this->create_publisher<ihb_msgs::msg::SingleArmStatus>("arm_status", 10);
    hall_sensors_.start(0); 
    
    // 定时器间隔 30ms (约 33Hz)，提高采样频率可以让滤波器更灵敏
    timer_ = this->create_wall_timer(
      30ms, std::bind(&HallSensorPublisher::publish_hall_data, this));
    
    RCLCPP_INFO(this->get_logger(), "混合滤波器已启动: 中值滤波(窗口5) + 一阶低通(Alpha 0.2)");
  }
  
  ~HallSensorPublisher() { hall_sensors_.stop(); }

private:
  void publish_hall_data()
  {
    auto message = ihb_msgs::msg::SingleArmStatus();
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";

    std::vector<float> raw_angles = hall_sensors_.getAllAngles();
    if (raw_angles.size() < 16) 
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "等待传感器数据... 当前长度: %zu (需要16)", raw_angles.size());
    return;


    // --- 混合滤波逻辑 ---
    for (size_t i = 0; i < 16; ++i) {
      // 1. 更新中值滑动窗口
      median_windows_[i].push_back(raw_angles[i]);
      median_windows_[i].pop_front();

      // 2. 计算中值 (Median Filter)
      std::vector<float> window_copy(median_windows_[i].begin(), median_windows_[i].end());
      std::nth_element(window_copy.begin(), window_copy.begin() + WINDOW_SIZE / 2, window_copy.end());
      float median_val = window_copy[WINDOW_SIZE / 2];

      // 3. 一阶低通滤波 (Low-Pass Filter)
      if (is_first_run_) {
        filtered_angles_[i] = median_val;
      } else {
        // alpha = 0.2: 80% 历史 + 20% 新中值，非常稳
        const float alpha = 0.2f;
        filtered_angles_[i] = alpha * median_val + (1.0f - alpha) * filtered_angles_[i];
      }
    }
    is_first_run_ = false;

    // 分配到肌肉关节（保持你原来的 ID 3-i 和索引映射）
    for (int i = 0; i < 4; i++) {
      message.muscle_joint[i].id = i + 1;
      
      // 注意：i 是传感器索引，8+i 是配对的传感器
      float angle1 = filtered_angles_[i];     
      float angle2 = filtered_angles_[8+i];   

      message.muscle_joint[3-i].angle[0] = static_cast<int16_t>(std::round(angle1));
      message.muscle_joint[3-i].angle[1] = static_cast<int16_t>(std::round(angle2));
      RCLCPP_INFO(this->get_logger(), "肌肉%d: 角度1=%f, 角度2=%f", i+1, message.muscle_joint[i].angle[0], message.muscle_joint[i].angle[1]);

    }
    
    publisher_->publish(message);
  }

  // 参数配置
  static constexpr int WINDOW_SIZE = 5; // 中值滤波窗口大小
  
  rclcpp::Publisher<ihb_msgs::msg::SingleArmStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  ihb::HallSensors hall_sensors_;

  // 状态存储
  std::vector<float> filtered_angles_;
  std::vector<std::deque<float>> median_windows_;
  bool is_first_run_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HallSensorPublisher>());
  rclcpp::shutdown();
  return 0;
}