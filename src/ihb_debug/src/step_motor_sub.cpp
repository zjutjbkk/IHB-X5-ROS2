#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_target.hpp"

// 按你要求：相对路径 include 底层头文件
#include "../lib/ihb_cpp/ihb_cpp/include/stepper_motor_joint.h"

class StepMotorSubscriber : public rclcpp::Node
{
public:
  StepMotorSubscriber()
  : Node("step_motor_subscriber"),
    motor_()
  {
    // 仅接收该 joint_id 的目标
    this->declare_parameter<int>("joint_id", 0);
    joint_id_ = static_cast<uint8_t>(this->get_parameter("joint_id").as_int());

    // 速度：pulses per second（传给 moveToHeight 的 speed）
    this->declare_parameter<int>("speed_pps", 10000);
    speed_pps_ = this->get_parameter("speed_pps").as_int();
    if (speed_pps_ < 1) speed_pps_ = 1;
    if (speed_pps_ > 65535) speed_pps_ = 65535;

    // 话题名称
    this->declare_parameter<std::string>("topic", "stepper_motor_joint_target");
    topic_ = this->get_parameter("topic").as_string();

    // 是否忽略相同目标（减少频繁 stop/start）
    this->declare_parameter<bool>("ignore_same_target", true);
    ignore_same_target_ = this->get_parameter("ignore_same_target").as_bool();

    // 启动后是否先 stop 再 resetZero（推荐打开）
    this->declare_parameter<bool>("stop_on_start", true);
    stop_on_start_ = this->get_parameter("stop_on_start").as_bool();

    // 启动延时（ms），给硬件初始化一点缓冲
    this->declare_parameter<int>("startup_delay_ms", 200);
    startup_delay_ms_ = this->get_parameter("startup_delay_ms").as_int();
    if (startup_delay_ms_ < 0) startup_delay_ms_ = 0;

    // QoS：只保留最新一条命令
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    sub_ = this->create_subscription<ihb_msgs::msg::StepperMotorJointTarget>(
      topic_, qos,
      std::bind(&StepMotorSubscriber::on_target, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Stepper subscriber started. topic='%s', accept id=%u, speed_pps=%d",
                topic_.c_str(), joint_id_, speed_pps_);

    // ---- 模仿你的测试程序：启动后重置当前位置为 0 点 ----
    // 用一个一次性 timer 来做，避免在构造函数里做耗时操作
    startup_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&StepMotorSubscriber::on_startup, this));
  }

private:
  void on_startup()
  {
    // 只执行一次
    if (!startup_timer_) return;
    startup_timer_->cancel();
    startup_timer_.reset();

    if (startup_delay_ms_ > 0) {
      RCLCPP_INFO(this->get_logger(), "Startup delay: %d ms", startup_delay_ms_);
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms_));
    }

    try {
      if (stop_on_start_) {
        RCLCPP_INFO(this->get_logger(), "Stop motor on start...");
        motor_.stopMotor();
      }

      RCLCPP_INFO(this->get_logger(), "Reset zero on start...");
      motor_.resetZero();

      RCLCPP_INFO(this->get_logger(), "Startup reset complete. current height=%d mm",
                  motor_.getHeight());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Startup reset failed: %s", e.what());
    }
  }

  void on_target(const ihb_msgs::msg::StepperMotorJointTarget::SharedPtr msg)
  {
    if (!msg) return;

    // 过滤 id
    if (msg->id != joint_id_) {
      RCLCPP_DEBUG(this->get_logger(), "Ignore target id=%u (expect %u)", msg->id, joint_id_);
      return;
    }

    // float32(mm) -> int16(mm) 四舍五入
    const int16_t target_mm =
      static_cast<int16_t>(std::lround(static_cast<double>(msg->height)));

    // 忽略重复目标
    if (ignore_same_target_) {
      std::lock_guard<std::mutex> lk(last_mutex_);
      if (has_last_target_ && target_mm == last_target_mm_) {
        RCLCPP_INFO(this->get_logger(), "Ignore same target: %d mm", target_mm);
        return;
      }
      last_target_mm_ = target_mm;
      has_last_target_ = true;
    }

    // 使用 moveToHeight 执行到目标点
    // 重要：根据你的实测，传入负值是上升，这里不做任何翻转处理
    RCLCPP_INFO(this->get_logger(),
                "Execute moveToHeight: id=%u target=%.3f mm -> %d mm, speed=%d pps",
                msg->id, msg->height, target_mm, speed_pps_);

    motor_.moveToHeight(target_mm, static_cast<uint16_t>(speed_pps_));
  }

private:
  rclcpp::Subscription<ihb_msgs::msg::StepperMotorJointTarget>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  uint8_t joint_id_{0};
  int speed_pps_{10000};
  std::string topic_;

  bool ignore_same_target_{true};
  bool stop_on_start_{true};
  int startup_delay_ms_{200};

  std::mutex last_mutex_;
  bool has_last_target_{false};
  int16_t last_target_mm_{0};

  ihb::StepperMotorJoint motor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StepMotorSubscriber>());
  rclcpp::shutdown();
  return 0;
}
