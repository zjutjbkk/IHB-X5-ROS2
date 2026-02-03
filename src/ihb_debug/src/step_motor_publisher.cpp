#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_target.hpp"

using namespace std::chrono_literals;

class StepMotorPublisher : public rclcpp::Node
{
public:
  StepMotorPublisher()
  : Node("step_motor_publisher"),
    current_height_mm_(0.0f),
    step_mm_(-kStepMm)  // 初始每次减 0.5mm
  {
    // 参数：joint_id（消息里的 id）
    this->declare_parameter<int>("joint_id", 0);
    joint_id_ = static_cast<uint8_t>(this->get_parameter("joint_id").as_int());

    // 话题名
    this->declare_parameter<std::string>("topic", "stepper_motor_joint_target");
    topic_ = this->get_parameter("topic").as_string();

    // QoS：只保留最新一条命令，避免堆积
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_ = this->create_publisher<ihb_msgs::msg::StepperMotorJointTarget>(topic_, qos);

    // 10 秒一次（如需 1Hz 改为 1s）
    timer_ = this->create_wall_timer(10s, std::bind(&StepMotorPublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
                "Stepper target publisher started. topic='%s', id=%u, start=%.2fmm, range=[%.2f,%.2f], step=%.2fmm",
                topic_.c_str(), joint_id_, current_height_mm_, kMinMm, kMaxMm, kStepMm);
  }

private:
  void on_timer()
  {
    // 1) 发布当前目标
    ihb_msgs::msg::StepperMotorJointTarget msg;
    msg.id = joint_id_;
    msg.height = current_height_mm_;
    pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Publish target: id=%u height=%.2fmm", msg.id, msg.height);

    // 2) 计算下一次目标
    float next = current_height_mm_ + step_mm_;

    // 触底：到 -2 后改为上升
    if (next <= kMinMm) {
      next = kMinMm;
      step_mm_ = +kStepMm;
    }

    // 触顶：到 2 后改为下降
    if (next >= kMaxMm) {
      next = kMaxMm;
      step_mm_ = -kStepMm;
    }

    current_height_mm_ = next;
  }

private:
  static constexpr float kMinMm  = -2.0f;
  static constexpr float kMaxMm  =  2.0f;
  static constexpr float kStepMm =  0.5f;

  rclcpp::Publisher<ihb_msgs::msg::StepperMotorJointTarget>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  uint8_t joint_id_{0};
  std::string topic_;

  float current_height_mm_;
  float step_mm_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StepMotorPublisher>());
  rclcpp::shutdown();
  return 0;
}
