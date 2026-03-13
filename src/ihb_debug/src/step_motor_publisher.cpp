#include <chrono>
#include <cstdint>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <atomic>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_target.hpp"

using namespace std::chrono_literals;

// 全局退出标志，用于接管 Ctrl+C
std::atomic<bool> g_exit{false};
void sigint_handler(int) {
  g_exit.store(true);
}

class StepMotorPublisher : public rclcpp::Node
{
public:
  StepMotorPublisher()
  : Node("step_motor_publisher"),
    current_height_mm_(0.0f)
  {
    this->declare_parameter<int>("joint_id", 0);
    joint_id_ = static_cast<uint8_t>(this->get_parameter("joint_id").as_int());

    this->declare_parameter<std::string>("topic", "stepper_motor_joint_target");
    topic_ = this->get_parameter("topic").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_ = this->create_publisher<ihb_msgs::msg::StepperMotorJointTarget>(topic_, qos);

    RCLCPP_INFO(this->get_logger(), "Stepper target publisher started. topic='%s', id=%u", topic_.c_str(), joint_id_);
    
    // 启动键盘监听独立线程
    keyboard_thread_ = std::thread(&StepMotorPublisher::keyboard_loop, this);
  }

  // 对外暴露的退出清理函数
  void shutdown_and_reset()
  {
    // 等待键盘线程安全退出
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "Node exiting... Resetting height to 0.0 mm");
    current_height_mm_ = 0.0f;
    publish_target(current_height_mm_);

    // 给 DDS 底层一点时间把这条 0.0 的消息发送出去
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

private:
  void publish_target(float height)
  {
    ihb_msgs::msg::StepperMotorJointTarget msg;
    msg.id = joint_id_;
    msg.height = height;
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published target: id=%u, height=%.2f mm", msg.id, msg.height);
  }

  void keyboard_loop()
  {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);          // 关闭行缓冲和回显
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;                      // 100ms 超时
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "\n=========================================\n"
              << "  Keyboard Control Started:\n"
              << "  [W] : Decrease height by 0.5 mm\n"
              << "  [S] : Increase height by 0.5 mm\n"
              << "  [R] : Reset height to 0.0 mm\n"
              << "  [Q] : Quit and reset height to 0.0 mm\n"
              << "  [Number + Enter] : Go to target mm (e.g., 10.5)\n"
              << "=========================================\n" << std::endl;

    std::string input_buf = "";

    while (rclcpp::ok() && !g_exit.load()) {
      unsigned char c;
      int bytes_read = read(STDIN_FILENO, &c, 1);
      
      if (bytes_read <= 0) {
        continue;
      }

      // 1. 处理回车键 (解析输入的数字)
      if (c == '\n' || c == '\r') {
        if (!input_buf.empty()) {
          std::cout << "\n"; // 换行，防止日志覆盖输入内容
          try {
            float val = std::stof(input_buf);
            current_height_mm_ = val;
            publish_target(current_height_mm_);
          } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Invalid input: %s", input_buf.c_str());
          }
          input_buf.clear();
        }
      }
      // 2. 处理退格键 (Backspace)
      else if (c == 127 || c == '\b') {
        if (!input_buf.empty()) {
          input_buf.pop_back();
          std::cout << "\b \b" << std::flush; // 在终端上抹掉最后一个字符
        }
      }
      // 3. 处理快捷键 (W, S, R, Q)
      else if (std::isalpha(c)) {
        char lc = std::tolower(c);
        if (lc == 'w' || lc == 's' || lc == 'r' || lc == 'q') {
          // 如果正在输入数字中途按了快捷键，则清空刚才输入的数字
          if (!input_buf.empty()) {
            std::cout << "\n[Input cancelled]\n";
            input_buf.clear();
          }

          if (lc == 'w') {
            current_height_mm_ -= 0.5f;
            publish_target(current_height_mm_);
          } else if (lc == 's') {
            current_height_mm_ += 0.5f;
            publish_target(current_height_mm_);
          } else if (lc == 'r') {
            current_height_mm_ = 0.0f;
            publish_target(current_height_mm_);
            std::cout << "--- Height Reset ---\n";
          } else if (lc == 'q') {
            std::cout << "\nQuit command received.\n";
            g_exit.store(true);
            break;
          }
        }
      }
      // 4. 处理数字及小数点、负号
      else if (std::isdigit(c) || c == '.' || c == '-') {
        input_buf += c;
        std::cout << c << std::flush; // 手动回显到终端
      }
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

private:
  rclcpp::Publisher<ihb_msgs::msg::StepperMotorJointTarget>::SharedPtr pub_;
  uint8_t joint_id_{0};
  std::string topic_;
  float current_height_mm_;
  std::thread keyboard_thread_;
};

int main(int argc, char * argv[])
{
  // 初始化配置：不让 ROS 在收到 Ctrl+C 时直接强杀，我们自己接管
  rclcpp::InitOptions opt;
  opt.shutdown_on_signal = false;
  rclcpp::init(argc, argv, opt);

  // 绑定系统的 Ctrl+C 信号
  std::signal(SIGINT, sigint_handler);

  auto node = std::make_shared<StepMotorPublisher>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // 自己控制 spin，按 Q 或者 Ctrl+C 都会退出这个 while
  while (rclcpp::ok() && !g_exit.load()) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // ======== 关键：退出前自动归 0 ========
  node->shutdown_and_reset();

  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}