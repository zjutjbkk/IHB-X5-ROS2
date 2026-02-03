#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <csignal>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_target.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_status.hpp"

// 按你要求：相对路径 include 底层头文件（最新版支持 double + getHeightPrecise）
#include "../lib/ihb_cpp/ihb_cpp/include/stepper_motor_joint.h"

// ----------------- 自己接管 Ctrl+C -----------------
static std::atomic<bool> g_exit{false};
static void sigint_handler(int) { g_exit.store(true); }

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

    // 订阅话题名称
    this->declare_parameter<std::string>("topic", "stepper_motor_joint_target");
    topic_ = this->get_parameter("topic").as_string();

    // 发布话题名称（状态）
    this->declare_parameter<std::string>("status_topic", "stepper_motor_joint_status");
    status_topic_ = this->get_parameter("status_topic").as_string();

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

    // ✅ 到位判定参数（小数版）
    // 允许误差（mm）——用 double 更适合小数运动
    this->declare_parameter<double>("arrive_tolerance_mm", 0.2); // 默认 0.2mm
    arrive_tolerance_mm_ = this->get_parameter("arrive_tolerance_mm").as_double();
    if (arrive_tolerance_mm_ < 0.0) arrive_tolerance_mm_ = 0.0;

    this->declare_parameter<int>("poll_ms", 20);              // 轮询周期（ms）
    poll_ms_ = this->get_parameter("poll_ms").as_int();
    if (poll_ms_ < 1) poll_ms_ = 1;

    this->declare_parameter<int>("arrive_timeout_ms", 8000);  // 超时（ms），0=不超时
    arrive_timeout_ms_ = this->get_parameter("arrive_timeout_ms").as_int();
    if (arrive_timeout_ms_ < 0) arrive_timeout_ms_ = 0;

    // ✅ 退出回零相关参数
    this->declare_parameter<int>("homing_speed_pps", 10000);
    homing_speed_pps_ = this->get_parameter("homing_speed_pps").as_int();
    if (homing_speed_pps_ < 1) homing_speed_pps_ = 1;
    if (homing_speed_pps_ > 65535) homing_speed_pps_ = 65535;

    this->declare_parameter<int>("shutdown_timeout_ms", 20000); // 回零等待超时，0=不超时
    shutdown_timeout_ms_ = this->get_parameter("shutdown_timeout_ms").as_int();
    if (shutdown_timeout_ms_ < 0) shutdown_timeout_ms_ = 0;

    // QoS：只保留最新一条命令
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    sub_ = this->create_subscription<ihb_msgs::msg::StepperMotorJointTarget>(
      topic_, qos,
      std::bind(&StepMotorSubscriber::on_target, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<ihb_msgs::msg::StepperMotorJointStatus>(status_topic_, qos);

    RCLCPP_INFO(this->get_logger(),
                "Stepper subscriber started. sub_topic='%s', status_topic='%s', accept id=%u, speed_pps=%d",
                topic_.c_str(), status_topic_.c_str(), joint_id_, speed_pps_);

    // 启动后重置当前位置为 0 点（一次性 timer）
    startup_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&StepMotorSubscriber::on_startup, this));

    // 启动后台线程：串行执行运动 + 到位后发布
    running_.store(true);
    worker_ = std::thread(&StepMotorSubscriber::worker_loop, this);
  }

  ~StepMotorSubscriber() override
  {
    stop_worker_();
  }

  // ✅ 退出前调用：回零 -> 等待完成 -> 发布0点状态
  void shutdown_home_and_publish()
  {
    // 先停 worker，避免 shutdown 时还在 move
    stop_worker_();

    try {
      RCLCPP_INFO(this->get_logger(), "Shutdown: homing to zero (moveToHeight(0.0)) ...");

      // 发回零命令（小数接口）
      motor_.moveToHeight(0.0, static_cast<uint16_t>(homing_speed_pps_));

      // 等待电机结束（优先用 isMoving）
      const auto t0 = std::chrono::steady_clock::now();
      while (rclcpp::ok()) {
        if (!motor_.isMoving()) break;

        if (shutdown_timeout_ms_ > 0) {
          const auto now = std::chrono::steady_clock::now();
          const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
          if (dt >= shutdown_timeout_ms_) {
            RCLCPP_WARN(this->get_logger(),
                        "Shutdown: homing wait timeout (%d ms). Will publish anyway.",
                        shutdown_timeout_ms_);
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms_));
      }

      // 发布回零状态（发布 0.0）
      publish_status(joint_id_, 0.0f);

      RCLCPP_INFO(this->get_logger(), "Shutdown: published zero status.");

      // 给 DDS 一点时间把消息发出去
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Shutdown homing failed: %s", e.what());
    }
  }

private:
  void stop_worker_()
  {
    running_.store(false);
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  void on_startup()
  {
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

      const double cur = motor_.getHeightPrecise();
      RCLCPP_INFO(this->get_logger(), "Startup reset complete. current height=%.3f mm", cur);

      // 启动也发一次状态（保留小数）
      publish_status(joint_id_, static_cast<float>(cur));
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

    // ✅ 直接使用 float32(mm) -> double(mm)，不再转 int16
    const double target_mm = static_cast<double>(msg->height);

    // 忽略重复目标（用 arrive_tolerance_mm_ 或非常小的 epsilon）
    if (ignore_same_target_) {
      std::lock_guard<std::mutex> lk(last_mutex_);
      if (has_last_target_) {
        const double diff = std::fabs(target_mm - last_target_mm_);
        const double eps = std::max(arrive_tolerance_mm_, 1e-6);
        if (diff <= eps) {
          RCLCPP_INFO(this->get_logger(), "Ignore same target: %.3f mm (diff=%.6f <= %.6f)", target_mm, diff, eps);
          return;
        }
      }
      last_target_mm_ = target_mm;
      has_last_target_ = true;
    }

    // 不在回调里直接 move（避免阻塞 executor），只“排队最新命令”
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      pending_target_mm_ = target_mm;
      has_pending_cmd_ = true;
    }
    cv_.notify_one();

    RCLCPP_INFO(this->get_logger(),
                "Queued moveToHeight: id=%u target=%.3f mm, speed=%d pps",
                msg->id, msg->height, speed_pps_);
  }

  void worker_loop()
  {
    while (running_.load()) {
      double target_mm = 0.0;

      {
        std::unique_lock<std::mutex> lk(cmd_mutex_);
        cv_.wait(lk, [&] { return !running_.load() || has_pending_cmd_; });
        if (!running_.load()) break;

        // 只取最新一条
        target_mm = pending_target_mm_;
        has_pending_cmd_ = false;
      }

      try {
        // ✅ 执行运动（小数接口）
        motor_.moveToHeight(target_mm, static_cast<uint16_t>(speed_pps_));

        // ✅ 用 getHeightPrecise 小数轮询判断到位
        const auto t0 = std::chrono::steady_clock::now();

        while (running_.load()) {
          const double cur = motor_.getHeightPrecise();
          const double err = std::fabs(cur - target_mm);

          if (err <= arrive_tolerance_mm_) {
            publish_status(joint_id_, static_cast<float>(cur));
            RCLCPP_INFO(this->get_logger(),
                        "Arrived. target=%.3f mm, current=%.3f mm (tol=%.3f). Published status.",
                        target_mm, cur, arrive_tolerance_mm_);
            break;
          }

          if (arrive_timeout_ms_ > 0) {
            const auto now = std::chrono::steady_clock::now();
            const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
            if (dt >= arrive_timeout_ms_) {
              publish_status(joint_id_, static_cast<float>(cur));
              RCLCPP_WARN(this->get_logger(),
                          "Arrive timeout. target=%.3f mm, current=%.3f mm (err=%.3f). Published status anyway.",
                          target_mm, cur, err);
              break;
            }
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms_));
        }

      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Move failed: %s", e.what());
      }
    }
  }

  void publish_status(uint8_t id, float height_mm)
  {
    ihb_msgs::msg::StepperMotorJointStatus st;
    st.id = id;
    st.height = height_mm;  // float32，保留小数
    status_pub_->publish(st);
  }

private:
  rclcpp::Subscription<ihb_msgs::msg::StepperMotorJointTarget>::SharedPtr sub_;
  rclcpp::Publisher<ihb_msgs::msg::StepperMotorJointStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  uint8_t joint_id_{0};
  int speed_pps_{10000};
  std::string topic_;
  std::string status_topic_;

  bool ignore_same_target_{true};
  bool stop_on_start_{true};
  int startup_delay_ms_{200};

  // ✅ 小数容差
  double arrive_tolerance_mm_{0.2};
  int poll_ms_{20};
  int arrive_timeout_ms_{8000};

  // ✅ shutdown 回零参数
  int homing_speed_pps_{10000};
  int shutdown_timeout_ms_{20000};

  // ✅ last target（小数）
  std::mutex last_mutex_;
  bool has_last_target_{false};
  double last_target_mm_{0.0};

  ihb::StepperMotorJoint motor_;

  // 命令队列（只保留最新）
  std::mutex cmd_mutex_;
  std::condition_variable cv_;
  bool has_pending_cmd_{false};
  double pending_target_mm_{0.0};

  std::atomic<bool> running_{false};
  std::thread worker_;
};

int main(int argc, char * argv[])
{
  // ✅ 关键：不让 ROS 自动在 Ctrl+C 时直接 shutdown
  rclcpp::InitOptions opt;
  opt.shutdown_on_signal = false;
  rclcpp::init(argc, argv, opt);

  std::signal(SIGINT, sigint_handler);

  auto node = std::make_shared<StepMotorSubscriber>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // 自己 spin，直到 Ctrl+C
  while (rclcpp::ok() && !g_exit.load()) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // ✅ shutdown 前：回零并发布 0 点状态（ROS 还活着，消息能发出去）
  node->shutdown_home_and_publish();

  // ✅ 再给 DDS 一点时间冲刷发送队列（可选但推荐）
  for (int i = 0; i < 20; ++i) { // ~200ms
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
