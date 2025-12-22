#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <cstdlib> // 用于system()函数
#include <thread>  // 用于std::this_thread::sleep_for
#include <fstream> // 用于读取文件检查CAN状态
#include <sys/socket.h> // 用于socket设置
#include <atomic>

#include "ihb_msgs/msg/ihb_status.hpp"
#include "ihb_msgs/msg/muscle_joint_status.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_status.hpp"
#include "ihb_msgs/msg/servo_motor_joint_status.hpp"
#include "ihb_msgs/msg/force_sensor_status.hpp"

// 包含霍尔传感器的头文件
#include "hall_sensors.h"
// 包含伺服电机的头文件
#include "servo_motorJoint.h"
// 包含步进电机的头文件
#include "stepper_motor_joint.h"
// 包含CAN驱动的头文件
#include "CanDriver.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std::chrono_literals;

// 标志变量，用于控制接收线程的运行
std::atomic<bool> g_running(true);

// 接收和处理CAN消息的线程函数
void receiveCanMessages(std::unique_ptr<ihb::ServoMotorJoint>& ser1, 
                        std::unique_ptr<ihb::ServoMotorJoint>& ser2,
                        std::unique_ptr<ihb::ServoMotorJoint>& ser3, 
                        std::unique_ptr<ihb::ServoMotorJoint>& ser4) {
    // 创建CAN驱动实例
    sockcanpp::CanDriver canDriver("can1", CAN_RAW);
    
    // 关闭回环，避免收到自己发送的消息
    int loopback = 0;
    setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    uint8_t data[8] = {0};

    // 持续接收消息，直到程序退出
    while (g_running) {
        // 优化：增加等待时间，减少CPU占用，提高系统整体性能
        if (canDriver.waitForMessages(std::chrono::milliseconds(5))) {
            // 读取一条消息
            sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

            // 将消息数据转换为数组
            for (int i = 0; i < receivedMessage.getFrameData().size(); i++) {
                data[i] = receivedMessage.getFrameData()[i];
            }

            // 获取CAN ID
            uint16_t canId = uint16_t(receivedMessage.getCanId());

            // 根据ID分发给对应的电机对象处理
            if (canId == 0x141) {
                ser1->processCanMessage(data);
            } else if (canId == 0x142) {
                ser2->processCanMessage(data);
            } else if (canId == 0x143) {
                ser3->processCanMessage(data);
            } else if (canId == 0x144) {
                ser4->processCanMessage(data);
            }
        } else {
            // 优化：没有消息时短暂休眠，避免忙等待
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

// 检查CAN总线状态
bool checkCanStatus(const std::string &interface = "can1")
{
  // 检查接口是否存在
  std::string path = "/sys/class/net/" + interface + "/operstate";
  std::ifstream statusFile(path);
  if (!statusFile.is_open())
  {
    return false; // 接口不存在
  }

  // 读取接口状态
  std::string status;
  std::getline(statusFile, status);
  statusFile.close();

  // 检查状态是否为"up"
  return (status == "up");
}

// 初始化CAN总线
bool initCanBus()
{
  // 首先检查CAN总线状态
  if (checkCanStatus("can1"))
  {
    return true; // 已经启动，无需重新初始化
  }

  int ret = 0;

  // 设置CAN总线比特率
  ret = system("sudo ip link set can1 type can bitrate 1000000");
  if (ret != 0)
  {
    return false;
  }

  // 休眠1秒
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // 启动CAN总线
  ret = system("sudo ip link set can1 up");
  if (ret != 0)
  {
    return false;
  }

  // 再休眠1秒确保CAN总线完全启动
  std::this_thread::sleep_for(std::chrono::seconds(5));

  return true;
}

class DataUpload : public rclcpp::Node
{
public:
  DataUpload()
      : Node("data_upload")
  {
    RCLCPP_INFO(this->get_logger(), "=== IHB-X5 数据上传节点启动 (线程安全优化版本) ===");

    // 首先初始化CAN总线
    if (initCanBus())
    {
      RCLCPP_INFO(this->get_logger(), "✅ CAN总线初始化成功或已经启动");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "❌ CAN总线初始化失败，请检查权限或手动启动");
    }

    // 创建发布器，发布IhbStatus消息
    publisher_ = this->create_publisher<ihb_msgs::msg::IhbStatus>(
        "ihb_status", 10);
    RCLCPP_INFO(this->get_logger(), "📡 ROS2发布器已创建: ihb_status");

    // 初始化伺服电机（先初始化CAN相关组件）
    RCLCPP_INFO(this->get_logger(), "🔧 初始化伺服电机...");
    try {
      ser1_ = std::make_unique<ihb::ServoMotorJoint>(1);
      ser2_ = std::make_unique<ihb::ServoMotorJoint>(2);
      ser3_ = std::make_unique<ihb::ServoMotorJoint>(3);
      ser4_ = std::make_unique<ihb::ServoMotorJoint>(4);
      RCLCPP_INFO(this->get_logger(), "✅ 伺服电机对象创建成功");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "❌ 伺服电机初始化失败: %s", e.what());
      throw;
    }

    // 初始化步进电机
    RCLCPP_INFO(this->get_logger(), "🔧 初始化步进电机...");
    try {
      stepper_motor_ = new ihb::StepperMotorJoint();
      RCLCPP_INFO(this->get_logger(), "✅ 步进电机初始化成功");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "❌ 步进电机初始化失败: %s", e.what());
      stepper_motor_ = nullptr;
    }

    // 启动伺服电机
    RCLCPP_INFO(this->get_logger(), "🚀 启动伺服电机...");
    ser1_->start();
    ser2_->start();
    ser3_->start();
    ser4_->start();
    
    // 重置运行标志并启动接收线程
    g_running = true;
    RCLCPP_INFO(this->get_logger(), "🔄 启动CAN接收线程...");
    receiveThread_ = std::thread(receiveCanMessages, std::ref(ser1_),
                                std::ref(ser2_), std::ref(ser3_), std::ref(ser4_));

    // 等待CAN接收线程启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(this->get_logger(), "✅ CAN接收线程已启动");

    // **重要：最后初始化霍尔传感器，避免与CAN/GPIO冲突**
    RCLCPP_INFO(this->get_logger(), "🔧 正在初始化霍尔传感器 (线程安全版本)...");
    try {
      hall_sensors_.start(0); // 启动但不显示数据

      // 给霍尔传感器充分的初始化时间
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // 测试所有传感器是否正常工作
      std::vector<float> test_angles = hall_sensors_.getAllAngles();
      int working_sensors = 0;
      for (int i = 0; i < 16; i++) {
        if (std::abs(test_angles[i]) > 0.1) { // 检查是否有有效读数
          working_sensors++;
        }
      }

      RCLCPP_INFO(this->get_logger(), "✅ 霍尔传感器初始化完成，%d/16个传感器有读数", working_sensors);

      if (working_sensors < 8) {
        RCLCPP_WARN(this->get_logger(), "⚠️ 部分霍尔传感器可能未正常工作，但会继续运行");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "❌ 霍尔传感器初始化异常: %s", e.what());
    }
    
    // 初始读取电机状态
    ser1_->readAngle();
    ser2_->readAngle();
    ser3_->readAngle();
    ser4_->readAngle();
    
    ser1_->readStatus2();
    ser2_->readStatus2();
    ser3_->readStatus2();
    ser4_->readStatus2();
    ser1_->setPositonWithSpeedLimit(0 * 100, 120);
    ser2_->setPositonWithSpeedLimit(0 * 100, 120);
    ser3_->setPositonWithSpeedLimit(5 * 100, 120);
    ser4_->setPositonWithSpeedLimit(0 * 100, 120);
    
    // 创建定时器，定期发布数据
    timer_ = this->create_wall_timer(
        60ms, std::bind(&DataUpload::publish_status_data, this));
    
    RCLCPP_INFO(this->get_logger(), "IHB状态发布节点已启动，将在ihb_status话题上发布数据");
    
    // 临时禁用步进电机测试线程，避免与霍尔传感器SPI通信冲突
    // TODO: 修复步进电机PWM与霍尔传感器SPI2的干扰问题
    RCLCPP_INFO(this->get_logger(), "步进电机测试已禁用，避免与霍尔传感器SPI通信冲突");
    
    stepper_test_thread_ = std::thread([this]() {
      try {
        // 等待5秒，确保系统稳定后再测试
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        if (stepper_motor_ != nullptr) {
          // 先归零
          RCLCPP_INFO(this->get_logger(), "步进电机归零...");
          stepper_motor_->moveToHeight(0, 1000);
          // 等待动作完成
          std::this_thread::sleep_for(std::chrono::seconds(3));
          
          // 后退10mm
          RCLCPP_INFO(this->get_logger(), "步进电机移动到 10mm...");
          stepper_motor_->moveToHeight(10, 2000);
          // 等待动作完成
          std::this_thread::sleep_for(std::chrono::seconds(8));
          
          // 读取高度并显示
          int height = stepper_motor_->getHeight();
          RCLCPP_INFO(this->get_logger(), "步进电机当前高度: %d mm", height);
          
          // 测试完成后归零
          RCLCPP_INFO(this->get_logger(), "测试完成，ctrl+c试试析构归零...");
          // stepper_motor_->moveToHeight(0, 2000);
          // std::this_thread::sleep_for(std::chrono::seconds(8));
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "步进电机测试异常: %s", e.what());
      }
    });
    // 分离线程，让它在后台运行
    stepper_test_thread_.detach();
  }
  
  ~DataUpload()
  {
    RCLCPP_INFO(this->get_logger(), "🛑 开始安全关闭IHB-X5系统...");

    // 首先停止CAN接收线程
    g_running = false;
    if (receiveThread_.joinable()) {
      RCLCPP_INFO(this->get_logger(), "🔄 等待CAN接收线程退出...");
      receiveThread_.join();
      RCLCPP_INFO(this->get_logger(), "✅ CAN接收线程已安全退出");
    }

    // 停止霍尔传感器
    RCLCPP_INFO(this->get_logger(), "🔄 停止霍尔传感器...");
    hall_sensors_.stop();

    // 伺服电机安全复位
    RCLCPP_INFO(this->get_logger(), "🔄 伺服电机复位到零位...");
    try {
      ser1_->setPositonWithSpeedLimit(0 * 100, 120);
      ser2_->setPositonWithSpeedLimit(0 * 100, 120);
      ser3_->setPositonWithSpeedLimit(0 * 100, 120);
      ser4_->setPositonWithSpeedLimit(0 * 100, 120);

      // 等待复位完成
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // 停止伺服电机
      ser1_->close();
      ser2_->close();
      ser3_->close();
      ser4_->close();
      RCLCPP_INFO(this->get_logger(), "✅ 伺服电机已安全关闭");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "⚠️ 伺服电机关闭异常: %s", e.what());
    }

    // 停止步进电机
    if (stepper_motor_ != nullptr) {
      RCLCPP_INFO(this->get_logger(), "🔄 停止步进电机...");
      try {
        delete stepper_motor_;
        stepper_motor_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "✅ 步进电机已安全关闭");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "⚠️ 步进电机关闭异常: %s", e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(), "✅ IHB-X5系统已安全关闭");
  }

private:
  void publish_status_data()
  {
    // 优化：使用线程安全接口，定期查询伺服电机状态
    static int query_counter = 0;
    query_counter++;

    // 每5次循环（约300ms）查询一次伺服电机状态，减少CAN总线负载
    if (query_counter % 5 == 0) {
      ser1_->readAngle();
      ser2_->readAngle();
      ser3_->readAngle();
      ser4_->readAngle();

      // 每10次循环查询一次详细状态
      if (query_counter % 10 == 0) {
        ser1_->readStatus2();
        ser2_->readStatus2();
        ser3_->readStatus2();
        ser4_->readStatus2();
      }
    }

    // 测试时运行电机的命令 (可根据需要启用)
    // ser1_->setPositonWithSpeedLimit(0 * 100, 120);
    // ser2_->setPositonWithSpeedLimit(10 * 100, 120);
    // ser3_->setPositonWithSpeedLimit(20 * 100, 120);
    // ser4_->setPositonWithSpeedLimit(0 * 100, 120);
    
    // 创建IhbStatus消息
    auto message = ihb_msgs::msg::IhbStatus();

    // 设置消息头
    message.header.stamp = this->now();
    message.header.frame_id = "";

    // 优化：使用线程安全的批量读取接口，一次性获取所有传感器数据
    std::vector<float> hall_angles = hall_sensors_.getAllAngles();

    // 左臂前四个肌肉关节映射 (保持原有角度关系不变)
    // 第一个肌肉：角度1 = 传感器0，角度2 = 传感器7
    // 第二个肌肉：角度1 = 传感器1，角度2 = 传感器6
    // 第三个肌肉：角度1 = 传感器2，角度2 = 传感器5
    // 第四个肌肉：角度1 = 传感器3，角度2 = 传感器4
    for (int i = 0; i < 4; i++)
    {
      // 设置肌肉关节ID (从1开始)
      message.muscle_joint[i].id = i + 1;

      // 设置压力值（默认为0）
      for (int j = 0; j < 4; j++)
      {
        message.muscle_joint[i].pressure[j] = 0;
      }

      // 设置刚度值（默认为0）
      for (int j = 0; j < 2; j++)
      {
        message.muscle_joint[i].stiffness[j] = 0;
      }

      // 优化：从批量读取的数据中获取角度值，保持原有映射关系
      int16_t angle1 = static_cast<int16_t>(hall_angles[i]);     // 第i个传感器
      int16_t angle2 = static_cast<int16_t>(hall_angles[7 - i]); // 第(7-i)个传感器

      // 设置角度值
      message.muscle_joint[i].angle[0] = angle1;
      message.muscle_joint[i].angle[1] = angle2;
    }

    // 优化：移除不必要的调试输出，减少日志开销
    // 如需调试传感器8-15数据，可临时启用以下代码：
    // for (int i = 8; i < 16; i++) {
    //   RCLCPP_DEBUG(this->get_logger(), "传感器%d: %.1f", i, hall_angles[i]);
    // }

    // 右臂后四个肌肉关节映射 (保持原有角度关系不变)
    // 第五个肌肉：角度1 = 传感器8，角度2 = 传感器15
    // 第六个肌肉：角度1 = 传感器9，角度2 = 传感器14
    // 第七个肌肉：角度1 = 传感器10，角度2 = 传感器13
    // 第八个肌肉：角度1 = 传感器11，角度2 = 传感器12
    for (int i = 4; i < 8; i++)
    {
      // 设置肌肉关节ID (从5开始)
      message.muscle_joint[i].id = i + 1;

      // 设置压力值（默认为0）
      for (int j = 0; j < 4; j++)
      {
        message.muscle_joint[i].pressure[j] = 0;
      }

      // 设置刚度值（默认为0）
      for (int j = 0; j < 2; j++)
      {
        message.muscle_joint[i].stiffness[j] = 0;
      }

      // 优化：从批量读取的数据中获取角度值，保持原有映射关系
      int16_t angle1 = static_cast<int16_t>(hall_angles[i + 4]);  // 第(i+4)个传感器 (8,9,10,11)
      int16_t angle2 = static_cast<int16_t>(hall_angles[19 - i]); // 第(19-i)个传感器 (15,14,13,12)

      // 详细调试输出 (可选启用)
      // RCLCPP_DEBUG(this->get_logger(), "肌肉%d: 传感器%d=%.1f, 传感器%d=%.1f",
      //              i+1, i+4, hall_angles[i+4], 19-i, hall_angles[19-i]);

      // 设置角度值
      message.muscle_joint[i].angle[0] = angle1;
      message.muscle_joint[i].angle[1] = angle2;
    }

    // 设置步进电机状态
    message.stepper_motor.id = 1; // 假设只有一个步进电机
    try {
      if (stepper_motor_ != nullptr) {
        message.stepper_motor.height = stepper_motor_->getHeight();
      } else {
        message.stepper_motor.height = 0;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "读取步进电机高度异常: %s", e.what());
      message.stepper_motor.height = 0; // 出错时设为0
    }
    // 可以添加更多步进电机的信息，如速度等

    // 优化：设置伺服电机状态，使用数组简化代码
    std::array<std::unique_ptr<ihb::ServoMotorJoint>*, 4> servo_motors = {
      &ser1_, &ser2_, &ser3_, &ser4_
    };

    for (int i = 0; i < 4; i++)
    {
      message.servo_motor[i].id = i + 1;

      // 使用线程安全接口获取伺服电机数据
      if (servo_motors[i] && *servo_motors[i]) {
        message.servo_motor[i].angle = (*servo_motors[i])->MAngle();
        message.servo_motor[i].current = (*servo_motors[i])->Current();
        message.servo_motor[i].speed = (*servo_motors[i])->Speed();
        message.servo_motor[i].temp = 0; // 默认温度为0
      } else {
        // 安全处理：如果电机对象无效，设置默认值
        message.servo_motor[i].angle = 0;
        message.servo_motor[i].current = 0;
        message.servo_motor[i].speed = 0;
        message.servo_motor[i].temp = 0;
      }
    }

    // 设置力传感器状态数组（默认为空）
    // 力传感器暂不处理

    // 发布消息
    publisher_->publish(message);

    // 优化：减少日志输出频率，避免影响实时性能，增加性能监控
    static int log_counter = 0;
    static auto last_log_time = std::chrono::steady_clock::now();
    log_counter++;

    // 每50次循环（约3秒）输出一次详细日志
    if (log_counter % 50 == 0) {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_log_time);
        double actual_frequency = 50.0 * 1000.0 / duration.count(); // 实际频率

        RCLCPP_INFO(this->get_logger(), "===== IHB-X5 系统状态 (线程安全优化版本) =====");
        RCLCPP_INFO(this->get_logger(), "📊 性能监控: 实际频率 %.1f Hz (目标: 16.7 Hz)", actual_frequency);
        RCLCPP_INFO(this->get_logger(), "🔧 步进电机高度: %d mm", message.stepper_motor.height);

        RCLCPP_INFO(this->get_logger(), "🦾 左臂数据:");
        RCLCPP_INFO(this->get_logger(), "  - 肩部伺服: [%ld°, %ld°]",
                    message.servo_motor[0].angle, message.servo_motor[1].angle);
        RCLCPP_INFO(this->get_logger(), "  - 肌肉1-4: [%d,%d] [%d,%d] [%d,%d] [%d,%d]",
                    message.muscle_joint[0].angle[0], message.muscle_joint[0].angle[1],
                    message.muscle_joint[1].angle[0], message.muscle_joint[1].angle[1],
                    message.muscle_joint[2].angle[0], message.muscle_joint[2].angle[1],
                    message.muscle_joint[3].angle[0], message.muscle_joint[3].angle[1]);

        RCLCPP_INFO(this->get_logger(), "🦾 右臂数据:");
        RCLCPP_INFO(this->get_logger(), "  - 肩部伺服: [%ld°, %ld°]",
                    message.servo_motor[2].angle, message.servo_motor[3].angle);
        RCLCPP_INFO(this->get_logger(), "  - 肌肉5-8: [%d,%d] [%d,%d] [%d,%d] [%d,%d]",
                    message.muscle_joint[4].angle[0], message.muscle_joint[4].angle[1],
                    message.muscle_joint[5].angle[0], message.muscle_joint[5].angle[1],
                    message.muscle_joint[6].angle[0], message.muscle_joint[6].angle[1],
                    message.muscle_joint[7].angle[0], message.muscle_joint[7].angle[1]);

        last_log_time = current_time;
    }

    // 原有的调试信息
    RCLCPP_DEBUG(this->get_logger(), "发布了新的IHB状态数据");
  }

  rclcpp::Publisher<ihb_msgs::msg::IhbStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  ihb::HallSensors hall_sensors_;

  // 伺服电机对象
  std::unique_ptr<ihb::ServoMotorJoint> ser1_;
  std::unique_ptr<ihb::ServoMotorJoint> ser2_;
  std::unique_ptr<ihb::ServoMotorJoint> ser3_;
  std::unique_ptr<ihb::ServoMotorJoint> ser4_;
  
  // 步进电机对象
  ihb::StepperMotorJoint* stepper_motor_ = nullptr;

  // 接收线程
  std::thread receiveThread_;
  
  // 步进电机测试线程
  std::thread stepper_test_thread_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataUpload>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}