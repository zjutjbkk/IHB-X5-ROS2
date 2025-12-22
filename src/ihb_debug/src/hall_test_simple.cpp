#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iomanip>
#include <thread>
#include <chrono>
#include <sstream>
#include "hall_sensors.h"

using namespace std::chrono_literals;

class HallTestSimple : public rclcpp::Node 
{
public:
    HallTestSimple() : Node("hall_test_simple")
    {
        // 初始化并启动霍尔传感器
        hall_sensors_.start(0);
        RCLCPP_INFO(this->get_logger(), "简化霍尔传感器测试节点已启动");
        
        // 创建定时器每秒输出一次
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&HallTestSimple::publish_sensor_data, this));
    }
    
    ~HallTestSimple()
    {
        hall_sensors_.stop();
    }

private:
    void publish_sensor_data()
    {
        // 优化：使用线程安全的批量读取接口
        std::vector<float> hall_angles = hall_sensors_.getAllAngles();

        // 添加调试输出：直接显示传感器8-15的原始值
        RCLCPP_INFO(this->get_logger(), "===== 简化测试：传感器8-15原始数据 (线程安全版本) =====");
        std::stringstream debug_ss;
        debug_ss << "传感器8-15: ";
        for (int i = 8; i < 16; i++) {
            debug_ss << std::setw(6) << static_cast<int16_t>(hall_angles[i]) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", debug_ss.str().c_str());

        // 也输出前8个作为对比
        std::stringstream debug_ss2;
        debug_ss2 << "传感器0-7:  ";
        for (int i = 0; i < 8; i++) {
            debug_ss2 << std::setw(6) << static_cast<int16_t>(hall_angles[i]) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", debug_ss2.str().c_str());
    }

    ihb::HallSensors hall_sensors_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HallTestSimple>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}