#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iomanip>
#include <thread>
#include <chrono>
#include "hall_sensors.h"

using namespace std::chrono_literals;

class HallTest : public rclcpp::Node 
{
public:
    HallTest() : Node("hall_test")
    {
        // 初始化并启动霍尔传感器
        hall_sensors_.start(0);
        RCLCPP_INFO(this->get_logger(), "霍尔传感器测试节点已启动");
        
        // 创建后台线程进行数据读取和显示 (使用线程安全的批量读取接口)
        read_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                // 优化：使用线程安全的批量读取接口，一次获取所有传感器数据
                std::vector<float> hall_angles = hall_sensors_.getAllFilteredAngles();

                // 直接读取并打印所有传感器值
                std::stringstream ss;
                ss << "传感器数据: ";
                for (int i = 0; i < 16; i++) {
                    ss << std::setw(6) << static_cast<int16_t>(hall_angles[i]) << " ";
                }
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

                // 打印分组数据，方便查看前8个和后8个
                std::stringstream ss1, ss2;
                ss1 << "前8个 (0-7):  ";
                for (int i = 0; i < 8; i++) {
                    ss1 << std::setw(6) << static_cast<int16_t>(hall_angles[i]) << " ";
                }
                RCLCPP_INFO(this->get_logger(), "%s", ss1.str().c_str());

                ss2 << "后8个 (8-15): ";
                for (int i = 8; i < 16; i++) {
                    ss2 << std::setw(6) << static_cast<int16_t>(hall_angles[i]) << " ";
                }
                RCLCPP_INFO(this->get_logger(), "%s", ss2.str().c_str());

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        });
    }
    
    ~HallTest()
    {
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        hall_sensors_.stop();
    }

private:
    ihb::HallSensors hall_sensors_;
    std::thread read_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HallTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 