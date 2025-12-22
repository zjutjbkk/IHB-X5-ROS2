#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <gpiod.hpp> // 使用 libgpiod C++ 绑定
// 将类放入 ihb 命名空间
namespace ihb
{
    class HallSensors
    {
    public:
        // 构造函数：初始化SPI和GPIO
        HallSensors();
        // 析构函数：释放资源
        ~HallSensors();

        // 禁止拷贝和赋值，因为包含独占的硬件资源
        HallSensors(const HallSensors &) = delete;
        HallSensors &operator=(const HallSensors &) = delete;

        /**
         * @brief 启动后台线程。
         * @param print_interval_ms 自动打印角度数据的时间间隔（单位：毫秒）。
         *                          如果为 0，则不启动自动打印功能。
         */
        void start(uint32_t print_interval_ms = 0); // 修改 start 函数签名，并提供默认值

        // 停止后台读取线程
        void stop();

        // 获取单个传感器的值（原始值，减去零点后）
        // 线程安全
        int16_t getValue(uint8_t sensor_index);

        // 获取单个传感器的角度值（浮点数）
        // 线程安全
        float getAngle(uint8_t sensor_index);

        // 获取所有16个传感器的角度值
        // 线程安全，返回一个值的副本，避免数据竞争
        std::vector<float> getAllAngles();

        // 设置当前位置为零点
        // 线程安全
        void setZero();

    private:
        // --- 新增: 打印线程相关 ---
        void printLoop();                            // 打印线程的循环函数
        std::thread print_thread_;                   // 打印线程对象
        std::atomic<uint32_t> print_interval_ms_{0}; // 打印间隔
        // 后台读取循环
        void readLoop();

        // SPI和GPIO硬件资源
        int spi1_fd_ = -1;
        int spi2_fd_ = -1;

        // libgpiod 资源
        gpiod::chip gpio_chip_;
        gpiod::line cs1_line_; // CS for SPI1
        gpiod::line cs2_line_; // CS for SPI2

        // 线程管理
        std::thread read_thread_;
        std::atomic<bool> running_{false};

        // 数据存储与同步
        std::mutex data_mutex_;
        std::vector<uint16_t> raw_values_;
        std::vector<uint16_t> zero_offsets_;

        // 常量定义
        static constexpr const char *SPI1_DEVICE = "/dev/spidev1.0";
        static constexpr const char *SPI2_DEVICE = "/dev/spidev2.0";
        static constexpr uint32_t SPI_SPEED_HZ = 1000000; // 提高速率以获得更好性能
        static constexpr uint8_t SPI_BITS_PER_WORD = 16;
        static constexpr uint8_t SPI_MODE = 1; // SPI_MODE_1

        // 根据 gpioinfo 的输出进行精确定义
        // gpiochip4 controls pins 379-410
        static constexpr const char *GPIO_CHIP_NAME = "gpiochip4";
        // Pin 394 (CS for SPI1) on gpiochip4 is at offset (394-379) = 15
        static constexpr int CS1_PIN_OFFSET = 15;
        // Pin 400 (CS for SPI2) on gpiochip4 is at offset (400-379) = 21
        static constexpr int CS2_PIN_OFFSET = 21;

        static constexpr int SENSOR_COUNT = 16;
        static constexpr float RAW_TO_DEGREE = 180.0f / 8192.0f;
    };
} // namespace ihb