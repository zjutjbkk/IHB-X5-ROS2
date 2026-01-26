#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <algorithm>
#include <gpiod.hpp>

namespace ihb
{
    class HallSensors
    {
    public:
        HallSensors();
        ~HallSensors();

        // 禁止拷贝
        HallSensors(const HallSensors &) = delete;
        HallSensors &operator=(const HallSensors &) = delete;

        // 启动/停止读取线程
        void start(uint32_t print_interval_ms = 0);
        void stop();

        // 获取单个传感器角度（已滤波）
        float getAngle(uint8_t sensor_index);

        // 获取所有16个传感器的角度（已滤波）
        std::vector<float> getAllAngles();

        // 专门为你添加的接口名
        std::vector<float> getAllFilteredAngles() { return getAllAngles(); }

        // 设置零点
        void setZero();

    private:
        void readLoop();
        void printLoop();

        // 硬件资源
        int spi1_fd_ = -1;
        int spi2_fd_ = -1;
        gpiod::chip gpio_chip_;
        gpiod::line cs1_line_;
        gpiod::line cs2_line_;

        // 线程与同步
        std::thread read_thread_;
        std::thread print_thread_;
        std::atomic<bool> running_{false};
        std::atomic<uint32_t> print_interval_ms_{0};
        std::mutex data_mutex_;

        // 数据存储
        static constexpr int SENSOR_COUNT = 16;
        static constexpr int MEDIAN_WINDOW = 5; // 3点滤波

        std::vector<uint16_t> raw_values_;   // 存放滤波后的原始值
        std::vector<uint16_t> zero_offsets_; // 零点偏移

        // 每个传感器的历史记录：传感器索引 -> 3个历史采样值
        std::array<std::array<uint16_t, MEDIAN_WINDOW>, SENSOR_COUNT> history_buffers_;
        std::array<size_t, SENSOR_COUNT> history_ptr_ = {0};

        // 常量定义
        static constexpr const char *SPI1_DEVICE = "/dev/spidev1.0";
        static constexpr const char *SPI2_DEVICE = "/dev/spidev2.0";
        static constexpr uint32_t SPI_SPEED_HZ = 1000000;
        static constexpr uint8_t SPI_BITS_PER_WORD = 16;
        static constexpr uint8_t SPI_MODE = 1;

        static constexpr const char *GPIO_CHIP_NAME = "gpiochip4";
        static constexpr int CS1_PIN_OFFSET = 15; // Pin 394
        static constexpr int CS2_PIN_OFFSET = 21; // Pin 400
        static constexpr float RAW_TO_DEGREE = 180.0f / 8192.0f;

        static constexpr float LPF_ALPHA = 0.4f;                      // 平滑系数，0.4 是一个兼顾实时与平滑的平衡点
        std::array<float, SENSOR_COUNT> last_smooth_angles_ = {0.0f}; // 存储上一次滤波后的角度
    };
}