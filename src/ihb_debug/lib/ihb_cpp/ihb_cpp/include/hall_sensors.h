#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>     // 新增：用于中值滤波的滑动窗口
#include <gpiod.hpp> 

namespace ihb
{
    class HallSensors
    {
    public:
        HallSensors();
        ~HallSensors();

        HallSensors(const HallSensors &) = delete;
        HallSensors &operator=(const HallSensors &) = delete;

        /**
         * @brief 启动后台线程。
         */
        void start(uint32_t print_interval_ms = 0);

        void stop();

        // 获取原始值
        int16_t getValue(uint8_t sensor_index);
        float getAngle(uint8_t sensor_index);
        std::vector<float> getAllAngles();

        /**
         * @brief 获取混合滤波后的角度值 (中值滤波 + 一阶低通)
         * @param alpha 一阶低通系数 (0.0-1.0)，越小越平滑，延迟越大
         * @param window_size 中值滤波窗口大小，必须为奇数
         */
        std::vector<float> getFilteredAngles(float alpha = 0.2f, size_t window_size = 5);

        void setZero();

    private:
        void printLoop();                            
        std::thread print_thread_;                   
        std::atomic<uint32_t> print_interval_ms_{0}; 
        void readLoop();

        // SPI和GPIO硬件资源
        int spi1_fd_ = -1;
        int spi2_fd_ = -1;
        gpiod::chip gpio_chip_;
        gpiod::line cs1_line_; 
        gpiod::line cs2_line_; 

        // 线程管理
        std::thread read_thread_;
        std::atomic<bool> running_{false};

        // 数据存储与同步
        std::mutex data_mutex_;
        std::vector<uint16_t> raw_values_;
        std::vector<uint16_t> zero_offsets_;

        // --- 新增：滤波状态存储 ---
        std::vector<float> last_lpf_values_;          // 存储上一时刻的一阶低通值
        std::vector<std::deque<float>> median_queues_; // 存储每个传感器的中值窗口
        bool filters_initialized_ = false;

        // 常量定义
        static constexpr const char *SPI1_DEVICE = "/dev/spidev1.0";
        static constexpr const char *SPI2_DEVICE = "/dev/spidev2.0";
        static constexpr uint32_t SPI_SPEED_HZ = 1000000; 
        static constexpr uint8_t SPI_BITS_PER_WORD = 16;
        static constexpr uint8_t SPI_MODE = 1; 

        static constexpr const char *GPIO_CHIP_NAME = "gpiochip4";
        static constexpr int CS1_PIN_OFFSET = 15;
        static constexpr int CS2_PIN_OFFSET = 21;

        static constexpr int SENSOR_COUNT = 16;
        static constexpr float RAW_TO_DEGREE = 180.0f / 8192.0f;
    };
} // namespace ihb