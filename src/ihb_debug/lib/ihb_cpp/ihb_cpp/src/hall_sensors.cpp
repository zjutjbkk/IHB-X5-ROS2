#include "hall_sensors.h"
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <algorithm> // 新增：用于排序实现中值滤波

namespace ihb 
{

HallSensors::HallSensors()
    : raw_values_(SENSOR_COUNT, 0), zero_offsets_(SENSOR_COUNT, 0) 
{
    // --- 1. 初始化GPIO ---
    try {
        gpio_chip_ = gpiod::chip(GPIO_CHIP_NAME, gpiod::chip::OPEN_BY_NAME);
        cs1_line_ = gpio_chip_.get_line(CS1_PIN_OFFSET);
        cs1_line_.request({"hall_sensors_cs1", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
        cs2_line_ = gpio_chip_.get_line(CS2_PIN_OFFSET);
        cs2_line_.request({"hall_sensors_cs2", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("GPIO 初始化失败: ") + e.what());
    }
    
    // --- 2. 初始化SPI ---
    auto init_spi_device = [&](const char* device_path) -> int {
        int fd = open(device_path, O_RDWR);
        if (fd < 0) throw std::runtime_error(std::string("无法打开 SPI 设备: ") + device_path);
        
        uint8_t mode = SPI_MODE;
        uint8_t bits = SPI_BITS_PER_WORD;
        uint32_t speed = SPI_SPEED_HZ;

        if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            close(fd);
            throw std::runtime_error(std::string("配置 SPI 设备失败: ") + device_path);
        }
        return fd;
    };

    try {
        spi1_fd_ = init_spi_device(SPI1_DEVICE);
        spi2_fd_ = init_spi_device(SPI2_DEVICE);
    } catch (const std::runtime_error& e) {
        if (spi1_fd_ >= 0) close(spi1_fd_);
        if (spi2_fd_ >= 0) close(spi2_fd_);
        throw; 
    }
    
    std::cout << "HallSensors 初始化成功！" << std::endl;
}

HallSensors::~HallSensors() {
    stop(); 
    if (spi1_fd_ >= 0) close(spi1_fd_);
    if (spi2_fd_ >= 0) close(spi2_fd_);
    std::cout << "HallSensors 资源已释放。" << std::endl;
}

void HallSensors::start(uint32_t print_interval_ms) {
    if (running_.exchange(true)) {
        print_interval_ms_.store(print_interval_ms);
        return;
    }

    read_thread_ = std::thread(&HallSensors::readLoop, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    setZero();
    
    print_interval_ms_.store(print_interval_ms);
    if (print_interval_ms > 0) {
        print_thread_ = std::thread(&HallSensors::printLoop, this);
    }
}

void HallSensors::printLoop() {
    while (running_) {
        uint32_t interval = print_interval_ms_.load();
        if (interval == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        // 修改点：打印循环可以改用滤波后的值，观察效果更明显
        std::vector<float> angles = getAllAngles();
        
        std::cout << "\r"; 
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            std::cout << "S" << std::setw(2) << i << ":" 
                      << std::fixed << std::setprecision(2) << std::setw(7) << angles[i] << " | ";
            if (i == 7) std::cout << "\n";
        }
        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    }
}

void HallSensors::stop() {
    if (running_.exchange(false)) {
        if (read_thread_.joinable()) read_thread_.join();
        if (print_thread_.joinable()) print_thread_.join();
    }
}

void HallSensors::readLoop() {
    uint16_t tx_buffer[8] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    uint16_t rx_buffer[8];
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = sizeof(tx_buffer),
        .speed_hz = SPI_SPEED_HZ,
        .bits_per_word = SPI_BITS_PER_WORD,
    };

    // 定义突变阈值：14位下 2度约为 91 个原始数值单位 (16384 / 360 * 2)
    const int32_t JUMP_THRESHOLD = 100; 

    while (running_) {
        // --- 处理 SPI1 (传感器 0-7) ---
        cs1_line_.set_value(0);
        ioctl(spi1_fd_, SPI_IOC_MESSAGE(1), &tr);
        cs1_line_.set_value(1);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 8; ++i) {
                uint16_t current_raw = rx_buffer[i] & 0x3FFF;
                
                // 计算与上一帧的差值 (需考虑 14位 0-16383 的回绕情况)
                int32_t diff = static_cast<int32_t>(current_raw) - static_cast<int32_t>(raw_values_[i]);
                if (diff > 8192) diff -= 16384;
                else if (diff < -8192) diff += 16384;

                // 核心修正：如果变化太快（绝对值 > 阈值），判定为干扰，不更新 raw_values_
                // 首次采样或 filters_initialized_ 为 false 时跳过校验
                if (!filters_initialized_ || std::abs(diff) < JUMP_THRESHOLD) {
                    raw_values_[i] = current_raw;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));

        // --- 处理 SPI2 (传感器 8-15) ---
        cs2_line_.set_value(0);
        ioctl(spi2_fd_, SPI_IOC_MESSAGE(1), &tr);
        cs2_line_.set_value(1);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 8; ++i) {
                uint16_t current_raw = rx_buffer[i] & 0x3FFF;
                int32_t diff = static_cast<int32_t>(current_raw) - static_cast<int32_t>(raw_values_[i+8]);
                if (diff > 8192) diff -= 16384;
                else if (diff < -8192) diff += 16384;

                if (!filters_initialized_ || std::abs(diff) < JUMP_THRESHOLD) {
                    raw_values_[i+8] = current_raw;
                }
            }
        }
        
        // 标记至少已经成功采集过一次数据
        // if (!filters_initialized_) filters_initialized_ = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int16_t HallSensors::getValue(uint8_t sensor_index) {
    if (sensor_index >= SENSOR_COUNT) return 0;
    std::lock_guard<std::mutex> lock(data_mutex_);
    int16_t value = static_cast<int16_t>(raw_values_[sensor_index] - zero_offsets_[sensor_index]);
    if (value >= 8192) value -= 16384;
    else if (value < -8192) value += 16384;
    return value;
}

float HallSensors::getAngle(uint8_t sensor_index) {
    return static_cast<float>(getValue(sensor_index)) * RAW_TO_DEGREE;
}

std::vector<float> HallSensors::getAllAngles() {
    std::vector<float> angles(SENSOR_COUNT);
    for (int i = 0; i < SENSOR_COUNT; ++i) angles[i] = getAngle(i);
    return getFilteredAngles(0.1f, 5);//这样就直接用的滤波后的数据
}

// --- 新增：混合滤波实现 ---
std::vector<float> HallSensors::getFilteredAngles(float alpha, size_t window_size) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 1. 初始化滤波容器
    if (!filters_initialized_) {
        median_queues_.assign(SENSOR_COUNT, std::deque<float>());
        last_lpf_values_.assign(SENSOR_COUNT, 0.0f);
        
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            float current = static_cast<float>(static_cast<int16_t>(raw_values_[i] - zero_offsets_[i])) * RAW_TO_DEGREE;
            // 处理回绕
            if (current >= 180.0f) current -= 360.0f;
            else if (current < -180.0f) current += 360.0f;

            for (size_t j = 0; j < window_size; ++j) median_queues_[i].push_back(current);
            last_lpf_values_[i] = current;
        }
        filters_initialized_ = true;
    }

    std::vector<float> results(SENSOR_COUNT);
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        // 计算当前原始角度
        int16_t val = static_cast<int16_t>(raw_values_[i] - zero_offsets_[i]);
        if (val >= 8192) val -= 16384;
        else if (val < -8192) val += 16384;
        float current_angle = static_cast<float>(val) * RAW_TO_DEGREE;

        // A. 中值滤波更新
        median_queues_[i].push_back(current_angle);
        if (median_queues_[i].size() > window_size) median_queues_[i].pop_front();

        std::vector<float> sort_buf(median_queues_[i].begin(), median_queues_[i].end());
        std::sort(sort_buf.begin(), sort_buf.end());
        float median_val = sort_buf[sort_buf.size() / 2];

        // B. 一阶低通滤波
        // y(n) = alpha * x(n) + (1-alpha) * y(n-1)
        float filtered_val = (alpha * median_val) + (1.0f - alpha) * last_lpf_values_[i];
        last_lpf_values_[i] = filtered_val;
        results[i] = filtered_val;
    }
    return results;
}

void HallSensors::setZero() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    zero_offsets_ = raw_values_;
    filters_initialized_ = false; // 重置零点后建议重新初始化滤波器
}
} // namespace ihb