#include "hall_sensors.h"
#include <iostream>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <chrono>
#include <iomanip>

namespace ihb {

HallSensors::HallSensors()
    : raw_values_(SENSOR_COUNT, 0), zero_offsets_(SENSOR_COUNT, 0) {
    
    // 1. 初始化滤波缓冲区
    for(auto& buf : history_buffers_) buf.fill(0);

    // 2. 初始化GPIO (libgpiod)
    try {
        gpio_chip_ = gpiod::chip(GPIO_CHIP_NAME, gpiod::chip::OPEN_BY_NAME);
        cs1_line_ = gpio_chip_.get_line(CS1_PIN_OFFSET);
        cs1_line_.request({"hall_cs1", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
        cs2_line_ = gpio_chip_.get_line(CS2_PIN_OFFSET);
        cs2_line_.request({"hall_cs2", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
    } catch (const std::exception& e) {
        throw std::runtime_error("GPIO Init Failed: " + std::string(e.what()));
    }

    // 3. 初始化SPI
    auto init_spi = [&](const char* path) {
        int fd = open(path, O_RDWR);
        if (fd < 0) throw std::runtime_error("SPI Open Failed");
        uint8_t mode = SPI_MODE;
        uint8_t bits = SPI_BITS_PER_WORD;
        uint32_t speed = SPI_SPEED_HZ;
        ioctl(fd, SPI_IOC_WR_MODE, &mode);
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        return fd;
    };
    spi1_fd_ = init_spi(SPI1_DEVICE);
    spi2_fd_ = init_spi(SPI2_DEVICE);
}

HallSensors::~HallSensors() {
    stop();
    if (spi1_fd_ >= 0) close(spi1_fd_);
    if (spi2_fd_ >= 0) close(spi2_fd_);
}

void HallSensors::start(uint32_t print_interval_ms) {
    if (running_.exchange(true)) return;
    read_thread_ = std::thread(&HallSensors::readLoop, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待数据稳定
    setZero();
    if (print_interval_ms > 0) {
        print_interval_ms_.store(print_interval_ms);
        print_thread_ = std::thread(&HallSensors::printLoop, this);
    }
}

void HallSensors::stop() {
    running_.store(false);
    if (read_thread_.joinable()) read_thread_.join();
    if (print_thread_.joinable()) print_thread_.join();
}

void HallSensors::readLoop() {
    uint16_t tx[8] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    uint16_t rx[8];
    struct spi_ioc_transfer tr = { (unsigned long)tx, (unsigned long)rx, sizeof(tx), SPI_SPEED_HZ, SPI_BITS_PER_WORD, 0 };

    while (running_) {
        // --- 处理 SPI1 (0-7) ---
        cs1_line_.set_value(0);
        ioctl(spi1_fd_, SPI_IOC_MESSAGE(1), &tr);
        cs1_line_.set_value(1);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 8; ++i) {
                uint16_t val = rx[i] & 0x3FFF;
                // 更新环形缓冲区
                history_buffers_[i][history_ptr_[i]] = val;
                history_ptr_[i] = (history_ptr_[i] + 1) % MEDIAN_WINDOW;
                // 排序取中值
                std::array<uint16_t, MEDIAN_WINDOW> temp = history_buffers_[i];
                std::sort(temp.begin(), temp.end());
                raw_values_[i] = temp[MEDIAN_WINDOW / 2];
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));

        // --- 处理 SPI2 (8-15) ---
        cs2_line_.set_value(0);
        ioctl(spi2_fd_, SPI_IOC_MESSAGE(1), &tr);
        cs2_line_.set_value(1);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 8; ++i) {
                uint16_t val = rx[i] & 0x3FFF;
                int idx = i + 8;
                history_buffers_[idx][history_ptr_[idx]] = val;
                history_ptr_[idx] = (history_ptr_[idx] + 1) % MEDIAN_WINDOW;
                std::array<uint16_t, MEDIAN_WINDOW> temp = history_buffers_[idx];
                std::sort(temp.begin(), temp.end());
                raw_values_[idx] = temp[MEDIAN_WINDOW / 2];
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

float HallSensors::getAngle(uint8_t idx) {
    if (idx >= SENSOR_COUNT) return 0.0f;
    
    uint16_t current_raw;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_raw = raw_values_[idx];
    }

    // 1. 计算当前的中值角度（减去零点，处理回绕）
    int16_t val = static_cast<int16_t>(current_raw - zero_offsets_[idx]);
    if (val >= 8192) val -= 16384;
    else if (val < -8192) val += 16384;
    float current_median_angle = static_cast<float>(val) * RAW_TO_DEGREE;

    // 2. 一阶低通滤波 (EMA)
    // 只有在数据更新时才进行平滑，防止零漂
    float smoothed = LPF_ALPHA * current_median_angle + (1.0f - LPF_ALPHA) * last_smooth_angles_[idx];
    
    // 3. 更新并返回
    last_smooth_angles_[idx] = smoothed;
    return smoothed;
}

std::vector<float> HallSensors::getAllAngles() {
    std::vector<float> angles(SENSOR_COUNT);
    for (int i = 0; i < SENSOR_COUNT; ++i) angles[i] = getAngle(i);
    return angles;
}

void HallSensors::setZero() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    zero_offsets_ = raw_values_;
}

void HallSensors::printLoop() {
    while (running_) {
        auto angles = getAllAngles();
        std::cout << "\r" << std::fixed << std::setprecision(2);
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            std::cout << "S" << i << ":" << std::setw(7) << angles[i] << (i == 7 ? " |\n" : " | ");
        }
        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(print_interval_ms_.load()));
    }
}

} // namespace ihb