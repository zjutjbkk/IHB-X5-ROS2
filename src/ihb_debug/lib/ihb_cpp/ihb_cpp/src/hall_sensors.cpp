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
#include <iomanip> // 用于格式化输出

namespace ihb 
{

HallSensors::HallSensors()
    : raw_values_(SENSOR_COUNT, 0), zero_offsets_(SENSOR_COUNT, 0) 
{
    // --- 1. 初始化GPIO ---
    try {
        // 打开 gpiochip4
        gpio_chip_ = gpiod::chip(GPIO_CHIP_NAME, gpiod::chip::OPEN_BY_NAME);
        
        // 获取 line 15 (pin 394)
        cs1_line_ = gpio_chip_.get_line(CS1_PIN_OFFSET);
        cs1_line_.request({"hall_sensors_cs1", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);

        // 获取 line 21 (pin 400)
        cs2_line_ = gpio_chip_.get_line(CS2_PIN_OFFSET);
        cs2_line_.request({"hall_sensors_cs2", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);

    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("GPIO 初始化失败: ") + e.what());
    }
    
    // --- 2. 初始化SPI ---
    auto init_spi_device = [&](const char* device_path) -> int {
        int fd = open(device_path, O_RDWR);
        if (fd < 0) {
            throw std::runtime_error(std::string("无法打开 SPI 设备: ") + device_path);
        }
        
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
        // 清理已打开的资源
        if (spi1_fd_ >= 0) close(spi1_fd_);
        if (spi2_fd_ >= 0) close(spi2_fd_);
        // libgpiod line/chip 会在析构时自动释放
        throw; // 重新抛出异常
    }
    
    std::cout << "HallSensors 初始化成功！" << std::endl;
}

HallSensors::~HallSensors() {
    stop(); // 确保线程停止

    // 释放SPI文件描述符
    if (spi1_fd_ >= 0) {
        close(spi1_fd_);
    }
    if (spi2_fd_ >= 0) {
        close(spi2_fd_);
    }

    // libgpiod 会在 cs1_line_ 和 cs2_line_ 析构时自动释放GPIO
    std::cout << "HallSensors 资源已释放。" << std::endl;
}

void HallSensors::start(uint32_t print_interval_ms) {
    if (running_.exchange(true)) {
        // 如果已经运行，只更新打印间隔
        print_interval_ms_.store(print_interval_ms);
        std::cout << "霍尔传感器已在运行，打印间隔更新为 " << print_interval_ms << " ms。" << std::endl;
        return;
    }

    // 启动读取线程
    read_thread_ = std::thread(&HallSensors::readLoop, this);
    
    // 等待第一次数据稳定后设置零点
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    setZero();
    
    std::cout << "霍尔传感器读取线程已启动。" << std::endl;

    // 如果需要自动打印，则启动打印线程
    print_interval_ms_.store(print_interval_ms);
    if (print_interval_ms > 0) {
        print_thread_ = std::thread(&HallSensors::printLoop, this);
        std::cout << "自动打印功能已启动，间隔: " << print_interval_ms << " ms。" << std::endl;
    }
}


    // --- 新增: printLoop 实现 ---
    void HallSensors::printLoop() {
        while (running_) {
            uint32_t interval = print_interval_ms_.load();
            if (interval == 0) {
                // 如果间隔被设为0，线程可以休眠更长时间或直接退出
                // 这里选择长休眠，以便之后可以重新设置间隔来唤醒它
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            // 获取所有角度
            std::vector<float> angles = getAllAngles();
            
            // 格式化打印
            std::cout << "\r"; // 回到行首实现动态刷新
            for (int i = 0; i < SENSOR_COUNT; ++i) {
                std::cout << "S" << std::setw(2) << i << ":" 
                          << std::fixed << std::setprecision(2) << std::setw(8) << angles[i] << " | ";
                if (i == 7) {
                    std::cout << "\n"; // 8个一行
                }
            }
            std::cout << std::flush;
            
            // 等待指定的时间间隔
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
        std::cout << std::endl; // 停止时换行，避免命令行提示符错位
    }

void HallSensors::stop() {
    if (running_.exchange(false)) {
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (print_thread_.joinable()) {
            print_thread_.join();
        }
        std::cout << "所有后台线程已停止。" << std::endl;
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
        .cs_change = 0,
    };

    while (running_) {
        // --- 读取 SPI1 (传感器 0-7) ---
        cs1_line_.set_value(0); // 片选拉低
        if (ioctl(spi1_fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
            perror("SPI1 传输失败");
        }
        cs1_line_.set_value(1); // 片选拉高

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 8; ++i) {
                raw_values_[i] = rx_buffer[i] & 0x3FFF;
            }
        }
        
        // 短暂延时，给设备处理时间
        std::this_thread::sleep_for(std::chrono::microseconds(50));

        // --- 读取 SPI2 (传感器 8-15) ---
        cs2_line_.set_value(0); // 片选拉低
        if (ioctl(spi2_fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
            perror("SPI2 传输失败");
        }
        cs2_line_.set_value(1); // 片选拉高

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 8; ++i) {
                raw_values_[i + 8] = rx_buffer[i] & 0x3FFF;
            }
        }
        
        // 控制读取频率
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int16_t HallSensors::getValue(uint8_t sensor_index) {
    if (sensor_index >= SENSOR_COUNT) {
        return 0;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    int16_t value = static_cast<int16_t>(raw_values_[sensor_index] - zero_offsets_[sensor_index]);
    
    // 处理14位数据回绕
    if (value >= 8192) {
        value -= 16384;
    } else if (value < -8192) {
        value += 16384;
    }
    return value;
}

float HallSensors::getAngle(uint8_t sensor_index) {
    return static_cast<float>(getValue(sensor_index)) * RAW_TO_DEGREE;
}

std::vector<float> HallSensors::getAllAngles() {
    std::vector<float> angles(SENSOR_COUNT);
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        angles[i] = getAngle(i);
    }
    return angles;
}

void HallSensors::setZero() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    zero_offsets_ = raw_values_;
}
}