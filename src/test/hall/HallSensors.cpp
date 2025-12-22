// #include "HallSensors.hpp"
#include "HallSensors.h"
// c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
// c++
#include <iostream>
#include <thread>
#include <mutex>
#include <iomanip>
#include <atomic>

// SPI RDKX5设备路径和参数定义
#define SPI_DEVICE "/dev/spidev1.0"
#define SPI2_DEVICE "/dev/spidev2.0"

#define SPI_SPEED_HZ 100000
#define SPI_BITS_PER_WORD 16
#define SPI_MODE SPI_MODE_1

// GPIO  RDKX5相关路径和引脚定义
#define SYSFS_GPIO_EXPORT "/sys/class/gpio/export"
#define SYSFS_GPIO_UNEXPORT "/sys/class/gpio/unexport"
#define SYSFS_GPIO_PIN_DIR "/sys/class/gpio/gpio%d/direction"
#define SYSFS_GPIO_PIN_VAL "/sys/class/gpio/gpio%d/value"
#define SPI_CS0 394
#define SPI_CS1 396

// 初始化指定GPIO引脚，并设置方向（"in"或"out"）
void gpio_init(int pin, const char *direction)
{
    int fd;
    char pin_str[10];
    char path[100];

    // Export the GPIO pin
    fd = open(SYSFS_GPIO_EXPORT, O_WRONLY);
    if (fd == -1)
    {
        perror("Failed to open export");
        exit(EXIT_FAILURE);
    }
    snprintf(pin_str, sizeof(pin_str), "%d", pin);
    write(fd, pin_str, strlen(pin_str));
    close(fd);

    // Set the direction of the GPIO pin
    snprintf(path, sizeof(path), SYSFS_GPIO_PIN_DIR, pin);
    fd = open(path, O_WRONLY);
    if (fd == -1)
    {
        perror("Failed to open direction");
        exit(EXIT_FAILURE);
    }
    write(fd, direction, strlen(direction));
    close(fd);
}

// 设置GPIO引脚的电平（高或低）
void gpio_set(int pin, int value)
{
    int fd;
    char path[100];
    char value_str[2];

    snprintf(path, sizeof(path), SYSFS_GPIO_PIN_VAL, pin);
    fd = open(path, O_WRONLY);
    if (fd == -1)
    {
        perror("Failed to open value");
        exit(EXIT_FAILURE);
    }
    snprintf(value_str, sizeof(value_str), "%d", value);
    write(fd, value_str, strlen(value_str));
    close(fd);
}

// 释放GPIO引脚
void gpio_uninit(int pin)
{
    int fd;
    char pin_str[10];

    fd = open(SYSFS_GPIO_UNEXPORT, O_WRONLY);
    if (fd == -1)
    {
        perror("Failed to open unexport");
        exit(EXIT_FAILURE);
    }
    snprintf(pin_str, sizeof(pin_str), "%d", pin);
    write(fd, pin_str, strlen(pin_str));
    close(fd);
}

// 初始化SPI1和SPI2设备，设置模式、位宽和速率
void spi_init()
{
    int fd;
    uint8_t spi_mode = SPI_MODE;
    uint8_t spi_bits_per_word = SPI_BITS_PER_WORD;
    uint32_t spi_speed_hz = SPI_SPEED_HZ;

    // 打开 SPI 设备
    fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0)
    {
        perror("无法打开 SPI1 设备");
        exit(EXIT_FAILURE);
    }

    // 配置 SPI 设备
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
    {
        perror("无法设置 SPI1 模式");
        exit(EXIT_FAILURE);
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0)
    {
        perror("无法设置位宽1");
        exit(EXIT_FAILURE);
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz) < 0)
    {
        perror("无法设置时钟速率1");
        exit(EXIT_FAILURE);
    }

    close(fd);

    // 打开 SPI2 设备
    fd = open(SPI2_DEVICE, O_RDWR);
    if (fd < 0)
    {
        perror("无法打开 SPI1 设备");
        exit(EXIT_FAILURE);
    }

    // 配置 SPI 设备
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
    {
        perror("无法设置 SPI1 模式");
        exit(EXIT_FAILURE);
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0)
    {
        perror("无法设置位宽1");
        exit(EXIT_FAILURE);
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz) < 0)
    {
        perror("无法设置时钟速率1");
        exit(EXIT_FAILURE);
    }

    close(fd);
}

// SPI数据收发函数，index=1表示SPI1，index=2表示SPI2
void spi_transfer(uint8_t index, uint16_t *tx_buffer, uint16_t *rx_buffer, uint32_t len)
{

    int fd;
    // struct spi_ioc_transfer tr = {
    //     .tx_buf = (unsigned long)tx_buffer,
    //     .rx_buf = (unsigned long)rx_buffer,
    //     .len = len,
    //     .speed_hz = SPI_SPEED_HZ,
    //     .bits_per_word = SPI_BITS_PER_WORD,
    //     .cs_change = 0,
    // };

    struct spi_ioc_transfer tr{}; // 把所有字段先都置为 0    初始化结构体所有字段为0

    tr.tx_buf = reinterpret_cast<unsigned long>(tx_buffer);
    tr.rx_buf = reinterpret_cast<unsigned long>(rx_buffer);
    tr.len = len;
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = SPI_BITS_PER_WORD;
    tr.cs_change = 0;

    // 打开 SPI 设备 根据index选择SPI设备和片选引脚
    if (index == 1)
    {
        gpio_set(394, 0);
        fd = open(SPI_DEVICE, O_RDWR);
    }
    else
    {
        gpio_set(400, 0);
        fd = open(SPI2_DEVICE, O_RDWR);
    }

    if (fd < 0)
    {
        perror("无法打开 SPI 设备");
        exit(EXIT_FAILURE);
    }

    // 发送和接收数据
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0)
    {
        perror("SPI 传输失败");
        exit(EXIT_FAILURE);
    }
    if (index == 1)
    {
        gpio_set(394, 1);
    }
    else
    {
        gpio_set(400, 1);
    }
    close(fd);
}

namespace Hall
{

    std::atomic<bool> readThread(true); // 控制读取线程的原子变量

    // 构造函数，初始化SPI和GPIO
    HallSensors::HallSensors()
    {
        // _id = id;
        spi_init();
        gpio_init(394, "out");
        gpio_set(394, 1);
        gpio_init(400, "out");
        gpio_set(400, 1);
    }

    // 启动读取线程，show>0时循环打印角度数据
    void HallSensors::start(unsigned int show)
    {
        readThread = true;
        _readThread = std::thread(&HallSensors::read, this);
        _readThread.detach(); // 分离线程
        // t1.detach();
        if (show)
        { // 显示数据
            while (1)
            {
                for (int i = 0; i < 16; i++)
                {
                    // 计算零点校正后的值
                    _values_zero[i] = _values[i] - _zero[i];
                    if (_values_zero[i] >= 8192)
                    {
                        _values_zero[i] = _values_zero[i] - 16383;
                    }
                    else if (_values_zero[i] < -8192)
                    {
                        _values_zero[i] = 16383 + _values_zero[i];
                    }
                    // printf("%6d ",_values[i]);
                    // 打印角度值（单位：度）
                    std::cout << std::fixed << std::setprecision(1) << std::setw(6) << (_values_zero[i] / 8192.0 * 180) << " ";
                }
                std::cout << std::endl;
                usleep(show * 1000); // 延时show毫秒
            }
        }
    }

    // 停止读取线程
    void HallSensors::stop()
    {
        // 关闭spi线程
        readThread = false;
    }

    // 读取线程函数，循环读取SPI数据并处理
    void HallSensors::read()
    {

        uint16_t tx_buffer[8] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}; // 发送的数据
        uint16_t rx_buffer[8] = {0};
        uint16_t rx_buffer1[8] = {0}; // 接收的数据
        bool zero = false;            // 是否已校零 初始为false

        while (readThread)
        {
            // 读取前8个传感器数据（SPI1）
            spi_transfer(1, tx_buffer, rx_buffer, sizeof(tx_buffer));
            spi_transfer(1, tx_buffer, rx_buffer, sizeof(tx_buffer));
            // gpio_set(394, 1);
            // 数据处理
            // 获取互斥量
            // _mutex.lock();

            // 处理前8个数据
            for (int i = 0; i < 8; i++)
            {
                // 顺序可能需要修改
                _values[i] = rx_buffer[i] & 0x3FFF; // 取低14位有效数据
            }
            // sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(3)); // 稍作延时
            // _mutex.unlock();
            // gpio_set(396, 0);

            // 读取后8个传感器数据（SPI2）
            spi_transfer(2, tx_buffer, rx_buffer1, sizeof(tx_buffer));
            // gpio_set(396, 1);
            // gpio_set(396, 0);
            spi_transfer(2, tx_buffer, rx_buffer1, sizeof(tx_buffer));
            // gpio_set(396, 1);
            // // 数据处理
            // // _mutex.lock();
            // 处理后8个数据
            for (int i = 8; i < 16; i++)
            {
                // 顺序可能需要修改
                _values[i] = rx_buffer1[i - 8] & 0x3FFF; // 取低14位有效数据
                // _values[i] = 0;
            }

            // zero 第一次读取时，保存零点数据
            if (!zero)
            {
                for (int i = 0; i < 16; i++)
                {
                    _zero[i] = _values[i];
                }
                zero = true;
            }
            // 计算零点校正后的值
            for (int i = 0; i < 16; i++)
            {
                _values_zero[i] = _values[i] - _zero[i];
                if (_values_zero[i] >= 8192)
                {
                    _values_zero[i] = _values_zero[i] - 16383;
                }
                else if (_values_zero[i] < -8192)
                {
                    _values_zero[i] = 16383 + _values_zero[i];
                }
                // printf("%6d ",_values[i]);

                // std::cout << std::fixed << std::setprecision(1) << std::setw(6) << (_values_zero[i]/8192.0*180) << " ";
            }

            // _mutex.unlock();
        }
        //
        // gpio_set(396, 0);
    }

    // 析构函数，释放GPIO引脚
    HallSensors::~HallSensors()
    {
    }

    // 手动设置零点（未实现，预留接口）
    void HallSensors::setZero()
    {
        // {
        //     if (_readThread)
        //     {
        // _readThread->join();
        // this->start(0);

        // this->stop();
        // while (readThread)
        // {
        // };
        // _mutex.lock();
        // for (int i = 0; i < 16; i++)
        // {
        //     _zero[i] = _values[i];
        // }
        // _mutex.unlock();
        // }
        // 存储当前值
    }
    // void HallSensors::processCanMessage(uint8_t *data){
    //     // TODO
    // }
} // namespace Hall

int main()
{
    Hall::HallSensors hallSensors;

    // 启动传感器读取线程并设置数据展示间隔（例如 100 毫秒）
    hallSensors.start(300);

    // 模拟运行一段时间后停止
    sleep(10);
    hallSensors.stop();

    return 0;
}











