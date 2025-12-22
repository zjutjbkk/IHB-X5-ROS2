#include "force_sensors.h"
// #include "ads1115_rpi.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <cmath>

// extern "C"{
//     int gpio_control(const char *pin, const char *direction, const char *value);
//     int selectCHannel(int channel);
// }

int gpio_control(const char *pin, const char *direction, const char *value);
int selectCHannel(int channel);
int openI2CBus(char *bus);
int setI2CSlave(unsigned char deviceAddr);
uint16_t readVoltage(int channel);
void configDevice(unsigned int config);
int gpio_init(const char *pin);

char i2c_device[] = "/dev/i2c-0";

namespace ihb
{

    ForceSensors::ForceSensors(int8_t index)
    {
        _index = index;
        // _id = id;
        // 硬件初始化
        if (openI2CBus("/dev/i2c-0") == -1)
        {
            std::cout << "Failed to open i2c bus" << std::endl;
        }
        setI2CSlave(0x49);
        gpio_init("388");
        gpio_init("380");
        gpio_init("381");

        gpio_init("379");
        gpio_init("387");
        gpio_init("382");
        // 读取线程
        // _readThread = new std::thread(&ForceSensors::readAll, this);
        // 脱离主线程
        // _readThread->join();
        if (index == -1)
        {
            std::thread bgThread(&ForceSensors::readAll, this);
            bgThread.detach();
        }
        else
        {
            std::thread bgThread(&ForceSensors::readOne, this);
            bgThread.detach();
        }
    }
    float ForceSensors::getValue(uint8_t index)
    {
        // 优化：添加线程安全保护
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _values[index];
    }
    ForceSensors::~ForceSensors()
    {
    }
    void ForceSensors::readAll()
    {
        while (1)
        {
            // 优化：使用局部缓冲区减少锁的持有时间
            float temp_values[32];

            // 单个引脚频率只有13左右   传感器太多了
            // 调整gpio引脚，循环读取
            for (int i = 0; i < 32; i++)
            {
                selectCHannel(i);
                // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                // 5v 电阻
                temp_values[i] = (32767.0 / readVoltage(0) - 1) * 48.6;
                // 保留两位小数
                temp_values[i] = std::round(temp_values[i] * 100) / 100;
                if (temp_values[i] < 0.1)
                {
                    temp_values[i] = 0;
                }
            }

            // 优化：批量更新数据，减少锁的持有时间
            {
                std::lock_guard<std::mutex> lock(_data_mutex);
                for (int i = 0; i < 16; i++) {  // 只更新前16个有效值
                    _values[i] = temp_values[i];
                }
            }

            // 添加适当的延迟，避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    void ForceSensors::readOne()
    {
        while (1)
        {
            int i = _index;
            selectCHannel(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            // 5v 电阻
            float temp_value = (32767.0 / readVoltage(0) - 1) * 3.47;
            // 保留两位小数
            temp_value = std::round(temp_value * 100) / 100;
            if (temp_value < 0.1)
            {
                temp_value = 0;
            }

            // 优化：添加线程安全保护
            {
                std::lock_guard<std::mutex> lock(_data_mutex);
                _values[i] = temp_value;
            }
        }
    }
}
// voicd ForceSensors::processCanMessage(uint8_t *data){
//     // TODO
// }
// }

int selectCHannel(int channel)
{
    if (channel < 0 || channel > 32)
    {
        return -1;
    }

    int row = 0;
    int col = 0;
    // 多路复用选择
    // 行 channel/8

    //      adc 引脚选择
    //      s2  s1  s0
    // GPIO  22  17  16
    // 管脚号 388 380 381

    // 列 channel%8

    //   GND 引脚选择
    //      s2  s1  s0
    // GPIO  26  25  24
    // 管脚号 401 387 402

    if (channel < 16)
    {
        row = channel / 4;  //1
        col = channel % 4;  //0
    }
    else
    {
        row = (channel - 16) / 4 + 4;
        col = (channel - 16) % 4 + 4;
    }

    // 设置gpio引脚
    gpio_control("388", "out", std::to_string((row >> 2) & 0x01).c_str()); // 22
    gpio_control("380", "out", std::to_string((row >> 1) & 0x01).c_str()); // 17
    gpio_control("381", "out", std::to_string(row & 0x01).c_str());        // 16

    gpio_control("379", "out", std::to_string((col >> 2) & 0x01).c_str()); // 27
    gpio_control("387", "out", std::to_string((col >> 1) & 0x01).c_str()); // 25
    gpio_control("382", "out", std::to_string(col & 0x01).c_str());        // 23
    // std::cout << "row: " << row << " col: " << col << std::endl;
    // std::cout << "channel: " << channel << std::endl;
    // std::cout << "gpio: " << (row >> 2) << (row >> 1) << (row & 0x01) << std::endl;
    // std::cout << "gpio: " << (col >> 2) << (col >> 1) << (col & 0x01) << std::endl;
    return 0;
}
int gpio_init(const char *pin)
{
    char path[64];
    int fd;
    const char *direction = "out";
    // 导出 GPIO 引脚
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1)
    {
        printf("ERR: GPIO export open error for pin %s.\n", pin);
        return -1;
    }
    write(fd, pin, strlen(pin));
    close(fd);

    // 设置 GPIO 方向
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%s/direction", pin);
    fd = open(path, O_WRONLY);
    if (fd == -1)
    {
        printf("ERR: GPIO direction open error for pin %s.\n", pin);
        return -1;
    }
    write(fd, direction, strlen(direction));
    close(fd);
    return 0;
}

// 通用GPIO操作函数
int gpio_control(const char *pin, const char *direction, const char *value)
{
    char path[64];
    int fd;

    // 设置 GPIO 值
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%s/value", pin);
    fd = open(path, O_WRONLY);
    if (fd == -1)
    {
        printf("ERR: GPIO value open error for pin %s.\n", pin);
        return -1;
    }
    write(fd, value, strlen(value));
    close(fd);

    return 0;
}

// char *bus = "/dev/i2c-1";
int i2cFile;
// int deviceAddr = 0x48;
unsigned char writeBuf[3] = {0};

int openI2CBus(char *bus)
{
    if ((i2cFile = open(bus, O_RDWR)) < 0)
    {
        printf("Failed to open the bus. \n");
        return -1;
    }
    else
    {
        printf("Bus open \n");
        return 1;
    }
}

int setI2CSlave(unsigned char deviceAddr)
{
    if (ioctl(i2cFile, I2C_SLAVE, deviceAddr) < 0)
    {
        printf("Failed to set I2C_SLAVE at address: 0x%x. \n", deviceAddr);
        return -1;
    }
    else
    {
        printf("I2C_SLAVE set at address: 0x%x \n", deviceAddr);
        return 1;
    }
}

void configDevice(unsigned int config)
{
    writeBuf[0] = 0x01;
    writeBuf[1] = config >> 8;
    writeBuf[2] = config & 0xFF;
    write(i2cFile, writeBuf, 3);
}

uint16_t readVoltage(int channel)
{
    unsigned char readBuf[2] = {0};
    unsigned int analogVal;
    float voltage;
    unsigned int config = 0;

    config = CONFIG_REG_OS_SINGLE |
             CONFIG_REG_PGA_4_096V |
             CONFIG_REG_MODE_SINGLE |
             CONFIG_REG_DR_860SPS |
             CONFIG_REG_CMODE_TRAD |
             CONFIG_REG_CPOL_ACTIV_LOW |
             CONFIG_REG_CLATCH_NONLATCH |
             CONFIG_REG_CQUE_NONE;

    switch (channel)
    {
    case 0:
        config |= CONFIG_REG_MUX_CHAN_0;
        break;
    case 1:
        config |= CONFIG_REG_MUX_CHAN_1;
        break;
    case 2:
        config |= CONFIG_REG_MUX_CHAN_2;
        break;
    case 3:
        config |= CONFIG_REG_MUX_CHAN_3;
        break;
    default:
        printf("Give a channel between 0-3\n");
    }
    configDevice(config);

    while (readBuf[0] >> 7 != 1)
    {
        read(i2cFile, readBuf, 2);
    }

    writeBuf[0] = 0x00;
    write(i2cFile, writeBuf, 1);

    if (read(i2cFile, readBuf, 2) != 2) // read data and check error
    {
        printf("Error : Input/Output Error \n");
    }
    else
    {
        analogVal = readBuf[0] << 8 | readBuf[1];
        // voltage = (float)analogVal * 4.096 / 32767.0;
    }

    return analogVal;
}
