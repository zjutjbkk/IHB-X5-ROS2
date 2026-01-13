#include "stepper_motor_joint.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>

namespace ihb
{
    // === 构造与析构 ===
    StepperMotorJoint::StepperMotorJoint()
        : height_(0),
          current_speed_(DEFAULT_SPEED),
          brake_enabled_(false),
          moving_(false),
          thread_running_(false)
    {
        initHardware();
    }

    StepperMotorJoint::~StepperMotorJoint()
    {
        try
        {
            stopMotor();

            int16_t current_height = height_.load();
            if (current_height != 0)
            {
                std::cout << "步进电机析构中，正在归零..." << std::endl;
                int32_t steps_to_zero = std::abs(heightToSteps(current_height));
                uint16_t homing_speed = 10000;
                bool dir_to_zero = (current_height > 0) ? false : true;

                {
                    std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                    dir_pin_line_.set_value(dir_to_zero ? 0 : 1);

                    if (homing_speed > 0 && steps_to_zero > 0)
                    {
                        int period_ns = 1000000000 / homing_speed;
                        pwm_set_period(period_ns);
                        pwm_set_duty_cycle(period_ns / 2);
                        pwm_enable(true);

                        int run_time_ms = (steps_to_zero * 1000) / homing_speed;
                        std::this_thread::sleep_for(std::chrono::milliseconds(run_time_ms + 50));
                    }
                }
                std::cout << "步进电机归零完成。" << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "析构函数中发生错误: " << e.what() << std::endl;
        }
        cleanupHardware();
    }

    // === 硬件初始化与清理 ===
    void StepperMotorJoint::initHardware()
    {
        try
        {
            gpio_chip_ = gpiod::chip(DIR_GPIO_CHIP, gpiod::chip::OPEN_BY_NAME);
            if (!gpio_chip_)
            {
                throw std::runtime_error("无法打开 GPIO chip: " + std::string(DIR_GPIO_CHIP));
            }
            dir_pin_line_ = gpio_chip_.get_line(DIR_GPIO_LINE);
            dir_pin_line_.request({"stepper_dir", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);

            // 强制重置PWM状态并初始化
            pwm_force_reset_and_init();
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("硬件初始化失败: " + std::string(e.what()));
        }
    }

    void StepperMotorJoint::cleanupHardware()
    {
        pwm_enable(false);
        pwm_unexport();
        std::cout << "硬件资源已清理。" << std::endl;
    }
    void ihb::StepperMotorJoint::homeToZeroBlocking(uint16_t homing_speed)
    {
        stopMotor();

        int16_t current_height = height_.load();
        if (current_height == 0) {
            return;
        }

        std::cout << "归零中..." << std::endl;
        int32_t steps_to_zero = std::abs(heightToSteps(current_height));
        bool dir_to_zero = (current_height > 0) ? false : true;

        {
            std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
            dir_pin_line_.set_value(dir_to_zero ? 0 : 1);

            if (homing_speed > 0 && steps_to_zero > 0) {
                int period_ns = 1000000000 / homing_speed;
                pwm_set_period(period_ns);
                pwm_set_duty_cycle(period_ns / 2);
                pwm_enable(true);

                int run_time_ms = (steps_to_zero * 1000) / homing_speed;
                std::this_thread::sleep_for(std::chrono::milliseconds(run_time_ms + 50));

                pwm_enable(false);
            }
        }

        height_.store(0);          // ✅ 关键：回零完成后更新高度
        moving_.store(false);
        thread_running_.store(false);

        std::cout << "归零完成。" << std::endl;
    }    
    
    // === 核心运动逻辑 ===
    void StepperMotorJoint::moveToHeight(int16_t targetHeight, uint16_t speed)
    {
        if (speed == 0)
            return;

        int16_t current_height = height_.load();
        int32_t stepsToMove = heightToSteps(targetHeight) - heightToSteps(current_height);
        if (stepsToMove == 0)
            return;

        stopMotor();

        bool moveDirection = (stepsToMove > 0);
        current_speed_.store(speed);

        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            thread_running_.store(true);
            moving_.store(true);
            height_.store(targetHeight);

            std::thread(&StepperMotorJoint::motor_thread_loop, this, std::abs(stepsToMove), speed, moveDirection).detach();
        }
    }

    void StepperMotorJoint::motor_thread_loop(int32_t steps, uint16_t speed, bool move_direction)
    {
        try
        {
            {
                std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                dir_pin_line_.set_value(move_direction ? 0 : 1);

                // 确保PWM被禁用
                pwm_enable(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                // 先设置period，再设置duty_cycle
                int period_ns = 1000000000 / speed;
                pwm_set_period(period_ns);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                pwm_set_duty_cycle(period_ns / 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 最后启用PWM
                pwm_enable(true);
            }

            int run_time_ms = (steps * 1000) / speed;
            std::this_thread::sleep_for(std::chrono::milliseconds(run_time_ms));
        }
        catch (const std::exception &e)
        {
            std::cerr << "电机线程中发生错误: " << e.what() << std::endl;
        }

        if (thread_running_.load())
        {
            std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
            pwm_enable(false);
        }

        moving_.store(false);
        thread_running_.store(false);
    }

    void StepperMotorJoint::stopMotor()
    {
        if (thread_running_.exchange(false))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            try
            {
                std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                pwm_enable(false);
            }
            catch (...)
            { /* 忽略停止时的错误 */
            }
        }
        moving_.store(false);
    }

    // === 公有API实现 ===
    int16_t StepperMotorJoint::getHeight() const { return height_.load(); }
    void StepperMotorJoint::setHeight(int16_t targetHeight) { moveToHeight(targetHeight, current_speed_.load()); }
    void StepperMotorJoint::moveRelative(int16_t distance, uint16_t speed) { moveToHeight(height_.load() + distance, speed); }
    void StepperMotorJoint::setSpeed(uint16_t pulses_per_second) { current_speed_.store(pulses_per_second); }
    void StepperMotorJoint::resetZero() { height_.store(0); }
    bool StepperMotorJoint::isMoving() const { return moving_.load(); }
    bool StepperMotorJoint::setBrake(bool brake)
    {
        brake_enabled_.store(brake);
        return brake_enabled_.load();
    }
    int16_t StepperMotorJoint::stepsToHeight(int32_t steps) const { return static_cast<int16_t>(steps / STEPS_PER_MM); }
    int32_t StepperMotorJoint::heightToSteps(int16_t height_mm) const { return static_cast<int32_t>(height_mm * STEPS_PER_MM); }

    // === 私有PWM实现 ===
    namespace
    { // 使用匿名命名空间来定义辅助函数，限制其作用域在此文件内
        void write_pwm_sysfs(const std::string &path, const std::string &value)
        {
            int fd = open(path.c_str(), O_WRONLY);
            if (fd < 0)
            {
                if (errno == EBUSY)
                    return;
                throw std::runtime_error("PWM sysfs error: Cannot open " + path + " (" + strerror(errno) + ")");
            }
            if (write(fd, value.c_str(), value.length()) < 0)
            {
                close(fd);
                if (errno == EBUSY)
                    return;
                throw std::runtime_error("PWM sysfs error: Cannot write to " + path);
            }
            close(fd);
        }
    }

    void StepperMotorJoint::pwm_export() const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/export";
        try
        {
            write_pwm_sysfs(path, std::to_string(PWM_CHANNEL));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (const std::runtime_error &e)
        {
            if (std::string(e.what()).find("busy") == std::string::npos)
                throw;
        }
    }

    void StepperMotorJoint::pwm_unexport() const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/unexport";
        try
        {
            write_pwm_sysfs(path, std::to_string(PWM_CHANNEL));
        }
        catch (...)
        {
        }
    }

    void StepperMotorJoint::pwm_set_period(int period_ns) const
    {
        if (period_ns <= 0) {
            throw std::runtime_error("无效的PWM周期值: " + std::to_string(period_ns));
        }

        // 先禁用PWM
        std::string enable_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/enable";
        write_pwm_sysfs(enable_path, "0");
        
        // 设置duty_cycle为0
        std::string duty_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
        write_pwm_sysfs(duty_path, "0");
        
        // 设置新的period
        std::string period_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/period";
        write_pwm_sysfs(period_path, std::to_string(period_ns));
    }

    void StepperMotorJoint::pwm_set_duty_cycle(int duty_cycle_ns) const
    {
        if (duty_cycle_ns < 0) {
            throw std::runtime_error("无效的PWM占空比值: " + std::to_string(duty_cycle_ns));
        }

        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
        write_pwm_sysfs(path, std::to_string(duty_cycle_ns));
    }

    void StepperMotorJoint::pwm_enable(bool enable) const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/enable";

        // 在enable之前，检查并确保PWM状态正确
        if (enable) {
            try {
                // 检查period是否为0，如果是则重新设置
                std::string period_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/period";
                std::ifstream period_file(period_path);
                std::string period_str;
                if (period_file >> period_str) {
                    int current_period = std::stoi(period_str);
                    if (current_period == 0) {
                        std::cout << "⚠️ 检测到PWM period为0，重新设置..." << std::endl;
                        // 直接重新设置PWM参数，避免递归调用
                        pwm_set_period(1000000);  // 1ms = 1kHz
                        pwm_set_duty_cycle(0);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                }
            } catch (...) {
                std::cout << "⚠️ PWM状态检查失败，重新设置基本参数..." << std::endl;
                try {
                    pwm_set_period(1000000);  // 1ms = 1kHz
                    pwm_set_duty_cycle(0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                } catch (...) {
                    // 如果还是失败，就继续尝试enable
                }
            }
        }

        write_pwm_sysfs(path, enable ? "1" : "0");
    }

    void StepperMotorJoint::pwm_force_reset_and_init() const
    {
        std::cout << "🔧 强制重置PWM状态..." << std::endl;

        try {
            // 1. 先尝试导出PWM (如果已经导出会忽略错误)
            pwm_export();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 2. 检查当前period状态
            std::string period_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/period";
            std::ifstream period_file(period_path);
            std::string period_str;
            int current_period = 0;
            if (period_file >> period_str) {
                current_period = std::stoi(period_str);
            }

            std::cout << "当前PWM period: " << current_period << " ns" << std::endl;

            // 3. 如果period为0，直接设置period，不要先设置其他参数
            if (current_period == 0) {
                std::cout << "Period为0，直接设置新的period..." << std::endl;
                int default_period_ns = 1000000;  // 1ms = 1kHz
                write_pwm_sysfs(period_path, std::to_string(default_period_ns));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                // 4. 如果period不为0，先禁用PWM
                try {
                    std::string enable_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/enable";
                    write_pwm_sysfs(enable_path, "0");
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                } catch (...) {
                    // 忽略禁用时的错误
                }
            }

            // 5. 设置duty_cycle为0
            std::string duty_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
            write_pwm_sysfs(duty_path, "0");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // 6. 确保PWM处于禁用状态
            std::string enable_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/enable";
            write_pwm_sysfs(enable_path, "0");

            std::cout << "✅ PWM状态重置完成" << std::endl;

        } catch (const std::exception& e) {
            std::cout << "⚠️ PWM重置过程中出现错误: " << e.what() << std::endl;
            std::cout << "尝试继续初始化..." << std::endl;
        }
    }

} // namespace ihb