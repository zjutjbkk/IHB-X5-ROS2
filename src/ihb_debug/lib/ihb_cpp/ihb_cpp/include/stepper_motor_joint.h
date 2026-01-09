#pragma once

#include <cstdint>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <gpiod.hpp> // 使用 libgpiod C++ 库

namespace ihb
{
    class StepperMotorJoint
    {
    public:
        StepperMotorJoint();
        ~StepperMotorJoint();
        
        // 禁止拷贝和赋值
        StepperMotorJoint(const StepperMotorJoint&) = delete;
        StepperMotorJoint& operator=(const StepperMotorJoint&) = delete;

        // --- 公有API ---
        int16_t getHeight() const;
        void setHeight(int16_t targetHeight);
        void moveToHeight(int16_t targetHeight, uint16_t speed);
        void moveRelative(int16_t distance, uint16_t speed);
        
        void setSpeed(uint16_t pulses_per_second);
        void stopMotor();
        void resetZero();
        void homeToZeroBlocking(uint16_t speed = 10000);
        bool isMoving() const;
        bool setBrake(bool brake);

    private:
        // --- 内部辅助方法 ---
        void initHardware();
        void cleanupHardware();
        int16_t stepsToHeight(int32_t steps) const;
        int32_t heightToSteps(int16_t height_mm) const;
        
        // PWM控制函数
        void pwm_export() const;
        void pwm_unexport() const;
        void pwm_set_period(int period_ns) const;
        void pwm_set_duty_cycle(int duty_cycle_ns) const;
        void pwm_enable(bool enable) const;
        void pwm_force_reset_and_init() const;

        // 电机运动线程函数
        void motor_thread_loop(int32_t steps, uint16_t speed, bool move_direction);

        // --- 配置参数 ---
        static constexpr uint16_t DEFAULT_SPEED = 1000;
        static constexpr const char* DIR_GPIO_CHIP = "gpiochip4";
        static constexpr int DIR_GPIO_LINE = 2; // Pin 381
        static constexpr int PWM_CHIP_NUM = 0;
        static constexpr int PWM_CHANNEL = 0;
        static constexpr float STEPS_PER_MM = 2000.0f;

        // --- 状态与资源管理 ---
        std::atomic<int16_t> height_;
        std::atomic<uint16_t> current_speed_;
        std::atomic<bool> brake_enabled_;
        
        std::atomic<bool> moving_;
        std::atomic<bool> thread_running_;
        std::mutex motor_mutex_; 
        std::mutex hardware_mutex_; // 保护硬件资源的互斥锁

        gpiod::chip gpio_chip_;
        gpiod::line dir_pin_line_;
    };
} // namespace ihb