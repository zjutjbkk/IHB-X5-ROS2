#pragma once

#include <cstdint>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <gpiod.hpp> // libgpiod C++ API

namespace ihb
{
    class StepperMotorJoint
    {
    public:
        StepperMotorJoint();
        ~StepperMotorJoint();

        StepperMotorJoint(const StepperMotorJoint&) = delete;
        StepperMotorJoint& operator=(const StepperMotorJoint&) = delete;

        // --- 公有API（兼容旧接口）---
        // 仍返回“整数mm”（四舍五入），兼容你现在 printf("%d")
        int16_t getHeight() const;

        // 新增：返回“精确mm（double）”
        double getHeightPrecise() const;

        // 兼容旧接口：整数mm
        void setHeight(int16_t targetHeight_mm);
        void moveToHeight(int16_t targetHeight_mm, uint32_t speed);
        void moveRelative(int16_t distance_mm, uint32_t speed);

        // --- 新接口：支持小数mm（double）---
        void setHeight(double targetHeight_mm);
        void moveToHeight(double targetHeight_mm, uint32_t speed);
        void moveRelative(double distance_mm, uint32_t speed);

        void setSpeed(uint32_t pulses_per_second);
        void stopMotor();
        void resetZero();

        bool isMoving() const;
        bool setBrake(bool brake);

    private:
        // --- 内部辅助方法 ---
        void initHardware();
        void cleanupHardware();

        // steps <-> mm
        double stepsToHeight(int64_t steps) const;
        int64_t heightToSteps(double height_mm) const;

        // PWM控制函数
        void pwm_export() const;
        void pwm_unexport() const;
        void pwm_set_period(int period_ns) const;
        void pwm_set_duty_cycle(int duty_cycle_ns) const;
        void pwm_enable(bool enable) const;

        // 电机运动线程函数（steps为脉冲数）
        void motor_thread_loop(int64_t steps,
                               uint32_t speed,
                               bool move_direction,
                               int64_t final_target_steps);

        // --- 配置参数 ---
        static constexpr uint32_t DEFAULT_SPEED = 1000;
        static constexpr const char* DIR_GPIO_CHIP = "gpiochip4";
        static constexpr int DIR_GPIO_LINE = 2; // Pin 381
        static constexpr int PWM_CHIP_NUM = 0;
        static constexpr int PWM_CHANNEL = 0;

        // 关键：每毫米多少步（允许非整数mm靠 steps 表达）
        static constexpr double STEPS_PER_MM = 1000.0;

        // --- 状态与资源管理 ---
        // 以 steps 作为内部“权威位置”，避免 double 原子问题与累计误差
        std::atomic<int64_t> current_steps_{0};

        std::atomic<uint32_t> current_speed_{DEFAULT_SPEED};
        std::atomic<bool> brake_enabled_{false};

        std::atomic<bool> moving_{false};
        std::atomic<bool> thread_running_{false};

        std::mutex motor_mutex_;
        std::mutex hardware_mutex_;

        std::thread motor_thread_;

        gpiod::chip gpio_chip_;
        gpiod::line dir_pin_line_;
    };
} // namespace ihb
