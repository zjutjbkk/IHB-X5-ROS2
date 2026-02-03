#include "stepper_motor_joint.h"

#include <iostream>
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
    // === 匿名命名空间：写 sysfs 辅助 ===
    namespace
    {
        void write_pwm_sysfs(const std::string& path, const std::string& value)
        {
            int fd = open(path.c_str(), O_WRONLY);
            if (fd < 0)
            {
                // 某些情况下 export/unexport 会返回 busy
                if (errno == EBUSY) return;
                throw std::runtime_error("PWM sysfs error: Cannot open " + path + " (" + std::string(strerror(errno)) + ")");
            }

            ssize_t rc = write(fd, value.c_str(), value.length());
            if (rc < 0)
            {
                close(fd);
                if (errno == EBUSY) return;
                throw std::runtime_error("PWM sysfs error: Cannot write to " + path + " (" + std::string(strerror(errno)) + ")");
            }
            close(fd);
        }

        inline int clamp_int(int v, int lo, int hi)
        {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }
    }

    // === 构造与析构 ===
    StepperMotorJoint::StepperMotorJoint()
    {
        initHardware();
    }

    StepperMotorJoint::~StepperMotorJoint()
    {
        try
        {
            // 安全停止（会 join 线程）
            stopMotor();

            // 析构时尽量归零（仍然用 steps 作为权威）
            int64_t cur_steps = current_steps_.load();
            if (cur_steps != 0)
            {
                std::cout << "步进电机析构中，正在归零..." << std::endl;

                int64_t steps_to_zero = std::llabs(cur_steps);
                uint32_t homing_speed = 1000; // 可按需要调整
                bool dir_to_zero = (cur_steps > 0) ? false : true; // 正方向为“增高”，回零要反向

                {
                    std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                    dir_pin_line_.set_value(dir_to_zero ? 1 : 0);

                    if (homing_speed > 0 && steps_to_zero > 0)
                    {
                        // period_ns = 1e9 / speed
                        int period_ns = static_cast<int>(1000000000LL / static_cast<long long>(homing_speed));
                        period_ns = clamp_int(period_ns, 50, 1000000000); // 简单保护

                        pwm_enable(false);
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));

                        pwm_set_period(period_ns);
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        pwm_set_duty_cycle(period_ns / 2);
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        pwm_enable(true);
                    }
                }

                // 等待运动时间（这里在析构里，直接粗略等待即可）
                int64_t run_time_ms = (steps_to_zero * 1000LL) / static_cast<long long>(homing_speed);
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(run_time_ms + 50)));

                {
                    std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                    pwm_enable(false);
                }

                current_steps_.store(0);
                std::cout << "步进电机归零完成。" << std::endl;
            }
        }
        catch (const std::exception& e)
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
            dir_pin_line_ = gpio_chip_.get_line(DIR_GPIO_LINE);
            dir_pin_line_.request({ "stepper_dir", gpiod::line_request::DIRECTION_OUTPUT, 0 }, 0);

            pwm_export();

            // 默认确保 PWM 关闭
            pwm_enable(false);
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("硬件初始化失败: " + std::string(e.what()));
        }
    }

    void StepperMotorJoint::cleanupHardware()
    {
        try
        {
            pwm_enable(false);
        }
        catch (...) {}

        try
        {
            pwm_unexport();
        }
        catch (...) {}

        std::cout << "硬件资源已清理。" << std::endl;
    }

    // === steps <-> mm ===
    double StepperMotorJoint::stepsToHeight(int64_t steps) const
    {
        return static_cast<double>(steps) / STEPS_PER_MM;
    }

    int64_t StepperMotorJoint::heightToSteps(double height_mm) const
    {
        // 四舍五入到最接近的整数 steps
        return static_cast<int64_t>(std::llround(height_mm * STEPS_PER_MM));
    }

    // === 公有API实现（高度读取）===
    int16_t StepperMotorJoint::getHeight() const
    {
        // 兼容旧接口：返回整数mm（四舍五入）
        double h = getHeightPrecise();
        double r = std::round(h);
        if (r > static_cast<double>(INT16_MAX)) r = static_cast<double>(INT16_MAX);
        if (r < static_cast<double>(INT16_MIN)) r = static_cast<double>(INT16_MIN);
        return static_cast<int16_t>(r);
    }

    double StepperMotorJoint::getHeightPrecise() const
    {
        return stepsToHeight(current_steps_.load());
    }

    // === 兼容旧接口：整数 mm ===
    void StepperMotorJoint::setHeight(int16_t targetHeight_mm)
    {
        setHeight(static_cast<double>(targetHeight_mm));
    }

    void StepperMotorJoint::moveToHeight(int16_t targetHeight_mm, uint32_t speed)
    {
        moveToHeight(static_cast<double>(targetHeight_mm), speed);
    }

    void StepperMotorJoint::moveRelative(int16_t distance_mm, uint32_t speed)
    {
        moveRelative(static_cast<double>(distance_mm), speed);
    }

    // === 新接口：double mm ===
    void StepperMotorJoint::setHeight(double targetHeight_mm)
    {
        moveToHeight(targetHeight_mm, current_speed_.load());
    }

    void StepperMotorJoint::moveRelative(double distance_mm, uint32_t speed)
    {
        double cur_h = getHeightPrecise();
        moveToHeight(cur_h + distance_mm, speed);
    }

    void StepperMotorJoint::moveToHeight(double targetHeight_mm, uint32_t speed)
    {
        if (speed == 0) return;

        // 先把目标换算成 steps（整数）
        int64_t start_steps = current_steps_.load();
        int64_t target_steps = heightToSteps(targetHeight_mm);
        int64_t delta = target_steps - start_steps;

        if (delta == 0) return;

        // 停止上一段运动（join）
        stopMotor();

        bool moveDirection = (delta > 0);
        int64_t stepsToMove = std::llabs(delta);

        current_speed_.store(speed);

        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            thread_running_.store(true);
            moving_.store(true);

            // 启动线程（不 detach，保证析构安全）
            motor_thread_ = std::thread(&StepperMotorJoint::motor_thread_loop,
                                       this,
                                       stepsToMove,
                                       speed,
                                       moveDirection,
                                       target_steps);
        }
    }

    // === 线程运动逻辑 ===
    void StepperMotorJoint::motor_thread_loop(int64_t steps,
                                              uint32_t speed,
                                              bool move_direction,
                                              int64_t final_target_steps)
    {
        try
        {
            {
                std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                dir_pin_line_.set_value(move_direction ? 1 : 0);

                // 先禁用 PWM，避免 period/duty 写入冲突
                pwm_enable(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                // 设置 period / duty
                int period_ns = static_cast<int>(1000000000LL / static_cast<long long>(speed));
                period_ns = clamp_int(period_ns, 50, 1000000000);

                pwm_set_period(period_ns);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                pwm_set_duty_cycle(period_ns / 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                // 最后启用
                pwm_enable(true);
            }

            // 预计运行时间（ms）
            // 时间 = steps / speed (s) => ms = steps*1000/speed
            long long total_ms = (steps * 1000LL) / static_cast<long long>(speed);
            if (total_ms < 0) total_ms = 0;

            // 分片 sleep：允许 stopMotor 更快生效
            auto end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(total_ms);
            while (thread_running_.load() && std::chrono::steady_clock::now() < end_time)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "电机线程中发生错误: " << e.what() << std::endl;
        }

        // 关闭 PWM
        try
        {
            std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
            pwm_enable(false);
        }
        catch (...) {}

        // 如果是正常结束（没被 stop），更新最终位置
        if (thread_running_.load())
        {
            current_steps_.store(final_target_steps);
        }

        moving_.store(false);
        thread_running_.store(false);
    }

    void StepperMotorJoint::stopMotor()
    {
        // 通知线程停止
        thread_running_.store(false);

        // join 线程，防止 detach 导致析构后仍访问 this
        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            if (motor_thread_.joinable())
            {
                motor_thread_.join();
            }
        }

        // 最终确保 PWM 关闭
        try
        {
            std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
            pwm_enable(false);
        }
        catch (...) {}

        moving_.store(false);
    }

    void StepperMotorJoint::resetZero()
    {
        // 逻辑零点：直接把当前位置 steps 置 0
        current_steps_.store(0);
    }

    bool StepperMotorJoint::isMoving() const
    {
        return moving_.load();
    }

    bool StepperMotorJoint::setBrake(bool brake)
    {
        brake_enabled_.store(brake);
        return brake_enabled_.load();
    }

    void StepperMotorJoint::setSpeed(uint32_t pulses_per_second)
    {
        current_speed_.store(pulses_per_second);
    }

    // === PWM sysfs 实现 ===
    void StepperMotorJoint::pwm_export() const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/export";
        write_pwm_sysfs(path, std::to_string(PWM_CHANNEL));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void StepperMotorJoint::pwm_unexport() const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/unexport";
        write_pwm_sysfs(path, std::to_string(PWM_CHANNEL));
    }

    void StepperMotorJoint::pwm_set_period(int period_ns) const
    {
        if (period_ns <= 0)
        {
            throw std::runtime_error("无效的PWM周期值: " + std::to_string(period_ns));
        }

        // 建议：写 period 前先 disable + duty=0（你原来的做法保留）
        std::string enable_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM)
                                + "/pwm" + std::to_string(PWM_CHANNEL) + "/enable";
        write_pwm_sysfs(enable_path, "0");

        std::string duty_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM)
                              + "/pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
        write_pwm_sysfs(duty_path, "0");

        std::string period_path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM)
                                + "/pwm" + std::to_string(PWM_CHANNEL) + "/period";
        write_pwm_sysfs(period_path, std::to_string(period_ns));
    }

    void StepperMotorJoint::pwm_set_duty_cycle(int duty_cycle_ns) const
    {
        if (duty_cycle_ns < 0)
        {
            throw std::runtime_error("无效的PWM占空比值: " + std::to_string(duty_cycle_ns));
        }

        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM)
                         + "/pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
        write_pwm_sysfs(path, std::to_string(duty_cycle_ns));
    }

    void StepperMotorJoint::pwm_enable(bool enable) const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM)
                         + "/pwm" + std::to_string(PWM_CHANNEL) + "/enable";
        write_pwm_sysfs(path, enable ? "1" : "0");
    }

} // namespace ihb
