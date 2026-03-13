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
#include <sys/stat.h> // 用于检查文件是否存在

namespace ihb
{
    // === 匿名命名空间：写 sysfs 辅助 ===
    namespace
    {
        // 检查文件或目录是否存在
        bool file_exists(const std::string& path) {
            struct stat buffer;
            return (stat(path.c_str(), &buffer) == 0);
        }

        // 基础写函数：报错时会抛出异常，包含 errno 信息
        void write_pwm_sysfs(const std::string& path, const std::string& value)
        {
            int fd = open(path.c_str(), O_WRONLY);
            if (fd < 0)
            {
                throw std::runtime_error("PWM sysfs open error: " + path + " (" + std::string(strerror(errno)) + ")");
            }

            ssize_t rc = write(fd, value.c_str(), value.length());
            int err_save = errno; // 保存错误码
            close(fd);

            if (rc < 0)
            {
                throw std::runtime_error("PWM sysfs write error: '" + value + "' to " + path + " (" + std::string(strerror(err_save)) + ")");
            }
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
            // 停止线程
            stopMotor();

            // 析构归零逻辑
            int64_t cur_steps = current_steps_.load();
            if (cur_steps != 0)
            {
                std::cout << "步进电机析构中，正在归零..." << std::endl;
                int64_t steps_to_zero = std::llabs(cur_steps);
                uint32_t homing_speed = 1000; 
                bool dir_to_zero = (cur_steps > 0) ? false : true; 

                {
                    std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                    if (dir_pin_line_) dir_pin_line_.set_value(dir_to_zero ? 0 : 1);

                    if (homing_speed > 0 && steps_to_zero > 0)
                    {
                        int period_ns = static_cast<int>(1000000000LL / static_cast<long long>(homing_speed));
                        period_ns = clamp_int(period_ns, 50, 1000000000);

                        // 使用修改后的 pwm_set_period，它现在更健壮
                        pwm_set_period(period_ns); 
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        
                        pwm_set_duty_cycle(period_ns / 2);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        
                        pwm_enable(true);
                    }
                }

                // 简单的等待归零完成
                int64_t run_time_ms = (steps_to_zero * 1000LL) / static_cast<long long>(homing_speed);
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(run_time_ms + 100)));

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
            // 1. 初始化 GPIO
            gpio_chip_ = gpiod::chip(DIR_GPIO_CHIP, gpiod::chip::OPEN_BY_NAME);
            dir_pin_line_ = gpio_chip_.get_line(DIR_GPIO_LINE);
            if (!dir_pin_line_) {
                throw std::runtime_error("无法获取GPIO Line " + std::to_string(DIR_GPIO_LINE));
            }
            dir_pin_line_.request({ "stepper_dir", gpiod::line_request::DIRECTION_OUTPUT, 0 }, 0);

            // 2. 初始化 PWM
            pwm_export();

            // === 关键修复：解决 Invalid argument 问题 ===
            // 问题：如果 period 为 0，驱动会拒绝执行 echo 0 > enable (Invalid argument)。
            // 解决：强制先写入一个合法的默认 period。
            
            std::string prefix = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL);
            
            // 为了安全，先把 duty 设为 0
            try { write_pwm_sysfs(prefix + "/duty_cycle", "0"); } catch(...) {} 

            // 强制写入默认周期 (1ms = 1000000ns)
            // 这会让驱动认为配置合法，从而允许我们在下一步执行 disable
            try { 
                write_pwm_sysfs(prefix + "/period", "1000000"); 
            } catch (const std::exception& e) {
                // 如果这里失败，可能意味着文件被锁，或者硬件严重故障
                std::cerr << "警告: 初始化设置默认周期失败 (可能导致无法 Disable): " << e.what() << std::endl;
            }

            // 现在配置合法了，确保 PWM 处于关闭状态
            pwm_enable(false);
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("硬件初始化失败: " + std::string(e.what()));
        }
    }

    void StepperMotorJoint::cleanupHardware()
    {
        try { pwm_enable(false); } catch (...) {}
        // 通常不建议在运行中 unexport，除非明确不需要再用了。
        // 这里为了稳定性，可以选择保留 export 状态。
        // try { pwm_unexport(); } catch (...) {} 
        
        if (dir_pin_line_) dir_pin_line_.release();
        
        std::cout << "硬件资源已清理。" << std::endl;
    }

    // === steps <-> mm 转换 ===
    double StepperMotorJoint::stepsToHeight(int64_t steps) const { return static_cast<double>(steps) / STEPS_PER_MM; }
    int64_t StepperMotorJoint::heightToSteps(double height_mm) const { return static_cast<int64_t>(std::llround(height_mm * STEPS_PER_MM)); }

    // === 公有API实现 ===
    int16_t StepperMotorJoint::getHeight() const { return static_cast<int16_t>(std::round(getHeightPrecise())); }
    double StepperMotorJoint::getHeightPrecise() const { return stepsToHeight(current_steps_.load()); }

    void StepperMotorJoint::setHeight(int16_t targetHeight_mm) { setHeight(static_cast<double>(targetHeight_mm)); }
    void StepperMotorJoint::moveToHeight(int16_t targetHeight_mm, uint32_t speed) { moveToHeight(static_cast<double>(targetHeight_mm), speed); }
    void StepperMotorJoint::moveRelative(int16_t distance_mm, uint32_t speed) { moveRelative(static_cast<double>(distance_mm), speed); }

    void StepperMotorJoint::setHeight(double targetHeight_mm) { moveToHeight(targetHeight_mm, current_speed_.load()); }
    void StepperMotorJoint::moveRelative(double distance_mm, uint32_t speed) { moveToHeight(getHeightPrecise() + distance_mm, speed); }

    bool StepperMotorJoint::isMoving() const { return moving_.load(); }
    bool StepperMotorJoint::setBrake(bool brake) { brake_enabled_.store(brake); return brake_enabled_.load(); }
    void StepperMotorJoint::setSpeed(uint32_t pulses_per_second) { current_speed_.store(pulses_per_second); }
    void StepperMotorJoint::resetZero() { current_steps_.store(0); }

    void StepperMotorJoint::moveToHeight(double targetHeight_mm, uint32_t speed)
    {
        if (speed == 0) return;

        int64_t start_steps = current_steps_.load();
        int64_t target_steps = heightToSteps(targetHeight_mm);
        int64_t delta = target_steps - start_steps;

        if (delta == 0) return;

        stopMotor();

        bool moveDirection = (delta > 0);
        int64_t stepsToMove = std::llabs(delta);

        current_speed_.store(speed);

        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            thread_running_.store(true);
            moving_.store(true);
            motor_thread_ = std::thread(&StepperMotorJoint::motor_thread_loop, this, stepsToMove, speed, moveDirection, target_steps);
        }
    }

    void StepperMotorJoint::motor_thread_loop(int64_t steps, uint32_t speed, bool move_direction, int64_t final_target_steps)
    {
        try
        {
            {
                std::lock_guard<std::mutex> hw_lock(hardware_mutex_);
                if (dir_pin_line_) dir_pin_line_.set_value(move_direction ? 0 : 1);

                // 计算周期
                int period_ns = static_cast<int>(1000000000LL / static_cast<long long>(speed));
                period_ns = clamp_int(period_ns, 50, 1000000000);

                // 设置 PWM 参数
                // 注意：pwm_set_period 内部会先 disable -> duty=0 -> set period
                pwm_set_period(period_ns);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                pwm_set_duty_cycle(period_ns / 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                pwm_enable(true);
            }

            long long total_ms = (steps * 1000LL) / static_cast<long long>(speed);
            auto end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(total_ms);
            
            while (thread_running_.load() && std::chrono::steady_clock::now() < end_time)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "电机线程错误: " << e.what() << std::endl;
        }

        // 停止 PWM
        try { std::lock_guard<std::mutex> hw_lock(hardware_mutex_); pwm_enable(false); } catch (...) {}

        if (thread_running_.load()) {
            current_steps_.store(final_target_steps);
        }
        moving_.store(false);
        thread_running_.store(false);
    }

    void StepperMotorJoint::stopMotor()
    {
        thread_running_.store(false);
        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            if (motor_thread_.joinable()) motor_thread_.join();
        }
        try { std::lock_guard<std::mutex> hw_lock(hardware_mutex_); pwm_enable(false); } catch (...) {}
        moving_.store(false);
    }

    // === PWM sysfs 实现 (核心修正部分) ===
    
    void StepperMotorJoint::pwm_export() const
    {
        std::string pwm_dir = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL);
        
        // 检查目录是否存在，存在则跳过 export 步骤
        if (file_exists(pwm_dir)) {
            return;
        }

        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/export";
        try {
            write_pwm_sysfs(path, std::to_string(PWM_CHANNEL));
            // export 后等待 udev 创建节点，防止后续立即访问失败
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        } catch (const std::exception& e) {
            // 如果文件不存在但 export 失败，可能是权限问题
            throw; 
        }
    }

    void StepperMotorJoint::pwm_unexport() const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/unexport";
        try {
            write_pwm_sysfs(path, std::to_string(PWM_CHANNEL));
        } catch (...) {}
    }

    void StepperMotorJoint::pwm_set_period(int period_ns) const
    {
        if (period_ns <= 0) throw std::runtime_error("Invalid Period");

        std::string prefix = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL);

        // 1. 尝试 Disable
        // 使用 try-catch 忽略 Invalid argument 错误。
        // 如果 Period 为 0，Disable 会报错，但我们可以安全地忽略它，因为我们的目的是重新设置配置。
        try {
            write_pwm_sysfs(prefix + "/enable", "0");
        } catch (...) {}

        // 2. Duty 归零
        write_pwm_sysfs(prefix + "/duty_cycle", "0"); 
        
        // 3. 设置新 Period
        write_pwm_sysfs(prefix + "/period", std::to_string(period_ns)); 
    }

    void StepperMotorJoint::pwm_set_duty_cycle(int duty_cycle_ns) const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
        write_pwm_sysfs(path, std::to_string(duty_cycle_ns));
    }

    void StepperMotorJoint::pwm_enable(bool enable) const
    {
        std::string path = "/sys/class/pwm/pwmchip" + std::to_string(PWM_CHIP_NUM) + "/pwm" + std::to_string(PWM_CHANNEL) + "/enable";
        write_pwm_sysfs(path, enable ? "1" : "0");
    }

} // namespace ihb