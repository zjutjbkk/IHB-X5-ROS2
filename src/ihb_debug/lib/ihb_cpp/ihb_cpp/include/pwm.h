#pragma once

#include <cstdint>

namespace ihb
{
    namespace pwm
    {
        // PWM相关函数
        void exportChannel(int chip, int channel);
        void unexportChannel(int chip, int channel);
        void setPeriod(int chip, int channel, int period_ns);
        void setDutyCycle(int chip, int channel, int duty_cycle_ns);
        void enable(int chip, int channel, int enable);
    }
} // namespace ihb 