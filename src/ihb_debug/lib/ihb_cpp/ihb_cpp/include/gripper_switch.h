#pragma once

#include <string>
#include <gpiod.hpp>
#include <chrono>

namespace ihb
{
    class GripperSwitch
    {
    public:
        /**
         * @brief 构造函数
         * @param chip_name 例如 "gpiochip4"
         * @param line_offset 例如 0 (对应GPIO27) 或 1 (对应GPIO17)
         * @param debounce_ms 软件消抖时间，默认 20ms
         */
        GripperSwitch(const std::string& chip_name, int line_offset, int debounce_ms = 20);
        ~GripperSwitch();

        // 禁止拷贝
        GripperSwitch(const GripperSwitch &) = delete;
        GripperSwitch &operator=(const GripperSwitch &) = delete;

        /**
         * @brief 获取夹爪开关状态
         * @return true: 闭合/按下 (高电平), false: 断开/松开 (低电平)
         */
        bool isTriggered();

    private:
        gpiod::chip gpio_chip_;
        gpiod::line switch_line_;
        
        // 防抖变量
        int debounce_ms_;
        bool last_stable_state_ = false;
        std::chrono::steady_clock::time_point last_change_time_;
    };
}