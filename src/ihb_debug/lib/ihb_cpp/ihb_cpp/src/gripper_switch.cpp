#include "gripper_switch.h"
#include <iostream>
#include <stdexcept>

namespace ihb {

GripperSwitch::GripperSwitch(const std::string& chip_name, int line_offset, int debounce_ms)
    : debounce_ms_(debounce_ms)
{
    try {
        gpio_chip_ = gpiod::chip(chip_name, gpiod::chip::OPEN_BY_NAME);
        switch_line_ = gpio_chip_.get_line(line_offset);

        gpiod::line_request config;
        config.consumer = "gripper_check";
        config.request_type = gpiod::line_request::DIRECTION_INPUT;
        config.flags = gpiod::line_request::FLAG_BIAS_PULL_UP; 

        switch_line_.request(config);
        
        // 初始化状态
        last_raw_state_ = (switch_line_.get_value() == 0); 
        last_stable_state_ = last_raw_state_;
        last_change_time_ = std::chrono::steady_clock::now();

    } catch (const std::exception& e) {
        throw std::runtime_error("GripperSwitch Init Failed: " + std::string(e.what()));
    }
}

GripperSwitch::~GripperSwitch() {
    if (switch_line_) {
        switch_line_.release();
    }
}

bool GripperSwitch::isTriggered() {
    int val = switch_line_.get_value();
    
    // 0 = 接地 = 按下(true); 1 = 上拉 = 松开(false)
    bool current_raw_pressed = (val == 0); 
    auto now = std::chrono::steady_clock::now();

    // 核心更改点：只要原始电平发生任何跳动（不管是有效操作还是噪声干扰）
    // 就立刻重置计时器。
    if (current_raw_pressed != last_raw_state_) {
        last_change_time_ = now;
        last_raw_state_ = current_raw_pressed;
    }

    // 只有当当前的电平状态持续保持了 debounce_ms_ 以上，才认为是稳定的
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_change_time_).count();
    if (duration > debounce_ms_) {
        if (current_raw_pressed != last_stable_state_) {
            last_stable_state_ = current_raw_pressed;
        }
    }

    return last_stable_state_;
}

} // namespace ihb