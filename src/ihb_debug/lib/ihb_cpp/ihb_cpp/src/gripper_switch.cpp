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

        // --- 变更点 1: 配置改为 上拉 (PULL_UP) ---
        // 既然板子默认是高，我们显式开启上拉，确保稳定
        gpiod::line_request config;
        config.consumer = "gripper_check";
        config.request_type = gpiod::line_request::DIRECTION_INPUT;
        config.flags = gpiod::line_request::FLAG_BIAS_PULL_UP; 

        switch_line_.request(config);
        
        // 这里的初始状态逻辑不用变，会自动读取当前的
        last_stable_state_ = (switch_line_.get_value() == 0); // 注意：这里我预存的逻辑可能需要根据需求调整，见下方 isTriggered
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
    
    // --- 变更点 2: 逻辑反转 ---
    // 接地接法：
    // 按下 = 接地 = 0
    // 松开 = 上拉 = 1
    bool current_is_pressed = (val == 0); 

    auto now = std::chrono::steady_clock::now();

    // 下面的防抖逻辑完全不用动，只需要改上面的判定即可
    if (current_is_pressed != last_stable_state_) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_change_time_).count();
        if (duration > debounce_ms_) {
            last_stable_state_ = current_is_pressed;
            last_change_time_ = now;
        }
    } else {
        last_change_time_ = now;
    }

    return last_stable_state_;
}

} // namespace ihb