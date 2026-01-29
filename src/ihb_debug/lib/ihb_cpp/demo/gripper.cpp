#include "gripper_switch.h"
#include <iostream>
#include <thread>
#include <vector>

// --- RDK X5 引脚配置 ---
// 基于你的图表和HallSensors代码推算：gpiochip4 基址为 379
static constexpr const char *GRIPPER_GPIO_CHIP = "gpiochip4";

// 物理引脚 13 (GPIO27) -> X5 pin 379 -> Offset 0
static constexpr int PIN_GPIO27_OFFSET = 0; 

// 物理引脚 11 (GPIO17) -> X5 pin 380 -> Offset 1
static constexpr int PIN_GPIO17_OFFSET = 1;

int main() {
    std::cout << "=== RDK X5 Gripper Switch Test ===" << std::endl;

    try {
        // 假设你接的是物理引脚 11 (GPIO17)
        // 如果接的是引脚 13，把下面换成 PIN_GPIO27_OFFSET
        int target_offset = PIN_GPIO27_OFFSET; 

        std::cout << "Initializing GPIO on " << GRIPPER_GPIO_CHIP 
                  << " Line " << target_offset << "..." << std::endl;

        ihb::GripperSwitch gripper(GRIPPER_GPIO_CHIP, target_offset);

        bool last_state = false;

        while (true) {
            bool triggered = gripper.isTriggered();

            if (triggered != last_state) {
                if (triggered) {
                    std::cout << "\033[32m[Gripper] 闭合/接触 (High) \033[0m" << std::endl;
                    // TODO: 触发抓取逻辑
                } else {
                    std::cout << "\033[31m[Gripper] 断开/松开 (Low) \033[0m" << std::endl;
                    // TODO: 触发释放逻辑
                }
                last_state = triggered;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    } catch (const std::exception &e) {
        std::cerr << "Fatal Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}