#include "stepper_motor_joint.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    try {
        std::cout << "--- 创建步进电机对象 ---" << std::endl;
        ihb::StepperMotorJoint motor;
        
        std::cout << "\n--- 测试1: 向上移动 20mm ---" << std::endl;
        motor.moveRelative(20, 4000);
        
        std::cout << "等待移动完成 (轮询 isMoving)..." << std::endl;
        while(motor.isMoving()) {
            std::cout << ".";
            std::cout.flush();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "\n移动完成。当前高度: " << motor.getHeight() << "mm" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\n--- 测试2: 移动到高度 -10mm ---" << std::endl;
        motor.moveToHeight(-10, 2000);
        
        std::cout << "等待移动完成..." << std::endl;
        while(motor.isMoving()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cout << "移动完成。当前高度: " << motor.getHeight() << "mm" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n--- 测试结束，对象即将析构，自动归零 ---" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n程序发生严重错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "程序正常退出。" << std::endl;
    return 0;
}