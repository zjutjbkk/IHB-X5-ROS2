#include "hall_sensors.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <csignal>
#include <memory>

// 全局指针，用于信号处理
std::unique_ptr<ihb::HallSensors> hall_sensors_ptr;

// 信号处理函数，用于优雅地退出
void signalHandler(int signum) {
    std::cout << "\n捕获到中断信号 (" << signum << ")，正在停止传感器..." << std::endl;
    if (hall_sensors_ptr) {
        // 调用stop来停止线程并等待其结束
        // HallSensors的析构函数会自动处理资源释放
    }
    // 让main函数自然结束，unique_ptr会自动调用析构函数
    exit(signum);
}

int main() {
    // 注册信号处理
    signal(SIGINT, signalHandler);  // 处理 Ctrl+C
    signal(SIGTERM, signalHandler); // 处理 kill 命令

    try {
        hall_sensors_ptr = std::make_unique<ihb::HallSensors>();
        hall_sensors_ptr->start(1000);
    } catch (const std::runtime_error& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }


    while(true){
        // 主线程可以执行其他任务，或者保持空闲等待信号
        std::this_thread::sleep_for(std::chrono::seconds(10000));
    }
    // while (true) {
    //     // 使用 getAllAngles 获取数据快照
    //     std::vector<float> angles = hall_sensors_ptr->getAllAngles();

    //     // 打印左臂传感器 (0-7)
    //     std::cout << "左臂 (0-7): ";
    //     for (int i = 0; i < 8; ++i) {
    //         std::cout << std::fixed << std::setprecision(1) << std::setw(6) << angles[i] << " ";
    //     }
    //     std::cout << std::endl;

    //     // 打印右臂传感器 (8-15)
    //     std::cout << "右臂 (8-15):";
    //     for (int i = 8; i < 16; ++i) {
    //         std::cout << std::fixed << std::setprecision(1) << std::setw(6) << angles[i] << " ";
    //     }
    //     std::cout << std::endl << std::endl;

    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    return 0; // 理论上不会执行到这里
}