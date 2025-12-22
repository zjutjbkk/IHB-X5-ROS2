#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <iomanip>
#include <csignal>
#include "hall_sensors.h"

// 全局变量用于信号处理
static ihb::HallSensors* g_hallSensors = nullptr;
static bool g_running = true;

// 信号处理函数
void signalHandler(int signum) {
    std::cout << "\n捕获到信号 " << signum << "，程序即将退出..." << std::endl;
    g_running = false;
    if (g_hallSensors != nullptr) {
        g_hallSensors->stop();
    }
    exit(signum);
}

int main(int argc, char **argv)
{
    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== 霍尔传感器测试程序 ===" << std::endl;
    std::cout << "使用线程安全的新接口" << std::endl;
    std::cout << "按 Ctrl+C 退出程序" << std::endl;
    std::cout << "=========================" << std::endl;

    // 创建霍尔传感器对象
    ihb::HallSensors hallSensors;
    g_hallSensors = &hallSensors;

    // 选择测试模式
    std::cout << "选择测试模式:" << std::endl;
    std::cout << "1. 自动打印模式 (每500ms打印一次)" << std::endl;
    std::cout << "2. 手动读取模式 (每秒读取一次)" << std::endl;
    std::cout << "请输入选择 (1 或 2): ";

    int choice;
    std::cin >> choice;

    if (choice == 1) {
        // 自动打印模式 - 启动内置打印功能
        std::cout << "启动自动打印模式..." << std::endl;
        hallSensors.start(500); // 每500ms打印一次

        // 等待用户中断
        std::cout << "自动打印已启动，按 Ctrl+C 退出..." << std::endl;
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        // 手动读取模式 - 不启动自动打印
        std::cout << "启动手动读取模式..." << std::endl;
        hallSensors.start(0); // 不自动打印

        // 等待传感器稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "传感器已启动，开始读取数据..." << std::endl;
        std::cout << "格式: S00:角度值 | S01:角度值 | ..." << std::endl;

        int loop_count = 0;
        while (g_running) {
            // 使用线程安全的接口读取所有传感器数据
            std::vector<float> angles = hallSensors.getAllAngles();

            // 格式化输出
            std::cout << "Loop " << std::setw(3) << loop_count++ << ": ";
            for (int i = 0; i < 16; i++) {
                std::cout << "S" << std::setw(2) << std::setfill('0') << i << ":"
                          << std::fixed << std::setprecision(1) << std::setw(6) << angles[i] << " | ";
                if (i == 7) {
                    std::cout << std::endl << "         ";
                }
            }
            std::cout << std::endl;

            // 每秒更新一次
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    // 清理资源
    std::cout << "正在停止霍尔传感器..." << std::endl;
    hallSensors.stop();
    g_hallSensors = nullptr;

    std::cout << "霍尔传感器测试完成！" << std::endl;
    return 0;
}
    
