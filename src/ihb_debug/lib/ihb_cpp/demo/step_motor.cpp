#include <iostream>
#include <string>
#include <csignal>
#include <cstdlib> // 用于system()函数
#include <fstream> // 用于读取文件检查CAN状态

#include <sstream>
#include <queue>
#include <thread>
#include <cmath> // 包含round函数的头文件

#include "muscle_joint.h"
#include "CanDriver.hpp"
#include "servo_motorJoint.h" // 其实头文件不该大写的，但是我懒得改了
#include "force_sensors.h"
#include "hall_sensors.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <vector>
#include <chrono>
#include <iomanip>
#include "MiniPID.h"


#include "stepper_motor_joint.h"
#include <cstdio>
#include <unistd.h>
#include <csignal>
#include <cstdlib>

// 全局电机对象，用于信号处理
static ihb::StepperMotorJoint* g_motor = nullptr;

// 信号处理函数
void signalHandler(int signum) {
    printf("\n捕获到信号 %d，程序即将退出...\n", signum);
    
    // 如果电机对象存在，执行清理（会自动归零）
    if (g_motor != nullptr) {
        printf("正在归零步进电机...\n");
        delete g_motor;
        g_motor = nullptr;
    }
    
    exit(signum);
}

int main() {
    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    printf("=== 步进电机高度控制示例程序 ===\n");
    printf("使用线程安全的新接口\n");
    printf("按 Ctrl+C 可随时安全退出并自动归零\n");
    printf("================================\n");

    try {
        // 创建步进电机对象 (使用线程安全的新接口)
        g_motor = new ihb::StepperMotorJoint();

        // 打印初始状态
        printf("✅ 步进电机初始化成功\n");
        printf("📍 初始高度: %d mm\n", g_motor->getHeight());
        printf("🔄 电机状态: %s\n", g_motor->isMoving() ? "运动中" : "静止");

        // 等待用户输入继续
        printf("\n按Enter开始测试...");
        getchar();
    } catch (const std::exception& e) {
        printf("❌ 步进电机初始化失败: %s\n", e.what());
        return -1;
    }
    
    // 重置当前位置为零点
    printf("🔄 重置零点位置\n");
    g_motor->resetZero();

    // 移动到-10mm高度，速度为1000脉冲/秒
    printf("📍 移动到高度 -10 mm (速度: 1000 脉冲/秒)...\n");
    g_motor->moveToHeight(-100, 1000);

    // 等待运动完成
    printf("⏳ 等待运动完成...\n");
    while (g_motor->isMoving()) {
        printf("   当前高度: %d mm, 状态: 运动中\n", g_motor->getHeight());
        sleep(1);
    }

    // 获取当前高度
    printf("✅ 运动完成! 当前高度: %d mm\n", g_motor->getHeight());
    
    // 等待用户输入继续
    printf("按Enter继续...");
    getchar();
    
    // 相对移动，上升5mm
    printf("相对移动 -5 mm...\n");
    g_motor->moveRelative(-50, 2000);
    
    // 等待一段时间
    printf("等待3秒...\n");
    sleep(3);
    
    // 获取当前高度
    printf("当前高度: %d mm\n", g_motor->getHeight());
    
    // 等待用户输入继续
    printf("按Enter继续...");
    getchar();
    
    // 相对移动，下降10mm
    printf("相对移动 10 mm...\n");
    g_motor->moveRelative(10, 1500);
    
    // 等待一段时间
    printf("等待3秒...\n");
    sleep(3);
    
    // 获取当前高度
    printf("当前高度: %d mm\n", g_motor->getHeight());
    
    // 等待用户输入继续
    printf("按Enter继续...");
    getchar();
    
    // 程序结束，清理资源（析构函数会自动归零）
    printf("程序结束，释放资源，电机将自动归零...\n");
    delete g_motor;
    g_motor = nullptr;
    
    return 0;
} 