#include <iostream>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <termios.h>

#include "stepper_motor_joint.h"

// 全局电机对象，用于信号处理
static ihb::StepperMotorJoint* g_motor = nullptr;

// ======== 终端按键读取（无需回车，非阻塞轮询）========
static struct termios g_old_term;

void enableRawMode() {
    tcgetattr(STDIN_FILENO, &g_old_term);
    struct termios raw = g_old_term;

    // 关闭规范模式（ICANON）+ 关闭回显（ECHO）
    raw.c_lflag &= ~(ICANON | ECHO);

    // 非阻塞读取：没有按键时 read() 在短超时后返回 0
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 1;   // 0.1s（单位是 0.1 秒）

    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_term);
}

char readKey() {
    char c = 0;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) return 0;
    return c;
}

// ======== 信号处理函数 ========
void signalHandler(int signum) {
    printf("\n捕获到信号 %d，程序即将退出...\n", signum);

    // 恢复终端
    disableRawMode();

    // 如果电机对象存在，执行清理（析构会自动归零）
    if (g_motor != nullptr) {
        printf("正在释放电机资源（将自动归零）...\n");
        delete g_motor;
        g_motor = nullptr;
    }

    exit(signum);
}

int main() {
    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    printf("=== 步进电机高度控制（W/S/R/Q）===\n");
    printf("w: 上移 5mm\n");
    printf("s: 下移 5mm\n");
    printf("r: 重置当前位置为零点（若运动中则等待停止后执行）\n");
    printf("q: 退出（自动归零）\n");
    printf("===================================\n");

    try {
        // 创建步进电机对象
        g_motor = new ihb::StepperMotorJoint();

        printf("✅ 步进电机初始化成功\n");
        printf("📍 初始高度: %d mm\n", g_motor->getHeight());
        printf("🔄 重置零点位置\n");
        g_motor->resetZero();
        printf("📍 当前高度: %d mm\n", g_motor->getHeight());

    } catch (const std::exception& e) {
        printf("❌ 步进电机初始化失败: %s\n", e.what());
        return -1;
    }

    // 启用“无回车读按键（非阻塞）”
    enableRawMode();

    const int step_mm = 5;
    const int speed_pulse = 10000;

    bool last_moving = g_motor->isMoving();
    bool pending_reset_zero = false;  // r按下后，若运动中则挂起等待停止再执行

    printf("\n进入按键控制：w上移5mm, s下移5mm, r重置零点, q退出\n");

    while (true) {
        // 监控运动状态变化：开始/结束只提示一次
        bool moving = g_motor->isMoving();

        // 从“运动中 -> 静止”：打印一次“完成”
        if (last_moving && !moving) {
            printf("✅ 运动完成！当前高度: %d mm\n", g_motor->getHeight());

            // 若有挂起的零点重置请求，则在停止后执行
            if (pending_reset_zero) {
                printf("🔄 执行挂起操作：重置当前位置为零点...\n");
                g_motor->resetZero();
                printf("✅ 已重置零点，当前高度: %d mm\n", g_motor->getHeight());
                pending_reset_zero = false;
            }
        }

        last_moving = moving;

        // 读取按键（非阻塞）
        char key = readKey();
        if (key == 0) {
            // 没按键就继续循环，让程序可以及时检测“运动结束”
            continue;
        }

        if (key == 'q' || key == 'Q') {
            printf("收到退出指令 q，准备结束...\n");
            break;
        }

        if (key == 'w' || key == 'W') {
            if (g_motor->isMoving()) {
                printf("⚠️ 电机运动中，忽略本次 w 指令\n");
                continue;
            }
            printf("⬆️ 开始上移 %d mm...\n", step_mm);

            // 注意：按你原先习惯，上移用负号；若实际方向反了，把 +/- 对调即可
            g_motor->moveRelative(-step_mm, speed_pulse);

        } else if (key == 's' || key == 'S') {
            if (g_motor->isMoving()) {
                printf("⚠️ 电机运动中，忽略本次 s 指令\n");
                continue;
            }
            printf("⬇️ 开始下移 %d mm...\n", step_mm);
            g_motor->moveRelative(+step_mm, speed_pulse);

        } else if (key == 'r' || key == 'R') {
            if (g_motor->isMoving()) {
                pending_reset_zero = true;
                printf("🕒 已收到 r：电机停止后将自动重置零点\n");
                continue;
            }
            printf("🔄 重置当前位置为零点...\n");
            g_motor->resetZero();
            printf("✅ 已重置零点，当前高度: %d mm\n", g_motor->getHeight());

        } else {
            printf("(忽略按键: %c)\n", key);
        }
    }

    // 恢复终端
    disableRawMode();

    // 程序结束，清理资源（析构函数会自动归零）
    printf("程序结束，释放资源，电机将自动归零...\n");
    delete g_motor;
    g_motor = nullptr;

    return 0;
}
