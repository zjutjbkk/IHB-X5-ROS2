/**
 * @file pid.cpp
 * @brief PID调试和数据记录测试程序
 * @details 用于记录肌肉关节PID控制过程中的角度数据，便于后续分析和调试
 *
 * 线程架构（优化版 v2）：
 * - CAN 接收线程：处理 CAN 总线消息
 * - PID 控制线程：10ms 周期（读取传感器、处理角度、执行 PID 控制）
 * - 日志记录线程：100ms 周期（记录数据到 CSV）
 *
 * 线程管理优化：
 * - 使用 join() 方式管理线程，替代 detach()，提高安全性
 * - 程序退出时确保所有线程正确清理和退出
 * - 信号处理函数只设置停止标志，由主函数统一处理退出流程
 *
 * 控制优化：
 * - 将角度读取和 PID 控制合并到同一线程，减少线程间同步延迟
 * - 角度读取和 PID 更新在同一周期内完成，时序更加一致
 * - 简化了线程架构，降低了系统复杂度
 */

#include <iostream>
#include <string>
#include <csignal>
#include <sstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <atomic>
#include <mutex>

#include "muscle_joint.h"
#include "CanDriver.hpp"
#include "servo_motorJoint.h"
#include "hall_sensors.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include "servo_motorJoint.h"
// 全局对象
ihb::HallSensors hallSensors;

// 注意：由于MuscleJoint和ServoMotorJoint构造函数会初始化CAN驱动，
// 在没有硬件的情况下会抛出异常。这是预期行为。
// 实际使用时需要确保CAN总线已正确配置。

// 伺服电机对象（新增）
ihb::ServoMotorJoint ser1(1);
ihb::ServoMotorJoint ser2(2);
ihb::ServoMotorJoint ser3(3);
ihb::ServoMotorJoint ser4(4);

// 肌肉关节对象（带 PID 参数初始化）
// 参数：id, kp, ki, kd
 ihb::MuscleJoint muscle1(0,30,1.8, 120);
// ihb::MuscleJoint muscle1(0,31,1.85, 100);//这个备份一下
// ihb::MuscleJoint muscle2(1, 60, 8, 150);//这个备份一下 为啥这个关节的pid差了这么多
ihb::MuscleJoint muscle2(1, 60, 5, 200);
ihb::MuscleJoint muscle3(3, 300, 10, 150);
// ihb::MuscleJoint muscle4(4, 400, 30, 100);//这个凑合能看吧
ihb::MuscleJoint muscle4(4, 250, 20, 200);
// 
// 全局控制标志
std::atomic<bool> running(true);

// PID目标角度（所有关节设为0度）
const float TARGET_ANGLES[4][2] = {
    {5.0f, 5.0f},   // muscle1
    {5.0f, 5.0f},   // muscle2
    {5.0f, 0.0f},   // muscle3
    {8.0f, 0.0f}    // muscle4
};

// 函数声明
void receiveCanFrames();
void logPidData();
void pidControlLoop();  // 已合并角度读取功能
void signalHandler(int signum);


// 检查CAN总线状态
bool checkCanStatus(const std::string& interface = "can1") {
    // 检查接口是否存在
    std::string path = "/sys/class/net/" + interface + "/operstate";
    std::ifstream statusFile(path);
    if (!statusFile.is_open()) {
        std::cout << "CAN接口 " << interface << " 不存在！" << std::endl;
        return false;
    }
    
    // 读取接口状态
    std::string status;
    std::getline(statusFile, status);
    statusFile.close();
    
    // 检查状态是否为"up"
    if (status == "up") {
        std::cout << "CAN接口 " << interface << " 已经启动！" << std::endl;
        return true;
    } else {
        std::cout << "CAN接口 " << interface << " 状态: " << status << std::endl;
        return false;
    }
}

// 初始化CAN总线
bool initCanBus() {
    // 首先检查CAN总线状态
    if (checkCanStatus("can1")) {
        std::cout << "CAN总线已经启动，无需重新初始化" << std::endl;
        return true;
    }
    
    int ret = 0;
    
    std::cout << "开始初始化CAN总线..." << std::endl;
    
    // 设置CAN总线比特率
    ret = system("sudo ip link set can1 type can bitrate 1000000");
    if (ret != 0) {
        std::cerr << "设置CAN总线比特率失败！" << std::endl;
        return false;
    }
    
    // 休眠1秒
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 启动CAN总线
    ret = system("sudo ip link set can1 up");
    if (ret != 0) {
        std::cerr << "启动CAN总线失败！" << std::endl;
        return false;
    }
    
    // 再休眠5秒确保CAN总线完全启动
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    std::cout << "CAN总线初始化成功！" << std::endl;
    return true;
}
/**
 * @brief 主函数
 */
int main(int argc, char **argv)
{
    // 注册信号处理器（Ctrl+C优雅退出）
    signal(SIGINT, signalHandler);
        // 首先初始化CAN总线
        if (!initCanBus()) {
            std::cerr << "❌ CAN总线初始化失败，程序将继续但可能无法通信！" << std::endl;
        }
    // 启动霍尔传感器
    hallSensors.start();
    std::cout << "✓ 霍尔传感器已启动" << std::endl;

    // 等待传感器稳定
    std::cout << "\n等待传感器稳定..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "========================================" << std::endl;
    std::cout << "  PID调试数据记录程序 v4.0（优化版）" << std::endl;
    std::cout << "  线程架构：" << std::endl;
    std::cout << "    - CAN 接收线程" << std::endl;
    std::cout << "    - PID 控制线程（含角度读取）：10ms" << std::endl;
    std::cout << "    - 日志记录线程：100ms" << std::endl;
    std::cout << "  优化：角度读取和 PID 控制合并" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "肌肉关节ID: " << muscle1.id() << ", " << muscle2.id()
              << ", " << muscle3.id() << ", " << muscle4.id() << std::endl;

    // 初始化肌肉关节气压为0
    muscle1.setPressure(0, 0, 0, 0);
    muscle2.setPressure(0, 0, 0, 0);
    muscle3.setPressure(0, 0, 0, 0);
    muscle4.setPressure(0, 0, 0, 0);
    std::cout << "✓ 肌肉关节已初始化" << std::endl;

    // 初始化伺服电机位置为0
    ser1.start();
    ser2.start();
    ser1.setPositonWithSpeedLimit(5*100, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ser2.setPositonWithSpeedLimit(5*100, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "✓ 伺服电机已初始化" << std::endl;

    // 启动CAN接收线程（使用 join 方式管理，更安全）
    std::thread receiveThread(receiveCanFrames);

    // 启动日志记录线程（独立）
    std::thread logThread(logPidData);

    // 启动 PID 控制线程（已合并角度读取功能）
    std::thread pidThread(pidControlLoop);

    std::cout << "\n开始PID控制..." << std::endl;
    std::cout << "目标角度设置（所有关节）:" << std::endl;
    std::cout << "  Muscle1: (" << TARGET_ANGLES[0][0] << "°, " << TARGET_ANGLES[0][1] << "°)" << std::endl;
    std::cout << "  Muscle2: (" << TARGET_ANGLES[1][0] << "°, " << TARGET_ANGLES[1][1] << "°)" << std::endl;
    std::cout << "  Muscle3: (" << TARGET_ANGLES[2][0] << "°, " << TARGET_ANGLES[2][1] << "°)" << std::endl;
    std::cout << "  Muscle4: (" << TARGET_ANGLES[3][0] << "°, " << TARGET_ANGLES[3][1] << "°)" << std::endl;
    std::cout << "\n按 Ctrl+C 停止程序..." << std::endl;

    // 设置初始目标角度
    // muscle1.setTargetAngle(TARGET_ANGLES[0][0], TARGET_ANGLES[0][1]);
    // muscle2.setTargetAngle(TARGET_ANGLES[1][0], TARGET_ANGLES[1][1]);
    // muscle3.setTargetAngle(TARGET_ANGLES[2][0], TARGET_ANGLES[2][1]);
    // muscle4.setTargetAngle(TARGET_ANGLES[3][0], TARGET_ANGLES[3][1]);
    
    // 动态修改目标角度示例（每 6 秒切换一次）
    std::cout << "\n开始动态目标角度测试..." << std::endl;
// // //轨迹1号
//     std::cout << "→ 切换目标角度: Muscle4 (10°, 0°)" << std::endl;
//     muscle1.setTargetAngle(6, -4);

//     std::this_thread::sleep_for(std::chrono::seconds(6));
//     std::cout << "→ 切换目标角度: Muscle4 (0°, -10°)" << std::endl;
//     muscle1.setTargetAngle(-4, 6);

//     std::this_thread::sleep_for(std::chrono::seconds(6));
//     std::cout << "→ 切换目标角度: Muscle4 (-10°, 0°)" << std::endl;
//     muscle1.setTargetAngle(6,-4);

    // std::this_thread::sleep_for(std::chrono::seconds(6));
    // std::cout << "→ 切换目标角度: Muscle4 (0°, 10°)" << std::endl;
    // muscle4.setTargetAngle(-6, -8);

    // std::this_thread::sleep_for(std::chrono::seconds(6));


// // 轨迹2号
//         muscle1.setTargetAngle(6, -4);
//         std::this_thread::sleep_for(std::chrono::seconds(6));
//         muscle1.setTargetAngle(0, 0);
//         std::this_thread::sleep_for(std::chrono::seconds(6));
//         muscle1.setTargetAngle(-4, 6);
//         std::this_thread::sleep_for(std::chrono::seconds(6));
//     std::cout << "→ 切换目标角度: Muscle4 (0°, 0°)" << std::endl;
//     muscle1.setTargetAngle(0, 0);



// // //多关节轨迹1号
//         muscle1.setTargetAngle(4, -5);
//         muscle2.setTargetAngle(6, -7);
//         muscle3.setTargetAngle(8, -9);
//         muscle4.setTargetAngle(10, -11);
//         std::this_thread::sleep_for(std::chrono::seconds(10));
//         muscle1.setTargetAngle(-4, 5);
//         muscle2.setTargetAngle(-6, 7);
//         muscle3.setTargetAngle(-8, 9);
//         muscle4.setTargetAngle(-10, 11);
//         std::this_thread::sleep_for(std::chrono::seconds(10));
//            muscle1.setTargetAngle(4, -5);
//         muscle2.setTargetAngle(6, -7);
//         muscle3.setTargetAngle(8, -9);
//         muscle4.setTargetAngle(10, -11);
    
//         std::this_thread::sleep_for(std::chrono::seconds(10));
 

// //多关节轨迹2号
//           muscle1.setTargetAngle(-4, 5);
//         muscle2.setTargetAngle(-6, 7);
//         muscle3.setTargetAngle(-8, 9);
//         muscle4.setTargetAngle(-10, 11);
//         std::this_thread::sleep_for(std::chrono::seconds(10));
//         muscle1.setTargetAngle(0, 0);
//         muscle2.setTargetAngle(0, 0);
//         muscle3.setTargetAngle(0, 0);
//         muscle4.setTargetAngle(0, 0);
//         std::this_thread::sleep_for(std::chrono::seconds(10));
//            muscle1.setTargetAngle(-4, 5);
//         muscle2.setTargetAngle(-6, 7);
//         muscle3.setTargetAngle(-8, 9);
//         muscle4.setTargetAngle(-10, 11);
//         std::this_thread::sleep_for(std::chrono::seconds(10));
//           muscle1.setTargetAngle(0, 0);
//         muscle2.setTargetAngle(0, 0);
//         muscle3.setTargetAngle(0, 0);
//         muscle4.setTargetAngle(0, 0);
 

    // 主线程等待（30秒后自动退出，或按Ctrl+C提前退出）
    int countdown = 30;
    while (running && countdown > 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        countdown--;
    }

    // ========== 程序退出流程 ==========
    std::cout << "\n程序结束，正在清理..." << std::endl;

    // 1. 停止所有线程的循环
    running = false;
    std::cout << "✓ 已发送停止信号到所有线程" << std::endl;

    // 2. 归零所有气压输出（安全措施）
    muscle1.setPressure(0, 0, 0, 0);
    muscle2.setPressure(0, 0, 0, 0);
    muscle3.setPressure(0, 0, 0, 0);
    muscle4.setPressure(0, 0, 0, 0);
    ser1.setPositonWithSpeedLimit(0, 120);
    ser2.setPositonWithSpeedLimit(0, 120);
    std::cout << "✓ 所有气压输出已归零" << std::endl;

    // 3. 等待所有线程正常退出（使用 join）
    std::cout << "正在等待线程退出..." << std::endl;

    if (pidThread.joinable()) {
        pidThread.join();
        std::cout << "  ✓ PID 控制线程已退出" << std::endl;
    }

    if (logThread.joinable()) {
        logThread.join();
        std::cout << "  ✓ 日志记录线程已退出" << std::endl;
    }

    if (receiveThread.joinable()) {
        receiveThread.join();
        std::cout << "  ✓ CAN 接收线程已退出" << std::endl;
    }

    std::cout << "\n✓ 所有线程已安全退出" << std::endl;
    std::cout << "✓ 程序已退出" << std::endl;
    return 0;
}

/**
 * @brief 信号处理函数（Ctrl+C）
 * @details 只设置停止标志，让主函数的退出流程来处理线程 join 和资源清理
 *          这是更安全的做法，避免在信号处理函数中执行复杂操作
 */
void signalHandler(int signum)
{
    std::cout << "\n\n收到中断信号 (Ctrl+C)，正在停止..." << std::endl;

    // 设置停止标志，让所有线程退出循环
    running = false;

    // 注意：不在信号处理函数中执行复杂操作（如 join、资源清理等）
    // 这些操作将在主函数的退出流程中完成
    std::cout << "提示：等待线程安全退出..." << std::endl;
}

/**
 * @brief CAN帧接收线程（新增伺服电机消息处理）
 */
void receiveCanFrames()
{
    try {
        sockcanpp::CanDriver canDriver("can1", CAN_RAW);

        // 关闭回环
        int loopback = 0;
        setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK,
                   &loopback, sizeof(loopback));

        uint8_t data[8] = {0};

        std::cout << "✓ CAN接口已连接" << std::endl;

        while (running)
        {
            if (canDriver.waitForMessages(std::chrono::milliseconds(1)))
            {
                sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

                for (int i = 0; i < receivedMessage.getFrameData().size(); i++)
                {
                    data[i] = receivedMessage.getFrameData()[i];
                }

                uint16_t canId = uint16_t(receivedMessage.getCanId());

                // 处理伺服电机CAN消息（参考serAngle.cpp）
                if (canId == 0x141){
                    ser1.processCanMessage(data);
                }
                else if (canId == 0x142) {
                    ser2.processCanMessage(data);}
                else if (canId == 0x143) {
                    ser3.processCanMessage(data);
                } else if (canId == 0x144) {
                    ser4.processCanMessage(data);
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "⚠ CAN接口初始化失败: " << e.what() << std::endl;
        std::cerr << "  程序将继续运行（仅记录模拟数据）" << std::endl;

        // CAN接口不可用时，线程空转
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

/**
 * @brief 角度处理线程（已废弃 - 功能已合并到 pidControlLoop）
 * @details 此函数已不再使用，角度读取功能已合并到 pidControlLoop() 中
 *          保留此代码仅供参考
 */
/*
void processAngle()
{
    std::cout << "✓ 角度处理线程已启动" << std::endl;


    while (running)
    {
        // 1. 读取伺服电机角度（参考arm.cpp）
        ser1.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser1.readStatus2();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser2.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser2.readStatus2();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // 2. 读取霍尔传感器角度并传递给肌肉关节
        float hall_angles[8];
             std::vector<float> angles = hallSensors.getAllAngles();
        hall_angles[0] =angles[0];
        hall_angles[1] = angles[1];
        hall_angles[2] = angles[2];
        hall_angles[3] = angles[3];
        hall_angles[4] = angles[4];
        hall_angles[5] = angles[5];
        hall_angles[6] = -angles[8];
        hall_angles[7] = angles[0];

        muscle1.processAngle(hall_angles[0], hall_angles[7]);
        muscle2.processAngle(hall_angles[1], hall_angles[6]);
        muscle3.processAngle(hall_angles[2], hall_angles[5]);
        muscle4.processAngle(hall_angles[6], hall_angles[7]);

        // 延迟6ms（与arm.cpp保持一致，总周期约10ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(6));
    }

    std::cout << "✓ 角度处理线程已退出" << std::endl;
}
*/

/**
 * @brief 日志记录线程（独立版本）
 * @details 定期记录肌肉关节和伺服电机的角度数据到CSV文件
 *          采样周期：100ms（比PID控制周期稍快，数据量合理）
 */
void logPidData()
{
    std::cout << "✓ 日志记录线程已启动" << std::endl;

    // 创建带时间戳的日志文件名
    std::time_t now = std::time(nullptr);
    std::tm *ltm = std::localtime(&now);
    std::stringstream ss;
    ss << "log/pid_log_" << std::put_time(ltm, "%Y%m%d_%H%M%S") << ".csv";

    std::ofstream logFile(ss.str(), std::ios::out);

    if (!logFile.is_open())
    {
        std::cerr << "✗ 无法创建日志文件: " << ss.str() << std::endl;
        std::cerr << "  请确保 log/ 目录存在" << std::endl;
        return;
    }

    std::cout << "✓ 日志文件已创建: " << ss.str() << std::endl;

    // 写入CSV头部
    // logFile << "iteration,current_angle1,current_angle2,target_angle1,target_angle2,motor1_angle,motor2_angle" << std::endl;

    int iteration = 0;

    while (running)
    {
        // 1. 读取霍尔传感器角度
        float hall_angles[8];
          std::vector<float> angles = hallSensors.getAllAngles();
         hall_angles[0] = -angles[11];
        hall_angles[1] = angles[3];
        hall_angles[2] = -angles[10];
        hall_angles[3] = angles[2];
        hall_angles[4] = -angles[9];
        hall_angles[5] = angles[1];
        hall_angles[6] = -angles[8];
        hall_angles[7] = angles[0];

        // 2. 读取伺服电机角度
        float motor_angles[2];
        motor_angles[0] = ser1.MAngle();
        motor_angles[1] = ser2.MAngle();

        // 3. 获取实时目标角度（使用 getter 函数）
        double target_angle1 = muscle1.getTargetAngle1();
        double target_angle2 = muscle1.getTargetAngle2();
        double target_angle3 = muscle2.getTargetAngle1();
        double target_angle4 = muscle2.getTargetAngle2();
        double target_angle5 = muscle3.getTargetAngle1();
        double target_angle6 = muscle3.getTargetAngle2();
        double target_angle7 = muscle4.getTargetAngle1();
        double target_angle8 = muscle4.getTargetAngle2();

        // // 4. 记录数据到CSV
        // // // 记录 muscle4 的数据（当前角度、实时目标角度、伺服电机角度）
        // logFile << iteration*0.050 << ","
        //         << hall_angles[0] << "," << hall_angles[1] << ","
        //         << target_angle1 << "," << target_angle2 << ","
        //         << motor_angles[0] << "," << motor_angles[1] << std::endl;
              // 记录 muscle3 4 的数据（当前角度、实时目标角度、伺服电机角度）
        // logFile << iteration*0.050 << ","
        //         << hall_angles[4] << "," << hall_angles[5] << ","
        //         << hall_angles[6] << "," << hall_angles[7] << ","
        //         << target_angle5 << "," << target_angle6 << ","
        //         << target_angle7 << "," << target_angle8 << std::endl;

        //记录整臂的角度
             logFile << iteration*0.050 << ","
                    << hall_angles[0] << "," << hall_angles[1] << ","
                    << hall_angles[2] << "," << hall_angles[3] << ","
                << hall_angles[4] << "," << hall_angles[5] << ","
                << hall_angles[6] << "," << hall_angles[7] << ","
                << target_angle1 << "," << target_angle2 << ","
                << target_angle3 << "," << target_angle4 << "," 
                << target_angle5 << "," << target_angle6 << "," 
                << target_angle7 << "," << target_angle8 << ","<< motor_angles[0] << "," << motor_angles[1] << std::endl;
    

        // 定期刷新文件缓冲区（每10次迭代，即1秒）
        if (iteration % 10 == 0) {
            logFile.flush();
        }

        iteration++;

        // 延迟100ms（采样周期）
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 关闭日志文件
    if (logFile.is_open()) {
        logFile.close();
        std::cout << "✓ 日志文件已保存，共记录 " << iteration << " 次迭代" << std::endl;
    }

    std::cout << "✓ 日志记录线程已退出" << std::endl;
}

/**
 * @brief PID 控制线程（优化版：合并角度读取和 PID 控制）
 * @details 在同一个线程中完成：
 *          1. 读取传感器角度（霍尔传感器和伺服电机）
 *          2. 更新角度数据到 MuscleJoint 对象
 *          3. 执行 PID 计算和气压输出
 *          控制周期：10ms（角度读取 4ms + PID 计算 + 延时 6ms）
 */
void pidControlLoop()
{
    std::cout << "✓ PID 控制线程已启动（合并角度读取和 PID 控制）" << std::endl;

    while (running)
    {
        // ========== 步骤 1: 读取伺服电机角度 ==========
        ser1.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser1.readStatus2();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser2.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser2.readStatus2();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // ========== 步骤 2: 读取霍尔传感器角度 ==========
        //这个是我静力学模型单臂对应的角度 这个顺序完全是为了和静力学模型对应
        float hall_angles[8];
        std::vector<float> angles = hallSensors.getAllAngles();
        hall_angles[0] = -angles[11];
        hall_angles[1] = angles[3];
        hall_angles[2] = -angles[10];
        hall_angles[3] = angles[2];
        hall_angles[4] = -angles[9];
        hall_angles[5] = angles[1];
        hall_angles[6] = -angles[8];
        hall_angles[7] = angles[0];

        // ========== 步骤 3: 更新角度数据到 MuscleJoint 对象 ==========
        muscle1.processAngle(hall_angles[0], hall_angles[1]);
        muscle2.processAngle(hall_angles[2], hall_angles[3]);
        muscle3.processAngle(hall_angles[4], hall_angles[5]);
        muscle4.processAngle(hall_angles[6], hall_angles[7]);

        // ========== 步骤 4: 执行 PID 计算和气压输出 ==========
        // 注意：只更新 muscle4，因为原代码中只有 muscle4 在运行
        // 如需控制其他关节，取消注释相应行
        muscle1.update();
        muscle2.update();
        muscle3.update();
        muscle4.update();

        // ========== 步骤 5: 控制周期延时 ========== 
        // 延迟 6ms，总周期约 10ms（4ms 伺服电机读取 + PID 计算 + 6ms 延时）
        std::this_thread::sleep_for(std::chrono::milliseconds(24));
    }

    // 线程停止时，将所有气压输出归零
    std::cout << "✓ PID 控制线程正在停止，归零所有气压输出..." << std::endl;
    muscle1.setPressure(0, 0, 0, 0);
    muscle2.setPressure(0, 0, 0, 0);
    muscle3.setPressure(0, 0, 0, 0);
    muscle4.setPressure(0, 0, 0, 0);

    std::cout << "✓ PID 控制线程已退出" << std::endl;
}

