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
#include "stepper_motor_joint.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <vector>
#include <chrono>
#include <iomanip>
#include "MiniPID.h"
#include "statics_model.h"
using namespace ihb;
// === 全局对象 - 使用线程安全的新接口 ===
ihb::HallSensors hallSensors;
// 注意：force_sensors 已被用户标记为不使用，所以注释掉
// ihb::ForceSensors forceSensors;

// === 全局控制变量 ===
std::atomic<bool> g_running(true);
std::atomic<int> g_message_count(0);

// === 函数声明 ===
void receiveCanFramesExample(); // 接收CAN帧的示例函数 - 已优化线程安全
void progressAngle();           // 处理角度传感器数据 - 已优化性能
void signalHandler(int signum); // 信号处理函数

// 检查CAN总线状态
bool checkCanStatus(const std::string &interface = "can1")
{
    // 检查接口是否存在
    std::string path = "/sys/class/net/" + interface + "/operstate";
    std::ifstream statusFile(path);
    if (!statusFile.is_open())
    {
        std::cout << "CAN接口 " << interface << " 不存在！" << std::endl;
        return false;
    }

    // 读取接口状态
    std::string status;
    std::getline(statusFile, status);
    statusFile.close();

    // 检查状态是否为"up"
    if (status == "up")
    {
        std::cout << "CAN接口 " << interface << " 已经启动！" << std::endl;
        return true;
    }
    else
    {
        std::cout << "CAN接口 " << interface << " 状态: " << status << std::endl;
        return false;
    }
}

// 初始化CAN总线
bool initCanBus()
{
    // 首先检查CAN总线状态
    if (checkCanStatus("can1"))
    {
        std::cout << "CAN总线已经启动，无需重新初始化" << std::endl;
        return true;
    }

    int ret = 0;

    std::cout << "开始初始化CAN总线..." << std::endl;

    // 设置CAN总线比特率
    ret = system("sudo ip link set can1 type can bitrate 1000000");
    if (ret != 0)
    {
        std::cerr << "设置CAN总线比特率失败！" << std::endl;
        return false;
    }

    // 休眠1秒
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 启动CAN总线
    ret = system("sudo ip link set can1 up");
    if (ret != 0)
    {
        std::cerr << "启动CAN总线失败！" << std::endl;
        return false;
    }

    // 再休眠5秒确保CAN总线完全启动
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "CAN总线初始化成功！" << std::endl;
    return true;
}

// void logW(); // 记录数据

// === 全局设备对象 - 线程安全设计 ===
ihb::ServoMotorJoint ser1(1);
ihb::ServoMotorJoint ser2(2);
ihb::ServoMotorJoint ser3(3); // 伺服电机
ihb::ServoMotorJoint ser4(4);

ihb::MuscleJoint muscle1(0); // l1 - 左臂肌肉关节
ihb::MuscleJoint muscle2(1);
ihb::MuscleJoint muscle3(2);
ihb::MuscleJoint muscle4(3);
ihb::MuscleJoint muscle5(4); // r1 - 右臂肌肉关节
ihb::MuscleJoint muscle6(5);
ihb::MuscleJoint muscle7(6);
ihb::MuscleJoint muscle8(7);
ihb::MuscleJoint muscle9(8); // 暂定的末端执行器

// 信号处理函数
void signalHandler(int signum)
{
    std::cout << "\n捕获到信号 " << signum << "，程序即将安全退出..." << std::endl;
    g_running = false;

    // 安全停止所有设备
    std::cout << "正在停止霍尔传感器..." << std::endl;
    hallSensors.stop();

    std::cout << "正在停止伺服电机..." << std::endl;
    ser1.close();
    ser2.close();
    ser3.close();
    ser4.close();

    std::cout << "正在停止肌肉关节..." << std::endl;
    muscle1.setPressure(0, 0, 0, 0);
    muscle2.setPressure(0, 0, 0, 0);
    muscle3.setPressure(0, 0, 0, 0);
    muscle4.setPressure(0, 0, 0, 0);
    muscle5.setPressure(0, 0, 0, 0);
    muscle6.setPressure(0, 0, 0, 0);
    muscle7.setPressure(0, 0, 0, 0);
    muscle8.setPressure(0, 0, 0, 0);
    muscle9.setPressure(0, 0, 0, 0);

    exit(signum);
}

int main(int argc, char **argv)
{
    // 注册信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== IHB-X5 伺服电机与霍尔传感器测试程序 ===" << std::endl;
    std::cout << "使用线程安全的优化接口" << std::endl;
    std::cout << "按 Ctrl+C 可随时安全退出" << std::endl;
    std::cout << "=========================================" << std::endl;

    // 首先初始化CAN总线
    if (!initCanBus())
    {
        std::cerr << "❌ CAN总线初始化失败，程序将继续但可能无法通信！" << std::endl;
    }

    std::cout << "🔧 肌肉关节ID: " << muscle1.id() << ", " << muscle2.id()
              << ", " << muscle3.id() << ", " << muscle4.id() << std::endl;

    // 初始化所有肌肉关节为零压力
    std::cout << "🔄 初始化肌肉关节..." << std::endl;
    muscle1.setPressure(0, 0, 0, 0);
    muscle2.setPressure(0, 0, 0, 0);
    muscle3.setPressure(0, 0, 0, 0);
    muscle4.setPressure(0, 0, 0, 0);
    muscle5.setPressure(0, 0, 0, 0);
    muscle6.setPressure(0, 0, 0, 0);
    muscle7.setPressure(0, 0, 0, 0);
    muscle8.setPressure(0, 0, 0, 0);
    muscle9.setPressure(0, 0, 0, 0);

    // 启动伺服电机
    std::cout << "🔄 启动伺服电机..." << std::endl;
    ser1.start();
    ser2.start();
    ser3.start();
    ser4.start();

    // // 设置伺服电机初始位置
    ser1.setPositonWithSpeedLimit(0, 200);
    ser2.setPositonWithSpeedLimit(0, 200);
    ser3.setPositonWithSpeedLimit(0, 120);
    ser4.setPositonWithSpeedLimit(0, 120);

    // 启动霍尔传感器 (不自动打印，手动读取)
    std::cout << "🔄 启动霍尔传感器..." << std::endl;
    hallSensors.start(0);

    // 等待传感器稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 创建并启动线程 (使用优化的线程管理)
    std::cout << "🚀 启动后台线程..." << std::endl;
    std::thread receiveThread(receiveCanFramesExample);
    std::thread angleThread(progressAngle);

    receiveThread.detach();
    angleThread.detach();

    std::cout << "✅ 系统初始化完成！" << std::endl;
    std::cout << "📊 开始数据监控..." << std::endl;
    std::cout << "Press Enter to start motor movement test...";
    std::cin.get();

    // // 设置电机测试位置 (使用线程安全接口，无需外部加锁)
    // std::cout << "🔄 设置电机测试位置..." << std::endl;
    // ser1.setPositonWithSpeedLimit(0 * 100, 120);  // 10度
    // ser2.setPositonWithSpeedLimit(0 * 100, 120);  // 20度

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "📈 开始实时数据监控 (每秒更新)..." << std::endl;
    std::cout << "格式: 伺服电机角度 | 霍尔传感器角度值" << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;

    ser1.setPositonWithSpeedLimit(0, 200);
    ser2.setPositonWithSpeedLimit(5*100, 200);
    ser3.setPositonWithSpeedLimit(0, 120);
    ser4.setPositonWithSpeedLimit(5*100, 120);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
 
    // int loop_count = 0;
    // muscle7.setPressure(100, 0, 0, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // muscle7.setPressure(0, 100, 0, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // muscle7.setPressure(0, 0, 100, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // muscle7.setPressure(0, 0, 0, 100);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // muscle7.setPressure(0, 0, 0, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // muscle8.setPressure(100, 0, 0, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // muscle8.setPressure(0, 100, 0, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // muscle8.setPressure(0, 0, 100, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // muscle8.setPressure(0, 0, 0, 100);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // muscle9.setPressure(300, 0, 0, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // muscle9.setPressure(0, 300, 0, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // muscle9.setPressure(0, 0, 300, 0);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // muscle9.setPressure(0, 0, 0, 300);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    //    muscle1.setPressure(100, 0, 0, 0);
    // //静力学输出
    //  StaticsModel model;
    // //     std::array<float, 10> joint_angles = {
    // //     0.0f, 0.0f, 10.0f, 0.0f, 10.0f,
    // //     0.0f, 10.0f, 0.0f, 10.0f, 0.0f
    // // };
    //      std::array<float, 10> joint_angles = {
    //     0.0f, 0.0f, 0.0f, 10.0f, 0.0f,
    //     10.0f, 0.0f, 10.0f, 0.0f, 10.0f
    // };
    // std::array<float, 16> muscle_pressures;
    // // bool success = model.computeGimbalMusclePressures(joint_angles, muscle_pressures);
    // // muscle1.setDoublePressure(120, 0);
    // // muscle3.setDoublePressure(120, 0);
    // // muscle3.setDoublePressure(120, 0);
    // //   muscle4.setDoublePressure(98, 0);
    // muscle1.setPressure(muscle_pressures[0], muscle_pressures[1], muscle_pressures[2], muscle_pressures[3]);
    // // muscle2.setPressure(muscle_pressures[4], muscle_pressures[5], muscle_pressures[6], muscle_pressures[7]);
    // // muscle3.setPressure(muscle_pressures[8], muscle_pressures[9], muscle_pressures[10], muscle_pressures[11]);
    // // muscle4.setPressure(muscle_pressures[12], muscle_pressures[13], muscle_pressures[14], muscle_pressures[15]);
    // // muscle1.setPressure(100, 100, 0, 0);
    // // muscle2.setPressure(100, 100, 0, 0);
    // // muscle3.setPressure(100, 100, 0, 0);
    // // muscle4.setPressure(100, 100, 0, 0);
    // // muscle1.setPressure(0, 100, 100, 0);
    // // muscle2.setPressure(0, 100, 100, 0);
    // // muscle3.setPressure(0, 100, 100, 0);
    // // muscle4.setPressure(0, 100, 100, 0);
    // sleep(5);
    // muscle1.setPressure(0, 0, 0, 0);
    // muscle2.setPressure(0, 0, 0, 0);
    // muscle3.setPressure(0, 0, 0, 0);
    // muscle4.setPressure(0, 0, 0, 0);

    ser1.setPositonWithSpeedLimit(0, 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ser2.setPositonWithSpeedLimit(0, 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ser3.setPositonWithSpeedLimit(0, 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ser4.setPositonWithSpeedLimit(0, 10);

    // while(g_running) {
    //     // 使用线程安全接口读取伺服电机角度 (无需外部加锁)
    //     std::cout << "Loop " << std::setw(3) << loop_count++ << " | ";
    //     std::cout << "Servo: ["
    //               << std::setw(6) << ser1.MAngle() << ", "
    //               << std::setw(6) << ser2.MAngle() << ", "
    //               << std::setw(6) << ser3.MAngle() << ", "
    //               << std::setw(6) << ser4.MAngle() << "] | ";

    //     // 使用线程安全接口批量读取霍尔传感器数据
    //     std::vector<float> hall_angles = hallSensors.getAllAngles();
    //     std::cout << "Hall: [";
    //     for (int i = 0; i < 16; i++) {
    //         std::cout << std::fixed << std::setprecision(1) << std::setw(5) << hall_angles[i];
    //         if (i < 15) std::cout << ",";
    //     }
    //     std::cout << "]" << std::endl;

    //     // 显示消息统计
    //     if (loop_count % 10 == 0) {
    //         std::cout << "📊 CAN消息计数: " << g_message_count.load() << std::endl;
    //     }

    //     // 优化：减少输出频率，提高性能
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }

    // force 无需启动
    // forceSensors.start();
    // muscle1.setAnglePID(13, -13);

    // // creat muscle1 pid thread and detach ni ming han shu
    // std::thread muscle1PidThread([&]()
    //                              { muscle1.setAnglePID(-8, 8); });
    // muscle1PidThread.detach();

    // // creat muscle2 pid thread and detach ni ming han shu
    // std::thread muscle2PidThread([&]()
    //                              { muscle2.setAnglePID(-8, -8); });
    // muscle2PidThread.detach();

    // // creat muscle3 pid thread and detach ni ming han shu
    // std::thread muscle3PidThread([&]()
    //                              { muscle3.setAnglePID(-8, -8); });
    // muscle3PidThread.detach();

    // creat muscle4 pid thread and detach ni ming han shu
    // std::thread muscle4PidThread([&]()
    //                              { muscle4.setAnglePID(-8, -8); });
    // muscle4PidThread.detach();

    // while(1){

    // }

    muscle1.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle2.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle3.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle4.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle5.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle6.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle7.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle8.setPressure(0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    muscle9.setPressure(0, 0, 0, 0);
    return 0;
}

// void logW()
// {
//     std::time_t now = std::time(nullptr);
//     std::tm *ltm = std::localtime(&now);
//     // 创建带有时间戳文件名的日志
//     std::stringstream ss;
//     ss << "log/hulog_" << std::put_time(ltm, "%Y%m%d_%H%M%S") << ".txt";
//     std::ofstream logFile(ss.str(), std::ios::app);

//     if (!logFile.is_open())
//     {
//         std::cerr << "Failed to open log file." << std::endl;
//         return;
//     }
//     logFile << "time, muscle1_angle, muscle2_angle, muscle3_angle, muscle4_angle, motor1_angle, motor2_angle";
//     int i = 0;
//     while (1)
//     {
//         logFile << i << "," << hallSensors.getFValue(0) << ","
//                 << hallSensors.getFValue(1) << "," << hallSensors.getFValue(2) << "," << hallSensors.getFValue(3) << "," << hallSensors.getFValue(4) << ","
//                 << hallSensors.getFValue(5) << "," << hallSensors.getFValue(6) << "," << hallSensors.getFValue(7) << "," << ser1.MAngle() << "," << ser2.MAngle() << std::endl;
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }
//     logFile.close();
// }

void progressAngle()
{
    std::cout << "🔄 角度查询线程已启动 (线程安全优化版本)" << std::endl;

    while (g_running)
    {
        // 使用线程安全接口查询伺服电机角度 (无需外部加锁)
        ser1.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        ser2.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        ser3.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        ser4.readAngle();

        // 优化：适当增加延迟，减少CAN总线负载
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        // 可选：处理肌肉关节角度映射 (使用线程安全接口)
        // 如需启用，取消注释以下代码：
        // std::vector<float> hall_angles = hallSensors.getAllAngles();
        // muscle1.processAngle(hall_angles[0], hall_angles[7]);
        // muscle2.processAngle(hall_angles[1], hall_angles[6]);
        // muscle3.processAngle(hall_angles[2], hall_angles[5]);
        // muscle4.processAngle(hall_angles[3], hall_angles[4]);
    }

    std::cout << "🛑 角度查询线程已安全退出" << std::endl;
}

void receiveCanFramesExample()
{
    std::cout << "🔄 CAN接收线程已启动 (线程安全优化版本)" << std::endl;

    sockcanpp::CanDriver canDriver("can1", CAN_RAW);
    // 关闭回环
    int loopback = 0;
    setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    uint8_t data[8] = {0};

    while (g_running)
    {
        // 优化：增加等待时间，减少CPU占用
        if (canDriver.waitForMessages(std::chrono::milliseconds(5)))
        {
            // read a single message
            sockcanpp::CanMessage receivedMessage = canDriver.readMessage();
            // can已经全收了 分id接受即可

            // convert message to data
            for (int i = 0; i < receivedMessage.getFrameData().size(); i++)
            {
                data[i] = receivedMessage.getFrameData()[i];
            }

            uint16_t canId = uint16_t(receivedMessage.getCanId());

            // 使用线程安全接口处理CAN消息 (无需外部加锁)
            if (canId == 0x141)
            {
                ser1.processCanMessage(data);
                g_message_count++;
            }
            else if (canId == 0x142)
            {
                ser2.processCanMessage(data);
                g_message_count++;
            }
            else if (canId == 0x143)
            {
                ser3.processCanMessage(data);
                g_message_count++;
            }
            else if (canId == 0x144)
            {
                ser4.processCanMessage(data);
                g_message_count++;
                // 0x92读取多圈角度
                //  优化：移除频繁的调试输出
                //  std::cout << ser1.MAngle() << "," << ser2.MAngle() << std::endl;
            }
            // 优化：移除不必要的调试输出
            // std::cout << "ID: " << std::hex << uint16_t(receivedMessage.getCanId()) << " DATA: ";
            // std::cout << ser1.Speed() << "\t" << ser1.Current() << "\t" << ser1.Angle() << "\t" << ser1.MAngle() << std::endl;
        }
        else
        {
            // 优化：没有消息时短暂休眠，避免忙等待
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cout << "🛑 CAN接收线程已安全退出" << std::endl;
}
