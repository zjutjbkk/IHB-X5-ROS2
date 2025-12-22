#include <iostream>
#include <string>
#include <csignal>

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
#include <fstream>
#include "MiniPID.h"

// canDriver =
ihb::HallSensors hallSensors;
ihb::ForceSensors forceSensors;

// void setPressure(uint8_t id, uint16_t pressure);
// float calcPressure(float angle); // 前馈值
// void moveAngle(float angle);
// void restartCanDriver();
// void getVirAngle(double alpha, double beta, double *v_ang1, double *v_ang2);

void receiveCanFramesExample(); // 接收CAN帧的示例函数
void progressAngle();           // 处理角度传感器数据

void logW(); // 记录数据

ihb::ServoMotorJoint ser1(3);
ihb::ServoMotorJoint ser2(4);
uint16_t id = 0; // 假设你想传递的参数是1

ihb::MuscleJoint muscle1(16); // l1
ihb::MuscleJoint muscle2(17);
ihb::MuscleJoint muscle3(18);
ihb::MuscleJoint muscle4(19);

int main(int argc, char **argv)
{
    // muscle1.setId(1);  // why???
    // muscle2.setId(2);
    // muscle3.setId(3);
    // muscle4.setId(4);

    std::cout << muscle1.id() << muscle2.id() << std::endl;

    // creat receive thread and detach
    std::thread receiveThread(receiveCanFramesExample);
    receiveThread.detach();
    // angle thread
    std::thread angleThread(progressAngle);

    muscle1.setPressure(0, 0, 0, 0);
    muscle2.setPressure(0, 0, 0, 0);
    muscle3.setPressure(0, 0, 0, 0);
    muscle4.setPressure(0, 0, 0, 0);
    ser1.setPositonWithSpeedLimit(0, 120);
    ser2.setPositonWithSpeedLimit(0, 120);

    angleThread.detach();

    // hall 设置零点并启动
    hallSensors.start();

    // creat log thread and detach

    //
    std::cout << "Press Enter to continue...";
    std::cin.get();

    std::thread logThread(logW);
    logThread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ser1.setPositonWithSpeedLimit(30 * 100, 120);
    ser2.setPositonWithSpeedLimit(30 * 100, 120);
    // force 无需启动
    // forceSensors.start();
    // muscle1.setAnglePID(13, -13);

    // creat muscle1 pid thread and detach ni ming han shu
    std::thread muscle1PidThread([&]()
                                 { muscle1.setAnglePID(-18, 13); });
    muscle1PidThread.detach();

    // creat muscle2 pid thread and detach ni ming han shu
    std::thread muscle2PidThread([&]()
                                 { muscle2.setAnglePID(-18, -23); });
    muscle2PidThread.detach();

    // creat muscle3 pid thread and detach ni ming han shu
    std::thread muscle3PidThread([&]()
                                 { muscle3.setAnglePID(-20, -13); });
    muscle3PidThread.detach();

    // creat muscle4 pid thread and detach ni ming han shu
    std::thread muscle4PidThread([&]()
                                 { muscle4.setAnglePID(-18, -20); });
    muscle4PidThread.detach();

    // while(1){

    // }

    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

    return 0;
}

void logW()
{
    std::time_t now = std::time(nullptr);
    std::tm *ltm = std::localtime(&now);
    // 创建带有时间戳文件名的日志
    std::stringstream ss;
    ss << "log/hulog_" << std::put_time(ltm, "%Y%m%d_%H%M%S") << ".txt";
    std::ofstream logFile(ss.str(), std::ios::app);

    if (!logFile.is_open())
    {
        std::cerr << "Failed to open log file." << std::endl;
        return;
    }
    logFile << "time, muscle1_angle, muscle2_angle, muscle3_angle, muscle4_angle, motor1_angle, motor2_angle";
    int i = 0;
    while (1)
    {
        logFile << i << "," << hallSensors.getAngle(0) << ","
                << hallSensors.getAngle(1) << "," << hallSensors.getAngle(2) << "," << hallSensors.getAngle(3) << "," << hallSensors.getAngle(4) << ","
                << hallSensors.getAngle(5) << "," << hallSensors.getAngle(6) << "," << hallSensors.getAngle(7) << ","
                << ser1.MAngle() << "," << ser2.MAngle() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    logFile.close();
}

void progressAngle()
{

    while (1)
    {

        // 发送电机查询命令
        ser1.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser1.readStatus2();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser2.readAngle();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ser2.readStatus2();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // 获取角度并传递  TODO 序号需要修改
        muscle1.processAngle(hallSensors.getAngle(0), hallSensors.getAngle(7));
        muscle2.processAngle(hallSensors.getAngle(1), hallSensors.getAngle(6));
        muscle3.processAngle(hallSensors.getAngle(2), hallSensors.getAngle(5));
        muscle4.processAngle(hallSensors.getAngle(3), hallSensors.getAngle(4));
        std::this_thread::sleep_for(std::chrono::milliseconds(6));
    }
}

void receiveCanFramesExample()
{
    sockcanpp::CanDriver canDriver("can1", CAN_RAW);
    // 关闭回环
    int loopback = 0;
    setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    uint8_t data[8] = {0};

    // cannot convert ‘receiveCanFramesExample()::<lambda()>’ to ‘std::chrono::milliseconds’ {aka ‘std::chrono::duration<long int, std::ratio<1, 1000> >’}
    while (1)
    {
        if (canDriver.waitForMessages(std::chrono::milliseconds(1)))
        {
            // read a single message
            sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

            // convert message to data
            for (int i = 0; i < receivedMessage.getFrameData().size(); i++)
            {

                data[i] = receivedMessage.getFrameData()[i];
            }

            if (uint16_t(receivedMessage.getCanId()) == 0x143)
            {
                ser1.processCanMessage(data);
            }
            if (uint16_t(receivedMessage.getCanId()) == 0x144)
            {
                ser2.processCanMessage(data);
                // std::cout << ser1.MAngle() << "," << ser2.MAngle() << std::endl;
            }
            // std::cout << "ID: " << std::hex << uint16_t(receivedMessage.getCanId()) << " DATA: ";
            // std::cout << ser1.Speed() << "\t" << ser1.Current() << "\t" << ser1.Angle() << "\t" << ser1.MAngle() << std::endl;
        }
    }
}

