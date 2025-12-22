#include <iostream>
// #include "CanDriver.hpp"
#include <string>
#include <csignal>

#include <sstream>
#include <queue>
#include <thread>
#include "muscle_joint.h"
#include "CanDriver.hpp"
#include "servo_motorJoint.h" // 其实头文件不该大写的，但是我懒得改了
#include "force_sensors.h"
#include "hall_sensors.h"

// can
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <vector>

#define RIGHT 1
#define LEFT 1

// 电机
ihb::ServoMotorJoint servoMotorJoint1(1);
ihb::ServoMotorJoint servoMotorJoint2(2);
ihb::ServoMotorJoint servoMotorJoint3(3);
ihb::ServoMotorJoint servoMotorJoint4(4);


// 肌肉
// std::vector<ihb::MuscleJoint> muscleJoints(8);
// ihb::MuscleJoint shou(9);



void receiveCanFramesExample()
{
    sockcanpp::CanDriver canDriver("can1", CAN_RAW);
    // 关闭回环
    int loopback = 0;
    setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // cannot convert ‘receiveCanFramesExample()::<lambda()>’ to ‘std::chrono::milliseconds’ {aka ‘std::chrono::duration<long int, std::ratio<1, 1000> >’}
    while (1)
    {
        if (canDriver.waitForMessages(std::chrono::milliseconds(1)))
        {
            // read a single message
            sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

            // 输出 他这个格式转换好烦人
            std::cout << "READ  ID: " << std::hex << uint16_t(receivedMessage.getCanId()) << " DATA: ";
            for (auto &byte : receivedMessage.getFrameData())
            {
                std::cout << int(uint8_t(byte)) << " ";
            }
            std::cout << std::endl;
        }
    }
}

// 发送CAN报文
void sendCanFramesExample()
{
    sockcanpp::CanDriver canDriver("can0", CAN_RAW);

    // cannot convert ‘receiveCanFramesExample()::<lambda()>’ to ‘std::chrono::milliseconds’ {aka ‘std::chrono::duration<long int, std::ratio<1, 1000> >’}
    // can 发送测试
    std::cout << "SEND  ID: 141      DATA: A2 00 00 00 A0 8C 00 00" << std::endl;
    for (int i = 0; i < 10; i++)
    {
        // ID: 141      DATA: A2 00 00 00 A0 8C 00 00 // 速度闭环
        // ID: 141      DATA: 9C 00 00 00 00 00 00 00 // 读取状态2
        unsigned char data[8] = {0xA2, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x00, 0x00};
        data[0] = i;
        sockcanpp::CanMessage canMessage(0x140, std::string((char *)data, 8));
        canDriver.sendMessage(canMessage);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    // sleep 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发送10个 队列发送
    std::queue<sockcanpp::CanMessage> canMessageQueue;
    for (int i = 0; i < 10; i++)
    {
        unsigned char data[8] = {0xA2, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x00, 0x00};
        data[6] = i;
        // sockcanpp::CanMessage canMessage(0x141, std::string((char *)data, 8));
        canMessageQueue.push(sockcanpp::CanMessage(0x141, std::string((char *)data, 8)));
    }
    canDriver.sendMessageQueue(canMessageQueue);
}




int main(int argc, char **argv)
{   
    // 创建接收线程
    std::thread receiveThread(receiveCanFramesExample);
    receiveThread.detach();

    // 创建发送线程
    std::thread sendThread(sendCanFramesExample);
    sendThread.detach();

    while (true){
        
    }

    

    // 延时
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // stopCan();
    return 0;
}