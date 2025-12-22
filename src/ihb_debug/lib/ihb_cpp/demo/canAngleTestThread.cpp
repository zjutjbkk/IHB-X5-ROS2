#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <mutex>

// 添加必要的头文件
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>

#include "CanDriver.hpp"
#include "servo_motorJoint.h"

// === 全局变量 - 线程安全优化版本 ===
std::atomic<bool> running(true);
std::mutex print_mutex;
std::mutex angle_mutex;
std::atomic<int> zero_count(0);
std::atomic<int> msg_count(0);

// 共享变量：存储最新角度值 (线程安全保护)
double current_angle = 0;
uint8_t last_data[8] = {0}; // 保存最后一次接收到的原始数据

// 模拟serAngle.cpp中的receiveCanFramesExample函数
void receiveThread() {
    sockcanpp::CanDriver canDriver("can1", CAN_RAW);
    // 关闭回环
    int loopback = 0;
    setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    uint8_t data[8] = {0};
    ihb::ServoMotorJoint ser1(1);
    
    while (running) {
        // 优化：增加等待时间，减少CPU占用，提高系统稳定性
        if (canDriver.waitForMessages(std::chrono::milliseconds(5))) {
            sockcanpp::CanMessage receivedMessage = canDriver.readMessage();
            
            // 只处理ID为0x141的消息
            if (uint16_t(receivedMessage.getCanId()) == 0x141) {
                // 转换数据
                for (int i = 0; i < receivedMessage.getFrameData().size(); i++) {
                    data[i] = receivedMessage.getFrameData()[i];
                }
                
                // 使用线程安全接口处理消息
                ser1.processCanMessage(data);

                // 如果是0x92命令(角度读取响应)，更新共享变量
                if (data[0] == 0x92) {
                    double angle = ser1.MAngle(); // 线程安全接口
                    msg_count++;

                    // 线程安全地保存角度和原始数据到共享变量
                    {
                        std::lock_guard<std::mutex> lock(angle_mutex);
                        current_angle = angle;

                        for (int i = 0; i < 8; i++) {
                            last_data[i] = data[i];
                        }
                    }
                    
                    if (angle == 0) {
                        zero_count++;
                    }
                }
            }
        }
    }
}

// 模拟serAngle.cpp中的progressAngle函数
void angleThread() {
    ihb::ServoMotorJoint ser1(1);
    int request_count = 0;
    
    while (running && request_count < 500) { // 增加请求次数以便观察
        // 使用线程安全接口发送角度查询命令
        ser1.readAngle();
        request_count++;

        // 优化：增加延迟，减少CAN总线负载，提高稳定性
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    // 不设置running=false，让主线程控制程序结束
}

// 主函数
int main() {
    std::cout << "开始多线程测试，模拟serAngle.cpp中的线程环境..." << std::endl;
    
    // 初始化电机
    ihb::ServoMotorJoint ser1(1);
    ser1.setPositonWithSpeedLimit(0, 120);
    std::cout << "初始化电机位置，按Enter继续..." << std::endl;
    std::cin.get();
    
    // 设置电机目标位置
    ser1.setPositonWithSpeedLimit(10 * 100, 120);
    std::cout << "设置电机位置到1000，等待电机到达..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 创建线程
    std::cout << "启动多线程测试..." << std::endl;
    std::thread t1(receiveThread);
    std::thread t2(angleThread);
    
    // 分离线程，让它们在后台运行
    t1.detach();
    t2.detach();
    
    // 记录上次打印的角度值，用于检测变化
    double last_printed_angle = -1;
    int display_count = 0;
    int zero_displays = 0;
    
    // 主循环：获取并打印角度信息
    std::cout << "开始在主函数中监控角度数据..." << std::endl;
    
    // 运行30秒或手动中断
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time).count() < 30) {
        
        // 获取当前角度
        double angle;
        uint8_t data_copy[8];
        {
            std::lock_guard<std::mutex> lock(angle_mutex);
            angle = current_angle;
            std::memcpy(data_copy, last_data, sizeof(data_copy));
        }
        
        // 只在角度变化时或每10次循环打印一次
        if (angle != last_printed_angle || display_count % 10 == 0) {
            display_count++;
            last_printed_angle = angle;
            
            // 打印当前角度和消息计数
            std::cout << "ser1 angle:" << angle;
            
            // 检查是否为零值
            if (angle == 0) {
                zero_displays++;
                std::cout << " <-- 零值! 原始数据: [";
                for (int i = 0; i < 8; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data_copy[i];
                    if (i < 7) std::cout << ",";
                }
                std::cout << std::dec << "]";
            }
            
            std::cout << " (消息#" << msg_count << ")" << std::endl;
        }
        
        // 短暂休眠，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 结束测试
    running = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 给线程一些时间结束
    
    // 总结测试结果
    std::cout << "\n测试完成! 总共接收: " << msg_count << " 条消息，零值出现: " << zero_count 
              << " 次，打印零值: " << zero_displays << " 次" << std::endl;
    
    if (zero_count > 0) {
        std::cout << "问题复现成功！零值出现比例: " << (float)zero_count / msg_count * 100 << "%" << std::endl;
        std::cout << "可能的原因分析:" << std::endl;
        std::cout << "1. 高频率的查询请求(间隔仅1ms)导致某些CAN消息丢失或冲突" << std::endl;
        std::cout << "2. 线程同步问题导致消息被错误处理" << std::endl;
        std::cout << "3. 在processCanMessage函数中缺少数据有效性验证" << std::endl;
        
        std::cout << "\n建议修复方案:" << std::endl;
        std::cout << "1. 增加请求之间的等待时间，从1ms增加到5-10ms" << std::endl;
        std::cout << "2. 在ServoMotorJoint.cpp中的processCanMessage函数增加数据验证" << std::endl;
        std::cout << "3. 实现MAngle()方法记住上次有效值的功能，当接收到零值时返回上次值" << std::endl;
    } else {
        std::cout << "未能复现零值问题。建议测试更长时间或增加请求频率。" << std::endl;
    }
    
    return 0;
} 