#pragma once
#include <cstdint>
#include "CanDriver.hpp"
#include <memory>
#include <mutex>
// Reduction ratio 减速比
#define Ratio 36
namespace ihb
{

    class ServoMotorJoint
    {
    public:
        ServoMotorJoint(uint16_t id = 0);
        ~ServoMotorJoint();
        void setId(uint16_t id)
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            _id = id + 0x140;
        }
        uint16_t id()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return _id;
        }
        // 置零
        // 谨慎使用
        // void setZero();             // 因为内部flash问题，不建议频繁使用
        // void setTorque(int torque); // 会一直转圈

        // 带有速度限制的位置闭环
        void setPositonWithSpeedLimit(int position, int speedLimit);
        void setCanDriver(std::shared_ptr<sockcanpp::CanDriver> driver) {
        std::lock_guard<std::mutex> lock(_data_mutex);
                canDriver = driver;
                       }
        // 速度闭环
        // void setSpeed(int speed);
        // 转矩闭环

        void readAngle();
        // 读取当前状态 温度和相电流数据
        void readStatus2();
        // 停止
        void stop();
        void close();
        // 启动
        void start();
        // 处理CAN报文
        // 保护 内部状态不会被外部修改
        double MAngle()
        {
            
            std::lock_guard<std::mutex> lock(_data_mutex);
            // return std::round((m_angle / Ratio/100.0 )* 10) / 10;//保留一位小数  
            // std::round：对移动后的值进行四舍五入到最接近的整数 (round(18.3) -> 18.0)。这个配合double作为返回值可以实现一位小数精确变换
            double precise_angle = static_cast<double>(m_angle) / 100.0 / static_cast<double>(Ratio);
            return std::round(precise_angle*10.0)/10.0;
        }
        int16_t Angle()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return angle;
        }
        int8_t Temp()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return temp;
        }
        uint16_t Voltage()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return voltage;
        }
        int16_t Current()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return current;
        }
        int16_t Speed()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return speed;
        }

        void processCanMessage(uint8_t *data);

    private:
        // 添加互斥锁保护数据
        std::mutex _data_mutex;
        
        int64_t m_angle=0;
        int16_t angle;
        int8_t temp;
        uint16_t voltage;
        int16_t current;
        int16_t speed;
        // can
        // sockcanpp::CanDriver *canDriver;
        // std::unique_ptr<sockcanpp::CanDriver> canDriver;
        // std::s
        std::shared_ptr<sockcanpp::CanDriver> canDriver;
        // can message
        // sockcanpp::CanMessage *canMessage;
        // can data
        uint8_t _canData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        uint16_t _id;
        // 电机状态
        int _status;
        int _position;

        int _torque;
        // int _encoderPosition;
        // 电机控制
        // int _targetPosition;
        // int _targetSpeed;
        // acc
        int32_t _acc;
        //         _encoder = data[2] | (data[3] << 8);  // 当前编码器值 （原始 - 电机零点）
        // _encoder_i = data[4] | (data[5] << 8); //原始编码器值
        // _encoder_zero = data[6] | (data[7] << 8); // 电机零点
        uint16_t _encoder;
        uint16_t _encoder_i;
        uint16_t _encoder_zero;

        uint8_t _errorStatus;
    };

} // namespace ihb