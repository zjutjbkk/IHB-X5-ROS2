#pragma once
#include <cstdint>
#include <mutex>
#include "CanDriver.hpp"
#include "MiniPID.h"
#include <memory>
namespace ihb
{

    class MuscleJoint
    {
    public:
        MuscleJoint(uint16_t id = 0, double kp = 0, double ki = 0, double kd = 0);
        ~MuscleJoint();
        void setCanDriver(std::shared_ptr<sockcanpp::CanDriver> driver)
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            this->canDriver = driver;
        }

        void setId(uint16_t id)
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            _id = id;
        }
        int16_t id()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return _id;
        }
        int Angle1()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return _f_angle[0];
        }
        int Angle2()
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            return _f_angle[1];
        }

        void getAngle(int16_t &angle1, int16_t &angle2);
        void getPressure(uint16_t &pressure1, uint16_t &pressure2, uint16_t &pressure3, uint16_t &pressure4);
        void getStiffness(uint8_t &stiffness1, uint8_t &stiffness2);
        void getStiffness(uint8_t &stiffness1);

        // 设置角度
        void setAngle(int16_t angle1, int16_t angle2);
        void setAnglePID(float angle1, float angle2, float kp, float ki, float kd); // PID控制
        // 设置零点
        void setZero();
        // 设置变刚度
        void setStiffness(int16_t stiffness1, int16_t stiffness2); // 目前开环控制，后续可能会改为闭环控制
        void setStiffness(int16_t stiffness1);                     // 目前开环控制，后续可能会改为闭环控制

        void processCanMessage(uint8_t *data);
        void processAngle(float a1, float a2);

        void setPressure(uint16_t pressure_1, uint16_t pressure_2, uint16_t pressure_3, uint16_t pressure_4);
        void setDoublePressure(double pressure_1, double pressure_2);

        // 新增：设置目标角度
        void setTargetAngle(double target1, double target2);

        // 新增：获取目标角度
        double getTargetAngle1() const;
        double getTargetAngle2() const;

        // 新增：PID更新函数
        void update();

        // 新增：设置 PID 参数
        // 在类定义中修改声明
        void setPidParameters(double kp1, double ki1, double kd1, double kp2, double ki2, double kd2);

    private:
        uint8_t _id;
        int16_t _angle[2];
        int16_t _targetAngle[2];
        uint16_t _pressure[4];
        uint8_t _stiffness[2];
        float _f_angle[2];  // 角度浮点数据 外部提供
        float _fv_angle[2]; // 虚拟关节角度数据 计算获得
        // sockcanpp::CanDriver *canDriver;
        std::shared_ptr<sockcanpp::CanDriver> canDriver;
        // void setPressure();

        // PID 控制器成员变量
        MiniPID _pid1;
        MiniPID _pid2;
        double _target_angle1;
        double _target_angle2;

        void calcVirAngle(float alpha, float beta, float &G, float &H);

        // 内部方法（假设调用者已持有锁）
        void setPressureInternal(uint16_t pressure_1, uint16_t pressure_2, uint16_t pressure_3, uint16_t pressure_4);

        // 线程安全保护
        mutable std::mutex _data_mutex;

        // 优化：添加原子变量减少锁竞争（可选优化）
        // std::atomic<float> _f_angle_atomic[2];  // 如需高频访问可启用
    };

} // namespace ihb