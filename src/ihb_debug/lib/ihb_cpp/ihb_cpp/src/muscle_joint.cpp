#include "muscle_joint.h"
#include "sysconfig.h"
#include "MiniPID.h"
#include <iostream>
#include <algorithm> // for std::min

namespace ihb
{

    MuscleJoint::MuscleJoint(uint16_t id, double kp, double ki, double kd)
        : _pid1(kp, ki, kd), _pid2(kp, ki, kd), _target_angle1(0.0), _target_angle2(0.0)
    {
        _id = id;
        // std::cout << "MuscleJoint id: " << _id << std::endl;
        // canDriver = new sockcanpp::CanDriver(CAN_NAME, CAN_RAW);
        // std::unique_ptr<sockcanpp::CanDriver> canDriver(new sockcanpp::CanDriver(CAN_NAME, CAN_RAW, 0));
        canDriver = std::make_shared<sockcanpp::CanDriver>(CAN_NAME, CAN_RAW, 0);

        // 配置 PID 控制器
        _pid1.setOutputLimits(-250, 250);
        _pid2.setOutputLimits(-250, 250);
        _pid1.setMaxIOutput(250);
        _pid2.setMaxIOutput(250);
        _pid1.setOutputRampRate(250);
        _pid2.setOutputRampRate(250);
    }

    MuscleJoint::~MuscleJoint()
    {
    }

    void MuscleJoint::getAngle(int16_t &angle1, int16_t &angle2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        angle1 = _f_angle[0];
        angle2 = _f_angle[1];
    }
    void MuscleJoint::getPressure(uint16_t &pressure1, uint16_t &pressure2, uint16_t &pressure3, uint16_t &pressure4)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        pressure1 = _pressure[0];
        pressure2 = _pressure[1];
        pressure3 = _pressure[2];
        pressure4 = _pressure[3];
    }
    void MuscleJoint::getStiffness(uint8_t &stiffness1, uint8_t &stiffness2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        stiffness1 = _stiffness[0];
        stiffness2 = _stiffness[1];
    }
    void MuscleJoint::getStiffness(uint8_t &stiffness1)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        stiffness1 = _stiffness[0];
    }

    void MuscleJoint::setAngle(int16_t angle1, int16_t angle2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _targetAngle[0] = angle1;
        _targetAngle[1] = angle2;
        // TODO: 实现角度设置逻辑
    }
    void MuscleJoint::setAnglePID(float angle1, float angle2, float kp, float ki, float kd) // PID控制
    {
        MiniPID pid1 = MiniPID(kp, ki, kd);
        MiniPID pid2 = MiniPID(kp, ki, kd);

        pid1.setOutputLimits(-200, 200);
        pid2.setOutputLimits(-200, 200);
        pid1.setOutputRampRate(200);
        pid2.setOutputRampRate(200);

        float target_angle1 = angle1;
        float target_angle2 = angle2;
        float target_vir1 = 0;
        float target_vir2 = 0;
        float current_angle1, current_angle2;
        float current_vir1, current_vir2;
        float output1 = 0;
        float output2 = 0;

        // 获取初始角度值（线程安全）
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            current_angle1 = _f_angle[0];
            current_angle2 = _f_angle[1];
            // current_vir1 = _fv_angle[0];
            // current_vir2 = _fv_angle[1];
        }

        calcVirAngle(target_angle1, target_angle2, target_vir1, target_vir2);
        calcVirAngle(current_angle1, current_angle2, current_vir1, current_vir2);

        int iteration = 0;
        while (fabs(target_angle1 - current_angle1) > 0.1 || fabs(target_angle2 - current_angle2) > 0.1)
        {
            if (iteration > 1000)
                break;

            // 优化：减少锁的持有时间，批量获取数据
            {
                std::lock_guard<std::mutex> lock(_data_mutex);
                current_angle1 = _f_angle[0];
                current_angle2 = _f_angle[1];
            }

            calcVirAngle(current_angle1, current_angle2, current_vir1, current_vir2);
            output1 = pid1.getOutput(current_vir1, target_vir1);
            output2 = pid2.getOutput(current_vir2, target_vir2);
            setDoublePressure(output1, output2);
            iteration++;

            // 延迟（不持锁）- 优化：减少日志输出频率
            std::this_thread::sleep_for(std::chrono::milliseconds(30));

            // 减少日志输出频率，避免影响性能
            if (iteration % 10 == 0)
            {
                std::cout << "ID" << id() << "\t迭代:" << iteration << std::endl;
            }
        }
    }
    void MuscleJoint::setDoublePressure(double pressure_1, double pressure_2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);

        // 判断正负号并转换为整数后充气
        if (pressure_1 >= 0)
        {
            _pressure[1] = (uint16_t)pressure_1;
            _pressure[3] = 0;
        }
        else
        {
            _pressure[1] = 0;
            _pressure[3] = (uint16_t)fabs(pressure_1);
        }
        if (pressure_2 >= 0)
        {
            _pressure[2] = (uint16_t)pressure_2;
            _pressure[0] = 0;
        }
        else
        {
            _pressure[2] = 0;
            _pressure[0] = (uint16_t)fabs(pressure_2);
        }
        // 直接调用内部实现，避免死锁（已经持有锁）
        setPressureInternal(_pressure[0], _pressure[1], _pressure[2], _pressure[3]);
        // std::cout << "setPressure: " << _pressure[0] << ", " << _pressure[1] << ", " << _pressure[2] << ", " << _pressure[3] << std::endl;
    }

    void MuscleJoint::setZero()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        // 重置角度为零点
        _angle[0] = 0;
        _angle[1] = 0;
        _f_angle[0] = 0.0f;
        _f_angle[1] = 0.0f;
        _fv_angle[0] = 0.0f;
        _fv_angle[1] = 0.0f;
    }
    void MuscleJoint::calcVirAngle(float alpha, float beta, float &G, float &H)
    {
        // double alpha = 0;  // 1
        // double beta = 10.0; // 2
        // float alpha = _f_angle[0];
        // float beta = _f_angle[1];

        // 将角度转换为弧度
        double alpha_rad = alpha * M_PI / 180.0;
        double beta_rad = beta * M_PI / 180.0;

        // 计算F矩阵的元素
        double F1 = (sqrt(2) / 2) * (sin(beta_rad) - sin(alpha_rad) * cos(beta_rad));
        double F2 = (sqrt(2) / 2) * (sin(beta_rad) + sin(alpha_rad) * cos(beta_rad));
        double F3 = -cos(beta_rad) * cos(alpha_rad);

        // 计算 G 和 H（弧度）
        double G_rad = atan2(F3, F2);
        double H_rad = atan2(F3, F1);

        // 将弧度转换为角度并加上/减去90度
        G = G_rad * 180.0 / M_PI + 90;
        H = H_rad * 180.0 / M_PI + 90;
        // 保留2位
        G = round(G * 100) / 100;
        H = round(H * 100) / 100;

        // _fv_angle[0] = G;
        // _fv_angle[1] = H;
        // 输出结果
        // printf("G = %.2f\n", G);
        // printf("H = %.2f\n", H);
        // return G;
    }
    void MuscleJoint::setStiffness(int16_t stiffness1, int16_t stiffness2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _stiffness[0] = stiffness1;
        _stiffness[1] = stiffness2;

        unsigned char data[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00};

        data[0] = 40 + _id;
        data[2] = _stiffness[0] >> 8;
        data[3] = _stiffness[0] & 0xff;
        data[4] = 40 + _id;
        data[6] = _stiffness[1] >> 8;
        data[7] = _stiffness[1] & 0xff;
        sockcanpp::CanMessage canMessage(0x101, std::string((char *)data, 8));
        canDriver->sendMessage(canMessage);
        // delay
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    void MuscleJoint::setStiffness(int16_t stiffness1)
    {
        setStiffness(stiffness1, stiffness1);
    }
    void MuscleJoint::processCanMessage(uint8_t *data)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        // TODO
        if (data[0] == 0x01)
        {
            _angle[0] = data[1] << 8 | data[2];
            _angle[1] = data[3] << 8 | data[4];
            // _f_angle[0] = _angle[0];
            // _f_angle[1] = _angle[1];
        }
    }
    void MuscleJoint::processAngle(float a1, float a2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _f_angle[0] = a1;
        _f_angle[1] = a2;
    }

    void MuscleJoint::setPressure(uint16_t pressure_1, uint16_t pressure_2, uint16_t pressure_3, uint16_t pressure_4)

    {
        std::lock_guard<std::mutex> lock(_data_mutex);

        setPressureInternal(pressure_1, pressure_2, pressure_3, pressure_4);
    }

    void MuscleJoint::setPressureInternal(uint16_t pressure_1, uint16_t pressure_2, uint16_t pressure_3, uint16_t pressure_4)
    {
        // 内部实现，假设调用者已经持有锁
        // id =0~7,对应CAN ID = 0x 110~117

        int canID = 0x110 + _id;
        unsigned char data[8];
        memset(data, 0, sizeof(data));
        // 定义气压上限，使用常量可以方便未来修改
        const uint16_t MAX_PRESSURE_LIMIT = 280;

        // 使用 std::min 函数对每个气压值进行钳位操作
        // std::min(a, b) 会返回 a 和 b 中较小的值
        uint16_t p1_clamped = std::min(pressure_1, MAX_PRESSURE_LIMIT);
        uint16_t p2_clamped = std::min(pressure_2, MAX_PRESSURE_LIMIT);
        uint16_t p3_clamped = std::min(pressure_3, MAX_PRESSURE_LIMIT);
        uint16_t p4_clamped = std::min(pressure_4, MAX_PRESSURE_LIMIT);
        data[0] = (p1_clamped >> 8) & 0x00FF;
        data[1] = p1_clamped & 0x00FF;
        data[2] = (p2_clamped >> 8) & 0x00FF;
        data[3] = p2_clamped & 0x00FF;
        data[4] = (p3_clamped >> 8) & 0x00FF;
        data[5] = p3_clamped & 0x00FF;
        data[6] = (p4_clamped >> 8) & 0x00FF;
        data[7] = p4_clamped & 0x00FF;
        canDriver->sendMessage(sockcanpp::CanMessage(canID, std::string((char *)data, 8)));
    }

    void MuscleJoint::setTargetAngle(double target1, double target2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _target_angle1 = target1;
        _target_angle2 = target2;

        // // 重置 PID 控制器，清除积分项
        // _pid1.reset();
        // _pid2.reset();
    }

    double MuscleJoint::getTargetAngle1() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _target_angle1;
    }

    double MuscleJoint::getTargetAngle2() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _target_angle2;
    }

    void MuscleJoint::update()
    {
        float current_angle1, current_angle2;
        float current_vir1, current_vir2;
        float target_vir1, target_vir2;
        double target1, target2;

        // 读取当前角度和目标角度（线程安全）
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            current_angle1 = _f_angle[0];
            current_angle2 = _f_angle[1];
            target1 = _target_angle1;
            target2 = _target_angle2;
        }

        // 计算虚拟关节角度
        calcVirAngle(target1, target2, target_vir1, target_vir2);
        calcVirAngle(current_angle1, current_angle2, current_vir1, current_vir2);

        // 计算 PID 输出
        float output1 = _pid1.getOutput(current_vir1, target_vir1);
        float output2 = _pid2.getOutput(current_vir2, target_vir2);
        // std::cout <<  target_vir1 << "\t"
        //         <<target_vir2 << "\t"
        //         <<  current_vir1 << "\t"
        //         << current_vir2 << "\t"
        //          <<output1 << "\t"
        //         <<  output2 << std::endl;        // 输出到气压控制系统
        setDoublePressure(output1, output2);
    }

    void MuscleJoint::setPidParameters(double kp1, double ki1, double kd1, double kp2, double ki2, double kd2)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);

        // 重新配置 PID 控制器
        _pid1.setP(kp1);
        _pid1.setI(ki1);
        _pid1.setD(kd1);
        _pid2.setP(kp2);
        _pid2.setI(ki2);
        _pid2.setD(kd2);

        // // 重置 PID 控制器状态
        // _pid1.reset();
        // _pid2.reset();
    }

}
