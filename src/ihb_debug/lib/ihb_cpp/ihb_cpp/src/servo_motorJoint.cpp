#include "servo_motorJoint.h"
#include "sysconfig.h"

namespace ihb
{

    ServoMotorJoint::ServoMotorJoint(uint16_t id)
    {
        // 初始化你的伺服电机关节
        std::lock_guard<std::mutex> lock(_data_mutex);
        _id = 0x140 + id;
        canDriver = std::make_shared<sockcanpp::CanDriver>(CAN_NAME, CAN_RAW, 0);
        // canMessage = new sockcanpp::CanMessage( 0x140+_id,std::string((char *)_canData, 8));
        // canDriver = new sockcanpp::CanDriver(CAN_NAME, CAN_RAW, 0);
        
    }

    ServoMotorJoint::~ServoMotorJoint()
    {
        // 清理你的伺服电机关节
        // delete canDriver;
        // 判断是否为空指针
        // if (canDriver)
        // {
        //     delete canDriver;
        //     canDriver = nullptr;
        // }
        // delete canMessage;
    }
    //  写入当前位置到 ROM 作为电机零点命令 0x19
    // 数据域 说明 数据
    // DATA[0] 命令字节 0x19
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00

    // 回复 电机在收到命令后回复主机，回复命令和接收命令一致

    // void ServoMotorJoint::setZero()
    // {
    //     // 设置当前位置为零点
    //     // candata置零
    //     _canData[0] = 0x19;
    //     _canData[1] = 0x00;
    //     _canData[2] = 0x00;
    //     _canData[3] = 0x00;
    //     _canData[4] = 0x00;
    //     _canData[5] = 0x00;
    //     _canData[6] = 0x00;
    //     _canData[7] = 0x00;
    //     canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    // }
    //  位置闭环控制命令 2 0xA4
    // 主机发送该命令以控制电机的位置（多圈角度）， 控制值 angleControl 为 int32_t 类型，对应实际位
    // 置为 0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定。
    // 控制值 maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转速 1dps/LSB。
    // 数据域 说明 数据
    // DATA[0] 命令字节 0xA4
    // DATA[1] NULL 0x00
    // DATA[2] 速度限制低字节 DATA[2] = *(uint8_t *)(&maxSpeed)
    // DATA[3] 速度限制高字节 DATA[3] = *((uint8_t *)(&maxSpeed)+1)
    // DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
    // DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
    // DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
    // DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
    // 备注：
    // 1. 该命令下的控制值 angleControl 受上位机(LK-Motor Tool)中的 Max Angle 值限制。
    // 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
    // 3. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
    // 制。
    void ServoMotorJoint::setPositonWithSpeedLimit(int position, int speedLimit)
    {
        // 设置你的伺服电机关节到指定位置，带有速度限制
        std::lock_guard<std::mutex> lock(_data_mutex);
        position = position * Ratio;
        _canData[0] = 0xA4;
        _canData[1] = 0x00;
        _canData[2] = speedLimit & 0xFF;
        _canData[3] = (speedLimit >> 8) & 0xFF;
        _canData[4] = position & 0xFF;
        _canData[5] = (position >> 8) & 0xFF;
        _canData[6] = (position >> 16) & 0xFF;
        _canData[7] = (position >> 24) & 0xFF;
        canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    }

    //  速度闭环控制命令 0xA2
    // 主机发送该命令以控制电机的速度，控制值 speedControl 为 int32_t 类型，对应实际转速为 0.01dps/LSB。
    // 数据域 说明 数据
    // DATA[0] 命令字节 0xA2
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] 速度控制低字节 DATA[4] = *(uint8_t *)(&speedControl)
    // DATA[5] 速度控制 DATA[5] = *((uint8_t *)(&speedControl)+1)
    // DATA[6] 速度控制 DATA[6] = *((uint8_t *)(&speedControl)+2)
    // DATA[7] 速度控制高字节 DATA[7] = *((uint8_t *)(&speedControl)+3)

    // 1. 该命令下电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限制。
    // 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。

    // void ServoMotorJoint::setSpeed(int speed)
    // {
    //     // 设置你的伺服电机关节的速度
    //     speed = speed * Ratio;
    //     _canData[0] = 0xA2;
    //     _canData[1] = 0x00;
    //     _canData[2] = 0x00;
    //     _canData[3] = 0x00;
    //     _canData[4] = speed & 0xFF;
    //     _canData[5] = (speed >> 8) & 0xFF;
    //     _canData[6] = (speed >> 16) & 0xFF;
    //     _canData[7] = (speed >> 24) & 0xFF;
    //     canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    // }

    //  转矩闭环控制命令 0xA1
    // 主机发送该命令以控制电机的转矩电流输出，控制值 iqControl 为 int16_t 类型，数值范围-2000~ 2000，
    // 对应实际转矩电流范围-32A~32A（母线电流和电机的实际扭矩因不同电机而异）。
    // 数据域 说明 数据
    // DATA[0] 命令字节 0xA1
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] 转矩电流控制值低字节 DATA[4] = *(uint8_t *)(&iqControl)
    // DATA[5] 转矩电流控制值高字节 DATA[5] = *((uint8_t *)(&iqControl)+1)
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    // void ServoMotorJoint::setTorque(int torque)
    // {
    //     // 判断数值是否超过范围
    //     if (torque > 2000)
    //     {
    //         torque = 2000;
    //     }
    //     else if (torque < -2000)
    //     {
    //         torque = -2000;
    //     }
    //     // 设置你的伺服电机关节的转矩
    //     _canData[0] = 0xA1;
    //     _canData[1] = 0x00;
    //     _canData[2] = 0x00;
    //     _canData[3] = 0x00;
    //     _canData[4] = torque & 0xFF;
    //     _canData[5] = (torque >> 8) & 0xFF;
    //     _canData[6] = 0x00;
    //     _canData[7] = 0x00;
    //     canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    // }
    //  读取多圈角度命令 0x92
    // 主机发送该命令以读取当前电机的多圈绝对角度值
    // 数据域 说明 数据
    // DATA[0] 命令字节 0x92
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00
    void ServoMotorJoint::readAngle()
    {
        // 读取你的伺服电机关节的当前位置
        std::lock_guard<std::mutex> lock(_data_mutex);
        _canData[0] = 0x92;
        _canData[1] = 0x00;
        _canData[2] = 0x00;
        _canData[3] = 0x00;
        _canData[4] = 0x00;
        _canData[5] = 0x00;
        _canData[6] = 0x00;
        _canData[7] = 0x00;
        canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    }

    // 该命令读取当前电机的温度、电压、转速、编码器位置。
    // 数据域 说明 数据
    // DATA[0] 命令字节 0x9C
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00

    // 驱动回复（1 帧）
    // 电机在收到命令后回复主机，该帧数据中包含了以下参数。
    // 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
    // 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
    // 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
    // 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
    // 数据域 说明 数据
    // DATA[0] 命令字节 0x9C
    // DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
    // DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
    // DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
    // DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
    // DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
    // DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
    // DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

    void ServoMotorJoint::readStatus2()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _canData[0] = 0x9C;
        _canData[1] = 0x00;
        _canData[2] = 0x00;
        _canData[3] = 0x00;
        _canData[4] = 0x00;
        _canData[5] = 0x00;
        _canData[6] = 0x00;
        _canData[7] = 0x00;
        canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    }

    //  电机停止命令 0x81
    // 停止电机，但不清除电机运行状态和之前接收的控制指令
    // 数据域 说明 数据
    // DATA[0] 命令字节 0x81
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00

    // 回复 电机在收到命令后回复主机，回复命令和接收命令一致

    void ServoMotorJoint::stop()
    {
        // 停止你的伺服电机关节
        std::lock_guard<std::mutex> lock(_data_mutex);
        _canData[0] = 0x81;
        _canData[1] = 0x00;
        _canData[2] = 0x00;
        _canData[3] = 0x00;
        _canData[4] = 0x00;
        _canData[5] = 0x00;
        _canData[6] = 0x00;
        _canData[7] = 0x00;
        canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    }
    //  电机运行命令 0x88
    // 从电机停止命令中恢复电机运行（恢复停止前的控制方式）
    // 数据域 说明 数据
    // DATA[0] 命令字节 0x88
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00

    // 回复 电机在收到命令后回复主机，回复命令和接收命令一致

    void ServoMotorJoint::start()
    {
        // 启动你的伺服电机关节
        std::lock_guard<std::mutex> lock(_data_mutex);
        _canData[0] = 0x88;
        _canData[1] = 0x00;
        _canData[2] = 0x00;
        _canData[3] = 0x00;
        _canData[4] = 0x00;
        _canData[5] = 0x00;
        _canData[6] = 0x00;
        _canData[7] = 0x00;
        canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    }
    //  电机关闭命令 0x80

    // 关闭电机，同时清除电机运行状态和之前接收的控制指令
    // 数据域 说明 数据
    // DATA[0] 命令字节 0x80
    // DATA[1] NULL 0x00
    // DATA[2] NULL 0x00
    // DATA[3] NULL 0x00
    // DATA[4] NULL 0x00
    // DATA[5] NULL 0x00
    // DATA[6] NULL 0x00
    // DATA[7] NULL 0x00

    // 回复 电机在收到命令后回复主机，回复命令和接收命令一致

    void ServoMotorJoint::close()
    {
        // 关闭你的伺服电机关节
        std::lock_guard<std::mutex> lock(_data_mutex);
        _canData[0] = 0x80;
        _canData[1] = 0x00;
        _canData[2] = 0x00;
        _canData[3] = 0x00;
        _canData[4] = 0x00;
        _canData[5] = 0x00;
        _canData[6] = 0x00;
        _canData[7] = 0x00;
        canDriver->sendMessage(sockcanpp::CanMessage(_id, std::string((char *)_canData, 8)));
    }
    void ServoMotorJoint::processCanMessage(uint8_t *data)
    {
        // 处理从CAN总线接收到的消息
        std::lock_guard<std::mutex> lock(_data_mutex);
        uint8_t cmd = data[0];
        switch (cmd)
        {
        //
        // case 0x30:  // 读取PID参数命令
        //     _pi[0].p = data[2];
        //     _pi[0].i = data[3];
        //     _pi[1].p = data[4];
        //     _pi[1].i = data[5];
        //     _pi[2].p = data[6];
        //     _pi[2].i = data[7];
        //     break;
        // case 0x31:  // 写入PID参数到RAM命令
        //     _pi[0].p = data[2];
        //     _pi[0].i = data[3];
        //     _pi[1].p = data[4];
        //     _pi[1].i = data[5];
        //     _pi[2].p = data[6];
        //     _pi[2].i = data[7];
        //     break;
        // case 0x32:  // 写入PID参数到ROM命令
        //     _pi[0].p = data[2];
        //     _pi[0].i = data[3];
        //     _pi[1].p = data[4];
        //     _pi[1].i = data[5];
        //     _pi[2].p = data[6];
        //     _pi[2].i = data[7];
        //     break;
        case 0x33: // 读取加速度命令
            // 32bit  7 6 5 4
            _acc = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
            break;
        case 0x34: // 写入加速度到RAM命令
            _acc = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
            break;
        case 0x90: // 读取编码器命令

            _encoder = data[2] | (data[3] << 8);      // 当前编码器值 （原始 - 电机零点）
            _encoder_i = data[4] | (data[5] << 8);    // 原始编码器值
            _encoder_zero = data[6] | (data[7] << 8); // 电机零点
            break;
        case 0x91: // 写入编码器值到ROM作为电机零点命令
            // _encoder = data[2] | (data[3] << 8);
            // _encoder_i = data[4] | (data[5] << 8);
            _encoder_zero = data[6] | (data[7] << 8);
            break;
        case 0x19: // 写入当前位置到ROM作为电机零点命令
            // _encoder = data[2] | (data[3] << 8);
            // _encoder_i = data[4] | (data[5] << 8);
            _encoder_zero = data[6] | (data[7] << 8);
            break;
        case 0x92: // 读取多圈角度命令
                   // int64
            //  angle2_0 = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24) ;
            //  angle2_1 = data[5] | (data[6] << 8) | (data[7] << 16) ;
            m_angle = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24) | ((int64_t)data[5] << 32) | ((int64_t)data[6] << 40) | ((int64_t)data[7] << 48);

            break;
        case 0x94: // 读取单圈角度命令
            // int 16
            angle = data[6] | (data[7] << 8);
            break;
        case 0x95: // 清除电机角度命令（设置电机初始位置）//官方暂未实现
            // _encoder = data[2] | (data[3] << 8);
            // _encoder_i = data[4] | (data[5] << 8);
            // _encoder_zero = data[6] | (data[7] << 8);
            break;
        case 0x96: // 读取电机温度命令
            temp = data[1];
            // 电压
            voltage = data[3] | (data[4] << 8);
            // 状态 7
            _status = data[7];
            break;
        case 0x9B: // 清除错误命令
            temp = data[1];
            // 电压
            voltage = data[3] | (data[4] << 8);
            // 状态 7
            _status = data[7];
            break;
        case 0x9C: // 读取电机状态2命令
            // 温度 1 电流 23 速度 45 编码器位置 67
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 电压
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0x9D: // 读取电机状态3命令
            break; // 命令复杂，不需要
        case 0x80: // 电机关闭
            break;
        case 0x81: // 电机停止
            break;
        case 0x82: // 电机运行
            break;
        case 0xA1: // 转矩闭环
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0xA2: // 速度闭环
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0xA3: // 位置闭环1
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0xA4: // 位置闭环2
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0xA5: // 位置闭环3
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0xA6: // 位置闭环4
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0xA7: // 位置闭环5
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;
        case 0xA8: // 位置闭环6
            // temp
            temp = data[1];
            // 电流
            current = data[2] | (data[3] << 8);
            // 速度
             speed = data[4] | (data[5] << 8);
            // 编码器
            _encoder = data[6] | (data[7] << 8);
            break;

        default:
            break;
        }
    }
} // namespace name
