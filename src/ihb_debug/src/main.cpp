#include <fstream>
#include <atomic>
#include <thread>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"

#include "ihb_msgs/msg/ihb_status.hpp"
#include "ihb_msgs/msg/muscle_joint_target.hpp"
#include "ihb_msgs/msg/servo_motor_joint_target.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_target.hpp"
#include "ihb_msgs/msg/arm_target.hpp"

#include "muscle_joint.h"
#include "servo_motorJoint.h"
#include "stepper_motor_joint.h"
#include "hall_sensors.h"
#include "statics_model.h"
#include <numeric> // 需要包含这个头文件来使用 std::all_of 和 std::accumulate


// #include "deep_motor_sdk.h"

// 系统状态枚举
enum class SystemState
{
    STOPPED, // 停止状态
    RUNNING, // 运行状态（PID 控制中）
    PAUSED   // 暂停状态
};

class IhbDebugNode : public rclcpp::Node
{
public:
    ihb_msgs::msg::IhbStatus message = ihb_msgs::msg::IhbStatus();
    IhbDebugNode() : Node("ihb_debug_node"), should_exit_(false), pid_running_(false),
                     system_state_(SystemState::STOPPED), can_initialized_(false)
    {
        RCLCPP_INFO(this->get_logger(), "========== IHB Debug Node 初始化开始 ==========");

        // 首先初始化 CAN 总线
        can_initialized_ = initCanBus();
        if (!can_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "⚠ CAN 总线初始化失败，程序将继续但可能无法与硬件通信！");
        }

        // 8 个肌肉关节 - 初始化 ID 和 PID 参数（参考 pid.cpp）
        RCLCPP_INFO(this->get_logger(), "初始化肌肉关节（带 PID 参数）...");

        // 设置 肌肉关节  换平台记得改

        muscle_joints[0].setId(0);
        muscle_joints[1].setId(1);
        muscle_joints[2].setId(3);
        muscle_joints[3].setId(4);

        // 设置 PID 参数（参考 pid.cpp 中的配置）
        muscle_joints[0].setPidParameters(30, 1.8, 120); // muscle1
        muscle_joints[1].setPidParameters(60, 5, 200);   // muscle2
        muscle_joints[2].setPidParameters(300, 10, 150); // muscle3
        muscle_joints[3].setPidParameters(250, 20, 200); // muscle4
        // 其余没使用

        // 初始化所有肌肉关节的 PID 启用状态为 false
        for (int i = 0; i < 8; i++)
        {
            muscle_joint_pid_enabled_[i].store(false);
        }

        RCLCPP_INFO(this->get_logger(), "✓ 8 个肌肉关节已初始化");

        // 启动霍尔传感器
        RCLCPP_INFO(this->get_logger(), "启动霍尔传感器...");
        hall.start();
        RCLCPP_INFO(this->get_logger(), "✓ 霍尔传感器已启动");
        // 32 个压力传感器（已禁用）
        // for (int i = 0; i < 32; i++)
        // {
        //     force_sensors[i].setId(i);
        // }

        // 4个伺服电机
        RCLCPP_INFO(this->get_logger(), "初始化伺服电机关节...");
        for (int i = 0; i < 4; i++)
        {
            servo_motor_joints[i].setId(i + 1);
        }
        RCLCPP_INFO(this->get_logger(), "✓ 4 个伺服电机关节已初始化");

        // 1个步进电机（已禁用）
        // for (int i = 0; i < 1; i++)
        // {
        //     // stepper_motor_joints[i].setId(i);
        // }

        // 创建发布者，最多缓存 10 条消息，超过丢弃
        ihb_status_publisher_ = this->create_publisher<ihb_msgs::msg::IhbStatus>("ihb_status", 10);
        RCLCPP_INFO(this->get_logger(), "✓ 状态发布者已创建");

        // 订阅muscle并打印
        ihb_muscle_joint_sub_ = this->create_subscription<ihb_msgs::msg::MuscleJointTarget>(
            "ihb_muscle_joint_target", 10, [this](const ihb_msgs::msg::MuscleJointTarget::SharedPtr msg)
            {
                // 验证 ID 范围
                if (msg->id < 0 || msg->id >= 8)
                {
                    RCLCPP_WARN(this->get_logger(), "⚠ 无效的肌肉关节 ID: %d（有效范围: 0-7）", msg->id);
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "muscle joint heard: ID=%d, CMD=%d, Angle=[%f,%f], Pressure=[%d,%d,%d,%d]",
                           msg->id, msg->cmd, msg->angle[0], msg->angle[1],
                           msg->pressure[0], msg->pressure[1], msg->pressure[2], msg->pressure[3]);

                // 使用已初始化的对象，而不是重复创建
                auto& muscle_joint = muscle_joints[msg->id];

                switch (msg->cmd)
                {
                case 0:
                    // 停止指令 - 停止单个肌肉关节的 PID 控制并归零气压
                    RCLCPP_INFO(this->get_logger(), "[Case 0 - Stop] 停止肌肉关节 %d", msg->id);
                    stopPidControl({static_cast<int>(msg->id)});
                    break;

                case 1:
                    // 开环气压指令
                    RCLCPP_INFO(this->get_logger(), "[Case 1 - Open Loop Pressure] ID: %d", msg->id);
                    muscle_joint.setPressure(msg->pressure[0], msg->pressure[1], msg->pressure[2], msg->pressure[3]);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    break;

                case 2:
                    // 角度闭环指令 PID（非阻塞式）
                    RCLCPP_INFO(this->get_logger(), "[Case 2 - Angle PID Non-Blocking] ID: %d, Target: [%f, %f]",
                               msg->id, msg->angle[0], msg->angle[1]);

                    // 设置目标角度（非阻塞，线程安全）
                    muscle_joint.setTargetAngle(msg->angle[0], msg->angle[1]);
                    muscle_joint.setPidParameters(msg->kp, msg->ki, msg->kd);

                    // 启用该肌肉关节的 PID 控制
                    muscle_joint_pid_enabled_[msg->id].store(true);

                    // 如果 PID 控制线程未运行，启动它
                    if (!pid_running_.load())
                    {
                        RCLCPP_INFO(this->get_logger(), "启动 PID 控制线程...");
                        startPidControl();
                    }

                    RCLCPP_INFO(this->get_logger(), "✓ 肌肉关节 %d 目标角度已设置并启用 PID，参数: kp=%.2f ki=%.2f kd=%.2f", msg->id, msg->kp, msg->ki, msg->kd);
                    break;

                case 3:
                    // 静力学前馈开环指令（已废弃 - 请使用整臂控制话题）
                    RCLCPP_WARN(this->get_logger(),
                               "[Case 3 - Deprecated] 静力学模型控制已迁移到整臂话题 /right_arm/target 或 /left_arm/target");
                    break;

                case 4:
                    // 静力学前馈闭环指令（已废弃 - 请使用整臂控制话题）
                    RCLCPP_WARN(this->get_logger(),
                               "[Case 4 - Deprecated] PID 控制已迁移到整臂话题 /right_arm/target 或 /left_arm/target");
                    break;

                case 5:
                    // 动力学前馈开环指令
                    RCLCPP_INFO(this->get_logger(), "[Case 5 - Dynamic Feedforward Open Loop] ID: %d", msg->id);
                    break;

                case 6:
                    // 动力学前馈闭环指令
                    RCLCPP_INFO(this->get_logger(), "[Case 6 - Dynamic Feedforward Closed Loop] ID: %d", msg->id);
                    break;

                default:
                    RCLCPP_WARN(this->get_logger(), "⚠ 未知的命令: %d", msg->cmd);
                    break;
                } });
        // 订阅servo并打印
        ihb_servo_motor_sub_ = this->create_subscription<ihb_msgs::msg::ServoMotorJointTarget>(
            "ihb_servo_motor_joint_target", 10, [this](const ihb_msgs::msg::ServoMotorJointTarget::SharedPtr msg)
            {
                // 验证 ID 范围（瓴控电机 ID 从 1 开始，数组索引从 0 开始）
                if (msg->id < 1 || msg->id > 4)
                {
                    RCLCPP_WARN(this->get_logger(), "⚠ 无效的伺服电机 ID: %d（有效范围: 1-4）", msg->id);
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "servo motor joint heard: ID=%d, CMD=%d, Angle=%f, SpeedLimit=%d",
                           msg->id, msg->cmd, msg->angle, msg->speed_limit);

                // 使用已初始化的对象，而不是重复创建
                // 注意：msg->id 是 1-4，数组索引是 0-3
                auto& servo_motor_joint = servo_motor_joints[msg->id - 1];

                switch (msg->cmd)
                {
                case 1:
                    // 开机
                    RCLCPP_INFO(this->get_logger(), "[Servo CMD 1 - Start] ID: %d", msg->id);
                    servo_motor_joint.start();
                    break;

                case 2:
                    // 关机
                    RCLCPP_INFO(this->get_logger(), "[Servo CMD 2 - Close] ID: %d", msg->id);
                    servo_motor_joint.close();
                    break;

                case 3:
                    // 运动指令
                    RCLCPP_INFO(this->get_logger(), "[Servo CMD 3 - Move] ID: %d, Angle: %f, Speed: %d",
                               msg->id, msg->angle, msg->speed_limit);
                    servo_motor_joint.setPositonWithSpeedLimit(msg->angle, msg->speed_limit);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    break;

                case 4:
                    // 写入零点到ROM（不建议频繁使用）
                    RCLCPP_WARN(this->get_logger(), "[Servo CMD 4 - Set Zero] ID: %d（已禁用）", msg->id);
                    // servo_motor_joint.setZero();
                    break;

                case 5:
                    // 读取电机温度、电压、转速、编码器位置
                    RCLCPP_INFO(this->get_logger(), "[Servo CMD 5 - Read Status] ID: %d", msg->id);
                    break;

                case 6:
                    // 读取温度、电流
                    RCLCPP_INFO(this->get_logger(), "[Servo CMD 6 - Read Temp/Current] ID: %d", msg->id);
                    break;

                default:
                    RCLCPP_WARN(this->get_logger(), "⚠ 未知的伺服电机命令: %d", msg->cmd);
                    break;
                } });
        // 订阅stepper并打印
        ihb_stepper_motor_sub_ = this->create_subscription<ihb_msgs::msg::StepperMotorJointTarget>(
            "ihb_stepper_motor_joint_target", 10, [this](const ihb_msgs::msg::StepperMotorJointTarget::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "stepper motor joint heard: '%d' '%f' ", msg->id, msg->height);
                // ihb::StepperMotorJoint stepper_motor_joint(msg->id);
                // todo 调试后针对每个数据进行判断
                //  stepper_motor_joint.setHeight(msg->height);
            });

        // 订阅右臂整臂控制话题
        right_arm_target_sub_ = this->create_subscription<ihb_msgs::msg::ArmTarget>(
            "/right_arm/target", 10, std::bind(&IhbDebugNode::rightArmTargetCallback, this, std::placeholders::_1));

        // 订阅左臂整臂控制话题
        left_arm_target_sub_ = this->create_subscription<ihb_msgs::msg::ArmTarget>(
            "/left_arm/target", 10, std::bind(&IhbDebugNode::leftArmTargetCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "✓ 整臂控制订阅器已创建 (/right_arm/target, /left_arm/target)");

        // 创建定时器，发布数据
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IhbDebugNode::publisher_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "========== ✓ IHB Debug Node 初始化完成 ==========");
    }

    /**
     * @brief 检查 CAN 总线状态
     * @param interface CAN 接口名称（默认 "can1"）
     * @return true 如果 CAN 接口已启动，false 否则
     */
    bool checkCanStatus(const std::string &interface = "can1")
    {
        // 检查接口是否存在
        std::string path = "/sys/class/net/" + interface + "/operstate";
        std::ifstream statusFile(path);
        if (!statusFile.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "CAN 接口 %s 不存在！", interface.c_str());
            return false;
        }

        // 读取接口状态
        std::string status;
        std::getline(statusFile, status);
        statusFile.close();

        // 检查状态是否为 "up"
        if (status == "up")
        {
            RCLCPP_INFO(this->get_logger(), "CAN 接口 %s 已经启动（状态: %s）", interface.c_str(), status.c_str());
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "CAN 接口 %s 状态: %s（未启动）", interface.c_str(), status.c_str());
            return false;
        }
    }

    /**
     * @brief 初始化 CAN 总线
     * @return true 如果初始化成功，false 否则
     */
    bool initCanBus()
    {
        RCLCPP_INFO(this->get_logger(), "========== CAN 总线初始化开始 ==========");

        // 首先检查 CAN 总线状态
        if (checkCanStatus("can1"))
        {
            RCLCPP_INFO(this->get_logger(), "CAN 总线已经启动，无需重新初始化");
            return true;
        }

        int ret = 0;

        RCLCPP_INFO(this->get_logger(), "开始配置 CAN 总线...");

        // 设置 CAN 总线比特率为 1Mbps
        RCLCPP_INFO(this->get_logger(), "步骤 1/2: 设置 CAN 总线比特率为 1Mbps...");
        ret = system("sudo ip link set can1 type can bitrate 1000000");
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 设置 CAN 总线比特率失败！返回码: %d", ret);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "✓ CAN 总线比特率设置成功");

        // 等待配置生效
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 启动 CAN 总线
        RCLCPP_INFO(this->get_logger(), "步骤 2/2: 启动 CAN 总线接口...");
        ret = system("sudo ip link set can1 up");
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 启动 CAN 总线失败！返回码: %d", ret);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "✓ CAN 总线接口启动成功");

        // 等待 CAN 总线完全启动
        RCLCPP_INFO(this->get_logger(), "等待 CAN 总线完全启动（5秒）...");
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 再次检查状态确认
        if (checkCanStatus("can1"))
        {
            RCLCPP_INFO(this->get_logger(), "========== ✓ CAN 总线初始化成功 ==========");
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "========== ❌ CAN 总线初始化失败（状态检查未通过） ==========");
            return false;
        }
    }

    void publisher_timer_callback() // 反馈当前硬件状态
    {
        ihb_msgs::msg::IhbStatus message = ihb_msgs::msg::IhbStatus();

        // ========== 读取伺服电机角度 ==========
        // 伺服电机关节 id angle current speed temperature
        for (int i = 0; i < 4; i++)
        {
            message.servo_motor[i].id = i + 1; // 伺服电机 ID 从 1 开始
            // 读取电机角度（MAngle 方法内部有互斥锁保护，线程安全）
            message.servo_motor[i].angle = servo_motor_joints[i].MAngle();
            // 可选：读取其他状态信息
            // message.servo_motor[i].current = servo_motor_joints[i].Current();
            // message.servo_motor[i].speed = servo_motor_joints[i].Speed();
            // message.servo_motor[i].temp = servo_motor_joints[i].Temp();
        }

        // ========== 读取霍尔传感器角度并填充到肌肉关节 ==========
        // 获取所有霍尔传感器角度（getAllAngles 内部有互斥锁保护，线程安全）
        std::vector<float> hall_angles_raw = hall.getAllAngles();

        // 根据 pidControlLoop 中的映射关系，将霍尔角度映射到肌肉关节
        // 肌肉关节 0: hall_angles[0] = -angles[11], hall_angles[1] = angles[3]
        // 肌肉关节 1: hall_angles[2] = -angles[10], hall_angles[3] = angles[2]
        // 肌肉关节 2: hall_angles[4] = -angles[9],  hall_angles[5] = angles[1]
        // 肌肉关节 3: hall_angles[6] = -angles[8],  hall_angles[7] = angles[0]

        // 肌肉关节 id angle pressure stiffness
        for (int i = 0; i < 8; i++)
        {
            message.muscle_joint[i].id = muscle_joints[i].id();

            // 读取气压和刚度（这些方法内部有互斥锁保护，线程安全）
            muscle_joints[i].getPressure(
                message.muscle_joint[i].pressure[0],
                message.muscle_joint[i].pressure[1],
                message.muscle_joint[i].pressure[2],
                message.muscle_joint[i].pressure[3]);
            muscle_joints[i].getStiffness(
                message.muscle_joint[i].stiffness[0],
                message.muscle_joint[i].stiffness[1]);
        }

        // 填充前 4 个肌肉关节的角度（从霍尔传感器读取）
        // 肌肉关节 0
        message.muscle_joint[0].angle[0] = -hall_angles_raw[11];
        message.muscle_joint[0].angle[1] = hall_angles_raw[3];

        // 肌肉关节 1
        message.muscle_joint[1].angle[0] = -hall_angles_raw[10];
        message.muscle_joint[1].angle[1] = hall_angles_raw[2];

        // 肌肉关节 2
        message.muscle_joint[2].angle[0] = -hall_angles_raw[9];
        message.muscle_joint[2].angle[1] = hall_angles_raw[1];

        // 肌肉关节 3
        message.muscle_joint[3].angle[0] = -hall_angles_raw[8];
        message.muscle_joint[3].angle[1] = hall_angles_raw[0];

        // 其余肌肉关节（4-7）角度设为 0（未使用）
        for (int i = 4; i < 8; i++)
        {
            message.muscle_joint[i].angle[0] = 0.0f;
            message.muscle_joint[i].angle[1] = 0.0f;
        }

        // 发布状态消息
        ihb_status_publisher_->publish(message);
    }

    void processCanMessage()
    {
        try
        {
            sockcanpp::CanDriver canDriver("can1", CAN_RAW);

            // 关闭 CAN 回环模式（参考 pid.cpp）
            int loopback = 0;
            setsockopt(canDriver.getSocketFd(), SOL_CAN_RAW, CAN_RAW_LOOPBACK,
                       &loopback, sizeof(loopback));

            RCLCPP_INFO(this->get_logger(), "✓ CAN 消息处理线程已启动（接口: can1，回环模式: 关闭）");

            uint8_t data[8] = {0};

            while (!should_exit_.load())
            {
                if (canDriver.waitForMessages(std::chrono::milliseconds(1)))
                {
                    // 读取 CAN 消息
                    sockcanpp::CanMessage receivedMessage = canDriver.readMessage();

                    // 提取数据
                    for (int i = 0; i < receivedMessage.getFrameData().size(); i++)
                    {
                        data[i] = receivedMessage.getFrameData()[i];
                    }

                    uint16_t canId = uint16_t(receivedMessage.getCanId());

                    // 处理伺服电机 CAN 消息（使用精确 ID 匹配，参考 pid.cpp）
                    if (canId == 0x141)
                    {
                        servo_motor_joints[0].processCanMessage(data);
                    }
                    else if (canId == 0x142)
                    {
                        servo_motor_joints[1].processCanMessage(data);
                    }
                    else if (canId == 0x143)
                    {
                        servo_motor_joints[2].processCanMessage(data);
                    }
                    else if (canId == 0x144)
                    {
                        servo_motor_joints[3].processCanMessage(data);
                    }
                    // 使用 ID 高位匹配其他设备（保留原有逻辑）
                    else
                    {
                        switch (canId >> 4)
                        {
                        case 0x10: // 气阀板返回数据
                        {
                            // todo
                        }
                        break;
                        case 0x20: // 步进电机板返回数据
                        {
                            // stepper_motor_joints[canId & 0x0F].processCanMessage(data);
                        }
                        break;
                        case 0x30: // 压力传感器板返回数据
                        {
                            // 压力传感器处理逻辑（已注释）
                        }
                        break;
                        case 0x18: // 肌肉板返回数据
                        {
                            muscle_joints[canId & 0x0F].processCanMessage(data);
                            // 调试日志（可选）
                            // RCLCPP_DEBUG(this->get_logger(), "Muscle joint heard: %d '%d' '%d' '%d'",
                            //             canId, muscle_joints[canId & 0x0F].id(),
                            //             muscle_joints[canId & 0x0F].Angle1(),
                            //             muscle_joints[canId & 0x0F].Angle2());
                        }
                        break;

                        // 云深处电机相关消息
                        case 0x5: // ENABLE
                        {
                            RCLCPP_INFO(this->get_logger(), "电机 ENABLE 成功");
                        }
                        break;
                        case 0x3: // DISABLE
                        {
                            RCLCPP_INFO(this->get_logger(), "电机 DISABLE 成功");
                        }
                        break;
                        case 0x2F: // CHECK ONLINE
                        {
                            // 在线检查
                        }
                        break;
                        case 0x9: // 电机返回数据
                        {
                            std::string frameData = receivedMessage.getFrameData();
                            uint64_t data_64 = 0;
                            std::memcpy(&data_64, frameData.data(), frameData.size());
                            // servo_motor_joints[(canId & 0x0F) - 1].processDEEPCanMessage(data_64);
                        }
                        break;
                        default:
                            // 未知 CAN ID
                            break;
                        }
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "⚠ CAN 接口初始化失败: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "  程序将继续运行（CAN 消息处理线程空转）");

            // CAN 接口不可用时，线程空转等待退出信号
            while (!should_exit_.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        RCLCPP_INFO(this->get_logger(), "✓ CAN 消息处理线程已退出");
    }

    void writeSaveLog()
    { // 保存日志 同时 保证终端的整洁 每10ms运行一次
      // 日志流
      // info log
        RCLCPP_INFO(this->get_logger(), "Log writing thread started");
        std::time_t now = std::time(nullptr);
        std::tm *ltm = std::localtime(&now);
        // 创建带有时间戳文件名的日志
        std::stringstream ss;
        ss << "log/ihblog_" << std::put_time(ltm, "%Y%m%d_%H%M%S") << ".txt";
        std::ofstream logFile(ss.str(), std::ios::app);

        std::stringstream logBuffer;
        auto start = std::chrono::system_clock::now();
        while (!should_exit_.load())
        {
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            auto value = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_seconds);

            // send
            for (int i = 0; i < 4; i++)
            {
                // RCLCPP_INFO(this->get_logger(), "Writing log START2");
                servo_motor_joints[i].readAngle();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                servo_motor_joints[i].readStatus2();
            }

            logBuffer << "Time\t" << value.count() << "\t";
            // 电机
            for (auto &motor : servo_motor_joints)
            {
                logBuffer << motor.MAngle() << "\t" << motor.Speed() << "\t";
            }
            // 压力传感器

            // 优化：使用线程安全的批量读取接口
            std::vector<float> hall_angles = hall.getAllAngles();
            for (size_t i = 0; i < 16; i++)
            {
                logBuffer << hall_angles[i] << "\t";
                // RCLCPP_DEBUG(this->get_logger(), "hall.getAllAngles()[%d] = %f", i, hall_angles[i]);
            }
            // for (size_t i = 0; i < 32; i++)
            // {
            //     logBuffer << force_sensors.getValue(i) << "\t";
            // }

            // debug log
            // RCLCPP_INFO(this->get_logger(), logBuffer.str().c_str());
            // 打开日志文件 time name day
            // std::string  logFileName = "ihb_debug_" + std::to_string(time.tm_year + 1900) + "-" + std::to_string(time.tm_mon + 1) + "-" + std::to_string(time.tm_mday) + ".log";

            // std::ofstream logFile("ihb_debug18.log", std::ios::app);
            if (logFile.is_open())
            {
                logFile << logBuffer.str() << std::endl;
                logFile.close();
            }
            logBuffer.str("");
            // delay
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // 关闭日志文件
        if (logFile.is_open())
        {
            logFile.close();
        }
        RCLCPP_INFO(this->get_logger(), "Log writing thread exiting");
        // 时间戳 毫秒

        // 日志格式 为便于excel使用，全部使用制表符
        // 时间戳ms 电机1角度 电机1转矩 电机1速度
    }

    /**
     * @brief PID 控制线程（参考 pid.cpp）
     * @details 在独立线程中运行，周期性读取传感器角度并执行 PID 控制
     */
    void pidControlLoop()
    {
        RCLCPP_INFO(this->get_logger(), "✓ PID 控制线程已启动（合并角度读取和 PID 控制）");

        while (!should_exit_.load() && pid_running_.load())
        {
            // ========== 步骤 1: 读取伺服电机角度 ==========
            servo_motor_joints[0].readAngle();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            servo_motor_joints[0].readStatus2();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            servo_motor_joints[1].readAngle();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            servo_motor_joints[1].readStatus2();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // ========== 步骤 2: 读取霍尔传感器角度 ==========
            // 参考 pid.cpp 中的角度映射关系
            float hall_angles[8];
            std::vector<float> angles = hall.getAllAngles();
            hall_angles[0] = -angles[11];
            hall_angles[1] = angles[3];
            hall_angles[2] = -angles[10];
            hall_angles[3] = angles[2];
            hall_angles[4] = -angles[9];
            hall_angles[5] = angles[1];
            hall_angles[6] = -angles[8];
            hall_angles[7] = angles[0];

            // ========== 步骤 3: 更新角度数据到 MuscleJoint 对象 ==========
            muscle_joints[0].processAngle(hall_angles[0], hall_angles[1]);
            muscle_joints[1].processAngle(hall_angles[2], hall_angles[3]);
            muscle_joints[2].processAngle(hall_angles[4], hall_angles[5]);
            muscle_joints[3].processAngle(hall_angles[6], hall_angles[7]);

            // ========== 步骤 4: 执行 PID 计算和气压输出（只更新启用了 PID 的肌肉关节）==========
            for (int i = 0; i < 4; i++)
            {
                if (muscle_joint_pid_enabled_[i].load())
                {
                    muscle_joints[i].update();
                }
            }

            // ========== 步骤 5: 控制周期延时 ==========
            // 延迟 24ms，总周期约 30ms（4ms 伺服电机读取 + PID 计算 + 24ms 延时）
            std::this_thread::sleep_for(std::chrono::milliseconds(24));
        }

        // 线程停止时，将所有气压输出归零
        RCLCPP_INFO(this->get_logger(), "✓ PID 控制线程正在停止，归零所有气压输出...");
        for (int i = 0; i < 8; i++)
        {
            muscle_joints[i].setPressure(0, 0, 0, 0);
        }

        RCLCPP_INFO(this->get_logger(), "✓ PID 控制线程已退出");
    }

    /**
     * @brief 启动 PID 控制
     */
    void startPidControl()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (pid_running_.load())
        {
            RCLCPP_WARN(this->get_logger(), "⚠ PID 控制已经在运行中");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "========== 启动 PID 控制 ==========");

        // 设置运行标志
        pid_running_.store(true);
        system_state_ = SystemState::RUNNING;

        // 启动 PID 线程
        pid_thread_ = std::thread([this]()
                                  { this->pidControlLoop(); });

        RCLCPP_INFO(this->get_logger(), "✓ PID 控制已启动");
    }

    /**
     * @brief 停止 PID 控制
     * @param muscle_ids 要停止的肌肉关节 ID 列表（空表示停止所有）
     */
    void stopPidControl(const std::vector<int>& muscle_ids = {})
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 如果列表为空，停止所有肌肉关节
        std::vector<int> ids_to_stop = muscle_ids;
        if (ids_to_stop.empty())
        {
            // 停止所有肌肉关节（0-7）
            for (int i = 0; i < 8; i++)
            {
                ids_to_stop.push_back(i);
            }
            RCLCPP_INFO(this->get_logger(), "========== 停止所有 PID 控制 ==========");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "========== 停止部分肌肉关节 PID 控制 ==========");
        }

        // 禁用指定肌肉关节的 PID
        for (int id : ids_to_stop)
        {
            if (id >= 0 && id < 8)
            {
                muscle_joint_pid_enabled_[id].store(false);
                RCLCPP_INFO(this->get_logger(), "禁用肌肉关节 %d 的 PID 控制", id);
            }
        }

        // 检查是否所有肌肉关节的 PID 都已禁用
        bool all_disabled = true;
        for (int i = 0; i < 8; i++)
        {
            if (muscle_joint_pid_enabled_[i].load())
            {
                all_disabled = false;
                break;
            }
        }

        // 如果所有肌肉关节的 PID 都已禁用，停止 PID 线程
        if (all_disabled && pid_running_.load())
        {
            RCLCPP_INFO(this->get_logger(), "所有肌肉关节 PID 已禁用，停止 PID 线程...");

            // 设置停止标志
            pid_running_.store(false);
            system_state_ = SystemState::STOPPED;

            // 等待 PID 线程退出
            if (pid_thread_.joinable())
            {
                RCLCPP_INFO(this->get_logger(), "等待 PID 线程退出...");
                pid_thread_.join();
                RCLCPP_INFO(this->get_logger(), "✓ PID 线程已退出");
            }
        }

        // 归零指定肌肉关节的气压输出（安全措施）
        RCLCPP_INFO(this->get_logger(), "归零指定肌肉关节的气压输出...");
        for (int id : ids_to_stop)
        {
            if (id >= 0 && id < 8)
            {
                muscle_joints[id].setPressure(0, 0, 0, 0);
            }
        }

        // 如果停止所有，也归零伺服电机位置
        if (muscle_ids.empty())
        {
            RCLCPP_INFO(this->get_logger(), "归零伺服电机位置...");
            for (int i = 0; i < 4; i++)
            {
                servo_motor_joints[i].setPositonWithSpeedLimit(0, 120);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        RCLCPP_INFO(this->get_logger(), "✓ PID 控制已停止");
    }

    /**
     * @brief 右臂整臂控制回调函数
     * @param msg 整臂目标消息
     */
    void rightArmTargetCallback(const ihb_msgs::msg::ArmTarget::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "========== 右臂整臂控制 ==========");
        RCLCPP_INFO(this->get_logger(), "控制模式: %d (0=关闭PID, 1=静力学模型, 2=PID控制)", msg->cmd);

        processArmTarget(msg);
    }

    /**
     * @brief 左臂整臂控制回调函数
     * @param msg 整臂目标消息
     */
    void leftArmTargetCallback(const ihb_msgs::msg::ArmTarget::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "========== 左臂整臂控制 ==========");
        RCLCPP_INFO(this->get_logger(), "控制模式: %d (0=关闭PID, 1=静力学模型, 2=PID控制)", msg->cmd);

        processArmTarget(msg);
    }

    /**
     * @brief 处理整臂控制逻辑
     * @param msg 整臂目标消息
     */
    void processArmTarget(const ihb_msgs::msg::ArmTarget::SharedPtr msg)
    {
        switch (msg->cmd)
        {
        case 0:
            // 关闭 PID 控制
            RCLCPP_INFO(this->get_logger(), "[整臂 CMD 0] 关闭 PID 控制");
            stopPidControl();
            break;

        case 1:
            // 静力学模型控制
            RCLCPP_INFO(this->get_logger(), "[整臂 CMD 1] 静力学模型控制");
            applyStaticsModel(msg);
            break;

        case 2:
            // PID 控制
            RCLCPP_INFO(this->get_logger(), "[整臂 CMD 2] PID 控制");
            applyPidControl(msg);
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "⚠ 未知的整臂控制命令: %d", msg->cmd);
            break;
        }
    }

    /**
     * @brief 应用静力学模型控制
     * @param msg 整臂目标消息
     */
    void applyStaticsModel(const ihb_msgs::msg::ArmTarget::SharedPtr msg)
    {
        // 1. 先控制伺服电机移动到目标位置（直接使用消息中的 ID）
        for (int i = 0; i < 2; i++)
        {
            int servo_id = msg->servo_joints[i].id;
            RCLCPP_INFO(this->get_logger(), "设置伺服电机 ID=%d, 角度=%f, 速度限制=%d",
                        servo_id, msg->servo_joints[i].angle, msg->servo_joints[i].speed_limit);

            servo_motor_joints[servo_id - 1].setPositonWithSpeedLimit(
                msg->servo_joints[i].angle,
                msg->servo_joints[i].speed_limit);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // 2. 构建 10 个关节角度数组
        std::array<float, 10> joint_angles;

        // 从消息中提取角度数据
        // servo_joints[0] 和 servo_joints[1] 提供 2 个角度
        // muscle_joints[0-3] 每个提供 2 个角度，共 8 个角度
        joint_angles[0] = static_cast<float>(msg->servo_joints[0].angle) / 100.0f; // 伺服电机角度需要除以100
        joint_angles[1] = static_cast<float>(msg->servo_joints[1].angle) / 100.0f;

        for (int i = 0; i < 4; i++)
        {
            joint_angles[2 + i * 2] = static_cast<float>(msg->muscle_joints[i].angle[0]);
            joint_angles[3 + i * 2] = static_cast<float>(msg->muscle_joints[i].angle[1]);
        }

        // --- 新增功能：检查所有角度是否都为零 ---
        // 使用 std::all_of 算法，如果数组中所有元素都满足 lambda 表达式 (== 0.0f)，则返回 true。
        // 这是一个非常高效且可读性强的写法。
        bool all_angles_are_zero = std::all_of(joint_angles.cbegin(), joint_angles.cend(), [](float angle)
                                               { return angle == 0.0f; });

        // 如果所有角度都为零
        if (all_angles_are_zero)
        {
            RCLCPP_INFO(this->get_logger(), "所有目标角度为零，直接设置气压为0，跳过静力学计算。");

            // 遍历所有相关的肌肉关节，并将它们的气压设置为0
            for (int i = 0; i < 4; i++)
            {
                int muscle_id = msg->muscle_joints[i].id;
                muscle_joints[muscle_id].setPressure(0, 0, 0, 0);
            }

            // 提前返回，不再执行后续的计算 因为静力学模型计算会有一点点输出 既然全是0 直接全部归零就好了
            return;
        }
        // 3. 调用静力学模型计算气压值
        std::array<float, 16> muscle_pressures;
        bool success = statics_model_.computeGimbalMusclePressures(joint_angles, muscle_pressures);

        if (success)
        {
            // 4. 将气压值应用到 4 个肌肉关节（直接使用消息中的 ID）
            for (int i = 0; i < 4; i++)
            {
                int muscle_id = msg->muscle_joints[i].id;
                int pressure_base = i * 4;

                RCLCPP_INFO(this->get_logger(), "设置肌肉关节 ID=%d, 气压=[%d, %d, %d, %d]",
                            muscle_id,
                            static_cast<uint16_t>(muscle_pressures[pressure_base + 0]),
                            static_cast<uint16_t>(muscle_pressures[pressure_base + 1]),
                            static_cast<uint16_t>(muscle_pressures[pressure_base + 2]),
                            static_cast<uint16_t>(muscle_pressures[pressure_base + 3]));

                muscle_joints[muscle_id].setPressure(
                    static_cast<uint16_t>(muscle_pressures[pressure_base + 0]),
                    static_cast<uint16_t>(muscle_pressures[pressure_base + 1]),
                    static_cast<uint16_t>(muscle_pressures[pressure_base + 2]),
                    static_cast<uint16_t>(muscle_pressures[pressure_base + 3]));
            }

            RCLCPP_INFO(this->get_logger(), "✓ 静力学模型计算成功，伺服电机和气压值已应用");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 静力学模型计算失败！");
        }
    }

    /**
     * @brief 应用 PID 控制
     * @param msg 整臂目标消息
     */
    void applyPidControl(const ihb_msgs::msg::ArmTarget::SharedPtr msg)
    {
        // 1. 构建 10 个关节角度数组
        std::array<float, 10> joint_angles;

        // 从消息中提取角度数据
        // servo_joints[0] 和 servo_joints[1] 提供 2 个角度
        // muscle_joints[0-3] 每个提供 2 个角度，共 8 个角度
        joint_angles[0] = static_cast<float>(msg->servo_joints[0].angle) / 100.0f; // 伺服电机角度需要除以100
        joint_angles[1] = static_cast<float>(msg->servo_joints[1].angle) / 100.0f;

        for (int i = 0; i < 4; i++)
        {
            joint_angles[2 + i * 2] = static_cast<float>(msg->muscle_joints[i].angle[0]);
            joint_angles[3 + i * 2] = static_cast<float>(msg->muscle_joints[i].angle[1]);
        }

        // 检查所有角度是否都为零
        bool all_angles_are_zero = std::all_of(joint_angles.cbegin(), joint_angles.cend(), [](float angle)
        { return angle == 0.0f; });

        // 如果所有角度都为零，停止对应肌肉关节的 PID 控制
        if (all_angles_are_zero)
        {
            RCLCPP_INFO(this->get_logger(), "所有目标角度为零，停止对应肌肉关节的 PID 控制");

            // 收集要停止的肌肉关节 ID
            std::vector<int> muscle_ids_to_stop;
            for (int i = 0; i < 4; i++)
            {
                muscle_ids_to_stop.push_back(msg->muscle_joints[i].id);
            }

            // 停止对应肌肉关节的 PID 控制
            stopPidControl(muscle_ids_to_stop);

            // 提前返回，不再执行后续的计算
            return;
        }

        // 2. 设置伺服电机目标位置（直接使用消息中的 ID）
        for (int i = 0; i < 2; i++)
        {
            int servo_id = msg->servo_joints[i].id;
            RCLCPP_INFO(this->get_logger(), "设置伺服电机 ID=%d, 角度=%f, 速度限制=%d",
                        servo_id, msg->servo_joints[i].angle, msg->servo_joints[i].speed_limit);

            servo_motor_joints[servo_id - 1].setPositonWithSpeedLimit(
                msg->servo_joints[i].angle,
                msg->servo_joints[i].speed_limit);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // 3. 设置肌肉关节目标角度并启用 PID（直接使用消息中的 ID）
        for (int i = 0; i < 4; i++)
        {
            int muscle_id = msg->muscle_joints[i].id;

            RCLCPP_INFO(this->get_logger(), "设置肌肉关节 ID=%d, 目标角度=[%.2f, %.2f]",
                        muscle_id, msg->muscle_joints[i].angle[0], msg->muscle_joints[i].angle[1]);

            muscle_joints[muscle_id].setTargetAngle(
                msg->muscle_joints[i].angle[0],
                msg->muscle_joints[i].angle[1]);

            // 启用该肌肉关节的 PID 控制
            muscle_joint_pid_enabled_[muscle_id].store(true);
        }

        // 4. 如果 PID 线程未运行，启动它
        if (!pid_running_.load())
        {
            startPidControl();
        }

        RCLCPP_INFO(this->get_logger(), "✓ PID 控制目标已设置");
    }

    // 析构函数：确保线程正确退出
    ~IhbDebugNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down IhbDebugNode...");

        // 首先停止 PID 控制
        if (pid_running_.load())
        {
            stopPidControl();
        }

        // 设置退出标志
        should_exit_.store(true);

        // 等待线程结束
        if (can_thread_.joinable())
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for CAN thread to finish...");
            can_thread_.join();
        }
        if (log_thread_.joinable())
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for log thread to finish...");
            log_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "IhbDebugNode shutdown complete");
    }

    // 启动后台线程的方法
    void startBackgroundThreads()
    {
        RCLCPP_INFO(this->get_logger(), "========== 启动后台线程 ==========");

        // 创建 CAN 消息处理线程
        RCLCPP_INFO(this->get_logger(), "启动 CAN 消息处理线程...");
        can_thread_ = std::thread([this]()
                                  { this->processCanMessage(); });

        if (!can_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "⚠ 注意：CAN 总线未成功初始化，CAN 消息处理线程将空转");
        }

        // 创建日志保存线程（当前已禁用）
        // RCLCPP_INFO(this->get_logger(), "启动日志保存线程...");
        // log_thread_ = std::thread([this]() {
        //     this->writeSaveLog();
        // });

        RCLCPP_INFO(this->get_logger(), "========== ✓ 后台线程启动完成 ==========");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    uint16_t pub_num = 0;
    rclcpp::Publisher<ihb_msgs::msg::IhbStatus>::SharedPtr ihb_status_publisher_;
    // sub Muscle
    rclcpp::Subscription<ihb_msgs::msg::MuscleJointTarget>::SharedPtr ihb_muscle_joint_sub_;
    // sub Servo
    rclcpp::Subscription<ihb_msgs::msg::ServoMotorJointTarget>::SharedPtr ihb_servo_motor_sub_;
    // sub Stepper
    rclcpp::Subscription<ihb_msgs::msg::StepperMotorJointTarget>::SharedPtr ihb_stepper_motor_sub_;
    // sub Right Arm Target (整臂控制)
    rclcpp::Subscription<ihb_msgs::msg::ArmTarget>::SharedPtr right_arm_target_sub_;
    // sub Left Arm Target (整臂控制)
    rclcpp::Subscription<ihb_msgs::msg::ArmTarget>::SharedPtr left_arm_target_sub_;

    // 8 个肌肉关节 使用vector
    ihb::MuscleJoint muscle_joints[8];
    // 4个伺服电机
    ihb::ServoMotorJoint servo_motor_joints[4];
    // 1个步进电机
    ihb::StepperMotorJoint stepper_motor_joints[1];
    // 16个霍尔传感器
    ihb::HallSensors hall;

    // 静力学模型
    ihb::StaticsModel statics_model_;

    // 线程管理
    std::thread can_thread_;
    std::thread log_thread_;
    std::thread pid_thread_;        // PID 控制线程
    std::atomic<bool> should_exit_; // 线程退出标志
    std::atomic<bool> pid_running_; // PID 线程运行标志
    SystemState system_state_;      // 系统状态
    std::mutex state_mutex_;        // 状态互斥锁

    // 每个肌肉关节的 PID 启用状态（支持独立控制左右臂）
    std::array<std::atomic<bool>, 8> muscle_joint_pid_enabled_;

    // CAN 总线状态
    bool can_initialized_; // CAN 总线是否成功初始化
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IhbDebugNode>();

    RCLCPP_INFO(node->get_logger(), "Ihb_debug node has been started.");

    // 启动后台线程（使用成员方法管理线程生命周期）
    node->startBackgroundThreads();

    // 根据命令行参数调整日志等级

    // 主线程进入 ROS2 事件循环
    rclcpp::spin(node);

    // 清理资源（析构函数会自动处理线程的 join）
    RCLCPP_INFO(node->get_logger(), "Shutting down node...");
    rclcpp::shutdown();
    return 0;
}