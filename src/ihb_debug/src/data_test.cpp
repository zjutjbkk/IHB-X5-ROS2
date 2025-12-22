#include "rclcpp/rclcpp.hpp"
#include "ihb_msgs/msg/ihb_status.hpp"
#include "ihb_msgs/msg/muscle_joint_target.hpp"
#include "ihb_msgs/msg/servo_motor_joint_target.hpp"
#include "ihb_msgs/msg/stepper_motor_joint_target.hpp"

#include <random>

std::default_random_engine generator;
std::uniform_int_distribution<int> distribution(1, 100);

// #include "rclcpp/node.hpp"
// 自定义节点类
class IhbDebugNode : public rclcpp::Node
{
public:
    IhbDebugNode() : Node("ihb_debug_node")
    {
        ihb_status_publisher_ = this->create_publisher<ihb_msgs::msg::IhbStatus>("ihb_status", 10);
        // 生成随机数据并定时发送 频率为100Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IhbDebugNode::publisher_timer_callback, this));
        // 订阅muscle并打印
        ihb_muscle_joint_sub_ = this->create_subscription<ihb_msgs::msg::MuscleJointTarget>(
            "ihb_muscle_joint_target", 10, [this](const ihb_msgs::msg::MuscleJointTarget::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "muscle joint heard: '%d' '%d' '%d' '%d'", msg->id, msg->angle[0], msg->pressure[0], msg->stiffness[0]);
            });
        // 订阅servo并打印
        ihb_servo_motor_sub_ = this->create_subscription<ihb_msgs::msg::ServoMotorJointTarget>(
            "ihb_servo_motor_joint_target", 10, [this](const ihb_msgs::msg::ServoMotorJointTarget::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "servo motor joint heard:  '%d' '%d' '%d' ", msg->id, msg->angle, msg->speed_limit);
            });
        // 订阅stepper并打印
        ihb_stepper_motor_sub_ = this->create_subscription<ihb_msgs::msg::StepperMotorJointTarget>(
            "ihb_stepper_motor_joint_target", 10, [this](const ihb_msgs::msg::StepperMotorJointTarget::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "stepper motor joint heard: '%d' '%d' ", msg->id, msg->height);
            });
    }

private:
    void publisher_timer_callback()
    {
        ihb_msgs::msg::IhbStatus message = ihb_msgs::msg::IhbStatus();
        // 生成一个随机数
        int random_number = distribution(generator);

        // message.header.stamp = this->now();
        // message.header.frame_id = "ihb——test";
        // 步进电机关节 id height
        message.stepper_motor.id = 1;
        message.stepper_motor.height = 2 * random_number;
        // 伺服电机关节 id angle current speed temperature
        for (int i = 0; i < 4; i++)
        {
            random_number = distribution(generator);
            message.servo_motor[i].id = i;
            message.servo_motor[i].angle = random_number;
            message.servo_motor[i].current = (int16_t)(random_number * 1.2);
            message.servo_motor[i].speed = (int16_t)random_number * 1.5;
            message.servo_motor[i].temp = (uint8_t)random_number * 1.8;
        }
        // 肌肉关节 id angle pressure stiffness
        for (int i = 0; i < 8; i++)
        {
            random_number = distribution(generator);
            message.muscle_joint[i].id = i;
            message.muscle_joint[i].angle[0] = random_number / 2;
            message.muscle_joint[i].angle[1] = random_number / 3;
            message.muscle_joint[i].pressure[0] = random_number * 1.9;
            message.muscle_joint[i].pressure[1] = random_number * 1.8;
            message.muscle_joint[i].pressure[2] = random_number * 1.7;
            message.muscle_joint[i].pressure[3] = random_number * 1.6;
            message.muscle_joint[i].stiffness[0] = random_number * 0.7;
            message.muscle_joint[i].stiffness[1] = random_number * 0.8;
        }
        // force sensor
        for (int i = 0; i < 32; i++)
        {
            random_number = distribution(generator);
            message.force_sensor[i].id = i;
            message.force_sensor[i].force = random_number/100.0*4095;
        }

        ihb_status_publisher_->publish(message);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'",pub_num++);
    }



    rclcpp::TimerBase::SharedPtr timer_;
    uint16_t pub_num = 0;
    rclcpp::Publisher<ihb_msgs::msg::IhbStatus>::SharedPtr ihb_status_publisher_;
    //sub Muscle
    rclcpp::Subscription<ihb_msgs::msg::MuscleJointTarget>::SharedPtr ihb_muscle_joint_sub_;
    //sub Servo
    rclcpp::Subscription<ihb_msgs::msg::ServoMotorJointTarget>::SharedPtr ihb_servo_motor_sub_;
    //sub Stepper
    rclcpp::Subscription<ihb_msgs::msg::StepperMotorJointTarget>::SharedPtr ihb_stepper_motor_sub_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IhbDebugNode>();


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
