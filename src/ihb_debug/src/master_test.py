#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from ihb_msgs.msg import ArmTarget, ServoJointTarget, MuscleJointTarget
import sys

class IhbTestSender(Node):
    def __init__(self):
        super().__init__('ihb_test_sender')
        
        # 创建发布者，话题名称必须与从臂订阅的一致
        self.publisher_ = self.create_publisher(ArmTarget, 'arm_target', 10)
        
        self.get_logger().info('>>> [Python 调试端] 启动成功！')
        self.get_logger().info('你可以通过此脚本发送目标点来测试从臂 PID 效果。')
        
        # 初始化默认目标值
        self.target_servo_angle = 0.0
        self.target_muscle_angles = [0, 0, 0, 0] # 4个肌肉关节的基础角度

    def send_target(self, s1_angle, s2_angle, m_angles):
        """
        构造并发送 ArmTarget 消息
        s1_angle, s2_angle: 伺服电机角度 (float)
        m_angles: 长度为 4 的列表，表示 4 个肌肉关节的角度 (int)
        """
        msg = ArmTarget()
        msg.cmd = 2  # PID控制模式

        # 1. 填充伺服电机数据 (固定数组大小为 2)
        # 对应从臂的 ser1 (ID 1)
        msg.servo_joints[0].id = 1
        msg.servo_joints[0].angle = float(s1_angle)
        msg.servo_joints[0].speed_limit = 120.0

        # 对应从臂的 ser2 (ID 2)
        msg.servo_joints[1].id = 2
        msg.servo_joints[1].angle = float(s2_angle)
        msg.servo_joints[1].speed_limit = 120.0

        # 2. 填充 4 个肌肉关节数据 (固定数组大小为 4)
        for i in range(4):
            msg.muscle_joints[i].id = i + 1
            # 这里模拟主臂的逻辑：angle[0] 是 Hall[i], angle[1] 是 Hall[7-i]
            # 为了方便调试，我们假设两边是对称的
            msg.muscle_joints[i].angle[0] = int(m_angles[i])
            msg.muscle_joints[i].angle[1] = int(m_angles[i])

        self.publisher_.publish(msg)
        self.get_logger().info(f'已发送 -> 伺服:[{s1_angle}, {s2_angle}] 肌肉:{m_angles}')

def main(args=None):
    rclpy.init(args=args)
    node = IhbTestSender()

    try:
        while rclpy.ok():
            print("\n" + "="*40)
            print("请输入目标指令 (格式: 伺服1 伺服2 肌肉全角度)")
            print("例如: '30 -30 1500' 表示伺服转到30度和-30度，所有肌肉关节设为1500")
            print("输入 'q' 退出")
            
            user_input = input(">> ").split()
            
            if len(user_input) == 0:
                continue
            if user_input[0].lower() == 'q':
                break
            
            try:
                if len(user_input) >= 3:
                    s1 = float(user_input[0])
                    s2 = float(user_input[1])
                    m_val = int(user_input[2])
                    m_list = [m_val] * 4
                    node.send_target(s1, s2, m_list)
                else:
                    print("输入格式错误，请重新输入")
            except ValueError:
                print("错误：请输入数字！")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()