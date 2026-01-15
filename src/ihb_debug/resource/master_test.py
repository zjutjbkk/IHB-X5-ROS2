#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ihb_msgs.msg import ArmTarget, MuscleJointTarget, ServoMotorJointTarget 
import time

class IhbSingleShotControl(Node):
    def __init__(self):
        super().__init__('ihb_single_shot_control')
        self.publisher = self.create_publisher(ArmTarget, 'arm_target', 10)
        
        # ================== 核心参数修改区 ==================
        # 1. 肌肉关节 8 路原始角度值
        self.target_angles = [10.0, 15.0, 5.0, 0.0, 0.0, -5.0, -15.0, -10.0]

        # 2. 4个肌肉关节的独立 PID 参数
        self.pid_params = [
            {'kp': 35.0, 'ki': 1.5, 'kd': 100.0}, # 关节 1
            {'kp': 35.0, 'ki': 1.5, 'kd': 100.0}, # 关节 2
            {'kp': 40.0, 'ki': 2.0, 'kd': 110.0}, # 关节 3
            {'kp': 30.0, 'ki': 1.0, 'kd': 90.0},  # 关节 4
        ]

        # 3. 舵机参数 (角度, 速度限制)
        self.servo_params = [
            {'angle': 45.0, 'speed': 120}, # 舵机 1
            {'angle': -45.0, 'speed': 120}, # 舵机 2
        ]
        # ===================================================

    def print_summary(self):
        """在终端打印本次发送的具体参数内容"""
        print("\n" + "="*60)
        print(f"{'ID':^5} | {'Target Angles (Dual)':^25} | {'PID (P/I/D)':^20}")
        print("-"*60)
        
        # 肌肉关节打印
        for i in range(4):
            angles = f"[{self.target_angles[i]:>6.1f}, {self.target_angles[7-i]:>6.1f}]"
            pid = f"{self.pid_params[i]['kp']}/{self.pid_params[i]['ki']}/{self.pid_params[i]['kd']}"
            print(f"M_{i+1:^2} | {angles:^25} | {pid:^20}")
        
        print("-"*60)
        # 舵机关节打印
        for i in range(2):
            s = self.servo_params[i]
            print(f"S_{i+1:^2} | Angle: {s['angle']:>17.1f} | Speed Limit: {s['speed']:>7}")
        
        print("="*60 + "\n")

    def run_once(self):
        # 等待节点发现，确保从端能收到
        time.sleep(0.5)
        
        msg = ArmTarget()
        msg.cmd = 2  # PID模式
        
        # --- 填充肌肉关节 ---
        muscle_list = []
        for i in range(4):
            m_joint = MuscleJointTarget()
            m_joint.id = i + 1
            m_joint.angle = [float(self.target_angles[i]), float(self.target_angles[7 - i])]
            p = self.pid_params[i]
            m_joint.kp, m_joint.ki, m_joint.kd = p['kp'], p['ki'], p['kd']
            muscle_list.append(m_joint)
        msg.muscle_joints = muscle_list

        # --- 填充舵机关节 ---
        servo_list = []
        for i in range(2):
            s_joint = ServoMotorJointTarget()
            s_joint.id = i + 1
            s_joint.angle = float(self.servo_params[i]['angle'])
            s_joint.speed_limit = self.servo_params[i]['speed']
            servo_list.append(s_joint)
        msg.servo_joints = servo_list

        # --- 打印、发布并退出 ---
        self.print_summary()
        self.publisher.publish(msg)
        self.get_logger().info('指令已发布，正在关闭节点...')
        
        # 预留一点时间完成物理发送
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = IhbSingleShotControl()
    node.run_once()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()