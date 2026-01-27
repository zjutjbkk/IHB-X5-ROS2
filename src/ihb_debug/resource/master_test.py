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
        # 1. 肌肉关节 8 路原始角度值 (按照顺序排列)
        # 映射关系: [J1_m1, J1_m2, J2_m1, J2_m2, J3_m1, J3_m2, J4_m1, J4_m2]
        # 现在的设置意味着：只有关节 4 的两个肌肉有 5.0 的目标值
        self.target_angles = [
            # 5.0, -5.0,  # 关节 1 (ID:1)
            # 6.0, -6.0,  # 关节 2 (ID:2)
            # 7.0, -7.0,  # 关节 3 (ID:3)
            # 8.0, -8.0   # 关节 4 (ID:4)
            0.0,5.0,  # 关节 1 (ID:1)
            0.0, 5.0,  # 关节 2 (ID:2)
            0.0, 5.0,  # 关节 3 (ID:3)
            0.0, 5.0   # 关节 4 (ID:4)
        ]

        # 2. 4个肌肉关节的独立 PID 参数
        self.pid_params = [

            {'kp':100.0, 'ki': 0.8, 'kd': 1.5}, # 关节 1-1
            {'kp': 50.0, 'ki': 1.0, 'kd': 1.0}, # 关节 1-2
            {'kp':0.0, 'ki': 0.0, 'kd': 0.0}, # 关节 2-1
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0},  # 关节 2-2
            {'kp':0.0, 'ki': 0.0, 'kd': 0.0}, # 关节 3-1
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0},  # 关节 3-2
            {'kp':0.0, 'ki': 0.0, 'kd': 0.0}, # 关节 4-1
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0},  # 关节 4-2


            # {'kp':250.0, 'ki': 0.8, 'kd': 1.5}, # 关节 1-1
            # {'kp':180.0, 'ki': 1.0, 'kd': 1.0}, # 关节 1-2
            # # {'kp':200.0, 'ki': 1.5, 'kd': 0.0}, # 关节 1-1
            # # {'kp': 160.0, 'ki': 1.6, 'kd': 0.0}, # 关节 1-2
            # {'kp': 180.0, 'ki': 1.8, 'kd': 0.0}, # 关节 2-1
            # {'kp': 160.0, 'ki': 1.5, 'kd': 0.0},  # 关节 2-2
            # # {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # 关节 3-1
            # # {'kp': 0.0, 'ki': 0.0, 'kd': 0.0},  # 关节 3-2
            # # {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # 关节 4
            # {'kp':180.0, 'ki': 1.0, 'kd': 0.5}, # 关节 3-1
            # {'kp': 240.0, 'ki': 1.0, 'kd': 1.0}, # 关节 3-2
            # {'kp': 240.0, 'ki': 1.0, 'kd': 0.1}, # 关节 4-1
            # {'kp': 220.0, 'ki': 1.2, 'kd': 0.1},  # 关节 4-2
            # # {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, #4-1
            # # {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, #4-2
        ]

        # 3. 舵机参数 (角度, 速度限制0)
        self.servo_params = [
            {'angle':-500.0, 'speed': 120}, # 舵机 1
            {'angle': 0.0, 'speed': 120}, # 舵机 2
        ]
        # ===================================================

    def print_summary(self):
        """在终端打印本次发送的具体参数内容"""
        print("\n" + "="*60)
        print(f"{'ID':^5} | {'Target Angles (Pair)':^25} | {'PID1 (P/I/D)':^20}| {'PID2 (P/I/D)':^20}")
        print("-"*60)
        
        # 肌肉关节打印：改为 2*i 和 2*i+1 的步进方式
        for i in range(4):
            # 获取当前关节对应的两个肌肉索引
            idx1 = i * 2
            idx2 = i * 2 + 1
            angles = f"[{self.target_angles[idx1]:>6.1f}, {self.target_angles[idx2]:>6.1f}]"
            p1=self.pid_params[idx1]
            p2=self.pid_params[idx2]
            pid1_str = f"{p1['kp']:.1f}/{p1['ki']:.1f}/{p1['kd']:.1f}"
            pid2_str = f"{p2['kp']:.1f}/{p2['ki']:.1f}/{p2['kd']:.1f}"
            print(f"M_{i+1:^2} | {angles:^25} | {pid1_str:^20}| {pid2_str:^20}") #^20这个代表的是居中对齐20个字符的kuandu

        print("-"*60)
        # 舵机关节打印
        for i in range(2):
            s = self.servo_params[i]
            print(f"S_{i+1:^2} | Angle: {s['angle']:>17.1f} | Speed Limit: {s['speed']:>7}")
        
        print("="*60 + "\n")

    def run_once(self):
        # 等待节点发现，确保从端能收到
        
        time.sleep(1.5)
        
        msg = ArmTarget()
        msg.cmd = 2  # PID模式
        
        # --- 填充肌肉关节 ---
        muscle_list = []
        for i in range(4):
            m_joint = MuscleJointTarget()
            m_joint.id = i + 1
            # 修改映射逻辑：由 [i, 7-i] 改为 [i*2, i*2+1]
            idx1 = i * 2
            idx2 = i * 2 + 1
            m_joint.angle = [float(self.target_angles[idx1]), float(self.target_angles[idx2])]
            
            p = self.pid_params[i]
            m_joint.kp = [float(self.pid_params[idx1]['kp']), float(self.pid_params[idx2]['kp'])]
            m_joint.ki = [float(self.pid_params[idx1]['ki']), float(self.pid_params[idx2]['ki'])]
            m_joint.kd = [float(self.pid_params[idx1]['kd']), float(self.pid_params[idx2]['kd'])]
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
        self.get_logger().info('双 PID 指令已发布')
        
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