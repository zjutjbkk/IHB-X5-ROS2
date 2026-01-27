#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ihb_msgs.msg import ArmTarget, MuscleJointTarget, ServoMotorJointTarget 
import time
import math

class IhbContinuousControl(Node):
    def __init__(self):
        super().__init__('ihb_continuous_control')
        self.publisher = self.create_publisher(ArmTarget, 'arm_target', 10)
        
        # 1. 设定发布频率：10Hz (每秒10次)
        self.timer_period = 0.1  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 2. 运动参数
        self.start_time = time.time()
        self.cycle_duration = 20.0  # 完成 0 -> 5 -> 0 一个完整周期所需的时间(秒)
        self.max_angle = 5.0       # 最大角度
        self.min_angle = 0.0       # 最小角度

        # 3. PID 参数（保持你之前的配置）
        self.pid_configs = [
            # {'kp': 250.0, 'ki': 0.8, 'kd': 1.5}, # J1-1
            # {'kp': 180.0, 'ki': 1.0, 'kd': 1.0}, # J1-2
            # {'kp': 180.0, 'ki': 1.8, 'kd': 0.0}, # J2-1
            # {'kp': 160.0, 'ki': 1.5, 'kd': 0.0}, # J2-2
            # {'kp': 180.0, 'ki': 1.0, 'kd': 0.5}, # J3-1
            # {'kp': 240.0, 'ki': 1.0, 'kd': 1.0}, # J3-2
            # {'kp': 240.0, 'ki': 1.0, 'kd': 0.1}, # J4-1
            # {'kp': 220.0, 'ki': 1.2, 'kd': 0.1}, # J4-2
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # J3-1
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # J1-2
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # J2-1
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # J2-2
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # J3-1
            {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}, # J3-2
            {'kp': 100.0, 'ki': 0.8, 'kd': 1.5}, # J1-1
            {'kp': 100.0, 'ki': 1.0, 'kd': 1.0}, # J4-2
        ]

    def get_current_angle(self):
        """
        计算当前时刻应有的角度值 (0 -> 5 -> 0)
        这里使用三角函数实现平滑过渡，如果你需要严格线性，可以改用三角波公式
        """
        elapsed_time = time.time() - self.start_time
        # 计算相位 (0 到 2*PI)
        phase = (elapsed_time % self.cycle_duration) / self.cycle_duration * (2 * math.pi)
        
        # 使用 cos 函数实现从 0 到 5 再到 0 的往复
        # cos(0)=1, cos(pi)=-1. 我们将其映射到 0.0-5.0
        # 结果 = 中间值 - 幅度 * cos(相位)
        amplitude = (self.max_angle - self.min_angle) / 2
        offset = self.min_angle + amplitude
        angle = offset - amplitude * math.cos(phase)
        return angle

    def timer_callback(self):
        target_val = self.get_current_angle()
        
        msg = ArmTarget()
        msg.cmd = 2  # PID模式
        
        # --- 填充肌肉关节 ---
        muscle_list = []
        for i in range(4):
            m_joint = MuscleJointTarget()
            m_joint.id = i + 1
            idx1, idx2 = i * 2, i * 2 + 1
            
            # 设置动态角度：这里假设所有关节都同步做 0-5 度运动
            # 奇数位给正，偶数位给负（根据你原脚本的逻辑）
            m_joint.angle = [float(target_val), float(-target_val)]
            
            m_joint.kp = [self.pid_configs[idx1]['kp'], self.pid_configs[idx2]['kp']]
            m_joint.ki = [self.pid_configs[idx1]['ki'], self.pid_configs[idx2]['ki']]
            m_joint.kd = [self.pid_configs[idx1]['kd'], self.pid_configs[idx2]['kd']]
            muscle_list.append(m_joint)
        
        msg.muscle_joints = muscle_list

        # --- 填充舵机关节 (保持静态或跟随) ---
        servo_list = []
        for i in range(2):
            s_joint = ServoMotorJointTarget()
            s_joint.id = i + 1
            s_joint.angle = -500.0 if i == 0 else 0.0
            s_joint.speed_limit = 120
            servo_list.append(s_joint)
        msg.servo_joints = servo_list

        self.publisher.publish(msg)
        self.get_logger().info(f'发布角度: {target_val:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = IhbContinuousControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('控制停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()