#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ihb_msgs.msg import ArmTarget, MuscleJointStatus, ServoMotorJointStatus
import sys

class SlaveArmCommander(Node):
    def __init__(self):
        super().__init__('slave_arm_commander')
        
        # 创建发布器，发布到从臂监听的 'slave_arm_target' 话题
        self.publisher_ = self.create_publisher(ArmTarget, 'slave_arm_target', 10)
        
        # 初始化一个完整的整臂目标消息
        self.target_msg = ArmTarget()
        
        # 预先初始化 4 个肌肉关节的数据结构
        for i in range(4):
            m = MuscleJointStatus()
            m.id = i + 1
            m.angle = [0, 0] # 初始角度
            # 这里可以设置一些默认的 PID，或者保持 0（由 C++ 端使用默认值）
            m.kp = 0.0 
            m.ki = 0.0
            m.kd = 0.0
            self.target_msg.muscle_joints.append(m)

        # 预先初始化 2 个伺服电机（虽然你 C++ 里注释了，但消息结构建议保留）
        for i in range(2):
            s = ServoMotorJointStatus()
            s.id = i + 1
            s.angle = 0.0
            s.speed_limit = 50.0
            self.target_msg.servo_joints.append(s)

        self.get_logger().info("=== IHB 从臂角度/PID 调试工具已启动 ===")
        self.get_logger().info("您可以实时输入参数来调整从臂状态")

    def publish_cmd(self):
        """执行发布动作"""
        self.publisher_.publish(self.target_msg)
        self.get_logger().info("指令已发送至从臂！")

    def run_console(self):
        """交互式终端界面"""
        while rclpy.ok():
            print("\n" + "="*40)
            print("当前肌肉关节状态预览 (M1-M4):")
            for i in range(4):
                m = self.target_msg.muscle_joints[i]
                print(f"M{m.id}: 角度{m.angle} | PID: [{m.kp}, {m.ki}, {m.kd}]")
            
            print("-" * 40)
            print("操作选项: [1]修改角度 [2]修改PID [3]发送当前数据 [q]退出")
            cmd = input("请输入指令: ").strip().lower()

            if cmd == 'q':
                break
            
            try:
                if cmd == '1':
                    idx = int(input("选择肌肉关节 ID (1-4): ")) - 1
                    a1 = int(input("输入目标角度 1 (int16): "))
                    a2 = int(input("输入目标角度 2 (int16): "))
                    self.target_msg.muscle_joints[idx].angle = [a1, a2]
                    self.publish_cmd()

                elif cmd == '2':
                    idx = int(input("选择肌肉关节 ID (1-4): ")) - 1
                    kp = float(input("输入 Kp: "))
                    ki = float(input("输入 Ki: "))
                    kd = float(input("输入 Kd: "))
                    self.target_msg.muscle_joints[idx].kp = kp
                    self.target_msg.muscle_joints[idx].ki = ki
                    self.target_msg.muscle_joints[idx].kd = kd
                    self.publish_cmd()

                elif cmd == '3':
                    self.publish_cmd()
                
            except ValueError:
                print("输入格式错误，请输入数字。")
            except Exception as e:
                print(f"发生错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SlaveArmCommander()
    
    # 使用线程运行终端交互，避免阻塞 ROS 处理
    import threading
    thread = threading.Thread(target=node.run_console)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()