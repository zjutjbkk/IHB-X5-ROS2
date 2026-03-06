import rclpy
from rclpy.node import Node
import time

# 引入你的消息定义 (请根据你的实际情况确认包名)
from ihb_msgs.msg import IhbTarget

# =========================================================================================
# ⬇️⬇️⬇️ PID 与 目标角度 调试配置区 ⬇️⬇️⬇️
# =========================================================================================
# 说明：
# - angle_0 对应该关节的 第 1 个自由度 (对应 C++ 里的 angle[0])
# - angle_1 对应该关节的 第 2 个自由度 (对应 C++ 里的 angle[1])
# - 每一对 angle 都有独立的 P, I, D 参数
# =========================================================================================

PID_CONFIG = [
    # ----------------------------------- 【 左 臂 (Left Arm) 】 -----------------------------------
    # Joint 1 (对应代码 muscle_joint[0])
    {
        # "angle_0": -9.0,  "p_0": 60.0, "i_0": 1.6, "d_0": 0.0,   # 第1自由度
        # "angle_1": -9.0,  "p_1":  45.0, "i_1": 1.6, "d_1": 0.0    # 第2自由度
        "angle_0": 0.0,  "p_0": 0.0, "i_0": 0.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 0.0, "i_1": 0.0, "d_1": 0.0  
    },
    # Joint 2 (对应代码 muscle_joint[1])
    {
        # "angle_0": -5.0,  "p_0": 40.0, "i_0": 1.3, "d_0": 0.0,   
        # "angle_1": -5.0,  "p_1": 30.0, "i_1": 1.4, "d_1": 0.0  
        "angle_0": 0.0,  "p_0": 0.0, "i_0": 0.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 0.0, "i_1": 0.0, "d_1": 0.0      
    }, 
    # Joint 3 (对应代码 muscle_joint[2])
    {
        # "angle_0": -5.0,  "p_0":60.0, "i_0": 1.3, "d_0": 0.0,   
        # "angle_1": 0.0,  "p_1": 60.0, "i_1": 1.1, "d_1": 0.0    
        "angle_0": 0.0,  "p_0": 0.0, "i_0": 0.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 0.0, "i_1": 0.0, "d_1": 0.0    
    },
    # Joint 4 (对应代码 muscle_joint[3])
    {
        # "angle_0": -5.0,  "p_0": 40.0, "i_0": 1.0, "d_0": 0.0,   
        # "angle_1": -5.0,  "p_1": 40.0, "i_1": 1.0, "d_1": 0.0    
         "angle_0": 0.0,  "p_0": 0.0, "i_0": 0.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 0.0, "i_1": 0.0, "d_1": 0.0    
    },

    # ----------------------------------- 【 右 臂 (Right Arm) 】 -----------------------------------
    # Joint 1 (对应代码 muscle_joint[4])
    {
        # "angle_0": 5.0,  "p_0": 25.0, "i_0": 1.0, "d_0": 0.0,   # 第1自由度
        # "angle_1": 0.0,  "p_1": 28.0, "i_1": 1.0, "d_1": 0.0    # 第2自由度
        "angle_0": 0.0,  "p_0": 0.0, "i_0": 0.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 0.0, "i_1": 0.0, "d_1": 0.0   
    },
    # Joint 2 (对应代码 muscle_joint[5])
    {
        # "angle_0": 5.0,  "p_0": 32.0, "i_0": 0.55, "d_0": 0.0,   
        # "angle_1": 0.0,  "p_1": 35.0, "i_1": 0.55, "d_1": 0.0   
         "angle_0": 0.0,  "p_0": 0.0, "i_0": 0.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 0.0, "i_1": 0.0, "d_1": 0.0   
    },
    # Joint 3 (对应代码 muscle_joint[6])
    {
        # "angle_0": 5.0,  "p_0": 35.0, "i_0": 0.7, "d_0": 0.0,   
        # "angle_1": 0.0,  "p_1": 20.0, "i_1": 0.5, "d_1": 0.0  
           "angle_0": 0.0,  "p_0": 0.0, "i_0": 0.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 0.0, "i_1": 0.0, "d_1": 0.0  
    },
    # Joint 4 (对应代码 muscle_joint[7])
    {
        "angle_0": 5.0,  "p_0": 28.0, "i_0": 1.0, "d_0": 0.0,   
        "angle_1": 0.0,  "p_1": 28.0, "i_1": 1.0, "d_1": 0.0    
    }
]

# =========================================================================================
# ⬆️⬆️⬆️ 仅修改上面的区域即可，无需修改下方代码 ⬆️⬆️⬆️
# =========================================================================================

class PidTargetPublisher(Node):
    def __init__(self):
        super().__init__('pid_target_publisher')
        self.publisher_ = self.create_publisher(IhbTarget, 'arm_target', 10)
        
        self.get_logger().info("等待与底层控制器建立连接...")
        time.sleep(1.0) # 延迟1秒确保发布者注册成功，防止丢失消息
        self.publish_target()

    def publish_target(self):
        msg = IhbTarget()
        
        # 2 表示 CMD_PID_CONTROL，C++底层只有收到 2 才会更新 PID 参数
        msg.cmd = 2 
        
        # 遍历 8 个肌肉关节，将顶部的配置写入 ROS 消息中
        for i in range(8):
            cfg = PID_CONFIG[i]
            
            # --- 赋值第一个自由度 (angle[0]) 及其 PID ---
            msg.muscle_joint[i].angle[0] = float(cfg["angle_0"])
            msg.muscle_joint[i].kp[0]    = float(cfg["p_0"])
            msg.muscle_joint[i].ki[0]    = float(cfg["i_0"])
            msg.muscle_joint[i].kd[0]    = float(cfg["d_0"])
            
            # --- 赋值第二个自由度 (angle[1]) 及其 PID ---
            msg.muscle_joint[i].angle[1] = float(cfg["angle_1"])
            msg.muscle_joint[i].kp[1]    = float(cfg["p_1"])
            msg.muscle_joint[i].ki[1]    = float(cfg["i_1"])
            msg.muscle_joint[i].kd[1]    = float(cfg["d_1"])

        # 为了安全，将 4 个伺服电机的目标也初始化一下，防止发送随机内存垃圾值
        for i in range(4):
            msg.servo_joint[i].angle = 0.0
            msg.servo_joint[i].speed_limit = 50

        self.publisher_.publish(msg)
        self.get_logger().info("✅ 成功下发 16个角度目标 和 16组PID参数！")


def main(args=None):
    rclpy.init(args=args)
    node = PidTargetPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()