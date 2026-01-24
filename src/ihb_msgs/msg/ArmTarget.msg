# [建议] 添加标准消息头，用于追踪时间戳和坐标系
std_msgs/Header header

# --- 控制模式常量定义 ---
uint8 CMD_OFF = 0          # 关闭 PID / 释放电机
uint8 CMD_STATICS = 1      # 静力学模型控制
uint8 CMD_PID_CONTROL = 2  # PID 闭环控制

# 控制模式字段
uint32 cmd

# --- 关节定义 ---
# 2个舵机关节: 数组大小固定为 2
ServoMotorJointTarget[2] servo_joints

# 4个肌肉关节: 数组大小固定为 4
MuscleJointTarget[4] muscle_joints