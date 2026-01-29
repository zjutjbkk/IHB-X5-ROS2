# [建议] 添加标准消息头，用于追踪时间戳和坐标系
std_msgs/Header header

# --- 控制模式常量定义 ---
uint8 CMD_OFF = 0          # 关闭 PID / 释放电机
uint8 CMD_STATICS = 1      # 静力学模型控制
uint8 CMD_PID_CONTROL = 2  # PID 闭环控制

# 控制模式字段
uint32 cmd

# --- 关节定义 ---
# 4个舵机关节: 数组大小固定为 4
ServoMotorJointTarget[4] servo_joint

# 8个肌肉关节: 数组大小固定为 8
MuscleJointTarget[8] muscle_joint

# --- 2. 夹爪/末端反馈 (GPIO) ---
# 对应你的两个 GPIO 开关 (GripperSwitch.isTriggered)
# true = 闭合/接触, false = 松开/悬空
bool left_gripper_touched   # 对应 GPIO 27 (Pin 13)
bool right_gripper_touched  # 对应 GPIO 17 (Pin 11)