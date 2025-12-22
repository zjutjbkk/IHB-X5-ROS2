# 机械臂目标控制消息 (分层组合设计)
# 通过话题命名空间区分左右臂: /left_arm/target 和 /right_arm/target

# 控制模式
uint8 cmd
# cmd 取值说明:
# 0 = 关闭PID
# 1 = 静力学模型
# 2 = PID控制 (需要发送所有肌肉关节的 kp, ki, kd 参数)

# 2个舵机关节 (保持原有字段定义和ID编号)
ServoMotorJointTarget[2] servo_joints

# 4个肌肉关节 (保持原有字段定义和ID编号)
MuscleJointTarget[4] muscle_joints

