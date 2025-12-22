uint8 id
uint8 cmd
# 添加违规默认值防止误触发
uint16[4] pressure [19999, 19999, 19999, 19999] 
float32[2] angle [999,999]
uint8[2] stiffness [255,255]
float32 kp
float32 kd
float32 ki
