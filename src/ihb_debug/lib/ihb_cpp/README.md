#  CMake 升级

https://cmake.org/files/v3.31/cmake-3.31.0-rc1-linux-aarch64.sh
sudo bash cmake-3.31.0-rc1-linux-aarch64.sh --skip-licence --prefix=/usr

y  n

# IHB_DEMO
ihb 底层驱动的 cpp 实现，为 hardware 做准备


can https://docs.qq.com/sheet/DYU9tcHVGVUZOTGJz?tab=BB08J2

48 个比例阀  双臂 32 + 变刚度 8 剩余 8 个给手部 

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=0 -p in_mode:=ros -p in_format:=rgb8 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/camera/color/image_raw -p  pub_topic:=/image_jpeg --log-level error &

想把霍尔和步进电机也弄过来，可惜不支持c语言，再去问问吧
## 类型定义

#### StepperMotorJoint（步进电机升降） stepper_motor_joint.h 0x201

获取当前位置
写入当前位置到 Flash
位置控制命令
把当前位置写入 flash 作为零点
电机停止（）

#### ServoMotorJoint（关节伺服电机） servo_motor_joint.h  0x141  0x142 0x143 0x144

标识符：0x140 + ID (1~32) 帧格式：DATA  帧类型：标准帧   DLC：8 字节

写入当前位置到 ROM 作为电机零点命令
位置闭环控制命令 2（1 帧）（带有速度控制的位置闭环）
速度闭环控制命令
转矩闭环控制命令
读取多圈角度命令（需要将多圈角度转换为关节角度）
读取电机状态 2 命令（1 帧）（主要是要用速度和转矩电流，编码器位置暂时没用）

(17) 电机停止命令（1 帧）停止电机，但不清除电机运行状态和之前接收的控制指令
(18) 电机运行命令（1 帧）从电机停止命令中恢复电机运行（恢复停止前的控制方式）
#### MuscleJoint（气动肌肉关节）需要等待控制组 muscle_joint.h  
 0x180 0x18F



获取当前位置 （角度）
写入当前位置到 Flash
位置控制命令
把当前位置写入 flash 作为零点
电机停止（）