##步进电机控制原理
1.步进电机通过pwm控制，通过DM860H将控制板信号放大，拨码开关设置的脉冲数一万”，即 10000 脉冲/转，“脉冲数一万” = 每转需要 10000 个脉冲才能让电机完整旋转一圈。标准 1.8° 步距角 的两相步进电机，每个脉冲对应的角度 = 360° ÷ 10000 = 0.036°，发送 10000 个脉冲 → 电机刚好转一圈（360°），就是pwm的频率 1s 10000hz  
转速100 RPM  每秒转数 = 100 ÷ 60 ≈ 1.67 转/秒 所需脉冲频率 = 1.67 × 10000 = 16,700 Hz（约 16.7kHz）

static constexpr float STEPS_PER_MM = 2000.0f;

int32_t heightToSteps(int16_t height_mm) const {
  return static_cast<int32_t>(height_mm * STEPS_PER_MM);
}
int16_t stepsToHeight(int32_t steps) const {
  return static_cast<int16_t>(steps / STEPS_PER_MM);
}
1mm 对应 2000 步
设置频率 speed是一秒多少步
dir_pin_line_.set_value(move_direction ? 1 : 0);
// 确保PWM被禁用
pwm_enable(false);
sleep 10ms;
// 先设置period，再设置duty_cycle
int period_ns = 1000000000 / speed;
pwm_set_period(period_ns);
sleep 10ms;
pwm_set_duty_cycle(period_ns / 2);
sleep 10ms;

// 最后启用PWM
pwm_enable(true);

计算用多少微秒，输出对应步数需要多少脉冲
int run_time_ms = (steps * 1000) / speed;
std::this_thread::sleep_for(std::chrono::milliseconds(run_time_ms));

速度设置10000 一圈5mm 10000步 所以2000步1mm 
！！！！！！！！！一定要共地，遇到问题先检查共地，找不到问题没事的话就重新剥线！！！！！！
创建了step_motor_publisher和step_motor_sub两个cpp文件，一个用来定时发送-5mm的目标位置，一个用来接收目标位置并控制步进电机
传入-5mm,即向上移动5mm
订阅节点关闭后会自动订阅析构函数回到零点

