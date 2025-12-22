/**
 * @file statics_model.cpp
 * @brief 机械臂静力学模型类实现
 * @details 基于肌肉控制系统的静力学计算实现
 */

#include "statics_model.h"
#include "matrix_math.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace ihb {

/* 常量定义 */
constexpr size_t MAX_JOINTS = 10;
constexpr size_t NUM_GIMBALS = 4;
constexpr size_t MUSCLES_PER_GIMBAL = 4;
constexpr size_t MAX_ACTUATOR_CHANNELS = 24;
constexpr float GRAVITY_CONSTANT = 9.81f;
constexpr float PI = 3.14159265359f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

/* D-H参数结构体 */
struct LinkParameters {
    float theta;    // 关节角度 (rad)
    float d;        // 连杆偏移 (mm)
    float a;        // 连杆长度 (mm)
    float alpha;    // 连杆扭转角 (rad)
};

/* 万向节特定参数结构体 */
struct GimbalParams {
    // 弹性项多项式系数 (三次多项式: p1*x³ + p2*x² + p3*x + p4)
    float ktheta_p1;
    float ktheta_p2;
    float ktheta_p3;
    float ktheta_p4;
    
    // 力转气压转换参数
    float p_a;
    float p_b;
    float p_c;
    
    // 执行器通道映射
    uint8_t actuator_channels[MUSCLES_PER_GIMBAL];
};

/* 机器人模型结构体 */
struct RobotModel {
    LinkParameters links[MAX_JOINTS];
    math::Matrix4x4 base_transform;
    math::Matrix4x4 tool_transform;
    uint8_t num_joints;
};

/* 肌肉控制参数结构体 */
struct MuscleControlParams {
    // 万向节配置
    uint8_t num_gimbals;
    uint8_t gimbal_joints[NUM_GIMBALS];
    uint8_t gimbal_pairs[NUM_GIMBALS][2];
    
    // 质量参数
    float gimbal_masses[NUM_GIMBALS];
    float end_effector_mass;
    
    // 肌肉几何参数
    float muscle_angles_rad[MUSCLES_PER_GIMBAL];
    float muscle_arm_length;
    
    // 万向节特定参数
    GimbalParams gimbal_params[NUM_GIMBALS];
    
    // 物理约束
    float max_muscle_force;
    
    // 预计算的雅可比矩阵
    float jacobian_muscle[2][MUSCLES_PER_GIMBAL];
};

/* StaticsModel::Impl 实现类 */
struct StaticsModel::Impl {
    RobotModel robot_model;
    MuscleControlParams muscle_params;
    bool initialized;
    float last_max_balance_error;
    
    Impl();
    
    // 初始化函数
    void initializeRobotModel();
    void initializeMuscleParams();
    void buildMuscleJacobian();
    
    // 计算函数
    bool compute(const std::array<float, 10>& joint_angles_deg,
                StaticsModelResult& result);
    
    // 内部计算函数
    void calculateForwardKinematics(const float joint_angles_rad[MAX_JOINTS],
                                   std::array<math::Matrix4x4, MAX_JOINTS + 1>& transforms);
    
    void calculateGravityTorques(const float joint_angles_rad[MAX_JOINTS],
                                const std::array<math::Matrix4x4, MAX_JOINTS + 1>& transforms,
                                float gravity_torques_local[3][NUM_GIMBALS]);
    
    void calculateElasticTorques(const float joint_angles_deg[MAX_JOINTS],
                                float elastic_torques_yz[2][NUM_GIMBALS]);
    
    void solveMuscleForces(const float target_torques_yz[2][NUM_GIMBALS],
                          float muscle_forces[NUM_GIMBALS][MUSCLES_PER_GIMBAL],
                          float& max_balance_error);
    
    void convertForcesToPressures(const float muscle_forces[NUM_GIMBALS][MUSCLES_PER_GIMBAL],
                                 float muscle_pressures[NUM_GIMBALS][MUSCLES_PER_GIMBAL]);
    
    void mapPressuresToActuators(const float muscle_pressures[NUM_GIMBALS][MUSCLES_PER_GIMBAL],
                                float actuator_pressures[MAX_ACTUATOR_CHANNELS]);
    
    // 辅助函数
    void convertToEquivalentPlaneAngles(float theta1_deg, float theta2_deg,
                                       float& alpha_deg, float& beta_deg);
    
    float getElasticTorqueMagnitude(float beta_deg, const GimbalParams& gimbal_params);
    
    float forceToPressure(float force, const GimbalParams& gimbal_params);
};

/* Impl 构造函数 */
StaticsModel::Impl::Impl() 
    : initialized(false), last_max_balance_error(0.0f) {
    initializeRobotModel();
    initializeMuscleParams();
    initialized = true;
}

/* 初始化机器人模型 */
void StaticsModel::Impl::initializeRobotModel() {
    robot_model.num_joints = MAX_JOINTS;
    
    // D-H参数设置
    // Link 1: [0, 120, 0, 0]
    robot_model.links[0] = {0.0f, 120.0f, 0.0f, 0.0f};
    
    // Link 2: [0, 0, 0, pi/2]
    robot_model.links[1] = {0.0f, 0.0f, 0.0f, PI / 2.0f};
    
    // Links 3-10: 交替的参数
    for (uint8_t i = 2; i < MAX_JOINTS; i++) {
        robot_model.links[i].theta = 0.0f;
        robot_model.links[i].d = 0.0f;
        
        if (i % 2 == 0) {  // 万向节主关节
            robot_model.links[i].a = 110.0f;
            robot_model.links[i].alpha = -PI / 2.0f;
        } else {           // 万向节副关节
            robot_model.links[i].a = 0.0f;
            robot_model.links[i].alpha = PI / 2.0f;
        }
    }
    
    // 特殊设置第3个连杆
    robot_model.links[2].a = 176.5f;
    
    // 基座变换矩阵：troty(pi/2)
    robot_model.base_transform.setIdentity();
    robot_model.base_transform(0, 0) = 0.0f;
    robot_model.base_transform(0, 2) = 1.0f;
    robot_model.base_transform(2, 0) = -1.0f;
    robot_model.base_transform(2, 2) = 0.0f;
    
    // 工具变换矩阵：transl(73, 0, 0)
    robot_model.tool_transform.setIdentity();
    robot_model.tool_transform(0, 3) = 73.0f / 1000.0f;  // 转换为米
}

/* 初始化肌肉控制参数 */
void StaticsModel::Impl::initializeMuscleParams() {
    // 万向节配置
    muscle_params.num_gimbals = NUM_GIMBALS;
    muscle_params.gimbal_joints[0] = 2;
    muscle_params.gimbal_joints[1] = 4;
    muscle_params.gimbal_joints[2] = 6;
    muscle_params.gimbal_joints[3] = 8;
    
    // 万向节关节对
    muscle_params.gimbal_pairs[0][0] = 2; muscle_params.gimbal_pairs[0][1] = 3;
    muscle_params.gimbal_pairs[1][0] = 4; muscle_params.gimbal_pairs[1][1] = 5;
    muscle_params.gimbal_pairs[2][0] = 6; muscle_params.gimbal_pairs[2][1] = 7;
    muscle_params.gimbal_pairs[3][0] = 8; muscle_params.gimbal_pairs[3][1] = 9;
    
    // 质量参数
    muscle_params.gimbal_masses[0] = 0.69f;
    muscle_params.gimbal_masses[1] = 0.69f;
    muscle_params.gimbal_masses[2] = 0.69f;
    muscle_params.gimbal_masses[3] = 0.47f;
    muscle_params.end_effector_mass = 0.23f;
    
    // 肌肉几何参数
    muscle_params.muscle_angles_rad[0] = 45.0f * DEG_TO_RAD;
    muscle_params.muscle_angles_rad[1] = 135.0f * DEG_TO_RAD;
    muscle_params.muscle_angles_rad[2] = 225.0f * DEG_TO_RAD;
    muscle_params.muscle_angles_rad[3] = 315.0f * DEG_TO_RAD;
    muscle_params.muscle_arm_length = 0.073f;
    
    // 万向节0参数
    muscle_params.gimbal_params[0].ktheta_p1 = 23.06f;
    muscle_params.gimbal_params[0].ktheta_p2 = -14.6f;
    muscle_params.gimbal_params[0].ktheta_p3 = 8.915f;
    muscle_params.gimbal_params[0].ktheta_p4 = 0.5882f;
    muscle_params.gimbal_params[0].p_a = 0.3564f;
    muscle_params.gimbal_params[0].p_b = 55.26f;
    muscle_params.gimbal_params[0].p_c = -0.0004f;
    muscle_params.gimbal_params[0].actuator_channels[0] = 0;
    muscle_params.gimbal_params[0].actuator_channels[1] = 1;
    muscle_params.gimbal_params[0].actuator_channels[2] = 2;
    muscle_params.gimbal_params[0].actuator_channels[3] = 3;
    
    // 万向节1参数
    muscle_params.gimbal_params[1].ktheta_p1 = 23.06f;
    muscle_params.gimbal_params[1].ktheta_p2 = -14.6f;
    muscle_params.gimbal_params[1].ktheta_p3 = 8.915f;
    muscle_params.gimbal_params[1].ktheta_p4 = 0.5882f;
    muscle_params.gimbal_params[1].p_a = 0.3564f;
    muscle_params.gimbal_params[1].p_b = 55.26f;
    muscle_params.gimbal_params[1].p_c = -0.0004f;
    muscle_params.gimbal_params[1].actuator_channels[0] = 4;
    muscle_params.gimbal_params[1].actuator_channels[1] = 5;
    muscle_params.gimbal_params[1].actuator_channels[2] = 6;
    muscle_params.gimbal_params[1].actuator_channels[3] = 7;
    
    // 万向节2参数
    muscle_params.gimbal_params[2].ktheta_p1 = 23.06f;
    muscle_params.gimbal_params[2].ktheta_p2 = -14.6f;
    muscle_params.gimbal_params[2].ktheta_p3 = 8.915f;
    muscle_params.gimbal_params[2].ktheta_p4 = 0.5882f;
    muscle_params.gimbal_params[2].p_a = 0.3564f;
    muscle_params.gimbal_params[2].p_b = 55.26f;
    muscle_params.gimbal_params[2].p_c = -0.0004f;
    muscle_params.gimbal_params[2].actuator_channels[0] = 12;
    muscle_params.gimbal_params[2].actuator_channels[1] = 13;
    muscle_params.gimbal_params[2].actuator_channels[2] = 14;
    muscle_params.gimbal_params[2].actuator_channels[3] = 15;
    
    // 万向节3参数
    muscle_params.gimbal_params[3].ktheta_p1 = 31.95f;
    muscle_params.gimbal_params[3].ktheta_p2 = -14.9f;
    muscle_params.gimbal_params[3].ktheta_p3 = 6.869f;
    muscle_params.gimbal_params[3].ktheta_p4 = 0.8208f;
    muscle_params.gimbal_params[3].p_a = 0.3321f;
    muscle_params.gimbal_params[3].p_b = 41.97f;
    muscle_params.gimbal_params[3].p_c = -0.0003f;
    muscle_params.gimbal_params[3].actuator_channels[0] = 16;
    muscle_params.gimbal_params[3].actuator_channels[1] = 17;
    muscle_params.gimbal_params[3].actuator_channels[2] = 18;
    muscle_params.gimbal_params[3].actuator_channels[3] = 19;
    
    // 物理约束
    muscle_params.max_muscle_force = 100.0f;
    
    // 构建肌肉雅可比矩阵
    buildMuscleJacobian();
}

/* 构建肌肉雅可比矩阵 */
void StaticsModel::Impl::buildMuscleJacobian() {
    for (uint8_t i = 0; i < MUSCLES_PER_GIMBAL; i++) {
        float theta_i = muscle_params.muscle_angles_rad[i];

        // 作用点位置矢量
        math::Vector3D r_i(0.0f,
                          muscle_params.muscle_arm_length * std::cos(theta_i),
                          muscle_params.muscle_arm_length * std::sin(theta_i));

        // 推力方向矢量（平行于X轴）
        math::Vector3D F_unit(1.0f, 0.0f, 0.0f);

        // 单位力矩矢量：τ = r × F
        math::Vector3D tau_unit = r_i.cross(F_unit);

        // 提取Y和Z分量
        muscle_params.jacobian_muscle[0][i] = tau_unit.y;
        muscle_params.jacobian_muscle[1][i] = tau_unit.z;
    }
}

/* 主计算函数 */
bool StaticsModel::Impl::compute(const std::array<float, 10>& joint_angles_deg,
                                StaticsModelResult& result) {
    result.success = false;

    // 转换角度为弧度
    float joint_angles_rad[MAX_JOINTS];
    for (size_t i = 0; i < MAX_JOINTS; i++) {
        joint_angles_rad[i] = joint_angles_deg[i] * DEG_TO_RAD;
    }

    // 计算前向运动学
    std::array<math::Matrix4x4, MAX_JOINTS + 1> transforms;
    calculateForwardKinematics(joint_angles_rad, transforms);

    // 计算重力矩
    float gravity_torques_local[3][NUM_GIMBALS];
    calculateGravityTorques(joint_angles_rad, transforms, gravity_torques_local);

    // 计算弹性力矩
    float elastic_torques_yz[2][NUM_GIMBALS];
    calculateElasticTorques(joint_angles_deg.data(), elastic_torques_yz);

    // 合并总目标力矩
    float total_target_torques_yz[2][NUM_GIMBALS];
    for (size_t i = 0; i < NUM_GIMBALS; i++) {
        total_target_torques_yz[0][i] = gravity_torques_local[1][i] + elastic_torques_yz[0][i];
        total_target_torques_yz[1][i] = gravity_torques_local[2][i] + elastic_torques_yz[1][i];
    }

    // 求解肌肉力
    float muscle_forces[NUM_GIMBALS][MUSCLES_PER_GIMBAL];
    float max_balance_error = 0.0f;
    solveMuscleForces(total_target_torques_yz, muscle_forces, max_balance_error);

    // 转换为气压值
    float muscle_pressures[NUM_GIMBALS][MUSCLES_PER_GIMBAL];
    convertForcesToPressures(muscle_forces, muscle_pressures);

    // 映射到执行器通道
    float actuator_pressures[MAX_ACTUATOR_CHANNELS];
    mapPressuresToActuators(muscle_pressures, actuator_pressures);

    // 填充结果
    for (size_t i = 0; i < NUM_GIMBALS; i++) {
        for (size_t j = 0; j < MUSCLES_PER_GIMBAL; j++) {
            size_t idx = i * MUSCLES_PER_GIMBAL + j;
            result.muscle_forces[idx] = muscle_forces[i][j];
            result.muscle_pressures[idx] = muscle_pressures[i][j];
        }
    }

    for (size_t i = 0; i < MAX_ACTUATOR_CHANNELS; i++) {
        result.actuator_pressures[i] = actuator_pressures[i];
    }

    result.max_balance_error = max_balance_error;
    result.success = true;
    last_max_balance_error = max_balance_error;

    return true;
}

/* 计算前向运动学 */
void StaticsModel::Impl::calculateForwardKinematics(
    const float joint_angles_rad[MAX_JOINTS],
    std::array<math::Matrix4x4, MAX_JOINTS + 1>& transforms) {

    // 基座变换
    transforms[0] = robot_model.base_transform;

    // 逐步计算每个关节的累积变换
    for (size_t i = 0; i < robot_model.num_joints; i++) {
        const LinkParameters& link = robot_model.links[i];

        // 构建D-H变换矩阵
        math::Matrix4x4 dh_transform;
        dh_transform.setIdentity();

        float theta = link.theta + joint_angles_rad[i];
        float d = link.d / 1000.0f;  // 转换为米
        float a = link.a / 1000.0f;  // 转换为米
        float alpha = link.alpha;

        float ct = std::cos(theta);
        float st = std::sin(theta);
        float ca = std::cos(alpha);
        float sa = std::sin(alpha);

        // 修正D-H变换矩阵
        dh_transform(0, 0) = ct;
        dh_transform(0, 1) = -st;
        dh_transform(0, 2) = 0.0f;
        dh_transform(0, 3) = a;

        dh_transform(1, 0) = st * ca;
        dh_transform(1, 1) = ct * ca;
        dh_transform(1, 2) = -sa;
        dh_transform(1, 3) = -sa * d;

        dh_transform(2, 0) = st * sa;
        dh_transform(2, 1) = ct * sa;
        dh_transform(2, 2) = ca;
        dh_transform(2, 3) = ca * d;

        dh_transform(3, 0) = 0.0f;
        dh_transform(3, 1) = 0.0f;
        dh_transform(3, 2) = 0.0f;
        dh_transform(3, 3) = 1.0f;

        // 累积变换
        transforms[i + 1] = transforms[i] * dh_transform;
    }
}

/* 计算重力矩 */
void StaticsModel::Impl::calculateGravityTorques(
    const float joint_angles_rad[MAX_JOINTS],
    const std::array<math::Matrix4x4, MAX_JOINTS + 1>& transforms,
    float gravity_torques_local[3][NUM_GIMBALS]) {

    // 计算末端执行器位置
    math::Matrix4x4 end_transform = transforms[MAX_JOINTS] * robot_model.tool_transform;
    math::Vector3D end_effector_pos = math::extractTranslation(end_transform);

    // 重力矢量
    math::Vector3D gravity_vector(0.0f, 0.0f, -GRAVITY_CONSTANT);

    // 万向节位置索引
    uint8_t gimbal_transform_indices[NUM_GIMBALS] = {3, 5, 7, 9};

    for (size_t i = 0; i < NUM_GIMBALS; i++) {
        // 获取当前万向节位置
        math::Vector3D current_gimbal_pos = math::extractTranslation(
            transforms[gimbal_transform_indices[i]]);

        math::Vector3D total_torque_world(0.0f, 0.0f, 0.0f);

        // 贡献1：来自所有下游万向节的重力矩
        for (size_t j = i + 1; j < NUM_GIMBALS; j++) {
            math::Vector3D downstream_gimbal_pos = math::extractTranslation(
                transforms[gimbal_transform_indices[j]]);

            math::Vector3D r_vector = downstream_gimbal_pos - current_gimbal_pos;
            math::Vector3D F_gravity = gravity_vector * muscle_params.gimbal_masses[j];
            math::Vector3D torque_j = r_vector.cross(F_gravity);

            total_torque_world = total_torque_world + torque_j;
        }

        // 贡献2：来自末端执行器的重力矩
        math::Vector3D r_end_effector = end_effector_pos - current_gimbal_pos;
        math::Vector3D F_end_gravity = gravity_vector * muscle_params.end_effector_mass;
        math::Vector3D torque_end = r_end_effector.cross(F_end_gravity);
        total_torque_world = total_torque_world + torque_end;

        // 获取当前万向节的旋转矩阵
        math::Matrix3x3 rotation_matrix;
        uint8_t gimbal_joint_idx = muscle_params.gimbal_joints[i];

        if (gimbal_joint_idx == 0) {
            rotation_matrix = transforms[0].extractRotation();
        } else {
            rotation_matrix = transforms[gimbal_joint_idx].extractRotation();
        }

        // 转置旋转矩阵以获得世界到局部的变换
        math::Matrix3x3 rotation_transpose = rotation_matrix.transpose();

        // 变换到局部坐标系
        math::Vector3D tau_local = math::multiply(rotation_transpose, total_torque_world);

        // 存储结果
        gravity_torques_local[0][i] = tau_local.x;
        gravity_torques_local[1][i] = tau_local.y;
        gravity_torques_local[2][i] = tau_local.z;
    }
}

/* 计算弹性力矩 */
void StaticsModel::Impl::calculateElasticTorques(
    const float joint_angles_deg[MAX_JOINTS],
    float elastic_torques_yz[2][NUM_GIMBALS]) {

    for (size_t i = 0; i < NUM_GIMBALS; i++) {
        // 获取万向节对应的两个关节角度
        uint8_t joint1_idx = muscle_params.gimbal_pairs[i][0];
        uint8_t joint2_idx = muscle_params.gimbal_pairs[i][1];

        float theta_j1_deg = joint_angles_deg[joint1_idx];
        float theta_j2_deg = joint_angles_deg[joint2_idx];

        // 转换为等效平面角度
        float alpha_deg, beta_deg;
        convertToEquivalentPlaneAngles(theta_j1_deg, theta_j2_deg, alpha_deg, beta_deg);

        // 计算弹性力矩大小
        const GimbalParams& gimbal_params = muscle_params.gimbal_params[i];
        float tau_elastic_magnitude = getElasticTorqueMagnitude(beta_deg, gimbal_params);

        // 计算弹性力矩矢量
        float alpha_rad = (90.0f + alpha_deg) * DEG_TO_RAD;

        elastic_torques_yz[0][i] = tau_elastic_magnitude * (-std::cos(alpha_rad));
        elastic_torques_yz[1][i] = tau_elastic_magnitude * (-std::sin(alpha_rad));
    }
}

/* 转换为等效平面角度 */
void StaticsModel::Impl::convertToEquivalentPlaneAngles(
    float theta1_deg, float theta2_deg,
    float& alpha_deg, float& beta_deg) {

    float theta1_rad = theta1_deg * DEG_TO_RAD;
    float theta2_rad = theta2_deg * DEG_TO_RAD;

    float cos_beta_val = std::cos(theta1_rad) * std::cos(theta2_rad);

    // 数值稳定性处理
    cos_beta_val = std::max(-1.0f, std::min(1.0f, cos_beta_val));

    float beta_rad = std::acos(cos_beta_val);

    float y_component = -std::sin(theta1_rad) * std::cos(theta2_rad);
    float x_component = std::sin(theta2_rad);

    float alpha_rad;
    if (std::fabs(beta_rad) < 1e-9f) {
        alpha_rad = 0.0f;  // 奇点处理
    } else {
        alpha_rad = std::atan2(y_component, x_component);
    }

    alpha_deg = alpha_rad * RAD_TO_DEG;
    beta_deg = beta_rad * RAD_TO_DEG;
}

/* 获取弹性力矩大小 */
float StaticsModel::Impl::getElasticTorqueMagnitude(
    float beta_deg, const GimbalParams& gimbal_params) {

    float beta_rad = beta_deg * DEG_TO_RAD;

    // 三次多项式
    float tau_magnitude = gimbal_params.ktheta_p1 * beta_rad * beta_rad * beta_rad +
                         gimbal_params.ktheta_p2 * beta_rad * beta_rad +
                         gimbal_params.ktheta_p3 * beta_rad +
                         gimbal_params.ktheta_p4;

    return std::fabs(tau_magnitude);
}

/* 求解肌肉力（QP方法） */
void StaticsModel::Impl::solveMuscleForces(
    const float target_torques_yz[2][NUM_GIMBALS],
    float muscle_forces[NUM_GIMBALS][MUSCLES_PER_GIMBAL],
    float& max_balance_error) {

    max_balance_error = 0.0f;

    // 肌肉组合：由于互斥约束，只有4种可能的组合
    uint8_t combinations[4][2] = {
        {0, 1},  // 肌肉1(45°), 肌肉2(135°)
        {0, 3},  // 肌肉1(45°), 肌肉4(315°)
        {2, 1},  // 肌肉3(225°), 肌肉2(135°)
        {2, 3}   // 肌肉3(225°), 肌肉4(315°)
    };

    for (size_t gimbal_idx = 0; gimbal_idx < NUM_GIMBALS; gimbal_idx++) {
        float target_y = target_torques_yz[0][gimbal_idx];
        float target_z = target_torques_yz[1][gimbal_idx];

        // 检查是否有力矩需求
        float target_norm = std::sqrt(target_y * target_y + target_z * target_z);

        if (target_norm < 1e-6f) {
            // 无力矩需求，肌肉推力为零
            for (size_t j = 0; j < MUSCLES_PER_GIMBAL; j++) {
                muscle_forces[gimbal_idx][j] = 0.0f;
            }
            continue;
        }

        // 初始化最优解
        float best_solution[MUSCLES_PER_GIMBAL] = {0.0f, 0.0f, 0.0f, 0.0f};
        float best_error = 1e6f;

        // 枚举4种互斥组合
        for (size_t comb_idx = 0; comb_idx < 4; comb_idx++) {
            uint8_t muscle1_idx = combinations[comb_idx][0];
            uint8_t muscle2_idx = combinations[comb_idx][1];

            // 构建约简的2x2雅可比矩阵
            math::Matrix2x2 J_reduced;
            J_reduced(0, 0) = muscle_params.jacobian_muscle[0][muscle1_idx];
            J_reduced(0, 1) = muscle_params.jacobian_muscle[0][muscle2_idx];
            J_reduced(1, 0) = muscle_params.jacobian_muscle[1][muscle1_idx];
            J_reduced(1, 1) = muscle_params.jacobian_muscle[1][muscle2_idx];

            // 求解2x2线性方程组
            float target_vec[2] = {-target_y, -target_z};
            float f_reduced[2];

            if (J_reduced.solve(target_vec, f_reduced)) {
                // 检查解的可行性
                if (f_reduced[0] >= 0.0f && f_reduced[1] >= 0.0f &&
                    f_reduced[0] <= muscle_params.max_muscle_force &&
                    f_reduced[1] <= muscle_params.max_muscle_force) {

                    // 构建完整解向量
                    float f_full[MUSCLES_PER_GIMBAL] = {0.0f, 0.0f, 0.0f, 0.0f};
                    f_full[muscle1_idx] = f_reduced[0];
                    f_full[muscle2_idx] = f_reduced[1];

                    // 计算实际产生的力矩
                    float actual_y = 0.0f, actual_z = 0.0f;
                    for (size_t k = 0; k < MUSCLES_PER_GIMBAL; k++) {
                        actual_y += muscle_params.jacobian_muscle[0][k] * f_full[k];
                        actual_z += muscle_params.jacobian_muscle[1][k] * f_full[k];
                    }

                    // 计算平衡误差
                    float error_y = actual_y + target_y;
                    float error_z = actual_z + target_z;
                    float balance_error = std::sqrt(error_y * error_y + error_z * error_z);

                    // 选择最优解
                    if (balance_error < best_error) {
                        best_error = balance_error;
                        std::memcpy(best_solution, f_full, sizeof(f_full));
                    }
                }
            }
        }

        // 使用最优解
        for (size_t j = 0; j < MUSCLES_PER_GIMBAL; j++) {
            muscle_forces[gimbal_idx][j] = best_solution[j];
        }

        // 更新最大平衡误差
        if (best_error > max_balance_error) {
            max_balance_error = best_error;
        }
    }
}

/* 力转气压转换 */
void StaticsModel::Impl::convertForcesToPressures(
    const float muscle_forces[NUM_GIMBALS][MUSCLES_PER_GIMBAL],
    float muscle_pressures[NUM_GIMBALS][MUSCLES_PER_GIMBAL]) {

    for (size_t gimbal_idx = 0; gimbal_idx < NUM_GIMBALS; gimbal_idx++) {
        const GimbalParams& gimbal_params = muscle_params.gimbal_params[gimbal_idx];

        for (size_t muscle_idx = 0; muscle_idx < MUSCLES_PER_GIMBAL; muscle_idx++) {
            float force = muscle_forces[gimbal_idx][muscle_idx];
            muscle_pressures[gimbal_idx][muscle_idx] = forceToPressure(force, gimbal_params);
        }
    }
}

/* 力到气压转换函数 */
float StaticsModel::Impl::forceToPressure(float force, const GimbalParams& gimbal_params) {
    float y = std::fabs(force);

    // 计算判别式
    float discriminant = gimbal_params.p_a * gimbal_params.p_a + 4.0f * gimbal_params.p_c * y;

    if (discriminant < 0) {
        return 0.0f;  // 没有实数解
    }

    // 计算平方根项
    float sqrt_term = std::sqrt(discriminant);

    // 计算解
    float p = gimbal_params.p_b + (-gimbal_params.p_a + sqrt_term) / (2.0f * gimbal_params.p_c);

    // 如果原始力值为负，则返回负压力
    return (force < 0) ? -p : p;
}

/* 映射气压值到执行器通道 */
void StaticsModel::Impl::mapPressuresToActuators(
    const float muscle_pressures[NUM_GIMBALS][MUSCLES_PER_GIMBAL],
    float actuator_pressures[MAX_ACTUATOR_CHANNELS]) {

    // 初始化所有执行器通道为0
    for (size_t i = 0; i < MAX_ACTUATOR_CHANNELS; i++) {
        actuator_pressures[i] = 0.0f;
    }

    // 根据万向节参数中的通道映射分配气压值
    for (size_t gimbal_idx = 0; gimbal_idx < NUM_GIMBALS; gimbal_idx++) {
        const GimbalParams& gimbal_params = muscle_params.gimbal_params[gimbal_idx];

        for (size_t muscle_idx = 0; muscle_idx < MUSCLES_PER_GIMBAL; muscle_idx++) {
            uint8_t channel = gimbal_params.actuator_channels[muscle_idx];
            if (channel < MAX_ACTUATOR_CHANNELS) {
                actuator_pressures[channel] = muscle_pressures[gimbal_idx][muscle_idx];
            }
        }
    }
}

/* StaticsModel 公共接口实现 */

StaticsModel::StaticsModel()
    : pImpl_(std::make_unique<Impl>()) {
}

StaticsModel::~StaticsModel() = default;

bool StaticsModel::computePressures(const std::array<float, 10>& joint_angles_deg,
                                    std::array<float, 24>& pressures_kpa) {
    std::lock_guard<std::mutex> lock(mutex_);

    StaticsModelResult result;
    bool success = pImpl_->compute(joint_angles_deg, result);

    if (success) {
        pressures_kpa = result.actuator_pressures;
    }

    return success;
}

StaticsModelResult StaticsModel::computeDetailed(const std::array<float, 10>& joint_angles_deg) {
    std::lock_guard<std::mutex> lock(mutex_);

    StaticsModelResult result;
    pImpl_->compute(joint_angles_deg, result);

    return result;
}

bool StaticsModel::computeGimbalMusclePressures(const std::array<float, 10>& joint_angles_deg,
                                                 std::array<float, 16>& muscle_pressures_kpa) {
    std::lock_guard<std::mutex> lock(mutex_);

    StaticsModelResult result;
    bool success = pImpl_->compute(joint_angles_deg, result);

    if (success) {
        muscle_pressures_kpa = result.muscle_pressures;
    }

    return success;
}

bool StaticsModel::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pImpl_->initialized;
}

float StaticsModel::getLastMaxBalanceError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pImpl_->last_max_balance_error;
}

void StaticsModel::setMaxMuscleForce(float max_force) {
    std::lock_guard<std::mutex> lock(mutex_);
    pImpl_->muscle_params.max_muscle_force = max_force;
}

void StaticsModel::setGimbalMass(int gimbal_idx, float mass) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (gimbal_idx >= 0 && gimbal_idx < static_cast<int>(NUM_GIMBALS)) {
        pImpl_->muscle_params.gimbal_masses[gimbal_idx] = mass;
    }
}

void StaticsModel::setEndEffectorMass(float mass) {
    std::lock_guard<std::mutex> lock(mutex_);
    pImpl_->muscle_params.end_effector_mass = mass;
}

} // namespace ihb
