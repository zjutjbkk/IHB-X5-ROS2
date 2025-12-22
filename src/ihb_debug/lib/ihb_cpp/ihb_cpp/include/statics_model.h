/**
 * @file statics_model.h
 * @brief 机械臂静力学模型类头文件
 * @details 基于肌肉控制系统的静力学计算，输入10个关节角度，输出气压值
 * @author Converted from STM32 reference implementation
 * @date 2024
 * @version 1.0
 * 
 * 核心功能：
 * - 输入：10个关节角度（度）
 * - 输出：24个执行器通道的气压值（kPa）
 * - 算法：正向运动学、重力矩计算、弹性力矩计算、QP求解、力到气压转换
 */

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <array>
#include <vector>

namespace ihb {

/**
 * @brief 静力学模型详细计算结果
 */
struct StaticsModelResult {
    std::array<float, 16> muscle_forces;      /**< 16个肌肉的力值 (N) */
    std::array<float, 16> muscle_pressures;   /**< 16个肌肉的气压值 (kPa) */
    std::array<float, 24> actuator_pressures; /**< 24个执行器通道的气压值 (kPa) */
    float max_balance_error;                  /**< 最大平衡误差 (N·m) */
    bool success;                             /**< 计算是否成功 */
    
    StaticsModelResult() 
        : max_balance_error(0.0f), success(false) {
        muscle_forces.fill(0.0f);
        muscle_pressures.fill(0.0f);
        actuator_pressures.fill(0.0f);
    }
};

/**
 * @brief 静力学模型类
 * @details 使用PIMPL模式隐藏实现细节，线程安全
 */
class StaticsModel {
public:
    /**
     * @brief 构造函数
     * @details 初始化机器人模型和肌肉控制参数
     */
    StaticsModel();
    
    /**
     * @brief 析构函数
     */
    ~StaticsModel();
    
    // 禁止拷贝和赋值
    StaticsModel(const StaticsModel&) = delete;
    StaticsModel& operator=(const StaticsModel&) = delete;
    
    /**
     * @brief 计算气压值（简化接口）
     * @param joint_angles_deg 10个关节角度（度）
     * @param pressures_kpa 输出：24个执行器通道的气压值（kPa）
     * @return true 计算成功，false 计算失败
     */
    bool computePressures(const std::array<float, 10>& joint_angles_deg,
                         std::array<float, 24>& pressures_kpa);
    
    /**
     * @brief 计算详细结果（完整接口）
     * @param joint_angles_deg 10个关节角度（度）
     * @return 详细计算结果
     */
    StaticsModelResult computeDetailed(const std::array<float, 10>& joint_angles_deg);

    /**
     * @brief 计算万向节肌肉气压值（专用接口）
     * @details 返回4个万向节共16个肌肉的气压值，不经过执行器通道映射
     * @param joint_angles_deg 10个关节角度（度）
     * @param muscle_pressures_kpa 输出：16个万向节肌肉的气压值（kPa）
     *                             索引0-3: 万向节0的4个肌肉
     *                             索引4-7: 万向节1的4个肌肉
     *                             索引8-11: 万向节2的4个肌肉
     *                             索引12-15: 万向节3的4个肌肉
     * @return true 计算成功，false 计算失败
     */
    bool computeGimbalMusclePressures(const std::array<float, 10>& joint_angles_deg,
                                       std::array<float, 16>& muscle_pressures_kpa);

    /**
     * @brief 检查模型是否已初始化
     * @return true 已初始化，false 未初始化
     */
    bool isInitialized() const;
    
    /**
     * @brief 获取上次计算的最大平衡误差
     * @return 最大平衡误差 (N·m)
     */
    float getLastMaxBalanceError() const;
    
    /**
     * @brief 设置最大肌肉推力
     * @param max_force 最大肌肉推力 (N)
     */
    void setMaxMuscleForce(float max_force);
    
    /**
     * @brief 设置万向节质量
     * @param gimbal_idx 万向节索引 (0-3)
     * @param mass 质量 (kg)
     */
    void setGimbalMass(int gimbal_idx, float mass);
    
    /**
     * @brief 设置末端执行器质量
     * @param mass 质量 (kg)
     */
    void setEndEffectorMass(float mass);

private:
    // PIMPL模式：隐藏实现细节
    struct Impl;
    std::unique_ptr<Impl> pImpl_;
    
    // 线程安全保护
    mutable std::mutex mutex_;
};

} // namespace ihb

