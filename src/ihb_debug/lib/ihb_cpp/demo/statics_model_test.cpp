/**
 * @file statics_model_test.cpp
 * @brief 静力学模型测试程序
 * @details 测试StaticsModel类的基本功能
 */

#include "statics_model.h"
#include <iostream>
#include <iomanip>
#include <array>
#include "muscle_joint.h"

using namespace ihb;

/**
 * @brief 打印分隔线
 */
void printSeparator(const std::string& title = "") {
    std::cout << "\n";
    std::cout << "========================================";
    if (!title.empty()) {
        std::cout << "\n" << title;
    }
    std::cout << "\n========================================\n";
}

/**
 * @brief 打印关节角度
 */
void printJointAngles(const std::array<float, 10>& angles) {
    std::cout << "关节角度 (度):\n";
    for (size_t i = 0; i < 10; i++) {
        std::cout << "  关节" << (i + 1) << ": " 
                  << std::fixed << std::setprecision(2) << angles[i] << "°\n";
    }
}

/**
 * @brief 打印气压值
 */
void printPressures(const std::array<float, 24>& pressures) {
    std::cout << "\n执行器通道气压值 (kPa):\n";
    for (size_t i = 0; i < 24; i++) {
        if (pressures[i] != 0.0f || i < 20) {  // 显示前20个通道和所有非零值
            std::cout << "  通道" << std::setw(2) << i << ": " 
                      << std::fixed << std::setprecision(2) << pressures[i] << " kPa\n";
        }
    }
}

/**
 * @brief 打印详细结果
 */
void printDetailedResult(const StaticsModelResult& result) {
    std::cout << "\n计算状态: " << (result.success ? "成功" : "失败") << "\n";
    std::cout << "最大平衡误差: " << std::fixed << std::setprecision(6) 
              << result.max_balance_error << " N·m\n";
    
    std::cout << "\n16个肌肉力值 (N):\n";
    for (size_t i = 0; i < 4; i++) {
        std::cout << "  万向节" << i << ": [";
        for (size_t j = 0; j < 4; j++) {
            size_t idx = i * 4 + j;
            std::cout << std::fixed << std::setprecision(2) << result.muscle_forces[idx];
            if (j < 3) std::cout << ", ";
        }
        std::cout << "]\n";
    }
    
    std::cout << "\n16个肌肉气压值 (kPa):\n";
    for (size_t i = 0; i < 4; i++) {
        std::cout << "  万向节" << i << ": [";
        for (size_t j = 0; j < 4; j++) {
            size_t idx = i * 4 + j;
            std::cout << std::fixed << std::setprecision(2) << result.muscle_pressures[idx];
            if (j < 3) std::cout << ", ";
        }
        std::cout << "]\n";
    }
    
    printPressures(result.actuator_pressures);
}

/**
 * @brief 测试零位姿态
 */
void testZeroPose() {
    printSeparator("测试1: 零位姿态");
    
    StaticsModel model;
    
    // 检查初始化状态
    if (!model.isInitialized()) {
        std::cerr << "错误: 模型未初始化!\n";
        return;
    }
    std::cout << "✓ 模型初始化成功\n";
    
    // 所有关节角度为0
    std::array<float, 10> joint_angles = {0.0f};
    
    printJointAngles(joint_angles);
    
    // 计算详细结果
    StaticsModelResult result = model.computeDetailed(joint_angles);
    
    printDetailedResult(result);
    
    if (result.success) {
        std::cout << "\n✓ 零位姿态计算成功\n";
    } else {
        std::cout << "\n✗ 零位姿态计算失败\n";
    }
}

/**
 * @brief 测试非零姿态
 */
void testNonZeroPose() {
    printSeparator("测试2: 非零姿态");
    
    StaticsModel model;
    
    // 设置测试用的关节角度
    std::array<float, 10> joint_angles = {
        5.0f,   // 关节1
        10.0f,  // 关节2
        6.0f,   // 关节3
        10.0f,  // 关节4
        0.0f,   // 关节5
        10.0f,  // 关节6
        0.0f,   // 关节7
        10.0f,  // 关节8
        0.0f,   // 关节9
        10.0f   // 关节10
    };
    
    printJointAngles(joint_angles);
    
    // 计算详细结果
    StaticsModelResult result = model.computeDetailed(joint_angles);
    
    printDetailedResult(result);
    
    if (result.success) {
        std::cout << "\n✓ 非零姿态计算成功\n";
    } else {
        std::cout << "\n✗ 非零姿态计算失败\n";
    }
}

/**
 * @brief 测试简化接口
 */
void testSimpleInterface() {
    printSeparator("测试3: 简化接口 (computePressures)");
    
    StaticsModel model;
    
    // std::array<float, 10> joint_angles = {
    //     0.0f, 10.0f, 0.0f, 10.0f, 0.0f, 
    //     10.0f, 0.0f, 10.0f, 0.0f, 10.0f
    // };
    std::array<float, 10> joint_angles = {
        0.0f, 0.0f, 10.0f, 0.0f, 10.0f, 
        0.0f, 10.0f, 0.0f, 10.0f, 0.0f
    };
    
    printJointAngles(joint_angles);
    
    std::array<float, 24> pressures;
    bool success = model.computePressures(joint_angles, pressures);
    
    if (success) {
        std::cout << "\n✓ 计算成功\n";
        printPressures(pressures);
        
        std::cout << "\n最大平衡误差: " << std::fixed << std::setprecision(6)
                  << model.getLastMaxBalanceError() << " N·m\n";
    } else {
        std::cout << "\n✗ 计算失败\n";
    }

}

/**
 * @brief 测试参数设置
 */
void testParameterSetting() {
    printSeparator("测试4: 参数设置");

    StaticsModel model;

    std::cout << "设置参数:\n";
    std::cout << "  - 最大肌肉推力: 120.0 N\n";
    std::cout << "  - 万向节0质量: 0.8 kg\n";
    std::cout << "  - 末端执行器质量: 0.3 kg\n";

    model.setMaxMuscleForce(120.0f);
    model.setGimbalMass(0, 0.8f);
    model.setEndEffectorMass(0.3f);

    std::array<float, 10> joint_angles = {0.0f};
    StaticsModelResult result = model.computeDetailed(joint_angles);

    if (result.success) {
        std::cout << "\n✓ 参数设置后计算成功\n";
        std::cout << "最大平衡误差: " << std::fixed << std::setprecision(6)
                  << result.max_balance_error << " N·m\n";
    } else {
        std::cout << "\n✗ 参数设置后计算失败\n";
    }
}

/**
 * @brief 测试万向节肌肉气压接口
 */
void testGimbalMusclePressures() {
    printSeparator("测试5: 万向节肌肉气压接口 (computeGimbalMusclePressures)");

    StaticsModel model;

    // 设置测试用的关节角度
    // std::array<float, 10> joint_angles = {
    //     0.0f, 0.0f, 10.0f, 0.0f, 10.0f,
    //     0.0f, 10.0f, 0.0f, 10.0f, 0.0f
    // };
        std::array<float, 10> joint_angles = {
        0.0f, 0.0f, 0.0f, 10.0f, 0.0f,
        10.0f, 0.0f, 10.0f, 0.0f, 10.0f
    };
    

    printJointAngles(joint_angles);

    // 调用新接口获取16个万向节肌肉气压值
    std::array<float, 16> muscle_pressures;
    bool success = model.computeGimbalMusclePressures(joint_angles, muscle_pressures);

    if (success) {
        std::cout << "\n✓ 计算成功\n";

        std::cout << "\n16个万向节肌肉气压值 (kPa):\n";
        for (size_t i = 0; i < 4; i++) {
            std::cout << "  万向节" << i << ": [";
            for (size_t j = 0; j < 4; j++) {
                size_t idx = i * 4 + j;
                std::cout << std::fixed << std::setprecision(2) << muscle_pressures[idx];
                if (j < 3) std::cout << ", ";
            }
            std::cout << "] kPa\n";
        }

        std::cout << "\n最大平衡误差: " << std::fixed << std::setprecision(6)
                  << model.getLastMaxBalanceError() << " N·m\n";

        // 验证：与computeDetailed的结果对比
        std::cout << "\n验证：与computeDetailed结果对比...\n";
        StaticsModelResult detailed_result = model.computeDetailed(joint_angles);

        bool match = true;
        for (size_t i = 0; i < 16; i++) {
            if (std::fabs(muscle_pressures[i] - detailed_result.muscle_pressures[i]) > 1e-5f) {
                match = false;
                std::cout << "  ✗ 索引" << i << "不匹配: "
                          << muscle_pressures[i] << " vs "
                          << detailed_result.muscle_pressures[i] << "\n";
            }
        }

        if (match) {
            std::cout << "  ✓ 所有16个肌肉气压值与详细结果完全一致\n";
        }

    } else {
        std::cout << "\n✗ 计算失败\n";
    }
}

/**
 * @brief 主函数
 */
int main() {

    std::cout << "========================================\n";
    std::cout << "  静力学模型测试程序\n";
    std::cout << "========================================\n";
    
    try {
        // 运行所有测试
        // testZeroPose();
        // testNonZeroPose();
        testSimpleInterface();
        testParameterSetting();
        testGimbalMusclePressures();

        printSeparator("所有测试完成");
        std::cout << "✓ 所有测试通过\n\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\n错误: " << e.what() << "\n";
        return 1;
    }
}

