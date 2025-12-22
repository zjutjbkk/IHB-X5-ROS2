# 静力学模型 (StaticsModel) 使用文档

## 概述

`StaticsModel` 类实现了机械臂的静力学模型，用于根据关节角度计算所需的气压值。该模型基于肌肉控制系统，通过正向运动学、重力矩计算、弹性力矩计算和二次规划求解来确定最优的肌肉推力分配。

## 核心功能

- **输入**: 10个关节角度（度）
- **输出**: 24个执行器通道的气压值（kPa）
- **线程安全**: 使用互斥锁保护，支持多线程调用

## 文件结构

```
ihb_cpp/
├── include/
│   ├── statics_model.h      # 静力学模型主接口
│   └── matrix_math.hpp      # C++矩阵数学库
├── src/
│   ├── statics_model.cpp    # 静力学模型实现
│   └── matrix_math.cpp      # 矩阵数学库实现
└── docs/
    └── StaticsModel_README.md  # 本文档
```

## 快速开始

### 1. 基本使用

```cpp
#include "statics_model.h"
#include <array>

using namespace ihb;

int main() {
    // 创建静力学模型实例
    StaticsModel model;
    
    // 检查初始化状态
    if (!model.isInitialized()) {
        std::cerr << "模型初始化失败!" << std::endl;
        return -1;
    }
    
    // 设置关节角度（度）
    std::array<float, 10> joint_angles = {
        0.0f, 0.0f, 10.0f, 0.0f, 10.0f,
        0.0f, 10.0f, 0.0f, 10.0f, 0.0f
    };
    
    // 计算气压值
    std::array<float, 24> pressures;
    bool success = model.computePressures(joint_angles, pressures);
    
    if (success) {
        // 使用气压值
        for (size_t i = 0; i < 24; i++) {
            std::cout << "通道" << i << ": " << pressures[i] << " kPa" << std::endl;
        }
    }
    
    return 0;
}
```

### 2. 获取详细结果

```cpp
#include "statics_model.h"

using namespace ihb;

int main() {
    StaticsModel model;
    
    std::array<float, 10> joint_angles = {/* ... */};
    
    // 获取详细计算结果
    StaticsModelResult result = model.computeDetailed(joint_angles);
    
    if (result.success) {
        std::cout << "最大平衡误差: " << result.max_balance_error << " N·m" << std::endl;
        
        // 16个肌肉力值
        for (size_t i = 0; i < 16; i++) {
            std::cout << "肌肉" << i << "推力: " << result.muscle_forces[i] << " N" << std::endl;
        }
        
        // 16个肌肉气压值
        for (size_t i = 0; i < 16; i++) {
            std::cout << "肌肉" << i << "气压: " << result.muscle_pressures[i] << " kPa" << std::endl;
        }
        
        // 24个执行器通道气压值
        for (size_t i = 0; i < 24; i++) {
            std::cout << "通道" << i << ": " << result.actuator_pressures[i] << " kPa" << std::endl;
        }
    }
    
    return 0;
}
```

### 3. 参数配置

```cpp
#include "statics_model.h"

using namespace ihb;

int main() {
    StaticsModel model;
    
    // 设置最大肌肉推力（N）
    model.setMaxMuscleForce(120.0f);
    
    // 设置万向节质量（kg）
    model.setGimbalMass(0, 0.8f);  // 万向节0
    model.setGimbalMass(1, 0.8f);  // 万向节1
    model.setGimbalMass(2, 0.8f);  // 万向节2
    model.setGimbalMass(3, 0.5f);  // 万向节3
    
    // 设置末端执行器质量（kg）
    model.setEndEffectorMass(0.3f);
    
    // 进行计算
    std::array<float, 10> joint_angles = {/* ... */};
    std::array<float, 24> pressures;
    model.computePressures(joint_angles, pressures);
    
    return 0;
}
```

## API 参考

### StaticsModel 类

#### 构造函数

```cpp
StaticsModel();
```

创建静力学模型实例并自动初始化。

#### 主要方法

##### computePressures

```cpp
bool computePressures(const std::array<float, 10>& joint_angles_deg,
                     std::array<float, 24>& pressures_kpa);
```

计算给定关节角度下的执行器气压值。

- **参数**:
  - `joint_angles_deg`: 10个关节角度（度）
  - `pressures_kpa`: 输出的24个执行器通道气压值（kPa）
- **返回值**: 计算成功返回 `true`，失败返回 `false`
- **线程安全**: 是

##### computeDetailed

```cpp
StaticsModelResult computeDetailed(const std::array<float, 10>& joint_angles_deg);
```

计算详细结果，包括肌肉力、肌肉气压和执行器气压。

- **参数**:
  - `joint_angles_deg`: 10个关节角度（度）
- **返回值**: `StaticsModelResult` 结构体，包含详细计算结果
- **线程安全**: 是

##### isInitialized

```cpp
bool isInitialized() const;
```

检查模型是否已初始化。

- **返回值**: 已初始化返回 `true`，否则返回 `false`
- **线程安全**: 是

##### getLastMaxBalanceError

```cpp
float getLastMaxBalanceError() const;
```

获取上次计算的最大平衡误差。

- **返回值**: 最大平衡误差（N·m）
- **线程安全**: 是

##### setMaxMuscleForce

```cpp
void setMaxMuscleForce(float max_force);
```

设置最大肌肉推力约束。

- **参数**:
  - `max_force`: 最大肌肉推力（N），默认值为 100.0 N
- **线程安全**: 是

##### setGimbalMass

```cpp
void setGimbalMass(int gimbal_idx, float mass);
```

设置指定万向节的质量。

- **参数**:
  - `gimbal_idx`: 万向节索引（0-3）
  - `mass`: 质量（kg）
- **线程安全**: 是

##### setEndEffectorMass

```cpp
void setEndEffectorMass(float mass);
```

设置末端执行器质量。

- **参数**:
  - `mass`: 质量（kg），默认值为 0.23 kg
- **线程安全**: 是

### StaticsModelResult 结构体

```cpp
struct StaticsModelResult {
    std::array<float, 16> muscle_forces;        // 16个肌肉推力（N）
    std::array<float, 16> muscle_pressures;     // 16个肌肉气压（kPa）
    std::array<float, 24> actuator_pressures;   // 24个执行器通道气压（kPa）
    float max_balance_error;                    // 最大平衡误差（N·m）
    bool success;                               // 计算是否成功
};
```

## 系统架构

### 万向节配置

系统包含4个万向节，每个万向节有4个肌肉：

- **万向节0**: 关节2-3，执行器通道 0-3
- **万向节1**: 关节4-5，执行器通道 4-7
- **万向节2**: 关节6-7，执行器通道 12-15
- **万向节3**: 关节8-9，执行器通道 16-19

### 计算流程

1. **前向运动学**: 根据D-H参数计算各关节的变换矩阵
2. **重力矩计算**: 计算各万向节受到的重力矩
3. **弹性力矩计算**: 计算弹性恢复力矩
4. **力矩合成**: 合并重力矩和弹性力矩
5. **QP求解**: 使用二次规划求解最优肌肉推力分配
6. **力到气压转换**: 将肌肉推力转换为气压值
7. **通道映射**: 将肌肉气压映射到执行器通道

## 编译和测试

### 编译

```bash
cd demo
mkdir -p build
cd build
cmake ..
make statics_model_test
```

### 运行测试

```bash
./statics_model_test
```

测试程序包含以下测试用例：

1. **零位姿态测试**: 所有关节角度为0
2. **非零姿态测试**: 多个关节有非零角度
3. **简化接口测试**: 测试 `computePressures` 方法
4. **参数设置测试**: 测试参数配置功能

## 注意事项

1. **角度单位**: 输入的关节角度单位为度（°），内部会自动转换为弧度
2. **气压单位**: 输出的气压值单位为千帕（kPa）
3. **线程安全**: 所有公共方法都是线程安全的，可以在多线程环境中使用
4. **执行器通道**: 共24个通道（0-23），其中16个通道被实际使用
5. **平衡误差**: 平衡误差越小表示力矩平衡越好，通常应小于0.01 N·m

## 参考

- 原始实现: `reference/muscle_controller.c`
- 矩阵数学库: `reference/matrix_math.c`
- 测试代码: `demo/statics_model_test.cpp`

