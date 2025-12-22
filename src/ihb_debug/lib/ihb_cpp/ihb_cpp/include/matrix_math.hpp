/**
 * @file matrix_math.hpp
 * @brief 轻量级矩阵数学库头文件 (C++版本)
 * @details 为机器人静力学计算优化的矩阵和向量运算库
 * @author Converted from C version for ihb_cpp project
 * @date 2024
 * @version 1.0
 * 
 * 特性：
 * - 使用float类型，优化性能
 * - 自包含，无外部依赖
 * - 针对3x3、4x4矩阵和3x1向量优化
 * - C++命名空间封装
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include <array>

namespace ihb {
namespace math {

/* 数学常量 */
constexpr float MATRIX_EPSILON = 1e-6f;   /**< 矩阵运算精度阈值 */
constexpr float MATRIX_PI = 3.14159265359f;

/* 矩阵维度定义 */
constexpr size_t MATRIX_3X3_SIZE = 9;     /**< 3x3矩阵元素数量 */
constexpr size_t MATRIX_4X4_SIZE = 16;    /**< 4x4矩阵元素数量 */
constexpr size_t VECTOR_3D_SIZE = 3;      /**< 3D向量元素数量 */

/**
 * @brief 3x3矩阵类
 * @details 行主序存储：[0][1][2]
 *                      [3][4][5]
 *                      [6][7][8]
 */
class Matrix3x3 {
public:
    std::array<float, MATRIX_3X3_SIZE> data;
    
    Matrix3x3() { data.fill(0.0f); }
    
    // 设置为单位矩阵
    void setIdentity();
    
    // 设置为零矩阵
    void setZero();
    
    // 矩阵乘法
    Matrix3x3 operator*(const Matrix3x3& other) const;
    
    // 转置
    Matrix3x3 transpose() const;
    
    // 求逆
    bool inverse(Matrix3x3& result) const;
    
    // 访问元素 (i, j)
    float& operator()(size_t i, size_t j) { return data[i * 3 + j]; }
    const float& operator()(size_t i, size_t j) const { return data[i * 3 + j]; }
};

/**
 * @brief 4x4矩阵类
 * @details 行主序存储，用于齐次变换矩阵
 */
class Matrix4x4 {
public:
    std::array<float, MATRIX_4X4_SIZE> data;
    
    Matrix4x4() { data.fill(0.0f); }
    
    // 设置为单位矩阵
    void setIdentity();
    
    // 矩阵乘法
    Matrix4x4 operator*(const Matrix4x4& other) const;
    
    // 提取3x3旋转矩阵
    Matrix3x3 extractRotation() const;
    
    // 访问元素 (i, j)
    float& operator()(size_t i, size_t j) { return data[i * 4 + j]; }
    const float& operator()(size_t i, size_t j) const { return data[i * 4 + j]; }
};

/**
 * @brief 3D向量类
 */
class Vector3D {
public:
    float x, y, z;
    
    Vector3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    // 向量加法
    Vector3D operator+(const Vector3D& other) const;
    
    // 向量减法
    Vector3D operator-(const Vector3D& other) const;
    
    // 标量乘法
    Vector3D operator*(float scalar) const;
    
    // 叉乘
    Vector3D cross(const Vector3D& other) const;
    
    // 点乘
    float dot(const Vector3D& other) const;
    
    // 模长
    float norm() const;
    
    // 归一化
    bool normalize(Vector3D& result) const;
};

/**
 * @brief 2x2矩阵类
 * @details 用于QP求解器中的小规模线性方程组
 */
class Matrix2x2 {
public:
    std::array<float, 4> data;  /**< 矩阵数据：[a11, a12, a21, a22] */
    
    Matrix2x2() { data.fill(0.0f); }
    
    // 求逆
    bool inverse(Matrix2x2& result) const;
    
    // 矩阵向量乘法
    void multiply(const float x[2], float y[2]) const;
    
    // 求解2x2线性方程组 Ax = b
    bool solve(const float b[2], float x[2]) const;
    
    // 访问元素 (i, j)
    float& operator()(size_t i, size_t j) { return data[i * 2 + j]; }
    const float& operator()(size_t i, size_t j) const { return data[i * 2 + j]; }
};

/* 工具函数 */

/**
 * @brief 从4x4变换矩阵提取平移向量
 */
Vector3D extractTranslation(const Matrix4x4& transform);

/**
 * @brief 3x3矩阵与3D向量相乘
 */
Vector3D multiply(const Matrix3x3& mat, const Vector3D& vec);

/**
 * @brief 创建绕X轴旋转的3x3旋转矩阵
 */
Matrix3x3 rotationX(float angle_rad);

/**
 * @brief 创建绕Y轴旋转的3x3旋转矩阵
 */
Matrix3x3 rotationY(float angle_rad);

/**
 * @brief 创建绕Z轴旋转的3x3旋转矩阵
 */
Matrix3x3 rotationZ(float angle_rad);

/**
 * @brief 创建4x4平移变换矩阵
 */
Matrix4x4 translation(float tx, float ty, float tz);

} // namespace math
} // namespace ihb

