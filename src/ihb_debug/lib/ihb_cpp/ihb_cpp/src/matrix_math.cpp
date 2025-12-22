/**
 * @file matrix_math.cpp
 * @brief 轻量级矩阵数学库实现 (C++版本)
 * @details 为机器人静力学计算优化的矩阵和向量运算库实现
 */

#include "matrix_math.hpp"
#include <cmath>
#include <cstring>

namespace ihb {
namespace math {

/* Matrix3x3 实现 */

void Matrix3x3::setIdentity() {
    data.fill(0.0f);
    data[0] = 1.0f;  // [0,0]
    data[4] = 1.0f;  // [1,1]
    data[8] = 1.0f;  // [2,2]
}

void Matrix3x3::setZero() {
    data.fill(0.0f);
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const {
    Matrix3x3 result;
    
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            result.data[i * 3 + j] = 0.0f;
            for (size_t k = 0; k < 3; k++) {
                result.data[i * 3 + j] += data[i * 3 + k] * other.data[k * 3 + j];
            }
        }
    }
    
    return result;
}

Matrix3x3 Matrix3x3::transpose() const {
    Matrix3x3 result;
    
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            result.data[j * 3 + i] = data[i * 3 + j];
        }
    }
    
    return result;
}

bool Matrix3x3::inverse(Matrix3x3& result) const {
    // 使用伴随矩阵方法计算3x3矩阵的逆
    const float* m = data.data();
    
    // 计算行列式
    float det = m[0] * (m[4] * m[8] - m[5] * m[7]) -
                m[1] * (m[3] * m[8] - m[5] * m[6]) +
                m[2] * (m[3] * m[7] - m[4] * m[6]);
    
    if (std::fabs(det) < MATRIX_EPSILON) {
        return false;  // 矩阵奇异
    }
    
    float inv_det = 1.0f / det;
    
    // 计算伴随矩阵并除以行列式
    result.data[0] = (m[4] * m[8] - m[5] * m[7]) * inv_det;
    result.data[1] = (m[2] * m[7] - m[1] * m[8]) * inv_det;
    result.data[2] = (m[1] * m[5] - m[2] * m[4]) * inv_det;
    result.data[3] = (m[5] * m[6] - m[3] * m[8]) * inv_det;
    result.data[4] = (m[0] * m[8] - m[2] * m[6]) * inv_det;
    result.data[5] = (m[2] * m[3] - m[0] * m[5]) * inv_det;
    result.data[6] = (m[3] * m[7] - m[4] * m[6]) * inv_det;
    result.data[7] = (m[1] * m[6] - m[0] * m[7]) * inv_det;
    result.data[8] = (m[0] * m[4] - m[1] * m[3]) * inv_det;
    
    return true;
}

/* Matrix4x4 实现 */

void Matrix4x4::setIdentity() {
    data.fill(0.0f);
    data[0]  = 1.0f;  // [0,0]
    data[5]  = 1.0f;  // [1,1]
    data[10] = 1.0f;  // [2,2]
    data[15] = 1.0f;  // [3,3]
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
    Matrix4x4 result;
    
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            result.data[i * 4 + j] = 0.0f;
            for (size_t k = 0; k < 4; k++) {
                result.data[i * 4 + j] += data[i * 4 + k] * other.data[k * 4 + j];
            }
        }
    }
    
    return result;
}

Matrix3x3 Matrix4x4::extractRotation() const {
    Matrix3x3 rotation;
    const float* t = data.data();
    float* r = rotation.data.data();
    
    r[0] = t[0];  r[1] = t[1];  r[2] = t[2];   // 第一行
    r[3] = t[4];  r[4] = t[5];  r[5] = t[6];   // 第二行
    r[6] = t[8];  r[7] = t[9];  r[8] = t[10];  // 第三行
    
    return rotation;
}

/* Vector3D 实现 */

Vector3D Vector3D::operator+(const Vector3D& other) const {
    return Vector3D(x + other.x, y + other.y, z + other.z);
}

Vector3D Vector3D::operator-(const Vector3D& other) const {
    return Vector3D(x - other.x, y - other.y, z - other.z);
}

Vector3D Vector3D::operator*(float scalar) const {
    return Vector3D(x * scalar, y * scalar, z * scalar);
}

Vector3D Vector3D::cross(const Vector3D& other) const {
    return Vector3D(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

float Vector3D::dot(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

float Vector3D::norm() const {
    return std::sqrt(x * x + y * y + z * z);
}

bool Vector3D::normalize(Vector3D& result) const {
    float n = norm();
    
    if (n < MATRIX_EPSILON) {
        return false;  // 零向量无法归一化
    }
    
    float inv_norm = 1.0f / n;
    result = (*this) * inv_norm;
    return true;
}

/* Matrix2x2 实现 */

bool Matrix2x2::inverse(Matrix2x2& result) const {
    // 2x2矩阵逆：[a11 a12]^(-1) = 1/det * [ a22 -a12]
    //           [a21 a22]              [-a21  a11]
    
    float det = data[0] * data[3] - data[1] * data[2];
    
    if (std::fabs(det) < MATRIX_EPSILON) {
        return false;  // 矩阵奇异
    }
    
    float inv_det = 1.0f / det;
    
    result.data[0] =  data[3] * inv_det;  // a22
    result.data[1] = -data[1] * inv_det;  // -a12
    result.data[2] = -data[2] * inv_det;  // -a21
    result.data[3] =  data[0] * inv_det;  // a11
    
    return true;
}

void Matrix2x2::multiply(const float x[2], float y[2]) const {
    y[0] = data[0] * x[0] + data[1] * x[1];
    y[1] = data[2] * x[0] + data[3] * x[1];
}

bool Matrix2x2::solve(const float b[2], float x[2]) const {
    // 求解2x2线性方程组 Ax = b
    // 使用克莱姆法则实现
    
    // 计算行列式 det(A) = a11*a22 - a12*a21
    float det = data[0] * data[3] - data[1] * data[2];
    
    // 检查矩阵是否奇异
    if (std::fabs(det) < MATRIX_EPSILON) {
        return false;  // 矩阵奇异，无唯一解
    }
    
    // 使用克莱姆法则求解
    float inv_det = 1.0f / det;
    
    x[0] = (b[0] * data[3] - b[1] * data[1]) * inv_det;  // x1
    x[1] = (data[0] * b[1] - data[2] * b[0]) * inv_det;  // x2
    
    return true;
}

/* 工具函数实现 */

Vector3D extractTranslation(const Matrix4x4& transform) {
    return Vector3D(
        transform.data[3],   // [0,3]
        transform.data[7],   // [1,3]
        transform.data[11]   // [2,3]
    );
}

Vector3D multiply(const Matrix3x3& mat, const Vector3D& vec) {
    const float* m = mat.data.data();
    
    return Vector3D(
        m[0] * vec.x + m[1] * vec.y + m[2] * vec.z,
        m[3] * vec.x + m[4] * vec.y + m[5] * vec.z,
        m[6] * vec.x + m[7] * vec.y + m[8] * vec.z
    );
}

Matrix3x3 rotationX(float angle_rad) {
    float c = std::cos(angle_rad);
    float s = std::sin(angle_rad);
    
    Matrix3x3 mat;
    mat.setZero();
    mat.data[0] = 1.0f;
    mat.data[4] = c;   mat.data[5] = -s;
    mat.data[7] = s;   mat.data[8] = c;
    
    return mat;
}

Matrix3x3 rotationY(float angle_rad) {
    float c = std::cos(angle_rad);
    float s = std::sin(angle_rad);
    
    Matrix3x3 mat;
    mat.setZero();
    mat.data[0] = c;   mat.data[2] = s;
    mat.data[4] = 1.0f;
    mat.data[6] = -s;  mat.data[8] = c;
    
    return mat;
}

Matrix3x3 rotationZ(float angle_rad) {
    float c = std::cos(angle_rad);
    float s = std::sin(angle_rad);
    
    Matrix3x3 mat;
    mat.setZero();
    mat.data[0] = c;   mat.data[1] = -s;
    mat.data[3] = s;   mat.data[4] = c;
    mat.data[8] = 1.0f;
    
    return mat;
}

Matrix4x4 translation(float tx, float ty, float tz) {
    Matrix4x4 mat;
    mat.setIdentity();
    mat.data[3]  = tx;  // [0,3]
    mat.data[7]  = ty;  // [1,3]
    mat.data[11] = tz;  // [2,3]
    
    return mat;
}

} // namespace math
} // namespace ihb

