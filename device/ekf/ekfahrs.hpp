// EKF_AHRS.h - 平台无关的EKF姿态解算库头文件
#ifndef EKF_AHRS_H
#define EKF_AHRS_H

#include <cmath>
#include <cstring>
#include "mklog.hpp"

// 平台无关的矩阵运算辅助类
class Matrix {
private:
    float* data;
    int rows, cols;
    
public:
    // 构造函数
    Matrix(int r, int c) : rows(r), cols(c) {
        data = new float[r * c];
        memset(data, 0, sizeof(float) * r * c);
    }
    
    // 拷贝构造函数
    Matrix(const Matrix& other) : rows(other.rows), cols(other.cols) {
        data = new float[rows * cols];
        memcpy(data, other.data, sizeof(float) * rows * cols);
    }
    
    // 析构函数
    ~Matrix() {
        if (data) {
            delete[] data;
            data = nullptr;
        }
    }

    // 获取元素
    float& operator()(int i, int j) {
        return data[i * cols + j];
    }
    
    // 获取常量元素
    const float& operator()(int i, int j) const {
        return data[i * cols + j];
    }
    
    // 矩阵乘法 - 优化版本，避免临时对象
    Matrix operator*(const Matrix& other) const {
        if (cols != other.rows) {
            // 返回空矩阵
            return Matrix(0, 0);
        }
        
        Matrix result(rows, other.cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                float sum = 0.0f;
                for (int k = 0; k < cols; k++) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }
    
    // 矩阵转置
    Matrix transpose() const {
        Matrix result(cols, rows);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }
    // 矩阵加法
    Matrix operator+(const Matrix& other) const {
        if (rows != other.rows || cols != other.cols) {
            // 返回空矩阵或抛出异常
            return Matrix(0, 0);
        }
        Matrix result(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result(i, j) = (*this)(i, j) + other(i, j);
            }
        }
        return result;
    }
    
    // 矩阵标量乘法
    Matrix operator*(float scalar) const {
        Matrix result(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result(i, j) = (*this)(i, j) * scalar;
            }
        }
        return result;
    }
    
    // 矩阵赋值
    Matrix& operator=(const Matrix& other) {
        if (this != &other) {
            delete[] data;
            rows = other.rows;
            cols = other.cols;
            data = new float[rows * cols];
            memcpy(data, other.data, sizeof(float) * rows * cols);
        }
        return *this;
    }
    
    // 获取子矩阵
    Matrix getSubMatrix(int startRow, int startCol, int numRows, int numCols) const {
        Matrix sub(numRows, numCols);
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numCols; j++) {
                sub(i, j) = (*this)(startRow + i, startCol + j);
            }
        }
        return sub;
    }
    
    // 设置子矩阵
    void setSubMatrix(int startRow, int startCol, const Matrix& sub) {
        for (int i = 0; i < sub.getRows(); i++) {
            for (int j = 0; j < sub.getCols(); j++) {
                (*this)(startRow + i, startCol + j) = sub(i, j);
            }
        }
    }
    // 单位矩阵
    static Matrix identity(int n) {
        Matrix I(n, n);
        for (int i = 0; i < n; i++) {
            I(i, i) = 1.0f;
        }
        return I;
    }
    
    int getRows() const { return rows; }
    int getCols() const { return cols; }
};

// EKF姿态解算主类
class EKFAHRS {
private:
    // 状态向量: [q0, q1, q2, q3, bgx, bgy, bgz]^T
    // 其中q0-q3是四元数，bgx-bgz是陀螺仪偏置
    Matrix x;
    
    // 协方差矩阵
    Matrix P;
    
    // 过程噪声协方差
    Matrix Q;
    
    // 测量噪声协方差
    Matrix R_accel;
    Matrix R_mag;
    
    // 上一次更新时间
    float last_time;
    
    // 地球磁场参考向量 (归一化)
    float mag_ref[3];
    
    // 私有方法
    void normalizeVector(float v[3]);
    void quaternionToRotationMatrix(const float q[4], float R[3][3]);
    void updateCovarianceMatrix(float dt);
    
public:
    // 构造函数
    EKFAHRS();
    
    // 初始化滤波器
    void initialize(float ax, float ay, float az, float mx, float my, float mz);
    
    // 更新姿态 (使用IMU数据)
    void update(float gx, float gy, float gz, 
                float ax, float ay, float az,
                float mx, float my, float mz,
                float current_time);
    
    // 获取欧拉角 (弧度)
    void getEulerAngles(float& roll, float& pitch, float& yaw);
    
    // 获取四元数
    void getQuaternion(float& q0, float& q1, float& q2, float& q3);
    
    // 获取陀螺仪偏置
    void getGyroBias(float& bx, float& by, float& bz);
};

#endif // EKF_AHRS_H