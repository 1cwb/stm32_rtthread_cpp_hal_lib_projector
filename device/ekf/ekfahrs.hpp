// EKF_AHRS.h - 平台无关的EKF姿态解算库头文件
#ifndef EKF_AHRS_H
#define EKF_AHRS_H

#include <cmath>
#include <cstring>
#include "mklog.hpp"

// ---------------------------------------------------------------------------
//  STM32 零动态内存 Matrix 替换块
// ---------------------------------------------------------------------------
class Matrix {
public:
    /* 生命周期 --------------------------------------------------------------*/
    Matrix(int r, int c) noexcept : rows_(r), cols_(c) {
        // 超过7×7直接停在断言，也可改成返回空对象
        if (r > MAX || c > MAX) { rows_ = cols_ = 0; }
    }
    /* 新增：3 参数构造列向量，c 必须 =1，第三参数只做占位 */
    Matrix(int r, int c, const float* src) noexcept : rows_(r), cols_(c) {
        if (c != 1 || r > MAX) { rows_ = cols_ = 0; return; }  // 只接受列向量
        for (int i = 0; i < r; ++i) buf_[i] = src[i];
    }
    Matrix(const Matrix& o) noexcept : rows_(o.rows_), cols_(o.cols_) {
        for (int i = 0; i < rows_ * cols_; ++i) buf_[i] = o.buf_[i];
    }
    Matrix(Matrix&& o) noexcept : rows_(o.rows_), cols_(o.cols_) {
        for (int i = 0; i < rows_ * cols_; ++i) buf_[i] = o.buf_[i];
    }
    Matrix& operator=(Matrix o) noexcept {
        rows_ = o.rows_; cols_ = o.cols_;
        for (int i = 0; i < rows_ * cols_; ++i) buf_[i] = o.buf_[i];
        return *this;
    }
    ~Matrix() = default;

    /* 基本访问 --------------------------------------------------------------*/
    int  getRows() const { return rows_; }
    int  getCols() const { return cols_; }
    float&       operator()(int i, int j)       { return buf_[i * cols_ + j]; }
    const float& operator()(int i, int j) const { return buf_[i * cols_ + j]; }

    /* 运算接口 --------------------------------------------------------------*/
    Matrix operator*(const Matrix& b) const noexcept {
        Matrix c(rows_, b.cols_);
        gemm(*this, b, c);
        return c;
    }
    Matrix operator+(const Matrix& b) const noexcept {
        Matrix c(rows_, cols_);
        const int n = rows_ * cols_;
        for (int i = 0; i < n; ++i) c.buf_[i] = buf_[i] + b.buf_[i];
        return c;
    }
    /* 新增：矩阵减法 */
    Matrix operator-(const Matrix& b) const noexcept {
        Matrix c(rows_, cols_);
        const int n = rows_ * cols_;
        for (int i = 0; i < n; ++i) c.buf_[i] = buf_[i] - b.buf_[i];
        return c;
    }
    Matrix operator*(float s) const noexcept {
        Matrix c(rows_, cols_);
        const int n = rows_ * cols_;
        for (int i = 0; i < n; ++i) c.buf_[i] = buf_[i] * s;
        return c;
    }
    Matrix transpose() const noexcept {
        Matrix t(cols_, rows_);
        for (int i = 0; i < rows_; ++i)
            for (int j = 0; j < cols_; ++j)
                t(j, i) = (*this)(i, j);
        return t;
    }
    // 3×3 原地转置，EKF 旋转矩阵专用
    void transpose_this_3x3() noexcept {
        std::swap(buf_[1], buf_[3]);
        std::swap(buf_[2], buf_[6]);
        std::swap(buf_[5], buf_[7]);
    }
    static Matrix identity(int n) noexcept {
        Matrix I(n, n);
        for (int i = 0; i < n; ++i) I(i, i) = 1.0f;
        return I;
    }
    Matrix getSubMatrix(int sr, int sc, int nr, int nc) const noexcept {
        Matrix sub(nr, nc);
        for (int i = 0; i < nr; ++i)
            for (int j = 0; j < nc; ++j)
                sub(i, j) = (*this)(sr + i, sc + j);
        return sub;
    }
    void setSubMatrix(int sr, int sc, const Matrix& o) noexcept {
        for (int i = 0; i < o.rows_; ++i)
            for (int j = 0; j < o.cols_; ++j)
                (*this)(sr + i, sc + j) = o(i, j);
    }

private:
    static constexpr int MAX = 7;          // EKF 最大 7×7
    int     rows_ = 0;
    int     cols_ = 0;
    float   buf_[MAX * MAX] = {0.0f};      // 固定栈缓冲区

    static void gemm(const Matrix& a, const Matrix& b, Matrix& c) noexcept {
        const int M = a.rows_;
        const int N = b.cols_;
        const int K = a.cols_;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N; ++j) {
                float s = 0.0f;
                for (int k = 0; k < K; ++k)
                    s += a(i, k) * b(k, j);
                c(i, j) = s;
            }
    }
};
// ---------------------------------------------------------------------------
//  零内存分配 Matrix 替换块结束，下方原 EKF_AHRS.h 内容保持不动
// ---------------------------------------------------------------------------

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