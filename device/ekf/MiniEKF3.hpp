#pragma once
#include <cstdint>
#include <cmath>

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    static Vec3 zero() { return Vec3(0,0,0); }
    float norm() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 normalized() const { float n=norm(); return n>0 ? Vec3(x/n,y/n,z/n) : *this; }
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x,y+v.y,z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x,y-v.y,z-v.z); }
    Vec3 operator*(float s) const { return Vec3(x*s,y*s,z*s); }
    Vec3 operator/(float s) const { return Vec3(x/s,y/s,z/s); }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator/=(float s)       { x /= s;  y /= s;  z /= s;  return *this; }
};

class MiniEKF3 {
public:
    MiniEKF3();
    void setGyroBias(const Vec3& b) { state_.gyro_bias = b; }
    const Vec3& getGyroBias() const { return state_.gyro_bias; }
    // 需要周期性调用：
    // dt : 采样周期 (秒)
    // gyro : rad/s
    // acc  : m/s²
    // mag  : µT (未校准也可)
    // baro : m   （可用海拔或相对高度）
    void update(float dt,
                const Vec3& gyro,
                const Vec3& acc,
                const Vec3& mag,
                float baro);

    // 获取当前姿态（rad）
    float getRoll()  const { return roll_;  }
    float getPitch() const { return pitch_; }
    float getYaw()   const { return yaw_;   }

private:
    void predict(float dt, const Vec3& gyro);
    void updateAcc(const Vec3& acc);
    void updateMag(const Vec3& mag);

    // 状态：四元数 + 陀螺零偏
    struct State {
        float q[4];      // [w, x, y, z]
        Vec3  gyro_bias;
    } state_;

    float roll_, pitch_, yaw_;

    // 协方差（仅对角，简化）
    float P_q[4];      // 四元数误差
    float P_bias[3];   // 陀螺零偏

    // 噪声参数
    static constexpr float gyro_noise = 1e-3f;
    static constexpr float acc_noise  = 1e-1f;
    static constexpr float mag_noise  = 5e-2f;

    /* 状态向量:  [q0 q1 q2 q3 bx by bz]^T  (7×1) */
    float x_[7];

    /* 7×7 协方差矩阵，按行优先展平：P[i][j] = P[i*7+j] */
    float P_[49];

    /* 噪声常数 */
    static constexpr float SIGMA_Q   = 1e-3f;  // 陀螺噪声
    static constexpr float SIGMA_BG  = 1e-6f;  // 零偏随机游走
    static constexpr float SIGMA_ACC = 1e-1f;  // 加速度计
};
