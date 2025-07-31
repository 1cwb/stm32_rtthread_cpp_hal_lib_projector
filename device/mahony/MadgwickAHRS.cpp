#include "MadgwickAHRS.hpp"
/**
 * MadgwickAHRS.cpp
 * 
 * Implementation of Madgwick's IMU and AHRS algorithms.
 */
void MadgwickAHRS::update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz)
{
    // 静止检测
    float gyroMag  = sqrtf(gx*gx + gy*gy + gz*gz);
    float accelMag = sqrtf(ax*ax + ay*ay + az*az);
    bool isStatic  = (gyroMag < 0.1f) && (fabs(accelMag - 1.0f) < 0.2f);

    if (isStatic) {
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);
        q0 = cy; q1 = 0; q2 = 0; q3 = sy;
        getEulerAngles();
        return;
    }

    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    commonUpdate(gx, gy, gz, ax, ay, az, true, mx, my, mz);
}

void MadgwickAHRS::updateIMU(float gx, float gy, float gz,
                             float ax, float ay, float az)
{
    // 静止检测
    float gyroMag  = sqrtf(gx*gx + gy*gy + gz*gz);
    float accelMag = sqrtf(ax*ax + ay*ay + az*az);
    bool isStatic  = (gyroMag < 0.1f) && (fabs(accelMag - 1.0f) < 0.2f);

    if (isStatic) {
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);
        q0 = cy; q1 = 0; q2 = 0; q3 = sy;
        getEulerAngles();
        return;
    }
    commonUpdate(gx, gy, gz, ax, ay, az, false, 0, 0, 0);
}

// 新增：公共部分实现
void MadgwickAHRS::commonUpdate(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    // 1. 陀螺仪积分
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 归一化加速度计
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // ---------- 梯度下降 ----------
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;

        if (useMag) {
            // Madgwick MARG 梯度（使用磁力计）
            // 归一化磁力计
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
            float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
            float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

            // 参考磁场方向
            float hx = mx * (q0q0 + q1q1 - q2q2 - q3q3)
                     + 2.0f * (q1q2*my + q1q3*mz - q0q3*my + q0q2*mz);
            float hy = 2.0f * (q0q3*mx + q1q2*mx + q1q3*my - q0q2*my)
                     + my * (q0q0 - q1q1 + q2q2 - q3*q3);
            float bx = sqrtf(hx*hx + hy*hy);
            float bz = 2.0f * (q0q1*mz - q0q2*mx + q1q2*mz + q2q3*my)
                     + mz * (q0q0 - q1q1 - q2q2 + q3*q3);

            // 梯度下降步长计算
            s0 = -_2q2 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
                 +  _2q1 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
                 + -2.0f * bz * q2 * (bx * (0.5f - q2q2 - q3q3)
                                      + bz * (q1q3 - q0q2) - mx)
                 + (-2.0f * bx * q3 + 2.0f * bz * q1)
                   * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3) - my)
                 +  2.0f * bx * q2 * (bx * (q0q2 + q1q3)
                                      + bz * (0.5f - q1q1 - q2q2) - mz);

            s1 =  _2q3 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
                 +  _2q0 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
                 + -4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
                 +  2.0f * bz * q3 * (bx * (0.5f - q2q2 - q3q3)
                                      + bz * (q1q3 - q0q2) - mx)
                 + ( 2.0f * bx * q2 + 2.0f * bz * q0)
                   * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3) - my)
                 + ( 2.0f * bx * q3 - 4.0f * bz * q1)
                   * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2) - mz);

            s2 = -_2q0 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
                 +  _2q3 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
                 + -4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
                 + (-4.0f * bx * q2 - 2.0f * bz * q0)
                   * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2) - mx)
                 + ( 2.0f * bx * q1 + 2.0f * bz * q3)
                   * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3) - my)
                 + ( 2.0f * bx * q0 - 4.0f * bz * q2)
                   * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2) - mz);

            s3 =  _2q1 * (2.0f * q1q3 - 2.0f * q0q2 - ax)
                 +  _2q2 * (2.0f * q0q1 + 2.0f * q2q3 - ay)
                 + (-4.0f * bx * q3 + 2.0f * bz * q1)
                   * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2) - mx)
                 + (-2.0f * bx * q0 + 2.0f * bz * q2)
                   * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3) - my)
                 +  2.0f * bx * q1 * (bx * (q0q2 + q1q3)
                                      + bz * (0.5f - q1q1 - q2q2) - mz);
        } else {
            // Madgwick IMU 梯度（仅用加速度计）
            float _4q0 = 4.0f * q0;
            float _4q1 = 4.0f * q1;
            float _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1;
            float _8q2 = 8.0f * q2;
            float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
                 - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                 - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        }

        // 归一化步长
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用梯度下降修正
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 2. 四元数积分
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // 3. 四元数归一化
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // 4. 更新欧拉角
    getEulerAngles();
}

void MadgwickAHRS::getEulerAngles() {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2f(sinr_cosp, cosr_cosp);  // 范围[-π, π]

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f) {
        pitch = copysignf(M_PI / 2, sinp); // 处理奇异点（±90°）
    } else {
        pitch = asinf(sinp);               // 正常情况
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2f(siny_cosp, cosy_cosp);    // 范围[-π, π]
}

float MadgwickAHRS::invSqrt(float x) const {
    // Fast inverse square root approximation
    // This is the famous "Quake" fast inverse square root with some modifications
    // for better accuracy and platform compatibility
    
    union {
        float f;
        uint32_t i;
    } conv;
    
    float xhalf = 0.5f * x;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    x = conv.f;
    x = x * (1.5f - xhalf * x * x); // One Newton-Raphson iteration for better accuracy
    
    return x;
}