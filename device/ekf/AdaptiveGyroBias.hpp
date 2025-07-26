#pragma once
#include "MiniEKF3.hpp"
#include <vector>
#include <algorithm>

class AdaptiveGyroBias {
public:
    // still_thr : rad/s
    // tc        : 零偏学习时间常数，秒
    explicit AdaptiveGyroBias(float still_thr = 0.015f, float tc = 30.0f)
        : still_thr_(still_thr), alpha_(1.0f / (tc * 100.0f)) {}

    // 每次收到传感器后调用
    void update(MiniEKF3& ekf,
                const Vec3& raw_gyro,
                const Vec3& acc,
                float dt) {
        // 1. 判断静止：加速度模接近 1 g 且陀螺很小
        float acc_norm = acc.norm();
        bool moving = (acc_norm < 9.0f || acc_norm > 11.0f) ||
                      (raw_gyro.norm() > still_thr_);

        if (!moving) {
            Vec3 err = raw_gyro - ekf.getGyroBias();
            Vec3 bias = ekf.getGyroBias() + err * alpha_;
            ekf.setGyroBias(bias);
        }
    }

private:
    float still_thr_;
    float alpha_;    // 指数平滑系数
};