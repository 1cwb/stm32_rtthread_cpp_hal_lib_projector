#include "pid.hpp"
#include <stdio.h>
namespace control {

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
PIDController<T, UseAntiWindup, UseDerivativeFilter>::PIDController(const Config& config)
    : config_(config),
      integrator_(0),
      prevError_(0),
      differentiator_(0),
      prevMeasurement_(0),
      isSaturated_(false) {
    // 参数校验
    if (config_.sampleTime <= 0) {
        // 在嵌入式系统中，可以替换为适当的错误处理
        config_.sampleTime = 0.01f; // 默认100Hz
    }
}

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
T PIDController<T, UseAntiWindup, UseDerivativeFilter>::update(T setpoint, T measurement) {
    // 计算误差
    T error = setpoint - measurement;

    // 比例项
    T proportional = config_.kp * error;

    // 积分项
    integrator_ = integrator_ + 0.5f * config_.ki * config_.sampleTime * (error + prevError_);

    // 应用积分限制
    if (integrator_ > config_.limMaxInt) {
        integrator_ = config_.limMaxInt;
    } else if (integrator_ < config_.limMinInt) {
        integrator_ = config_.limMinInt;
    }

    // 微分项（带滤波器）
    T derivative;
    if constexpr (UseDerivativeFilter) {
        derivative = (2.0f * config_.kd * (measurement - prevMeasurement_)
                     + (2.0f * config_.tau - config_.sampleTime) * differentiator_)
                     / (2.0f * config_.tau + config_.sampleTime);
    } else {
        derivative = config_.kd * (error - prevError_) / config_.sampleTime;
    }

    // 计算输出
    T output = proportional + integrator_ + derivative;

    // 应用输出限制
    if (output > config_.limMax) {
        output = config_.limMax;
        isSaturated_ = true;
    } else if (output < config_.limMin) {
        output = config_.limMin;
        isSaturated_ = true;
    } else {
        isSaturated_ = false;
    }

    // 应用抗积分饱和
    if constexpr (UseAntiWindup) {
        applyAntiWindup(error, output);
    }

    // 更新状态
    prevError_ = error;
    prevMeasurement_ = measurement;
    differentiator_ = derivative;

    return output;
}

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
void PIDController<T, UseAntiWindup, UseDerivativeFilter>::reset() {
    integrator_ = 0;
    prevError_ = 0;
    differentiator_ = 0;
    prevMeasurement_ = 0;
    isSaturated_ = false;
}

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
void PIDController<T, UseAntiWindup, UseDerivativeFilter>::setTunings(T kp, T ki, T kd) {
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
}

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
void PIDController<T, UseAntiWindup, UseDerivativeFilter>::setOutputLimits(T min, T max) {
    config_.limMin = min;
    config_.limMax = max;
}

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
void PIDController<T, UseAntiWindup, UseDerivativeFilter>::setIntegralLimits(T min, T max) {
    config_.limMinInt = min;
    config_.limMaxInt = max;
}

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
void PIDController<T, UseAntiWindup, UseDerivativeFilter>::setSampleTime(T sampleTime) {
    if (sampleTime > 0) {
        // 调整积分和微分参数以适应新的采样时间
        T ratio = sampleTime / config_.sampleTime;
        integrator_ *= ratio;
        differentiator_ /= ratio;
        config_.sampleTime = sampleTime;
    }
}

template<typename T, bool UseAntiWindup, bool UseDerivativeFilter>
void PIDController<T, UseAntiWindup, UseDerivativeFilter>::applyAntiWindup(T error, T output) {
    if (isSaturated_ && ((error > 0 && integrator_ > 0) || (error < 0 && integrator_ < 0))) {
        // 反向调整积分项
        integrator_ -= 0.1f * config_.ki * error * config_.sampleTime;
    }
}

// 显式实例化常用模板
template class PIDController<float, true, true>;
template class PIDController<float, true, false>;
template class PIDController<float, false, true>;
template class PIDController<float, false, false>;

// 如果需要支持定点数，可以添加如下实例化
// template class PIDController<int32_t, true, true>;
// ...

// 角度环配置 (200Hz) - 同步调整
PIDController<float>::Config anglePidConfig = {
    .kp = 22.5f,         // P增益保持
    .ki = 0.25f,         // I增益保持
    .kd = 0.10f,         // D增益保持
    .tau = 0.01f,        // 保持
    .limMin = -1000.0f,  // 【同步调整】与角速度环的limMin一致
    .limMax = 1000.0f,   // 【同步调整】角度环输出就是角速度环的设定值
    .limMinInt = -50.0f, // 【同步调整】积分限制设为总范围的5%
    .limMaxInt = 50.0f,  // 【同步调整】limMaxInt = 0.05 * limMax
    .sampleTime = 0.005f // 200Hz
};

// 角速度环配置 (200Hz) - 基于物理系统设计
PIDController<float>::Config ratePidConfig = {
    .kp = 0.65f,         // P增益保持
    .ki = 0.40f,         // I增益保持  
    .kd = 0.175f,        // D增益保持
    .tau = 0.005f,       // 保持
    .limMin = -1000.0f,  // 【关键修改】基于实际最大角速度：典型3寸机可达±1000°/s
    .limMax = 1000.0f,   // 【关键修改】对应物理系统的角速度限制
    .limMinInt = -200.0f, // 【同步调整】积分限制设为总范围的20%
    .limMaxInt = 200.0f,  // 【同步调整】limMaxInt = 0.2 * limMax
    .sampleTime = 0.005f // 200Hz
};
} // namespace control