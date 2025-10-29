#include "pid.hpp"

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

// 角度模式（Angle/Horizon）配置示例
PIDController<float>::Config AnglePidConfig = {
    .kp = 3.0f,          // 比例增益（需根据飞机调整）
    .ki = 0.05f,         // 积分增益（Betaflight默认较小）
    .kd = 0.01f,         // 微分增益
    .tau = 0.1f,         // 微分滤波器时间常数（等效于Betaflight的D滤波器）
    .limMin = -500.0f,   // 角度模式输出下限（对应angle_mode_limit）
    .limMax = 500.0f,    // 角度模式输出上限
    .limMinInt = -250.0f,// 积分项下限（约为总限制的50%）
    .limMaxInt = 250.0f, // 积分项上限
    .sampleTime = 0.005f // 假设1kHz控制频率（Betaflight默认）
};

} // namespace control