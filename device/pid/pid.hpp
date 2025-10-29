/**
 * @file pid_controller.hpp
 * @brief PID控制器实现，适用于STM32和RTOS环境
 */

#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>

namespace control {

/**
 * @brief PID控制器类模板
 * @tparam T 数据类型（float或定点数类型）
 * @tparam UseAntiWindup 是否使用抗积分饱和
 * @tparam UseDerivativeFilter 是否使用微分滤波器
 */
template<typename T = float, bool UseAntiWindup = true, bool UseDerivativeFilter = true>
class PIDController {
public:
    /**
     * @brief PID控制器配置参数
     */
    struct Config {
        T kp;           // 比例增益
        T ki;           // 积分增益
        T kd;           // 微分增益
        T tau;          // 微分滤波器时间常数（仅当UseDerivativeFilter为true时使用）
        T limMin;       // 输出下限
        T limMax;       // 输出上限
        T limMinInt;    // 积分项下限
        T limMaxInt;    // 积分项上限
        T sampleTime;   // 采样时间（秒）
    };

    /**
     * @brief 构造函数
     * @param config PID配置参数
     */
    explicit PIDController(const Config& config);

    /**
     * @brief 更新PID控制器
     * @param setpoint 设定值
     * @param measurement 测量值
     * @return 控制输出
     */
    T update(T setpoint, T measurement);

    /**
     * @brief 重置PID控制器状态
     */
    void reset();

    /**
     * @brief 设置新的PID参数
     * @param kp 新的比例增益
     * @param ki 新的积分增益
     * @param kd 新的微分增益
     */
    void setTunings(T kp, T ki, T kd);

    /**
     * @brief 设置输出限制
     * @param min 最小值
     * @param max 最大值
     */
    void setOutputLimits(T min, T max);

    /**
     * @brief 设置积分项限制
     * @param min 最小值
     * @param max 最大值
     */
    void setIntegralLimits(T min, T max);

    /**
     * @brief 设置采样时间
     * @param sampleTime 采样时间（秒）
     */
    void setSampleTime(T sampleTime);

private:
    Config config_;

    // 控制器状态
    T integrator_;
    T prevError_;           // 上一次误差（用于积分计算）
    T differentiator_;
    T prevMeasurement_;     // 上一次测量值（用于微分计算）

    // 饱和标志（用于抗积分饱和）
    bool isSaturated_;

    /**
     * @brief 检查并应用积分抗饱和
     * @param error 当前误差
     * @param output 当前输出
     */
    void applyAntiWindup(T error, T output);
};
// 在文件末尾，namespace control 内部
// 角度模式（Angle/Horizon）配置声明
extern PIDController<float>::Config AnglePidConfig;

} // namespace control