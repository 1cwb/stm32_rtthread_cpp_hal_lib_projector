#include <cmath>
#include <algorithm>
#include <array>
#include <string>

class PID {
public:
    enum class FilterType {
        NONE,
        PT1,
        BIQUAD
    };

    struct PIDConfig {
        // 基础PID参数
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float ff = 0.0f;
        
        // 限幅参数
        float i_max = 0.0f;
        float d_min = 0.0f;
        float d_max = 0.0f;
        float output_max = 0.0f;
        
        // 滤波参数
        FilterType dterm_filter_type = FilterType::NONE;
        float dterm_lpf_hz = 0.0f;
        float dterm_notch_hz = 0.0f;
        float dterm_notch_cutoff = 0.0f;
        
        // 高级参数
        float setpoint_weight = 1.0f;
        float iterm_relax_cutoff = 0.0f;
        float throttle_boost = 0.0f;
        float tpa_breakpoint = 0.0f;
        float tpa_rate = 0.0f;
        
        float dt = 0.001f;  // 默认控制周期1ms
    };

    PID(const PIDConfig& config) : config_(config) {
        reset();
        updateFilterCoefficients();
    }

    void reset() {
        integral_ = 0.0f;
        last_error_ = 0.0f;
        last_setpoint_ = 0.0f;
        last_measurement_ = 0.0f;
        std::fill(std::begin(dterm_filter_state_), std::end(dterm_filter_state_), 0.0f);
    }

    void setConfig(const PIDConfig& config) {
        config_ = config;
        reset();
        updateFilterCoefficients();
    }

    // Betaflight风格的参数更新 - 不重置状态
    void updateParam(const std::string& param, float value) {
        // 参数范围验证（类似Betaflight的约束检查）
        if (param == "kp") {
            config_.kp = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "ki") {
            config_.ki = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "kd") {
            config_.kd = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "ff") {
            config_.ff = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "i_max") {
            config_.i_max = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "d_min") {
            config_.d_min = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "d_max") {
            config_.d_max = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "output_max") {
            config_.output_max = constrainf(value, 0.0f, 1000.0f);
        }
        else if (param == "dterm_lpf_hz") {
            config_.dterm_lpf_hz = constrainf(value, 0.0f, 500.0f);
            updateFilterCoefficients();
        }
        else if (param == "setpoint_weight") {
            config_.setpoint_weight = constrainf(value, 0.0f, 2.0f);
        }
        else if (param == "iterm_relax_cutoff") {
            config_.iterm_relax_cutoff = constrainf(value, 0.0f, 50.0f);
        }
        else if (param == "tpa_breakpoint") {
            config_.tpa_breakpoint = constrainf(value, 0.0f, 1.0f);
        }
        else if (param == "tpa_rate") {
            config_.tpa_rate = constrainf(value, 0.0f, 1.0f);
        }
        // 注意：不调用 reset()！保持状态连续性
    }
    
    // Betaflight风格的批量参数更新（带平滑过渡）
    void updateParamsSmooth(const PIDConfig& new_config, float transition_rate = 0.02f) {
        // 渐进式参数过渡（类似Betaflight的pidInitSettings）
        config_.kp = config_.kp * (1.0f - transition_rate) + new_config.kp * transition_rate;
        config_.ki = config_.ki * (1.0f - transition_rate) + new_config.ki * transition_rate;
        config_.kd = config_.kd * (1.0f - transition_rate) + new_config.kd * transition_rate;
        config_.ff = config_.ff * (1.0f - transition_rate) + new_config.ff * transition_rate;
        
        // 直接更新非关键参数
        config_.i_max = new_config.i_max;
        config_.d_min = new_config.d_min;
        config_.d_max = new_config.d_max;
        config_.output_max = new_config.output_max;
        config_.setpoint_weight = new_config.setpoint_weight;
        config_.iterm_relax_cutoff = new_config.iterm_relax_cutoff;
        config_.tpa_breakpoint = new_config.tpa_breakpoint;
        config_.tpa_rate = new_config.tpa_rate;
        
        // 滤波器参数特殊处理
        if (config_.dterm_lpf_hz != new_config.dterm_lpf_hz) {
            config_.dterm_lpf_hz = new_config.dterm_lpf_hz;
            updateFilterCoefficients();
        }
    }
    
    float update(float setpoint, float measurement, float throttle = 0.0f) {
        // TPA (Throttle PID Attenuation)
        float tpa_factor = calculateTPAFactor(throttle);
        
        float error = setpoint - measurement;
        
        // 比例项
        float p_term = config_.kp * error * tpa_factor;
        
        // 积分项 (带放松)
        float i_term = updateITerm(error, tpa_factor);
        
        // 微分项 (带设定值权重和滤波)
        float d_term = updateDTerm(setpoint, measurement, tpa_factor);
        
        // 前馈项
        float ff_term = config_.ff * setpoint * tpa_factor;
        
        // 总和
        float output = p_term + i_term + d_term + ff_term;
        
        // 输出限幅
        output = constrain(output, -config_.output_max, config_.output_max);
        
        last_error_ = error;
        last_setpoint_ = setpoint;
        last_measurement_ = measurement;
        
        return output;
    }
    
    const PIDConfig& getConfig() const { return config_; }
    
private:
    static constexpr float M_PI = 3.14159265358979323846f;
    
    static float constrainf(float value, float min_val, float max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
    
    float constrain(float value, float min_val, float max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
    
    // 新增：滤波器系数更新（Betaflight风格）
    void updateFilterCoefficients() {
        if (config_.dterm_filter_type == FilterType::PT1 && config_.dterm_lpf_hz > 0) {
            float rc = 1.0f / (2.0f * M_PI * config_.dterm_lpf_hz);
            filter_coeffs_.alpha = config_.dt / (rc + config_.dt);
        }
    }
    
    float calculateTPAFactor(float throttle) {
        if (throttle <= config_.tpa_breakpoint || config_.tpa_rate <= 0.0f) {
            return 1.0f;
        }
        
        float tpa_range = 1.0f - config_.tpa_breakpoint;
        float tpa_position = (throttle - config_.tpa_breakpoint) / tpa_range;
        float attenuation = config_.tpa_rate * tpa_position;
        
        return std::max(0.0f, 1.0f - attenuation);
    }
    
    float updateITerm(float error, float tpa_factor) {
        // 积分放松
        float iterm_relax_factor = 1.0f;
        if (config_.iterm_relax_cutoff > 0) {
            float error_change = std::abs(error) - std::abs(last_error_);
            if (error_change > config_.iterm_relax_cutoff) {
                iterm_relax_factor = 0.0f; // 完全放松
            } else if (error_change > 0) {
                iterm_relax_factor = 1.0f - error_change / config_.iterm_relax_cutoff;
            }
        }
        
        integral_ += error * config_.dt * iterm_relax_factor * tpa_factor;
        
        // 积分限幅
        if (config_.i_max > 0) {
            integral_ = constrain(integral_, -config_.i_max, config_.i_max);
        }
        
        return config_.ki * integral_;
    }
    
    float updateDTerm(float setpoint, float measurement, float tpa_factor) {
        // 带设定值权重的微分计算
        float setpoint_derivative = (setpoint - last_setpoint_) / config_.dt;
        float measurement_derivative = (measurement - last_measurement_) / config_.dt;
        
        float raw_derivative = setpoint_derivative * config_.setpoint_weight - measurement_derivative;
        
        // 应用滤波
        float filtered_derivative = applyDtermFilter(raw_derivative);
        
        // D项限幅
        if (config_.d_min > 0) {
            filtered_derivative = std::copysign(
                std::max(std::abs(filtered_derivative), config_.d_min), 
                filtered_derivative
            );
        }
        if (config_.d_max > 0) {
            filtered_derivative = constrain(filtered_derivative, -config_.d_max, config_.d_max);
        }
        
        return config_.kd * filtered_derivative * tpa_factor;
    }
    
    float applyDtermFilter(float input) {
        switch (config_.dterm_filter_type) {
            case FilterType::PT1:
                return applyPT1Filter(input);
            case FilterType::BIQUAD:
                return applyBiquadFilter(input);
            case FilterType::NONE:
            default:
                return input;
        }
    }
    
    float applyPT1Filter(float input) {
        if (config_.dterm_lpf_hz <= 0) return input;
        
        dterm_filter_state_[0] = dterm_filter_state_[0] + filter_coeffs_.alpha * (input - dterm_filter_state_[0]);
        return dterm_filter_state_[0];
    }
    
    float applyBiquadFilter(float input) {
        if (config_.dterm_lpf_hz <= 0) return input;
        
        float omega = 2.0f * M_PI * config_.dterm_lpf_hz * config_.dt;
        float sn = std::sin(omega);
        float cs = std::cos(omega);
        float alpha = sn / (2.0f * 0.7071f); // Q = 0.7071
        
        float b0 = (1 - cs) / 2;
        float b1 = 1 - cs;
        float b2 = b0;
        float a0 = 1 + alpha;
        float a1 = -2 * cs;
        float a2 = 1 - alpha;
        
        // 规范化系数
        b0 /= a0; b1 /= a0; b2 /= a0;
        a1 /= a0; a2 /= a0;
        
        float output = b0 * input + b1 * dterm_filter_state_[0] + b2 * dterm_filter_state_[1] 
                      - a1 * dterm_filter_state_[2] - a2 * dterm_filter_state_[3];
        
        // 更新状态
        dterm_filter_state_[1] = dterm_filter_state_[0];
        dterm_filter_state_[0] = input;
        dterm_filter_state_[3] = dterm_filter_state_[2];
        dterm_filter_state_[2] = output;
        
        return output;
    }
    
    // 滤波器系数结构体
    struct FilterCoeffs {
        float alpha = 0.0f;
    } filter_coeffs_;
    
private:
    PIDConfig config_;
    float integral_ = 0.0f;
    float last_error_ = 0.0f;
    float last_setpoint_ = 0.0f;
    float last_measurement_ = 0.0f;
    
    // 滤波器状态
    std::array<float, 4> dterm_filter_state_ = {0};
};

class FlightController {
public:
    struct FlightConfig {
        PID::PIDConfig angle_pid;   // 角度环
        PID::PIDConfig rate_pid;    // 角速率环
        float max_angle = 45.0f;    // 最大角度(度)
        
        // 电机特性参数
        float motor_deadband_low = 0.05f;    // 低油门死区阈值
        float motor_deadband_high = 0.95f;   // 高油门死区阈值
        float motor_deadband_compensation = 0.1f; // 死区补偿值
        float motor_nonlinearity_exponent = 1.5f; // 非线性校正指数
    };
    
    // 机型预设模板
    struct DronePreset {
        std::string name;
        FlightConfig config;
    };

    FlightController(const FlightConfig& config) 
        : config_(config),
          roll_angle_pid(config.angle_pid),
          pitch_angle_pid(config.angle_pid), 
          yaw_angle_pid(config.angle_pid),
          roll_rate_pid(config.rate_pid),
          pitch_rate_pid(config.rate_pid),
          yaw_rate_pid(config.rate_pid) {}

    void reset() {
        roll_angle_pid.reset();
        pitch_angle_pid.reset();
        yaw_angle_pid.reset();
        roll_rate_pid.reset();
        pitch_rate_pid.reset();
        yaw_rate_pid.reset();
    }

    // 应用预设模板
    void applyPreset(const DronePreset& preset) {
        setFlightConfig(preset.config);
    }

    // 获取当前配置
    const FlightConfig& getConfig() const {
        return config_;
    }

    // 设置完整配置
    void setFlightConfig(const FlightConfig& config) {
        config_ = config;
        
        roll_angle_pid.setConfig(config.angle_pid);
        pitch_angle_pid.setConfig(config.angle_pid);
        yaw_angle_pid.setConfig(config.angle_pid);
        
        roll_rate_pid.setConfig(config.rate_pid);
        pitch_rate_pid.setConfig(config.rate_pid);
        yaw_rate_pid.setConfig(config.rate_pid);
    }

    // 空中实时调参
    void updatePIDParam(const std::string& pid_type, const std::string& axis, 
                       const std::string& param, float value) {
        PID* target_pid = nullptr;
        
        // 确定PID类型
        if (pid_type == "angle") {
            if (axis == "roll") target_pid = &roll_angle_pid;
            else if (axis == "pitch") target_pid = &pitch_angle_pid;
            else if (axis == "yaw") target_pid = &yaw_angle_pid;
        } else if (pid_type == "rate") {
            if (axis == "roll") target_pid = &roll_rate_pid;
            else if (axis == "pitch") target_pid = &pitch_rate_pid;
            else if (axis == "yaw") target_pid = &yaw_rate_pid;
        }
        
        if (target_pid) {
            // Betaflight风格：直接原子更新，不通过setConfig
            target_pid->updateParam(param, value);
        }
    }

    // 新增：批量参数更新（类似Betaflight的pidProfile更新）
    void updatePIDProfile(const std::string& pid_type, const PID::PIDConfig& new_config) {
        if (pid_type == "angle") {
            roll_angle_pid.updateParamsSmooth(new_config);
            pitch_angle_pid.updateParamsSmooth(new_config);
            yaw_angle_pid.updateParamsSmooth(new_config);
            config_.angle_pid = new_config;
        } else if (pid_type == "rate") {
            roll_rate_pid.updateParamsSmooth(new_config);
            pitch_rate_pid.updateParamsSmooth(new_config);
            yaw_rate_pid.updateParamsSmooth(new_config);
            config_.rate_pid = new_config;
        }
    }

    void update(float dt, 
                const std::array<float, 3>& angle_setpoint,
                const std::array<float, 3>& angle_measure,
                const std::array<float, 3>& rate_measure,
                float throttle,
                std::array<float, 4>& motor_output) {
        
        // 更新PID控制周期
        updatePIDConfigDt(config_.angle_pid, dt);
        updatePIDConfigDt(config_.rate_pid, dt);
        
        // 角度环PID (外环)
        std::array<float, 3> rate_setpoint;
        rate_setpoint[0] = roll_angle_pid.update(
            constrain(angle_setpoint[0], -config_.max_angle, config_.max_angle), 
            angle_measure[0], 
            throttle
        );
        
        rate_setpoint[1] = pitch_angle_pid.update(
            constrain(angle_setpoint[1], -config_.max_angle, config_.max_angle), 
            angle_measure[1], 
            throttle
        );
        
        rate_setpoint[2] = yaw_angle_pid.update(
            angle_setpoint[2], 
            angle_measure[2], 
            throttle
        );
        
        // 角速度限制
        for (int i = 0; i < 3; i++) {
            rate_setpoint[i] = constrain(rate_setpoint[i], -500.0f, 500.0f);
        }
        
        // 角速率环PID (内环)
        std::array<float, 3> rate_output;
        rate_output[0] = roll_rate_pid.update(rate_setpoint[0], rate_measure[0], throttle);
        rate_output[1] = pitch_rate_pid.update(rate_setpoint[1], rate_measure[1], throttle);
        rate_output[2] = yaw_rate_pid.update(rate_setpoint[2], rate_measure[2], throttle);
        
        // 混控器 (QuadX布局)
        mixOutputs(rate_output, throttle, motor_output);
    }
    
private:
    FlightConfig config_;
    PID roll_angle_pid, pitch_angle_pid, yaw_angle_pid;
    PID roll_rate_pid, pitch_rate_pid, yaw_rate_pid;
    
    void updatePIDConfigDt(PID::PIDConfig& config, float dt) {
        config.dt = dt;
    }
    
    void mixOutputs(const std::array<float, 3>& rate_output, float throttle, 
                   std::array<float, 4>& motor_output) {
        // QuadX 混控器
        float roll = rate_output[0];
        float pitch = rate_output[1];
        float yaw = rate_output[2];
        
        // 基础混控
        motor_output[0] = throttle + pitch + roll - yaw;  // 前左
        motor_output[1] = throttle + pitch - roll + yaw;  // 前右  
        motor_output[2] = throttle - pitch + roll + yaw;  // 后左
        motor_output[3] = throttle - pitch - roll - yaw;  // 后右
        
        // 死区补偿和非线性校正
        for (int i = 0; i < 4; i++) {
            // 死区补偿
            if (motor_output[i] < config_.motor_deadband_low) {
                motor_output[i] += config_.motor_deadband_compensation;
            } else if (motor_output[i] > config_.motor_deadband_high) {
                motor_output[i] -= config_.motor_deadband_compensation;
            }
            
            // 非线性校正
            motor_output[i] = std::pow(motor_output[i], config_.motor_nonlinearity_exponent);
        }
        
        // 归一化到 [0, 1] 范围
        float min_output = *std::min_element(motor_output.begin(), motor_output.end());
        float max_output = *std::max_element(motor_output.begin(), motor_output.end());
        float range = max_output - min_output;
        
        if (range > 1e-6f) {
            for (int i = 0; i < 4; i++) {
                motor_output[i] = (motor_output[i] - min_output) / range;
            }
        }
        
        // 确保在 [0, 1] 范围内
        for (int i = 0; i < 4; i++) {
            motor_output[i] = constrain(motor_output[i], 0.0f, 1.0f);
        }
    }
    
    static float constrain(float value, float min_val, float max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
};

// 四轴穿越机预设模板
namespace DronePresets {
    // 5寸竞速穿越机预设
    const FlightController::DronePreset RACING_5INCH = {
        "5寸竞速穿越机",
        {
            // 角度环配置
            {
                1.5f,   // kp
                0.05f,  // ki
                0.01f,  // kd
                0.0f,   // ff
                50.0f,  // i_max
                0.0f,   // d_min
                100.0f, // d_max
                500.0f, // output_max
                PID::FilterType::PT1, // dterm_filter_type
                40.0f,  // dterm_lpf_hz
                0.0f,   // dterm_notch_hz
                0.0f,   // dterm_notch_cutoff
                0.85f,  // setpoint_weight
                5.0f,   // iterm_relax_cutoff
                0.0f,   // throttle_boost
                0.5f,   // tpa_breakpoint
                0.5f,   // tpa_rate
                0.001f  // dt
            },
            // 角速度环配置
            {
                0.1f,   // kp
                0.06f,  // ki
                0.0008f,// kd
                0.0f,   // ff
                50.0f,  // i_max
                0.0f,   // d_min
                100.0f, // d_max
                500.0f, // output_max
                PID::FilterType::BIQUAD, // dterm_filter_type
                100.0f, // dterm_lpf_hz
                0.0f,   // dterm_notch_hz
                0.0f,   // dterm_notch_cutoff
                1.0f,   // setpoint_weight
                0.0f,   // iterm_relax_cutoff
                0.0f,   // throttle_boost
                0.5f,   // tpa_breakpoint
                0.5f,   // tpa_rate
                0.001f  // dt
            },
            // 其他参数
            55.0f,    // max_angle
            0.05f,    // motor_deadband_low
            0.95f,    // motor_deadband_high
            0.1f,     // motor_deadband_compensation
            1.6f      // motor_nonlinearity_exponent
        }
    };

    // 3寸花飞穿越机预设
    const FlightController::DronePreset FREESTYLE_3INCH = {
        "3寸花飞穿越机",
        {
            // 角度环配置
            {
                1.2f,   // kp
                0.04f,  // ki
                0.008f, // kd
                0.0f,   // ff
                40.0f,  // i_max
                0.0f,   // d_min
                80.0f,  // d_max
                400.0f, // output_max
                PID::FilterType::PT1, // dterm_filter_type
                35.0f,  // dterm_lpf_hz
                0.0f,   // dterm_notch_hz
                0.0f,   // dterm_notch_cutoff
                0.8f,   // setpoint_weight
                4.0f,   // iterm_relax_cutoff
                0.0f,   // throttle_boost
                0.4f,   // tpa_breakpoint
                0.6f,   // tpa_rate
                0.001f  // dt
            },
            // 角速度环配置
            {
                0.07f,  // kp
                0.04f,  // ki
                0.0006f,// kd
                0.0f,   // ff
                40.0f,  // i_max
                0.0f,   // d_min
                80.0f,  // d_max
                400.0f, // output_max
                PID::FilterType::BIQUAD, // dterm_filter_type
                80.0f,  // dterm_lpf_hz
                0.0f,   // dterm_notch_hz
                0.0f,   // dterm_notch_cutoff
                1.0f,   // setpoint_weight
                0.0f,   // iterm_relax_cutoff
                0.0f,   // throttle_boost
                0.4f,   // tpa_breakpoint
                0.6f,   // tpa_rate
                0.001f  // dt
            },
            // 其他参数
            60.0f,    // max_angle
            0.06f,    // motor_deadband_low
            0.94f,    // motor_deadband_high
            0.12f,    // motor_deadband_compensation
            1.4f      // motor_nonlinearity_exponent
        }
    };

    // 7寸长航时穿越机预设
    const FlightController::DronePreset LONG_RANGE_7INCH = {
        "7寸长航时穿越机",
        {
            // 角度环配置
            {
                1.0f,   // kp
                0.03f,  // ki
                0.006f, // kd
                0.0f,   // ff
                30.0f,  // i_max
                0.0f,   // d_min
                60.0f,  // d_max
                300.0f, // output_max
                PID::FilterType::PT1, // dterm_filter_type
                30.0f,  // dterm_lpf_hz
                0.0f,   // dterm_notch_hz
                0.0f,   // dterm_notch_cutoff
                0.75f,  // setpoint_weight
                3.0f,   // iterm_relax_cutoff
                0.0f,   // throttle_boost
                0.6f,   // tpa_breakpoint
                0.4f,   // tpa_rate
                0.001f  // dt
            },
            // 角速度环配置
            {
                0.05f,  // kp
                0.03f,  // ki
                0.0004f,// kd
                0.0f,   // ff
                30.0f,  // i_max
                0.0f,   // d_min
                60.0f,  // d_max
                300.0f, // output_max
                PID::FilterType::BIQUAD, // dterm_filter_type
                60.0f,  // dterm_lpf_hz
                0.0f,   // dterm_notch_hz
                0.0f,   // dterm_notch_cutoff
                1.0f,   // setpoint_weight
                0.0f,   // iterm_relax_cutoff
                0.0f,   // throttle_boost
                0.6f,   // tpa_breakpoint
                0.4f,   // tpa_rate
                0.001f  // dt
            },
            // 其他参数
            40.0f,    // max_angle
            0.04f,    // motor_deadband_low
            0.96f,    // motor_deadband_high
            0.08f,    // motor_deadband_compensation
            1.3f      // motor_nonlinearity_exponent
        }
    };
}
#if 0
// 测试用例
int main() {
    // 使用预设模板创建飞行控制器
    FlightController flight_controller(DronePresets::RACING_5INCH.config);
    
    // 模拟数据
    std::array<float, 3> angle_setpoint = {10.0f, 5.0f, 0.0f};  // 目标角度
    std::array<float, 3> angle_measure = {8.0f, 4.5f, 0.1f};    // 当前角度
    std::array<float, 3> rate_measure = {2.0f, 1.5f, 0.5f};     // 当前角速度
    float throttle = 0.6f;                                      // 油门
    std::array<float, 4> motor_output;                           // 电机输出
    
    // 更新控制器
    flight_controller.update(0.001f, angle_setpoint, angle_measure, rate_measure, throttle, motor_output);
    
    // 输出电机值
    std::cout << "电机输出: ";
    for (float output : motor_output) {
        std::cout << output << " ";
    }
    std::cout << std::endl;
    
    // 空中实时调参示例
    flight_controller.updatePIDParam("angle", "roll", "kp", 1.8f);  // 增加横滚角度环P增益
    flight_controller.updatePIDParam("rate", "pitch", "kd", 0.001f); // 增加俯仰角速度环D增益
    
    // 再次更新控制器
    flight_controller.update(0.001f, angle_setpoint, angle_measure, rate_measure, throttle, motor_output);
    
    std::cout << "调参后电机输出: ";
    for (float output : motor_output) {
        std::cout << output << " ";
    }
    std::cout << std::endl;
    
    // 切换到另一个预设模板
    flight_controller.applyPreset(DronePresets::FREESTYLE_3INCH);
    
    // 再次更新控制器
    flight_controller.update(0.001f, angle_setpoint, angle_measure, rate_measure, throttle, motor_output);
    
    std::cout << "切换预设后电机输出: ";
    for (float output : motor_output) {
        std::cout << output << " ";
    }
    std::cout << std::endl;
    
    return 0;
}
#endif