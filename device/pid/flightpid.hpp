#pragma once
#include "pid.hpp"
#include <array>

class FlightPid
{
public:
    // 修改构造函数，支持角度环和角速度环两套参数
    FlightPid(const control::PIDController<float>::Config& anglePidConfig, 
              const control::PIDController<float>::Config& ratePidConfig)
    : anglePidRoll(anglePidConfig), anglePidPitch(anglePidConfig), anglePidYaw(anglePidConfig),
      ratePidRoll(ratePidConfig), ratePidPitch(ratePidConfig), ratePidYaw(ratePidConfig)
    {}
    
    ~FlightPid() = default;
    
    // 实现角度环+角速度环级联控制
    bool update(const std::array<float, 3>& angleSetpoint, 
                const std::array<float, 3>& angle, const std::array<float, 3>& degRate, 
                std::array<float, 3>& output) 
    {
        // 角度环（外环）计算 - 输出作为角速度设定值
        std::array<float, 3> rateSetpoint;
        rateSetpoint[0] = anglePidRoll.update(angleSetpoint[0], angle[0]);   // 期望Roll角速度
        rateSetpoint[1] = anglePidPitch.update(angleSetpoint[1], angle[1]);  // 期望Pitch角速度  
        rateSetpoint[2] = anglePidYaw.update(angleSetpoint[2], angle[2]);   // 期望Yaw角速度
        
        // 角速度环（内环）计算 - 输出最终控制量
        output[0] = ratePidRoll.update(rateSetpoint[0], degRate[0]);   // Roll输出
        output[1] = ratePidPitch.update(rateSetpoint[1], degRate[1]); // Pitch输出
        output[2] = ratePidYaw.update(rateSetpoint[2], degRate[2]);  // Yaw输出
        
        return true; 
    }

private:
    // 角度环PID控制器
    control::PIDController<float> anglePidRoll;
    control::PIDController<float> anglePidPitch;
    control::PIDController<float> anglePidYaw;
    
    // 角速度环PID控制器
    control::PIDController<float> ratePidRoll;
    control::PIDController<float> ratePidPitch;
    control::PIDController<float> ratePidYaw;
};