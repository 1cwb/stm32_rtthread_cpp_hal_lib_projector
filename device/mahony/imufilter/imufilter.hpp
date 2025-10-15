// adaptive_imu_filter.hpp
#pragma once
#include <stdint.h>
//#include <cmath>
#include "vector.hpp"  // 使用你的Vector3类
#include "maths.hpp"

using namespace bfimu;

class AdaptiveIMUFilter {
public:
    AdaptiveIMUFilter() {
        setSampleRate(1000); // 默认1kHz
    }
    
    // 设置采样频率，自动计算最佳截止频率
    void setSampleRate(uint32_t sample_rate) {
        sample_rate_ = sample_rate;
        calculateOptimalCutoffs();
        resetStates();
    }
    
    // 直接处理Vector3的重载函数
    Vector3 applyAccLpf(const Vector3& acc) {
        // 创建临时数组用于滤波
        float acc_array[3] = {acc.x(), acc.y(), acc.z()};
        applyAccLpf(acc_array);
        return Vector3(acc_array[0], acc_array[1], acc_array[2]);
    }
    
    Vector3 applyGyroLpf(const Vector3& gyro) {
        float gyro_array[3] = {gyro.x(), gyro.y(), gyro.z()};
        applyGyroLpf(gyro_array);
        return Vector3(gyro_array[0], gyro_array[1], gyro_array[2]);
    }
    
    // 磁力计滤波函数
    Vector3 applyMagLpf(const Vector3& mag) {
        float mag_array[3] = {mag.x(), mag.y(), mag.z()};
        applyMagLpf(mag_array);
        return Vector3(mag_array[0], mag_array[1], mag_array[2]);
    }
    
    // 原地修改Vector3的滤波函数（更高效）
    void filterAcc(Vector3& acc) {
        acc_filtered_.x() = acc_alpha_ * acc.x() + (1.0f - acc_alpha_) * acc_filtered_.x();
        acc_filtered_.y() = acc_alpha_ * acc.y() + (1.0f - acc_alpha_) * acc_filtered_.y();
        acc_filtered_.z() = acc_alpha_ * acc.z() + (1.0f - acc_alpha_) * acc_filtered_.z();
        acc = acc_filtered_;
    }
    
    void filterGyro(Vector3& gyro) {
        gyro_filtered_.x() = gyro_alpha_ * gyro.x() + (1.0f - gyro_alpha_) * gyro_filtered_.x();
        gyro_filtered_.y() = gyro_alpha_ * gyro.y() + (1.0f - gyro_alpha_) * gyro_filtered_.y();
        gyro_filtered_.z() = gyro_alpha_ * gyro.z() + (1.0f - gyro_alpha_) * gyro_filtered_.z();
        gyro = gyro_filtered_;
    }
    
    void filterMag(Vector3& mag) {
        mag_filtered_.x() = mag_alpha_ * mag.x() + (1.0f - mag_alpha_) * mag_filtered_.x();
        mag_filtered_.y() = mag_alpha_ * mag.y() + (1.0f - mag_alpha_) * mag_filtered_.y();
        mag_filtered_.z() = mag_alpha_ * mag.z() + (1.0f - mag_alpha_) * mag_filtered_.z();
        mag = mag_filtered_;
    }
    
    // 同时滤波加速度、陀螺仪和磁力计（更高效）
    void filterIMU(Vector3& acc, Vector3& gyro) {
        filterAcc(acc);
        filterGyro(gyro);
    }
    
    // 同时滤波所有传感器数据
    void filterAll(Vector3& acc, Vector3& gyro, Vector3& mag) {
        filterAcc(acc);
        filterGyro(gyro);
        filterMag(mag);
    }
    
    // 获取当前配置信息
    void getConfig(float& acc_cutoff, float& gyro_cutoff, float& sample_rate) {
        acc_cutoff = acc_cutoff_;
        gyro_cutoff = gyro_cutoff_;
        sample_rate = static_cast<float>(sample_rate_);
    }
    
    // 获取包含磁力计的完整配置信息
    void getConfig(float& acc_cutoff, float& gyro_cutoff, float& mag_cutoff, float& sample_rate) {
        acc_cutoff = acc_cutoff_;
        gyro_cutoff = gyro_cutoff_;
        mag_cutoff = mag_cutoff_;
        sample_rate = static_cast<float>(sample_rate_);
    }
    
    // 获取滤波器状态（调试用）
    Vector3 getAccFilteredState() const { return acc_filtered_; }
    Vector3 getGyroFilteredState() const { return gyro_filtered_; }
    Vector3 getMagFilteredState() const { return mag_filtered_; }
    
private:
    void applyAccLpf(float acc[3]) {
        for (int i = 0; i < 3; i++) {
            acc_filtered_[i] = acc_alpha_ * acc[i] + (1.0f - acc_alpha_) * acc_filtered_[i];
            acc[i] = acc_filtered_[i];
        }
    }
    
    void applyGyroLpf(float gyro[3]) {
        for (int i = 0; i < 3; i++) {
            gyro_filtered_[i] = gyro_alpha_ * gyro[i] + (1.0f - gyro_alpha_) * gyro_filtered_[i];
            gyro[i] = gyro_filtered_[i];
        }
    }
    
    void applyMagLpf(float mag[3]) {
        for (int i = 0; i < 3; i++) {
            mag_filtered_[i] = mag_alpha_ * mag[i] + (1.0f - mag_alpha_) * mag_filtered_[i];
            mag[i] = mag_filtered_[i];
        }
    }
    
    void calculateOptimalCutoffs() {
        float fs = static_cast<float>(sample_rate_);
        
        // 加速度计：使用采样频率的8-12%，最大不超过50Hz
        acc_cutoff_ = fs * 0.10f;  // 10% of sample rate
        if (acc_cutoff_ > 50.0f) acc_cutoff_ = 50.0f;
        if (acc_cutoff_ < 8.0f) acc_cutoff_ = 8.0f;
        
        // 陀螺仪：使用采样频率的20-25%，最大不超过200Hz
        gyro_cutoff_ = fs * 0.22f;  // 22% of sample rate
        if (gyro_cutoff_ > 200.0f) gyro_cutoff_ = 200.0f;
        if (gyro_cutoff_ < 15.0f) gyro_cutoff_ = 15.0f;
        
        // 磁力计：使用采样频率的5-8%，最大不超过30Hz
        mag_cutoff_ = fs * 0.06f;  // 6% of sample rate
        if (mag_cutoff_ > 30.0f) mag_cutoff_ = 30.0f;
        if (mag_cutoff_ < 5.0f) mag_cutoff_ = 5.0f;
        
        calculateCoefficients();
    }
    
    void calculateCoefficients() {
        float dt = 1.0f / static_cast<float>(sample_rate_);
        
        // 一阶IIR低通滤波器：alpha = dt / (RC + dt)
        // 其中 RC = 1/(2*pi*cutoff)
        float rc_acc = 1.0f / (2.0f * M_PIf * acc_cutoff_);
        acc_alpha_ = dt / (rc_acc + dt);
        
        float rc_gyro = 1.0f / (2.0f * M_PIf * gyro_cutoff_);
        gyro_alpha_ = dt / (rc_gyro + dt);
        
        float rc_mag = 1.0f / (2.0f * M_PIf * mag_cutoff_);
        mag_alpha_ = dt / (rc_mag + dt);
        
        // 确保稳定性：alpha应在0.001到0.9之间
        if (acc_alpha_ < 0.001f) acc_alpha_ = 0.001f;
        if (acc_alpha_ > 0.9f) acc_alpha_ = 0.9f;
        if (gyro_alpha_ < 0.001f) gyro_alpha_ = 0.001f;
        if (gyro_alpha_ > 0.9f) gyro_alpha_ = 0.9f;
        if (mag_alpha_ < 0.001f) mag_alpha_ = 0.001f;
        if (mag_alpha_ > 0.9f) mag_alpha_ = 0.9f;
    }
    
    void resetStates() {
        acc_filtered_.zero();
        gyro_filtered_.zero();
        mag_filtered_.zero();
    }
    
    uint32_t sample_rate_;
    float acc_cutoff_;
    float gyro_cutoff_;
    float mag_cutoff_;
    float acc_alpha_;
    float gyro_alpha_;
    float mag_alpha_;
    Vector3 acc_filtered_;
    Vector3 gyro_filtered_;
    Vector3 mag_filtered_;
};