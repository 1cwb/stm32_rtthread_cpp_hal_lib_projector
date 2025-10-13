#pragma once

#include <cstdint>
#include <cstring>
#include <array>
#include "vector.hpp"

namespace bfimu {

// Constants from Betaflight
static constexpr float LAMBDA_MIN = 0.999f;      // 最小遗忘因子
static constexpr float P0 = 1000.0f;              // 初始协方差
static constexpr float GYRO_NORM_SQUARED_MIN = 0.01f; // 运动检测阈值
static constexpr uint32_t CALIBRATION_TIME_US = 30000000; // 30秒校准时间
static constexpr uint32_t CALIBRATION_WAIT_US = 2000000;  // 2秒等待时间

// 偏差估计器结构体定义
struct compassBiasEstimator_t {
    float lambda_min, lambda;
    float b[3];
    float theta[4];
    float U[4][4];
    float D[4];
};

class StandaloneMagCalibration {
private:
    // 校准状态变量
    bool magCalProcessActive;
    bool didMovementStart;
    uint64_t magCalEndTime;
    
    // 偏差估计器状态
    compassBiasEstimator_t biasEstimator;
    
    // 校准结果
    Vector3 bias;
    
    // 运动检测相关
    float gyroNormSquaredMin;
    
    // 内部辅助函数
    void initBiasEstimator() {
        std::memset(&biasEstimator, 0, sizeof(biasEstimator));
        
        // 创建单位矩阵
        for (unsigned i = 0; i < 4; i++) {
            biasEstimator.U[i][i] = 1.0f;
        }
        
        // 初始化参数
        biasEstimator.lambda_min = LAMBDA_MIN;
        biasEstimator.lambda = LAMBDA_MIN;
        
        // 设置初始协方差
        for (unsigned i = 0; i < 4; i++) {
            biasEstimator.D[i] = P0;
        }
    }
    
    void updateBiasEstimator(const Vector3& mag) {
        // 更新phi向量
        float phi[4];
        phi[0] = mag.x() * mag.x() + mag.y() * mag.y() + mag.z() * mag.z();
        phi[1] = mag.x();
        phi[2] = mag.y();
        phi[3] = mag.z();

        // 计算误差e
        float e = 1.0f;
        for (unsigned i = 0; i < 4; i++) {
            e -= phi[i] * biasEstimator.theta[i];
        }
        
        // U D U^T 计算
        float f[4];
        float v[4];
        for (unsigned i = 0; i < 4; i++) {
            f[i] = 0.0f;
            for (unsigned j = 0; j <= i; j++) {
                f[i] += biasEstimator.U[j][i] * phi[j];
            }
            v[i] = biasEstimator.D[i] * f[i];
        }
        
        // 第一次迭代
        float alpha[4];
        float k[4] = {0};
        alpha[0] = biasEstimator.lambda + v[0] * f[0];
        biasEstimator.D[0] /= alpha[0];
        k[0] = v[0];
        
        // 剩余迭代
        for (unsigned i = 1; i < 4; i++) {
            alpha[i] = alpha[i - 1] + v[i] * f[i];
            biasEstimator.D[i] *= alpha[i - 1] / (alpha[i] * biasEstimator.lambda);
            for (unsigned j = 0; j < i; j++) {
                float dU = -(f[i] / alpha[i - 1]) * k[j];
                k[j] += v[i] * biasEstimator.U[j][i];
                biasEstimator.U[j][i] += dU;
            }
            k[i] += v[i];
        }
        
        // 参数更新
        for (unsigned i = 0; i < 4; i++) {
            biasEstimator.theta[i] += (k[i] / alpha[3]) * e;
        }
        
        // 偏差更新
        for (unsigned i = 0; i < 3; i++) {
            biasEstimator.b[i] = -0.5f * biasEstimator.theta[i + 1] / biasEstimator.theta[0];
        }
        
        // 计算zn
        float U_v;
        float phiTrans_U_v = 0.0f;
        for (unsigned i = 0; i < 4; i++) {
            U_v = 0.0f;
            for (unsigned j = i; j < 4; j++) {
                U_v += biasEstimator.U[i][j] * v[j];
            }
            phiTrans_U_v += phi[i] * U_v;
        }
        float zn = biasEstimator.lambda / (biasEstimator.lambda + phiTrans_U_v);
        
        // 更新lambda
        biasEstimator.lambda = biasEstimator.lambda_min + (1.0f - biasEstimator.lambda_min) * (zn * zn);
    }

public:
    StandaloneMagCalibration() : 
        magCalProcessActive(false),
        didMovementStart(false),
        magCalEndTime(0),
        bias(0.0f, 0.0f, 0.0f),
        gyroNormSquaredMin(GYRO_NORM_SQUARED_MIN)
    {
        initBiasEstimator();
    }
    
    ~StandaloneMagCalibration() = default;
    
    // 开始校准
    void startCalibration(uint64_t currentTimeUs) {
        magCalProcessActive = true;
        didMovementStart = false;
        magCalEndTime = currentTimeUs + CALIBRATION_WAIT_US;
        
        // 重置偏差估计器
        initBiasEstimator();
        
        // 清零当前偏差
        bias = Vector3(0.0f, 0.0f, 0.0f);
    }
    
    // 检查校准是否完成
    bool isCalibrationComplete() const {
        return !magCalProcessActive;  // 简单直接
    }
    
    // 检查校准是否正在进行
    bool isCalibrationActive() const {
        return magCalProcessActive;
    }
    
    // 主校准函数 - 独立版本
    void performMagnetometerCalibration(
        const Vector3& rawMagData,
        const Vector3& gyroData,  // 改为Vector3类型
        uint64_t currentTimeUs,
        bool useMotionDetection = true)
    {
        if (!magCalProcessActive) {
            return;
        }
        
        // 检查是否还在等待期
        if (currentTimeUs < magCalEndTime) {
            // 检测运动开始
            if (useMotionDetection && !didMovementStart) {
                // 计算陀螺仪范数平方
                float gyroNormSquared = 0.0f;
                for (int axis = 0; axis < 3; axis++) {
                    gyroNormSquared += gyroData[axis] * gyroData[axis];  // 使用Vector3的operator[]
                }
                
                // 转换为弧度并检查是否超过阈值
                gyroNormSquared *= 0.0003046f; // (π/180)^2 近似值
                if (gyroNormSquared > gyroNormSquaredMin) {
                    didMovementStart = true;
                    // 重新开始30秒校准期
                    magCalEndTime = currentTimeUs + CALIBRATION_TIME_US;
                }
            }
        } else {
            // 校准时间结束 - 无论是否检测到运动都完成
            if (didMovementStart) {
                // 如果检测到运动，保存校准结果
                bias = Vector3(biasEstimator.b[0], biasEstimator.b[1], biasEstimator.b[2]);
            } else {
                // 超时未检测到运动，使用当前估计作为结果
                bias = Vector3(biasEstimator.b[0], biasEstimator.b[1], biasEstimator.b[2]);
            }
            // 校准完成，重置状态
            magCalProcessActive = false;
            magCalEndTime = 0;
            return;
        }
        
        // 如果运动已经开始，更新偏差估计
        if (didMovementStart) {
            updateBiasEstimator(rawMagData);
        }
    }

    // 获取当前偏差估计
    const Vector3& getBias() const {
        return bias;
    }
    
    // 获取当前偏差估计（实时更新）
    Vector3 getCurrentBias() const {
        return Vector3(biasEstimator.b[0], biasEstimator.b[1], biasEstimator.b[2]);
    }
    
    // 获取校准进度 (0.0 to 1.0)
    float getProgress(uint64_t currentTimeUs) const {
        if (!magCalProcessActive) {
            return 1.0f;
        }
        
        if (!didMovementStart) {
            // 等待期进度
            if (currentTimeUs >= magCalEndTime) {
                return 0.0f; // 应该开始运动检测了
            }
            return 1.0f - (static_cast<float>(magCalEndTime - currentTimeUs) / CALIBRATION_WAIT_US);
        } else {
            // 校准期进度
            uint64_t calStartTime = magCalEndTime - CALIBRATION_TIME_US;
            uint64_t elapsed = currentTimeUs - calStartTime;
            return static_cast<float>(elapsed) / CALIBRATION_TIME_US;
        }
    }
    
    // 检查是否检测到运动
    bool isMovementDetected() const {
        return didMovementStart;
    }
    
    // 重置校准状态
    void reset() {
        magCalProcessActive = false;
        didMovementStart = false;
        magCalEndTime = 0;
        bias = Vector3(0.0f, 0.0f, 0.0f);
        initBiasEstimator();
    }
    
    // 设置运动检测阈值
    void setMotionThreshold(float threshold) {
        gyroNormSquaredMin = threshold;
    }
    
    // 应用校准到原始数据
    Vector3 applyCalibration(const Vector3& rawMagData) const {
        return rawMagData - bias;
    }
    
    // 获取自适应遗忘因子（调试用）
    float getAdaptiveLambda() const {
        return biasEstimator.lambda;
    }
    
    // 获取theta参数（调试用）
    std::array<float, 4> getThetaParams() const {
        return {biasEstimator.theta[0], biasEstimator.theta[1], 
                biasEstimator.theta[2], biasEstimator.theta[3]};
    }
};

} // namespace bfimu