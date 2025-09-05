#include "imucalibration.hpp"

IMU_Calibration::IMU_Calibration() {
    resetCalibration();
}

void IMU_Calibration::updateBias(float ax, float ay, float az,
                                 float gx, float gy, float gz)
{
    // 1. 加速度计模长，过滤掉明显非静止帧
    float accNorm = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(accNorm - 9.81f) > 0.5f) return;

    // 2. 放进滑动窗口
    auto idx = _calibWindowIndex;
    _accelCalibWindow[idx][0] = ax;
    _accelCalibWindow[idx][1] = ay;
    _accelCalibWindow[idx][2] = az;
    _gyroCalibWindow[idx][0]  = gx;
    _gyroCalibWindow[idx][1]  = gy;
    _gyroCalibWindow[idx][2]  = gz;
    _calibWindowIndex = (idx + 1) % CALIB_WINDOW_SIZE;

    // 3. 先算陀螺仪标准差，用于自适应阈值
    float gMean[3] = {0};
    for (uint8_t i = 0; i < CALIB_WINDOW_SIZE; ++i)
        for (uint8_t j = 0; j < 3; ++j)
            gMean[j] += _gyroCalibWindow[i][j];
    for (uint8_t j = 0; j < 3; ++j) gMean[j] /= CALIB_WINDOW_SIZE;

    float gVar = 0;
    for (uint8_t i = 0; i < CALIB_WINDOW_SIZE; ++i) {
        float d0 = _gyroCalibWindow[i][0] - gMean[0];
        float d1 = _gyroCalibWindow[i][1] - gMean[1];
        float d2 = _gyroCalibWindow[i][2] - gMean[2];
        gVar += d0*d0 + d1*d1 + d2*d2;
    }
    gVar = sqrtf(gVar / CALIB_WINDOW_SIZE);
    float gyroThreshold = 2.0f * gVar + 0.002f;   // 2σ + 小底噪

    // 4. 加权求偏置
    float wSum = 0.0f;
    float accBias[3] = {0}, gyrBias[3] = {0};

    for (uint8_t i = 0; i < CALIB_WINDOW_SIZE; ++i) {
        // 4.1 陀螺仪能量
        float g0 = _gyroCalibWindow[i][0];
        float g1 = _gyroCalibWindow[i][1];
        float g2 = _gyroCalibWindow[i][2];
        if (fabsf(g0) > gyroThreshold ||
            fabsf(g1) > gyroThreshold ||
            fabsf(g2) > gyroThreshold) continue;

        // 4.2 加速度计偏差因子
        float a0 = _accelCalibWindow[i][0];
        float a1 = _accelCalibWindow[i][1];
        float a2 = _accelCalibWindow[i][2];
        float err = fabsf(sqrtf(a0*a0+a1*a1+a2*a2) - 9.81f);
        float w = 1.0f - err / 0.5f;
        w = fmaxf(w, 0.1f);

        // 4.3 累加
        accBias[0] += a0 * w; accBias[1] += a1 * w; accBias[2] += a2 * w;
        gyrBias[0] += g0 * w; gyrBias[1] += g1 * w; gyrBias[2] += g2 * w;
        wSum += w;
    }

    if (wSum > FLT_EPSILON) {
        for (uint8_t j = 0; j < 3; ++j) {
            _accel_bias[j] = accBias[j] / wSum;
            _gyro_bias[j]  = gyrBias[j]  / wSum;
        }
        _accel_calibrated = _gyro_calibrated = true;
    } else {
        // 可选：打印调试信息
    }
}

void IMU_Calibration::resetCalibration() {
    _accel_bias[0] = _accel_bias[1] = _accel_bias[2] = 0.f;
    _gyro_bias[0] = _gyro_bias[1] = _gyro_bias[2] = 0.f;
    
    _accel_calibrated = false;
    _gyro_calibrated = false;
}