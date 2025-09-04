#pragma once
#include <cstdint>
#include <cmath> // 添加cmath头文件以使用fabs
#include <cfloat> // 添加cfloat头文件以使用FLT_EPSILON
class IMU_Calibration {
public:
    IMU_Calibration();
    
    // 更新零偏校准
    void updateBias(float accel_x, float accel_y, float accel_z, 
                   float gyro_x, float gyro_y, float gyro_z);
    float getAccelBias(uint8_t axis) const { return _accel_bias[axis]; }
    float getGyroBias(uint8_t axis) const { return _gyro_bias[axis]; }
    // 重置校准
    void resetCalibration();
    
    // 检查校准状态
    bool isAccelCalibrated() const { return _accel_calibrated; }
    bool isGyroCalibrated() const { return _gyro_calibrated; }
    
private:
    // 在类定义中添加成员变量（放在private区域）
    static constexpr uint8_t CALIB_WINDOW_SIZE = 10;
    float _accelCalibWindow[CALIB_WINDOW_SIZE][3] = {0};
    float _gyroCalibWindow[CALIB_WINDOW_SIZE][3] = {0};
    uint8_t _calibWindowIndex = 0;
    float _accel_bias[3];
    float _gyro_bias[3];
    bool _accel_calibrated;
    bool _gyro_calibrated;
};