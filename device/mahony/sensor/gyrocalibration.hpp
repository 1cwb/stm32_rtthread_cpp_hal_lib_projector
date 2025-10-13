/*
 * Standalone Gyroscope Calibration Implementation
 * Based on Betaflight's performGyroCalibration function
 * This implementation is independent of the Betaflight architecture
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include "vector.hpp"
#include "axis.hpp"

namespace bfimu {

// Constants from Betaflight
static constexpr int GYRO_CALIBRATION_CYCLES = 500;  // 500 cycles at 1kHz = 0.5 seconds
static constexpr float GYRO_MOVEMENT_CALIBRATION_THRESHOLD = 48.0f;  // Movement detection threshold

// Structure for standard deviation calculation (from Betaflight's stdev_t)
struct stdev_s {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
};

class StandaloneGyroCalibration {
private:
    // Static variables to maintain state between calls
    uint16_t calibratingG;
    std::array<std::array<int32_t, 3>, 2> accumulator;  // [sum, variance_sum] for each axis
    std::array<stdev_s, 3> variance;  // Variance calculation for each axis
    Vector3 raw;
    bool calibrationSuccess;

    // Helper functions for variance calculation (from Betaflight)
    void devClear(stdev_s *dev) {
        dev->m_n = 0;
    }

    void devPush(stdev_s *dev, float x) {
        dev->m_n++;
        if (dev->m_n == 1) {
            dev->m_oldM = dev->m_newM = x;
            dev->m_oldS = 0.0f;
        } else {
            dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
            dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
            dev->m_oldM = dev->m_newM;
            dev->m_oldS = dev->m_newS;
        }
    }

    float devVariance(stdev_s *dev)const{
        return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
    }

    float devStandardDeviation(stdev_s *dev)const {
        return std::sqrt(devVariance(dev));
    }

public:
    StandaloneGyroCalibration() :
        calibratingG(0),
        calibrationSuccess(false)
    {
        reset();
    }

    ~StandaloneGyroCalibration() = default;

    // Initialize calibration state
    void startCalibration() {
        calibratingG = GYRO_CALIBRATION_CYCLES;
        calibrationSuccess = false;
        
        // Initialize accumulators and variance calculators
        for (int axis = 0; axis < 3; axis++) {
            accumulator[0][axis] = 0;  // Sum
            accumulator[1][axis] = 0;  // Sum of squares (for variance)
            devClear(&variance[axis]);
        }
        
        raw = Vector3(0.0f, 0.0f, 0.0f);
    }
    
    // Check if calibration is complete
    bool isCalibrationComplete() const {
        return calibratingG == 0;
    }
    
    // Check if calibration was successful
    bool isCalibrationSuccessful() const {
        return calibrationSuccess;
    }
    
    // Check if this is the first calibration cycle
    bool isOnFirstCalibrationCycle() const {
        return calibratingG == GYRO_CALIBRATION_CYCLES;
    }
    
    // Check if this is the final calibration cycle
    bool isOnFinalCalibrationCycle() const {
        return calibratingG == 1;
    }
    
    // Main calibration function - standalone version
    void performGyroCalibration(const Vector3& rawGyroData, float movementThreshold = GYRO_MOVEMENT_CALIBRATION_THRESHOLD) {
        // Process each axis
        for (int axis = 0; axis < 3; axis++) {
            // Reset accumulators at start of calibration
            if (isOnFirstCalibrationCycle()) {
                accumulator[0][axis] = 0;
                accumulator[1][axis] = 0;
                devClear(&variance[axis]);
            }
            
            // Sum up readings and calculate variance
            float gyroValue = rawGyroData[axis];
            accumulator[0][axis] += static_cast<int32_t>(gyroValue);
            devPush(&variance[axis], gyroValue);
        }
        
        // Check for movement on the final cycle
        if (isOnFinalCalibrationCycle()) {
            calibrationSuccess = true;
            
            // Check each axis for excessive movement
            for (int axis = 0; axis < 3; axis++) {
                float stdDev = devStandardDeviation(&variance[axis]);
                
                // If movement detected (standard deviation too high), calibration fails
                if (stdDev > movementThreshold) {
                    calibrationSuccess = false;
                    calibratingG = GYRO_CALIBRATION_CYCLES;  // Restart calibration
                    return;
                }
            }
            
            // If no excessive movement, calculate final offset values
            if (calibrationSuccess) {
                raw[0] = static_cast<float>(accumulator[0][X]) / GYRO_CALIBRATION_CYCLES;
                raw[1] = static_cast<float>(accumulator[0][Y]) / GYRO_CALIBRATION_CYCLES;
                raw[2] = static_cast<float>(accumulator[0][Z]) / GYRO_CALIBRATION_CYCLES;
                
                // Calibration is now complete
                calibratingG = 0;
                return;
            }
        }
        
        // Decrement calibration counter
        calibratingG--;
    }
    
    // Get current calibration cycle count (for debugging/monitoring)
    uint16_t getCalibrationCycle() const {
        return calibratingG;
    }
    
    // Get calibration results (gyro offsets)
    const Vector3& getRaw() const {
        return raw;
    }
    
    // Get calibration progress (0.0 to 1.0)
    float getProgress() const {
        if (calibratingG == 0) {
            return 1.0f;
        }
        return 1.0f - (static_cast<float>(calibratingG) / GYRO_CALIBRATION_CYCLES);
    }
    
    // Get standard deviation for a specific axis (for debugging)
    float getStandardDeviation(int axis) const {
        if (axis >= 0 && axis < 3) {
            return devStandardDeviation(const_cast<stdev_s*>(&variance[axis]));
        }
        return 0.0f;
    }
    
    // Check if movement was detected on a specific axis
    bool isMovementDetected(int axis, float threshold = GYRO_MOVEMENT_CALIBRATION_THRESHOLD) const {
        if (axis >= 0 && axis < 3) {
            return getStandardDeviation(axis) > threshold;
        }
        return false;
    }
    
    // Apply calibration compensation to raw gyroscope data
    Vector3 applyCalibration(const Vector3& rawGyroData) const {
        // Subtract bias from raw data
        return Vector3(
            rawGyroData[0] - raw[0],
            rawGyroData[1] - raw[1], 
            rawGyroData[2] - raw[2]
        );
    }
    
    // Get calibrated data (convenience method)
    Vector3 getCalibratedData(const Vector3& rawGyroData) const {
        return applyCalibration(rawGyroData);
    }
    
    // Reset calibration state
    void reset() {
        calibratingG = 0;
        calibrationSuccess = false;
        
        for (int axis = 0; axis < 3; axis++) {
            accumulator[0][axis] = 0;
            accumulator[1][axis] = 0;
            devClear(&variance[axis]);
        }
        
        raw = Vector3(0.0f, 0.0f, 0.0f);
    }
};

} // namespace bfimu