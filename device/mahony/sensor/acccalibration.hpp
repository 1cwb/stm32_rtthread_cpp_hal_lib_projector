/*
 * Standalone Accelerometer Calibration Implementation
 * Based on Betaflight's performAcclerationCalibration function
 * This implementation is independent of the Betaflight architecture
 */

#pragma once

#include <cstdint>
#include <cstring>
#include "vector.hpp"
#include "axis.hpp"

namespace bfimu {

// Constants from Betaflight
static constexpr int CALIBRATING_ACC_CYCLES = 400;

class StandaloneAccCalibration {
private:
    // Static variables to maintain state between calls
    uint16_t calibratingA;
    std::array<float, 3> accumulator;
    Vector3 raw;

public:
    StandaloneAccCalibration() : 
        calibratingA(0), 
        accumulator{0, 0, 0},
        raw(0.0f, 0.0f, 0.0f) 
    {
    }

    ~StandaloneAccCalibration() = default;

    // Initialize calibration state
    void startCalibration() {
        calibratingA = CALIBRATING_ACC_CYCLES;
    }
    
    // Check if calibration is complete
    bool isCalibrationComplete() const {
        return calibratingA == 0;
    }
    
    // Check if this is the first calibration cycle
    bool isOnFirstCalibrationCycle() const {
        return calibratingA == CALIBRATING_ACC_CYCLES;
    }
    
    // Check if this is the final calibration cycle
    bool isOnFinalCalibrationCycle() const {
        return calibratingA == 1;
    }
    
    // Main calibration function - standalone version
    void performAccelerationCalibration(
        const Vector3& rawAccData,
        uint16_t acc1G)
    {
        // Process each axis
        for (int axis = 0; axis < 3; axis++) {
            // Reset accumulator at start of calibration
            if (isOnFirstCalibrationCycle()) {
                accumulator[axis] = 0;
            }
            
            // Sum up CALIBRATING_ACC_CYCLES readings
            accumulator[axis] += static_cast<int32_t>(rawAccData[axis]);
        }
        
        // Calculate final values on the last cycle
        if (isOnFinalCalibrationCycle()) {
            // Calculate average with rounding, shift Z down by acc_1G
            raw[0] = static_cast<float>((accumulator[X] + (float)(CALIBRATING_ACC_CYCLES / 2.0f)) / (float)CALIBRATING_ACC_CYCLES);
            raw[1] = static_cast<float>((accumulator[Y] + (float)(CALIBRATING_ACC_CYCLES / 2.0f)) / (float)CALIBRATING_ACC_CYCLES);
            raw[2] = static_cast<float>((accumulator[Z] + (float)(CALIBRATING_ACC_CYCLES / 2.0f)) / (float)CALIBRATING_ACC_CYCLES - acc1G);
            
            // Calibration is now complete
            calibratingA = 0;
            return;
        }
        
        // Decrement calibration counter
        calibratingA--;
    }
    
    // Get current calibration cycle count (for debugging/monitoring)
    uint16_t getCalibrationCycle() const {
        return calibratingA;
    }
    
    // Get calibration results
    const Vector3& getRaw() const {
        return raw;
    }
    
    // Get calibration progress (0.0 to 1.0)
    float getProgress() const {
        if (calibratingA == 0) {
            return 1.0f;
        }
        return 1.0f - (static_cast<float>(calibratingA) / CALIBRATING_ACC_CYCLES);
    }
    
    // Reset calibration state
    void reset() {
        calibratingA = 0;
        accumulator = {0, 0, 0};
        raw = Vector3(0.0f, 0.0f, 0.0f);
    }
};

} // namespace bfimu