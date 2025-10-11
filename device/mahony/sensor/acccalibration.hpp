/*
 * Standalone Accelerometer Calibration Implementation
 * Based on Betaflight's performAcclerationCalibration function
 * This implementation is independent of the Betaflight architecture
 */

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
    int32_t accumulator[3];
    vector3_t raw;
public:
    StandaloneAccCalibration() : calibratingA(0), accumulator{0, 0, 0} {
    }

    ~StandaloneAccCalibration() {
    }

    // Initialize calibration state
    void startCalibration() {
        calibratingA = CALIBRATING_ACC_CYCLES;
    }
    
    // Check if calibration is complete
    bool isCalibrationComplete() {
        return calibratingA == 0;
    }
    
    // Check if this is the first calibration cycle
    bool isOnFirstCalibrationCycle() {
        return calibratingA == CALIBRATING_ACC_CYCLES;
    }
    
    // Check if this is the final calibration cycle
    bool isOnFinalCalibrationCycle() {
        return calibratingA == 1;
    }
    
    // Main calibration function - standalone version
    void performAccelerationCalibration(
        const vector3_t& rawAccData,
        uint16_t acc1G)
    {
        // Process each axis
        for (int axis = 0; axis < 3; axis++) {
            // Reset accumulator at start of calibration
            if (isOnFirstCalibrationCycle()) {
                accumulator[axis] = 0;
            }
            
            // Sum up CALIBRATING_ACC_CYCLES readings
            accumulator[axis] += static_cast<int32_t>(rawAccData.v[axis]);
        }
        
        // Calculate final values on the last cycle
        if (isOnFinalCalibrationCycle()) {
            // Calculate average with rounding, shift Z down by acc_1G
            raw.v[X] = static_cast<int16_t>((accumulator[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES);
            raw.v[Y] = static_cast<int16_t>((accumulator[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES);
            raw.v[Z] = static_cast<int16_t>((accumulator[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc1G);
            
            // Calibration is now complete
            calibratingA = 0;
            return;
        }
        
        // Decrement calibration counter
        calibratingA--;
    }
    
    // Get current calibration cycle count (for debugging/monitoring)
    uint16_t getCalibrationCycle() {
        return calibratingA;
    }
    vector3_t* getRaw() {
        return &raw;
    }
};
}