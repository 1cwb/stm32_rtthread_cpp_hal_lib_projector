#pragma once
#include <stdint.h>
#include "vector.hpp"
#include "axis.hpp"

namespace bfimu {

union quaternion_t{
    float v[4];
    struct {
        float w=1, x=0, y=0, z=0;
    };
};

struct quaternionProducts{
    float ww=1, wx=0, wy=0, wz=0, xx=0, xy=0, xz=0, yy=0, yz=0, zz=0;
};

union attitudeEulerAngles_t{
    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        // absolute angle inclination in multiple of 0.1 degree  eg attitude.values.yaw 180 deg = 1800
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } values;

};

extern attitudeEulerAngles_t attitude;
extern matrix33_t rMat;
extern quaternion_t imuAttitudeQuaternion; //attitude quaternion to use in blackbox

struct imuConfig_t {
    uint16_t imu_dcm_kp;          // DCM filter proportional gain ( x 10000)
    uint16_t imu_dcm_ki;          // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t imu_process_denom;
    uint16_t mag_declination;     // Magnetic declination in degrees * 10
};

struct imuRuntimeConfig_t {
    float imuDcmKi;
    float imuDcmKp;
};

// FIXME some of these are flight modes, some of these are general status indicators
typedef enum {
    ARMED                       = (1 << 0),
    WAS_EVER_ARMED              = (1 << 1),
    WAS_ARMED_WITH_PREARM       = (1 << 2)
} armingFlag_e;

extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))
void imuInit(void);
void imuCalculateEstimatedAttitude(uint64_t currentTimeUs,
                                float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz);
}