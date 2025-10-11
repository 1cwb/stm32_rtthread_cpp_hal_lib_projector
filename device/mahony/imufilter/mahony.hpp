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

struct imuConfig_t {
    uint16_t imu_dcm_kp;          // DCM filter proportional gain ( x 10000)
    uint16_t imu_dcm_ki;          // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
    uint8_t imu_process_denom;
    uint16_t mag_declination;     // Magnetic declination in degrees * 10
};

struct imuRuntimeConfig_t {
    float imuDcmKi = 0.0f;
    float imuDcmKp = 0.0f;
};

// FIXME some of these are flight modes, some of these are general status indicators
typedef enum {
    ARMED                       = (1 << 0),
    WAS_EVER_ARMED              = (1 << 1),
    WAS_ARMED_WITH_PREARM       = (1 << 2)
} armingFlag_e;

class Mahony{
public:
    Mahony():
        q{1, 0, 0, 0},
        qP{1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        throttleAngleScale(0.0f),
        throttleAngleValue(0),
        smallAngleCosZ(0.0f),
        armingFlags(0)
    {
        imuConfig = {
        .imu_dcm_kp = 2500,      // 1.0 * 10000
        .imu_dcm_ki = 0,         // 0.003 * 10000
        .small_angle = 25,//DEFAULT_SMALL_ANGLE,
        .imu_process_denom = 2,
        .mag_declination = 0
    };
    }
    ~Mahony(){}
    void imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value);
    void imuInit(void);
    void update(uint64_t currentTimeUs,
                                float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float mx, float my, float mz);
    void disableArmingFlag(armingFlag_e mask) { armingFlags &= ~(mask); }
    void enableArmingFlag(armingFlag_e mask) { armingFlags |= (mask); }
    bool isArmingFlagSet(armingFlag_e mask) { return (armingFlags & (mask)); }
    float getPitch(void) const { return attitude.values.pitch; }
    float getRoll(void) const { return attitude.values.roll; }
    float getYaw(void) const { return attitude.values.yaw; }
private:
    inline void imuQuaternionComputeProducts(quaternion_t *quat, quaternionProducts *quatProd);
    inline void imuComputeRotationMatrix(void);
    inline float calculateThrottleAngleScale(uint16_t throttle_correction_angle);
    inline float imuCalcMagErr(float x, float y, float z);
    inline static float invSqrt(float x);
    void imuMahonyAHRSupdate(float dt,
                                float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                float headingErrMag, float headingErrCog,
                                const float dcmKpGain);
    void imuUpdateEulerAngles(void);
    float imuCalcKpGain(uint64_t currentTimeUs, bool useAcc, float *gyroAverage);
private:

    static constexpr int SPIN_RATE_LIMIT = 20;
    static constexpr uint64_t ATTITUDE_RESET_QUIET_TIME = 250000;   // 250ms - gyro quiet period after disarm before attitude reset
    static constexpr float ATTITUDE_RESET_GYRO_LIMIT = 15.0f;       // 15 deg/sec - gyro limit for quiet period
    static constexpr uint64_t ATTITUDE_RESET_ACTIVE_TIME = 500000;  // 500ms - Time to wait for attitude to converge at high gain
    static constexpr uint8_t DEFAULT_SMALL_ANGLE = 25;

    matrix33_t rMat;
    vector2_t north_ef;
    imuConfig_t imuConfig;
    imuRuntimeConfig_t imuRuntimeConfig;

    // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
    attitudeEulerAngles_t attitude;
    quaternion_t imuAttitudeQuaternion;

    // quaternion of sensor frame relative to earth frame
    quaternion_t q;
    quaternionProducts qP;

    float throttleAngleScale;
    int throttleAngleValue;
    float smallAngleCosZ;
    uint8_t armingFlags;
};
}