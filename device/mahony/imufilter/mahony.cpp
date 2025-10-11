#include "mahony.hpp"
#include <cmath>
#include "maths.hpp"
#include "vector.hpp"

namespace bfimu {

Mahony::Mahony() :
    throttleAngleScale(0.0f),
    throttleAngleValue(0),
    smallAngleCosZ(0.0f),
    armingFlags(0)
{
    imuConfig = {
        .imu_dcm_kp = 2500,      // 1.0 * 10000
        .imu_dcm_ki = 0,         // 0.003 * 10000
        .small_angle = 25,       // DEFAULT_SMALL_ANGLE
        .imu_process_denom = 2,
        .mag_declination = 0
    };
}

void Mahony::imuQuaternionComputeProducts(const quaternion_t& quat, quaternionProducts& quatProd)
{
    quatProd.ww = quat.w * quat.w;
    quatProd.wx = quat.w * quat.x;
    quatProd.wy = quat.w * quat.y;
    quatProd.wz = quat.w * quat.z;
    quatProd.xx = quat.x * quat.x;
    quatProd.xy = quat.x * quat.y;
    quatProd.xz = quat.x * quat.z;
    quatProd.yy = quat.y * quat.y;
    quatProd.yz = quat.y * quat.z;
    quatProd.zz = quat.z * quat.z;
}

void Mahony::imuComputeRotationMatrix(void)
{
    imuQuaternionComputeProducts(q, qP);

    rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
}

float Mahony::calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

void Mahony::imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value)
{
    // current default for imu_dcm_kp is 2500; our 'normal' or baseline value for imuDcmKp is 0.25
    imuRuntimeConfig.imuDcmKp = imuConfig.imu_dcm_kp / 10000.0f;
    imuRuntimeConfig.imuDcmKi = imuConfig.imu_dcm_ki / 10000.0f;
    
    // magnetic declination has negative sign (positive clockwise when seen from top)
    const float imuMagneticDeclinationRad = DEGREES_TO_RADIANS(imuConfig.mag_declination / 10.0f);
    north_ef = Vector2(cos_approx(imuMagneticDeclinationRad), -sin_approx(imuMagneticDeclinationRad));

    smallAngleCosZ = cos_approx(degreesToRadians(imuConfig.small_angle));

    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
    throttleAngleValue = throttle_correction_value;
}

void Mahony::imuInit(void)
{
    imuConfigure(0, 0);
    imuComputeRotationMatrix();
}

float Mahony::imuCalcMagErr(float x, float y, float z)
{
    // Use measured magnetic field vector
    Vector3 mag_bf(x, y, z);
    float magNormSquared = mag_bf.normSq();

    if (magNormSquared > 0.01f) {
        // project magnetometer reading into Earth frame
        Vector3 mag_ef = rMat.transform(mag_bf); // BF->EF true north
        
        // Normalise magnetometer measurement
        mag_ef = mag_ef * (1.0f / std::sqrt(magNormSquared));

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles
        Vector2 mag2d_ef(mag_ef.x(), mag_ef.y());
        
        // mag2d_ef - measured mag field vector in EF (2D ground plane projection)
        // north_ef - reference mag field vector heading due North in EF (2D ground plane projection).
        //              Adjusted for magnetic declination (in imuConfigure)

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        // increase gain on large misalignment
        const float dot = mag2d_ef.dot(north_ef);
        const float cross = mag2d_ef.cross(north_ef);
        return (dot > 0) ? cross : (cross < 0 ? -1.0f : 1.0f) * mag2d_ef.norm();
    } else {
        // invalid magnetometer data
        return 0.0f;
    }
}

float Mahony::invSqrt(float x)
{
    return 1.0f / std::sqrt(x);
}

void Mahony::imuMahonyAHRSupdate(float dt,
                                float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                float headingErrMag, float headingErrCog,
                                const float dcmKpGain)
{
    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

    // Calculate general spin rate (rad/s)
    const float spin_rate = std::sqrt(sq(gx) + sq(gy) + sq(gz));

    float ex = 0, ey = 0, ez = 0;

    // Add error from magnetometer and Cog
    // just rotate input value to body frame
    ex += rMat[2][0] * (headingErrCog + headingErrMag);
    ey += rMat[2][1] * (headingErrCog + headingErrMag);
    ez += rMat[2][2] * (headingErrCog + headingErrMag);

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement; useAcc is true when all smoothed acc axes are within 20% of 1G
        recipAccNorm = invSqrt(recipAccNorm);

        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (imuRuntimeConfig.imuDcmKi > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.imuDcmKi;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion_t buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}

void Mahony::imuUpdateEulerAngles(void)
{
    attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
    attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
    attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf)));

    imuAttitudeQuaternion = q; // using current q quaternion for blackbox log

    if (attitude.values.yaw < 0) {
        attitude.values.yaw += 3600;
    }
}

float Mahony::imuCalcKpGain(uint64_t currentTimeUs, bool useAcc, const std::array<float, 3>& gyroAverage)
{
    static enum {
        stArmed,
        stRestart,
        stQuiet,
        stReset,
        stDisarmed
    } arState = stDisarmed;

    static uint64_t stateTimeout;

    const bool armState = isArmingFlagSet(ARMED);

    if (!armState) {
        // If gyro activity exceeds the threshold then restart the quiet period.
        // Also, if the attitude reset has been complete and there is subsequent gyro activity then
        //  start the reset cycle again. This addresses the case where the pilot rights the craft after a crash.
        if ((std::fabs(gyroAverage[X]) > ATTITUDE_RESET_GYRO_LIMIT)  // gyro axis limit exceeded
            || (std::fabs(gyroAverage[Y]) > ATTITUDE_RESET_GYRO_LIMIT)
            || (std::fabs(gyroAverage[Z]) > ATTITUDE_RESET_GYRO_LIMIT)
            || !useAcc                                              // acc reading out of range
            ) {
            arState = stRestart;
        }

        switch (arState) {
        default: // should not happen, safeguard only
        case stArmed:
        case stRestart:
            stateTimeout = currentTimeUs + ATTITUDE_RESET_QUIET_TIME;
            arState = stQuiet;
            // fallthrough
        case stQuiet:
            if (currentTimeUs - stateTimeout >= 0) {
                stateTimeout = currentTimeUs + ATTITUDE_RESET_ACTIVE_TIME;
                arState = stReset;
            }
            // low gain (default value of 0.25) during quiet phase
            return imuRuntimeConfig.imuDcmKp;
        case stReset:
            if (currentTimeUs - stateTimeout >= 0) {
                arState = stDisarmed;
            }
            // high gain, 100x greater than normal, or 25, after quiet period
            return imuRuntimeConfig.imuDcmKp * 100.0f;
        case stDisarmed:
            // Scale the kP to converge 10x faster when disarmed, ie 2.5
            return imuRuntimeConfig.imuDcmKp * 10.0f;
        }
    } else {
        arState = stArmed;
        return imuRuntimeConfig.imuDcmKp;
    }
}

void Mahony::update(uint64_t currentTimeUs,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz)
{
    static uint64_t previousIMUUpdateTime = 0;
    const uint64_t deltaT = currentTimeUs - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTimeUs;
    const float dt = deltaT * 1e-6f;

    // *** magnetometer based error estimate ***
    float magErr = imuCalcMagErr(mx, my, mz);

    // *** GoC based error estimate ***
    float cogErr = 0;

    std::array<float, 3> gyroAverage = {gx, gy, gz};
    
    const bool useAcc = true; // imuIsAccelerometerHealthy(); // all smoothed accADC values are within 10% of 1G
    
    imuMahonyAHRSupdate(dt,
                       gx, gy, gz,
                       useAcc, ax, ay, az,
                       magErr, cogErr,
                       imuCalcKpGain(currentTimeUs, useAcc, gyroAverage));

    imuUpdateEulerAngles();
}

} // namespace bfimu