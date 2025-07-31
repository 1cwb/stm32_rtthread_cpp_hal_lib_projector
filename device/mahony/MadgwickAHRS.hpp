/**
 * MadgwickAHRS.h
 * 
 * Implementation of Madgwick's IMU and AHRS algorithms.
 * See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 *
 * Date: 28 July 2025
 * Version: 1.0.0
 */

#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <cmath>
#include <cstdint>

class MadgwickAHRS {
public:
    /**
     * @brief Constructor for MadgwickAHRS class
     * @param sampleFrequency Sample frequency in Hz (default: 100Hz)
     * @param beta Algorithm gain beta (default: 0.1)
     */
    explicit MadgwickAHRS(float sampleFrequency = 100.0f, float beta = 0.1f)
        :   pitch(0.0f), roll(0.0f), yaw(0.0f),
			q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), 
          	sampleFreq(sampleFrequency), beta(beta) {}

    /**
     * @brief Update the filter with gyroscope, accelerometer and magnetometer data
     * @param gx Gyroscope x-axis measurement in radians/s
     * @param gy Gyroscope y-axis measurement in radians/s
     * @param gz Gyroscope z-axis measurement in radians/s
     * @param ax Accelerometer x-axis measurement in any units
     * @param ay Accelerometer y-axis measurement in any units
     * @param az Accelerometer z-axis measurement in any units
     * @param mx Magnetometer x-axis measurement in any units
     * @param my Magnetometer y-axis measurement in any units
     * @param mz Magnetometer z-axis measurement in any units
     */
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    /**
     * @brief Update the filter with gyroscope and accelerometer data (no magnetometer)
     * @param gx Gyroscope x-axis measurement in radians/s
     * @param gy Gyroscope y-axis measurement in radians/s
     * @param gz Gyroscope z-axis measurement in radians/s
     * @param ax Accelerometer x-axis measurement in any units
     * @param ay Accelerometer y-axis measurement in any units
     * @param az Accelerometer z-axis measurement in any units
     */
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    /**
     * @brief Get the quaternion components
     * @param q0 Quaternion scalar component
     * @param q1 Quaternion x component
     * @param q2 Quaternion y component
     * @param q3 Quaternion z component
     */
    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const {
        q0 = this->q0;
        q1 = this->q1;
        q2 = this->q2;
        q3 = this->q3;
    }

    /**
     * @brief Set the sample frequency
     * @param frequency Sample frequency in Hz
     */
    void setSampleFrequency(float frequency) {
        sampleFreq = frequency;
    }

    /**
     * @brief Set the algorithm gain beta
     * @param beta Algorithm gain beta
     */
    void setBeta(float beta) {
        this->beta = beta;
    }

    /**
     * @brief Reset the filter to initial state
     */
    void reset() {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }
	float getPitch(void) {
		return this->pitch;
	}
	float getRoll(void) {
		return this->roll;
	}
	float getYaw(void) {
		return this->yaw;
	}

private:
    /**
     * @brief Fast inverse square root approximation
     * @param x Input value
     * @return 1/sqrt(x)
     */
    float invSqrt(float x) const;

    /**
     * @brief Get the Euler angles in radians
     * @param roll Rotation around x-axis in radians
     * @param pitch Rotation around y-axis in radians
     * @param yaw Rotation around z-axis in radians
     */
    void getEulerAngles();
    void commonUpdate(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                bool useMag,               // true 表示同时用磁力计计算
                                float mx, float my, float mz);

private:
	constexpr static float M_PI = 3.14159265358979323846;	/* pi */

private:
    float pitch;
	float roll;
	float yaw;
    float q0, q1, q2, q3; // Quaternion components
    float sampleFreq;      // Sample frequency in Hz
    float beta;            // Algorithm gain beta
};

#endif // MADGWICK_AHRS_H