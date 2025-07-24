/*
 *  Betaflight Mahony AHRS – minimal C++17 single-file version
 *  2024-07-23  extracted & adapted from Betaflight 4.4
 *
 *  Usage:
 *      BetaflightImu imu;
 *      imu.update(dt,
 *                 gyro[X], gyro[Y], gyro[Z],
 *                 useAcc,  acc[X],  acc[Y],  acc[Z],
 *                 useMag,  mag[X],  mag[Y],  mag[Z],
 *                 useYaw,  yawErrorRad);
 *      auto [roll, pitch, yaw] = imu.getEulerDeg();   // 0.1° unit like BF
 */
#pragma once
#include <cmath>
#include <array>

namespace bfim {

/* ----------  maths.hpp (简化版)  ---------- */
constexpr float PI     = 3.14159265358979323846f;
constexpr float DEG2RAD = PI / 180.0f;
constexpr float RAD2DEG = 180.0f / PI;
constexpr float MS2_TO_G = 1.0f / 9.8f;     // 1/9.8

inline float sq(float x)   { return x * x; }
inline float invSqrt(float x) {
    union { float f; int32_t i; } v;
    v.f = x;
    v.i = 0x5F3759DF - (v.i >> 1);
    v.f = v.f * (1.5f - (x * 0.5f) * v.f * v.f);
    return v.f;
}

inline float sin_approx(float x) { return std::sin(x); }
inline float cos_approx(float x) { return std::cos(x); }
inline float acos_approx(float x) { return std::acos(x); }
inline float atan2_approx(float y, float x) { return std::atan2(y, x); }

/* ----------  quaternion 定义 ---------- */
struct Quaternion {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};

/* ----------  核心 IMU 类 ---------- */
class BetaflightImu {
public:
    /* 配置参数，你可以自行调节 */
    struct Config {
        float dcm_kp;
        float dcm_ki;
        Config() : dcm_kp(20.5f), dcm_ki(0.5f) {}
    };

    /**
     * @brief Construct a new BetaflightImu object with given configuration
     * @param c Configuration parameters for the IMU (defaults to empty struct)
     */
    explicit BetaflightImu(const Config &c = Config()) : cfg(c) {}

    /* 主入口 ----------------------------------------------------------------*/
    void update(float dt,
                float gx, float gy, float gz,
                bool  useAcc, float ax, float ay, float az,
                bool  useMag = false, float mx = 0, float my = 0, float mz = 0,
                bool  useYaw = false, float yawErrorRad = 0.0f)
    {
        gx = gx * DEG2RAD;
        gy = gy * DEG2RAD;
        gz = gz * DEG2RAD;

        // 2. 加速度：m/s² → g → 归一化
        ax = ax * MS2_TO_G;
        ay = ay * MS2_TO_G;
        az = az * MS2_TO_G;

        float recip = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
        ax *= recip;
        ay *= recip;
        az *= recip;

        mahonyAHRSupdate(dt,
                         gx, gy, gz,
                         useAcc, ax, ay, az,
                         useMag, mx, my, mz,
                         useYaw, yawErrorRad);
        updateEulerAngles();
    }

    /* 读取欧拉角 (0.1 度单位，与 Betaflight 一致) */
    std::array<int16_t, 3> getEulerDeg() const {
        return { attitude.values.roll, attitude.values.pitch, attitude.values.yaw };
    }

    /* 读取四元数 */
    const Quaternion &getQuaternion() const { return q; }

private:
    /* ---------- 内部数据结构 ---------- */
    Config cfg;
    Quaternion q{};
    struct {
        struct {
            int16_t roll  = 0;
            int16_t pitch = 0;
            int16_t yaw   = 0;
        } values;
    } attitude{};
    float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

    /* 3x3 方向余弦矩阵（行优先） */
    std::array<std::array<float, 3>, 3> rMat{};

    /* ---------- Mahony 算法主体 ---------- */
    void mahonyAHRSupdate(float dt,
                          float gx, float gy, float gz,
                          bool useAcc, float ax, float ay, float az,
                          bool useMag, float mx, float my, float mz,
                          bool useYaw, float yawError)
    {
        const float spin_rate = std::sqrt(sq(gx) + sq(gy) + sq(gz));

        /* 1. 误差向量 (ex,ey,ez) 初始化 */
        float ex = 0.0f, ey = 0.0f, ez = 0.0f;

        /* 1a. GPS/外部航向修正 */
        if (useYaw) {
            float err = yawError;
            while (err >  PI) err -= 2.0f * PI;
            while (err < -PI) err += 2.0f * PI;
            ez += sin_approx(err * 0.5f);
        }

        /* 1b. 磁力计修正 */
        if (useMag) {
            float recipNorm = invSqrt(sq(mx) + sq(my) + sq(mz));
            mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

            const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
            const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
            const float bx = std::sqrt(hx * hx + hy * hy);

            const float ez_ef = -(hy * bx);
            ex += rMat[2][0] * ez_ef;
            ey += rMat[2][1] * ez_ef;
            ez += rMat[2][2] * ez_ef;
        }

        /* 1c. 加速度计修正 */
        if (useAcc) {
            float recipNorm = invSqrt(sq(ax) + sq(ay) + sq(az));
            ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

            ex += (ay * rMat[2][2] - az * rMat[2][1]);
            ey += (az * rMat[2][0] - ax * rMat[2][2]);
            ez += (ax * rMat[2][1] - ay * rMat[2][0]);
        }

        /* 2. PI 补偿器 */
        if (cfg.dcm_ki > 0.0f) {
            const float limit = 180.0f * DEG2RAD;  // SPIN_RATE_LIMIT 约 180°/s
            if (spin_rate < limit) {
                integralFBx += cfg.dcm_ki * ex * dt;
                integralFBy += cfg.dcm_ki * ey * dt;
                integralFBz += cfg.dcm_ki * ez * dt;
            }
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }
        const float dcmKpGain = cfg.dcm_kp * getPGainScaleFactor();
        gx += dcmKpGain * ex + integralFBx;
        gy += dcmKpGain * ey + integralFBy;
        gz += dcmKpGain * ez + integralFBz;

        /* 3. 一阶龙格-库塔积分四元数 */
        gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
        Quaternion buffer = q;
        q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
        q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
        q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
        q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

        /* 4. 归一化 & 计算旋转矩阵 */
        float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
        q.w *= recipNorm; q.x *= recipNorm; q.y *= recipNorm; q.z *= recipNorm;
        computeRotationMatrix();
    }

    /* ---------- 工具 ---------- */
    void computeRotationMatrix() {
        const float &qw = q.w, &qx = q.x, &qy = q.y, &qz = q.z;
        const float qww = qw * qw, qxx = qx * qx, qyy = qy * qy, qzz = qz * qz;
        const float qwx = qw * qx, qwy = qw * qy, qwz = qw * qz;
        const float qxy = qx * qy, qxz = qx * qz, qyz = qy * qz;

        rMat[0][0] = qww + qxx - qyy - qzz;
        rMat[0][1] = 2.0f * (qxy - qwz);
        rMat[0][2] = 2.0f * (qxz + qwy);

        rMat[1][0] = 2.0f * (qxy + qwz);
        rMat[1][1] = qww - qxx + qyy - qzz;
        rMat[1][2] = 2.0f * (qyz - qwx);

        rMat[2][0] = 2.0f * (qxz - qwy);
        rMat[2][1] = 2.0f * (qyz + qwx);
        rMat[2][2] = qww - qxx - qyy + qzz;
    }

    void updateEulerAngles() {
        attitude.values.roll  = static_cast<int16_t>(
            atan2_approx(rMat[2][1], rMat[2][2]) * RAD2DEG * 10.0f);
        attitude.values.pitch = static_cast<int16_t>(
            (0.5f * PI - acos_approx(-rMat[2][0])) * RAD2DEG * 10.0f);
        attitude.values.yaw   = static_cast<int16_t>(
            (-atan2_approx(rMat[1][0], rMat[0][0]) * RAD2DEG * 10.0f));
        if (attitude.values.yaw < 0)
            attitude.values.yaw += 3600;
    }

    /* 起飞前 20 s 内使用更大 Kp，收敛更快 */
    static float getPGainScaleFactor() {
        // 这里用 millis() < 20000 的逻辑示意，实际可自行实现
        static uint32_t boot_ms = 0;
        if (boot_ms == 0) boot_ms = 0; // 仅占位
        return 1.0f;   // 如需 10 倍快速收敛，返回 10.0f
    }
};

} // namespace bfim