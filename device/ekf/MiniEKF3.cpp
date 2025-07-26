#include "MiniEKF3.hpp"
#include <cstring>
#include <algorithm>

inline static float quat_norm(const float q[4]) {
    return std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}
inline static void quat_normalize(float q[4]) {
    float n = quat_norm(q);
    if (n>0) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
}
inline static void quat_to_euler(const float q[4], float& roll, float& pitch, float& yaw) {
    roll  = atan2f( 2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]) );
    pitch = asinf ( 2*(q[0]*q[2] - q[3]*q[1]) );
    yaw   = atan2f( 2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]) );
}

MiniEKF3::MiniEKF3() {
    memset(&state_, 0, sizeof(state_));
    state_.q[0] = 1.0f;           // 单位四元数
    for (int i=0;i<4;i++) P_q[i] = 1.0f;
    for (int i=0;i<3;i++) P_bias[i] = 0.1f;
    roll_ = pitch_ = yaw_ = 0;
}

void MiniEKF3::predict(float dt, const Vec3& gyro) {
    Vec3 w = gyro - state_.gyro_bias;
    float dq[4] = {
        1.0f,                      // 近似一阶积分
        0.5f * w.x * dt,
        0.5f * w.y * dt,
        0.5f * w.z * dt
    };
    // 四元数乘法
    float q_new[4] = {
        state_.q[0]*dq[0] - state_.q[1]*dq[1] - state_.q[2]*dq[2] - state_.q[3]*dq[3],
        state_.q[0]*dq[1] + state_.q[1]*dq[0] + state_.q[2]*dq[3] - state_.q[3]*dq[2],
        state_.q[0]*dq[2] - state_.q[1]*dq[3] + state_.q[2]*dq[0] + state_.q[3]*dq[1],
        state_.q[0]*dq[3] + state_.q[1]*dq[2] - state_.q[2]*dq[1] + state_.q[3]*dq[0]
    };
    memcpy(state_.q, q_new, sizeof(state_.q));
    quat_normalize(state_.q);

    // 简单协方差预测
    for (int i=0;i<4;i++)  P_q[i]  += gyro_noise * dt;
    for (int i=0;i<3;i++)  P_bias[i] += 1e-6f * dt;
}

void MiniEKF3::updateAcc(const Vec3& acc) {
    Vec3 g(0,0,1); // 重力方向 NED
    float q[4] = {state_.q[0], state_.q[1], state_.q[2], state_.q[3]};
    // 把重力转到机体系
    Vec3 h;
    h.x = 2*(q[1]*q[3] - q[0]*q[2]);
    h.y = 2*(q[0]*q[1] + q[2]*q[3]);
    h.z = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    Vec3 error = acc.normalized() - h;
    if (error.norm() < 1.0f) return; // 忽略过大异常
    // 简化卡尔曼增益
    float K = 0.02f;
    // 姿态校正方向与误差方向一致
    state_.q[1] += K * error.x;
    state_.q[2] += K * error.y;
    state_.q[3] += K * error.z;
    quat_normalize(state_.q);
}

void MiniEKF3::updateMag(const Vec3& mag) {
    Vec3 mn(1,0,0); // 期望北向
    float q[4] = {state_.q[0], state_.q[1], state_.q[2], state_.q[3]};
    Vec3 h;
    h.x = 2*(q[0]*q[1] + q[2]*q[3]);
    h.y = 2*(q[1]*q[2] - q[0]*q[3]);
    h.z = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    Vec3 error = mag.normalized() - h;
    float K = 0.02f;
    state_.q[1] += K * error.x;
    state_.q[2] += K * error.y;
    state_.q[3] += K * error.z;
    quat_normalize(state_.q);
}

void MiniEKF3::update(float dt,
                      const Vec3& gyro,
                      const Vec3& acc,
                      const Vec3& mag,
                      float /*baro*/)
{
    predict(dt, gyro);
    updateAcc(acc);
    //updateMag(mag);
    quat_to_euler(state_.q, roll_, pitch_, yaw_);
}