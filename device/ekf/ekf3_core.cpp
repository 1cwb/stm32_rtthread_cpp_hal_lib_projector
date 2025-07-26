#include "ekf3_config.hpp"
#include "ekf3_math.cpp"

// 精简 24-state 状态： [q(4) | vel(3) | pos(3) | gyro_bias(3) | acc_bias(3) | mag_field(3) | wind(3) | terrain(2)]
static float x[EKF3_STATE_DIM] = {1,0,0,0};
static float P[EKF3_STATE_DIM][EKF3_STATE_DIM] = {};
static float q[4] = {1,0,0,0};

static void predict(const float dt, const float g[3], const float a[3]) {
    // 仅做四元数积分 + 简单陀螺零偏补偿
    float gb[3] = {g[0]-x[10], g[1]-x[11], g[2]-x[12]};
    float dq[4] = {0, 0.5f*gb[0]*dt, 0.5f*gb[1]*dt, 0.5f*gb[2]*dt};
    float q_new[4];
    // 一阶积分
    q_new[0] = q[0] + (-q[1]*dq[1] - q[2]*dq[2] - q[3]*dq[3]);
    q_new[1] = q[1] + ( q[0]*dq[1] + q[2]*dq[3] - q[3]*dq[2]);
    q_new[2] = q[2] + ( q[0]*dq[2] - q[1]*dq[3] + q[3]*dq[1]);
    q_new[3] = q[3] + ( q[0]*dq[3] + q[1]*dq[2] - q[2]*dq[1]);
    for (int i=0;i<4;i++) q[i]=q_new[i];
    quatNormalize(q);
}

static void fuseMag(const float m[3]) {
    // 极简磁罗盘融合：直接把磁向量旋转到机体，然后用标量增益修正
    float h[3];
    quatRotate(q, m, h);
    float err = atan2f(h[1], h[0]);
    float K = 0.02f;
    float dq[4] = {1, 0, 0, -K*err};
    float q_new[4];
    // 一阶近似
    q_new[0] = q[0] + (-q[3]*(-K*err))*0.5f;
    q_new[1] = q[1] + ( q[3]*(-K*err))*0.5f;
    q_new[2] = q[2] + ( q[0]*(-K*err))*0.5f;
    q_new[3] = q[3] + ( q[1]*(-K*err))*0.5f;
    for (int i=0;i<4;i++) q[i]=q_new[i];
    quatNormalize(q);
}

static inline void quat2euler(const float q[4], float& roll, float& pitch, float& yaw) {
    roll  = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])) * 57.2958f;
    pitch = asinf (2*(q[0]*q[2]-q[3]*q[1])) * 57.2958f;
    yaw   = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])) * 57.2958f;
}
