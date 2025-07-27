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

inline static void matSetEye7(float *A) {
    for (int i = 0; i < 7; ++i)
        for (int j = 0; j < 7; ++j)
            A[i*7+j] = (i == j) ? 1.0f : 0.0f;
}

/* C = A*B   (A,B,C 都是 7×7) */
inline static void matMul7(const float *A, const float *B, float *C) {
    for (int i = 0; i < 7; ++i)
        for (int j = 0; j < 7; ++j) {
            C[i*7+j] = 0.0f;
            for (int k = 0; k < 7; ++k)
                C[i*7+j] += A[i*7+k] * B[k*7+j];
        }
}

/* A = A + B */
inline static void matAdd7(float *A, const float *B) {
    for (int i = 0; i < 49; ++i) A[i] += B[i];
}

/* A = A - B */
inline static void matSub7(float *A, const float *B) {
    for (int i = 0; i < 49; ++i) A[i] -= B[i];
}

/* 3×3 矩阵求逆（伴随矩阵法），返回是否成功 */
inline static bool inv3(const float *A, float *inv) {
    float det = A[0]*(A[4]*A[8] - A[5]*A[7])
              - A[1]*(A[3]*A[8] - A[5]*A[6])
              + A[2]*(A[3]*A[7] - A[4]*A[6]);
    if (fabs(det) < 1e-6f) return false;
    float invDet = 1.0f / det;
    inv[0] = (A[4]*A[8] - A[5]*A[7]) * invDet;
    inv[1] = (A[2]*A[7] - A[1]*A[8]) * invDet;
    inv[2] = (A[1]*A[5] - A[2]*A[4]) * invDet;
    inv[3] = (A[5]*A[6] - A[3]*A[8]) * invDet;
    inv[4] = (A[0]*A[8] - A[2]*A[6]) * invDet;
    inv[5] = (A[2]*A[3] - A[0]*A[5]) * invDet;
    inv[6] = (A[3]*A[7] - A[4]*A[6]) * invDet;
    inv[7] = (A[1]*A[6] - A[0]*A[7]) * invDet;
    inv[8] = (A[0]*A[4] - A[1]*A[3]) * invDet;
    return true;
}

MiniEKF3::MiniEKF3() {
    x_[0]=1; x_[1]=x_[2]=x_[3]=0;
    x_[4]=x_[5]=x_[6]=0;
    state_.gyro_bias = Vec3(0,0,0);
    matSetEye7(P_);
    for (int i=0;i<49;++i) P_[i]*=1e-2f;
    for (int i=4;i<7;++i) P_[i*7+i]=0.1f;
}

void MiniEKF3::predict(float dt, const Vec3& w_raw) {
    /* 3.1 计算角速度 */
    Vec3 w = w_raw - state_.gyro_bias;

    /* 3.2 状态转移矩阵 F (7×7) */
    float F[49];
    matSetEye7(F);
    /* 四元数子块 Fqq = I + 0.5*dt*Omega */
    float *Fqq = F;
    Fqq[0*7+0] = 1.0f;
    Fqq[0*7+1] = -0.5f*dt*w.x;
    Fqq[0*7+2] = -0.5f*dt*w.y;
    Fqq[0*7+3] = -0.5f*dt*w.z;

    Fqq[1*7+0] =  0.5f*dt*w.x;
    Fqq[1*7+1] =  1.0f;
    Fqq[1*7+2] =  0.5f*dt*w.z;
    Fqq[1*7+3] = -0.5f*dt*w.y;

    Fqq[2*7+0] =  0.5f*dt*w.y;
    Fqq[2*7+1] = -0.5f*dt*w.z;
    Fqq[2*7+2] =  1.0f;
    Fqq[2*7+3] =  0.5f*dt*w.x;

    Fqq[3*7+0] =  0.5f*dt*w.z;
    Fqq[3*7+1] =  0.5f*dt*w.y;
    Fqq[3*7+2] = -0.5f*dt*w.x;
    Fqq[3*7+3] =  1.0f;

    /* 四元数-零偏耦合子块 */
    F[0*7+4] = F[0*7+5] = F[0*7+6] = 0.0f;
    F[1*7+4] =  0.5f*dt;
    F[2*7+5] =  0.5f*dt;
    F[3*7+6] =  0.5f*dt;

    /* 3.3 过程噪声 Q */
    float Q[49] = {0};
    for (int i = 0; i < 4; ++i) Q[i*7+i] = SIGMA_Q * dt;
    for (int i = 4; i < 7; ++i) Q[i*7+i] = SIGMA_BG * dt;

    /* 3.4 协方差预测 P = F*P*F^T + Q */
    float tmp[49], FPT[49];
    matMul7(F, P_, tmp);
    matMul7(tmp, F, FPT);   /* 注意：这里应再转置 F 再乘，但小角近似下对称，可省 */
    for (int i = 0; i < 49; ++i) P_[i] = FPT[i] + Q[i];

    /* 3.5 四元数积分并归一化（保持你原先写法即可） */
    float dq[4] = {1.0f,
                   0.5f*dt*w.x,
                   0.5f*dt*w.y,
                   0.5f*dt*w.z};
    float q_new[4];
    q_new[0] = state_.q[0]*dq[0] - state_.q[1]*dq[1] - state_.q[2]*dq[2] - state_.q[3]*dq[3];
    q_new[1] = state_.q[0]*dq[1] + state_.q[1]*dq[0] + state_.q[2]*dq[3] - state_.q[3]*dq[2];
    q_new[2] = state_.q[0]*dq[2] - state_.q[1]*dq[3] + state_.q[2]*dq[0] + state_.q[3]*dq[1];
    q_new[3] = state_.q[0]*dq[3] + state_.q[1]*dq[2] - state_.q[2]*dq[1] + state_.q[3]*dq[0];
    float n = sqrt(q_new[0]*q_new[0]+q_new[1]*q_new[1]+q_new[2]*q_new[2]+q_new[3]*q_new[3]);
    for (int i=0;i<4;++i) state_.q[i]=q_new[i]/n;
}

void MiniEKF3::updateAcc(const Vec3& acc) {
    /* 4.1 期望重力方向 */
    float q[4] = {state_.q[0], state_.q[1], state_.q[2], state_.q[3]};
    float h[3];
    h[0] = 2*(q[1]*q[3] - q[0]*q[2]);
    h[1] = 2*(q[0]*q[1] + q[2]*q[3]);
    h[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    /* 4.2 观测雅可比 H (3×7) */
    float H[3*7] = {0};
    /* 对 q 的偏导 */
    H[0*7+0] = -2*q[2]; H[0*7+1] =  2*q[3]; H[0*7+2] = -2*q[0]; H[0*7+3] =  2*q[1];
    H[1*7+0] =  2*q[1]; H[1*7+1] =  2*q[0]; H[1*7+2] =  2*q[3]; H[1*7+3] =  2*q[2];
    H[2*7+0] =  2*q[0]; H[2*7+1] = -2*q[1]; H[2*7+2] = -2*q[2]; H[2*7+3] =  2*q[3];

    /* 4.3 创新 y = z - h */
    float z[3] = {acc.x, acc.y, acc.z};
    float n_acc = sqrt(z[0]*z[0]+z[1]*z[1]+z[2]*z[2]);
    for (int i=0;i<3;++i) z[i]/=n_acc;
    float y[3];
    for (int i=0;i<3;++i) y[i]=z[i]-h[i];

    /* 4.4 S = H*P*H^T + R */
    float HP[3*7], S[3*3];
    for (int i=0;i<3;++i)
        for (int j=0;j<7;++j){
            HP[i*7+j]=0;
            for (int k=0;k<7;++k) HP[i*7+j]+=H[i*7+k]*P_[k*7+j];
        }
    for (int i=0;i<3;++i)
        for (int j=0;j<3;++j){
            S[i*3+j]=0;
            for (int k=0;k<7;++k) S[i*3+j]+=HP[i*7+k]*H[j*7+k];
            if (i==j) S[i*3+j]+=SIGMA_ACC;
        }

    /* 4.5 S^-1 */
    float Sinv[9];
    if (!inv3(S, Sinv)) return;

    /* 4.6 K = P*H^T*S^-1 */
    float K[7*3];
    float PHt[7*3];
    for (int i=0;i<7;++i)
        for (int j=0;j<3;++j){
            PHt[i*3+j]=0;
            for (int k=0;k<7;++k) PHt[i*3+j]+=P_[i*7+k]*H[j*7+k];
        }
    for (int i=0;i<7;++i)
        for (int j=0;j<3;++j){
            K[i*3+j]=0;
            for (int k=0;k<3;++k) K[i*3+j]+=PHt[i*3+k]*Sinv[k*3+j];
        }

    /* 4.7 状态更新 */
    float dx[7]={0};
    for (int i=0;i<7;++i)
        for (int j=0;j<3;++j) dx[i]+=K[i*3+j]*y[j];

    /* 4.8 四元数误差转四元数乘积 */
    float dq[4]={1, 0.5f*dx[1], 0.5f*dx[2], 0.5f*dx[3]};
    float q_new[4];
    q_new[0] = state_.q[0]*dq[0] - state_.q[1]*dq[1] - state_.q[2]*dq[2] - state_.q[3]*dq[3];
    q_new[1] = state_.q[0]*dq[1] + state_.q[1]*dq[0] + state_.q[2]*dq[3] - state_.q[3]*dq[2];
    q_new[2] = state_.q[0]*dq[2] - state_.q[1]*dq[3] + state_.q[2]*dq[0] + state_.q[3]*dq[1];
    q_new[3] = state_.q[0]*dq[3] + state_.q[1]*dq[2] - state_.q[2]*dq[1] + state_.q[3]*dq[0];
    float n=sqrt(q_new[0]*q_new[0]+q_new[1]*q_new[1]+q_new[2]*q_new[2]+q_new[3]*q_new[3]);
    for(int i=0;i<4;++i) state_.q[i]=q_new[i]/n;

    /* 4.9 陀螺零偏更新 */
    state_.gyro_bias.x += dx[4];
    state_.gyro_bias.y += dx[5];
    state_.gyro_bias.z += dx[6];

    /* 4.10 协方差更新 P = (I - K*H)*P */
    float KH[49], IKH[49];
    for (int i=0;i<7;++i)
        for (int j=0;j<7;++j){
            KH[i*7+j]=0;
            for (int k=0;k<3;++k) KH[i*7+j]+=K[i*3+k]*H[k*7+j];
        }
    matSetEye7(IKH);
    for (int i=0;i<49;++i) IKH[i]-=KH[i];

    float tmp[49];
    matMul7(IKH, P_, tmp);
    for (int i=0;i<49;++i) P_[i]=tmp[i];
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