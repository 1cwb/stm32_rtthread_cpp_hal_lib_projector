// EKF_AHRS.cpp - 平台无关的EKF姿态解算库实现
#include "ekfahrs.hpp"
#include <cmath>

// 定义常量
constexpr static float GRAVITY = 9.81f;
constexpr static float PI = 3.14159265358979323846f;
constexpr static float DEG_TO_RAD = (PI / 180.0f);
constexpr static float RAD_TO_DEG = (180.0f / PI);

// 构造函数
EKFAHRS::EKFAHRS() : 
    x(7, 1),     // 7x1 状态向量
    P(7, 7),     // 7x7 协方差矩阵
    Q(7, 7),     // 7x7 过程噪声
    R_accel(3, 3), // 3x3 加速度计测量噪声
    R_mag(3, 3)    // 3x3 磁力计测量噪声
{
    // 初始化状态向量 (单位四元数，零偏置)
    x(0, 0) = 1.0f; // q0
    x(1, 0) = 0.0f; // q1
    x(2, 0) = 0.0f; // q2
    x(3, 0) = 0.0f; // q3
    x(4, 0) = 0.0f; // bgx
    x(5, 0) = 0.0f; // bgy
    x(6, 0) = 0.0f; // bgz
    
    // 初始化协方差矩阵
    P = Matrix::identity(7);
    for(int i = 0; i < 4; i++) P(i, i) = 0.1f;   // 四元数初始不确定性
    for(int i = 4; i < 7; i++) P(i, i) = 0.01f;  // 偏置初始不确定性
    
    // 初始化过程噪声协方差
    Q = Matrix::identity(7);
    for(int i = 0; i < 4; i++) Q(i, i) = 1e-2f;  // 四元数过程噪声
    for(int i = 4; i < 7; i++) Q(i, i) = 1e-6f;  // 偏置过程噪声
    
    // 初始化测量噪声协方差
    R_accel = Matrix::identity(3);
    R_accel(0, 0) = 0.05f;  // 加速度计测量噪声
    R_accel(1, 1) = 0.05f;
    R_accel(2, 2) = 0.05f;
    
    R_mag = Matrix::identity(3);
    R_mag(0, 0) = 0.1f;    // 磁力计测量噪声
    R_mag(1, 1) = 0.1f;
    R_mag(2, 2) = 0.1f;
    
    // 初始化时间戳
    last_time = 0;
    
    // 初始化磁场参考向量
    mag_ref[0] = 1.0f;  // 默认指向北
    mag_ref[1] = 0.0f;
    mag_ref[2] = 0.0f;
}

// 归一化向量
void EKFAHRS::normalizeVector(float v[3]) {
    float norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm > 0.0001f) {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
}

// 四元数转旋转矩阵
void EKFAHRS::quaternionToRotationMatrix(const float q[4], float R[3][3]) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    
    R[0][0] = 1.0f - 2.0f*(q2*q2 + q3*q3);
    R[0][1] = 2.0f*(q1*q2 - q0*q3);
    R[0][2] = 2.0f*(q1*q3 + q0*q2);
    
    R[1][0] = 2.0f*(q1*q2 + q0*q3);
    R[1][1] = 1.0f - 2.0f*(q1*q1 + q3*q3);
    R[1][2] = 2.0f*(q2*q3 - q0*q1);
    
    R[2][0] = 2.0f*(q1*q3 - q0*q2);
    R[2][1] = 2.0f*(q2*q3 + q0*q1);
    R[2][2] = 1.0f - 2.0f*(q1*q1 + q2*q2);
}

// 初始化滤波器
void EKFAHRS::initialize(float ax, float ay, float az, float mx, float my, float mz) {
    // 归一化加速度计测量值
    float accel[3] = {ax, ay, az};
    normalizeVector(accel);
    
    // 计算初始俯仰和横滚
    float pitch = -asin(accel[0]);
    float roll = atan2(accel[1], accel[2]);
    
    // 归一化磁力计测量值
    float mag[3] = {mx, my, mz};
    normalizeVector(mag);
    
    // 计算初始偏航角
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    
    // 将磁场向量从机体坐标系转换到世界坐标系
    float mag_x = mag[0] * cos_pitch + mag[1] * sin_roll * sin_pitch + mag[2] * cos_roll * sin_pitch;
    float mag_y = mag[1] * cos_roll - mag[2] * sin_roll;
    float yaw = atan2(mag_y, mag_x);
    
    // 保存磁场参考向量
    mag_ref[0] = cos(yaw);
    mag_ref[1] = sin(yaw);
    mag_ref[2] = 0.0f;
    
    // 从欧拉角计算四元数
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    
    x(0, 0) = cy * cr * cp + sy * sr * sp;  // q0
    x(1, 0) = cy * sr * cp - sy * cr * sp;  // q1
    x(2, 0) = cy * cr * sp + sy * sr * cp;  // q2
    x(3, 0) = sy * cr * cp - cy * sr * sp;  // q3
    
    // 重置协方差矩阵
    P = Matrix::identity(7);
    for(int i = 0; i < 4; i++) P(i, i) = 0.1f;
    for(int i = 4; i < 7; i++) P(i, i) = 0.01f;
}

// 完整的协方差矩阵更新实现
void EKFAHRS::updateCovarianceMatrix(float dt) {
    // 获取当前状态
    float q0 = x(0, 0), q1 = x(1, 0), q2 = x(2, 0), q3 = x(3, 0);
    float bgx = x(4, 0), bgy = x(5, 0), bgz = x(6, 0);
    
    // 构建状态转移矩阵 F (7x7)
    Matrix F(7, 7);
    
    // 四元数部分的状态转移雅可比矩阵
    // ∂f/∂q - 四元数动力学方程的雅可比矩阵
    F(0, 0) = 1.0f; F(0, 1) = -0.5f * bgx * dt; F(0, 2) = -0.5f * bgy * dt; F(0, 3) = -0.5f * bgz * dt;
    F(1, 0) = 0.5f * bgx * dt; F(1, 1) = 1.0f; F(1, 2) = 0.5f * bgz * dt; F(1, 3) = -0.5f * bgy * dt;
    F(2, 0) = 0.5f * bgy * dt; F(2, 1) = -0.5f * bgz * dt; F(2, 2) = 1.0f; F(2, 3) = 0.5f * bgx * dt;
    F(3, 0) = 0.5f * bgz * dt; F(3, 1) = 0.5f * bgy * dt; F(3, 2) = -0.5f * bgx * dt; F(3, 3) = 1.0f;
    
    // ∂f/∂bg - 四元数对陀螺仪偏置的偏导
    F(0, 4) = 0.5f * q1 * dt; F(0, 5) = 0.5f * q2 * dt; F(0, 6) = 0.5f * q3 * dt;
    F(1, 4) = -0.5f * q0 * dt; F(1, 5) = 0.5f * q3 * dt; F(1, 6) = -0.5f * q2 * dt;
    F(2, 4) = -0.5f * q3 * dt; F(2, 5) = -0.5f * q0 * dt; F(2, 6) = 0.5f * q1 * dt;
    F(3, 4) = 0.5f * q2 * dt; F(3, 5) = -0.5f * q1 * dt; F(3, 6) = -0.5f * q0 * dt;
    
    // 陀螺仪偏置部分 - 假设为随机游走模型
    for (int i = 4; i < 7; i++) {
        for (int j = 4; j < 7; j++) {
            F(i, j) = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 四元数对陀螺仪偏置的交叉项（上面已经设置）
    // 陀螺仪偏置对四元数的偏导（上面已经设置）
    
    // 完整的预测步骤: P = F * P * F^T + Q * dt
    Matrix F_T = F.transpose();
    
    // 第一项: F * P * F^T
    Matrix temp1 = F * P;
    Matrix temp2 = temp1 * F_T;
    
    // 第二项: Q * dt
    Matrix Q_dt = Q;
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            Q_dt(i, j) = Q(i, j) * dt;
        }
    }
    
    // 更新协方差矩阵: P = F * P * F^T + Q * dt
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P(i, j) = temp2(i, j) + Q_dt(i, j);
        }
    }
    
    // 确保协方差矩阵保持对称正定
    for (int i = 0; i < 7; i++) {
        for (int j = i + 1; j < 7; j++) {
            // 强制对称: P(i,j) = P(j,i)
            float avg = 0.5f * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
    }
    
    // 添加小的正数到对角线以确保正定性
    for (int i = 0; i < 7; i++) {
        P(i, i) += 1e-10f;
    }
}

// 主更新函数
void EKFAHRS::update(float gx, float gy, float gz, 
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float current_time) {
    // 计算时间间隔
    float dt = current_time - last_time;
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;
    last_time = current_time;
    
    // 提取当前状态
    float q0 = x(0, 0), q1 = x(1, 0), q2 = x(2, 0), q3 = x(3, 0);
    float bgx = x(4, 0), bgy = x(5, 0), bgz = x(6, 0);
    
    // 补偿陀螺仪偏置
    float wx = gx - bgx;
    float wy = gy - bgy;
    float wz = gz - bgz;
    
    // 四元数微分方程
    float q0_dot = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    float q1_dot = 0.5f * ( q0 * wx - q3 * wy + q2 * wz);
    float q2_dot = 0.5f * ( q3 * wx + q0 * wy - q1 * wz);
    float q3_dot = 0.5f * (-q2 * wx + q1 * wy + q0 * wz);
    
    // 更新四元数
    x(0, 0) += q0_dot * dt;
    x(1, 0) += q1_dot * dt;
    x(2, 0) += q2_dot * dt;
    x(3, 0) += q3_dot * dt;
    
    // 归一化四元数
    float norm = sqrt(x(0,0)*x(0,0) + x(1,0)*x(1,0) + x(2,0)*x(2,0) + x(3,0)*x(3,0));
    if (norm > 0.0001f) {
        x(0, 0) /= norm;
        x(1, 0) /= norm;
        x(2, 0) /= norm;
        x(3, 0) /= norm;
    }
    
    // 更新协方差矩阵
    updateCovarianceMatrix(dt);
    
    // 测量更新 - 加速度计
    if (fabs(ax) + fabs(ay) + fabs(az) > 0.1f) {
        // 归一化加速度计测量值
        float accel[3] = {ax, ay, az};
        normalizeVector(accel);
        
        // 计算预期的重力向量
        float expected_gx = 2.0f * (q1*q3 - q0*q2);
        float expected_gy = 2.0f * (q2*q3 + q0*q1);
        float expected_gz = 2.0f * (q0*q0 + q3*q3) - 1.0f;
        
        // 测量残差
        float dz[3] = {
            accel[0] - expected_gx,
            accel[1] - expected_gy,
            accel[2] - expected_gz
        };
        
        // 简化的测量雅可比矩阵 H (3x7)
        Matrix H(3, 7);
        
        // 对四元数的偏导数
        H(0, 0) = -2.0f * q2; H(0, 1) =  2.0f * q3; H(0, 2) = -2.0f * q0; H(0, 3) =  2.0f * q1;
        H(1, 0) =  2.0f * q1; H(1, 1) =  2.0f * q0; H(1, 2) =  2.0f * q3; H(1, 3) =  2.0f * q2;
        H(2, 0) =  2.0f * q0; H(2, 1) = -2.0f * q1; H(2, 2) = -2.0f * q2; H(2, 3) =  2.0f * q3;
        
        // 对偏置的偏导数为0
        for (int i = 0; i < 3; i++) {
            H(i, 4) = 0.0f;
            H(i, 5) = 0.0f;
            H(i, 6) = 0.0f;
        }
        
        // 计算 H * P * H^T + R
        Matrix H_T = H.transpose();
        Matrix HP = H * P;
        Matrix HPH_T = HP * H_T;
        
        // 加上测量噪声
        for (int i = 0; i < 3; i++) {
            HPH_T(i, i) += R_accel(i, i);
        }
        
        // 计算卡尔曼增益 K = P * H^T * (HPH_T)^-1
        // 简化求逆 (假设对角矩阵)
        Matrix S_inv(3, 3);
        for(int i = 0; i < 3; i++) {
            S_inv(i, i) = 1.0f / HPH_T(i, i);
        }
        
        Matrix PH_T = P * H_T;
        Matrix K = PH_T * S_inv;
        
        // 更新状态估计
        for(int i = 0; i < 7; i++) {
            float correction = 0.0f;
            for(int j = 0; j < 3; j++) {
                correction += K(i, j) * dz[j];
            }
            x(i, 0) += correction;
        }
        
        // 更新协方差矩阵 P = (I - K * H) * P
        Matrix I = Matrix::identity(7);
        Matrix KH = K * H;
        Matrix I_KH(7, 7);
        
        for(int i = 0; i < 7; i++) {
            for(int j = 0; j < 7; j++) {
                I_KH(i, j) = I(i, j) - KH(i, j);
            }
        }
        
        P = I_KH * P;
        
        // 再次归一化四元数
        norm = sqrt(x(0,0)*x(0,0) + x(1,0)*x(1,0) + x(2,0)*x(2,0) + x(3,0)*x(3,0));
        if (norm > 0.0001f) {
            x(0, 0) /= norm;
            x(1, 0) /= norm;
            x(2, 0) /= norm;
            x(3, 0) /= norm;
        }
    }
    
    // 测量更新 - 磁力计 (类似加速度计的处理)
    // 测量更新 - 磁力计 (完整实现)
    if (fabs(mx) + fabs(my) + fabs(mz) > 0.1f) {
        // 归一化磁力计测量值
        float mag[3] = {mx, my, mz};
        normalizeVector(mag);
        
        // 获取当前四元数
        float q0 = x(0, 0), q1 = x(1, 0), q2 = x(2, 0), q3 = x(3, 0);
        
        // 计算旋转矩阵
        float R[3][3];
        quaternionToRotationMatrix(&x(0, 0), R);
        
        // 将机体坐标系的磁场向量转换到世界坐标系
        float hx = mag[0] * R[0][0] + mag[1] * R[1][0] + mag[2] * R[2][0];
        float hy = mag[0] * R[0][1] + mag[1] * R[1][1] + mag[2] * R[2][1];
        float hz = mag[0] * R[0][2] + mag[1] * R[1][2] + mag[2] * R[2][2];
        
        // 计算地磁场的水平分量（忽略Z轴干扰）
        float bx = sqrt(hx * hx + hy * hy);
        float by = 0.0f;  // 假设地磁场指向北（X轴方向）
        float bz = hz;    // 保留垂直分量
        
        // 将世界坐标系的参考磁场向量转换回机体坐标系
        float expected_mx = bx * R[0][0] + by * R[0][1] + bz * R[0][2];
        float expected_my = bx * R[1][0] + by * R[1][1] + bz * R[1][2];
        float expected_mz = bx * R[2][0] + by * R[2][1] + bz * R[2][2];
        
        // 测量残差
        float dz[3] = {
            mag[0] - expected_mx,
            mag[1] - expected_my,
            mag[2] - expected_mz
        };
        
        // 动态检测：如果残差过大，说明有磁干扰
        float residual_norm = sqrt(dz[0]*dz[0] + dz[1]*dz[1] + dz[2]*dz[2]);
        
        // 自适应测量噪声：磁干扰情况下增加噪声协方差
        Matrix R_mag_adaptive = R_mag;
        if (residual_norm > 0.5f) {  // 磁干扰阈值
            float scale = 1.0f + 20.0f * residual_norm;
            for (int i = 0; i < 3; i++) {
                R_mag_adaptive(i, i) = R_mag(i, i) * scale;
            }
        }
        
        // 磁力计测量雅可比矩阵 H (3x7)
        Matrix H(3, 7);
        
        // 对四元数的偏导数（简化版本，实际应该更复杂）
        // 这里使用近似的雅可比矩阵，完整推导很复杂
        H(0, 0) =  2.0f * (q3 * mag[1] - q2 * mag[2]);
        H(0, 1) =  2.0f * (q2 * mag[1] + q3 * mag[2]);
        H(0, 2) =  2.0f * (q1 * mag[1] + q0 * mag[2] - 2.0f * q2 * mag[0]);
        H(0, 3) =  2.0f * (q0 * mag[1] - q1 * mag[2] - 2.0f * q3 * mag[0]);
        
        H(1, 0) =  2.0f * (-q3 * mag[0] + q1 * mag[2]);
        H(1, 1) =  2.0f * (q2 * mag[0] - 2.0f * q1 * mag[1] + q0 * mag[2]);
        H(1, 2) =  2.0f * (q3 * mag[0] - 2.0f * q2 * mag[1] + q0 * mag[2]);
        H(1, 3) =  2.0f * (-q1 * mag[0] - q2 * mag[2] + 2.0f * q3 * mag[1]);
        
        H(2, 0) =  2.0f * (q2 * mag[0] - q1 * mag[1]);
        H(2, 1) =  2.0f * (q3 * mag[0] + q0 * mag[1] - 2.0f * q1 * mag[2]);
        H(2, 2) =  2.0f * (-q0 * mag[0] + q3 * mag[1] - 2.0f * q2 * mag[2]);
        H(2, 3) =  2.0f * (q1 * mag[0] + q2 * mag[1]);
        
        // 对偏置的偏导数为0（磁力计不影响陀螺仪偏置）
        for (int i = 0; i < 3; i++) {
            H(i, 4) = 0.0f;
            H(i, 5) = 0.0f;
            H(i, 6) = 0.0f;
        }
        
        // 计算 H * P * H^T + R_mag_adaptive
        Matrix H_T = H.transpose();
        Matrix HP = H * P;
        Matrix HPH_T = HP * H_T;
        
        // 加上自适应测量噪声
        for (int i = 0; i < 3; i++) {
            HPH_T(i, i) += R_mag_adaptive(i, i);
        }
        
        // 计算卡尔曼增益
        Matrix S_inv(3, 3);
        for(int i = 0; i < 3; i++) {
            S_inv(i, i) = 1.0f / HPH_T(i, i);
        }
        
        Matrix PH_T = P * H_T;
        Matrix K = PH_T * S_inv;
        
        // 限制卡尔曼增益的大小，防止磁干扰导致过大的更新
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 3; j++) {
                if (K(i, j) > 0.3f) K(i, j) = 0.3f;
                if (K(i, j) < -0.3f) K(i, j) = -0.3f;
            }
        }
        
        // 更新状态估计（主要影响航向角）
        for(int i = 0; i < 7; i++) {
            float correction = 0.0f;
            for(int j = 0; j < 3; j++) {
                correction += K(i, j) * dz[j];
            }
            // 对四元数更新施加更严格的限制（磁力计更新应该更温和）
            if (i < 4) {
                if (correction > 0.05f) correction = 0.05f;
                if (correction < -0.05f) correction = -0.05f;
            }
            x(i, 0) += correction;
        }
        
        // 更新协方差矩阵 P = (I - K * H) * P
        Matrix I = Matrix::identity(7);
        Matrix KH = K * H;
        Matrix I_KH(7, 7);
        
        for(int i = 0; i < 7; i++) {
            for(int j = 0; j < 7; j++) {
                I_KH(i, j) = I(i, j) - KH(i, j);
            }
        }
        
        P = I_KH * P;
        
        // 再次归一化四元数
        norm = sqrt(x(0,0)*x(0,0) + x(1,0)*x(1,0) + x(2,0)*x(2,0) + x(3,0)*x(3,0));
        if (norm > 0.0001f) {
            x(0, 0) /= norm;
            x(1, 0) /= norm;
            x(2, 0) /= norm;
            x(3, 0) /= norm;
        }
    }
}

// 获取欧拉角
void EKFAHRS::getEulerAngles(float& roll, float& pitch, float& yaw) {
    float q0 = x(0, 0), q1 = x(1, 0), q2 = x(2, 0), q3 = x(3, 0);
    
    // 横滚角 (绕X轴)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    // 俯仰角 (绕Y轴)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f) {
        pitch = copysign(PI / 2.0f, sinp);
    } else {
        pitch = asin(sinp);
    }
    
    // 偏航角 (绕Z轴)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// 获取四元数
void EKFAHRS::getQuaternion(float& q0, float& q1, float& q2, float& q3) {
    q0 = x(0, 0);
    q1 = x(1, 0);
    q2 = x(2, 0);
    q3 = x(3, 0);
}

// 获取陀螺仪偏置
void EKFAHRS::getGyroBias(float& bx, float& by, float& bz) {
    bx = x(4, 0);
    by = x(5, 0);
    bz = x(6, 0);
}