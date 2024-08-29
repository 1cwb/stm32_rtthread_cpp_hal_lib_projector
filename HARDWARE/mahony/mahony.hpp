#pragma once
#include <math.h>

class Mahony
{
public:
    Mahony()
    {

    }
    ~Mahony(){}
    Mahony(const Mahony&) = delete;
    Mahony(Mahony&&) = delete;
    Mahony& operator=(const Mahony&) = delete;
    Mahony& operator=(Mahony&&) = delete;

    //输入参数：g表陀螺仪角速度(弧度/s)，a表加计（m/s2或g都可以，会归一化）
    void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az);
    float getAngleX() const {return Q_ANGLE_X;}
    float getAngleY() const {return Q_ANGLE_Y;}
    float getAngleZ() const {return Q_ANGLE_Z;}
private:
    const float Kp = 10.0f;                  // 这里的KpKi是用于调整加速度计修正陀螺仪的速度
    const float Ki = 0.008f;
    const float halfT = 0.05f;               // 采样周期的一半，用于求解四元数微分方程时计算角增量

    float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始位置姿态角为：0、0、0，对应四元数为：1、0、0、0
    float exInt = 0, eyInt = 0, ezInt = 0;   //重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差的积分

    float  Q_ANGLE_X= 0, Q_ANGLE_Y = 0, Q_ANGLE_Z = 0;
};