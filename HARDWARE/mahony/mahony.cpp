#include "mahony.hpp"
//互补滤波函数
//输入参数：g表陀螺仪角速度(弧度/s)，a表加计（m/s2或g都可以，会归一化）
void Mahony::MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az)
{
 
	float q0temp,q1temp,q2temp,q3temp;//四元数暂存变量，求解微分方程时要用
	float norm; //矢量的模或四元数的范数
	float vx, vy, vz;//当前姿态计算得来的重力在三轴上的分量
	float ex, ey, ez;//当前加计测得的重力加速度在三轴上的分量
	//与用当前姿态计算得来的重力在三轴上的分量的误差
 
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q1q1 = q1*q1;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	if(ax*ay*az==0)//加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
		return;
	norm = sqrt(ax*ax + ay*ay + az*az);//单位化加速度计，
	ax = ax / norm;// 这样变更了量程也不需要修改KP参数，因为这里归一化了
	ay = ay / norm;
	az = az / norm;
	
	//用当前姿态计算出重力在三个轴上的分量，重力在n系下是[0,0,g]，乘以转换矩阵就转到b系
	//参考坐标n系转化到载体坐标b系，用四元数表示的方向余弦矩阵第三行即是
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	//计算测得的重力与计算得重力间的误差，这个误差是通过向量外积（也就是叉乘）求出来的
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
 
	exInt = exInt + ex * Ki;  //对误差进行积分
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	gx = gx + Kp*ex + exInt;  //将误差PI（比例和积分项）补偿到陀螺仪角速度
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;  //没有磁力计的话就无法修正偏航角
	
	//下面进行姿态的更新，也就是四元数微分方程的求解
	q0temp=q0;
	q1temp=q1;
	q2temp=q2;
	q3temp=q3;
	//采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
	q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
	q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
	q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
	q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
	
	//单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	//四元数到欧拉角的转换
	//其中YAW航向角由于加速度计对其没有修正作用，因此此处直接用陀螺仪积分代替
	//Q_ANGLE_Z = Q_ANGLE_Z + gz*halfT*2*57.3; // yaw
	Q_ANGLE_Z = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
	Q_ANGLE_Y = asin(-2 * q1 * q3 + 2 * q0* q2)*57.3; // pitch
	Q_ANGLE_X = atan2(2 * q2 * q3 + 2 * q0 * q1,-2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
}