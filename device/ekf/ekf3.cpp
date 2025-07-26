#include "ekf3.hpp"
#include "ekf3_core.cpp"

void Ekf3::init() {
    for (int i=0;i<EKF3_STATE_DIM;i++)
        for (int j=0;j<EKF3_STATE_DIM;j++)
            P[i][j] = (i==j) ? 1.0f : 0.0f;
    x[0]=1; x[1]=x[2]=x[3]=0;
    q[0]=1; q[1]=q[2]=q[3]=0;
}

void Ekf3::update(const Ekf3Input& in, Ekf3Output& out) {
    predict(in.dt, &in.gx, &in.ax);
    //fuseMag(&in.mx);
    quat2euler(q, out.roll, out.pitch, out.yaw);
}