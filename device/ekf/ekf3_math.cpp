#include <cmath>
#include "ekf3_config.hpp"

static inline float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

static inline void quatNormalize(float q[4]) {
    float n = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i=0;i<4;i++) q[i] *= n;
}

static inline void quatRotate(const float q[4], const float vin[3], float vout[3]) {
    float t[3] = {
        2*(q[1]*q[2] + q[0]*q[3]),
        2*(q[2]*q[3] - q[0]*q[1]),
        q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]
    };
    vout[0] = vin[0]*t[2] + vin[1]*t[0] + vin[2]*t[1];
    vout[1] = vin[0]*t[1] + vin[1]*t[2] - vin[2]*t[0];
    vout[2] = vin[0]*t[0] - vin[1]*t[1] + vin[2]*t[2];
}