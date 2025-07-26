#pragma once
#include <cstdint>

struct Ekf3Input {
    float dt;            // s
    float gx,gy,gz;      // rad/s
    float ax,ay,az;      // m/s²
    float mx,my,mz;      // μT
};

struct Ekf3Output {
    float roll, pitch, yaw; // deg
};

class Ekf3 {
public:
    void init();
    void update(const Ekf3Input& in, Ekf3Output& out);
};