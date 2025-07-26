#pragma once
#include "MiniEKF3.hpp"
#include <vector>
#include <numeric>

class GyroCal {
public:
    // still_threshold : rad/s，低于此值认为是静止
    // samples         : 静止后取多少个点做平均
    explicit GyroCal(float still_threshold = 0.02f, size_t samples = 200)
        : thr_(still_threshold), N_(samples) {}

    // 每次收到陀螺原始数据就调用
    // 返回 true 表示校准完成，bias 已写入 ekf
    bool update(MiniEKF3& ekf, const Vec3& raw_gyro) {
        if (cal_done_) return true;

        // 滑动窗口
        buf_.push_back(raw_gyro);
        if (buf_.size() > N_) buf_.erase(buf_.begin());

        // 判断整窗是否静止
        if (buf_.size() == N_) {
            Vec3 avg = std::accumulate(buf_.begin(), buf_.end(), Vec3::zero());
            avg = avg / float(N_);

            if ((avg - Vec3::zero()).norm() < thr_) {
                ekf.setGyroBias(avg);   // 见下方扩展
                cal_done_ = true;
                return true;
            }
        }
        return false;
    }

    bool done() const { return cal_done_; }

private:
    bool cal_done_ = false;
    float thr_;
    size_t N_;
    std::vector<Vec3> buf_;
};