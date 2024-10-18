#pragma once
#include "mscheduler.hpp"
#include "mthread.hpp"
#include "containers.hpp"
#include "mIdle.hpp"
#include "msystickdrv.hpp"

#define CPU_USAGE_CALC_TICK    10
#define CPU_USAGE_LOOP        100

class systemInfo
{
public:
    static systemInfo* getInstance()
    {
        static systemInfo sysinfo;
        return &sysinfo;
    }
    mResult init();
    float getCpuUsage(void)
    {
        printf("CPU Usage = %d.%d%%% \r\n",cpuUsageMajor,cpuUsageMinor);
        return 0.0f;
    }
    void showAllThreadStackSizeInfo();
private:

    systemInfo():cpuUsageMajor(0),cpuUsageMinor(0),totalCount(0),idle(nullptr),systime(nullptr)
    {
    }
    ~systemInfo()
    {

    }
    systemInfo(const systemInfo&) = delete;
    systemInfo(systemInfo&&) = delete;
    systemInfo& operator=(const systemInfo&) = delete;
    systemInfo& operator=(systemInfo&&) = delete;
    uint8_t  cpuUsageMajor = 0;
    uint8_t cpuUsageMinor= 0;
    uint32_t totalCount = 0;
    mIdle* idle;
    mDev::mSystick* systime;
};