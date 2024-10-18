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
    mResult init()
    {
        idle = mIdle::getInstance();
        if(!idle)
        {
            printf("Error: idleThread not found\r\n");
            return M_RESULT_ENOSYS;
        }
        systime = (mDev::mSystick*)mDev::mPlatform::getInstance()->getDevice("systick");
        if(!systime)
        {
            printf("Error: systime driver not found\r\n");
            return M_RESULT_EINVAL;
        }
        idle->regitserHookCallback([this](){
            uint64_t tick = 0;
            uint64_t count = 0;
            volatile uint32_t loop = 0;

        if (totalCount == 0)
        {                             
            /* get total count */
            mSchedule::getInstance()->enterCritical();                             
            tick = mClock::getInstance()->tickGet();                               
            while (mClock::getInstance()->tickGet() - tick < CPU_USAGE_CALC_TICK)
            {
                    totalCount ++;                         
                    loop = 0;
                    while (loop < CPU_USAGE_LOOP) loop ++;
            }
            mSchedule::getInstance()->exitCritical();  
        }
        count = 0;
        /* get CPU usage */
        tick = mClock::getInstance()->tickGet();                           
        while (mClock::getInstance()->tickGet() - tick < CPU_USAGE_CALC_TICK)
        {
                count ++;                                   
                loop  = 0;
                while (loop < CPU_USAGE_LOOP) loop ++;
        }

        /* calculate major and minor */
        if (count < totalCount) 
        {                          
            count = totalCount - count;
            cpuUsageMajor = (count * 100) / totalCount;
            cpuUsageMinor = ((count * 100) % totalCount) * 100 / totalCount;
        }
        else
        {
                totalCount = count;                                

                /* no CPU usage */
                cpuUsageMajor = 0;
                cpuUsageMinor = 0;
            }
        });
        
        return M_RESULT_EOK;
    }
    float getCpuUsage(void)
    {
        rt_kprintf("CPU利用率 = %d.%d%\r\n",cpuUsageMajor,cpuUsageMinor);
        return 0.0f;
    }
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