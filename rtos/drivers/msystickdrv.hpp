#pragma once
#include "mdevice.hpp"
#include "mclock.hpp"
#include "mhw.hpp"

namespace mDev
{
class mSystick : public mDevice
{
public:
    mSystick(const char* name):
    mDevice(name),
    clk(mClock::getInstance()){}
    virtual ~mSystick() {}
    uint64_t systimeNowUs()
    {
        uint64_t tickUs = 0;
        uint64_t tickMs = 0;
        uint16_t level = 0;
        level = HW::hwInterruptDisable();
        tickMs = clk->tickGet();
        tickUs = getSystickPeriod();
        HW::hwInterruptEnable(level);
        return (uint64_t)tickMs*(uint64_t)1000 + (uint64_t)((float)tickUs * getTickPerUs());
    }
    uint64_t systimeNowMs()
    {
        uint64_t tickMs = 0;
        uint16_t level = 0;
        level = HW::hwInterruptDisable();
        tickMs = clk->tickGet();
        HW::hwInterruptEnable(level);
        return tickMs * (uint32_t)(1000 / THREAD_TICK_PER_SECOND);
    }
    void delayUs(uint64_t us)
    {
        uint64_t target = systimeNowUs() + us;
        while (systimeNowUs() < target);
    }
    void mdelayMs(uint64_t ms)
    {
        uint64_t target = systimeNowMs() + ms;
        while (systimeNowMs() < target);
    }
protected:
    virtual uint32_t getSystickPeriod() const {return 0;}
    virtual float getTickPerUs () const {return 0;}
private:
    mClock* clk;
};
}