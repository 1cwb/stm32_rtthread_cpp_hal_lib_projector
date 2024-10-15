#pragma once
#include "msystickdrv.hpp"
#include "sys.h"

class systick : public mDev::mSystick
{
public:
    systick():mSystick("systick"),tickPerUs(1000000.0F/(float)HAL_RCC_GetSysClockFreq()){}
    virtual ~systick() {}
    void init()
    {
    }
protected:
    virtual uint32_t getSystickPeriod() const override
    {
        return SysTick->VAL;
    }
    virtual float getTickPerUs () const override
    {
        return tickPerUs;
    }

private:
    float tickPerUs;
};