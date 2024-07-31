#pragma once
#include "mtimer.hpp"
#include "stm32h7xx_hal_conf.h"

class timerx : public mDev::mTimer
{
public:
    timerx(const char* name, TIM_TypeDef* TIMx, uint32_t timefreq,
    uint32_t countMode = TIM_COUNTERMODE_UP,
    uint32_t repetCount = 0,
    uint32_t div = TIM_CLOCKDIVISION_DIV1,
    bool autoReload = true);
    ~timerx() = default;
    virtual void updateFreq(uint32_t timefreq);
    virtual uint32_t getTimeOut();
private:

};
