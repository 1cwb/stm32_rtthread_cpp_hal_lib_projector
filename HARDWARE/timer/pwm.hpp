#pragma once
#include "mpwm.hpp"
#include "timer.hpp"
#include "stm32h7xx_hal_conf.h"

class pwmx : public mDev::mPwm
{
public:
    pwmx(const char* name, mDev::mTimer* timerx):mDev::mPwm(name, timerx){}
    virtual ~pwmx() = default;
    mResult init(const mDev::initCallbackExt& cb, TIM_OC_InitTypeDef* TIM_OCInitStructure, uint32_t channel, TIM_BreakDeadTimeConfigTypeDef* TIM_BDTRInitStructure = nullptr, bool enableNch = false);
    mResult deInit();
    virtual void start()override;
    virtual void stop()override;
    virtual void updatePulse(uint32_t pulse)override{__HAL_TIM_SET_COMPARE(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(),_channel,pulse);}
    virtual uint32_t getMaxPulse()override{return reinterpret_cast<timerx*>(this->getTimer())->getTimHandle()->Init.Period;}
    virtual uint32_t getCurPulse()override{return __HAL_TIM_GET_COMPARE(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(), _channel);}
    virtual void setDutyCycle(float dutyCycle) override
    {
        if(dutyCycle < 1.0f)
        {
            dutyCycle = 1.0f;
        }
        if(dutyCycle > 100.0f)
        {
            dutyCycle = 100.0f;
        }
        uint32_t pulse = (uint32_t)((getMaxPulse() * dutyCycle) / 100.0f) - 1;
        updatePulse(pulse);
    }
    virtual float getDutyCycle() override
    {
        float dutyCycle = 0.0f;
        dutyCycle = (getCurPulse() + 1)*100.0f / getMaxPulse();
        return dutyCycle;
    } 
private:
    bool _bUseBreakDeadTime = false;
    uint32_t _channel;
    bool _bEnableNch = false;
};