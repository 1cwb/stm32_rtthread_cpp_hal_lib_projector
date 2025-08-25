#pragma once
#include "mpwmdrv.hpp"
#include "timer.hpp"
#include "stm32h7xx_hal_conf.h"

class PWMX : public mDev::mPWM
{
public:
    PWMX(const char* name, timerx* newTimer):mDev::mPWM(name),_newTimer(newTimer){}
    ~PWMX(){}
    mResult pwmConfig(TIM_OC_InitTypeDef* TIM_OCInitStructure, uint32_t channel, TIM_BreakDeadTimeConfigTypeDef* TIM_BDTRInitStructure = nullptr, bool enableNch = false)
    {
        if(!_newTimer)
        {
            return M_RESULT_ERROR;
        }
        if(TIM_BDTRInitStructure)
        {
            _bUseBreakDeadTime = true;
        }
        _bEnableNch = enableNch;
        _channel = channel;
        
        if(HAL_TIM_PWM_ConfigChannel(_newTimer->getTimHandle(),TIM_OCInitStructure,channel) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        if(_bUseBreakDeadTime)
        {
            if(HAL_TIMEx_ConfigBreakDeadTime(_newTimer->getTimHandle(), TIM_BDTRInitStructure) != HAL_OK)
            {
                return M_RESULT_ERROR;
            }
        }
        return M_RESULT_EOK;
    }
    virtual void updateFreq(uint32_t timefreq)override
    {
        if(_newTimer)
        {
            _newTimer->updateFreq(timefreq);
        }
    }
    virtual uint32_t getFreq() override
    {
        if(_newTimer)
        {
            return _newTimer->getFreq();
        }
        return 0;
    }
    virtual void start()override
    {
        if(!_newTimer)
        {
            return;
        }
        HAL_TIM_PWM_Start(_newTimer->getTimHandle(),_channel);
        if(_bEnableNch)
        {
            HAL_TIMEx_PWMN_Start(_newTimer->getTimHandle(),_channel);
        }
    }
    virtual void stop()override
    {
        if(!_newTimer)
        {
            return;
        }
        HAL_TIM_PWM_Stop(_newTimer->getTimHandle(),_channel);
        if(_bEnableNch)
        {
            HAL_TIMEx_PWMN_Stop(_newTimer->getTimHandle(),_channel);
        }
    }
    virtual void updatePulse(uint32_t pulse)override
    {
        __HAL_TIM_SET_COMPARE(_newTimer->getTimHandle(),_channel,pulse);
    }
    virtual uint32_t getMaxPulse()override
    {
        if(!_newTimer)
        {
            return 0;
        }
        return _newTimer->getTimHandle()->Init.Period;
    }
    virtual uint32_t getCurPulse()override
    {
        return __HAL_TIM_GET_COMPARE(_newTimer->getTimHandle(), _channel);
    }
    virtual void setDutyCycle(float dutyCycle)override
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
    virtual float getDutyCycle()override
    {
        float dutyCycle = 0.0f;
        dutyCycle = (getCurPulse() + 1)*100.0f / getMaxPulse();
        return dutyCycle;
    }
private:
    timerx* _newTimer = nullptr;
    uint32_t _channel = 0;
    bool _bUseBreakDeadTime = false;
    bool _bEnableNch = false;
};