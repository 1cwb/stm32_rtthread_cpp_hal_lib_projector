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
        _channelNum |= remapTimChannelToChannel(channel);
        
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
    }
    virtual void start(mDev::mCHANNEL channel = mDev::mCHANNEL::mCHANNEL_INVALED)override
    {
        if(!_newTimer)
        {
            return;
        }
        HAL_TIM_PWM_Start(_newTimer->getTimHandle(),remapChannelToTimChannel(channel));
        if(_bEnableNch)
        {
            HAL_TIMEx_PWMN_Start(_newTimer->getTimHandle(),remapChannelToTimChannel(channel));
        }
    }
    virtual void stop(mDev::mCHANNEL channel)override
    {
        if(!_newTimer)
        {
            return;
        }
        HAL_TIM_PWM_Stop(_newTimer->getTimHandle(),remapChannelToTimChannel(channel));
        if(_bEnableNch)
        {
            HAL_TIMEx_PWMN_Stop(_newTimer->getTimHandle(),remapChannelToTimChannel(channel));
        }
    }
    virtual void pwmUpdatePulse(uint32_t pulse, mDev::mCHANNEL channel)override
    {
        __HAL_TIM_SET_COMPARE(_newTimer->getTimHandle(),remapChannelToTimChannel(channel),pulse);
    }
    virtual uint32_t pwmGetMaxPulse()override
    {
        if(!_newTimer)
        {
            return 0;
        }
        return _newTimer->getTimHandle()->Init.Period;
    }
    virtual uint32_t pwmGetCurPulse(mDev::mCHANNEL channel)override
    {
        return __HAL_TIM_GET_COMPARE(_newTimer->getTimHandle(), remapChannelToTimChannel(channel));
    }
    virtual void pwmSetDutyCycle(float dutyCycle, mDev::mCHANNEL channel)override
    {
        if(dutyCycle < 1.0f)
        {
            dutyCycle = 1.0f;
        }
        if(dutyCycle > 100.0f)
        {
            dutyCycle = 100.0f;
        }
        uint32_t pulse = (uint32_t)((pwmGetMaxPulse() * dutyCycle) / 100.0f) - 1;
        pwmUpdatePulse(pulse,channel);
    }
    virtual float pwmGetDutyCycle(mDev::mCHANNEL channel)override
    {
        float dutyCycle = 0.0f;
        dutyCycle = (pwmGetCurPulse(channel) + 1)*100.0f / pwmGetMaxPulse();
        return dutyCycle;
    }
    bool isChannelInited(mDev::mCHANNEL ch)
    {
        if(_channelNum & (uint32_t)ch)
        {
            return true;
        }
        return false;
    }
private:
    uint32_t remapChannelToTimChannel(mDev::mCHANNEL ch)
    {
        switch(ch)
        {
            case mDev::mCHANNEL::CHANNEL_1: return TIM_CHANNEL_1;
            case mDev::mCHANNEL::CHANNEL_2: return TIM_CHANNEL_2;
            case mDev::mCHANNEL::CHANNEL_3: return TIM_CHANNEL_3;
            case mDev::mCHANNEL::CHANNEL_4: return TIM_CHANNEL_4;
            case mDev::mCHANNEL::CHANNEL_5: return TIM_CHANNEL_5;
            case mDev::mCHANNEL::CHANNEL_6: return TIM_CHANNEL_6;
            case mDev::mCHANNEL::CHANNEL_ALL: return TIM_CHANNEL_ALL;
            default:
            return 0;
        }
    }
    uint32_t remapTimChannelToChannel(uint32_t timch)
    {
        switch(timch)
        {
            case TIM_CHANNEL_1 : return (uint32_t)mDev::mCHANNEL::CHANNEL_1;
            case TIM_CHANNEL_2 : return (uint32_t)mDev::mCHANNEL::CHANNEL_2;
            case TIM_CHANNEL_3 : return (uint32_t)mDev::mCHANNEL::CHANNEL_3;
            case TIM_CHANNEL_4 : return (uint32_t)mDev::mCHANNEL::CHANNEL_4;
            case TIM_CHANNEL_5 : return (uint32_t)mDev::mCHANNEL::CHANNEL_5;
            case TIM_CHANNEL_6 : return (uint32_t)mDev::mCHANNEL::CHANNEL_6;
            case TIM_CHANNEL_ALL: return (uint32_t)mDev::mCHANNEL::CHANNEL_ALL;
            default:
            return 0;
        }
    }
private:
    timerx* _newTimer = nullptr;
    uint32_t _channelNum = 0;
    bool _bUseBreakDeadTime = false;
    bool _bEnableNch = false;
};