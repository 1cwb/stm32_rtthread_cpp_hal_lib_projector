#pragma once
#include "mtimerdrv.hpp"
#include "stm32h7xx_hal_conf.h"

class timerx : public mDev::mTimer
{
public:
    timerx(const char* name):mDev::mTimer(name){};
    ~timerx() = default;
    mResult baseTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, mDev::TIMESTARTMODE startMode = mDev::TIMESTARTMODE_NOMAL);
    mResult baseTimeDeInit();
    mResult pwmTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, mDev::TIMESTARTMODE startMode = mDev::TIMESTARTMODE_NOMAL);
    mResult pwmTimeDeInit();
    mResult pwmConfig(TIM_OC_InitTypeDef* TIM_OCInitStructure, uint32_t channel, TIM_BreakDeadTimeConfigTypeDef* TIM_BDTRInitStructure = nullptr, bool enableNch = false);
    mResult icTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, mDev::TIMESTARTMODE startMode = mDev::TIMESTARTMODE_NOMAL);
    mResult icTimeDeInit();
    mResult icConfig(TIM_IC_InitTypeDef *sConfig, uint32_t channel);
    mResult encoderTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, TIM_Encoder_InitTypeDef *sConfig, mDev::TIMESTARTMODE startMode = mDev::TIMESTARTMODE_NOMAL);
    mResult encoderTimeDeInit();

    void calcPeriodAndPrescalerByFreq(TIM_HandleTypeDef* TimHandle, uint32_t timefreqd);
    virtual void start(mDev::CHANNLE channel = mDev::CHANNLE::CHANNLE_INVALED, uint32_t* data1 = nullptr, uint32_t* data2 = nullptr, size_t len = 0) override;
    virtual void stop(mDev::CHANNLE channel) override;
    virtual void updateFreq(uint32_t timefreq) override;
    virtual uint32_t getTimeOutUs() override;

    virtual void pwmUpdatePulse(uint32_t pulse, mDev::CHANNLE channel)override{__HAL_TIM_SET_COMPARE(&_TimHandle,remapChannelToTimChannel(channel),pulse);}
    virtual uint32_t pwmGetMaxPulse()override{return _TimHandle.Init.Period;}
    virtual uint32_t pwmGetCurPulse(mDev::CHANNLE channel)override{return __HAL_TIM_GET_COMPARE(&_TimHandle, remapChannelToTimChannel(channel));}
    virtual void pwmSetDutyCycle(float dutyCycle, mDev::CHANNLE channel) override
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
    virtual float pwmGetDutyCycle(mDev::CHANNLE channel) override
    {
        float dutyCycle = 0.0f;
        dutyCycle = (pwmGetCurPulse(channel) + 1)*100.0f / pwmGetMaxPulse();
        return dutyCycle;
    }

    TIM_HandleTypeDef* getTimHandle()
    {
        return &_TimHandle;
    }
private:
    uint32_t getInputClk(TIM_TypeDef* TIMx)
    {
        uint32_t uiTIMxCLK;
        RCC_ClkInitTypeDef  RCC_ClkInitStruct;
        uint32_t pFLatency;
        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);
        if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM15) || (TIMx == TIM16) || (TIMx == TIM17))
        {
            /* APB2 定时器时钟 */
            if(RCC_ClkInitStruct.APB2CLKDivider != RCC_APB2_DIV1)
            {
                uiTIMxCLK = HAL_RCC_GetPCLK2Freq() * 2;
            }
            else
            {
                uiTIMxCLK = HAL_RCC_GetPCLK2Freq();
            }
        }
        else
        {
            /* APB1 定时器*/
            if(RCC_ClkInitStruct.APB1CLKDivider != RCC_APB1_DIV1)
            {
                uiTIMxCLK = HAL_RCC_GetPCLK1Freq() * 2;
            }
            else
            {
                uiTIMxCLK = HAL_RCC_GetPCLK1Freq();
            }
        }
        UNUSED(&pFLatency);
        return uiTIMxCLK;
    }
    uint32_t remapChannelToTimChannel(mDev::CHANNLE ch)
    {
        switch(ch)
        {
            case mDev::CHANNLE::CHANNEL_1: return TIM_CHANNEL_1;
            case mDev::CHANNLE::CHANNEL_2: return TIM_CHANNEL_2;
            case mDev::CHANNLE::CHANNEL_3: return TIM_CHANNEL_3;
            case mDev::CHANNLE::CHANNEL_4: return TIM_CHANNEL_4;
            case mDev::CHANNLE::CHANNEL_5: return TIM_CHANNEL_5;
            case mDev::CHANNLE::CHANNEL_6: return TIM_CHANNEL_6;
            case mDev::CHANNLE::CHANNEL_ALL: return TIM_CHANNEL_ALL;
            default:
            return 0;
        }
    }
    uint32_t remapTimChannelToChannel(uint32_t timch)
    {
        switch(timch)
        {
            case TIM_CHANNEL_1 : return (uint32_t)mDev::CHANNLE::CHANNEL_1;
            case TIM_CHANNEL_2 : return (uint32_t)mDev::CHANNLE::CHANNEL_2;
            case TIM_CHANNEL_3 : return (uint32_t)mDev::CHANNLE::CHANNEL_3;
            case TIM_CHANNEL_4 : return (uint32_t)mDev::CHANNLE::CHANNEL_4;
            case TIM_CHANNEL_5 : return (uint32_t)mDev::CHANNLE::CHANNEL_5;
            case TIM_CHANNEL_6 : return (uint32_t)mDev::CHANNLE::CHANNEL_6;
            case TIM_CHANNEL_ALL: return (uint32_t)mDev::CHANNLE::CHANNEL_ALL;
            default:
            return 0;
        }
    }
    bool isChannelInited(mDev::CHANNLE ch)
    {
        if(_channelNum & (uint32_t)ch)
        {
            return true;
        }
        return false;
    }
public:
    TIM_HandleTypeDef _TimHandle = {0};
    uint32_t _channelNum = 0;
    bool _bUseBreakDeadTime = false;
    bool _bEnableNch = false;
};
