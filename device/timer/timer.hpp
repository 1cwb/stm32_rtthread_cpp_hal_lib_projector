#pragma once
#include <cstddef>
#include "mtimerdrv.hpp"
#include "stm32h7xx_hal_conf.h"

class timerx :public mDev::mTimer
{
public:
    timerx(const char* name) : mDev::mTimer(name)
    {}
    virtual ~timerx() = default;

    mResult baseTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle);
    mResult baseTimeDeInit();

    virtual void updateFreq(uint32_t timefreq) override;
    virtual uint32_t getFreq() override
    {
        return _timeFreq;
    }
    virtual uint32_t getTimeOutUs() override
    {
        return (uint32_t)(((float)((_TimHandle.Init.Period+1)*1000.0f))/((float)(getInputClk(_TimHandle.Instance)/(_TimHandle.Init.Prescaler+1))) * 1000U);
    }
    virtual void start() override
    {
        HAL_TIM_Base_Start_IT(&_TimHandle);
    }
    virtual void stop() override
    {
        HAL_TIM_Base_Stop_IT(&_TimHandle);
    }
    virtual bool isInitialized() override
    {
        return _btimerInited;
    }
    void calcPeriodAndPrescalerByFreq(TIM_HandleTypeDef* TimHandle, uint32_t timefreq);
    TIM_HandleTypeDef* getTimHandle()
    {
        return &_TimHandle;
    }
    static timerx* GetObjectFromPrivateMember(TIM_HandleTypeDef* member_address) {
            // 使用模板函数，传入成员指针和地址
            return GetObjectFromMember(&timerx::_TimHandle, member_address);
    }
private:
    uint32_t getInputClk(TIM_TypeDef* TIMx);
private:
    TIM_HandleTypeDef _TimHandle = {0};
    uint32_t _timeFreq = 0;
    bool _btimerInited = false;
};