#pragma once
#include <cstddef>
#include "mnewtimerdrv.hpp"
#include "stm32h7xx_hal_conf.h"

class newTimerx :public mDev::mNewTimer
{
public:
    newTimerx(const char* name) : mDev::mNewTimer(name)
    {}
    virtual ~newTimerx() = default;

    mResult baseTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle)
    {
        _initcb = cb;
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));
        HAL_TIM_Base_DeInit(&_TimHandle);
        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        if (HAL_TIM_Base_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        _btimerInited = true;
        return M_RESULT_EOK;
    }
    mResult baseTimeDeInit()
    {
        HAL_TIM_Base_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }

    //virtual void updateFreq(uint32_t timefreq) override;
    //virtual uint32_t getFreq() override;
    //virtual uint32_t getTimeOutUs() override;
    //virtual void start() override;
    //virtual void stop() override;
    //virtual bool isInitialized() override;

    virtual void updateFreq(uint32_t timefreq) override
    {
        uint16_t usPeriod;
        uint16_t usPrescaler;
        uint32_t uiTIMxCLK = getInputClk(_TimHandle.Instance);

        RCC_ClkInitTypeDef  RCC_ClkInitStruct;
        uint32_t pFLatency;
        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);
        UNUSED(&pFLatency);

        _timeFreq = timefreq;
        if (_timeFreq < 100)
        {
            usPrescaler = 10000 - 1; /* 分频比 = 10000 */
            usPeriod = (uiTIMxCLK / 10000) / _timeFreq - 1; /* 自动重装的值 */
        }
        else if (_timeFreq < 3000)
        {
            usPrescaler = 100 - 1; /* 分频比 = 100 */
            usPeriod = (uiTIMxCLK / 100) / _timeFreq - 1; /* 自动重装的值 */
        }
        else /* 大于 4K 的频率，无需分频 */
        {
            usPrescaler = 0; /* 分频比 = 1 */
            usPeriod = uiTIMxCLK / _timeFreq - 1; /* 自动重装的值 */
        }
        _TimHandle.Init.Prescaler = usPrescaler;
        _TimHandle.Init.Period = usPeriod;
        __HAL_TIM_SET_PRESCALER(&_TimHandle, usPrescaler);
        __HAL_TIM_SET_AUTORELOAD(&_TimHandle, usPeriod);
        HAL_TIM_GenerateEvent(&_TimHandle, TIM_EVENTSOURCE_UPDATE);
    }
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
    void calcPeriodAndPrescalerByFreq(TIM_HandleTypeDef* TimHandle, uint32_t timefreq)
    {
        uint16_t usPeriod;
        uint16_t usPrescaler;
        uint32_t uiTIMxCLK = getInputClk(TimHandle->Instance);
        _timeFreq = timefreq;

        if (_timeFreq < 100)
        {
            usPrescaler = 10000 - 1; /* 分频比 = 10000 */
            usPeriod = (uiTIMxCLK / 10000) / _timeFreq - 1; /* 自动重装的值 */
        }
        else if (_timeFreq < 3000)
        {
            usPrescaler = 100 - 1; /* 分频比 = 100 */
            usPeriod = (uiTIMxCLK / 100) / _timeFreq - 1; /* 自动重装的值 */
        }
        else /* 大于 4K 的频率，无需分频 */
        {
            usPrescaler = 0; /* 分频比 = 1 */
            usPeriod = uiTIMxCLK / _timeFreq - 1; /* 自动重装的值 */
        }
        TimHandle->Init.Prescaler = usPrescaler;
        TimHandle->Init.Period = usPeriod;
    }
    const TIM_HandleTypeDef* getTimHandle() const
    {
        return &_TimHandle;
    }

    static newTimerx* GetObjectFromPrivateMember(TIM_HandleTypeDef* member_address) {
        // 临时对象用于计算偏移量（不调用构造函数）
        alignas(newTimerx) char temp_buf[sizeof(newTimerx)]{};
        newTimerx* temp_obj = reinterpret_cast<newTimerx*>(temp_buf);
        
        // 计算 _TimHandle 的偏移量
        uintptr_t offset = reinterpret_cast<uintptr_t>(&temp_obj->_TimHandle) - reinterpret_cast<uintptr_t>(temp_obj);
        
        // 返回实际对象地址
        return reinterpret_cast<newTimerx*>(reinterpret_cast<uintptr_t>(member_address) - offset);
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
private:
    TIM_HandleTypeDef _TimHandle = {0};
    uint32_t _timeFreq = 0;
    bool _btimerInited = false;
};