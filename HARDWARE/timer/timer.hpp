#pragma once
#include "mtimer.hpp"
#include "stm32h7xx_hal_conf.h"

class timerx : public mDev::mTimer
{
public:
    timerx(const char* name):mDev::mTimer(name){};
    ~timerx() = default;
    mResult baseInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle);
    mResult baseDeInit();
    mResult ocInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle);
    mResult ocDeInit();
    mResult pwmInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle);
    mResult pwmDeInit();
    mResult icInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle);
    mResult icDeInit();
    mResult onePulseInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, uint32_t OnePulseMode);
    mResult onePulseDeInit();
    mResult encoderInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, TIM_Encoder_InitTypeDef *sConfig);
    mResult encoderDeInit();
    void calcPeriodAndPrescalerByFreq(TIM_HandleTypeDef* TimHandle, uint32_t timefreqd);
    virtual void start() override;
    virtual void stop() override;
    virtual void updateFreq(uint32_t timefreq) override;
    virtual uint32_t getTimeOut() override;
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
    TIM_HandleTypeDef _TimHandle = {0};
};
