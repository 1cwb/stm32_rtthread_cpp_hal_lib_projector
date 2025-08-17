#include "newtimer.hpp"

mResult newTimerx::baseTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle)
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
mResult newTimerx::baseTimeDeInit()
{
    HAL_TIM_Base_DeInit(&_TimHandle);
    _btimerInited = false;
    return M_RESULT_EOK;
}

void newTimerx::updateFreq(uint32_t timefreq)
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
void newTimerx::calcPeriodAndPrescalerByFreq(TIM_HandleTypeDef* TimHandle, uint32_t timefreq)
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
uint32_t newTimerx::getInputClk(TIM_TypeDef* TIMx)
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
extern "C" void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim == newTimerx::GetObjectFromPrivateMember(htim)->getTimHandle()) {
        newTimerx::GetObjectFromPrivateMember(htim)->runInitCallback(true);
    }
}
extern "C" void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
    if(htim == newTimerx::GetObjectFromPrivateMember(htim)->getTimHandle()) {
        newTimerx::GetObjectFromPrivateMember(htim)->runInitCallback(false);
    }
}