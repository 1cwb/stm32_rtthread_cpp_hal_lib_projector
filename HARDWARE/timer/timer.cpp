#include "timer.hpp"
    mResult timerx::init(const mDev::initCallbackExt& cb, TIM_TypeDef* TIMx, uint32_t timefreq, uint32_t countMode, 
        uint32_t repetCount, uint32_t div, bool autoReload)
    {
        uint16_t usPeriod;
        uint16_t usPrescaler;
        uint32_t uiTIMxCLK = getInputClk(TIMx);
    
        _initcb = cb;
        if(_initcb)
        {
            _initcb(true);
        }

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
        
        TimHandle.Instance = TIMx;
        TimHandle.Init.Prescaler = usPrescaler;
        TimHandle.Init.Period = usPeriod;
        TimHandle.Init.ClockDivision = div;
        TimHandle.Init.CounterMode = countMode;
        TimHandle.Init.RepetitionCounter = repetCount;
        TimHandle.Init.AutoReloadPreload = autoReload;
        if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult timerx::deInit()
    {
        if(_initcb)
        {
            _initcb(false);
        }
        HAL_TIM_Base_DeInit(&TimHandle);
        return M_RESULT_EOK;
    }
    void timerx::start()
    {
        HAL_TIM_Base_Start_IT(&TimHandle);
    }
    void timerx::stop()
    {
        HAL_TIM_Base_Stop_IT(&TimHandle);
    }
    void timerx::updateFreq(uint32_t timefreq)
    {
        uint16_t usPeriod;
        uint16_t usPrescaler;
        uint32_t uiTIMxCLK = getInputClk(TimHandle.Instance);

        RCC_ClkInitTypeDef  RCC_ClkInitStruct;
        uint32_t pFLatency;
        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

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
        TimHandle.Init.Prescaler = usPrescaler;
        TimHandle.Init.Period = usPeriod;
        __HAL_TIM_SET_PRESCALER(&TimHandle, usPrescaler);
        __HAL_TIM_SET_AUTORELOAD(&TimHandle, usPeriod);
        HAL_TIM_GenerateEvent(&TimHandle, TIM_EVENTSOURCE_UPDATE);
    }
    uint32_t timerx::getTimeOut()
    {
        return TimHandle.Init.Period*1000000000/(getInputClk(TimHandle.Instance)/TimHandle.Init.Prescaler);
    }