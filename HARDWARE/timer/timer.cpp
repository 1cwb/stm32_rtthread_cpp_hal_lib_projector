#include "timer.hpp"
    void timerx::calcPeriodAndPrescalerByFreq(TIM_HandleTypeDef* TimHandle, uint32_t timefreq)
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
    mResult timerx::baseInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle)
    {
 
        _initcb = cb;
        if(_initcb)
        {
            _initcb(true);
        }
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));
        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        if (HAL_TIM_Base_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        _btimerInited = true;
        return M_RESULT_EOK;
    }
    mResult timerx::baseDeInit()
    {
        if(_initcb)
        {
            _initcb(false);
        }
        HAL_TIM_Base_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    mResult timerx::ocInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle)
    {
        _initcb = cb;
        if(_initcb)
        {
            _initcb(true);
        }

        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));

        if (HAL_TIM_OC_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult timerx::ocDeInit()
    {
        if(_initcb)
        {
            _initcb(false);
        }
        HAL_TIM_OC_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    mResult timerx::pwmInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle)
    {
        _initcb = cb;
        if(_initcb)
        {
            _initcb(true);
        }

        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));

        if (HAL_TIM_PWM_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult timerx::pwmDeInit()
    {
        if(_initcb)
        {
            _initcb(false);
        }
        HAL_TIM_PWM_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    mResult timerx::icInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle)
    {
        _initcb = cb;
        if(_initcb)
        {
            _initcb(true);
        }

        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));

        if (HAL_TIM_IC_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult timerx::icDeInit()
    {
        if(_initcb)
        {
            _initcb(false);
        }
        HAL_TIM_IC_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    mResult timerx::onePulseInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, uint32_t OnePulseMode)
    {
        _initcb = cb;
        if(_initcb)
        {
            _initcb(true);
        }

        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));

        if (HAL_TIM_OnePulse_Init(&_TimHandle, OnePulseMode) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult timerx::onePulseDeInit()
    {
        if(_initcb)
        {
            _initcb(false);
        }
        HAL_TIM_OnePulse_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }

    mResult timerx::encoderInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, TIM_Encoder_InitTypeDef *sConfig)
    {
        _initcb = cb;
        if(_initcb)
        {
            _initcb(true);
        }

        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));

        if (HAL_TIM_Encoder_Init(&_TimHandle, sConfig) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult timerx::encoderDeInit()
    {
        if(_initcb)
        {
            _initcb(false);
        }
        HAL_TIM_Encoder_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    void timerx::start()
    {
        HAL_TIM_Base_Start_IT(&_TimHandle);
    }
    void timerx::stop()
    {
        HAL_TIM_Base_Stop_IT(&_TimHandle);
    }
    void timerx::updateFreq(uint32_t timefreq)
    {
        uint16_t usPeriod;
        uint16_t usPrescaler;
        uint32_t uiTIMxCLK = getInputClk(_TimHandle.Instance);

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
        _TimHandle.Init.Prescaler = usPrescaler;
        _TimHandle.Init.Period = usPeriod;
        __HAL_TIM_SET_PRESCALER(&_TimHandle, usPrescaler);
        __HAL_TIM_SET_AUTORELOAD(&_TimHandle, usPeriod);
        HAL_TIM_GenerateEvent(&_TimHandle, TIM_EVENTSOURCE_UPDATE);
    }
    uint32_t timerx::getTimeOut()
    {
        return _TimHandle.Init.Period*1000000000/(getInputClk(_TimHandle.Instance)/_TimHandle.Init.Prescaler);
    }