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
    mResult timerx::baseTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, mDev::TIMESTARTMODE startMode)
    {
        _initcb = cb;
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));
        _TimHandle.State = HAL_TIM_STATE_RESET;
        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        if (HAL_TIM_Base_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        _btimerInited = true;
        _mode = mDev::TIMEMODE::TIMEMODE_BASE_TIM;
        _startMode = startMode;
        return M_RESULT_EOK;
    }
    mResult timerx::baseTimeDeInit()
    {
        HAL_TIM_Base_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    mResult timerx::pwmTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, mDev::TIMESTARTMODE startMode)
    {
        _initcb = cb;
        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));
        _TimHandle.State = HAL_TIM_STATE_RESET;
        if (HAL_TIM_PWM_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        _btimerInited = true;
        _mode = mDev::TIMEMODE::TIMEMODE_PWM;
        _startMode = startMode;
        return M_RESULT_EOK;
    }
    mResult timerx::pwmTimeDeInit()
    {
        HAL_TIM_PWM_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    mResult timerx::pwmConfig(TIM_OC_InitTypeDef* TIM_OCInitStructure, uint32_t channel, TIM_BreakDeadTimeConfigTypeDef* TIM_BDTRInitStructure, bool enableNch)
    {
        if(!_btimerInited)
        {
            return M_RESULT_ERROR;
        }
        if(TIM_BDTRInitStructure)
        {
            _bUseBreakDeadTime = true;
        }
        _bEnableNch = enableNch;
        _channelNum |= remapTimChannelToChannel(channel);
        
        if(HAL_TIM_PWM_ConfigChannel(&_TimHandle,TIM_OCInitStructure,channel) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        if(_bUseBreakDeadTime)
        {
            if(HAL_TIMEx_ConfigBreakDeadTime(&_TimHandle, TIM_BDTRInitStructure) != HAL_OK)
            {
                return M_RESULT_ERROR;
            }
        }
        return M_RESULT_EOK;
    }
    mResult timerx::icTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, mDev::TIMESTARTMODE startMode)
    {
        _initcb = cb;
        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));
        _TimHandle.State = HAL_TIM_STATE_RESET;
        if (HAL_TIM_IC_Init(&_TimHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        _btimerInited = true;
        _mode = mDev::TIMEMODE::TIMEMODE_IC;
        _startMode = startMode;
        return M_RESULT_EOK;
    }
    mResult timerx::icTimeDeInit()
    {
        HAL_TIM_IC_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    mResult timerx::icConfig(TIM_IC_InitTypeDef *sConfig, uint32_t channel)
    {
        if(!_btimerInited)
        {
            return M_RESULT_ERROR;
        }
        _channelNum |= remapTimChannelToChannel(channel);
        if(HAL_TIM_IC_ConfigChannel(&_TimHandle, sConfig, channel) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult timerx::encoderTimeInit(const mDev::initCallbackExt& cb, TIM_HandleTypeDef* TimHandle, TIM_Encoder_InitTypeDef *sConfig, mDev::TIMESTARTMODE startMode)
    {
        _initcb = cb;
        _timeFreq = getInputClk(TimHandle->Instance)/(TimHandle->Init.Prescaler+1)/(TimHandle->Init.Period+1);
        memcpy(&_TimHandle, TimHandle, sizeof(_TimHandle));
        _TimHandle.State = HAL_TIM_STATE_RESET;
        if (HAL_TIM_Encoder_Init(&_TimHandle, sConfig) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        _btimerInited = true;
        _mode = mDev::TIMEMODE::TIMEMODE_ENCODER;
        _startMode = startMode;
        return M_RESULT_EOK;
    }
    mResult timerx::encoderTimeDeInit()
    {
        HAL_TIM_Encoder_DeInit(&_TimHandle);
        _btimerInited = false;
        return M_RESULT_EOK;
    }
    void timerx::start(mDev::CHANNLE channel, uint32_t* data1, uint32_t* data2, size_t len)
    {
        switch(_mode)
        {
            case mDev::TIMEMODE::TIMEMODE_BASE_TIM:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_Base_Start(&_TimHandle);
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_Base_Start_IT(&_TimHandle);
                }
                else
                {
                    HAL_TIM_Base_Start_DMA(&_TimHandle,data1,len);
                }
                break;
            case mDev::TIMEMODE::TIMEMODE_PWM:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_PWM_Start(&_TimHandle,remapChannelToTimChannel(channel));
                    if(_bEnableNch)
                    {
                        HAL_TIMEx_PWMN_Start(&_TimHandle,remapChannelToTimChannel(channel));
                    }
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_PWM_Start_IT(&_TimHandle,remapChannelToTimChannel(channel));
                    if(_bEnableNch)
                    {
                        HAL_TIMEx_PWMN_Start_IT(&_TimHandle,remapChannelToTimChannel(channel));
                    }
                }
                else
                {
                    HAL_TIM_PWM_Start_DMA(&_TimHandle,remapChannelToTimChannel(channel),data1,len);
                    if(_bEnableNch)
                    {
                        HAL_TIMEx_PWMN_Start_DMA(&_TimHandle,remapChannelToTimChannel(channel),data2,len);
                    }
                }
                break;
            case mDev::TIMEMODE::TIMEMODE_IC:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_IC_Start(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_IC_Start_IT(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else
                {
                    HAL_TIM_IC_Start_DMA(&_TimHandle,remapChannelToTimChannel(channel),data1,len);
                }
                break;
            case mDev::TIMEMODE::TIMEMODE_ENCODER:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_Encoder_Start(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_Encoder_Start_IT(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else
                {
                    HAL_TIM_Encoder_Start_DMA(&_TimHandle,remapChannelToTimChannel(channel),data1,data2,len);
                }
                break;
            default:
                break;
        }
    }
    void timerx::stop(mDev::CHANNLE channel)
    {
        switch(_mode)
        {
            case mDev::TIMEMODE::TIMEMODE_BASE_TIM:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_Base_Stop(&_TimHandle);
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_Base_Stop_IT(&_TimHandle);
                }
                else
                {
                    HAL_TIM_Base_Stop_DMA(&_TimHandle);
                }
                break;
            case mDev::TIMEMODE::TIMEMODE_PWM:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_PWM_Stop(&_TimHandle,remapChannelToTimChannel(channel));
                    if(_bEnableNch)
                    {
                        HAL_TIMEx_PWMN_Stop(&_TimHandle,remapChannelToTimChannel(channel));
                    }
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_PWM_Stop_IT(&_TimHandle,remapChannelToTimChannel(channel));
                    if(_bEnableNch)
                    {
                        HAL_TIMEx_PWMN_Stop_IT(&_TimHandle,remapChannelToTimChannel(channel));
                    }
                }
                else
                {
                    HAL_TIM_PWM_Stop_DMA(&_TimHandle,remapChannelToTimChannel(channel));
                    if(_bEnableNch)
                    {
                        HAL_TIMEx_PWMN_Stop_DMA(&_TimHandle,remapChannelToTimChannel(channel));
                    }
                }
                break;
            case mDev::TIMEMODE::TIMEMODE_IC:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_IC_Stop(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_IC_Stop_IT(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else
                {
                    HAL_TIM_IC_Stop_DMA(&_TimHandle,remapChannelToTimChannel(channel));
                }
                break;
            case mDev::TIMEMODE::TIMEMODE_ENCODER:
                if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_NOMAL)
                {
                    HAL_TIM_Encoder_Stop(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else if(_startMode == mDev::TIMESTARTMODE::TIMESTARTMODE_IT)
                {
                    HAL_TIM_Encoder_Stop_IT(&_TimHandle,remapChannelToTimChannel(channel));
                }
                else
                {
                    HAL_TIM_Encoder_Stop_DMA(&_TimHandle,remapChannelToTimChannel(channel));
                }
                break;
            default:
                break;
        }
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
    uint32_t timerx::getTimeOutUs()
    {
        return (uint32_t)(((float)((_TimHandle.Init.Period+1)*1000.0f))/((float)(getInputClk(_TimHandle.Instance)/(_TimHandle.Init.Prescaler+1))) * 1000U);
    }

extern "C" 
{
void runCallBack(TIM_HandleTypeDef *htim, bool binit)
{
    timerx* tim = containerof(htim, timerx, _TimHandle);
    if(htim == tim->getTimHandle())
    {
        tim->runInitCallback(binit);
    }
}
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, true);
}
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, false);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, true);
}
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, false);
}
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, true);
}
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, false);
}
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, true);
}
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, false);
}
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, true);
}
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim)
{
    runCallBack(htim, false);
}
}