#include "adc.hpp"
mResult adcx::start(mDev::recvMode mode, uint32_t* value, uint32_t len)
{
    _recvMode = mode;
    printf("%s()%d: %d\r\n", __func__, __LINE__, mode);
    if (_recvMode == mDev::recvMode::RECV_MODE_NOMAL)
    {
        if(HAL_ADC_Start(&_adcHandle) != HAL_OK)
        {
            printf("%s()%d: HAL_ADC_Start() fail\r\n", __func__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_IT)
    {
        if(HAL_ADC_Start_IT(&_adcHandle)!= HAL_OK)
        {
            printf("%s()%d: HAL_ADC_Start_IT() fail\r\n", __func__, __LINE__);
            return M_RESULT_ERROR;
        }
        printf("%s()%d: HAL_ADC_Start_IT() end\r\n", __func__, __LINE__);
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_DMA)
    {
        if(HAL_ADC_Start_DMA(&_adcHandle,(uint32_t*)value,len)!= HAL_OK)
        {
            printf("%s()%d: HAL_ADC_Start_DMA() fail\r\n", __func__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult adcx::stop()
{
    if(_recvMode == mDev::recvMode::RECV_MODE_NOMAL)
    {
        if(HAL_ADC_Stop(&_adcHandle)!= HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_IT)
    {
        if(HAL_ADC_Stop_IT(&_adcHandle)!= HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_DMA)
    {
        if(HAL_ADC_Stop_DMA(&_adcHandle)!= HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult adcx::read(uint32_t* value)
{
    if(_recvMode == mDev::recvMode::RECV_MODE_NOMAL)
    {
        if(HAL_ADC_PollForConversion(&_adcHandle,100)!= HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        *value = HAL_ADC_GetValue(&_adcHandle);
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_IT)
    {
        *value = HAL_ADC_GetValue(&_adcHandle);
        return M_RESULT_ERROR;
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_DMA)
    {
        *value = 0;
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    adcx* adc = containerof(hadc, adcx, _adcHandle);
    if(hadc == adc->adcHandle())
    {
        adc->runInitCallback(true);
    }
}
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    adcx* adc = containerof(hadc, adcx, _adcHandle);
    if(hadc == adc->adcHandle())
    {
        adc->runInitCallback(false);
    }
}