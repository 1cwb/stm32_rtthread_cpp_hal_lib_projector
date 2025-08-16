#include "adc.hpp"
mResult adcx::init(const mDev::initCallbackExt& cb, ADC_HandleTypeDef* adcHandle, ADC_ChannelConfTypeDef* sConfig, DMA_HandleTypeDef* dmaHandle)
{
    _initcb = cb;
    memcpy(&_adcHandle, adcHandle, sizeof(ADC_HandleTypeDef));
    //memcpy(&_sConfig, sConfig, sizeof(ADC_ChannelConfTypeDef));

    if(dmaHandle)
    {
        memcpy(&_dmaHandle, dmaHandle, sizeof(DMA_HandleTypeDef));
        HAL_DMA_DeInit(&_dmaHandle);
        _buseRxDma = true;
    }
    if (HAL_ADC_DeInit(&_adcHandle) != HAL_OK)
    {
        printf("%s()%d: HAL_ADC_DeInit() fail\r\n", __func__, __LINE__);
        return M_RESULT_ERROR;
    }
    if (HAL_ADC_Init(&_adcHandle)!= HAL_OK)
    {
        printf("%s()%d: HAL_ADC_Init() fail\r\n", __func__, __LINE__);
        return M_RESULT_ERROR;
    }
    if (HAL_ADCEx_Calibration_Start(&_adcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        printf("%s()%d: HAL_ADCEx_Calibration_Start() fail\r\n", __func__, __LINE__);
        return M_RESULT_ERROR;
    }
    if (HAL_ADC_ConfigChannel(&_adcHandle, sConfig) != HAL_OK)
    {
        printf("%s()%d: HAL_ADC_ConfigChannel() fail\r\n", __func__, __LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult adcx::addChannel(ADC_ChannelConfTypeDef* sConfig)
{
    if (HAL_ADC_ConfigChannel(&_adcHandle, sConfig) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult adcx::dmaInit()
{
    if(_buseRxDma)
    {
        if (HAL_DMA_Init(&_dmaHandle) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
        __HAL_LINKDMA(&_adcHandle,DMA_Handle,_dmaHandle);
    }
    return M_RESULT_EOK;
}
mResult adcx::dmaDeInit()
{
    if(_buseRxDma)
    {
        if (HAL_DMA_DeInit(&_dmaHandle) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult adcx::start(mDev::recvMode mode, uint32_t* value, uint32_t len)
{
    _recvMode = mode;
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
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_DMA)
    {
        _dmaBuffsize = calDmaBuffsize(len);
        if(HAL_ADC_Start_DMA(&_adcHandle,(uint32_t*)value,_dmaBuffsize)!= HAL_OK)
        {
            printf("%s()%d: HAL_ADC_Start_DMA() fail\r\n", __func__, __LINE__);
            return M_RESULT_ERROR;
        }
        if(buseRxDma())
        {
            __HAL_DMA_DISABLE_IT(&_dmaHandle, DMA_IT_HT);
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
    if(adcx::GetObjectFromPrivateMember(hadc)->getAdcHandle() == hadc)
    {
        adcx::GetObjectFromPrivateMember(hadc)->runInitCallback(true);
    }
}
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if(adcx::GetObjectFromPrivateMember(hadc)->getAdcHandle() == hadc)
    {
        adcx::GetObjectFromPrivateMember(hadc)->runInitCallback(false);
    }
}
