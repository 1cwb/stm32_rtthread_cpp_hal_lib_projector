#pragma once
#include "madcdrv.hpp"
#include "sys.h"

class adcx : public mDev::mAdc
{
public:
    adcx() = delete;
    adcx(const char* name):mDev::mAdc(name), _buseRxDma(false){_rxBuff =(new alignas(32) uint8_t[RX_BUFF_LEN]);};
    mResult init(const mDev::initCallbackExt& cb, ADC_HandleTypeDef* adcHandle, ADC_ChannelConfTypeDef* sConfig, DMA_HandleTypeDef* dmaHandle = nullptr)
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
    mResult addChannel(ADC_ChannelConfTypeDef* sConfig)
    {
        if (HAL_ADC_ConfigChannel(&_adcHandle, sConfig) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult dmaInit()
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
    mResult dmaDeInit()
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
    ADC_HandleTypeDef* adcHandle() {return &_adcHandle;}
    DMA_HandleTypeDef* dmaHandle() {return &_dmaHandle;}
    virtual ~adcx(){if(_rxBuff) {delete[] _rxBuff;}};
    virtual mResult start(mDev::recvMode mode, uint32_t* value, uint32_t len)override;
    virtual mResult stop()override;
    virtual mResult read(uint32_t* value)override;
    uint16_t* getRxBuff() {return reinterpret_cast<uint16_t*>(_rxBuff);}
    bool buseRxDma() const {return _buseRxDma;}
    ADC_HandleTypeDef   _adcHandle;
	ADC_ChannelConfTypeDef   _sConfig;
    DMA_HandleTypeDef   _dmaHandle;

public:
    constexpr static int RX_BUFF_LEN = 64;
private:
    bool _buseRxDma;
    uint8_t* _rxBuff;
};