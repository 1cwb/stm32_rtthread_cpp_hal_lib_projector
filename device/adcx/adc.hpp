#pragma once
#include "madcdrv.hpp"
#include "sys.h"

class adcx : public mDev::mAdc
{
public:
    adcx() = delete;
    adcx(const char* name):mDev::mAdc(name), _buseRxDma(false),_dmaBuffsize(0),_itcoverCount(0){};
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
    virtual uint8_t getChannelNum()
    {
        return _adcHandle.Init.NbrOfConversion;
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
    virtual ~adcx(){};
    virtual mResult start(mDev::recvMode mode, uint32_t* value, uint32_t len)override;
    virtual mResult stop()override;
    virtual mResult read(uint32_t* value)override;
    bool buseRxDma() const {return _buseRxDma;}
    uint32_t getDmaBuffsize() const {
        return _dmaBuffsize;
    }
    uint32_t getDataPerSize()
    {
        return sizeof(uint16_t);
    }
    uint32_t getItcoverCount() const
    {
        return _itcoverCount;
    }
    void setItcoverCount(uint32_t count)
    {
        _itcoverCount = count;
    }
    inline uint32_t calDmaBuffsize(uint32_t totalBuffsize = RX_BUFF_LEN)
    {
        totalBuffsize = totalBuffsize/2;
        totalBuffsize = totalBuffsize/getChannelNum()*getChannelNum();

        printf("totalBuffsize = %lu\r\n", totalBuffsize);
        return totalBuffsize;
    }
    ADC_HandleTypeDef   _adcHandle;
	ADC_ChannelConfTypeDef   _sConfig;
    DMA_HandleTypeDef   _dmaHandle;

private:
    bool _buseRxDma;
    uint32_t _dmaBuffsize;
    uint32_t _itcoverCount;
};