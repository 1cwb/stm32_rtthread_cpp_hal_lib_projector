#pragma once
#include "madcdrv.hpp"
#include "sys.h"

class adcx : public mDev::mAdc
{
public:
    adcx() = delete;
    adcx(const char* name):mDev::mAdc(name), _buseRxDma(false),_dmaBuffsize(0),_itcoverCount(0){};
    mResult init(const mDev::initCallbackExt& cb, ADC_HandleTypeDef* adcHandle, ADC_ChannelConfTypeDef* sConfig, DMA_HandleTypeDef* dmaHandle = nullptr);
    mResult addChannel(ADC_ChannelConfTypeDef* sConfig);
    virtual uint8_t getChannelNum() override
    {
        return _adcHandle.Init.NbrOfConversion;
    }
    mResult dmaInit();
    mResult dmaDeInit();
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

    ADC_HandleTypeDef   _adcHandle;
	ADC_ChannelConfTypeDef   _sConfig;
    DMA_HandleTypeDef   _dmaHandle;
private:
    inline uint32_t calDmaBuffsize(uint32_t totalBuffsize = RX_BUFF_LEN)
    {
        totalBuffsize = totalBuffsize/2;
        totalBuffsize = totalBuffsize/getChannelNum()*getChannelNum();
        return totalBuffsize;
    }
private:
    bool _buseRxDma;
    uint32_t _dmaBuffsize;
    uint32_t _itcoverCount;
};