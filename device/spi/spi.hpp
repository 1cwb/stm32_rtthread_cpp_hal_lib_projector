#pragma once
#include "mspidrv.hpp"
#include "sys.h"
#include "gpio.hpp"
class spix : public mDev::mSpi
{
public:
    spix() = delete;
    explicit spix(const char* name);
    virtual ~spix();
    mResult init(const mDev::initCallbackExt& cb ,SPI_HandleTypeDef* spihandle, DMA_HandleTypeDef* hdmaUsartxTx = nullptr, DMA_HandleTypeDef* hdmaUsartxRx = nullptr);
    mResult deInit();
    inline virtual void csEnable(mDev::mGpio* cspin)override;
    inline virtual void csDisable(mDev::mGpio* cspin)override;
    virtual mResult write(const uint8_t* buff, size_t len)override;
    virtual mResult read(uint8_t* buff, size_t len)override;
    virtual bool btransferComplete()override;
    mResult writeRead(uint8_t* txbuff, uint8_t* rxbuff, size_t len);
    mResult txDmaInit();
    mResult txDmaDeInit();
    mResult rxDmaInit();
    mResult rxDmaDeInit();
    bool buseTxDma()const {return _buseTxDma;}
    bool buseRxDma() const {return _buseRxDma;}
    uint8_t* getRxBuff() {return _rxBuff;}
    SPI_HandleTypeDef* spixHandle() {return &_spixHandle;}
    DMA_HandleTypeDef* dmaTxHandle() {return &_hdmaTx;}
    DMA_HandleTypeDef* dmaRxHandle() {return &_hdmaRx;}
public:
    constexpr static int RX_BUFF_LEN = 64;
    SPI_HandleTypeDef _spixHandle;
    DMA_HandleTypeDef _hdmaTx;
    DMA_HandleTypeDef _hdmaRx;
private:
    bool _buseTxDma;
    bool _buseRxDma;
    uint8_t* _rxBuff;
};
