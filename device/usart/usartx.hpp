#pragma once
#include "musartdrv.hpp"
#include "sys.h"
class usart : public mDev::mUsart
{
public:
    usart(const char* name):mDev::mUsart(name),_buseTxDma(false),_buseRxDma(false) {
    }
    virtual ~usart() {}
    mResult duplicateHal(const mDev::initCallbackExt& cb, UART_HandleTypeDef* uartHandle, DMA_HandleTypeDef* hdmaUsartxTx = nullptr, DMA_HandleTypeDef* hdmaUsartxRx = nullptr);
    mResult init(const mDev::initCallbackExt& cb ,UART_HandleTypeDef* uartHandle,DMA_HandleTypeDef* hdmaUsartxTx = nullptr, DMA_HandleTypeDef* hdmaUsartxRx = nullptr);
    mResult deInit();
    mResult txDmaInit();
    mResult txDmaDeInit();
    mResult rxDmaInit();
    mResult rxDmaDeInit();
    UART_HandleTypeDef* usartHandle() {return &_uartHandle;}
    DMA_HandleTypeDef* dmaTxHandle() {return &_hdmaUsartxTx;}
    DMA_HandleTypeDef* dmaRxHandle() {return &_hdmaUsartxRx;}

    virtual mResult send(const uint8_t* data, uint32_t len) override;
    virtual mResult recv(uint8_t* data, uint32_t len) override;
    bool buseTxDma()const {return _buseTxDma;}
    bool buseRxDma() const {return _buseRxDma;}
public:
    UART_HandleTypeDef _uartHandle;
    DMA_HandleTypeDef _hdmaUsartxTx;
    DMA_HandleTypeDef _hdmaUsartxRx;

private:
    bool _buseTxDma;
    bool _buseRxDma;
};
void Debug_printf(const char *format, ...);