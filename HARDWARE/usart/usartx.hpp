#pragma once
#include "musartdrv.hpp"
#include "sys.h"
class usart : public mDev::mUsart
{
public:
    usart(const char* name):mDev::mUsart(name),_buseTxDma(false),_buseRxDma(false), _transferComplete(true) {}
    virtual ~usart() {}
    mResult init(const mDev::initCallbackExt& cb ,UART_HandleTypeDef* uartHandle,DMA_HandleTypeDef* hdmaUsartxTx, DMA_HandleTypeDef* hdmaUsartxRx);
    mResult deInit();
    UART_HandleTypeDef* usartHandle() {return &_uartHandle;}
    DMA_HandleTypeDef* dmaTxHandle() {return &_hdmaUsartxTx;}
    DMA_HandleTypeDef* dmaRxHandle() {return &_hdmaUsartxRx;}
    virtual mResult send(uint8_t* data, uint32_t len);
    virtual mResult recv(uint8_t* data, uint32_t len);
    virtual void* getObj() {return this;}
    bool buseTxDma()const {return _buseTxDma;}
    bool buseRxDma() const {return _buseRxDma;}
    bool btransferComplete() const {return _transferComplete;}
    void setTransferComplete(bool bcomplete) {_transferComplete = bcomplete;}
    UART_HandleTypeDef _uartHandle;
    DMA_HandleTypeDef _hdmaUsartxTx;
    DMA_HandleTypeDef _hdmaUsartxRx;
private:
    bool _buseTxDma;
    bool _buseRxDma;
    bool _transferComplete;
};
void Debug_printf(const char *format, ...);