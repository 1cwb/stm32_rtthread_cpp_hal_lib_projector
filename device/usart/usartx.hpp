#pragma once
#include "atomic.hpp"
#include "musartdrv.hpp"
#include "sys.h"
class usart : public mDev::mUsart
{
public:
    usart(const char* name):mDev::mUsart(name),_buseTxDma(false),_buseRxDma(false), _transferComplete(true) {
         _rxBuff =(new alignas(32) uint8_t[RX_BUFF_LEN]);
    }
    virtual ~usart() {if(_rxBuff) {delete[] _rxBuff;}}
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
    bool btransferComplete()  {return _transferComplete;}
    void setTransferComplete(bool bcomplete) {_transferComplete = bcomplete;}
    uint8_t* getRxBuff() {return _rxBuff;}
public:
    constexpr static int RX_BUFF_LEN = 64;
    UART_HandleTypeDef _uartHandle;
    DMA_HandleTypeDef _hdmaUsartxTx;
    DMA_HandleTypeDef _hdmaUsartxRx;

private:
    bool _buseTxDma;
    bool _buseRxDma;
    volatile bool _transferComplete;
    uint8_t* _rxBuff;
};
void Debug_printf(const char *format, ...);