#include "usartx.hpp"
#include "mgpiodrv.hpp"
#include "mplatform.hpp"
mResult usart::init(const mDev::initCallbackExt& cb ,UART_HandleTypeDef* uartHandle, DMA_HandleTypeDef* hdmaUsartxTx, DMA_HandleTypeDef* hdmaUsartxRx)
{
    _initcb = cb;
    memcpy(&_uartHandle, uartHandle, sizeof(UART_HandleTypeDef));
    _uartHandle.gState = HAL_UART_STATE_RESET;

    if(hdmaUsartxTx)
    {
        memcpy(&_hdmaUsartxTx, hdmaUsartxTx, sizeof(DMA_HandleTypeDef));
        _hdmaUsartxTx.State = HAL_DMA_STATE_RESET;
        _buseTxDma = true;
    }
    if(hdmaUsartxRx)
    {
        memcpy(&_hdmaUsartxRx, hdmaUsartxRx, sizeof(DMA_HandleTypeDef));
        _hdmaUsartxRx.State = HAL_DMA_STATE_RESET;
        _buseRxDma = true;
    }
    if (HAL_UART_Init(&_uartHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&_uartHandle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&_uartHandle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    if (HAL_UARTEx_DisableFifoMode(&_uartHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }

    return M_RESULT_EOK;
}
mResult usart::deInit()
{
    if (HAL_UART_DeInit(&_uartHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult usart::txDmaInit()
{
    if(_buseTxDma)
    {
        if (HAL_DMA_Init(&_hdmaUsartxTx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
        __HAL_LINKDMA(&_uartHandle,hdmatx,_hdmaUsartxTx);
    }
    return M_RESULT_EOK;
}
mResult usart::txDmaDeInit()
{
    if(_buseTxDma)
    {
        if (HAL_DMA_DeInit(&_hdmaUsartxTx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
    }
    return M_RESULT_EOK;
}
mResult usart::rxDmaInit()
{
    if(_buseRxDma)
    {
        if (HAL_DMA_Init(&_hdmaUsartxRx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
        __HAL_LINKDMA(&_uartHandle,hdmarx,_hdmaUsartxRx);
    }
    return M_RESULT_EOK;
}
mResult usart::rxDmaDeInit()
{
    if(_buseRxDma)
    {
        if (HAL_DMA_DeInit(&_hdmaUsartxRx) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult usart::send(const uint8_t* data, uint32_t len)
{
    if(_transferMode == mDev::transferMode::TRANSFER_MODE_NOMAL)
    {
        if(HAL_UART_Transmit(&_uartHandle, data, len, 0xFFFF) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_IT)
    {
        if(HAL_UART_Transmit_IT(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_DMA)
    {
        if(_transferComplete)
        {
            _transferComplete = false;
        }
        else
        {
            return M_RESULT_EBUSY;
        }
        SCB_CleanDCache_by_Addr(const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(data)), M_ALIGN(len,4)/4);
        if(HAL_UART_Transmit_DMA(&_uartHandle, data, len) != HAL_OK)
        {
            _transferComplete = true;
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult usart::recv(uint8_t* data, uint32_t len)
{
    if(_recvMode == mDev::recvMode::RECV_MODE_NOMAL)
    {
        if(HAL_UART_Receive(&_uartHandle, data, len, 0xFFFF) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_recvMode == mDev::recvMode::RECV_MODE_IT)
    {
        if(HAL_UART_Receive_IT(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_recvMode == mDev::recvMode::RECV_MODE_DMA)
    {
        SCB_CleanDCache_by_Addr(const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(data)), M_ALIGN(len,4)/4);
        if(HAL_UART_Receive_DMA(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        __HAL_DMA_DISABLE_IT(&_hdmaUsartxRx, DMA_IT_HT);
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_IT_RECV_IDLE)
    {
        if(HAL_UARTEx_ReceiveToIdle_IT(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_DMA_RECV_IDLE)
    {
        SCB_CleanDCache_by_Addr(const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(data)), M_ALIGN(len,4)/4);
        if(HAL_UARTEx_ReceiveToIdle_DMA(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
        __HAL_DMA_DISABLE_IT(&_hdmaUsartxRx, DMA_IT_HT);
    }
    return M_RESULT_EOK;
}
void usart::syncDataByAddr(uint32_t *addr, int32_t dsize)
{
    SCB_CleanDCache_by_Addr(addr, dsize);
}
extern "C" void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    usart* usartx = containerof(huart, usart, _uartHandle);
    if(huart == usartx->usartHandle())
    {
        usartx->runInitCallback(true);
    }
}
extern "C" void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    usart* usartx = containerof(huart, usart, _uartHandle);
    if(huart == usartx->usartHandle())
    {
        usartx->runInitCallback(false);
    }
}
