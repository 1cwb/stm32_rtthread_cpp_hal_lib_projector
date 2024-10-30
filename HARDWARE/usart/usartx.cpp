#include "usartx.hpp"
#if 0
mResult usart::init(const mDev::initCallbackExt& cb ,UART_HandleTypeDef* uartHandle, DMA_HandleTypeDef* hdmaUsartxTx, DMA_HandleTypeDef* hdmaUsartxRx)
{
    _initcb = cb;
    memcpy(&_uartHandle, uartHandle, sizeof(UART_HandleTypeDef));
    if(hdmaUsartxTx)
    {
        memcpy(&_hdmaUsartxTx, hdmaUsartxTx, sizeof(DMA_HandleTypeDef));
        /* USART1_TX Init */
        if (HAL_DMA_Init(&_hdmaUsartxTx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
        __HAL_LINKDMA(&_uartHandle,hdmatx,_hdmaUsartxTx);
        _buseTxDma = true;
    }
    if(hdmaUsartxRx)
    {
        memcpy(&_hdmaUsartxRx, hdmaUsartxRx, sizeof(DMA_HandleTypeDef));
        /* USART1_RX Init */
        if (HAL_DMA_Init(&_hdmaUsartxRx) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }

        __HAL_LINKDMA(&_uartHandle,hdmarx,_hdmaUsartxRx);
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
    if(buseTxDma())
    {
        if (HAL_DMA_DeInit(&_hdmaUsartxTx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
    }
    if(buseRxDma())
    {
        if (HAL_DMA_DeInit(&_hdmaUsartxRx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
    }
    return M_RESULT_EOK;
}
mResult usart::send(uint8_t* data, uint32_t len)
{
    if(_mode == mDev::transferMode::TRANSFER_MODE_NOMAL)
    {
        if(HAL_UART_Transmit(&_uartHandle, data, len, 0xFFFF) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_mode == mDev::transferMode::TRANSFER_MODE_IT)
    {
        if(HAL_UART_Transmit_IT(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_mode == mDev::transferMode::TRANSFER_MODE_DMA)
    {
        if(HAL_UART_Transmit_DMA(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult usart::recv(uint8_t* data, uint32_t len)
{
    if(_mode == mDev::transferMode::TRANSFER_MODE_NOMAL)
    {
        if(HAL_UART_Receive(&_uartHandle, data, len, 0xFFFF) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_mode == mDev::transferMode::TRANSFER_MODE_IT)
    {
        if(HAL_UART_Receive_IT(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    else if(_mode == mDev::transferMode::TRANSFER_MODE_DMA)
    {
        if(HAL_UART_Receive_DMA(&_uartHandle, data, len) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    usart* usartx = containerof(uartHandle, usart, _uartHandle);
    if(uartHandle == usartx->usartHandle())
    {
        usartx->runInitCallback(true);
    }
}
extern "C" void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
    usart* usartx = containerof(uartHandle, usart, _uartHandle);
    if(uartHandle == usartx->usartHandle())
    {
        usartx->runInitCallback(false);
    }
}
#endif