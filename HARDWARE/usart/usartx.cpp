#include "usartx.hpp"
#include "mgpiodrv.hpp"
#include "mplatform.hpp"
mResult usart::init(const mDev::initCallbackExt& cb ,UART_HandleTypeDef* uartHandle, DMA_HandleTypeDef* hdmaUsartxTx, DMA_HandleTypeDef* hdmaUsartxRx)
{
    _initcb = cb;
    memcpy(&_uartHandle, uartHandle, sizeof(UART_HandleTypeDef));
    _uartHandle.gState = HAL_UART_STATE_RESET;
          /* DMA controller clock enable */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* DMA interrupt init */
        /* DMA1_Stream0_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
        /* DMA1_Stream1_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    if(hdmaUsartxTx)
    {
        memcpy(&_hdmaUsartxTx, hdmaUsartxTx, sizeof(DMA_HandleTypeDef));
        _hdmaUsartxTx.State = HAL_DMA_STATE_RESET;
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
        _hdmaUsartxRx.State = HAL_DMA_STATE_RESET;
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
mResult usart::send(const uint8_t* data, uint32_t len)
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
void usart::syncDataByAddr(uint32_t *addr, int32_t dsize)
{
    SCB_CleanDCache_by_Addr(addr, dsize);
}
#if 1
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
#endif