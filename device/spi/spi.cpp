#include "spi.hpp"
#include "mklog.hpp"
spix::spix(const char* name) : mSpi(name), _buseTxDma(false),_buseRxDma(false)
{
    _rxBuff =(new alignas(32) uint8_t[RX_BUFF_LEN]);
}
spix::~spix()
{
    if(_rxBuff) {delete[] _rxBuff;}
}
mResult spix::init(const mDev::initCallbackExt& cb ,SPI_HandleTypeDef* spihandle, DMA_HandleTypeDef* hdmaTx, DMA_HandleTypeDef* hdmaRx)
{
    _initcb = cb;
    memcpy(&_spixHandle, spihandle, sizeof(SPI_HandleTypeDef));
    if(hdmaTx)
    {
        memcpy(&_hdmaTx, hdmaTx, sizeof(DMA_HandleTypeDef));
        HAL_DMA_DeInit(&_hdmaTx);
        _buseTxDma = true;
    }
    if(hdmaRx)
    {
        memcpy(&_hdmaRx, hdmaRx, sizeof(DMA_HandleTypeDef));
        HAL_DMA_DeInit(&_hdmaRx);
        _buseRxDma = true;
    }
    HAL_SPI_DeInit(&_spixHandle);
    if(HAL_SPI_Init(&_spixHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult spix::deInit()
{
    HAL_SPI_DeInit(&_spixHandle);
    return M_RESULT_EOK;
}
mResult spix::_write(const uint8_t* buff, size_t len)
{
    if(_transferMode == mDev::transferMode::TRANSFER_MODE_NOMAL)
    {
        int ret = 0;
        if((ret = HAL_SPI_Transmit(&_spixHandle, buff, len, 5000)) != HAL_OK)
        {
            KLOGE("Error: %s()%d %d\r\n",__FUNCTION__,__LINE__,ret);
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_IT)
    {
        if(HAL_SPI_Transmit_IT(&_spixHandle, buff, len)!= HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_DMA)
    {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buff, len);

        if(HAL_SPI_Transmit_DMA(&_spixHandle, buff, len)!= HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult spix::_read(uint8_t* buff, size_t len)
{
    if (_recvMode == mDev::recvMode::RECV_MODE_NOMAL)
    {
        if(HAL_SPI_Receive(&_spixHandle, buff, len, 5000) != HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_IT)
    {
        if(HAL_SPI_Receive_IT(&_spixHandle, buff, len)!= HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_DMA)
    {
        if(HAL_SPI_Receive_DMA(&_spixHandle, buff, len)!= HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }

    return M_RESULT_EOK;
}
mResult spix::_transfer(uint8_t* txbuff, uint8_t* rxbuff, size_t len)
{
    if(_transferMode == mDev::transferMode::TRANSFER_MODE_NOMAL)
    {
        if(HAL_SPI_TransmitReceive(&_spixHandle, txbuff, rxbuff, len, 5000) != HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_IT)
    {
        if(HAL_SPI_TransmitReceive_IT(&_spixHandle, txbuff, rxbuff, len)!= HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_DMA)
    {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t*)txbuff, len);
        SCB_CleanInvalidateDCache_by_Addr((uint32_t*)rxbuff, len);
        if(HAL_SPI_TransmitReceive_DMA(&_spixHandle, txbuff, rxbuff, len)!= HAL_OK)
        {
            KLOGE("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
bool spix::btransferComplete()
{
    return (HAL_SPI_GetState(&_spixHandle)==HAL_SPI_STATE_READY);
}
mResult spix::txDmaInit()
{
    if(_buseTxDma)
    {
        if (HAL_DMA_Init(&_hdmaTx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
        __HAL_LINKDMA(&_spixHandle,hdmatx,_hdmaTx);
    }
    return M_RESULT_EOK;
}
mResult spix::txDmaDeInit()
{
    if(_buseTxDma)
    {
        if (HAL_DMA_DeInit(&_hdmaTx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
    }
    return M_RESULT_EOK;
}
mResult spix::rxDmaInit()
{
    if(_buseRxDma)
    {
        if (HAL_DMA_Init(&_hdmaRx) != HAL_OK)
        {
            return M_RESULT_ERROR; 
        }
        __HAL_LINKDMA(&_spixHandle,hdmarx,_hdmaRx);
    }
    return M_RESULT_EOK;
}
mResult spix::rxDmaDeInit()
{
    if(_buseRxDma)
    {
        if (HAL_DMA_DeInit(&_hdmaRx) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    spix* spi = spix::GetObjectFromPrivateMember(hspi);
    if(hspi == spi->spixHandle())
    {
        spi->runInitCallback(true);
    }
}
extern "C" void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    spix* spi = spix::GetObjectFromPrivateMember(hspi);
    if(hspi == spi->spixHandle())
    {
        spi->runInitCallback(false);
    }
}