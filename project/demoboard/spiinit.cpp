#include "spi.hpp"
#include "gpio.hpp"
#include "project.hpp"

static spix* spi5 = nullptr;

extern "C" void DMA1_Stream2_IRQHandler(void)
{
  if(spi5)
  {
    HAL_DMA_IRQHandler(spi5->dmaTxHandle());
  }
}

extern "C" void SPI5_IRQHandler(void)
{
    if(spi5)
    {
        HAL_SPI_IRQHandler(spi5->spixHandle());
    }
}
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    if(hspi == pspix->spixHandle())
    {
        pspix->runInterruptCb(nullptr);
        //printf("%s transfer cmp\r\n",pspix->getDeviceName());
    }
}
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    if(hspi == pspix->spixHandle())
    {
        if(pspix->buseRxDma())
        {
            SCB_InvalidateDCache_by_Addr((uint32_t*)pspix->getRxBuff(), pspix->RX_BUFF_LEN);
        }
        pspix->read(pspix->getRxBuff(), pspix->RX_BUFF_LEN);
    }
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    if(hspi == pspix->spixHandle())
    {
        //printf("%s transfer-rx cmp\r\n",pspix->getDeviceName());
    }
}
extern "C" void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    if(hspi == pspix->spixHandle())
    {
        //printf("%s transfer half\r\n",pspix->getDeviceName());
    }
}
extern "C" void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    if(hspi == pspix->spixHandle())
    {
    }
}
extern "C" void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    if(hspi == pspix->spixHandle())
    {
      
    }
}
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    __HAL_UNLOCK(hspi);
    if(hspi == pspix->spixHandle())
    {
      printf("error happend %s ErrorCode = %ld\r\n",pspix->getDeviceName(),hspi->ErrorCode);
    }
}
extern "C" void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    __HAL_UNLOCK(hspi);
    if(hspi == pspix->spixHandle())
    {
        printf("error happend %s abort\r\n",pspix->getDeviceName());
    }
}
extern "C" void HAL_SPI_SuspendCallback(SPI_HandleTypeDef *hspi)
{
    spix* pspix = containerof(hspi, spix, _spixHandle);
    __HAL_UNLOCK(hspi);
    if(hspi == pspix->spixHandle())
    {
      printf("error happend %s suspend\r\n",pspix->getDeviceName());
    }
}

int spiInit()
{
    SPI_HandleTypeDef spixHandle;
    DMA_HandleTypeDef hdmatx;
    spi5 = new spix(DEV_SPI5);
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));
    spixHandle.Instance = SPI5;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spixHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_02DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    //DMA
    hdmatx.Instance = DMA1_Stream2;
    hdmatx.Init.Request = DMA_REQUEST_SPI5_TX;
    hdmatx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdmatx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdmatx.Init.MemInc = DMA_MINC_ENABLE;
    hdmatx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmatx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdmatx.Init.Mode = DMA_NORMAL;
    hdmatx.Init.Priority = DMA_PRIORITY_LOW;
    hdmatx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdmatx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdmatx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdmatx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    spi5->init([&](bool b){
        if(b)
        {
            __HAL_RCC_DMA1_CLK_ENABLE();

            /* DMA interrupt init */
            /* DMA1_Stream0_IRQn interrupt configuration */
            HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 4, 0);
            HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

            __HAL_RCC_SPI5_CLK_ENABLE();
            gpiox spi5mosi("spi5sck");
            spi5mosi.init([](bool b){if(b)__HAL_RCC_GPIOK_CLK_ENABLE();},GPIOK, GPIO_PIN_0, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI5);
            gpiox spi5sck("spi5mosi");
            spi5sck.init([](bool b){if(b)__HAL_RCC_GPIOJ_CLK_ENABLE();},GPIOJ, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI5);
            gpiox spi5cs("spi5cs");
            spi5sck.init([](bool b){if(b)__HAL_RCC_GPIOH_CLK_ENABLE();},GPIOH, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI5);
            spi5->txDmaInit();
            HAL_NVIC_SetPriority(SPI5_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(SPI5_IRQn);
        }
    },&spixHandle,&hdmatx);
    spi5->setTransferMode(mDev::transferMode::TRANSFER_MODE_NOMAL);
    return 0;
}
INIT_EXPORT(spiInit, "0.4");