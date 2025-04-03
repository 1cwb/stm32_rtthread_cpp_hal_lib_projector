#include "spi.hpp"
#include "gpio.hpp"
#include "project.hpp"

static spix* spi5 = nullptr;

int spiInit()
{
    SPI_HandleTypeDef spixHandle;
    spi5 = new spix(DEV_SPI5);
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));
    spixHandle.Instance = SPI5;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_1LINE;
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
    spi5->init([](bool b){
        if(b)
        {
            __HAL_RCC_SPI5_CLK_ENABLE();
            gpiox spi5mosi("spi5sck");
            spi5mosi.init([](bool b){if(b)__HAL_RCC_GPIOK_CLK_ENABLE();},GPIOK, GPIO_PIN_0, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI5);
            gpiox spi5sck("spi5mosi");
            spi5sck.init([](bool b){if(b)__HAL_RCC_GPIOJ_CLK_ENABLE();},GPIOJ, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI5);
            gpiox spi5cs("spi5cs");
            spi5sck.init([](bool b){if(b)__HAL_RCC_GPIOH_CLK_ENABLE();},GPIOH, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI5);
        }
    },&spixHandle);

    return 0;
}
INIT_EXPORT(spiInit, "0.4");