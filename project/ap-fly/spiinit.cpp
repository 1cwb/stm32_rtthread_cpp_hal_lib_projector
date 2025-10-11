#include "spi.hpp"
#include "gpio.hpp"
#include "project.hpp"

static spix* spi1 = nullptr;
static spix* spi2 = nullptr;
static spix* spi4 = nullptr;

int spiInit()
{
    SPI_HandleTypeDef spixHandle;
    spi1 = new spix(DEV_SPI1);
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));
    spixHandle.Instance = SPI1;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spixHandle.Init.NSS = SPI_NSS_SOFT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi1->init([](bool b){
        if(b)
        {
            __HAL_RCC_SPI1_CLK_ENABLE();
            gpiox spi1mosi("spi1sck");
            spi1mosi.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
            gpiox spi1miso("spi1miso");
            spi1miso.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
            gpiox spi1sck("spi1mosi");
            spi1sck.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
        }
    },&spixHandle);


    //SPI2 init
    // SPI2配置
    spi2 = new spix(DEV_SPI2);
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));
    spixHandle.Instance = SPI2;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;        // CPOL = 0 (SPI Mode 0)
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;            // CPHA = 0 (SPI Mode 0)
    spixHandle.Init.NSS = SPI_NSS_SOFT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  // 注意：根据实际时钟调整
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi2->init([](bool b){
        if(b)
        {
            __HAL_RCC_SPI2_CLK_ENABLE();
            gpiox spi2mosi("spi2mosi");
            spi2mosi.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();},GPIOC, GPIO_PIN_3, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI2);
            gpiox spi2miso("spi2miso");
            spi2miso.init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI2);
            gpiox spi2sck("spi2sck");
            spi2sck.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_3, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI2);
        }
    },&spixHandle);

    //SPI4 init
    spi4 = new spix(DEV_SPI4);
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));

    spixHandle.Instance = SPI4;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spixHandle.Init.NSS = SPI_NSS_SOFT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi4->init([](bool b){
        if(b)
        {
            RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
            PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
            PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
            HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
            __HAL_RCC_SPI4_CLK_ENABLE();
            __HAL_RCC_GPIOE_CLK_ENABLE();

            gpiox spi1mosi("spi4mosi");
            spi1mosi.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
            gpiox spi1miso("spi4miso");
            spi1miso.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
            gpiox spi1sck("spi4sck");
            spi1sck.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
        }
    },&spixHandle);
    return 0;
}
INIT_EXPORT(spiInit, "0.4");