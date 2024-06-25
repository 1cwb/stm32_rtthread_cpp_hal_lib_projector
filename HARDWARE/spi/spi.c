#include "spi.h"
static SPI_HandleTypeDef SPI4Handler;
void spi4Init()
{
  SPI4Handler.Instance = SPI4;
  SPI4Handler.Init.Mode = SPI_MODE_MASTER;
  SPI4Handler.Init.Direction = SPI_DIRECTION_2LINES;
  SPI4Handler.Init.DataSize = SPI_DATASIZE_8BIT;
  SPI4Handler.Init.CLKPolarity = SPI_POLARITY_LOW;
  SPI4Handler.Init.CLKPhase = SPI_PHASE_1EDGE;
  SPI4Handler.Init.NSS = SPI_NSS_SOFT;
  SPI4Handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  SPI4Handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
  SPI4Handler.Init.TIMode = SPI_TIMODE_DISABLE;
  SPI4Handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI4Handler.Init.CRCPolynomial = 0x0;
  SPI4Handler.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  SPI4Handler.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  SPI4Handler.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  SPI4Handler.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  SPI4Handler.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  SPI4Handler.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
  SPI4Handler.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
  SPI4Handler.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  SPI4Handler.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  SPI4Handler.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  HAL_SPI_Init(&SPI4Handler);
}

static SPI_HandleTypeDef SPI1Handler;
void spi1Init()
{
  SPI1Handler.Instance = SPI1;
  SPI1Handler.Init.Mode = SPI_MODE_MASTER;
  SPI1Handler.Init.Direction = SPI_DIRECTION_2LINES;
  SPI1Handler.Init.DataSize = SPI_DATASIZE_8BIT;
  SPI1Handler.Init.CLKPolarity = SPI_POLARITY_LOW;
  SPI1Handler.Init.CLKPhase = SPI_PHASE_1EDGE;
  SPI1Handler.Init.NSS = SPI_NSS_SOFT;
  SPI1Handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  SPI1Handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
  SPI1Handler.Init.TIMode = SPI_TIMODE_DISABLE;
  SPI1Handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI1Handler.Init.CRCPolynomial = 0x0;
  SPI1Handler.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  SPI1Handler.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  SPI1Handler.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  SPI1Handler.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  SPI1Handler.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  SPI1Handler.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
  SPI1Handler.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
  SPI1Handler.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  SPI1Handler.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  SPI1Handler.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  HAL_SPI_Init(&SPI1Handler) != HAL_OK;
}
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI4)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    __HAL_RCC_SPI4_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PE14     ------> SPI4_MOSI
    PE13     ------> SPI4_MISO
    PE12     ------> SPI4_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = GPIO_PIN_11; //CS
    GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP; //复用推挽输出
    GPIO_InitStruct.Pull=GPIO_PULLUP; //上拉
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH; //高速
    HAL_GPIO_Init(GPIOE,&GPIO_InitStruct); //初始化
  }
  if(spiHandle->Instance==SPI1)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI1CLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /*
    PD07     ------> SPI1_MOSI
    PA06     ------> SPI1_MISO
    PA05     ------> SPI1_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;//sck
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = GPIO_PIN_15; //CS
    GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP; //复用推挽输出
    GPIO_InitStruct.Pull=GPIO_PULLUP; //上拉
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH; //高速
    HAL_GPIO_Init(GPIOC,&GPIO_InitStruct); //初始化
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
  if(spiHandle->Instance==SPI4)
  {
    __HAL_RCC_SPI4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
  }
  if(spiHandle->Instance==SPI1)
  {
    __HAL_RCC_SPI1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_15);
  }
}

SPI_HandleTypeDef* getSpi4Handler()
{
  return &SPI4Handler;
}

SPI_HandleTypeDef* getSpi1Handler()
{
  return &SPI1Handler;
}
