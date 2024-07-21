#include "spi.hpp"
mSpi4::mSpi4(const char* name) : mSpi(name)
{

}
mSpi4::~mSpi4()
{

}
mResult mSpi4::init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    _spixHandle.Instance = SPI4;
    _spixHandle.Init.Mode = SPI_MODE_MASTER;
    _spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    _spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    _spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    _spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    _spixHandle.Init.NSS = SPI_NSS_SOFT;
    _spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    _spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    _spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    _spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    _spixHandle.Init.CRCPolynomial = 0x0;
    _spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    _spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    _spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    _spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    _spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    _spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
    _spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
    _spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    _spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    _spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if(HAL_SPI_Init(&_spixHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
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
    HAL_GPIO_Init(GPIOE,&GPIO_InitStruct); //初始化、
    _spiGpioCs = GPIO_PIN_11;
    _spiGpioGroup = GPIOE;
    return M_RESULT_EOK;
}
mResult mSpi4::deInit()
{
    __HAL_RCC_SPI4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
    return M_RESULT_EOK;
}
void mSpi4::csEnable()
{
    HAL_GPIO_WritePin(_spiGpioGroup, _spiGpioCs, GPIO_PIN_RESET);
}
void mSpi4::csDisable()
{
    HAL_GPIO_WritePin(_spiGpioGroup, _spiGpioCs, GPIO_PIN_SET);
}
mResult mSpi4::write(const uint8_t* buff, size_t len)
{
    if(HAL_SPI_Transmit(&_spixHandle, buff, len, 5000) != HAL_OK)
    {
        printf("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult mSpi4::read(uint8_t* buff, size_t len)
{
    if(HAL_SPI_Receive(&_spixHandle, buff, len, 5000) != HAL_OK)
    {
        printf("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}

static mSpi4* spi4 = nullptr;
int spi4InitFunc()
{
    spi4 = new mSpi4("spi4");
    if(!spi4)
    {
        return -1;
    }
    spi4->init();
    return 0;
}
INIT_EXPORT(spi4InitFunc, "1");