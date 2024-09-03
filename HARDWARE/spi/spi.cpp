#include "spi.hpp"
spix::spix(const char* name) : mSpi(name)
{

}
spix::~spix()
{

}
mResult spix::init(const mDev::initCallbackExt& cb ,SPI_HandleTypeDef* spihandle)
{
    _initcb = cb;
    memcpy(&_spixHandle, spihandle, sizeof(SPI_HandleTypeDef));
    _spixHandle.State = HAL_SPI_STATE_RESET;
    if(HAL_SPI_Init(&_spixHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    //_spiCs.init(mDev::initCallbackExt(),csgpiox,cspin,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    return M_RESULT_EOK;
}
mResult spix::deInit()
{
    HAL_SPI_DeInit(&_spixHandle);
    return M_RESULT_EOK;
}
void spix::csEnable(mDev::mGpio* cspin)
{
    if(cspin)
    {
        cspin->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    }
    else
    {
        printf("Error: %s()%d CS pin not set\r\n",__FUNCTION__,__LINE__);
    }
}
void spix::csDisable(mDev::mGpio* cspin)
{
    if(cspin)
    {
        cspin->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
    }
    else
    {
        printf("Error: %s()%d CS pin not set\r\n",__FUNCTION__,__LINE__);
    }
}
mResult spix::write(const uint8_t* buff, size_t len)
{
    if(HAL_SPI_Transmit(&_spixHandle, buff, len, 5000) != HAL_OK)
    {
        printf("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult spix::read(uint8_t* buff, size_t len)
{
    if(HAL_SPI_Receive(&_spixHandle, buff, len, 5000) != HAL_OK)
    {
        printf("Error: %s()%d\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}

extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    spix* spi = containerof(hspi, spix, _spixHandle);
    if(hspi == spi->spixHandle())
    {
        spi->runInitCallback(true);
    }
}
extern "C" void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    spix* spi = containerof(hspi, spix, _spixHandle);
    if(hspi == spi->spixHandle())
    {
        spi->runInitCallback(false);
    }
}