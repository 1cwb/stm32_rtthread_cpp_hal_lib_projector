#include "spi.hpp"
spix::spix(const char* name) : mSpi(name),_spiCs(std::string(name).append("cs").c_str())
{

}
spix::~spix()
{

}
mResult spix::init(const mDev::initCallbackExt& cb ,SPI_HandleTypeDef* spihandle ,GPIO_TypeDef* csgpiox, uint16_t cspin)
{
    _initcb = cb;
    memcpy(&_spixHandle, spihandle, sizeof(SPI_HandleTypeDef));
    _spixHandle.State = HAL_SPI_STATE_RESET;
    if(HAL_SPI_Init(&_spixHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    _spiCs.init(mDev::initCallbackExt(),csgpiox,cspin,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    return M_RESULT_EOK;
}
mResult spix::deInit()
{
    HAL_SPI_DeInit(&_spixHandle);
    return M_RESULT_EOK;
}
void spix::csEnable()
{
    _spiCs.setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
}
void spix::csDisable()
{
    _spiCs.setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
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