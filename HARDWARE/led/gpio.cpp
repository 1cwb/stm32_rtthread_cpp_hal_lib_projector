#include "gpio.hpp"
gpiox::gpiox(const char* name)
:mGpio(name)
{

}
gpiox::~gpiox()
{

}
mResult gpiox::init(const mDev::initCallbackExt& cb, GPIO_TypeDef * gpiox,
    uint16_t pin, uint32_t mode, uint32_t pull,
    uint32_t speed, uint32_t alternate)
{
    GPIO_InitTypeDef GPIO_Initure = {0};
    _initcb = cb;
    if(_initcb)
    {
        _initcb(true);
    }
    _gpiox = gpiox;
    _pin = pin;
    GPIO_Initure.Pin=_pin;
    GPIO_Initure.Mode=mode;
    GPIO_Initure.Pull=pull;
    GPIO_Initure.Speed= speed;
    GPIO_Initure.Alternate = alternate;
    HAL_GPIO_Init(_gpiox,&GPIO_Initure);
    return M_RESULT_EOK;
}
mResult gpiox::deInit()
{
    if(_initcb)
    {
        _initcb(false);
    }
    HAL_GPIO_DeInit(_gpiox, _pin);
    return M_RESULT_EOK;
}
void gpiox::setLevel(mDev::mGpio::GPIOLEVEL level)
{
    level == LEVEL_HIGH ? HAL_GPIO_WritePin(_gpiox, _pin, GPIO_PIN_SET) : HAL_GPIO_WritePin(_gpiox, _pin, GPIO_PIN_RESET);
}
mDev::mGpio::GPIOLEVEL gpiox::getLevel()
{
    GPIO_PinState state = HAL_GPIO_ReadPin(_gpiox, _pin);
    if(state == GPIO_PIN_SET) return LEVEL_HIGH;
    else return LEVEL_LOW;
}
void gpiox::toggle()
{
    HAL_GPIO_TogglePin(_gpiox, _pin);
}