#include "led.hpp"
#include "mbase.hpp"
#include "containers.hpp"
mResult ledx::init(const mDev::initCallbackExt& cb ,GPIO_TypeDef* gpiox, uint16_t pin ,bool highIsoff)
{
    GPIO_InitTypeDef GPIO_Initure;
    _initcb = cb;
    if(_initcb)
    {
        _initcb(true);
    }
    _gpiox = gpiox;
    _pin = pin;
    _highIsoff = highIsoff;
        
    GPIO_Initure.Pin=_pin; //PC9
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL; //上拉
    GPIO_Initure.Speed= GPIO_SPEED_FREQ_MEDIUM; //高速
    HAL_GPIO_Init(_gpiox,&GPIO_Initure);
    return M_RESULT_EOK;
}
mResult ledx::deInit()
{
    if(_initcb)
    {
        _initcb(false);
    }
    HAL_GPIO_DeInit(_gpiox, _pin);
    return M_RESULT_EOK;
}
void ledx::on()
{
    HAL_GPIO_WritePin(_gpiox, _pin, _highIsoff ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void ledx::off()
{
    HAL_GPIO_WritePin(_gpiox, _pin, _highIsoff ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void ledx::toggle()
{
    HAL_GPIO_TogglePin(_gpiox, _pin);
}
#if 0
int led0Init()
{
    static ledx*  led0 = new ledx("led0",GPIOE,GPIO_PIN_9,[](){__HAL_RCC_GPIOE_CLK_ENABLE();});
    led0->init();
    return 0;
}
INIT_EXPORT(led0Init, "0.1");

int led1Init()
{
    static ledx led1("led1",GPIOA,GPIO_PIN_7,[](){__HAL_RCC_GPIOA_CLK_ENABLE();});
    led1.init();
    return 0;
}
INIT_EXPORT(led1Init, "0.1");
#endif