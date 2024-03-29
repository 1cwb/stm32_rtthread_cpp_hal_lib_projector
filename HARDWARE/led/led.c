#include "led.h"
void led_init()
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOC_CLK_ENABLE(); //开启 GPIOC 时钟
    GPIO_Initure.Pin=GPIO_PIN_13; //PC13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL; //上拉
    GPIO_Initure.Speed= GPIO_SPEED_FREQ_MEDIUM; //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
}
void led_on()
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET); //PC13 置 0
}
void led_off()
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET); //PC13 置 1
}