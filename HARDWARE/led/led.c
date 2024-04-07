#include "led.h"
void led0Init()
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOE_CLK_ENABLE(); //开启 GPIOE 时钟
    GPIO_Initure.Pin=GPIO_PIN_9; //PC9
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL; //上拉
    GPIO_Initure.Speed= GPIO_SPEED_FREQ_MEDIUM; //高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}
void led0On()
{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
}
void led0Off()
{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
}
void led0Toggle()
{
    HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9);
}

void led1Init()
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE(); //开启 GPIOC 时钟
    GPIO_Initure.Pin=GPIO_PIN_7; //PC13
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; //推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL; //上拉
    GPIO_Initure.Speed= GPIO_SPEED_FREQ_MEDIUM; //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}
void led1On()
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); //PC13 置 0
}

void led1Off()
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); //PC13 置 1
}
void led1Toggle()
{
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7); //PC13 置 1
}
