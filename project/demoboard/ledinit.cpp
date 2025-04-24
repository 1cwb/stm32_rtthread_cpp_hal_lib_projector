#include "led.hpp"
#include "gpio.hpp"
#include "project.hpp"

int ledInit()
{
    #if 1
    gpiox* pi4 = new gpiox("pi4");
    pi4->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOI, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    gpiox* pi6 = new gpiox("pi6");
    pi6->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOI, GPIO_PIN_6, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    pi4->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    pi6->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    #endif

    ledx* led0 = new ledx(DEV_LED0);
    led0->init([](bool benable){ if(benable) __HAL_RCC_GPIOC_CLK_ENABLE(); },GPIOC,GPIO_PIN_13);
    ledx* led1 = new ledx(DEV_LED1);
    led1->init([](bool benable){ if(benable) __HAL_RCC_GPIOB_CLK_ENABLE(); },GPIOB,GPIO_PIN_15);
    ledx* led2 = new ledx(DEV_LED2);
    led2->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_11);
    return 0;
}
INIT_EXPORT(ledInit, "0.1");