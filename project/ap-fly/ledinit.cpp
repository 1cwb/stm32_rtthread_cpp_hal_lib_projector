#include "led.hpp"
#include "gpio.hpp"
#include "project.hpp"

int ledInit()
{
    //gpiox* pd8 = new gpiox("pd8");
    //pd8->init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    //gpiox* pd9 = new gpiox("pd9");
    //pd9->init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    //pd8->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    //pd9->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    ledx* led0 = new ledx(DEV_LED0);
    led0->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_15);
    ledx* led1 = new ledx(DEV_LED1);
    led1->init([](bool benable){ if(benable) __HAL_RCC_GPIOB_CLK_ENABLE(); },GPIOB,GPIO_PIN_15);
    ledx* led2 = new ledx(DEV_LED2);
    led2->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_11);
    return 0;
}
INIT_EXPORT(ledInit, "0.1");