#include "led.hpp"
#include "gpio.hpp"
#include "project.hpp"

int ledInit()
{
    gpiox* pe9 = new gpiox("pe9");
    pe9->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    gpiox* pe11 = new gpiox("pe11");
    pe11->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_11, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    pe9->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    pe11->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);

    ledx* led0 = new ledx(DEV_LED0);
    led0->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_15);
    ledx* led1 = new ledx(DEV_LED1);
    led1->init([](bool benable){ if(benable) __HAL_RCC_GPIOB_CLK_ENABLE(); },GPIOB,GPIO_PIN_15);
    ledx* led2 = new ledx(DEV_LED2);
    led2->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_11);
    return 0;
}
INIT_EXPORT(ledInit, "0.1");