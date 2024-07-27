#include "stm32h7xx_hal_conf.h"
#include "spi.hpp"
#include "led.hpp"
#include "DFRobot_ICM42688.h"
#include "mdevice.hpp"
#include "containers.hpp"
#include "mgpio.hpp"
#include "gpio.hpp"
#include "mplatform.hpp"

int initAllDevice()
{
    mDev::mDevice* led0 = new ledx("led0",GPIOE,GPIO_PIN_9,[](bool benable){ if(benable) __HAL_RCC_GPIOE_CLK_ENABLE(); else __HAL_RCC_GPIOE_CLK_DISABLE();});
    mDev::mDevice* led1 = new ledx("led1",GPIOA,GPIO_PIN_7,[](bool benable){if(benable) __HAL_RCC_GPIOA_CLK_ENABLE(); else __HAL_RCC_GPIOA_CLK_DISABLE();});
    mDev::mDevice* spi4 = new mSpi4("spi4",[](bool b){});
    mDev::mDevice* df42688 = new DFRobot_ICM42688_SPI();
    mDev::mDevice* gpiob0 = new gpiox("gpiob0",[](bool benable){
        if(benable)
        {
            printf("hal_exti_enalbe\r\n");
            __HAL_RCC_GPIOB_CLK_ENABLE();
            /* 配置 EXTI 中断源 到key1 引脚、配置中断优先级*/
            HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
            /* 使能中断 */
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        }
    },GPIOB,GPIO_PIN_0,GPIO_MODE_IT_RISING,GPIO_PULLDOWN);
    gpiob0->registerInterruptCb([](mDev::mDevice* dev){
        gpiox* gpio0 = reinterpret_cast<gpiox*>(dev);
        __HAL_GPIO_EXTI_CLEAR_IT(gpio0->getPin());
        printf("worinimabi\r\n");
    });
    return 0;
}
INIT_EXPORT(initAllDevice, "1");

extern "C" void EXTI0_IRQHandler(void)
{
    mDev::mDevice* dev = nullptr;
    if((dev = mDev::mPlatform::getInstance()->getDevice("gpiob0")) != nullptr)
    {
        dev->runInterruptCb();
    }
}