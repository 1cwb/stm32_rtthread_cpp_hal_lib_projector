#include "spi.hpp"
#include "led.hpp"
#include "DFRobot_ICM42688.h"
#include "mdevice.hpp"
#include "containers.hpp"
#include "stm32h7xx_hal_conf.h"

int initAllDevice()
{
    mDev::mDevice* led0 = new ledx("led0",GPIOE,GPIO_PIN_9,[](){__HAL_RCC_GPIOE_CLK_ENABLE();});
    led0->init();
    mDev::mDevice* led1 = new ledx("led1",GPIOA,GPIO_PIN_7,[](){__HAL_RCC_GPIOA_CLK_ENABLE();});
    led1->init();
    mDev::mDevice* spi4 = new mSpi4("spi4");
    spi4->init();
    mDev::mDevice* df42688 = new DFRobot_ICM42688_SPI();
    df42688->init();

    return 0;
}
INIT_EXPORT(initAllDevice, "1");