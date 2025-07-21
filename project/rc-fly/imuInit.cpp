#include "DFRobot_ICM42605.h"
#include "DFRobot_ICM42688.h"
#include "mspidrv.hpp"
#include "mplatform.hpp"
#include "gpio.hpp"
#include "project.hpp"

int imuSensorInit()
{
    mDev::mSpi* spi1 = (mDev::mSpi*)mDev::mPlatform::getInstance()->getDevice(DEV_SPI1);
    if(!spi1)
    {
        return -1;
    }
    gpiox* imu1cs = new gpiox("imu1cs");
    imu1cs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_15,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    imu1cs->setLevel(mDev::mGpio::LEVEL_HIGH);
    DFRobot_ICM42688_SPI* imu1 = new DFRobot_ICM42688_SPI(DEV_IMU1,spi1,imu1cs);


    mDev::mSpi* spi4 = (mDev::mSpi*)mDev::mPlatform::getInstance()->getDevice(DEV_SPI4);
    if(!spi4)
    {
        return -1;
    }
    gpiox* imu2cs = new gpiox("imu2cs");
    imu2cs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_13,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    imu2cs->setLevel(mDev::mGpio::LEVEL_HIGH);
    DFRobot_ICM42605_SPI* imu2 = new DFRobot_ICM42605_SPI(DEV_IMU2,spi4,imu2cs);
    return 0; 
}
INIT_EXPORT(imuSensorInit, "1");