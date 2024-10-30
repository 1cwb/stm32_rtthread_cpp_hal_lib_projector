#include "bmi088new.hpp"
#include "mspidrv.hpp"
#include "mplatform.hpp"
#include "gpio.hpp"

int bimxxInit()
{
    mDev::mSpi* spi1 = (mDev::mSpi*)mDev::mPlatform::getInstance()->getDevice("spi1");
    if(!spi1)
    {
        return -1;
    }
    gpiox* imu1acs = new gpiox("imu1acs");
    imu1acs->init([](bool b){if(b){__HAL_RCC_GPIOA_CLK_ENABLE();}},GPIOA,GPIO_PIN_2,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);

    gpiox* imu1gcs = new gpiox("imu1gcs");
    imu1gcs->init([](bool b){if(b){__HAL_RCC_GPIOA_CLK_ENABLE();}},GPIOA,GPIO_PIN_3,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);

    imu1acs->setLevel(mDev::mGpio::LEVEL_HIGH);
    imu1gcs->setLevel(mDev::mGpio::LEVEL_HIGH);

    bmi088* imu1 = new bmi088("imu1",spi1,imu1acs,imu1gcs);
    imu1->init();

    mDev::mSpi* spi4 = (mDev::mSpi*)mDev::mPlatform::getInstance()->getDevice("spi4");
    if(!spi4)
    {
        return -1;
    }
    gpiox* imu2acs = new gpiox("imu2acs");
    imu2acs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_13,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    gpiox* imu2gcs = new gpiox("imu2gcs");
    imu2gcs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_2,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    imu2acs->setLevel(mDev::mGpio::LEVEL_HIGH);
    imu2gcs->setLevel(mDev::mGpio::LEVEL_HIGH);

    bmi088* imu2 = new bmi088("imu2",spi4,imu2acs,imu2gcs);
    imu2->init();
    return 0; 
}
INIT_EXPORT(bimxxInit, "1");