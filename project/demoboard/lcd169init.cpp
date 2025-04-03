#include "mspidrv.hpp"
#include "lcd.hpp"
#include "project.hpp"
#include "gpio.hpp"
#include "sys.h"
#if 1
Lcd169* lcd0 = nullptr;
mDev::mSpi* spi5 = nullptr;

int lcdInit()
{
    gpiox* bl = new gpiox("blh6");
    bl->init([](bool b){if(b)__HAL_RCC_GPIOH_CLK_ENABLE();},GPIOH, GPIO_PIN_6, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    gpiox* dc = new gpiox("dcj11");
    dc->init([](bool b){if(b)__HAL_RCC_GPIOJ_CLK_ENABLE();},GPIOJ, GPIO_PIN_11, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    bl->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    dc->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);

    spi5 = (mDev::mSpi*)mDev::mPlatform::getInstance()->getDevice(DEV_SPI5);
    if(!spi5)
    {
        printf("Error: can not find spi5 drv\r\n");
        return -1;
    }
    lcd0 = new Lcd169(DEV_LCD0, 240, 280, 0, 20, 16);//RGB565 16 BIT 显示
    
    lcd0->init(spi5,dc,nullptr,bl,nullptr);
    
    lcd0->clear();
    
    lcd0->setBackColor(LCD_CYAN);
    lcd0->clearRect(0,0,240,40);
    lcd0->setColor(LCD_RED);
    lcd0->showText(10,10,"hellow world\r\n");
    return 0;
}
INIT_EXPORT(lcdInit, "0.5");
#endif