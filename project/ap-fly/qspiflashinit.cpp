#include "w25q128jvs.hpp"
#include "project.hpp"
#include "sys.h"
#include "mspidrv.hpp"
#include "gpio.hpp"
int qspiflashInit()
{
    gpiox* w25qcs = new gpiox("w25qcs");
            w25qcs->init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH);
    W25Q128JVS* w25qxx = new W25Q128JVS(DEV_W25QXX, (mDev::mSpi*)mDev::mDeviceManager::getInstance()->getDevice(DEV_SPI2), w25qcs);
    if(w25qxx->init() != M_RESULT_EOK)
    {
        return -1;
    }
#if 0
    if(w25qxx->chipErase() != M_RESULT_EOK)
    {
        printf("erase chip fail\r\n");
        return -1;
    }
    printf("erase qspiflash ok\r\n");
#endif
#if 1
uint8_t buf[10] = {0,1,2,3,4,5,6,7,7,7};
uint8_t rbuf[10] = {0};
if(w25qxx->writeBuffer(buf, 0x00, 10) != M_RESULT_EOK)
{
    return -1;
}
if(w25qxx->readBuffer(rbuf, 0x00, 10) != M_RESULT_EOK)
{
    return -1;
}
for (int i = 0; i < 10; i++)
{
    printf("0x%x ", rbuf[i]);
}
printf("\r\n");
#endif
    return 0;
}
INIT_EXPORT(qspiflashInit, "0.5");