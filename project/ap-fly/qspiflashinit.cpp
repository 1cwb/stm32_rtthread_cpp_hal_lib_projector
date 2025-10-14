#include "w25q128jvs.hpp"
#include "project.hpp"
#include "sys.h"
#include "mspidrv.hpp"
#include "gpio.hpp"
#include <string>

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
std::string qspiflashTestStr = "hello world"; 
char* buf = "hello world"; 
uint8_t rbuf[20] = {0}; 

// 1. 先擦除要写入的区域（4KB扇区擦除）
uint32_t test_addr = 0x00;
printf("Erasing sector at 0x%08X...\r\n", test_addr);
if(w25qxx->sectorErase(test_addr) != M_RESULT_EOK)
{
    printf("Sector erase failed!\r\n");
    return -1; 
}

// 2. 等待擦除完成
printf("Waiting for erase to complete...\r\n");
HAL_Delay(100);  // 等待擦除完成

// 3. 现在写入数据
printf("Writing: %s (len=%d)\r\n", buf, strlen(buf)+1);
if(w25qxx->writeBuffer((uint8_t*)buf, test_addr, strlen(buf)+1) != M_RESULT_EOK) 
{ 
    printf("Write failed!\r\n");
    return -1; 
} 

// 4. 读取并验证
if(w25qxx->readBuffer(rbuf, test_addr, strlen(buf)+1) != M_RESULT_EOK) 
{ 
    printf("Read failed!\r\n");
    return -1; 
} 

// 5. 打印结果
printf("Raw data read: ");
for(int i = 0; i < strlen(buf)+1; i++) {
    printf("%02x ", rbuf[i]);
}
printf("\r\n");
printf("read qspiflash ok: %s\r\n", (char*)rbuf); 
#endif
    return 0;
}
INIT_EXPORT(qspiflashInit, "0.5");