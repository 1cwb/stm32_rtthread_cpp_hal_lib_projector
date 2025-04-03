#include "w25qxx.hpp"
#include "project.hpp"
#include "sys.h"
int qspiflashInit()
{
    W25QXX* w25qxx = new W25QXX(DEV_W25QXX);
    if(w25qxx->QSPIW25QxxInit() != M_RESULT_EOK)
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
INIT_EXPORT(qspiflashInit, "0.4");