#include "stm32h7xx_hal.h"
#include "led.h"
#include "mthread.h"
#include "spi.h"
#include "icm42688.h"
#include <math.h>
#include <list>
#include "containers.hpp"
#include "atomic.h"
//#include "platform.hpp"
//using namespace std;
	int16_t iTemperature = 0;
	icm42688RawData_t stAccData;
	icm42688RawData_t stGyroData;
mAtomic<uint32_t> u32val(0);
int main(void)
{
    /*RCC_PeriphCLKInitTypeDef  PeriphClkInit;
    HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInit);
    printf("PeriphClkInit.Spi45ClockSelection = %ld, freq = %lu\r\n",PeriphClkInit.Spi45ClockSelection,HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45));
    memset(&PeriphClkInit, 0, sizeof(PeriphClkInit));
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI45;
    PeriphClkInit.Spi45ClockSelection = RCC_SPI45CLKSOURCE_HSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    memset(&PeriphClkInit, 0, sizeof(PeriphClkInit));
    printf("PeriphClkInit.Spi45ClockSelection = %ld, freq = %lu\r\n",PeriphClkInit.Spi45ClockSelection,HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45));*/
    led0Init();
    led1Init();
    led0Off();
    spi4Init();
    bsp_Icm42688Init();
    u32val |= 0x12;
    printf("u32 = %lu\r\n",u32val);
    mthread* th3 = mthread::create("th3",512,0,20,[&](){
        //using MyString = std::basic_string<char, std::char_traits<char>, mMemAllocator<char>>;
        //using MyList = std::list<test1*, mMemAllocator<test1*>>;
        //MyList listxx;
        
        while(1)
        {
            mthread::threadDelay(20);
            led1Toggle();
            bsp_IcmGetRawData(&stAccData,&stGyroData);
			printf("AccX:%d--AccY:%d--AccZ:%d----GyroX:%d--GyroY:%d--GyroZ:%d\r\n",stAccData.x,stAccData.y,stAccData.z,stGyroData.x,stGyroData.y,stGyroData.z);
            //printf("Accel[0] = %lu, Accel[1]=%lu, Accel[2]=%lu\r\n",AccelCorrected[0],AccelCorrected[1],AccelCorrected[2]);
            //printf("Gyro[0] = %lu, Gyro[1]=%lu, Gyro[2]=%lu\r\n",GyroCorrected[0],GyroCorrected[1],GyroCorrected[2]);
        }
    });
    if(th3)
    {
        th3->startup();
    }

    mthread* th4 = mthread::create("th4",512,0,20,[&](){
        //using MyString = std::basic_string<char, std::char_traits<char>, mMemAllocator<char>>;
        int m = 0;
        while(1)
        {
            //printf("recv ... m = %d\r\n",m);
        }
    });
    if(th4)
    {
        th4->startup();
    }

    int i = 0;
    mTimer* tim1 = mTimer::create("tim1", 100, TIMER_FLAG_PERIODIC, [&](){
        i++;
        if(i == 4)
        {
            i = 0;
        }
        tim1->setTimerAndStart(200);
        //printf("timeout\r\n");
    });
    tim1->start();
    while(1)
    {
       mthread::threadSleep(1000);
       led0On();
       mthread::threadSleep(1000);
       led0Off();
       //HAL_Delay(200);
       delay_ms(200);
       //printf("thread run now\r\n");
    }
    return 0;
}
