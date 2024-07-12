#include "stm32h7xx_hal.h"
#include "led.h"
#include "mthread.h"
#include "spi.h"
#include "icm42688.h"
#include <math.h>
#include <list>
#include "containers.hpp"
#include "atomic.h"
#include "waitqueue.hpp"

//using namespace std;
	int16_t iTemperature = 0;
	icm42688RawData_t stAccData;
	icm42688RawData_t stGyroData;
int main(void)
{
    spi4Init();
    bsp_Icm42688Init();
    delay_ms(1000);
    printf("WHOAMI:%x\r\n",bsp_WhoAmi());

    mthread* th3 = mthread::create("th3",512,0,20,[&](){
        
        while(1)
        {
            mthread::threadDelay(20);
            led1Toggle();
            bsp_IcmGetRawData(&stAccData,&stGyroData);
			printf("AccX:%d--AccY:%d--AccZ:%d----GyroX:%d--GyroY:%d--GyroZ:%d\r\n",stAccData.x,stAccData.y,stAccData.z,stGyroData.x,stGyroData.y,stGyroData.z);
        }
    });
    if(th3)
    {
        th3->startup();
    }

    mthread* th4 = mthread::create("th4",512,0,20,[&](){
        while(1)
        {
            mthread::threadDelay(2000);
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
       //led0On();
       mthread::threadSleep(1000);
       //led0Off();
       //HAL_Delay(200);
       delay_ms(200);
       //printf("thread run now\r\n");
    }
    return 0;
}
