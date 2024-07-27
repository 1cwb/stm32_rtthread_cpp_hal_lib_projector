#include "stm32h7xx_hal.h"
#include "mled.hpp"
#include "mthread.h"
#include <math.h>
#include <list>
#include "containers.hpp"
#include "atomic.h"
#include "waitqueue.hpp"
#include "DFRobot_ICM42688.h"
#include "mplatform.hpp"
//using namespace std;
	//int16_t iTemperature = 0;
	//icm42688RawData_t stAccData;
	//icm42688RawData_t stGyroData;
int main(void)
{
    //printf("WHOAMI:%x\r\n",bsp_WhoAmi());
    printf("tony %f\r\n",0.01);
    DFRobot_ICM42688_SPI* icm42688 = (DFRobot_ICM42688_SPI*)mDev::mPlatform::getInstance()->getDevice("icm42688");
    mDev::mLed* led0 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led0");
    mDev::mLed* led1 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led1");
    mthread* th3 = mthread::create("th3",1024,0,20,[&](){
        float accelDataX,accelDataY,accelDataZ,gyroDataX,gyroDataY,gyroDataZ,tempData;
        while(1)
        {
            mthread::threadDelay(20);
            led1->toggle();
            #if 1
            icm42688->getFIFOData();
            tempData= icm42688->getTemperature();
            accelDataX = icm42688->getAccelDataX();
            accelDataY= icm42688->getAccelDataY();
            accelDataZ= icm42688->getAccelDataZ();
            gyroDataX= icm42688->getGyroDataX();
            gyroDataY= icm42688->getGyroDataY();
            gyroDataZ= icm42688->getGyroDataZ();
            //printf("Temperature: %d C ",(int32_t)tempData);
            #if 0
            printf("A_X: %lf mg ",accelDataX);
            printf("A_Y: %lf mg ",accelDataY);
            printf("A_Z: %lf mg ",accelDataZ);
            printf("G_X: %lf dps ",gyroDataX);
            printf("G_Y: %lf dps ",gyroDataY);
            printf("G_Z: %lf dps\r\n",gyroDataZ);
            #endif
            #endif
            //bsp_IcmGetRawData(&stAccData,&stGyroData);
			//printf("AccX:%d--AccY:%d--AccZ:%d----GyroX:%d--GyroY:%d--GyroZ:%d\r\n",stAccData.x,stAccData.y,stAccData.z,stGyroData.x,stGyroData.y,stGyroData.z);
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
    });
    tim1->start();
    while(1)
    {
       mthread::threadSleep(1000);
       led0->on();
       mthread::threadSleep(1000);
       led0->off();
       //HAL_Delay(200);
       //delay_ms(200);
       //printf("thread run now\r\n");
    }
    return 0;
}
