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
#include "mgpio.hpp"
#include "mtimer.hpp"
//using namespace std;
	//int16_t iTemperature = 0;
	//icm42688RawData_t stAccData;
	//icm42688RawData_t stGyroData;

int main(void)
{
    //printf("WHOAMI:%x\r\n",bsp_WhoAmi());
    mEvent mevent;
    mevent.init("mEvnet1", IPC_FLAG_FIFO);
    printf("tony %f\r\n",0.01);
    DFRobot_ICM42688_SPI* icm42688 = (DFRobot_ICM42688_SPI*)mDev::mPlatform::getInstance()->getDevice("icm42688");
    mDev::mLed* led0 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led0");
    led0->off();
    mDev::mLed* led1 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led1");
    
    mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer2");
    mDev::mPlatform::getInstance()->getDevice("timer2")->registerInterruptCb([&](mDev::mDevice* dev){
        mevent.send(0X01);
    });
    timer2->start(mDev::CHANNEL_1);
    mDev::mTimer* timer1 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer1");
    timer1->registerInterruptCb([&](mDev::mDevice* dev){
        
    });
    timer1->start();
    
    //mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer2");
    //timer2->registerInterruptCb([&](mDev::mDevice* dev){
        //mevent.send(0X01);
    //});
    //timer2->start();
    printf("timer1 frq = %lu, timeout = %lu\r\n",timer1->getFreq(),timer1->getTimeOut());
    printf("timer2 frq = %lu, timeout = %lu\r\n",timer2->getFreq(),timer2->getTimeOut());
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
        uint32_t test;
        while(1)
        {
            //mthread::threadDelay(2000);
            mevent.recv(0x01,EVENT_FLAG_AND|EVENT_FLAG_CLEAR, WAITING_FOREVER, &test);
            led0->toggle();
        }
    });
    if(th4)
    {
        th4->startup();
    }

    int i = 0;
    mTimer* tim1 = mTimer::create("tim1", 3000, TIMER_FLAG_PERIODIC, [&](){
        i++;
        timer2->updateFreq(i*20);
        printf("update freq = %lu\r\n",i*20);
        //tim1->setTimerAndStart(3000);
    });
    tim1->start();
    timer1->updateFreq(20);
    int m = 0;
    printf("total  =  %lu\r\n",timer2->pwmGetMaxPulse());
    while(1)
    {
       timer2->pwmSetDutyCycle(m,mDev::CHANNEL_1);
       //printf("total  =  %lu, cur = %lu\r\n",bzpwm->getMaxPulse(), bzpwm->getCurPulse());
       m++;
       if(m >= 100)
       {
        m = 0;
       }
       //mthread::threadSleep(1000);
       //bzpwm->setDutyCycle(10.00);
       //led0->on();
       //bz->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
       mthread::threadSleep(40);
       //bzpwm->setDutyCycle(99.00);
       //led0->off();
       //bz->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
       //HAL_Delay(200);
       //delay_ms(200);
       //printf("thread run now++++++++++++\r\n");
    }
    return 0;
}
