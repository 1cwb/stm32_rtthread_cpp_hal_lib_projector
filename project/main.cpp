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

int main(void)
{
    mEvent mevent;
    mevent.init("mEvnet1", IPC_FLAG_FIFO);
    printf("tony %f\r\n",0.01);
    DFRobot_ICM42688_SPI* icm42688 = (DFRobot_ICM42688_SPI*)mDev::mPlatform::getInstance()->getDevice("icm42688");
    DFRobot_ICM42688_SPI* icm42605 = (DFRobot_ICM42688_SPI*)mDev::mPlatform::getInstance()->getDevice("icm42605");

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
            #if 1
            printf("4A_X: %lf mg ",accelDataX);
            printf("A_Y: %lf mg ",accelDataY);
            printf("A_Z: %lf mg ",accelDataZ);
            printf("G_X: %lf dps ",gyroDataX);
            printf("G_Y: %lf dps ",gyroDataY);
            printf("G_Z: %lf dps\r\n",gyroDataZ);
            #endif
            icm42605->getFIFOData();
            tempData= icm42605->getTemperature();
            accelDataX = icm42605->getAccelDataX();
            accelDataY= icm42605->getAccelDataY();
            accelDataZ= icm42605->getAccelDataZ();
            gyroDataX= icm42605->getGyroDataX();
            gyroDataY= icm42605->getGyroDataY();
            gyroDataZ= icm42605->getGyroDataZ();
            //printf("Temperature: %d C ",(int32_t)tempData);
            #if 1
            printf("5A_X: %lf mg ",accelDataX);
            printf("A_Y: %lf mg ",accelDataY);
            printf("A_Z: %lf mg ",accelDataZ);
            printf("G_X: %lf dps ",gyroDataX);
            printf("G_Y: %lf dps ",gyroDataY);
            printf("G_Z: %lf dps\r\n",gyroDataZ);
            #endif
            #endif
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
        }
    });
    if(th4)
    {
        th4->startup();
    }

    mTimer* tim1 = mTimer::create("tim1", 3000, TIMER_FLAG_PERIODIC, [&](){
        //tim1->setTimerAndStart(3000);
    });
    tim1->start();
    timer1->updateFreq(20);
    printf("total  =  %lu\r\n",timer2->pwmGetMaxPulse());
    while(1)
    {
       led0->toggle();
       mthread::threadSleep(40);
       //printf("thread run now++++++++++++\r\n");
    }
    return 0;
}
