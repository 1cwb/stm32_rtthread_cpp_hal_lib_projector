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
#include "delay.h"

int main(void)
{
    mEvent mevent;
    mevent.init("mEvnet1", IPC_FLAG_FIFO);
    printf("tony %f\r\n",0.01);
    mDev::mImu* icm42688 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice("icm42688");
    mDev::mImu* icm42605 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice("icm42605");
    if(!icm42688)printf("Error icm42688 = nullptr\r\n");
    if(!icm42605)printf("Error icm42605 = nullptr\r\n");
    mDev::mLed* led0 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led0");
    mDev::mLed* led1 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led1");
    mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer2");
    mDev::mPlatform::getInstance()->getDevice("timer2")->registerInterruptCb([&](mDev::mDevice* dev){
        
    });
    timer2->start(mDev::CHANNEL_1);
    mDev::mTimer* timer1 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer1");
    timer1->registerInterruptCb([&](mDev::mDevice* dev){
        mevent.send(0X01);
    });
    
    //mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer2");
    //timer2->registerInterruptCb([&](mDev::mDevice* dev){
        //mevent.send(0X01);
    //});
    //timer2->start();
    printf("timer1 frq = %lu, timeout = %lu\r\n",timer1->getFreq(),timer1->getTimeOut());
    printf("timer2 frq = %lu, timeout = %lu\r\n",timer2->getFreq(),timer2->getTimeOut());
    delay_ms(3000);
    mthread* th3 = mthread::create("th3",1024,0,20,[&](){
        while(1)
        {
            mthread::threadDelay(20);
            led1->toggle();
        }
    });
    if(th3)
    {
        th3->startup();
    }

    mthread* th4 = mthread::create("th4",1024,0,20,[&](){
        uint32_t test;
        while(1)
        {
            //mthread::threadDelay(2000);
            mevent.recv(0x01,EVENT_FLAG_AND|EVENT_FLAG_CLEAR, WAITING_FOREVER, &test);
            #if 1
                icm42688->updateData();
                printf("YAW:%8f ROLL:%8f PITCH:%8f\r\n",icm42688->getYaw(),icm42688->getRoll(),icm42688->getPitch());
                icm42605->updateData();
                printf("YAW:%8f ROLL:%8f PITCH:%8f\r\n",icm42605->getYaw(),icm42605->getRoll(),icm42605->getPitch());
            #endif
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
    timer1->start();
    printf("total  =  %lu\r\n",timer2->pwmGetMaxPulse());
    while(1)
    {
       led0->toggle();
       mthread::threadSleep(40);
       //printf("thread run now++++++++++++\r\n");
    }
    return 0;
}
