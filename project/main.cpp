#include "stm32h7xx_hal.h"
#include "mleddrv.hpp"
#include "mthread.hpp"
#include <math.h>
#include <list>
#include "containers.hpp"
#include "atomic.hpp"
#include "waitqueue.hpp"
#include "DFRobot_ICM42688.h"
#include "mplatform.hpp"
#include "mgpiodrv.hpp"
#include "mtimerdrv.hpp"
#include "delay.h"
#include "mmagnetmetordrv.hpp"
#include "mbarometordrv.hpp"
#include "MadgwickAHRS.hpp"
#include "bmi088.hpp"
#include "qmc5883.hpp"
#include "usart.h"
int main(void)
{
    mEvent mevent;
    mevent.init("mEvnet1", IPC_FLAG_FIFO);
    mDev::mImu* imu1 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice("imu1");
    mDev::mImu* imu2 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice("imu2");
    mDev::mMagnetmetor* mag1 = (mDev::mMagnetmetor*)mDev::mPlatform::getInstance()->getDevice("mag1");
    mDev::mBarometor* mb1 = (mDev::mBarometor*)mDev::mPlatform::getInstance()->getDevice("baro1");
    mDev::mLed* led0 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led0");
    mDev::mLed* led1 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led1");
    mDev::mLed* led2 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice("led2");
    mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer2");
    if(timer2)
    {
        timer2->registerInterruptCb([&](mDev::mDevice* dev){
            
        });
        timer2->start(mDev::CHANNEL_1);
        printf("timer2 frq = %lu, timeout = %lu\r\n",timer2->getFreq(),timer2->getTimeOutUs());
    }

    mDev::mTimer* timer1 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice("timer1");
    if(timer1)
    {
        timer1->registerInterruptCb([&](mDev::mDevice* dev){
            mevent.send(0X01);
        });
        timer1->start();
        printf("timer1 frq = %lu, timeout = %lu\r\n",timer1->getFreq(),timer1->getTimeOutUs());
    }

    mthread* IMUCALTHREAD = mthread::create("IMUTHREAD",1024,0,20,[&](){
        uint32_t test = 0;
        int n = 0;
        int p = 0;
        while(1)
        {
            if(led1)
            led1->toggle();
            mevent.recv(0x01,EVENT_FLAG_AND|EVENT_FLAG_CLEAR, WAITING_FOREVER, &test);
            if(test == 0x01)
            {
                #if 1
                if(imu1 && imu2)
                {
                    ((mDev::mGpio*)mDev::mPlatform::getInstance()->getDevice("pd9"))->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
                    imu1->updateData();
                    imu2->updateData();
                    ((mDev::mGpio*)mDev::mPlatform::getInstance()->getDevice("pd9"))->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
                    n++;
                    //if(n == 5)
                    {
                        n = 0;
                        //printf("IMU1 YAW:%8f ROLL:%8f PITCH:%8f\r\n",imu1->getYaw(),imu1->getRoll(),imu1->getPitch());
                        //ANO_DT_Send_Status((imu1->getRoll()+imu2->getRoll())*0.5, (imu1->getPitch()+imu2->getPitch())*0.5, (imu1->getYaw()+imu1->getYaw())*0.5, 0, 0, 1);
                    }
                }
                if(imu2)
                {
                    
                    p++;
                    if(p == 5)
                    {
                        p = 0;
                    //printf("IMU2 YAW:%8f ROLL:%8f PITCH:%8f\r\n",imu2->getYaw(),imu2->getRoll(),imu2->getPitch());
                    }
                }
                if(mb1)
                {
                    //mb1->updateData();
                }
                #endif
            }
        }
    });
    if(IMUCALTHREAD)
    {
        IMUCALTHREAD->startup();
    }

    mthread* magexx = mthread::create("magexx",1024,1,20,[&](){
        while (true)
        {
            if(mag1)
            {
                ((mDev::mGpio*)mDev::mPlatform::getInstance()->getDevice("pd8"))->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
                mag1->updateData();
                ((mDev::mGpio*)mDev::mPlatform::getInstance()->getDevice("pd8"))->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
            }
        }
        
    });
    if(magexx)
    {
        magexx->startup();
    }

#if 0
    mTimer* tim1 = mTimer::create("tim1", 3000, TIMER_FLAG_PERIODIC, [&](){
        //tim1->setTimerAndStart(3000);
    });
    tim1->start();
#endif
    while(1)
    {
        if(led0)
        led0->toggle();
        if(led1)
        led1->toggle();
        if(led2)
        led2->toggle();
        mthread::threadSleep(1000);
       //printf("thread run now++++++++++++\r\n");
    }
    return 0;
}
