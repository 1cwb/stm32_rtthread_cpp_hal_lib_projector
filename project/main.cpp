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
            static int timeCount = 0;
            timeCount++;
            if(timeCount == 5)
            {
                timeCount = 0;
                //mevent.send(0X02);
            }
            mevent.send(0X01);
        });
        timer1->start();
        printf("timer1 frq = %lu, timeout = %lu\r\n",timer1->getFreq(),timer1->getTimeOutUs());
    }
    mag1->registerInterruptCb([&](mDev::mDevice* dev){
        mevent.send(0X04);
    });
    mthread* IMUCALTHREAD = mthread::create("IMUTHREAD",1024,0,20,[&](void* p){
        uint32_t test = 0;
        mag1->prepareData();
        while(1)
        {
            if(led1)
            led1->toggle();
            mevent.recv(0x01|0x02|0x04,EVENT_FLAG_OR|EVENT_FLAG_CLEAR, WAITING_FOREVER, &test);
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
            if(test & 0x02)
            {
                if(mb1)
                {
                    //mb1->updateData();
                }
                if(mag1)
                {
                    
                }
            }
            else if(test&0x04)
            {
                //printf("mag x:%d, y:%d, z%d\r\n",mag1->getMageX(),mag1->getMageY(),mag1->getMageZ());
                                        mag1->prepareData();
            }
            else
            {
                if(imu1 && imu2)
                {
                    imu1->updateData();
                    imu2->updateData();
                    //ANO_DT_Send_Status(imu1->getRoll(), imu1->getPitch(), imu1->getYaw(), 0, 0, 1);
                    //printf("YAW:%10f ROLL:%10f PITCH:%10f P%10f\r\n",imu1->getYaw(),imu1->getRoll(),imu1->getPitch(),mb1->getPressure());
                    //HAL_UART_Transmit_DMA(&UART1_Handler,DMABUFF,13); 
                }
            }
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET);
        }
    },nullptr);
    if(IMUCALTHREAD)
    {
        IMUCALTHREAD->startup();
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
