#include "stm32h7xx_hal.h"
#include "mleddrv.hpp"
#include "mthread.hpp"
#include <math.h>
#include <list>
#include "containers.hpp"
#include "atomic.hpp"
#include "waitqueue.hpp"
#include "mdevicemanager.hpp"
#include "mgpiodrv.hpp"
#include "delay.h"
#include "mmagnetmetordrv.hpp"
#include "mbarometordrv.hpp"
#include "qmc5883.hpp"
#include "usart.h"
#include "workqueue.hpp"
#include "workqueuemanager.hpp"
#include "umcn.hpp"
#include "msystickdrv.hpp"
#include "systeminfo.hpp"
#include "mklog.hpp"
#include "musartdrv.hpp"
#include "project.hpp"
#include "datapublish.hpp"
#include "crsf.hpp"
#include "madcdrv.hpp"
#include "mpwmdrv.hpp"
#include "mtimerdrv.hpp"

int main(void)
{
    mDev::mLed* led0 = (mDev::mLed*)mDev::mDeviceManager::getInstance()->getDevice(DEV_LED0);
    mDev::mLed* led1 = (mDev::mLed*)mDev::mDeviceManager::getInstance()->getDevice(DEV_LED1);
    mDev::mLed* led2 = (mDev::mLed*)mDev::mDeviceManager::getInstance()->getDevice(DEV_LED2);
    mDev::mTimer* timer1 = (mDev::mTimer*)mDev::mDeviceManager::getInstance()->getDevice(DEV_TIMER1);
    mDev::mAdc* adc1 = (mDev::mAdc*)mDev::mDeviceManager::getInstance()->getDevice(DEV_ADC1);
    mDev::mPWM* pwm1 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM1);
    mDev::mPWM* pwm2 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM2);
    mDev::mPWM* pwm3 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM3);
    mDev::mPWM* pwm4 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM4);
    mDev::mPWM* pwm5 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM5);
    mDev::mPWM* pwm6 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM6);
    mDev::mPWM* pwm7 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM7);
    mDev::mPWM* pwm8 = (mDev::mPWM*)mDev::mDeviceManager::getInstance()->getDevice(DEV_PWM8);

    if(timer1)
    {
        timer1->registerInterruptCb([&](mDev::mDevice* dev, void* p){
           // printf("on time\r\n");
        });
        //timer1->start();
        //KLOGI("timer1 frq = %lu, timeout = %lu\r\n",timer1->getFreq(),timer1->getTimeOutUs());
    }
    if(pwm1)
    {
        pwm1->setDutyCycle(10.0f);
        pwm1->start();
    }
    if(pwm2)
    {
        pwm2->setDutyCycle(20.0f);
        pwm2->start();
    }
    if(pwm3)
    {
        pwm3->setDutyCycle(30.0f);
        pwm3->start();
    }
    if(pwm4)
    {
        pwm4->setDutyCycle(40.0f);
        pwm4->start();
    }
    if(pwm5)
    {
        pwm5->setDutyCycle(50.0f);
        pwm5->start();
    }
    if(pwm6)
    {
        pwm6->setDutyCycle(60.0f);
        pwm6->start();
    }
    if(pwm7)
    {
        pwm7->setDutyCycle(70.0f);
        pwm7->start();
    }
    if(pwm8)
    {
        pwm8->setDutyCycle(80.0f);
        pwm8->start();
    }
    workItem* ledWorkItem = new workItem("ledworkItem", 0, 200, [&](void* param){
        if(led1)
        led1->toggle();
        if(led2)
        led2->toggle();
        if(adc1)
        {
            adc1->start(mDev::recvMode::RECV_MODE_IT, (uint32_t*)adc1->getRxBuff(), adc1->RX_BUFF_LEN);
        }
    }, nullptr);
    workItem* dataSendbackItem = new workItem("dataSendbackItem", 1000, 20, [&](void* param){
        float ahrsData[4] = {0.0};
        float powerV = 0.0f;
        const uint16_t max_resolution_value = (1 << crsf::getInstance()->getResolutionBits()) - 1;
        if(ahrsHub->poll(ahrsSendBackToRemoteNode))
        {
            ahrsHub->copy(ahrsSendBackToRemoteNode, ahrsData);

            crsf::getInstance()->getTxChannelData()[0] = static_cast<uint16_t>(
                fmaxf(fminf(ahrsData[0], 360.0f), 0.0f) * ((float)max_resolution_value / 360.0f)); // YAW [0°,360°] -> [0,2047] (2047/360≈5.6861)

            crsf::getInstance()->getTxChannelData()[1] = static_cast<uint16_t>(
                (fmaxf(fminf(ahrsData[1], 180.0f), -180.0f) + 180.0f) * ((float)max_resolution_value / 360.0f)); // ROLL [-180°,180°]->[0,2047]
                
            crsf::getInstance()->getTxChannelData()[2] = static_cast<uint16_t>(
                (fmaxf(fminf(ahrsData[2], 90.0f), -90.0f) + 90.0f) * ((float)max_resolution_value / 180.0f)); // PITCH [-90°,90°]->[0,2047]
            if(powerHub->poll(powerSendbackNode))
            {
                powerHub->copy(ahrsSendBackToRemoteNode, &powerV);
                crsf::getInstance()->getTxChannelData()[3] = static_cast<uint16_t>(powerV * 100.0f);
            }
            
            crsf::getInstance()->packRcChannels(CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED,0,4);
            crsf::getInstance()->writeTelemetryData(crsf::getInstance()->getFrame(),crsf::getInstance()->getPacketLength());
            crsf::getInstance()->sendTelemetryData();

            //ALOGI("YAW:%d ROLL:%d PITCH:%d \r\n",crsf::getInstance()->getTxChannelData()[0], crsf::getInstance()->getTxChannelData()[1], crsf::getInstance()->getTxChannelData()[2]);
            //ALOGI("YAW:%10f ROLL:%10f PITCH:%10f P%10f\r\n",ahrsData[0], ahrsData[1], ahrsData[2], ahrsData[3]);
        }
    }, nullptr);
    workItem* sysInfoWorkItem = new workItem("sysinfo", 2000, 3000, [](void* param){
        #if 0
        ALOGI("memHeap Total:%lu Used:%lu(%0.2f%%)\r\n",mMem::getInstance()->total(),mMem::getInstance()->used(),((float)mMem::getInstance()->used()/(float)mMem::getInstance()->total() * 100.0F));
        ALOGI("thread stack Info:\r\n");
        systemInfo::getInstance()->showAllThreadStackSizeInfo();
        ALOGI("CPU USAGE: %f\r\n",systemInfo::getInstance()->getCpuUsage());
        #endif
    }, nullptr);

    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(ledWorkItem);
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(dataSendbackItem);
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(sysInfoWorkItem);

    while(1)
    {
        if(led0)
        led0->toggle();
        mthread::threadDelay(1000);
    }
    return 0;
}
