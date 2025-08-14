#include "stm32h7xx_hal.h"
#include "mleddrv.hpp"
#include "mthread.hpp"
#include <math.h>
#include <list>
#include "containers.hpp"
#include "atomic.hpp"
#include "waitqueue.hpp"
#include "DFRobot_ICM42688.h"
#include "mdevicemanager.hpp"
#include "mgpiodrv.hpp"
#include "mtimerdrv.hpp"
#include "delay.h"
#include "mmagnetmetordrv.hpp"
#include "mbarometordrv.hpp"
#include "MadgwickAHRS.hpp"
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
#include "ff.h"
#include "fatfsff.hpp"
#include "datapublish.hpp"
#include "crsf.hpp"
#include "madcdrv.hpp"


/*
*********************************************************************************************************
*	函 数 名: CreateNewFile
*	功能说明: 在SD卡创建一个新文件，文件内容填写“www.armfly.com”
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
char FsWriteBuf[1024] = {"FatFS Write Demo \r\n www.armfly.com caonima\r\n"};

static void CreateNewFile(void)
{
	FRESULT result;
	unsigned int bw;
	char path[32];

    mFatFs fs;
    mFile file;

 	/* 挂载文件系统 */
     result = fs.mount("0:/", 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("挂载文件系统失败 (%s)\r\n", mFatFs::errToStr(result));
        return;
	}

	/* 打开文件 */
	sprintf(path, "%sarmfly.txt", "0:/");
	result =file.open(path, FA_CREATE_ALWAYS | FA_WRITE);
	if (result == FR_OK)
	{
		printf("armfly.txt 文件打开成功\r\n");
	}
	else
	{
		printf("armfly.txt 文件打开失败  (%s)\r\n", mFatFs::errToStr(result));
	}

	/* 写一串数据 */
	result = file.write(FsWriteBuf, strlen(FsWriteBuf), &bw);
	if (result == FR_OK)
	{
		printf("armfly.txt 文件写入成功\r\n");
	}
	else
	{
		printf("armfly.txt 文件写入失败  (%s)\r\n", mFatFs::errToStr(result));
	}

	/* 关闭文件*/
	file.close();

	/* 卸载文件系统 */
	mFatFs::unmount("0:/");
}

int main(void)
{
    mDev::mLed* led0 = (mDev::mLed*)mDev::mDeviceManager::getInstance()->getDevice(DEV_LED0);
    mDev::mLed* led1 = (mDev::mLed*)mDev::mDeviceManager::getInstance()->getDevice(DEV_LED1);
    mDev::mLed* led2 = (mDev::mLed*)mDev::mDeviceManager::getInstance()->getDevice(DEV_LED2);
    mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mDeviceManager::getInstance()->getDevice(DEV_TIMER2);
    mDev::mTimer* timer1 = (mDev::mTimer*)mDev::mDeviceManager::getInstance()->getDevice(DEV_TIMER1);
    mDev::mAdc* adc1 = (mDev::mAdc*)mDev::mDeviceManager::getInstance()->getDevice(DEV_ADC1);
    uint8_t usbBuff[64];

    if(timer2)
    {
        timer2->registerInterruptCb([&](mDev::mDevice* dev, void* p){
            
        });
        timer2->start(mDev::CHANNEL_1);
        //KLOGI("timer2 frq = %lu, timeout = %lu\r\n",timer2->getFreq(),timer2->getTimeOutUs());
    }

    if(timer1)
    {
        timer1->registerInterruptCb([&](mDev::mDevice* dev, void* p){
            printf("time out \r\n");
        });
        timer1->start();
        //KLOGI("timer1 frq = %lu, timeout = %lu\r\n",timer1->getFreq(),timer1->getTimeOutUs());
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
    workItem* sysInfoWorkItem = new workItem("sysinfo", 2000, 1000, [](void* param){
        #if 1
        ALOGI("memHeap Total:%lu Used:%lu(%0.2f%%)\r\n",mMem::getInstance()->total(),mMem::getInstance()->used(),((float)mMem::getInstance()->used()/(float)mMem::getInstance()->total() * 100.0F));
        ALOGI("thread stack Info:\r\n");
        systemInfo::getInstance()->showAllThreadStackSizeInfo();
        ALOGI("CPU USAGE: %f\r\n",systemInfo::getInstance()->getCpuUsage());
        #endif
    }, nullptr);



    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(ledWorkItem);
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(dataSendbackItem);
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(sysInfoWorkItem);

    //uint32_t j;
    CreateNewFile();
    while(1)
    {
        if(led0)
        led0->toggle();

        mthread::threadDelay(1000);
    }
    return 0;
}
