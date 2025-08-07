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
#include "musbdevicedrv.hpp"
#include "mklog.hpp"
#include "musartdrv.hpp"
#include "project.hpp"
#include "ff.h"
#include "fatfsff.hpp"
#include "lvgl.h"
#include "lv_demo_benchmark.h"
#include "mdisplaydrv.hpp"
#include "mbuttondrv.hpp"
#include "madcdrv.hpp"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
uint8_t data_to_send[64] D2_MEM_ALIGN(4);
//uint8_t data_to_send[50];
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	volatile uint8_t _cnt=0;
	volatile int16_t _temp;
	volatile int32_t _temp2 = alt;
	memset(data_to_send, 0, 64);
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

    //((mDev::mUsbHidDevice*)mDev::mPlatform::getInstance()->getDevice(DEV_VCOM))->send(data_to_send,_cnt);
}

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
    mDev::mLed* led0 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED0);
    mDev::mLed* led1 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED1);
    mDev::mLed* led2 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED2);
    mDev::mAdc* adc1 = (mDev::mAdc*)mDev::mPlatform::getInstance()->getDevice(DEV_ADC1);
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
    workItem* i2cWorkItem = new workItem("i2cWorkItem", 2000, 20, [&](void* param){

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
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(i2cWorkItem);
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(sysInfoWorkItem);
    //uint32_t j;
    CreateNewFile();
    while(1)
    {
        if(led0)
        led0->toggle();
        /*if(hub.wait(WAITING_FOREVER))
        {
            hub.copy(hub.getNode("testNode"),&j);
        }*/
        mthread::threadDelay(1000);
    }
    return 0;
}
