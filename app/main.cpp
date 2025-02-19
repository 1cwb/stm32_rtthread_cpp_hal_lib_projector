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

    ((mDev::mUsbHidDevice*)mDev::mPlatform::getInstance()->getDevice(DEV_VCOM))->send(data_to_send,_cnt);
}

/*
*********************************************************************************************************
*	函 数 名: CreateNewFile
*	功能说明: 在SD卡创建一个新文件，文件内容填写“www.armfly.com”
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
FATFS fs;
FIL file;
char FsWriteBuf[1024] = {"FatFS Write Demo \r\n www.armfly.com \r\n"};
/* FatFs API的返回值 */
static const char * FR_Table[]= 
{
	"FR_OK：成功",				                             /* (0) Succeeded */
	"FR_DISK_ERR：底层硬件错误",			                 /* (1) A hard error occurred in the low level disk I/O layer */
	"FR_INT_ERR：断言失败",				                     /* (2) Assertion failed */
	"FR_NOT_READY：物理驱动没有工作",			             /* (3) The physical drive cannot work */
	"FR_NO_FILE：文件不存在",				                 /* (4) Could not find the file */
	"FR_NO_PATH：路径不存在",				                 /* (5) Could not find the path */
	"FR_INVALID_NAME：无效文件名",		                     /* (6) The path name format is invalid */
	"FR_DENIED：由于禁止访问或者目录已满访问被拒绝",         /* (7) Access denied due to prohibited access or directory full */
	"FR_EXIST：文件已经存在",			                     /* (8) Access denied due to prohibited access */
	"FR_INVALID_OBJECT：文件或者目录对象无效",		         /* (9) The file/directory object is invalid */
	"FR_WRITE_PROTECTED：物理驱动被写保护",		             /* (10) The physical drive is write protected */
	"FR_INVALID_DRIVE：逻辑驱动号无效",		                 /* (11) The logical drive number is invalid */
	"FR_NOT_ENABLED：卷中无工作区",			                 /* (12) The volume has no work area */
	"FR_NO_FILESYSTEM：没有有效的FAT卷",		             /* (13) There is no valid FAT volume */
	"FR_MKFS_ABORTED：由于参数错误f_mkfs()被终止",	         /* (14) The f_mkfs() aborted due to any parameter error */
	"FR_TIMEOUT：在规定的时间内无法获得访问卷的许可",		 /* (15) Could not get a grant to access the volume within defined period */
	"FR_LOCKED：由于文件共享策略操作被拒绝",				 /* (16) The operation is rejected according to the file sharing policy */
	"FR_NOT_ENOUGH_CORE：无法分配长文件名工作区",		     /* (17) LFN working buffer could not be allocated */
	"FR_TOO_MANY_OPEN_FILES：当前打开的文件数大于_FS_SHARE", /* (18) Number of open files > _FS_SHARE */
	"FR_INVALID_PARAMETER：参数无效"	                     /* (19) Given parameter is invalid */
};
static void CreateNewFile(void)
{
	FRESULT result;
	unsigned int bw;
	char path[32];


 	/* 挂载文件系统 */
	result = f_mount(&fs, "0:/", 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("挂载文件系统失败 (%s)\r\n", FR_Table[result]);
	}

	/* 打开文件 */
	sprintf(path, "%sarmfly.txt", "0:/");
	result = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
	if (result == FR_OK)
	{
		printf("armfly.txt 文件打开成功\r\n");
	}
	else
	{
		printf("armfly.txt 文件打开失败  (%s)\r\n", FR_Table[result]);
	}

	/* 写一串数据 */
	result = f_write(&file, FsWriteBuf, strlen(FsWriteBuf), &bw);
	if (result == FR_OK)
	{
		printf("armfly.txt 文件写入成功\r\n");
	}
	else
	{
		printf("armfly.txt 文件写入失败  (%s)\r\n", FR_Table[result]);
	}

	/* 关闭文件*/
	f_close(&file);

	/* 卸载文件系统 */
	f_mount(NULL, "0:/", 0);
}

int main(void)
{
    mEvent mevent;
    mevent.init("mEvnet1", IPC_FLAG_FIFO);

    mcnHub hub("test", 4);
    hub.init();
    hub.subscribe("testNode");
    mDev::mImu* imu1 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice(DEV_IMU1);
    mDev::mImu* imu2 = (mDev::mImu*)mDev::mPlatform::getInstance()->getDevice(DEV_IMU2);
    mDev::mMagnetmetor* mag1 = (mDev::mMagnetmetor*)mDev::mPlatform::getInstance()->getDevice(DEV_MAG1);
    mDev::mBarometor* mb1 = (mDev::mBarometor*)mDev::mPlatform::getInstance()->getDevice(DEV_BARO1);
    mDev::mLed* led0 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED0);
    mDev::mLed* led1 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED1);
    mDev::mLed* led2 = (mDev::mLed*)mDev::mPlatform::getInstance()->getDevice(DEV_LED2);
    mDev::mTimer* timer2 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice(DEV_TIMER2);
    mDev::mSystick* systickx = (mDev::mSystick*)mDev::mPlatform::getInstance()->getDevice(DEV_SYSTICK);

    uint8_t usbBuff[64];
    if(systickx)
    {
        //KLOGI("systick is find \r\n");
        
    }
    if(timer2)
    {
        timer2->registerInterruptCb([&](mDev::mDevice* dev, void* p){
            
        });
        timer2->start(mDev::CHANNEL_1);
        //KLOGI("timer2 frq = %lu, timeout = %lu\r\n",timer2->getFreq(),timer2->getTimeOutUs());
    }
    mDev::mTimer* timer1 = (mDev::mTimer*)mDev::mPlatform::getInstance()->getDevice(DEV_TIMER1);
    if(timer1)
    {
        timer1->registerInterruptCb([&](mDev::mDevice* dev, void* p){
            mevent.send(0X01);
        });
        timer1->start();
        //KLOGI("timer1 frq = %lu, timeout = %lu\r\n",timer1->getFreq(),timer1->getTimeOutUs());
    }
    workItem* ledWorkItem = new workItem("ledworkItem", 0, 200, [&](void* param){
        if(led1)
        led1->toggle();
        if(led2)
        led2->toggle();
    }, nullptr);
    workItem* i2cWorkItem = new workItem("i2cWorkItem", 2000, 20, [&](void* param){
        mag1->updateData();
        mb1->updateData();
        //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
        //ALOGI("YAW:%d ROLL:%10f PITCH:%10f P%10f\r\n",/*imu1->getYaw()*/mag1->getMageX(),imu1->getRoll(),imu1->getPitch(),mb1->getPressure());

        ANO_DT_Send_Status(imu2->getRoll(), imu2->getPitch(), imu2->getYaw(), 0, 0, 1);
       //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
        //ALOGI("YAW:%10f ROLL:%10f PITCH:%10f P%10f\r\n",imu2->getYaw(),imu2->getRoll(),imu2->getPitch(),mb1->getPressure());

    }, nullptr);
    workItem* sysInfoWorkItem = new workItem("sysinfo", 2000, 3000, [](void* param){
        #if 0
        ALOGI("memHeap Total:%lu Used:%lu(%0.2f%%)\r\n",mMem::getInstance()->total(),mMem::getInstance()->used(),((float)mMem::getInstance()->used()/(float)mMem::getInstance()->total() * 100.0F));
        ALOGI("thread stack Info:\r\n");
        systemInfo::getInstance()->showAllThreadStackSizeInfo();
        systemInfo::getInstance()->getCpuUsage();
        #endif
    }, nullptr);

    workItem* IMUItem = new workItem("imu", 2000, 5, [&](void* param){
        if(imu1 && imu2)
        {
            //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
            imu1->updateData();
            imu2->updateData();
            //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET);
            //ANO_DT_Send_Status((imu1->getRoll()+imu2->getRoll()/2.0f), (imu1->getPitch()+imu2->getPitch())/2.0f, (imu1->getYaw()+imu2->getYaw())/2.0f, 0, 0, 1);
            //ANO_DT_Send_Status(imu2->getRoll(), imu2->getPitch(), imu2->getYaw(), 0, 0, 1);
            //HAL_UART_Transmit_DMA(&UART1_Handler,DMABUFF,13); 
        }
    }, nullptr);

    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(ledWorkItem);
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(i2cWorkItem);
    workQueueManager::getInstance()->find(WORKQUEUE_LP_WORK)->scheduleWork(sysInfoWorkItem);
    workQueueManager::getInstance()->find(WORKQUEUE_HP_WORK)->scheduleWork(IMUItem);
    mthread* IMUCALTHREAD = mthread::create("IMUTHREAD",1024,0,20,[&](void* p){
        uint32_t test = 0;
        while(1)
        {
            mevent.recv(0x20,EVENT_FLAG_OR|EVENT_FLAG_CLEAR, WAITING_FOREVER, &test);
            if(test == 0x20)//USB RECV
            {
                printf("%s\r\n",usbBuff);
            }
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
