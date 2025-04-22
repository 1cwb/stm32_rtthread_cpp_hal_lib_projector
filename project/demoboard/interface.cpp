#include "mdevice.hpp"
#include "musartdrv.hpp"
#include "musbdevicedrv.hpp"
#include "umcn.hpp"
#include <string>
#include "project.hpp"
#include "mthread.hpp"
#include "sys.h"
#include "mipc.hpp"
#include "madcdrv.hpp"

enum INTERFACE_ID
{
    INTERFACE_ID_VCOM = 0,
    INTERFACE_ID_U1,
    INTERFACE_ID_U2,
    INTERFACE_ID_U3,
    INTERFACE_ID_U4,
    INTERFACE_ID_U5,
    INTERFACE_ID_U6,
    INTERFACE_ID_U8,
    INTERFACE_ID_ADC3,
};

struct interfaceData
{
    INTERFACE_ID id;
    uint8_t data[mDev::mUsart::RX_BUFF_LEN];
    uint32_t len;
    void* p;
};


mMessagequeue uartRecvQueue;
void usartRecvEnter(void* p);
int uartRecvQueueInit(void)
{
    uartRecvQueue.init("uartRecvQueue", sizeof(interfaceData), 20*(sizeof(mMessagequeue::mqMessage_t)+sizeof(interfaceData)), mIpcFlag::IPC_FLAG_FIFO);
    printf("uart queue maxsize = %d, used size = %d\r\n", uartRecvQueue.getMaxSize(), uartRecvQueue.getUsedSize());
    mDev::mUsbHidDevice* usbDev = (mDev::mUsbHidDevice*)mDev::mPlatform::getInstance()->getDevice(DEV_VCOM);
    mDev::mUsart* usartDev1 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART1);
    mDev::mUsart* usartDev2 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART2);
    mDev::mUsart* usartDev3 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART3);
    mDev::mUsart* usartDev4 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART4);
    mDev::mUsart* usartDev5 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART5);
    mDev::mUsart* usartDev6 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART6);
    mDev::mUsart* usartDev8 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART8);
    mDev::mAdc* adc3Dev = (mDev::mAdc*)mDev::mPlatform::getInstance()->getDevice(DEV_ADC3);

    if(usbDev)
    {
        usbDev->registerInterruptCb([&](mDev::mDevice* dev, void* p){
            if(p)
            {
                mDev::mUsbHidDevice::usbData* data = (mDev::mUsbHidDevice::usbData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_VCOM;
                ifdata.len = data->len;
                ifdata.p = dev;
                memset(ifdata.data, 0, sizeof(ifdata.data));
                memcpy(ifdata.data, data->data, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(usartDev1)
    {
        usartDev1->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mUsart::usartData* data = (mDev::mUsart::usartData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_U1;
                ifdata.len = data->len;
                ifdata.p = dev;
                memset(ifdata.data, 0, sizeof(ifdata.data));
                memcpy(ifdata.data, data->data, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(usartDev2)
    {
        usartDev2->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mUsart::usartData* data = (mDev::mUsart::usartData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_U2;
                ifdata.len = data->len;
                ifdata.p = dev;
                memcpy(ifdata.data, data->data, data->len);
                memset(data->data, 0, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(usartDev3)
    {
        usartDev3->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mUsart::usartData* data = (mDev::mUsart::usartData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_U3;
                ifdata.len = data->len;
                ifdata.p = dev;
                memcpy(ifdata.data, data->data, data->len);
                memset(data->data, 0, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(usartDev4)
    {
        usartDev4->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mUsart::usartData* data = (mDev::mUsart::usartData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_U4;
                ifdata.len = data->len;
                ifdata.p = dev;
                memcpy(ifdata.data, data->data, data->len);
                memset(data->data, 0, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(usartDev5)
    {
        usartDev5->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mUsart::usartData* data = (mDev::mUsart::usartData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_U5;
                ifdata.len = data->len;
                ifdata.p = dev;
                memcpy(ifdata.data, data->data, data->len);
                memset(data->data, 0, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(usartDev6)
    {
        usartDev6->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mUsart::usartData* data = (mDev::mUsart::usartData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_U6;
                ifdata.len = data->len;
                ifdata.p = dev;
                memcpy(ifdata.data, data->data, data->len);
                memset(data->data, 0, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(usartDev8)
    {
        usartDev8->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mUsart::usartData* data = (mDev::mUsart::usartData*)p;

                interfaceData ifdata;
                ifdata.id = INTERFACE_ID_U8;
                ifdata.len = data->len;
                ifdata.p = dev;
                memcpy(ifdata.data, data->data, data->len);
                memset(data->data, 0, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(adc3Dev)
    {
        adc3Dev->registerInterruptCb([](mDev::mDevice* dev, void* p){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(p)
            {
                mDev::mAdc::usartData* data = (mDev::mAdc::usartData*)p;
                interfaceData ifdata;
                if(data->type == mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_COMPLETE)
                {
                    ifdata.id = INTERFACE_ID_ADC3;
                    ifdata.len = data->len;
                    ifdata.p = dev;
                    memcpy(ifdata.data, data->data, data->len);
                    memset(data->data, 0, data->len);
                    uartRecvQueue.send(&ifdata, sizeof(interfaceData));
                }
                else
                {
                    //printf("adc3 event type = %u\r\n",data->type);
                }
            }
        });
    }
    mthread* hubthread = mthread::create("hubth",1024,2,20,usartRecvEnter,nullptr);
    if(hubthread)
    {
        hubthread->startup();
    }
    return 0;
}

void usartRecvEnter(void* p)
{
    interfaceData ifdata;
    while(true)
    {
        if(uartRecvQueue.recv(&ifdata, sizeof(ifdata), WAITING_FOREVER) == M_RESULT_EOK)
        {
            switch(ifdata.id)
            {
                case INTERFACE_ID_VCOM:
                for(uint32_t i = 0; i < ifdata.len; i++)
                {
                  printf("%x", ifdata.data[i]);
                }
                printf("\r\n");
                    break;
                case INTERFACE_ID_U1:
                    printf("tony recv %s\r\n",ifdata.data);
                    break;
                case INTERFACE_ID_U2:
                    
                    break;
                case INTERFACE_ID_U3:
                    
                    break;
                case INTERFACE_ID_U4:
                    
                    break;
                case INTERFACE_ID_U5:
                    
                    break;
                case INTERFACE_ID_U6:
                    
                    break;
                case INTERFACE_ID_U8:
                    
                    break;
                case INTERFACE_ID_ADC3:
                {
                    float AdcValues[5];
                    uint16_t TS_CAL1;
                    uint16_t TS_CAL2;
                    uint16_t* ADCxValues = (uint16_t*)ifdata.data;
                    /*
                       使用此函数要特别注意，第1个参数地址要32字节对齐，第2个参数要是32字节的整数倍
                    */
                    AdcValues[0] = ADCxValues[0] * 3.3 / 65536;
                    AdcValues[1] = ADCxValues[1] * 3.3 / 65536; 
                    //AdcValues[2] = ADCxValues[2] * 3.3 / 65536;     
                 
                    /* 根据参考手册给的公式计算温度值 */
                    TS_CAL1 = *(__IO uint16_t *)(0x1FF1E820);
                    TS_CAL2 = *(__IO uint16_t *)(0x1FF1E840);
                    
                    //AdcValues[2] = (110.0 - 30.0) * (ADCxValues[2] - TS_CAL1)/ (TS_CAL2 - TS_CAL1) + 30;  
                    AdcValues[2] = ((110.0f - 30.0f) / (TS_CAL2 - TS_CAL1)) * (ADCxValues[2] - TS_CAL1) + 30.0f;
                    printf("Vbat/4 = %5.3fV, VrefInt = %5.3fV, TempSensor = %5.3f℃\r\n", 
                            AdcValues[0],  AdcValues[1], AdcValues[2]);
                }
                    break;
                default:
                    break;
            }
        }
    }
}
TASK_EXPORT(uartRecvQueueInit, "2");