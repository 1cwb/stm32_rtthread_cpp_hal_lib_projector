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
#include "mgpiodrv.hpp"
#include "datapublish.hpp"
#include "crsf.hpp"
#include "mklog.hpp"

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
    INTERFACE_ID_ADC1,
    INTERFACE_ID_ADC2,
    INTERFACE_ID_ADC3,
};

struct interfaceData
{
    INTERFACE_ID id;
    uint8_t* data;
    uint32_t dataPerSize;
    uint32_t dataOfobjCount;
    uint32_t len;
    void* p;
};


mMessagequeue uartRecvQueue;
void usartRecvEnter(void* p);
int uartRecvQueueInit(void)
{
    uartRecvQueue.init("uartRecvQueue", sizeof(interfaceData), 40*(sizeof(mMessagequeue::mqMessage_t)+sizeof(interfaceData)), mIpcFlag::IPC_FLAG_FIFO);
    printf("uart queue maxsize = %d, used size = %d\r\n", uartRecvQueue.getMaxSize(), uartRecvQueue.getUsedSize());
    mDev::mUsbHidDevice* usbDev = (mDev::mUsbHidDevice*)mDev::mDeviceManager::getInstance()->getDevice(DEV_VCOM);
    mDev::mUsart* usartDev1 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART1);
    mDev::mUsart* usartDev2 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART2);
    mDev::mUsart* usartDev3 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART3);
    mDev::mUsart* usartDev4 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART4);
    mDev::mUsart* usartDev5 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART5);
    mDev::mUsart* usartDev6 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART6);
    mDev::mUsart* usartDev8 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART8);
    mDev::mAdc* adc1Dev = (mDev::mAdc*)mDev::mDeviceManager::getInstance()->getDevice(DEV_ADC1);
    mDev::mAdc* adc3Dev = (mDev::mAdc*)mDev::mDeviceManager::getInstance()->getDevice(DEV_ADC3);
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
                ifdata.data = data->data;
                //memset(ifdata.data, 0, sizeof(ifdata.data));
                //memcpy(ifdata.data, data->data, data->len);
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
                ifdata.data = data->data;
                //memcpy(ifdata.data, data->data, data->len);
                //memset(data->data, 0, data->len);
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
                ifdata.data = data->data;
                //memcpy(ifdata.data, data->data, data->len);
                //memset(data->data, 0, data->len);
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
                ifdata.data = data->data;
                //memcpy(ifdata.data, data->data, data->len);
                //memset(data->data, 0, data->len);
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
                ifdata.data = data->data;
                //memcpy(ifdata.data, data->data, data->len);
                //memset(data->data, 0, data->len);
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
                ifdata.data = data->data;
                //memcpy(ifdata.data, data->data, data->len);
                //memset(data->data, 0, data->len);
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
                ifdata.data = data->data;
                //memcpy(ifdata.data, data->data, data->len);
                //memset(data->data, 0, data->len);
                uartRecvQueue.send(&ifdata, sizeof(interfaceData));
            }
        });
    }
    if(adc1Dev)
    {
        adc1Dev->registerInterruptCb([](mDev::mDevice* dev, void* p){
            if(p)
            {
                mDev::mAdc::usartData* data = (mDev::mAdc::usartData*)p;
                interfaceData ifdata;
                if(data->type == mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_COMPLETE)
                {
                    ifdata.id = INTERFACE_ID_ADC1;
                    ifdata.len = data->len;
                    ifdata.dataPerSize = data->dataPerSize;
                    ifdata.dataOfobjCount = data->dataOfobjCount;
                    ifdata.p = dev;
                    //memcpy(ifdata.data, data->data, data->len);
                    ifdata.data = data->data;
                    uartRecvQueue.send(&ifdata, sizeof(interfaceData));
                }
                else
                {
                    //printf("adc3 event type = %u\r\n",data->type);
                }
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
                    ifdata.dataPerSize = data->dataPerSize;
                    ifdata.dataOfobjCount = data->dataOfobjCount;
                    ifdata.p = dev;
                    //memcpy(ifdata.data, data->data, data->len);
                    //memset(data->data, 0, data->len);
                    ifdata.data = data->data;
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
    uint32_t* adc1Data = nullptr;
    uint8_t adc1DataCount = 0;
    const uint8_t avragetime1 = 1;

    mDev::mUsart* usartDev2 = (mDev::mUsart*)mDev::mDeviceManager::getInstance()->getDevice(DEV_USART2);
    crsf::getInstance()->registerUartSend(usartDev2, &mDev::mUsart::sendData);
    while(true)
    {
        if(uartRecvQueue.recv(&ifdata, sizeof(ifdata), WAITING_FOREVER) == M_RESULT_EOK)
        {
            std::string str;
            switch(ifdata.id)
            {
                case INTERFACE_ID_VCOM:
                for(uint32_t i = 0; i < ifdata.len; i++)
                {
                  printf("%x ", ifdata.data[i]);
                }
                printf("\r\n");
                    break;
                case INTERFACE_ID_U1:
                    str = std::string((const char*)ifdata.data, ifdata.len);
                    ALOGD("tony recv %s\r\n",str.c_str());
                    if(strncmp(str.c_str(),"reboot",6) == 0)
                    {
                        SoftReset();
                    }
                    break;
                case INTERFACE_ID_U2:
                    /*for(uint32_t i = 0; i < ifdata.len; i++)
                    {
                      printf("%2x ", ifdata.data[i]);
                    }
                    printf("\r\n");*/
                    if(crsf::getInstance()->rxDataParse(ifdata.data, ifdata.len) == M_RESULT_EOK)
                    {
                        if(crsf::getInstance()->unpackRcChannels() == M_RESULT_EOK)
                        {
                            for (size_t i = 0; i < 4; i++)
                            {
                                //printf("%d ", crsf::getInstance()->getRxChannelData()[i]);
                            }
                            //printf("\r\n");
                        }
                    }
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
                case INTERFACE_ID_ADC1:
                {
                    if(!adc1Data)
                    {
                        adc1Data = new uint32_t[ifdata.dataOfobjCount];
                        memset(adc1Data, 0, ifdata.dataOfobjCount*sizeof(uint32_t));
                    }
                    //printf("ifdata.len = %lu, ifdata.dataPerSize = %lu, ifdata.dataOfobjCount = %lu\r\n",ifdata.len, ifdata.dataPerSize,ifdata.dataOfobjCount);
                    for(uint32_t i = 0; i < ifdata.len/ifdata.dataPerSize; i+= ifdata.dataOfobjCount)
                    {
                        for(uint32_t j = 0; j < ifdata.dataOfobjCount; j++)
                        {
                            if(ifdata.dataPerSize == 2)
                            {
                                adc1Data[j] += ((uint16_t*)ifdata.data)[i+j];
                            }
                            else if(ifdata.dataPerSize == 4)
                            {
                                adc1Data[j] += ((uint32_t*)ifdata.data)[i+j];
                            }
                            else
                            {
                                adc1Data[j] += ((uint8_t*)ifdata.data)[i+j];
                            }
                            //printf("%d ",((uint16_t*)ifdata.data)[i+j]);
                        }
                        //printf("\r\n");
                    }
                    adc1DataCount++;
                    if(adc1DataCount >= avragetime1)
                    {
                        uint32_t div = ifdata.len/(ifdata.dataPerSize*ifdata.dataOfobjCount)*avragetime1;
                        for(uint32_t j = 0; j < ifdata.dataOfobjCount; j++)
                        {
                            adc1Data[j] /= div;
                        }

                        if(powerHub)
                        {
                            float measured_voltage = (adc1Data[0] * 3.3f) / 65535.0f;
                            // 电阻误差补偿（假设已知实际电阻偏差）
                            const float R1_actual = 198000.0f; // 实测R1值
                            const float R2_actual = 22400.0f;  // 实测R2值
                            float true_voltage = measured_voltage * (R1_actual + R2_actual) / R2_actual;
                            powerHub->publish(&true_voltage,false);
                        }
                        #if 0
                        if(mcnJoyStickData && bNeedUpdate)
                        {
                            //crsf::getInstance()->bind();
                            const uint16_t max_resolution_value = (1 << crsf::getInstance()->getResolutionBits()) - 1;
    
                            // 将ADC值0-65535映射到0-2047范围
                            crsf::getInstance()->getTxChannelData()[0] = (adcData[1] * max_resolution_value) / 65535;
                            crsf::getInstance()->getTxChannelData()[1] = (adcData[2] * max_resolution_value) / 65535;
                            crsf::getInstance()->getTxChannelData()[2] = (adcData[3] * max_resolution_value) / 65535;
                            crsf::getInstance()->getTxChannelData()[3] = (adcData[4] * max_resolution_value) / 65535;

                            crsf::getInstance()->packRcChannels(CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED,0,4);
                            /*for(uint32_t i = 0; i < crsf::getInstance()->getPacketLength(); i++)
                            {
                                printf("%2x ",crsf::getInstance()->getFrame()->bytes[i]);
                            }
                            printf("\r\n");*/
                            crsf::getInstance()->writeTelemetryData(crsf::getInstance()->getFrame(),crsf::getInstance()->getPacketLength());
                            crsf::getInstance()->sendTelemetryData();
                            mcnJoyStickData->publish(adcData, false);
                            bNeedUpdate = false;
                        }
                        #endif
                        #if 0
                        for (uint32_t i = 0; i < ifdata.dataOfobjCount; i++)
                        {
                            printf("%d ",adc1Data[i]);
                        }
                        printf("\r\n");
                        // 在ADC采样代码中添加误差补偿
                        float measured_voltage = (adc1Data[0] * 3.3f) / 65535.0f;

                        // 电阻误差补偿（假设已知实际电阻偏差）
                        const float R1_actual = 198000.0f; // 实测R1值
                        const float R2_actual = 22400.0f;  // 实测R2值
                        float true_voltage = measured_voltage * (R1_actual + R2_actual) / R2_actual;
                        printf(" v = %f\r\n", true_voltage);
                        #endif
                        memset(adc1Data, 0, ifdata.dataOfobjCount*sizeof(uint32_t));
                        adc1DataCount = 0;
                    }
                    //printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\r\n");


                }
                    break;
                default:
                    break;
            }
        }
    }
}
TASK_EXPORT(uartRecvQueueInit, "2");