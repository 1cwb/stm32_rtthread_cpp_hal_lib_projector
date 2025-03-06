#include "mdevice.hpp"
#include "musartdrv.hpp"
#include "musbdevicedrv.hpp"
#include "umcn.hpp"
#include <string>
#include "project.hpp"
#include "mthread.hpp"
#include "sys.h"

mcnHub hubUsb("hubusb", 64);
mcnHub hubUsart1("hubUsart1", sizeof(mDev::mUsart::usartData));
mcnHub hubUsart2("hubUsart2", sizeof(mDev::mUsart::usartData));
mcnHub hubUsart3("hubUsart3", sizeof(mDev::mUsart::usartData));
mcnHub hubUsart4("hubUsart4", sizeof(mDev::mUsart::usartData));
mcnHub hubUsart5("hubUsart5", sizeof(mDev::mUsart::usartData));
mcnHub hubUsart6("hubUsart6", sizeof(mDev::mUsart::usartData));
mcnHub hubUsart8("hubUsart8", sizeof(mDev::mUsart::usartData));

mcnNode* usbNode = nullptr;
mcnNode* usart1Node = nullptr;
mcnNode* usart2Node = nullptr;
mcnNode* usart3Node = nullptr;
mcnNode* usart4Node = nullptr;
mcnNode* usart5Node = nullptr;
mcnNode* usart6Node = nullptr;
mcnNode* usart8Node = nullptr;

mthread* hubthread = nullptr;

void usartRecvEnter(void* p);

int hubUsartInit(void)
{
    hubUsb.init();
    usbNode = hubUsb.subscribe("vcom");

    hubUsart1.init();
    usart1Node = hubUsart1.subscribe("u1recv");

    hubUsart2.init();
    usart2Node = hubUsart2.subscribe("u2recv");

    hubUsart3.init();
    usart3Node = hubUsart3.subscribe("u3recv");

    hubUsart4.init();
    usart4Node = hubUsart4.subscribe("u4recv");

    hubUsart5.init();
    usart5Node = hubUsart5.subscribe("u5recv");

    hubUsart6.init();
    usart6Node = hubUsart6.subscribe("u6recv");

    hubUsart8.init();
    usart8Node = hubUsart8.subscribe("u8recv");

    mDev::mUsbHidDevice* usbDev = (mDev::mUsbHidDevice*)mDev::mPlatform::getInstance()->getDevice(DEV_VCOM);
    mDev::mUsart* usartDev1 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART1);
    mDev::mUsart* usartDev2 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART2);
    mDev::mUsart* usartDev3 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART3);
    mDev::mUsart* usartDev4 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART4);
    mDev::mUsart* usartDev5 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART5);
    mDev::mUsart* usartDev6 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART6);
    mDev::mUsart* usartDev8 = (mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART8);

    if(usbDev)
    {
        usbDev->registerInterruptCb([&](mDev::mDevice* dev, void* p){
            //mDev::mUsbHidDevice::usbData* data = (mDev::mUsbHidDevice::usbData*)p;
            if(p)
            {
                hubUsb.publish(p, false);
            }
        });
    }
    if(usartDev1)
    {
        usartDev1->registerInterruptCb([](mDev::mDevice* dev, void* data){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(data)
            {
                hubUsart1.publish(data, false);
            }
        });
    }
    if(usartDev2)
    {
        usartDev2->registerInterruptCb([](mDev::mDevice* dev, void* data){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(data)
            {
                hubUsart2.publish(data, false);
            }
        });
    }
    if(usartDev3)
    {
        usartDev3->registerInterruptCb([](mDev::mDevice* dev, void* data){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(data)
            {
                hubUsart3.publish(data, false);
            }
        });
    }
    if(usartDev4)
    {
        usartDev4->registerInterruptCb([](mDev::mDevice* dev, void* data){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(data)
            {
                hubUsart4.publish(data, false);
            }
        });
    }
    if(usartDev5)
    {
        usartDev5->registerInterruptCb([](mDev::mDevice* dev, void* data){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(data)
            {
                hubUsart5.publish(data, false);
            }
        });
    }
    if(usartDev6)
    {
        usartDev6->registerInterruptCb([](mDev::mDevice* dev, void* data){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(data)
            {
                hubUsart6.publish(data, false);
            }
        });
    }
    if(usartDev8)
    {
        usartDev8->registerInterruptCb([](mDev::mDevice* dev, void* data){
            //mDev::mUsart::usartData* pdata = reinterpret_cast<mDev::mUsart::usartData*>(data);
            if(data)
            {
                hubUsart8.publish(data, false);
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
    mDev::mUsart::usartData data;
    char buff[64] = {0};
    //int ret = 0;
    while(true)
    {
        if(hubUsb.poll(usbNode))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsb.copy(usbNode, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usbNode->getName() ,buff);
        }
        if(hubUsart1.poll(usart1Node))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsart1.copy(usart1Node, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usart1Node->getName() ,buff);
            //if(strncmp("reboot", (char*)data.data,strlen("reboot")) == 0)
            {
                SoftReset();
            }
        }
        if(hubUsart2.poll(usart2Node))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsart2.copy(usart2Node, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usart2Node->getName() ,buff);
        }
        if(hubUsart3.poll(usart3Node))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsart3.copy(usart3Node, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usart3Node->getName() ,buff);
        }
        if(hubUsart4.poll(usart4Node))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsart4.copy(usart4Node, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usart4Node->getName() ,buff);
        }
        if(hubUsart5.poll(usart5Node))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsart5.copy(usart5Node, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usart5Node->getName() ,buff);
        }
        if(hubUsart6.poll(usart6Node))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsart6.copy(usart6Node, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usart6Node->getName() ,buff);
        }
        if(hubUsart8.poll(usart8Node))
        {
            //memset(buff, 0, sizeof(buff));
            hubUsart8.copy(usart8Node, &data);
            memcpy(buff, data.data, data.len);
            printf("%s recv: %s\r\n",usart8Node->getName() ,buff);
        }
        mthread::threadDelay(5);
        //snprintf(buff, sizeof(buff), "usart2 %d\r\n", ret++);
        //((mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART2))->sendData((const uint8_t*)buff, strlen(buff)+1);
        //((mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART3))->sendData((const uint8_t*)"helloworld\r\n", 13);
        //((mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART4))->sendData((const uint8_t*)"helloworld\r\n", 13);
        //((mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART5))->sendData((const uint8_t*)"helloworld\r\n", 13);
        //((mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART6))->sendData((const uint8_t*)"helloworld\r\n", 13);
        //((mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice(DEV_USART8))->sendData((const uint8_t*)"helloworld\r\n", 13);

    }
}
TASK_EXPORT(hubUsartInit, "0.1");