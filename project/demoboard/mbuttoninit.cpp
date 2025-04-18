#include "gpio.hpp"
#include "project.hpp"
#include "mbuttondrv.hpp"
#include "mthread.hpp"

int buttonInit()
{
    gpiox* ph7 = new gpiox("H7");
    ph7->init([](bool b){if(b)__HAL_RCC_GPIOH_CLK_ENABLE();},GPIOH, GPIO_PIN_7, GPIO_MODE_INPUT, GPIO_PULLUP);
    mDev::mButton* bt1 = new mDev::mButton(DEV_BUTTON_UP, ph7);
    bt1->registerEventCb([](mDev::mButton* bt, mDev::mButtonEvent ev){
        
        printf("%s pressed ev = %d\r\n",bt->getDeviceName(), ev);
    });

    gpiox* PB13 = new gpiox("B13");
    PB13->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_13, GPIO_MODE_INPUT, GPIO_PULLUP);
    mDev::mButton* bt2 = new mDev::mButton(DEV_BUTTON_DOWN, PB13);
    bt2->registerEventCb([](mDev::mButton* bt, mDev::mButtonEvent ev){
        
        printf("%s pressed ev = %d\r\n",bt->getDeviceName(), ev);
    });

    gpiox* PB14 = new gpiox("B14");
    PB14->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_14, GPIO_MODE_INPUT, GPIO_PULLUP);
    mDev::mButton* bt3 = new mDev::mButton(DEV_BUTTON_LEFT, PB14);
    bt3->registerEventCb([](mDev::mButton* bt, mDev::mButtonEvent ev){
        
        printf("%s pressed ev = %d\r\n",bt->getDeviceName(), ev);
    });

    gpiox* PB12 = new gpiox("B12");
    PB12->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_12, GPIO_MODE_INPUT, GPIO_PULLUP);
    mDev::mButton* bt4 = new mDev::mButton(DEV_BUTTON_RIGHT, PB12);
    bt4->registerEventCb([](mDev::mButton* bt, mDev::mButtonEvent ev){
        
        printf("%s pressed ev = %d\r\n",bt->getDeviceName(), ev);
    });

    mDev::mButtonManager* btm = new mDev::mButtonManager(DEV_BUTTON_MANAGER);
    btm->addButton(bt1);
    btm->addButton(bt2);
    btm->addButton(bt3);
    btm->addButton(bt4);

    mthread* buttonScanThread = mthread::create("button", 2048, 2, 10, [&](void* p){
        mDev::mButtonManager* btm = (mDev::mButtonManager*)mDev::mPlatform::getInstance()->getDevice(DEV_BUTTON_MANAGER);
        while(true)
        {
            btm->process();
            mthread::threadMdelay(20);
        }
    },nullptr);
    buttonScanThread->startup();
    return 0;
}
TASK_EXPORT(buttonInit, "3");