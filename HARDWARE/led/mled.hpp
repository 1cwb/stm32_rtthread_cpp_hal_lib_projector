#pragma once

#include "mdev.hpp"
#include "led.h"

#define LED_BASE_DEVICE_PATH		"/dev/led"
#define LED0_DEVICE_PATH		"/dev/led0"
#define LED_ON 0
#define LED_OFF 1
#define LED_TOGGLE 2
class mled : public mdev::mDev
{
public:
    mled():mDev(LED0_DEVICE_PATH){}
    virtual ~mled(){}
    virtual mResult init() override;
    virtual mResult ioctl(mdev::file_t *filep, int cmd, unsigned long arg) override;
};