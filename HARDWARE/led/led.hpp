#pragma once
#include "mled.hpp"
#include "stm32h7xx_hal_conf.h"
#include <functional>

class ledx : public mDev::mLed
{
public:
    ledx(const char* name, GPIO_TypeDef* gpiox, uint16_t _pin, const mDev::initCallbackExt initcb, bool highIsoff = true):
    mLed(name, initcb), _gpiox(gpiox), _pin(_pin), _highIsoff(highIsoff){}
    virtual ~ledx(){}
    virtual mResult init();
    virtual mResult deInit();
    virtual void on();
    virtual void off();
    virtual void toggle();
private:
    GPIO_TypeDef * _gpiox;
    uint16_t _pin;
    bool _highIsoff;
};