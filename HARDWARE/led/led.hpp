#pragma once
#include "mleddrv.hpp"
#include "stm32h7xx_hal_conf.h"
#include <functional>

class ledx : public mDev::mLed
{
public:
    ledx(const char* name):
    mLed(name)
    {
    }
    virtual ~ledx(){}
    mResult init(const mDev::initCallbackExt& cb ,GPIO_TypeDef* gpiox, uint16_t pin, bool highIsoff = true);
    mResult deInit();
    virtual void on()override;
    virtual void off()override;
    virtual void toggle()override;
private:
    GPIO_TypeDef * _gpiox;
    uint16_t _pin;
    bool _highIsoff;
};