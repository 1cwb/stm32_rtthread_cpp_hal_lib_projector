#pragma once
#include "mled.hpp"
#include "stm32h7xx_hal_conf.h"
#include <functional>

class ledx : public mDev::mLed
{
    using rccEnableFunc = std::function<void()>;
public:
    ledx(const char* name, GPIO_TypeDef* gpiox, uint16_t _pin, const rccEnableFunc&& func, bool highIsoff = true):
    mLed(name), _gpiox(gpiox), _pin(_pin), _rccEanbleFunc(func), _highIsoff(highIsoff){}
    virtual ~ledx(){}
    virtual mResult init();
    virtual mResult deInit();
    virtual void on();
    virtual void off();
    virtual void toggle();
private:
    GPIO_TypeDef * _gpiox;
    uint16_t _pin;
    rccEnableFunc _rccEanbleFunc;
    bool _highIsoff;
};