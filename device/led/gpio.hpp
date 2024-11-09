#pragma once
#include "mgpiodrv.hpp"
#include "stm32h7xx_hal_conf.h"
#include <functional>
class gpiox : public mDev::mGpio
{
public:
    gpiox(const char* name);
    virtual ~gpiox();
    mResult init(const mDev::initCallbackExt& cb, GPIO_TypeDef * gpiox,
        uint16_t pin, uint32_t mode = GPIO_MODE_OUTPUT_PP, uint32_t pull = GPIO_NOPULL,
        uint32_t speed = GPIO_SPEED_FREQ_HIGH, uint32_t alternate = 0);
    mResult deInit();
    virtual void setLevel(mDev::mGpio::GPIOLEVEL level)override;
    virtual mDev::mGpio::GPIOLEVEL getLevel()override;
    virtual void toggle()override;
    virtual mResult interruptEnable(bool benable)override{return M_RESULT_EOK;}
    GPIO_TypeDef* getGpiox() {return _gpiox;}
    uint16_t getPin() const {return _pin;}
private:
    GPIO_TypeDef * _gpiox;
    uint16_t _pin;
};