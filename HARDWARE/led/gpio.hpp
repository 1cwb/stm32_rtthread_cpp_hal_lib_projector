#pragma once
#include "mgpio.hpp"
#include "stm32h7xx_hal_conf.h"
#include <functional>
class gpiox : public mDev::mGpio
{
public:
    gpiox(const char* name, const mDev::initCallbackExt initcb, GPIO_TypeDef * gpiox,
        uint16_t pin, uint32_t mode = GPIO_MODE_OUTPUT_PP, uint32_t pull = GPIO_NOPULL,
        uint32_t speed = GPIO_SPEED_FREQ_HIGH, uint32_t alternate = 0);
    virtual ~gpiox();
    virtual mResult init();
    virtual mResult deInit();
    virtual void setLevel(mDev::mGpio::GPIOLEVEL level);
    virtual mDev::mGpio::GPIOLEVEL getLevel();
    virtual void toggle();
    virtual mResult interruptEnable(bool benable){return M_RESULT_EOK;}
    GPIO_TypeDef* getGpiox() {return _gpiox;}
    uint16_t getPin() const {return _pin;}
private:
    GPIO_TypeDef * _gpiox;
    uint16_t _pin;
};