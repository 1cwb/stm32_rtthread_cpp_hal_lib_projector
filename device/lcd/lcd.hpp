#pragma once
#include "mdisplaydrv.hpp"
#include "mspidrv.hpp"
#include "mgpiodrv.hpp"
class Lcd169 : public mDev::mDisplay
{
public:
    Lcd169(const char* name, uint16_t width, uint16_t height, uint8_t xOffset, uint8_t yOffset, uint8_t bitPerPixel);
    virtual ~Lcd169();
    void init(mDev::mSpi* spix, mDev::mGpio* dc, mDev::mGpio* rst, mDev::mGpio* bl, mDev::mGpio* cs);
    void backlightlOn() {if(bl){bl->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);}}
    void backlightlOff() {if(bl){bl->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);}}
protected:
    virtual void setAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2) override;
    virtual void writeCommand(uint8_t command) override;
    virtual void writeData8bit(uint8_t data) override;
    virtual void writeData16bit(uint16_t data) override;
    virtual void writeBuff(uint8_t *DataBuff, uint16_t DataSize) override;
    virtual void internelSetDirection(mDev::Direction dir) override;
    void dataEnable() {if(dc){dc->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);}}
    void cmdEnable() {if(dc){dc->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);}}
private:
    mDev::mSpi* spix;
    mDev::mGpio* dc;
    mDev::mGpio* rst;
    mDev::mGpio* bl;
    mDev::mGpio* cs;
};