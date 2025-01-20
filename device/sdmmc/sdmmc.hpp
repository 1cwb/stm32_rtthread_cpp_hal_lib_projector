#pragma once
#include "msdmmcdrv.hpp"
#include "sys.h"
#include "gpio.hpp"

class sdmmc : public mDev::mSDMMC
{
public:
    sdmmc() = delete;
    explicit sdmmc(const char* name, mDev::mGpio* detectPin = nullptr);
    virtual ~sdmmc();
    mResult init(const mDev::initCallbackExt& cb ,SD_HandleTypeDef* sdhandle);
    mResult deInit();
    virtual mResult readBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)override;
    virtual mResult writeBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)override;
    virtual mResult erase(uint32_t StartAddr, uint32_t EndAddr)override;
    virtual mDev::MSDMMC_CARD_STATE getCardState()override;
    virtual void getCardInfo(mDev::MSDMMC_CARD_INFO *CardInfo)override;
    virtual bool isCardDetected()override;
    SD_HandleTypeDef* sdmmcHandle() {return &uSdHandle;}
public:
    SD_HandleTypeDef uSdHandle;
private:
    mDev::mGpio* mDetectPin;
};