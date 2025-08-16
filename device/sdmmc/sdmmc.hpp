#pragma once
#include "msdmmcdrv.hpp"
#include "sys.h"
#include "gpio.hpp"

class sdmmc : public mDev::mSDMMC
{
public:
    sdmmc() = delete;
    explicit sdmmc(const char* name, uint32_t dmaBuffSize = 1024, mDev::mGpio* detectPin = nullptr);
    virtual ~sdmmc();
    mResult init(const mDev::initCallbackExt& cb ,SD_HandleTypeDef* sdhandle);
    mResult deInit();
    virtual mResult readBlocks(uint8_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)override;
    virtual mResult writeBlocks(uint8_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)override;
    virtual mResult erase(uint32_t StartAddr, uint32_t EndAddr)override;
    virtual mDev::MSDMMC_CARD_STATE getCardState()override;
    virtual void getCardInfo(mDev::MSDMMC_CARD_INFO *CardInfo)override;
    virtual bool isCardDetected()override;
    bool waitSdCardReady();
    uint32_t getNbBlockSize() {return buffSize / BLOCKSIZE;}
    SD_HandleTypeDef* sdmmcHandle() {return &uSdHandle;}
    static sdmmc* GetObjectFromPrivateMember(SD_HandleTypeDef* member_address)
    {
        // 使用模板函数，传入成员指针和地址
        return GetObjectFromMember(&sdmmc::uSdHandle, member_address);
    }
private:
    SD_HandleTypeDef uSdHandle;
    mDev::mGpio* mDetectPin;
};