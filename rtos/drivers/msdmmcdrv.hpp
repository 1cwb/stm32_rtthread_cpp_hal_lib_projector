#pragma once
#include "mdevice.hpp"
namespace mDev
{
enum class MSDMMC_CARD_STATE
{
  SDMMC_TRANSFER_OK        =       ((uint8_t)0x00),
  SDMMC_TRANSFER_BUSY      =       ((uint8_t)0x01),
};

struct MSDMMC_CARD_INFO
{
  uint32_t cardType;                     /*!< Specifies the card Type                         */
  uint32_t cardVersion;                  /*!< Specifies the card version                      */
  uint32_t classes;                        /*!< Specifies the class of the card class           */
  uint32_t relCardAdd;                   /*!< Specifies the Relative Card Address             */
  uint32_t blockNbr;                     /*!< Specifies the Card Capacity in blocks           */
  uint32_t blockSize;                    /*!< Specifies one block size in bytes               */
  uint32_t logBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */
  uint32_t logBlockSize;                 /*!< Specifies logical block size in bytes           */
  uint32_t cardSpeed;                    /*!< Specifies the card Speed                        */
};

enum class MSDMMC_IRQ_TYPE
{
    SDMMC_IRQ_ABORT,
    SDMMC_IRQ_TX_COMPLETE,
    SDMMC_IRQ_RX_COMPLETE,
    SDMMC_IRQ_ERROR,
    SDMMC_IRQ_TX_1V8_MODE_RESET,
    SDMMC_IRQ_TX_1V8_MODE_SET
};

enum class MSDMMC_TRANSFER_MODE
{
    SDMMC_TRANSFER_MODE_NORMAL,
    SDMMC_TRANSFER_MODE_DMA,
};

using MSDMMCdata = devCbData<uint8_t*, MSDMMC_IRQ_TYPE>;

class mSDMMC : public mDevice
{
public:
    explicit mSDMMC(const char* name) : mDevice(name){}
    virtual ~mSDMMC() = default;
    virtual mResult readBlocks(uint8_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks) {return M_RESULT_EOK;}
    virtual mResult writeBlocks(uint8_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks){return M_RESULT_EOK;}
    virtual mResult erase(uint32_t StartAddr, uint32_t EndAddr){return M_RESULT_EOK;}
    virtual mDev::MSDMMC_CARD_STATE getCardState(){return MSDMMC_CARD_STATE::SDMMC_TRANSFER_OK;}
    virtual void getCardInfo(mDev::MSDMMC_CARD_INFO *CardInfo){}
    virtual bool isCardDetected(){return false;}

    virtual void setTransferMode(MSDMMC_TRANSFER_MODE mode) {_transferMode = mode;}
    virtual MSDMMC_TRANSFER_MODE getTransferMode() const {return _transferMode;}
protected:
    MSDMMC_TRANSFER_MODE _transferMode = MSDMMC_TRANSFER_MODE::SDMMC_TRANSFER_MODE_NORMAL;
};
}