#include "sdmmc.hpp"
#include "mklog.hpp"

sdmmc::sdmmc(const char* name, uint32_t dmaBuffSize, mDev::mGpio* detectPin) : mDev::mSDMMC(name, dmaBuffSize), mDetectPin(detectPin)
{
    txBuff =(new alignas(32) uint8_t[dmaBuffSize]);
    rxBuff =(new alignas(32) uint8_t[dmaBuffSize]);
}
sdmmc::~sdmmc()
{

}
mResult sdmmc::init(const mDev::initCallbackExt& cb ,SD_HandleTypeDef* sdhandle)
{
    _initcb = cb;
    if(sdhandle)
    {
        memcpy(&uSdHandle, sdhandle, sizeof(SD_HandleTypeDef));
        HAL_SD_DeInit(&uSdHandle);
    }
    if(!isCardDetected())
    {
        ALOGE("%s()%d isCardDetected Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    /* HAL SD initialization */
    if(HAL_SD_Init(&uSdHandle) != HAL_OK)
    {
        ALOGE("%s()%d HAL_SD_INIT Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    if(getCardState() != mDev::MSDMMC_CARD_STATE::SDMMC_TRANSFER_OK)
    {
        ALOGE("%s()%d getCardState Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult sdmmc::deInit()
{
  /* HAL SD deinitialization */
  if(HAL_SD_DeInit(&uSdHandle) != HAL_OK)
  {
    return M_RESULT_ERROR;
  }
  return M_RESULT_EOK;
}
mResult sdmmc::readBlocks(uint8_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
    if(!waitSdCardReady())
    {
        printf("Error: %s() SDMMC INTERFACE BUSY\r\n",__FUNCTION__);
        return M_RESULT_EBUSY;
    }
    if(_transferMode == mDev::MSDMMC_TRANSFER_MODE::SDMMC_TRANSFER_MODE_DMA)
    {
        if( HAL_SD_ReadBlocks_DMA(&uSdHandle, (uint8_t *)pData, ReadAddr, NumOfBlocks) == HAL_OK)
        {
            return M_RESULT_EOK;
        }
        else
        {
            return M_RESULT_ERROR;
        }
    }
    else
    {
        if (HAL_SD_ReadBlocks(&uSdHandle, (uint8_t *)pData, ReadAddr, NumOfBlocks, 1000) == HAL_OK)
        {
            return M_RESULT_EOK;
        }
        else
        {
            return M_RESULT_ERROR;
        }        
    }
}
mResult sdmmc::writeBlocks(uint8_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
    if(!waitSdCardReady())
    {
        printf("Error: %s() SDMMC INTERFACE BUSY\r\n",__FUNCTION__);
        return M_RESULT_EBUSY;
    }
    if(_transferMode == mDev::MSDMMC_TRANSFER_MODE::SDMMC_TRANSFER_MODE_DMA)
    {
        SCB_CleanDCache_by_Addr((uint32_t*)txBuff, buffSize);
        if( HAL_SD_WriteBlocks_DMA(&uSdHandle, (uint8_t *)pData, WriteAddr, NumOfBlocks) == HAL_OK)
        {
            return M_RESULT_EOK;
        }
        else
        {
            return M_RESULT_ERROR;
        }
    }
    else
    {
        if( HAL_SD_WriteBlocks(&uSdHandle, (uint8_t *)pData, WriteAddr, NumOfBlocks, 1000) == HAL_OK)
        {
            return M_RESULT_EOK;
        }
        else
        {
            return M_RESULT_ERROR;
        }
    }
}
mResult sdmmc::erase(uint32_t StartAddr, uint32_t EndAddr)
{
  if( HAL_SD_Erase(&uSdHandle, StartAddr, EndAddr) == HAL_OK)
  {
    return M_RESULT_EOK;
  }
  else
  {
    return M_RESULT_ERROR;
  }
}
mDev::MSDMMC_CARD_STATE sdmmc::getCardState()
{
    return((HAL_SD_GetCardState(&uSdHandle) == HAL_SD_CARD_TRANSFER ) ? mDev::MSDMMC_CARD_STATE::SDMMC_TRANSFER_OK : mDev::MSDMMC_CARD_STATE::SDMMC_TRANSFER_BUSY);
}
void sdmmc::getCardInfo(mDev::MSDMMC_CARD_INFO *CardInfo)
{
    HAL_SD_CardInfoTypeDef mCardInfo;
    HAL_SD_GetCardInfo(&uSdHandle, &mCardInfo);
    memcpy(CardInfo, &mCardInfo, sizeof(mCardInfo));
}
bool sdmmc::isCardDetected()
{
    if(mDetectPin)
    {
        return mDetectPin->getLevel() == mDev::mGpio::GPIOLEVEL::LEVEL_LOW;
    }
    return true;
}

bool sdmmc::waitSdCardReady()
{
    uint32_t loop = ((uint32_t)0x00100000U);
    
    /* Wait for the Erasing process is completed */
    /* Verify that SD card is ready to use after the Erase */
    while(loop > 0)
    {
        loop--;
        if(HAL_SD_GetCardState(&uSdHandle) == HAL_SD_CARD_TRANSFER)
        {
            return true;
        }
    }
    return false;
}

extern "C" void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    sdmmc* sdx = containerof(hsd, sdmmc, uSdHandle);
    if(hsd == sdx->sdmmcHandle())
    {
        sdx->runInitCallback(true);
    }
}

extern "C" void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
    sdmmc* sdx = containerof(hsd, sdmmc, uSdHandle);
    if(hsd == sdx->sdmmcHandle())
    {
        sdx->runInitCallback(false);
    }
}
