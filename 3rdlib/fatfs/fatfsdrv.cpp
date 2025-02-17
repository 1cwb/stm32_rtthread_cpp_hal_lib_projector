#include "fatfsdrv.hpp"
#include "sys.h"

#define SD_TIMEOUT 5 * 1000
#define SD_DEFAULT_BLOCK_SIZE 512

static volatile DSTATUS Stat = STA_NOINIT;
static volatile  UINT  WriteStatus = 0, ReadStatus = 0;

static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);

DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
DRESULT SD_ioctl (BYTE, BYTE, void*);

static mDev::mSDMMC* sd0 = nullptr; 
static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if(sd0->getCardState() == mDev::MSDMMC_CARD_STATE::SDMMC_TRANSFER_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
 DSTATUS SD_initialize(BYTE lun)
 {
    sd0 = (mDev::mSDMMC*)mDev::mPlatform::getInstance()->getDevice("sd0");
   if(sd0)
   {
     Stat = SD_CheckStatus(lun);
   }
       sd0->registerInterruptCb([](mDev::mDevice* dev, void* p){
        mDev::MSDMMCdata* data = (mDev::MSDMMCdata*)p;
        switch(data->type)
        {
            case mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_ABORT:
                printf("abort\r\n");
                break;
            case mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_TX_COMPLETE:
            WriteStatus = 1;
                printf("tx complete\r\n");
                break;
            case mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_RX_COMPLETE:
            ReadStatus = 1;
                printf("rx complete\r\n");
                break;
            case mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_ERROR:
                printf("error\r\n");
                break;
            case mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_TX_1V8_MODE_SET:
                printf("tx 1.8v set\r\n");
                break;
            case mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_TX_1V8_MODE_RESET:
                printf("tx 1.8v reset\r\n");
                break;
            default:
                break;
        }
    });
   return Stat;
 }

 /**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
 DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
 {
     mResult ret = M_RESULT_ERROR;
     uint32_t timeout;
     ReadStatus = 0;

    unsigned int i;

    for (i = 0; i < count; i++) 
    { 

        ret = sd0->readBlocks(sd0->getRxBuff(), (uint32_t)sector++, 1);

        if(ret == M_RESULT_EOK)
        {
            /* Wait that the reading process is completed or a timeout occurs */
            timeout = HAL_GetTick();
            while((ReadStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
            {
                
            }
            /* incase of a timeout return error */
            if (ReadStatus == 0)
            {
                break;
            }
            else
            {
                ReadStatus = 0;
                timeout = HAL_GetTick();

                while((HAL_GetTick() - timeout) < SD_TIMEOUT)
                {
                    if (sd0->getCardState() == mDev::MSDMMC_CARD_STATE::SDMMC_TRANSFER_OK)
                    {
                        memcpy(buff, sd0->getRxBuff(), BLOCKSIZE);
                        buff += BLOCKSIZE;
                        
                        break;
                    }
                }
            }
        }
        else
        {
            break;
        }
    }
    if ((i == count) && (ret == M_RESULT_EOK))
    {
      return RES_OK;
    }
     return RES_ERROR;
 }
 
 /**
   * @brief  Writes Sector(s)
   * @param  lun : not used
   * @param  *buff: Data to be written
   * @param  sector: Sector address (LBA)
   * @param  count: Number of sectors to write (1..128)
   * @retval DRESULT: Operation result
   */
 DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
 {
    mResult ret = M_RESULT_ERROR;
    uint32_t timeout;
    WriteStatus = 0;
    uint32_t i;
    
    for (i = 0; i < count; i++)
    {
        WriteStatus = 0;
        
        memcpy((void *)sd0->getTxBuff(), (void *)buff, BLOCKSIZE);
        buff += BLOCKSIZE;

        ret = sd0->writeBlocks(sd0->getTxBuff(), (uint32_t)sector++, 1);
        if(ret == M_RESULT_EOK)
        {
            /* Wait that writing process is completed or a timeout occurs */

            timeout = HAL_GetTick();
            while((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
            {
            }
            
            /* incase of a timeout return error */
            if (WriteStatus == 0)
            {
                break;
            }
            else
            {
                WriteStatus = 0;
                timeout = HAL_GetTick();

                while((HAL_GetTick() - timeout) < SD_TIMEOUT)
                {
                    if (sd0->getCardState() == mDev::MSDMMC_CARD_STATE::SDMMC_TRANSFER_OK)
                    {
                        break;
                    }
                }
            }
        }
        else
        {
            break;
        }
    }

    if ((i == count) && (ret == M_RESULT_EOK))
    {
        return RES_OK;           
    }
 
     return RES_ERROR;
 }
 
 /**
   * @brief  I/O control operation
   * @param  lun : not used
   * @param  cmd: Control code
   * @param  *buff: Buffer to send/receive control data
   * @retval DRESULT: Operation result
   */
 DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
 {
   DRESULT res = RES_ERROR;
   mDev::MSDMMC_CARD_INFO CardInfo;
 
   if (Stat & STA_NOINIT) return RES_NOTRDY;
 
   switch (cmd)
   {
   /* Make sure that no pending write process */
   case CTRL_SYNC :
     res = RES_OK;
     break;
 
   /* Get number of sectors on the disk (DWORD) */
   case GET_SECTOR_COUNT :
     sd0->getCardInfo(&CardInfo);
     *(DWORD*)buff = CardInfo.logBlockNbr;
     res = RES_OK;
     break;
 
   /* Get R/W sector size (WORD) */
   case GET_SECTOR_SIZE :
     sd0->getCardInfo(&CardInfo);
     *(WORD*)buff = CardInfo.logBlockSize;
     res = RES_OK;
     break;
 
   /* Get erase block size in unit of sector (DWORD) */
   case GET_BLOCK_SIZE :
     sd0->getCardInfo(&CardInfo);
     *(DWORD*)buff = CardInfo.logBlockSize / SD_DEFAULT_BLOCK_SIZE;
     res = RES_OK;
     break;
 
   default:
     res = RES_PARERR;
   }
 
   return res;
 }
