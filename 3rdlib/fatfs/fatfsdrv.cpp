#include "fatfsdrv.hpp"

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
     DRESULT res = RES_ERROR;
     uint32_t timeout;
     ReadStatus = 0;

    uint8_t ret;
    unsigned int i;

    for (i = 0; i < count; i++) 
    { 

        ret = sd0->readBlocks(sd0->); BSP_SD_ReadBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);

        if(ret == MSD_OK)
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
                    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                    {
                        #if (ENABLE_SD_DMA_CACHE_MAINTENANCE_READ == 1)
                            SCB_CleanInvalidateDCache();
                        #endif
                        
                        memcpy(buff, scratch, BLOCKSIZE);
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
    if ((i == count) && (ret == MSD_OK))
    {
    res = RES_OK;       
    }
     return res;
 }
 
 /**
   * @brief  Writes Sector(s)
   * @param  lun : not used
   * @param  *buff: Data to be written
   * @param  sector: Sector address (LBA)
   * @param  count: Number of sectors to write (1..128)
   * @retval DRESULT: Operation result
   */
 #if _USE_WRITE == 1
 DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
 {
     DRESULT res = RES_ERROR;
     uint32_t timeout;
     WriteStatus = 0;
 
 #if (ENABLE_SD_DMA_CACHE_MAINTENANCE_WRITE == 1)
    SCB_CleanInvalidateDCache();
 #endif
 
     if (!((uint32_t)buff & 0x3))
     {
         if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
                                 (uint32_t)(sector),
                                 count) == MSD_OK)
         {
             /* Wait that writing process is completed or a timeout occurs */
             timeout = HAL_GetTick();
             while((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
             {
             }
             
             /* incase of a timeout return error */
             if (WriteStatus == 0)
             {
                 res = RES_ERROR;
             }
             else
             {
                 WriteStatus = 0;
                 timeout = HAL_GetTick();
 
                 while((HAL_GetTick() - timeout) < SD_TIMEOUT)
                 {
                     if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
                     {
                         res = RES_OK;
                         break;
                     }
                 }
             }
         }
     }
     else
     {
         int i;
         uint8_t ret;
         
         for (i = 0; i < count; i++)
         {
             WriteStatus = 0;
             
             memcpy((void *)scratch, (void *)buff, BLOCKSIZE);
             buff += BLOCKSIZE;
 
             ret = BSP_SD_WriteBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
             if(ret == MSD_OK)
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
                         if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
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
 
         if ((i == count) && (ret == MSD_OK))
         {
             res = RES_OK;           
         }
     }
 
     return res;
 }
 #endif /* _USE_WRITE == 1 */
 
 /**
   * @brief  I/O control operation
   * @param  lun : not used
   * @param  cmd: Control code
   * @param  *buff: Buffer to send/receive control data
   * @retval DRESULT: Operation result
   */
 #if _USE_IOCTL == 1
 DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
 {
   DRESULT res = RES_ERROR;
   BSP_SD_CardInfo CardInfo;
 
   if (Stat & STA_NOINIT) return RES_NOTRDY;
 
   switch (cmd)
   {
   /* Make sure that no pending write process */
   case CTRL_SYNC :
     res = RES_OK;
     break;
 
   /* Get number of sectors on the disk (DWORD) */
   case GET_SECTOR_COUNT :
     BSP_SD_GetCardInfo(&CardInfo);
     *(DWORD*)buff = CardInfo.LogBlockNbr;
     res = RES_OK;
     break;
 
   /* Get R/W sector size (WORD) */
   case GET_SECTOR_SIZE :
     BSP_SD_GetCardInfo(&CardInfo);
     *(WORD*)buff = CardInfo.LogBlockSize;
     res = RES_OK;
     break;
 
   /* Get erase block size in unit of sector (DWORD) */
   case GET_BLOCK_SIZE :
     BSP_SD_GetCardInfo(&CardInfo);
     *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
     res = RES_OK;
     break;
 
   default:
     res = RES_PARERR;
   }
 
   return res;
 }
 #endif /* _USE_IOCTL == 1 */