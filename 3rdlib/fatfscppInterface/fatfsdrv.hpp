#pragma once
#include "diskio.h"
#include "msdmmcdrv.hpp"

DSTATUS SD_initialize(BYTE lun);
DSTATUS SD_status(BYTE lun);
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count);
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff);
