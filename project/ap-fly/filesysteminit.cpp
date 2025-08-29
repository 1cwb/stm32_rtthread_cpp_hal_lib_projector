#include "ff.h"
#include "fatfsff.hpp"
#include "mklog.hpp"

static mFatFs fs;
int filesysteminit(void)
{
    FRESULT result;
    result = fs.mount("0:/", 0);
    if (result != FR_OK)
    {
        ALOGE("mount 0:/ failed, Reason: (%s)\r\n", mFatFs::errToStr(result));
        return -1;
    }
    //mKlog::getInstance()->saveLogToFile("0:/log.txt");
    return 0;
}
INIT_EXPORT(filesysteminit, "0.3");