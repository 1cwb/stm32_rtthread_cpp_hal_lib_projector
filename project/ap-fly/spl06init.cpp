#include "spl06.hpp"
#include "mi2cdrv.hpp"
#include "mdevicemanager.hpp"
#include "project.hpp"

static ArtronShop_SPL06_001* barometor = nullptr;

int spl06Init()
{
    mDev::mI2c* i2c1 = (mDev::mI2c*)mDev::mPlatform::getInstance()->getDevice(DEV_I2C1);
    if(!i2c1)
    {
        return -1;
    }

    barometor = new ArtronShop_SPL06_001(DEV_BARO1, i2c1);
    barometor->begin();
    return 0;
}
INIT_EXPORT(spl06Init, "1");