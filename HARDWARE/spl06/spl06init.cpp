#include "spl06.hpp"
#include "mi2cdrv.hpp"
#include "mplatform.hpp"

 static ArtronShop_SPL06_001* barometor = nullptr;

int spl06Init()
{
    mDev::mI2c* i2c1 = (mDev::mI2c*)mDev::mPlatform::getInstance()->getDevice("i2c1");
    if(!i2c1)
    {
        return -1;
    }

    barometor = new ArtronShop_SPL06_001("baro1", i2c1);
    barometor->begin();
    return 0;
}
INIT_EXPORT(spl06Init, "1");