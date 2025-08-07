#include "qmc5883.hpp"
#include "mi2cdrv.hpp"
#include "mdevicemanager.hpp"
#include "project.hpp"

 static QMC5883LCompass* qmc5883l = nullptr;

int qmc5883lInit()
{
    mDev::mI2c* i2c4 = (mDev::mI2c*)mDev::mPlatform::getInstance()->getDevice(DEV_I2C4);
    if(!i2c4)
    {
        return -1;
    }
    qmc5883l = new QMC5883LCompass(DEV_MAG1,i2c4);
    qmc5883l->init();
    return 0;
}
INIT_EXPORT(qmc5883lInit, "1");