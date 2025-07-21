#include "qmc5883.hpp"
#include "mi2cdrv.hpp"
#include "mplatform.hpp"
#include "project.hpp"

 static QMC5883LCompass* qmc5883l = nullptr;

int qmc5883lInit()
{
    mDev::mI2c* i2c2 = (mDev::mI2c*)mDev::mPlatform::getInstance()->getDevice(DEV_I2C2);
    if(!i2c2)
    {
        return -1;
    }
    qmc5883l = new QMC5883LCompass(DEV_MAG1,i2c2);
    qmc5883l->init();
    return 0;
}
INIT_EXPORT(qmc5883lInit, "1");