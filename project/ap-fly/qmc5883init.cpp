#include "qmc5883.hpp"
#include "mi2cdrv.hpp"
#include "mplatform.hpp"

 static QMC5883LCompass* qmc5883l = nullptr;

int qmc5883lInit()
{
    mDev::mI2c* i2c4 = (mDev::mI2c*)mDev::mPlatform::getInstance()->getDevice("i2c4");
    if(!i2c4)
    {
        return -1;
    }
    qmc5883l = new QMC5883LCompass("mag1",i2c4);
    qmc5883l->init();
    return 0;
}
INIT_EXPORT(qmc5883lInit, "1");