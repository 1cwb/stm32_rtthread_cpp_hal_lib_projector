#include "mspi.hpp"
namespace mDev
{
mResult mSpi::writeReg(uint8_t reg, const uint8_t* buff, size_t len)
{
    mResult ret = M_RESULT_ERROR;
    csEnable();
    do{
        ret = write(&reg, 1);
        if(ret != M_RESULT_EOK)
        {
            break;
        }
        ret = write(buff, len);
        if(ret != M_RESULT_EOK)
        {
            break;
        }
    }while(0);
    csDisable();
    return ret;
}
mResult mSpi::readReg(uint8_t reg, uint8_t* buff, size_t len)
{
    mResult ret = M_RESULT_ERROR;
    reg |= 0x80;
    csEnable();
    do{
        ret = write(&reg, 1);
        if(ret != M_RESULT_EOK)
        {
            break;
        }
        ret = read(buff, len);
        if(ret != M_RESULT_EOK)
        {
            break;
        }
    }while(0);
    csDisable();
    return ret;
}
}