#include "i2c.hpp"
i2cx::i2cx(const char* name, mDev::I2C_TYPE type = mDev::I2C_TYPE_MASTER) : mDev::mI2c(name, type)
{

}
i2cx::~i2cx()
{

}
mResult i2cx::init(const mDev::initCallbackExt& cb ,I2C_HandleTypeDef* i2chandle)
{
    _initcb = cb;
    memcpy(&_i2cxHandle, i2chandle, sizeof(I2C_HandleTypeDef));
    _i2cxHandle.State = HAL_I2C_STATE_RESET;
    if(HAL_I2C_Init(&_i2cxHandle) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult i2cx::deInit()
{
    HAL_I2C_DeInit(&_i2cxHandle);
    return M_RESULT_EOK;
}
mResult i2cx::write(uint16_t slaveAddr, uint8_t* buff, size_t len)
{
    if(isMasterMode())
    {
        if(HAL_I2C_Master_Transmit(&_i2cxHandle, slaveAddr, buff, len, 5000) != HAL_OK)
        {
            printf("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else
    {
        if(HAL_I2C_Slave_Transmit(&_i2cxHandle, buff, len, 5000) != HAL_OK)
        {
            printf("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult i2cx::read(uint16_t slaveAddr, uint8_t* buff, size_t len)
{
    if(isMasterMode())
    {
        if(HAL_I2C_Master_Receive(&_i2cxHandle, slaveAddr, buff, len, 5000) != HAL_OK)
        {
            printf("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else
    {
        if(HAL_I2C_Slave_Receive(&_i2cxHandle, buff, len, 5000) != HAL_OK)
        {
            printf("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult i2cx::writeReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len)
{
    if(HAL_I2C_Mem_Write(&_i2cxHandle, slaveAddr, reg, I2C_MEMADD_SIZE_8BIT, buff, len, 5000) != HAL_OK)
    {
        printf("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult i2cx::readReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len)
{
    if(HAL_I2C_Mem_Read(&_i2cxHandle, slaveAddr, reg, I2C_MEMADD_SIZE_8BIT, buff, len, 5000) != HAL_OK)
    {
        printf("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}

extern "C" void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    i2cx* i2c = containerof(hi2c, i2cx, _i2cxHandle);
    if(hi2c == i2c->i2cxHandle())
    {
        i2c->runInitCallback(true);
    }
}
extern "C" void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    i2cx* i2c = containerof(hi2c, i2cx, _i2cxHandle);
    if(hi2c == i2c->i2cxHandle())
    {
        i2c->runInitCallback(false);
    }
}