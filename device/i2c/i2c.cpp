#include "i2c.hpp"
#include "mklog.hpp"

i2cx::i2cx(const char* name, mDev::I2C_TYPE type) : mDev::mI2c(name, type)
{

}
i2cx::~i2cx()
{

}
mResult i2cx::init(const mDev::initCallbackExt& cb ,I2C_HandleTypeDef* i2chandle, bool enableIsr, bool enableDma)
{
    _benableDMA = enableDma;
    _benableISR = enableIsr;
    _initcb = cb;
    memcpy(&_i2cxHandle, i2chandle, sizeof(I2C_HandleTypeDef));
    _i2cxHandle.State = HAL_I2C_STATE_RESET;
    if(HAL_I2C_Init(&_i2cxHandle) != HAL_OK)
    {
        KLOGE("Error: %s()%d i2c init fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    HAL_I2CEx_ConfigAnalogFilter(&_i2cxHandle, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&_i2cxHandle, 0);
    return M_RESULT_EOK;
}
mResult i2cx::deInit()
{
    HAL_I2C_DeInit(&_i2cxHandle);
    return M_RESULT_EOK;
}
mResult i2cx::_write(uint16_t slaveAddr, const uint8_t* buff, size_t len)
{
    if(_benableDMA)
    {
        if(isMasterMode())
        {
            if(HAL_I2C_Master_Transmit_DMA(&_i2cxHandle, slaveAddr << 1, (uint8_t*)buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
        else
        {
            if(HAL_I2C_Slave_Transmit_DMA(&_i2cxHandle, (uint8_t*)buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
    }
    else if(_benableISR)
    {
        if(isMasterMode())
        {
            if(HAL_I2C_Master_Transmit_IT(&_i2cxHandle, slaveAddr << 1, (uint8_t*)buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
        else
        {
            if(HAL_I2C_Slave_Transmit_IT(&_i2cxHandle, (uint8_t*)buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
    }
    else
    {
        if(isMasterMode())
        {
            if(HAL_I2C_Master_Transmit(&_i2cxHandle, slaveAddr << 1, (uint8_t*)buff, len, 5000) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
        else
        {
            if(HAL_I2C_Slave_Transmit(&_i2cxHandle, (uint8_t*)buff, len, 5000) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
    }
    return M_RESULT_EOK;
}
mResult i2cx::_read(uint16_t slaveAddr, uint8_t* buff, size_t len)
{
    if(_benableDMA)
    {
        if(isMasterMode())
        {
            if(HAL_I2C_Master_Receive_DMA(&_i2cxHandle, slaveAddr << 1, buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
        else
        {
            if(HAL_I2C_Slave_Receive_DMA(&_i2cxHandle, buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
    }
    else if(_benableISR)
    {
        if(isMasterMode())
        {
            if(HAL_I2C_Master_Receive_IT(&_i2cxHandle, slaveAddr << 1, buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
        else
        {
            if(HAL_I2C_Slave_Receive_IT(&_i2cxHandle, buff, len) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
    }
    else
    {
        if(isMasterMode())
        {
            if(HAL_I2C_Master_Receive(&_i2cxHandle, slaveAddr << 1, buff, len, 5000) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
        else
        {
            if(HAL_I2C_Slave_Receive(&_i2cxHandle, buff, len, 5000) != HAL_OK)
            {
                KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
                return M_RESULT_ERROR;
            }
        }
    }
    return M_RESULT_EOK;
}
mResult i2cx::_writeReg(uint16_t slaveAddr, uint8_t reg, const uint8_t* buff, size_t len)
{
    if(_benableDMA)
    {
        if(HAL_I2C_Mem_Write_DMA(&_i2cxHandle, slaveAddr << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buff, len) != HAL_OK)
        {
            KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if(_benableISR)
    {
        if(HAL_I2C_Mem_Write_IT(&_i2cxHandle, slaveAddr << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buff, len) != HAL_OK)
        {
            KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else
    {
        if(HAL_I2C_Mem_Write(&_i2cxHandle, slaveAddr << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buff, len, 5000) != HAL_OK)
        {
            KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult i2cx::_readReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len)
{
    if(_benableDMA)
    {
        if(HAL_I2C_Mem_Read_DMA(&_i2cxHandle, slaveAddr << 1, reg, I2C_MEMADD_SIZE_8BIT, buff, len) != HAL_OK)
        {
            KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if(_benableISR)
    {
        if(HAL_I2C_Mem_Read_IT(&_i2cxHandle, slaveAddr << 1, reg, I2C_MEMADD_SIZE_8BIT, buff, len) != HAL_OK)
        {
            KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else
    {
        if(HAL_I2C_Mem_Read(&_i2cxHandle, slaveAddr << 1, reg, I2C_MEMADD_SIZE_8BIT, buff, len, 5000) != HAL_OK)
        {
            KLOGE("Error %s()%d  Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}

extern "C" void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    i2cx* i2c = i2cx::GetObjectFromPrivateMember(hi2c);
    if(hi2c == i2c->i2cxHandle())
    {
        i2c->runInitCallback(true);
    }
}
extern "C" void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    i2cx* i2c = i2cx::GetObjectFromPrivateMember(hi2c);
    if(hi2c == i2c->i2cxHandle())
    {
        i2c->runInitCallback(false);
    }
}