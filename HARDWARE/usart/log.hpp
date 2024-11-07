#pragma once
#include "usartx.hpp"
#include "mlogdrv.hpp"

class logx : public mDev::mLog
{
public:
    logx(const char* name, usart* uartx) : mDev::mLog(name), _uartx(uartx)
    {
    }
    virtual ~logx(){}
    virtual mResult send(const uint8_t* data, uint32_t len) override
    {
        if(!_uartx)
        {
            return M_RESULT_EINVAL;
        }
        while(!_uartx->btransferComplete());
        _uartx->setTransferComplete(false);

        if(_uartx->sendData(data, len) != M_RESULT_EOK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    virtual void syncDataByAddr(uint32_t* addr, uint32_t len) override
    {
        if(_uartx)
        {
            _uartx->syncDataByAddr(addr, len);
        }
    }
private:
    usart* _uartx;
}; 