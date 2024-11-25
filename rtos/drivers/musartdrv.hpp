#pragma once
#include "mdevice.hpp"
#include "mipc.hpp"

namespace mDev 
{
enum class transferMode
{
    TRANSFER_MODE_NOMAL,
    TRANSFER_MODE_IT,
    TRANSFER_MODE_DMA,
    TRANSFER_MODE_IT_RECV_IDLE,
    TRANSFER_MODE_DMA_RECV_IDLE
};
class mUsart : public mDevice
{
public:
    using usartData = devCbData<uint8_t*>;
    mUsart(const char* name) : mDev::mDevice(name),_mode(transferMode::TRANSFER_MODE_NOMAL) {_sem.init(getDeviceName(),1,IPC_FLAG_FIFO);}
    virtual ~mUsart() {}
    mResult sendData(const uint8_t* data, uint32_t len)
    {
        _sem.semTake(WAITING_FOREVER);
        if(send(data, len) != M_RESULT_EOK)
        {
            _sem.semRelease();
            return M_RESULT_ERROR;
        }
        _sem.semRelease();
        return M_RESULT_EOK;
    }
    mResult recvData(uint8_t* data, uint32_t len)
    {
        _sem.semTake(WAITING_FOREVER);
        if(recv(data, len) != M_RESULT_EOK)
        {
            _sem.semRelease();
            return M_RESULT_ERROR;
        }
        _sem.semRelease();
        return M_RESULT_EOK;
    }
    void setTransferMode(transferMode mode) {_mode = mode;}
    virtual void* getObj() {return nullptr;}
    virtual void syncDataByAddr(uint32_t *addr, int32_t dsize){}
protected:
    virtual mResult send(const uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
    virtual mResult recv(uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
    transferMode _mode;
private:
    mSemaphore _sem;
};
}
