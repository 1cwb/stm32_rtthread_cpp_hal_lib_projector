#pragma once
#include "mdevice.hpp"
#include "mipc.hpp"

namespace mDev 
{
class mUsart : public mDevice
{
public:
    using usartData = devCbData<uint8_t*>;
    mUsart(const char* name) : mDev::mDevice(name),_transferMode(transferMode::TRANSFER_MODE_NOMAL),_recvMode(recvMode::RECV_MODE_NOMAL), _transferComplete(true) {_sem.init(getDeviceName(),1,IPC_FLAG_FIFO);}
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
    void setTransferMode(transferMode mode) {_transferMode = mode;}
    void setRecvMode(recvMode mode) {_recvMode = mode;}
    recvMode getRecvMode() const {return _recvMode;}
    bool btransferComplete()  {return _transferComplete;}
    void setTransferComplete(bool bcomplete) {_transferComplete = bcomplete;}
public:
    constexpr static int RX_BUFF_LEN = 64;
protected:
    virtual mResult send(const uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
    virtual mResult recv(uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
    transferMode _transferMode;
    recvMode _recvMode;
    volatile bool _transferComplete;
private:
    mSemaphore _sem;
};
}
