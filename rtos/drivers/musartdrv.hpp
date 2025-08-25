#pragma once
#include "mdevice.hpp"
#include "mipc.hpp"

namespace mDev 
{
class mUsart : public mDevice
{
public:
    using usartData = devCbData<uint8_t*>;
    mUsart(const char* name) : mDev::mDevice(name),_transferMode(transferMode::TRANSFER_MODE_NOMAL),_recvMode(recvMode::RECV_MODE_NOMAL), _transferComplete(true),_rxIndex(0)
    {
        _rxBuff[0] =(new alignas(32) uint8_t[RX_BUFF_LEN]);
        _rxBuff[1] =(new alignas(32) uint8_t[RX_BUFF_LEN]);
    }
    virtual ~mUsart() {if(_rxBuff[0]) {delete[] _rxBuff[0];}
                        if(_rxBuff[1]) {delete[] _rxBuff[1];}}
    mResult sendData(const uint8_t* data, uint32_t len)
    {
        if(send(data, len) != M_RESULT_EOK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    mResult recvData(uint8_t* data, uint32_t len)
    {
        if(recv(data, len) != M_RESULT_EOK)
        {
            return M_RESULT_ERROR;
        }
        return M_RESULT_EOK;
    }
    void setTransferMode(transferMode mode) {_transferMode = mode;}
    void setRecvMode(recvMode mode) {_recvMode = mode;}
    recvMode getRecvMode() const {return _recvMode;}
    bool btransferComplete()  {return _transferComplete;}
    void setTransferComplete(bool bcomplete) {_transferComplete = bcomplete;}
    uint8_t* getRxBuff() {return _rxBuff[_rxIndex];}
    void switchNextBuff() {if(_rxIndex == 0) _rxIndex = 1; else _rxIndex = 0;}
public:
    constexpr static int RX_BUFF_LEN = 64;
protected:
    virtual mResult send(const uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
    virtual mResult recv(uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
    transferMode _transferMode;
    recvMode _recvMode;
    volatile bool _transferComplete;
    uint8_t _rxIndex; 
    uint8_t* _rxBuff[2];
};
}
