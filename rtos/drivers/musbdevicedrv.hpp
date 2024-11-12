#pragma once
#include "mplatform.hpp"
#include "mdevice.hpp"

namespace mDev
{
class mUsbHidDevice : public mDevice
{
public:
    using usbData = devCbData<uint8_t*>;
    mUsbHidDevice(const char* name) : mDevice(name) {}
    virtual ~mUsbHidDevice() {}
    virtual mResult send(uint8_t *data, uint16_t len){return M_RESULT_EOK;}
    virtual mResult recv(uint8_t *data, uint16_t len){return M_RESULT_EOK;}
private:

};
}