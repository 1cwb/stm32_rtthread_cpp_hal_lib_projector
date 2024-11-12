#pragma once
#include "musbdevicedrv.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.hpp"

#define USB_HID_DEVICE  "Vcom"

class usbDeviceHID : public mDev::mUsbHidDevice
{
public:
    usbDeviceHID(const char* name = USB_HID_DEVICE) : mDev::mUsbHidDevice(name)
    {}
    virtual ~usbDeviceHID() {}
    virtual mResult send(uint8_t *data, uint16_t len) override;
    virtual mResult recv(uint8_t *data, uint16_t len)override;
    mResult init();
private:

};