#include "usbdevice.hpp"

int usbInit()
{
    usbDeviceHID* usbx = new usbDeviceHID;
    usbx->init();
    return 0;
}

INIT_EXPORT(usbInit, "0.2");