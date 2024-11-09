#include "usbdevice.hpp"
mResult usbDeviceHID::init()
{
    MX_USB_DEVICE_Init();
    return M_RESULT_EOK;
}
mResult usbDeviceHID::send(uint8_t *data, uint16_t len)
{
    if(USBD_CUSTOM_HID_SendReport_FS(data, len) != (uint8_t)USBD_OK)
    {
        return  M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult usbDeviceHID::recv(uint8_t *data, uint16_t len)
{
    USBD_CUSTOM_HID_RecvReport_FS(data, len);
    return M_RESULT_EOK;
}
