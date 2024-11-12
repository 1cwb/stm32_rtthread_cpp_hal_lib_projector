#include "usbdevice.hpp"
mResult usbDeviceHID::init()
{
    MX_USB_DEVICE_Init();
    return M_RESULT_EOK;
}
mResult usbDeviceHID::send(uint8_t *data, uint16_t len)
{
    if(len > CDC_DATA_FS_MAX_PACKET_SIZE)
    {
        len = CDC_DATA_FS_MAX_PACKET_SIZE;
    }
    if(CDC_Transmit_FS(data, len) != USBD_OK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult usbDeviceHID::recv(uint8_t *data, uint16_t len)
{
    
    return M_RESULT_EOK;
}
