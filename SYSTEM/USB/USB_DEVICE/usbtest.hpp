#pragma once
#if 0
#include "usb_device.h"
#include "usbd_custom_hid_if.hpp"

//接收中断标志。定义位置：usbd_custom_hid_if.c(26) : uint8_t usb_flag = 0;
extern uint8_t usb_flag;
//USB结构体
extern USBD_HandleTypeDef hUsbDeviceFS;
//USB发送接口
void usb_send_hid(uint8_t *report, uint16_t len)
{
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report,len);
}

//USB接收接口
void usb_recv_hid(uint8_t *report, uint16_t len)
{
	/*判断标志位*/
	if(1 == usb_flag)
	{
			if(hUsbDeviceFS.pClassData != NULL)
			{
				memcpy(report, hUsbDeviceFS.pClassData,len);
			}
			/*清空标志位*/
			usb_flag = 0;
	}
}
//发送测试
void usb_send_test(void)
{
	uint8_t s_char[64] = {"USB TEST _SEND"} ;
	usb_send_hid(s_char, sizeof(s_char));
}
//接收测试
void usb_recv_test(void)
{
	uint8_t s_char[64] = {0} ;
	usb_recv_hid(s_char,64);
	usb_send_hid(s_char, 64);
}
#endif