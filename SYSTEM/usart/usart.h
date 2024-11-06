#ifndef __USART_H__
#define __USART_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "stdio.h"	
#include "sys.h"
#if 0
#define USART_REC_LEN  			256 
extern uint8_t  USART_RX_BUF[USART_REC_LEN];
extern uint16_t USART_RX_STA; 
extern UART_HandleTypeDef UART2_Handler;

#define RXBUFFERSIZE   1
extern uint8_t aRxBuffer[RXBUFFERSIZE];
void uart_init(uint32_t bound);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
#endif
#ifdef __cplusplus
}
#endif
#endif