#ifndef __USART_H__
#define __USART_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "stdio.h"	
#include "sys.h"

#define USART_REC_LEN  			256 
extern uint8_t  USART_RX_BUF[USART_REC_LEN];
extern uint16_t USART_RX_STA; 
extern UART_HandleTypeDef UART1_Handler;

#define RXBUFFERSIZE   1
extern uint8_t aRxBuffer[RXBUFFERSIZE];
void uart_init(uint32_t bound);

#ifdef __cplusplus
}
#endif
#endif