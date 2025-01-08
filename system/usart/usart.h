#ifndef __USART_H__
#define __USART_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "stdio.h"	
#include "sys.h"

extern UART_HandleTypeDef UART1_Handler;
void uart_init(uint32_t bound);

#ifdef __cplusplus
}
#endif
#endif