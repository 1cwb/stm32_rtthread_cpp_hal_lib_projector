#ifndef __DELAY_H__
#define __DELAY_H__
#ifdef __cplusplus
 extern "C" {
#endif 
#include "sys.h"

void delay_init(uint8_t sysclk);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
#ifdef __cplusplus
}
#endif
#endif