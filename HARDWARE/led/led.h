#ifndef __LED_H__
#define __LED_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "sys.h"

void led_init();
void led_on();
void led_off();
#ifdef __cplusplus
}
#endif 
#endif