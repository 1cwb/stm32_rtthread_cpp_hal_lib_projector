#ifndef __LED_H__
#define __LED_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "sys.h"

int led0Init();
void led0On();
void led0Off();
void led0Toggle();

int led1Init();
void led1On();
void led1Off();
void led1Toggle();

#ifdef __cplusplus
}
#endif 
#endif