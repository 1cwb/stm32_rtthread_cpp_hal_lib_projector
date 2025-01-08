#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdio.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define SYSTEM_SUPPORT_OS   0

#define DTCM_MEM   __attribute__((section(".dtcmram")))
#define DTCM_MEM_ALIGN(x)   __attribute__((section(".dtcmram"))) __attribute__((aligned(x)))

#define D2_MEM   __attribute__((section(".ramd2")))
#define D2_MEM_ALIGN(x)   __attribute__((section(".ramd2"))) __attribute__((aligned(x)))

#define D3_MEM   __attribute__((section(".ramd3")))
#define D3_MEM_ALIGN(x)   __attribute__((section(".ramd3"))) __attribute__((aligned(x)))

HAL_StatusTypeDef Stm32_Clock_Init(uint32_t pllm, uint32_t plln, uint32_t pllp, uint32_t pllq, uint32_t pllr);

#ifdef __cplusplus
}
#endif
#endif
