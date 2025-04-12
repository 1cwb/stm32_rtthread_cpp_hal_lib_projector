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

#if USE_SDRAM
#define SDRAM_MEM_ADDR   (uint8_t*)SDRAM_ORIGIN
#endif

HAL_StatusTypeDef Stm32_Clock_Init();
void hwInit();
void SoftReset(void);
#ifdef __cplusplus
}
#endif
#endif
