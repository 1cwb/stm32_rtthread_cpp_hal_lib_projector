#ifndef __SDRAM_H__
#define __SDRAM_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "sys.h"
//HCLK = 240 
//SDRAM CLOCK = 240/2 = 120
#define SDRAM_BANK_ADDR     ((uint32_t)0xC0000000) 				// FMC SDRAM 数据基地址
#define FMC_COMMAND_TARGET_BANK   FMC_SDRAM_CMD_TARGET_BANK1	//	SDRAM 的bank选择
#define SDRAM_TIMEOUT     ((uint32_t)0x1000) 						// 超时判断时间

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000) 
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200) 

/*--------------------------------------- 函数声明 -------------------------------------*/

HAL_StatusTypeDef	MX_FMC_Init(void);			// SDRAM初始化
HAL_StatusTypeDef SDRAM_Check();
#ifdef __cplusplus
}
#endif 
#endif