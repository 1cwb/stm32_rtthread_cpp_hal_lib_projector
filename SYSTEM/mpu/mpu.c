#include "mpu.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

uint8_t MPU_SetProtection()
{
    MPU_Region_InitTypeDef MPU_Initure;
    HAL_MPU_Disable();
    MPU_Initure.Enable = MPU_REGION_ENABLE;
    MPU_Initure.Number = MPU_REGION_NUMBER0;
    MPU_Initure.BaseAddress = 0x20000000;
    MPU_Initure.Size = MPU_REGION_SIZE_1MB;
    MPU_Initure.SubRegionDisable = 0x00;
    MPU_Initure.TypeExtField = MPU_TEX_LEVEL1;
    MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_Initure.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_Initure.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_Initure.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_Initure.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_Initure);

    MPU_Initure.Enable = MPU_REGION_ENABLE;
    MPU_Initure.Number = MPU_REGION_NUMBER1;
    MPU_Initure.BaseAddress = 0x90000000;
    MPU_Initure.Size = MPU_REGION_SIZE_8MB;
    MPU_Initure.SubRegionDisable = 0x00;
    MPU_Initure.TypeExtField = MPU_TEX_LEVEL1;
    MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_Initure.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_Initure.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_Initure.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_Initure.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_Initure);

	MPU_Initure.Enable           = MPU_REGION_ENABLE;
	MPU_Initure.BaseAddress      = 0XC0000000;
	MPU_Initure.Size             = MPU_REGION_SIZE_32MB;
	MPU_Initure.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_Initure.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_Initure.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_Initure.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_Initure.Number           = MPU_REGION_NUMBER2;
	MPU_Initure.TypeExtField     = MPU_TEX_LEVEL0;
	MPU_Initure.SubRegionDisable = 0x00;
	MPU_Initure.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_Initure);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    return 0;
}