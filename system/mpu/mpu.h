#ifndef __MPU_H__
#define __MPU_H__
#ifdef __cplusplus
 extern "C" {
#endif 
#include "sys.h"

//设置某个区域的 MPU 保护
//baseaddr:MPU 保护区域的基址(首地址)
//size:MPU 保护区域的大小(必须是 32 的倍数,单位为字节)
//可设置的值参考:CORTEX_MPU_Region_Size
//rnum:MPU 保护区编号,范围:0~7,最大支持 8 个保护区域
//可设置的值参考： CORTEX_MPU_Region_Number
//ap:访问权限,访问关系如下:
//可设置的值参考： CORTEX_MPU_Region_Permission_Attributes
//MPU_REGION_NO_ACCESS,无访问（特权&用户都不可访问）
//MPU_REGION_PRIV_RW,仅支持特权读写访问
//MPU_REGION_PRIV_RW_URO,禁止用户写访问（特权可读写访问）
//MPU_REGION_FULL_ACCESS,全访问（特权&用户都可访问）
//MPU_REGION_PRIV_RO,仅支持特权读访问
//MPU_REGION_PRIV_RO_URO,只读（特权&用户都不可以写）
//详见:STM32H7 Series Cortex-M7 processor programming manual.pdf,4.6 节,Table 89.
//返回值;0,成功.
// 其他,错误
uint8_t MPU_SetProtection();
#ifdef __cplusplus
}
#endif
#endif