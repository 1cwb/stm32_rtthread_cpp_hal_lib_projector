#ifndef __SPI_H__
#define __SPI_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include "sys.h"
#include "delay.h"
int spi4Init();
void spi1Init();
SPI_HandleTypeDef* getSpi4Handler();
SPI_HandleTypeDef* getSpi1Handler();

#ifdef __cplusplus
}
#endif 
#endif