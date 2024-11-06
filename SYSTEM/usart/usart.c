#if 0
#include "usart.h"
#include "delay.h"
#include <string.h>
#include "sys.h"
int _read (int fd, char *pBuffer, int size)  
{  
    for (int i = 0; i < size; i++)  
    {  
        while((USART2->ISR&(1 << 5))==0);          //等待上一次串口数据发送完成  
        USART2->RDR = (uint8_t) pBuffer[i];    //写DR,串口1将发送数据
    }
    return size;
}
int _write (int fd, char *pBuffer, int size)  
{
    for (int i = 0; i < size; i++)  
    {  
        while((USART2->ISR&(1 << 6))==0);    //等待上一次串口数据发送完成  
        USART2->TDR = (uint8_t) pBuffer[i];    //写DR,串口1将发送数据
    }
    return size;
}

uint8_t USART_RX_BUF[USART_REC_LEN];
uint16_t USART_RX_STA=0;
uint8_t aRxBuffer[RXBUFFERSIZE];
UART_HandleTypeDef UART2_Handler;

void uart_init(uint32_t bound)
{
	UART2_Handler.Instance=USART2;					  
	UART2_Handler.Init.BaudRate=bound;				  
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B; 
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	  
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		  
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE; 
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;	

	GPIO_InitTypeDef GPIO_Initure;
	
	//if(huart->Instance==USART2)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();		
		__HAL_RCC_USART2_CLK_ENABLE();		
	
		GPIO_Initure.Pin=GPIO_PIN_5;		
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;	
		GPIO_Initure.Pull=GPIO_PULLUP;		
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;	
		GPIO_Initure.Alternate=GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD,&GPIO_Initure);	   

		GPIO_Initure.Pin=GPIO_PIN_6;		
		HAL_GPIO_Init(GPIOD,&GPIO_Initure);	   

		HAL_NVIC_EnableIRQ(USART2_IRQn);
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);
	}

	HAL_UART_Init(&UART2_Handler);					  
	
	HAL_UART_Receive_IT(&UART2_Handler, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
}
#if 0
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{

}
#endif
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
		if((USART_RX_STA&0x8000)==0)
		{
			if(USART_RX_STA&0x4000)
			{
				if(aRxBuffer[0]!=0x0a)
				{
					USART_RX_STA=0;
				}
				else USART_RX_STA|=0x8000;
			}
			else
			{	
				if(aRxBuffer[0]==0x0d){USART_RX_STA|=0x4000;HAL_UART_Transmit(huart,(const uint8_t*)"hello world\r\n",strlen("hello world\r\n"),2000);SoftReset();}
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;	  
				}		 
			}
		}
	}
}
void USART2_IRQHandler(void)                	
{ 
	uint32_t timeout=0;
	HAL_UART_IRQHandler(&UART2_Handler);
	
	timeout=0;
    while (HAL_UART_GetState(&UART2_Handler) != HAL_UART_STATE_READY)
	{
	 timeout++;
     if(timeout>HAL_MAX_DELAY) break;		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART2_Handler, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	{
	 timeout++; 
	 if(timeout>HAL_MAX_DELAY) break;	
	}
} 

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
uint8_t data_to_send[50] D2_MEM_ALIGN(4);
//uint8_t data_to_send[50];
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	int16_t _temp;
	int32_t _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit(&UART2_Handler,data_to_send,_cnt,2000);
}
#endif