#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include "mpu.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"

static void JumpToApp(void)
{
        uint32_t i=0;
        void (*SysMemBootJump)(void);        /* 声明一个函数指针 */
        __IO uint32_t BootAddr = APP_VTABLE_ADDR; /* STM32H7的系统BootLoader地址 */

        /* 设置所有时钟到默认状态，使用HSI时钟 */
        uart_deinit();
        HAL_RCC_DeInit();

        /* 关闭全局中断 */
        __disable_irq(); 

        /* 关闭滴答定时器，复位到默认值 */
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;

        /* 关闭所有中断，清除所有中断挂起标志 */
        for (i = 0; i < 8; i++)
        {
                NVIC->ICER[i]=0xFFFFFFFF;
                NVIC->ICPR[i]=0xFFFFFFFF;
        }        

        /* 使能全局中断 */
        __enable_irq();

        /* 跳转到系统BootLoader，首地址是MSP，地址+4是复位中断服务程序地址 */
        SysMemBootJump = (void (*)(void)) (*((uint32_t *) (BootAddr + 4)));

        /* 设置主堆栈指针 */
        __set_MSP(*(uint32_t *)BootAddr);
        
        /* 在RTOS工程，这条语句很重要，设置为特权级模式，使用MSP指针 */
        __set_CONTROL(0);

        /* 跳转到系统BootLoader */
        SysMemBootJump(); 

        /* 跳转成功的话，不会执行到这里，用户可以在这里添加代码 */
        while (1)
        {

        }
}

int main(void)
{
    MPU_SetProtection();
    SCB_EnableICache();		// 使能ICache
	SCB_EnableDCache();		// 使能DCache
    HAL_Init(); //初始化 HAL 库
    Stm32_Clock_Init(PLLM_VALUE,PLLN_VALUE,PLLP_VALUE,PLLQ_VALUE,PLLR_VALUE); //设置时钟,480Mhz
    /* System Clock Update */
    SystemCoreClockUpdate();
    delay_init(HAL_RCC_GetSysClockFreq()/1000000);//1US跑的tick数
    uart_init(115200);
    //led_init();
    printf("Uboot runing...\r\n");
    printf("sys clock is %lu\r\n",HAL_RCC_GetSysClockFreq());
    printf("HCLK clock is %lu\r\n",HAL_RCC_GetHCLKFreq());
    //SysTick->CTRL = 0;		// 关闭SysTick
	//SysTick->LOAD = 0;		// 清零重载值
	//SysTick->VAL = 0;			// 清零计数值
    SCB_DisableICache();		// 关闭ICache
	SCB_DisableDCache();		// 关闭Dcache

    JumpToApp();

    return 0;
}
