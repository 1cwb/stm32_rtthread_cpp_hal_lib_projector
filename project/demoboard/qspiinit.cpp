#include "qspi.hpp"
#include "gpio.hpp"
#include "stdio.h"
#include "project.hpp"
static Qspi* qspix = nullptr;
int qspiInit()
{
    QSPI_HandleTypeDef hqspi;
	hqspi.Instance 	= QUADSPI;    // QSPI外设
	
	/*本例程选择 HCLK 作为QSPI的内核时钟，速度为240M，再经过2分频得到120M驱动时钟，
	  关于 QSPI内核时钟 的设置，请参考 main.c文件里的 SystemClock_Config 函数*/
	// 需要注意的是，当使用内存映射模式时，这里的分频系数不能设置为0！！否则会读取错误
	hqspi.Init.ClockPrescaler 		= 1;									// 时钟分频值，将QSPI内核时钟进行 1+1 分频得到QSPI通信驱动时钟
	hqspi.Init.FifoThreshold 		= 32;								    // FIFO阈值
	hqspi.Init.SampleShifting		= QSPI_SAMPLE_SHIFTING_HALFCYCLE;	    // 半个CLK周期之后进行采样
	hqspi.Init.FlashSize 			= 22;											// flash大小，FLASH 中的字节数 = 2^[FSIZE+1]，核心板采用是8M字节的W25Q64，这里设置为22
	hqspi.Init.ChipSelectHighTime   = QSPI_CS_HIGH_TIME_1_CYCLE;			// 片选保持高电平的时间
	hqspi.Init.ClockMode 			= QSPI_CLOCK_MODE_3;						// 模式3
	hqspi.Init.FlashID 				= QSPI_FLASH_ID_1;						// 使用QSPI1
	hqspi.Init.DualFlash 			= QSPI_DUALFLASH_DISABLE;				// 禁止双闪存模式
    qspix = new Qspi(DEV_QSPI);
    if(!qspix)
    {
        return -1;
    }
    if(qspix->init([](bool binit){
        if(binit)
        {
            __HAL_RCC_QSPI_CLK_ENABLE();		// 使能QSPI时钟

            __HAL_RCC_QSPI_FORCE_RESET();		// 复位QSPI
            __HAL_RCC_QSPI_RELEASE_RESET();
            gpiox qspiclk("qspiclk");
            qspiclk.init([](bool b){if(b)__HAL_RCC_GPIOF_CLK_ENABLE();},GPIOF, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF9_QUADSPI);
            gpiox qspibk1ncs("qspibk1ncs");
            qspibk1ncs.init([](bool b){if(b)__HAL_RCC_GPIOG_CLK_ENABLE();},GPIOG, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF10_QUADSPI);
            gpiox qspibk1io0("qspibk1io0");
            qspibk1io0.init([](bool b){if(b)__HAL_RCC_GPIOF_CLK_ENABLE();},GPIOF, GPIO_PIN_8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF10_QUADSPI);
            gpiox qspibk1io1("qspibk1io1");
            qspibk1io1.init([](bool b){if(b)__HAL_RCC_GPIOF_CLK_ENABLE();},GPIOF, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF10_QUADSPI);
            gpiox qspibk1io2("qspibk1io2");
            qspibk1io2.init([](bool b){if(b)__HAL_RCC_GPIOF_CLK_ENABLE();},GPIOF, GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF9_QUADSPI);
            gpiox qspibk1io3("qspibk1io3");
            qspibk1io3.init([](bool b){if(b)__HAL_RCC_GPIOF_CLK_ENABLE();},GPIOF, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF9_QUADSPI);
        }
    }, &hqspi) != M_RESULT_EOK)
    {
        return -1;
    }
    return 0;
}
INIT_EXPORT(qspiInit, "0.3");