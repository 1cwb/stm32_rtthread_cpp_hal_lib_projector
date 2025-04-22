#include "sys.h"
#include "mpu.h"
#include "delay.h"
#include "usart.h"
#if USE_SDRAM 
#include "sdram.h"
#endif
/**
 * @brief       时钟设置函数
 * @param       plln: PLL1倍频系数(PLL倍频), 取值范围: 4~512.
 * @param       pllm: PLL1预分频系数(进PLL之前的分频), 取值范围: 2~63.
 * @param       pllp: PLL1的p分频系数(PLL之后的分频), 分频后作为系统时钟, 取值范围: 
*                       2~128.(且必 须是2的倍数)
 * @param       pllq: PLL1的q分频系数(PLL之后的分频), 取值范围: 1~128.
 * @note
 *
 *              Fvco: VCO频率
 *              Fsys: 系统时钟频率, 也是PLL1的p分频输出时钟频率
 *              Fq:   PLL1的q分频输出时钟频率
 *              Fs:   PLL输入时钟频率, 可以是HSI, CSI, HSE等.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllp = Fs * (plln / (pllm * pllp));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              外部晶振为25M的时候, 推荐值:plln = 192, pllm = 5, pllp = 2, pllq = 4.
 *              外部晶振为 8M的时候, 推荐值:plln = 240, pllm = 2, pllp = 2, pllq = 4.
 *              得到:Fvco = 8 * (240 / 2) = 960Mhz
 *                   Fsys = pll1_p_ck = 960 / 2 = 480Mhz
 *                   Fq   = pll1_q_ck = 960 / 4 = 240Mhz
 *
 *              H750默认需要配置的频率如下:
 *              CPU频率(rcc_c_ck) = sys_d1cpre_ck = 480Mhz
 *              rcc_aclk = rcc_hclk3 = 240Mhz
 *              AHB1/2/3/4(rcc_hclk1/2/3/4) = 240Mhz
 *              APB1/2/3/4(rcc_pclk1/2/3/4) = 120Mhz
 *              pll2_p_ck = (8 / 8) * 440 / 2) = 220Mhz
 *              pll2_r_ck = FMC时钟频率 = ((8 / 8) * 440 / 2) = 220Mhz
 *
 * @retval    错误代码: 0, 成功; 1, 错误;
 */

HAL_StatusTypeDef Stm32_Clock_Init()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = PLLM_VALUE;
    RCC_OscInitStruct.PLL.PLLN = PLLN_VALUE;
    RCC_OscInitStruct.PLL.PLLP = PLLP_VALUE;
    RCC_OscInitStruct.PLL.PLLQ = PLLQ_VALUE;
    RCC_OscInitStruct.PLL.PLLR = PLLR_VALUE;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        return HAL_ERROR;
    }

      /*
    * 配置 PLL2 的 R 分频输出, 为 220Mhz
    * 配置 FMC 时钟源是 PLL2R
    * 配置 QSPI 时钟源是 PLL2R
    * 配置串口 1 和 串口 6 的时钟源来自: PCLK2 = 120Mhz
    * 配置串口 2 / 3 / 4 / 5 / 7 / 8 的时钟源来自: PCLK1 = 120Mhz
    * USB 工作需要 48MHz 的时钟,可以由 PLL1Q,PLL3Q 和 HSI48 提供,这里配置时钟源是 HSI48
    */
    PeriphClkInitStruct.PeriphClockSelection =  RCC_PERIPHCLK_QSPI |RCC_PERIPHCLK_FMC | \
                                                RCC_PERIPHCLK_USART16 | RCC_PERIPHCLK_USART234578 \
                                               | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USB \
                                               | RCC_PERIPHCLK_SPI1 | RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_ADC;	   	// 设置时钟
    PeriphClkInitStruct.PLL2.PLL2M = PLL2M_VALUE;
    PeriphClkInitStruct.PLL2.PLL2N = PLL2N_VALUE;
    PeriphClkInitStruct.PLL2.PLL2P = PLL2P_VALUE;
    PeriphClkInitStruct.PLL2.PLL2Q = PLL2Q_VALUE;
    PeriphClkInitStruct.PLL2.PLL2R = PLL2R_VALUE;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
    PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI1CLKSOURCE_CLKP;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2; //100MHZ
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_PWREx_EnableUSBVoltageDetector(); /* 使能 USB 电压电平检测器 */
    __HAL_RCC_CSI_ENABLE() ; /* 使能 CSI 时钟 */
    __HAL_RCC_SYSCFG_CLK_ENABLE() ; /* 使能 SYSCFG 时钟 */
    HAL_EnableCompensationCell(); /* 使能 IO 补偿单元 */

    HAL_SYSTICK_Config(HAL_RCC_GetSysClockFreq()/THREAD_TICK_PER_SECOND);//systick时钟默认使用HCLK，可以手动设置为AHB/8，1ms 进一次中断
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    return HAL_OK;
}

void printAllClk()
{
    PLL1_ClocksTypeDef PLL1_Clocks;
    PLL2_ClocksTypeDef PLL2_Clocks;
    PLL3_ClocksTypeDef PLL3_Clocks;

    HAL_RCCEx_GetPLL1ClockFreq(&PLL1_Clocks);
    HAL_RCCEx_GetPLL2ClockFreq(&PLL2_Clocks);
    HAL_RCCEx_GetPLL3ClockFreq(&PLL3_Clocks);

    printf("PLL1_Clocks.PLL1P = %lu\r\n",PLL1_Clocks.PLL1_P_Frequency);
    printf("PLL1_Clocks.PLL1Q = %lu\r\n",PLL1_Clocks.PLL1_Q_Frequency);
    printf("PLL1_Clocks.PLL1R = %lu\r\n",PLL1_Clocks.PLL1_R_Frequency);
    printf("PLL2_Clocks.PLL2P = %lu\r\n",PLL2_Clocks.PLL2_P_Frequency);
    printf("PLL2_Clocks.PLL2Q = %lu\r\n",PLL2_Clocks.PLL2_Q_Frequency);
    printf("PLL2_Clocks.PLL2R = %lu\r\n",PLL2_Clocks.PLL2_R_Frequency);
    printf("PLL3_Clocks.PLL3P = %lu\r\n",PLL3_Clocks.PLL3_P_Frequency);
    printf("PLL3_Clocks.PLL3Q = %lu\r\n",PLL3_Clocks.PLL3_Q_Frequency);
    printf("PLL3_Clocks.PLL3R = %lu\r\n",PLL3_Clocks.PLL3_R_Frequency);
    printf("sys clock is %lu\r\n",HAL_RCC_GetSysClockFreq());
    printf("HCLK clock is %lu\r\n",HAL_RCC_GetHCLKFreq());
    printf("PCLK1 clock is %lu\r\n",HAL_RCC_GetPCLK1Freq());
    printf("PCLK2 clock is %lu\r\n",HAL_RCC_GetPCLK2Freq());
    printf("HCLK3 clock is %lu\r\n",HAL_RCC_GetHCLKFreq());
    printf("SPI1/2/3 peripheral clock = %lu\r\n",HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123));
    printf("ADC peripheral clock = %lu\r\n",HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC));
    printf("SDMMC peripheral clock = %lu\r\n",HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC));
    printf("SPI5 peripheral clock = %lu\r\n",HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI5));
}

void hwInit()
{
    MPU_SetProtection();
    SCB_EnableICache();		// 使能ICache
	  SCB_EnableDCache();		// 使能DCache
    HAL_Init(); //初始化 HAL 库
    Stm32_Clock_Init(); //设置时钟,480Mhz
    /* System Clock Update */
    SystemCoreClockUpdate();
    delay_init(HAL_RCC_GetSysClockFreq()/1000000);//1US跑的tick数
    uart_init(DEBUG_UART_BOUNDRATE);
    printAllClk();
    #if USE_SDRAM 
    if(MX_FMC_Init() != HAL_OK)
    {
        printf("ERROR MX_FMC_Init\r\n");
    }
    else
    {
        printf("MX_FMC_Init OK\r\n");
    }
    if(SDRAM_Check() != HAL_OK)
    {
        printf("ERROR Check SDRAM Fail\r\n");
    }
    printf("MX_FMC_CHECK OK\r\n");
    #endif
}

void SoftReset(void)
{
  __set_FAULTMASK(1); // 关闭所有中端
  NVIC_SystemReset(); // 复位
}