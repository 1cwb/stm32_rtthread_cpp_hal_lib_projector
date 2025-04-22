#include "adc.hpp"
#include "project.hpp"

adcx* adc1 = nullptr;
adcx* adc2 = nullptr;
adcx* adc3 = nullptr;

extern "C" void ADC3_IRQHandler(void)
{
    if(adc3)
    {
        HAL_ADC_IRQHandler(adc3->adcHandle());
    }
}
extern "C" void ADC_IRQHandler(void)
{
    if(adc1)
    {
        HAL_ADC_IRQHandler(adc1->adcHandle());
    }
    if(adc2)
    {
        HAL_ADC_IRQHandler(adc2->adcHandle());
    }
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
  if(adc3)
  {
    HAL_DMA_IRQHandler(adc3->dmaHandle());
  }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    adcx* padc = containerof(hadc, adcx, _adcHandle);
    if(hadc == padc->adcHandle())
    {
        if(padc->buseRxDma())
        {
            SCB_InvalidateDCache_by_Addr((uint32_t*)padc->getRxBuff(), padc->RX_BUFF_LEN);
        }
        else
        {
            padc->read((uint32_t*)padc->getRxBuff());
        }
        padc->setTransferComplete(true);
 /*
        mDev::mAdc::usartData data = {
            .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_COMPLETE,
            .data = padc->getRxBuff(),
            .len = hadc->Init.NbrOfConversion,
        };
        padc->runInterruptCb(&data);
        */
    }
}


extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    adcx* padc = containerof(hadc, adcx, _adcHandle);
    if(hadc == padc->adcHandle())
    {
        /*mDev::mAdc::usartData data = {
            .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_HALF_COMPLETE,
            .data = padc->getRxBuff(),
            .len = hadc->Init.NbrOfConversion,
        };
        padc->runInterruptCb(&data);*/
    }
}
extern "C" void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    adcx* padc = containerof(hadc, adcx, _adcHandle);
    if(hadc == padc->adcHandle())
    {
        /*mDev::mAdc::usartData data = {
            .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_LEVEL_OUT_OF_WINDOW,
            .data = padc->getRxBuff(),
            .len = hadc->Init.NbrOfConversion,
        };
        padc->runInterruptCb(&data);*/
    }
}
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    adcx* padc = containerof(hadc, adcx, _adcHandle);
    if(hadc == padc->adcHandle())
    {
        /*mDev::mAdc::usartData data = {
            .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_ERROR,
            .data = padc->getRxBuff(),
            .len = hadc->Init.NbrOfConversion,
        };
        padc->runInterruptCb(&data);*/
    }
}

int adcInit()
{
    DMA_HandleTypeDef   DMA_Handle = {0};
    ADC_HandleTypeDef   AdcHandle = {0};
	ADC_ChannelConfTypeDef   sConfig = {0};
#if 1
    DMA_Handle.Instance                 = DMA1_Stream3;            /* 使用的DMA1 Stream1 */
	DMA_Handle.Init.Request             = DMA_REQUEST_ADC3;  	   /* 请求类型采用DMA_REQUEST_ADC3 */  
	DMA_Handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* 传输方向是从存储器到外设 */  
	DMA_Handle.Init.PeriphInc           = DMA_PINC_DISABLE;        /* 外设地址自增禁止 */ 
	DMA_Handle.Init.MemInc              = DMA_MINC_ENABLE;         /* 存储器地址自增使能 */  
	DMA_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* 外设数据传输位宽选择半字，即16bit */     
	DMA_Handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;   /* 存储器数据传输位宽选择半字，即16bit */    
	DMA_Handle.Init.Mode                = DMA_CIRCULAR;               /* 普通模式 */   
	DMA_Handle.Init.Priority            = DMA_PRIORITY_LOW;        /* 优先级低 */  
	DMA_Handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;    /* 禁止FIFO*/
	DMA_Handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; /* 禁止FIFO此位不起作用，用于设置阀值 */
	DMA_Handle.Init.MemBurst            = DMA_MBURST_SINGLE;       /* 禁止FIFO此位不起作用，用于存储器突发 */
	DMA_Handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;       /* 禁止FIFO此位不起作用，用于外设突发 */

    AdcHandle.Instance = ADC3;
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;          /* 采用PLL异步时钟，2分频，即64MHz/2 = 32MHz */
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_16B;        /* 16位分辨率 */
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;           /* 使能扫描*/
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;          /* 整个EOC序列转换结束标志 */
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;                   /* 禁止低功耗自动延迟特性 */
    AdcHandle.Init.ContinuousConvMode    = ENABLE;                    /* 使能连续转换 */
    AdcHandle.Init.NbrOfConversion       = 3;                         /* 使用了3个转换通道 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                   /* 禁止不连续模式 */
    AdcHandle.Init.NbrOfDiscConversion   = 1;                         /* 禁止不连续模式后，此参数忽略，此位是用来配置不连续子组中通道数 */
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        /* 采用软件触发 */
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;    /* 采用软件触发的话，此位忽略 */
    AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* DMA循环模式接收ADC转换的数据 */
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;     	   /* ADC转换溢出的话，覆盖ADC的数据寄存器 */
    AdcHandle.Init.OversamplingMode      = DISABLE;                            /* 禁止过采样 */

    /* 配置ADC通道，序列2，采样Vbat/4 */
	sConfig.Channel      = ADC_CHANNEL_VBAT;            /* 配置使用的ADC通道 */
	sConfig.Rank         = ADC_REGULAR_RANK_1;          /* 采样序列里的第1个 */
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* 采样周期 */
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* 单端输入 */
	sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* 无偏移 */ 
	sConfig.Offset = 0;                                 /* 无偏移的情况下，此参数忽略 */
	sConfig.OffsetRightShift       = DISABLE;           /* 禁止右移 */
	sConfig.OffsetSignedSaturation = DISABLE;           /* 禁止有符号饱和 */

    adc3 = new adcx(DEV_ADC3);
    mResult ret = adc3->init([&](bool binit){
        if(binit)
        {
            __HAL_RCC_DMA1_CLK_ENABLE();
            __HAL_RCC_ADC3_CLK_ENABLE();
            adc3->dmaInit();
            if(adc3->buseRxDma())
            {
                printf("%s()%d: dma init\r\n", __func__, __LINE__);
                HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 4, 0);
                HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
            }
            //HAL_NVIC_SetPriority(ADC3_IRQn, 3, 0);
            //HAL_NVIC_EnableIRQ(ADC3_IRQn);
        }
        else
        {
            //adc3->dmaDeInit();
        }
    }, &AdcHandle, &sConfig, &DMA_Handle);
    if(ret != M_RESULT_EOK)
    {
        printf("error: adc3 init fail\r\n");
        return -1;
    }
    /* 配置ADC通道，序列3，采样VrefInt */
	sConfig.Channel      = ADC_CHANNEL_VREFINT;         /* 配置使用的ADC通道 */
	sConfig.Rank         = ADC_REGULAR_RANK_2;          /* 采样序列里的第1个 */
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* 采样周期 */
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* 单端输入 */
	sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* 无偏移 */ 
	sConfig.Offset = 0;                                 /* 无偏移的情况下，此参数忽略 */
	sConfig.OffsetRightShift       = DISABLE;           /* 禁止右移 */
	sConfig.OffsetSignedSaturation = DISABLE;           /* 禁止有符号饱和 */
	
    adc3->addChannel(&sConfig);

	/* 配置ADC通道，序列4，采样温度 */
	sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;      /* 配置使用的ADC通道 */
	sConfig.Rank         = ADC_REGULAR_RANK_3;          /* 采样序列里的第1个 */
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* 采样周期 */
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* 单端输入 */
	sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* 无偏移 */ 
	sConfig.Offset = 0;                                 /* 无偏移的情况下，此参数忽略 */
	sConfig.OffsetRightShift       = DISABLE;           /* 禁止右移 */
	sConfig.OffsetSignedSaturation = DISABLE;           /* 禁止有符号饱和 */

    adc3->addChannel(&sConfig);
    adc3->start(mDev::recvMode::RECV_MODE_DMA, (uint32_t*)adc3->getRxBuff(), 3);
    uint16_t TS_CAL1;
	uint16_t TS_CAL2;
    float mpu_temp = 0.0;
    while (1)
    {
        //SCB_InvalidateDCache_by_Addr((uint32_t *)adc3->getRxBuff(),  adc3->RX_BUFF_LEN);
        if(adc3->btransferComplete())
        {
            adc3->setTransferComplete(false);
            mpu_temp = adc3->getRxBuff()[2];	//读取ADC转换数据（16位数据）
            TS_CAL1 = *(__IO uint16_t *)(0x1FF1E820);
            TS_CAL2 = *(__IO uint16_t *)(0x1FF1E840);
            mpu_temp = ((110.0f - 30.0f) / (TS_CAL2 - TS_CAL1)) * (mpu_temp - TS_CAL1) + 30.0f;
            printf("temp: %f\r\n", mpu_temp);
            //adc3->start(mDev::recvMode::RECV_MODE_DMA, (uint32_t*)adc3->getRxBuff(), adc3->RX_BUFF_LEN/2);
        }
        //HAL_Delay(10);
    }
    
#endif
#if 0
    AdcHandle.Instance = ADC3;
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;          /* 采用PLL异步时钟，4分频，即100MHz/4 = 25MHz */
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_16B;        /* 16位分辨率 */
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;           /* 使能扫描*/
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;          /* 整个EOC序列转换结束标志 */
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;                   /* 禁止低功耗自动延迟特性 */
    AdcHandle.Init.ContinuousConvMode    = DISABLE;                    /* 使能连续转换 */
    AdcHandle.Init.NbrOfConversion       = 1;                         /* 使用了3个转换通道 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                   /* 禁止不连续模式 */
    AdcHandle.Init.NbrOfDiscConversion   = 0;                         /* 禁止不连续模式后，此参数忽略，此位是用来配置不连续子组中通道数 */
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        /* 采用软件触发 */
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;    /* 采用软件触发的话，此位忽略 */
    AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR; /* DMA循环模式接收ADC转换的数据 */
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;     	   /* ADC转换溢出的话，覆盖ADC的数据寄存器 */
    AdcHandle.Init.OversamplingMode      = DISABLE;                            /* 禁止过采样 */

	/* 配置ADC通道，序列4，采样温度 */
	sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;      /* 配置使用的ADC通道 */
	sConfig.Rank         = ADC_REGULAR_RANK_1;          /* 采样序列里的第1个 */
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* 采样周期 */
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* 单端输入 */
	sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* 无偏移 */ 
	sConfig.Offset = 0;                                 /* 无偏移的情况下，此参数忽略 */
	sConfig.OffsetRightShift       = DISABLE;           /* 禁止右移 */
	sConfig.OffsetSignedSaturation = DISABLE;           /* 禁止有符号饱和 */

    adc3 = new adcx(DEV_ADC3);
    mResult ret = adc3->init([&](bool binit){
        if(binit)
        {
            __HAL_RCC_ADC3_CLK_ENABLE();
            HAL_NVIC_SetPriority(ADC3_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(ADC3_IRQn);
        }
        else
        {
            //adc3->dmaDeInit();
        }
    }, &AdcHandle, &sConfig, nullptr);
    if(ret != M_RESULT_EOK)
    {
        printf("error: adc3 init fail\r\n");
        return -1;
    }
#endif
    return 0;
}
INIT_EXPORT(adcInit, "0.4");