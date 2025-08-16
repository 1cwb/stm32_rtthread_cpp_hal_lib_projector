#include "adc.hpp"
#include "project.hpp"
#include "gpio.hpp"

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
    adcx* padc = adcx::GetObjectFromPrivateMember(hadc);
    if(hadc == padc->adcHandle())
    {
        if(padc->buseRxDma())
        {
            SCB_InvalidateDCache_by_Addr((uint32_t*)padc->getRxBuff(), padc->RX_BUFF_LEN);
            mDev::mAdc::usartData data = {
                .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_COMPLETE,
                .data = padc->getRxBuff(),
                .dataPerSize = padc->getDataPerSize(),
                .dataOfobjCount = padc->getChannelNum(),
                .len = padc->getDataPerSize()*padc->getDmaBuffsize(),
            };
            padc->setTransferComplete(true);
            padc->runInterruptCb(&data);
        }
        else
        {
            padc->read((uint32_t*)(padc->getRxBuff() + padc->getItcoverCount()*padc->getDataPerSize()));
            padc->setItcoverCount(padc->getItcoverCount()+1);
            if(padc->getItcoverCount() == padc->getChannelNum())
            {
                mDev::mAdc::usartData data = {
                    .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_COMPLETE,
                    .data = padc->getRxBuff(),
                    .dataPerSize = padc->getDataPerSize(),
                    .dataOfobjCount = padc->getChannelNum(),
                    .len = padc->getItcoverCount()*padc->getDataPerSize(),
                };
                padc->setItcoverCount(0);
                padc->setTransferComplete(true);
                padc->runInterruptCb(&data);
            }
        }
    }
}

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    adcx* padc = adcx::GetObjectFromPrivateMember(hadc);
    if(hadc == padc->adcHandle())
    {
        if(padc->buseRxDma())
        {
            SCB_InvalidateDCache_by_Addr((uint32_t*)padc->getRxBuff(), padc->RX_BUFF_LEN);
            mDev::mAdc::usartData data = {
                .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_HALF_COMPLETE,
                .data = padc->getRxBuff(),
                .dataPerSize = padc->getDataPerSize(),
                .dataOfobjCount = padc->getChannelNum(),
                .len = padc->getDataPerSize()*padc->getDmaBuffsize()/2,
            };
            padc->setTransferComplete(true);
            padc->runInterruptCb(&data);
        }
    }
}
extern "C" void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    adcx* padc = adcx::GetObjectFromPrivateMember(hadc);
    if(hadc == padc->adcHandle())
    {
        mDev::mAdc::usartData data = {
            .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_LEVEL_OUT_OF_WINDOW,
            .data = (uint8_t*)padc->getRxBuff(),
            .dataPerSize = padc->getDataPerSize(),
            .len = padc->getDataPerSize()*padc->getDmaBuffsize(),
        };
        padc->runInterruptCb(&data);
    }
}
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    adcx* padc = adcx::GetObjectFromPrivateMember(hadc);
    if(hadc == padc->adcHandle())
    {
        mDev::mAdc::usartData data = {
            .type = mDev::ADC_EVENT_TYPE::ADC_EVNET_TYPE_CONV_ERROR,
            .data = (uint8_t*)padc->getRxBuff(),
            .dataPerSize = padc->getDataPerSize(),
            .len = padc->getDataPerSize()*padc->getDmaBuffsize(),
        };
        padc->runInterruptCb(&data);
    }
}
#if 0
int adcInit()
{
    DMA_HandleTypeDef   DMA_Handle = {0};
    ADC_HandleTypeDef   AdcHandle = {0};
	ADC_ChannelConfTypeDef   sConfig = {0};

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
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV10;          /* 采用PLL异步时钟，10分频，即100MHz/10 = 10MHz */
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_16B;        /* 16位分辨率 */
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;           /* 使能扫描*/
    AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;          /* 整个EOC序列转换结束标志 */
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;                   /* 禁止低功耗自动延迟特性 */
    AdcHandle.Init.ContinuousConvMode    = ENABLE;                    /* 使能连续转换 */
    AdcHandle.Init.NbrOfConversion       = 5;                         /* 使用了3个转换通道 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                   /* 禁止不连续模式 */
    AdcHandle.Init.NbrOfDiscConversion   = 1;                         /* 禁止不连续模式后，此参数忽略，此位是用来配置不连续子组中通道数 */
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        /* 采用软件触发 */
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;    /* 采用软件触发的话，此位忽略 */
    AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* DMA循环模式接收ADC转换的数据 */
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;     	   /* ADC转换溢出的话，覆盖ADC的数据寄存器 */
    AdcHandle.Init.OversamplingMode      = DISABLE;                            /* 禁止过采样 */

	/* 配置ADC通道，序列1，采样温度 */
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
            __HAL_RCC_GPIOC_CLK_ENABLE();
            /**ADC3 GPIO Configuration
            PC2_C     ------> ADC3_INP0
            PC3_C     ------> ADC3_INP1
            */
            HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_OPEN);
            HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);
            /** ADC3 CH11:C1 CH12:C2 */
            gpiox adc3ch11("adc3ch11");
            adc3ch11.init([&](bool binit){if(binit){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_1,GPIO_MODE_ANALOG,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH,0);
            gpiox adc3ch12("adc3ch12");
            adc3ch12.init([&](bool binit){if(binit){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_2,GPIO_MODE_ANALOG,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH,0);

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
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    adc3->addChannel(&sConfig);

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    adc3->addChannel(&sConfig);

    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    adc3->addChannel(&sConfig);

    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    adc3->addChannel(&sConfig);

    adc3->start(mDev::recvMode::RECV_MODE_DMA, (uint32_t*)adc3->getRxBuff(), adc3->RX_BUFF_LEN);

    return 0;
}
#endif
#if 1
int adcInit()
{
    ADC_HandleTypeDef   AdcHandle = {0};
	ADC_ChannelConfTypeDef   sConfig = {0};

    AdcHandle.Instance = ADC1;
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV6;          /* 采用PLL异步时钟，10分频，即100MHz/10 = 10MHz */
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_16B;        /* 16位分辨率 */
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;           /* 使能扫描*/
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;          /* 整个EOC序列转换结束标志 */
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;                   /* 禁止低功耗自动延迟特性 */
    AdcHandle.Init.ContinuousConvMode    = DISABLE;                    /* 不使能连续转换 转换完成一次所有通道后就停止，再次转换需要手动开始*/
    AdcHandle.Init.NbrOfConversion       = 1;                         /* 使用了1个转换通道 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                   /* 禁止不连续模式 */
    AdcHandle.Init.NbrOfDiscConversion   = 1;                         /* 禁止不连续模式后，此参数忽略，此位是用来配置不连续子组中通道数 */
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        /* 采用软件触发 */
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;    /* 采用软件触发的话，此位忽略 */
    AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR; /*寄存器模式接收ADC转换的数据 */
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;     	   /* ADC转换溢出的话，覆盖ADC的数据寄存器 */
    AdcHandle.Init.OversamplingMode      = DISABLE;                            /* 禁止过采样 */

	/* 配置ADC通道，序列4，采样温度 */
	sConfig.Channel      = ADC_CHANNEL_4;      /* 配置使用的ADC通道 */
	sConfig.Rank         = ADC_REGULAR_RANK_1;          /* 采样序列里的第1个 */
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;  /* 采样周期 */
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* 单端输入 */
	sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* 无偏移 */ 
	sConfig.Offset = 0;                                 /* 无偏移的情况下，此参数忽略 */
	sConfig.OffsetRightShift       = DISABLE;           /* 禁止右移 */
	sConfig.OffsetSignedSaturation = DISABLE;           /* 禁止有符号饱和 */

    adc1 = new adcx(DEV_ADC1);
    mResult ret = adc1->init([&](bool binit){
        if(binit)
        {
            __HAL_RCC_GPIOC_CLK_ENABLE();

            __HAL_RCC_ADC12_CLK_ENABLE();
            gpiox pc4("pc4");
            pc4.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();},GPIOC, GPIO_PIN_4, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
            HAL_NVIC_SetPriority(ADC_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(ADC_IRQn);
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
    return 0;
}
#endif
INIT_EXPORT(adcInit, "0.4");