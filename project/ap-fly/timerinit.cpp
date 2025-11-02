#include "timer.hpp"
#include "project.hpp"
#include "pwm.hpp"
#include "gpio.hpp"
#include "mdshotdrv.hpp"

static timerx* timer1 = nullptr;
static timerx* timer2 = nullptr;
static timerx* timer3 = nullptr;
static PWMX* pwm1 = nullptr;
static PWMX* pwm2 = nullptr;
static PWMX* pwm3 = nullptr;
static PWMX* pwm4 = nullptr;
static PWMX* pwm5 = nullptr;
static PWMX* pwm6 = nullptr;
//static PWMX* pwm7 = nullptr;
//static PWMX* pwm8 = nullptr;
static mDev::mDSHOT* dshot1 = nullptr;
static mDev::mDSHOT* dshot2 = nullptr;
static mDev::mDSHOT* dshot3 = nullptr;
static mDev::mDSHOT* dshot4 = nullptr;

/*********************Interrupt Callback******************************/
// 3. 添加DMA传输完成中断处理
extern "C" void DMA1_Stream4_IRQHandler(void)
{
    if(timer1 && timer1->getTimHandle()->hdma[TIM_DMA_ID_CC1])
    {
        HAL_DMA_IRQHandler(timer1->getTimHandle()->hdma[TIM_DMA_ID_CC1]);
    }
}

extern "C" void TIM1_UP_IRQHandler(void)
{
    if(timer1)
    {
        if(__HAL_TIM_GET_FLAG(timer1->getTimHandle(), TIM_FLAG_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG(timer1->getTimHandle(), TIM_FLAG_UPDATE);
            timer1->runInterruptCb(nullptr);
        }
    }
}

extern "C" void TIM2_IRQHandler(void)
{
    if(timer2)
    {
        if(__HAL_TIM_GET_FLAG(timer2->getTimHandle(), TIM_FLAG_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG(timer2->getTimHandle(), TIM_FLAG_UPDATE);
            timer2->runInterruptCb(nullptr);
        }
    }
}

extern "C" void TIM3_IRQHandler(void)
{
    if(timer3)
    {
        if(__HAL_TIM_GET_FLAG(timer3->getTimHandle(), TIM_FLAG_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG(timer3->getTimHandle(), TIM_FLAG_UPDATE);
            timer3->runInterruptCb(nullptr);
        }
    }
}
/*********************Interrupt Callback end******************************/

int timeInit()
{
   //TIM1 INIT
    TIM_HandleTypeDef timerst;
    memset(&timerst, 0, sizeof(TIM_HandleTypeDef));
    timerst.Instance = TIM1;
    timerst.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    timerst.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timerst.Init.CounterMode = TIM_COUNTERMODE_UP;
    timerst.Init.RepetitionCounter = 0;

    timer1 = new timerx(DEV_TIMER1);
    timer1->calcPeriodAndPrescalerByFreq(&timerst,600000);
    timer1->baseTimeInit([&](bool b){
        if(b)
        {
            static DMA_HandleTypeDef  hdma_tim;
            __HAL_RCC_TIM1_CLK_ENABLE();
            //HAL_NVIC_SetPriority(TIM1_UP_IRQn, 3, 0);
            //HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

            /* Enable DMA clock */
            __HAL_RCC_DMA1_CLK_ENABLE();

            /* Set the parameters to be configured */
            hdma_tim.Init.Request  = DMA_REQUEST_TIM1_CH1;
            hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
            hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
            hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
            // 在timer初始化中修改DMA配置
            hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  // 16位
            hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;     // 16位
            
            hdma_tim.Init.Mode = DMA_CIRCULAR;
            hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
            hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
            hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
            hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;

            /* Set hdma_tim instance */
            hdma_tim.Instance = DMA1_Stream4;

            /* Link hdma_tim to hdma[TIM_DMA_ID_CC3] (channel3) */
            __HAL_LINKDMA(timer1->getTimHandle(), hdma[TIM_DMA_ID_CC1], hdma_tim);

            /* Initialize TIMx DMA handle */
            HAL_DMA_Init(timer1->getTimHandle()->hdma[TIM_DMA_ID_CC1]);

            /*##-2- Configure the NVIC for DMA #########################################*/
            /* NVIC configuration for DMA transfer complete interrupt */
            HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
        }
    }, &timerst);

    timerst.Instance = TIM2;
    timer2 = new timerx(DEV_TIMER2);
    timer2->calcPeriodAndPrescalerByFreq(&timerst,200);
    timer2->baseTimeInit([](bool b){
        if(b)
        {
            __HAL_RCC_TIM2_CLK_ENABLE();
            HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(TIM2_IRQn);
        }
    }, &timerst);

    timerst.Instance = TIM3;
    timer3 = new timerx(DEV_TIMER3);
    timer3->calcPeriodAndPrescalerByFreq(&timerst,200);
    timer3->baseTimeInit([](bool b){
        if(b)
        {
            __HAL_RCC_TIM3_CLK_ENABLE();
            HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(TIM3_IRQn);
        }
    }, &timerst);

    TIM_OC_InitTypeDef sConfig;
    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* Set the pulse value for channel 1 */
    sConfig.Pulse = 0;
    pwm1 = new PWMX(DEV_PWM1, timer1);
    pwm1->pwmConfig(&sConfig, TIM_CHANNEL_1);
    gpiox* pe9 = new gpiox("pe9");
    pe9->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF1_TIM1);

    sConfig.Pulse = 0;
    pwm2 = new PWMX(DEV_PWM2, timer1);
    pwm2->pwmConfig(&sConfig, TIM_CHANNEL_2);
    gpiox* pe11 = new gpiox("pe11");
    pe11->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF1_TIM1);

    sConfig.Pulse = 0;
    pwm3 = new PWMX(DEV_PWM3, timer1);
    pwm3->pwmConfig(&sConfig, TIM_CHANNEL_3);
    gpiox* pe13 = new gpiox("pe13");
    pe13->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_13, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF1_TIM1);

    sConfig.Pulse = 0;
    pwm4 = new PWMX(DEV_PWM4, timer1);
    pwm4->pwmConfig(&sConfig, TIM_CHANNEL_4);
    gpiox* pe14 = new gpiox("pe14");
    pe14->init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF1_TIM1);

    sConfig.Pulse = 0;
    pwm5 = new PWMX(DEV_PWM5, timer2);
    pwm5->pwmConfig(&sConfig, TIM_CHANNEL_3);
    gpiox* pb10 = new gpiox("pb10");
    pb10->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF1_TIM2);

    pwm6 = new PWMX(DEV_PWM6, timer2);
    pwm6->pwmConfig(&sConfig, TIM_CHANNEL_4);
    gpiox* pb11 = new gpiox("pb11");
    pb11->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF1_TIM2);

    //pwm7 = new PWMX(DEV_PWM7, timer3);
    //pwm7->pwmConfig(&sConfig, TIM_CHANNEL_3);

    gpiox* pb0 = new gpiox("pb0");
    //pb0->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_0, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF2_TIM3);
    pb0->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH);

    //pwm8 = new PWMX(DEV_PWM8, timer3);
    //pwm8->pwmConfig(&sConfig, TIM_CHANNEL_4);
    gpiox* pb1 = new gpiox("pb1");
    //pb1->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_1, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF2_TIM3);
    pb1->init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_1, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH);

    dshot1 = new mDev::mDSHOT("dshot1", 240000000);
    dshot1->registerDmaTransferCompleteCb([&]()->bool {return HAL_DMA_GetState(timer1->getTimHandle()->hdma[TIM_DMA_ID_CC1]) == HAL_DMA_STATE_READY;});
    dshot1->registerDmaTransferStartCb([](uint16_t* data, uint32_t len){
        SCB_CleanDCache_by_Addr((uint32_t *) data, len * sizeof(uint16_t));
        HAL_TIM_PWM_Start_DMA(timer1->getTimHandle(), TIM_CHANNEL_1, 
                             reinterpret_cast<uint32_t*>(data), len);
    });
    return 0;
}
INIT_EXPORT(timeInit, "0.4");