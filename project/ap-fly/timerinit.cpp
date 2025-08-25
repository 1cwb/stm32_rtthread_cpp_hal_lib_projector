#include "timer.hpp"
#include "project.hpp"
#include "pwm.hpp"
#include "gpio.hpp"
static timerx* timer1 = nullptr;
static timerx* timer2 = nullptr;
static PWMX* pwm1 = nullptr;
static PWMX* pwm2 = nullptr;
static PWMX* pwm3 = nullptr;
static PWMX* pwm4 = nullptr;
/*********************Interrupt Callback******************************/
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

    timer1 = new timerx("timer1");
    timer1->calcPeriodAndPrescalerByFreq(&timerst,200);
    timer1->baseTimeInit([](bool b){
        if(b)
        {
            __HAL_RCC_TIM1_CLK_ENABLE();
            HAL_NVIC_SetPriority(TIM1_UP_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
        }
    }, &timerst);
#if 1
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
#endif
    return 0;
}
INIT_EXPORT(timeInit, "0.4");

