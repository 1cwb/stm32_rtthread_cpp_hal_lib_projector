#include "timer.hpp"
static timerx* timer1 = nullptr;
static timerx* timer2 = nullptr;

/*********************Interrupt Callback******************************/
extern "C" void TIM1_UP_IRQHandler(void)
{
    timerx* timx = timer1;
    if(timx  != nullptr)
    {
        if(__HAL_TIM_GET_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE);
            timx->runInterruptCb(nullptr);
        }
    }
}

extern "C" void TIM2_IRQHandler(void)
{
    timerx* timx = timer2;
    if(timx != nullptr)
    {
        if(__HAL_TIM_GET_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE);
            timx->runInterruptCb(nullptr);
        }
        if(__HAL_TIM_GET_FLAG(timx->getTimHandle(), TIM_FLAG_CC1))
        {
            __HAL_TIM_CLEAR_FLAG(timx->getTimHandle(), TIM_FLAG_CC1);
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
    }, &timerst,mDev::TIMESTARTMODE_IT);
    return 0;
}
INIT_EXPORT(timeInit, "0.2");

