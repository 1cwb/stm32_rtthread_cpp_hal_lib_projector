#include "timer.hpp"
timerx::timerx(const char* name, TIM_TypeDef* TIMx, uint32_t timefreq, uint32_t countMode, uint32_t repetCount, uint32_t div, bool autoReload):mDev::mTimer(name,timefreq)
{

}
void timerx::updateFreq(uint32_t timefreq)
{

}
uint32_t timerx::getTimeOut()
{

}