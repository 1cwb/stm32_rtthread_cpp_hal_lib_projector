#include "newtimer.hpp"


extern "C" void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim == newTimerx::GetObjectFromPrivateMember(htim)->getTimHandle()) {
        newTimerx::GetObjectFromPrivateMember(htim)->runInitCallback(true);
    }
}
extern "C" void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
    if(htim == newTimerx::GetObjectFromPrivateMember(htim)->getTimHandle()) {
        newTimerx::GetObjectFromPrivateMember(htim)->runInitCallback(false);
    }
}