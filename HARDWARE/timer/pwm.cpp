#include "pwm.hpp"
mResult pwmx::init(const mDev::initCallbackExt& cb, TIM_OC_InitTypeDef* TIM_OCInitStructure, uint32_t channel, TIM_BreakDeadTimeConfigTypeDef* TIM_BDTRInitStructure, bool enableNch)
{
    if(!TIM_OCInitStructure)
    {
        return M_RESULT_EINVAL;
    }
    if(!reinterpret_cast<timerx*>(this->getTimer())->isInited())
    {
        return M_RESULT_EINVAL;
    }
    if(TIM_BDTRInitStructure)
    {
        _bUseBreakDeadTime = true;
    }
    _initcb = cb;
    if(_initcb)
    {
        _initcb(true);
    }
    _channel = channel;
    _bEnableNch = enableNch;
    //memcpy(&_timOCInitStructure, TIM_OCInitStructure, sizeof(TIM_OC_InitTypeDef));
    if(HAL_TIM_PWM_ConfigChannel(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(),TIM_OCInitStructure,_channel) != HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    if(_bUseBreakDeadTime)
    {
        if(HAL_TIMEx_ConfigBreakDeadTime(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(), TIM_BDTRInitStructure) != HAL_OK)
        {
            return M_RESULT_ERROR;
        }
    }
    return M_RESULT_EOK;
}
mResult pwmx::deInit()
{
    if(_initcb)
    {
        _initcb(false);
    }
    return M_RESULT_EOK;
}
void pwmx::start()
{
    HAL_TIM_PWM_Start(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(),_channel);
    if(_bEnableNch)
    {
        HAL_TIMEx_PWMN_Start(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(),_channel);
    }
}
void pwmx::stop()
{
    HAL_TIM_PWM_Stop(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(),_channel);
    if(_bEnableNch)
    {
        HAL_TIMEx_PWMN_Stop(reinterpret_cast<timerx*>(this->getTimer())->getTimHandle(),_channel);
    }
}