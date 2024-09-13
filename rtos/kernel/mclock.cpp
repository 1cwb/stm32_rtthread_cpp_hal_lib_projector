#include "mclock.hpp"
#include "mhw.hpp"
#include "mscheduler.hpp"
/**
 * This function will return current tick from operating system startup
 *
 * @return current tick
 */
uint32_t mClock::tickGet(void)
{
    /* return the global tick */
    return tick;
}
/**
 * This function will set current tick
 */
void mClock::tickSet(uint32_t tick)
{
    long level;

    level = HW::hwInterruptDisable();
    tick = tick;
    HW::hwInterruptEnable(level);
}
/**
 * This function will notify kernel there is one tick passed. Normally,
 * this function is invoked by clock ISR.
 */
void mClock::tickIncrease(void)
{
    /* increase the global tick */
    ++ tick;

    /* check time slice */
    struct thread_t*& thread = mSchedule::getInstance()->getCurrentThread();

    -- thread->remainingTick;
    if (thread->remainingTick == 0)
    {
        /* change to initialized tick */
        thread->remainingTick = thread->initTick;

        /* yield */
        thread->yieldThread();
    }

    /* check timer */
    if(cb_)
    {
        cb_();
    }
}
/**
 * This function will calculate the tick from millisecond.
 *
 * @param ms the specified millisecond
 *           - Negative Number wait forever
 *           - Zero not wait
 *           - Max 0x7fffffff
 *
 * @return the calculated tick
 */
uint32_t mClock::tickFromMillisecond(int32_t ms)
{
    uint32_t tick;

    if (ms < 0)
    {
        tick = (uint32_t)WAITING_FOREVER;
    }
    else
    {
        tick = TICK_PER_SECOND * (ms / 1000);
        tick += (TICK_PER_SECOND * (ms % 1000) + 999) / 1000;
    }

    /* return the calculated tick */
    return tick;
}
mClock::mTimerCheckCallBack& mClock::getRegisterCb()
{
    return cb_;
}