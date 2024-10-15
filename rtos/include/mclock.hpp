#pragma once
#include "rtoscommon.hpp"

class mClock
{
    using mTimerCheckCallBack = void (*)();
public:
    static mClock* getInstance()
    {
        static mClock clk;
        return &clk;
    }
    /**
     * This function will return current tick from operating system startup
     *
     * @return current tick
     */
    uint64_t tickGet(void);
    /**
     * This function will set current tick
     */
    void tickSet(uint64_t tick);
    /**
     * This function will notify kernel there is one tick passed. Normally,
     * this function is invoked by clock ISR.
     */
    void tickIncrease(void);
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
    uint32_t tickFromMillisecond(int32_t ms);
    mTimerCheckCallBack& getRegisterCb();
private:
    mClock() : tick(0)
    {
    }
    ~mClock()
    {

    }
    mClock(const mClock&) = delete;
    mClock(mClock&&) = delete;
    mClock& operator=(const mClock&) = delete;
    mClock& operator=(mClock&&) = delete;


    uint64_t tick;
    mTimerCheckCallBack cb_;
};