#pragma once
#include "rtoscommon.hpp"
#include "mhw.hpp"
#include "mobject.hpp"
#include "mclock.hpp"
#include <functional>

class mTimer
{
    using mTimerCallBack = std::function<void()>;
public:
    mTimer()
    {
        if(mClock::getInstance()->getRegisterCb()==nullptr)
        {
            mClock::getInstance()->getRegisterCb() = timerCheck;
        }
    }
    ~mTimer()
    {

    }
    mTimer(const mTimer&) = delete;
    mTimer(mTimer&&) = delete;
    mTimer& operator=(const mTimer&) = delete;
    mTimer& operator=(mTimer&&) = delete;

    static void mTimerCommonCallbackFunc(void* p);

    static mTimer* create(const char *name,
                uint32_t   time,
                mTimerStateFlag   flag,
                const mTimerCallBack& cb);
    mResult timerCreate(const char *name,
            uint32_t   time,
            mTimerStateFlag   flag,
            const mTimerCallBack& cb);
    /**
     * This function will delete a timer and release timer memory
     *
     * @param timer the timer to be deleted
     *
     * @return the operation status, RT_EOK on OK; RT_ERROR on error
     */
    mResult timerDelete();
    void init(  const char *name,
                uint32_t   time,
                mTimerStateFlag   flag,
                const mTimerCallBack& cb);
    /**
     * This function will initialize a timer, normally this function is used to
     * initialize a static timer object.
     *
     * @param timer the static timer object
     * @param name the name of timer
     * @param timeout the timeout function
     * @param parameter the parameter of timeout function
     * @param time the tick of timer
     * @param flag the flag of timer
     */
    void init(  const char *name,
                void (*timeout)(void *parameter),
                void       *parameter,
                uint32_t   time,
                mTimerStateFlag   flag);
    /**
     * This function will detach a timer from timer management.
     *
     * @param timer the static timer object
     *
     * @return the operation status, RT_EOK on OK; RT_ERROR on error
     */
    mResult timerDetach();
    /**
     * This function will start the timer
     *
     * @param timer the timer to be started
     *
     * @return the operation status, RT_EOK on OK, -RT_ERROR on error
     */
    mResult start();

    /**
     * This function will stop the timer
     *
     * @param timer the timer to be stopped
     *
     * @return the operation status, RT_EOK on OK, -RT_ERROR on error
     */
    mResult stop();
    /**
     * This function will get or set some options of the timer
     *
     * @param timer the timer to be get or set
     * @param cmd the control command
     * @param arg the argument
     *
     * @return RT_EOK
     */
    mResult timerControl(mTimerCtrl cmd, void *arg);
    void setTimerAndStart(uint32_t timout);
    uint32_t getTime();
    bool isTimerActived();
    /**
     * This function will check timer list, if a timeout event happens, the
     * corresponding timeout function will be invoked.
     *
     * @note this function shall be invoked in operating system timer interrupt.
     */
    static void timerCheck(void);

private:

    void timerInit(void (*timeout)(void *parameter), void *parameter, uint32_t  time, mTimerStateFlag flag);
    /* the fist timer always in the last row */
    uint32_t timerListNextTimeout(mList_t timerList[]);

    static void timerRemove(mTimer_t* timer);
    /**
     * This function will create a timer
     *
     * @param name the name of timer
     * @param timeout the timeout function
     * @param parameter the parameter of timeout function
     * @param time the tick of timer
     * @param flag the flag of timer
     *
     * @return the created timer object
     */
    mResult timerCreate(const char *name,
                            void (*timeout)(void *parameter),
                            void       *parameter,
                            unsigned long   time,
                            mTimerStateFlag  flag);

private:
    mTimer_t timer_;
    mTimerCallBack cb_;
    /* hard timer list */
    static mList_t timerListTotal[TIMER_SKIP_LIST_LEVEL];
};