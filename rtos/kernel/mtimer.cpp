#include "mtimer.hpp"
#include "mklog.hpp"

mList_t mTimer::timerListTotal[TIMER_SKIP_LIST_LEVEL];

void mTimer::mTimerCommonCallbackFunc(void* p)
{
    if(p)
    {
        mTimer* ptimer = reinterpret_cast<mTimer*>(p);
        if(ptimer->cb_)
        {
            ptimer->cb_();
        }
    }
}

mTimer* mTimer::create(const char *name,
            uint32_t   time,
            mTimerStateFlag   flag,
            const mTimerCallBack& cb)
{
    mTimer* timer = new mTimer;
    if(timer)
    {
        timer->timerCreate(name, time, flag, cb);
        return timer;
    }
    return nullptr;
}

mResult mTimer::timerCreate(const char *name,
            uint32_t   time,
            mTimerStateFlag   flag,
            const mTimerCallBack& cb)
{
    cb_ = std::move(cb);
    return timerCreate(name, mTimerCommonCallbackFunc, this, time, flag);
}
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
mResult mTimer::timerCreate(const char *name,
                        void (*timeout)(void *parameter),
                        void       *parameter,
                        unsigned long   time,
                        mTimerStateFlag  flag)
{
    /* allocate a object */
    mObject::getInstance()->objectAdd((mObject_t*)this, M_OBJECT_CLASS_TIMER, name);
    timerInit(timeout, parameter, time, flag);

    return M_RESULT_EOK;
}
/**
 * This function will delete a timer and release timer memory
 *
 * @param timer the timer to be deleted
 *
 * @return the operation status, RT_EOK on OK; RT_ERROR on error
 */
mResult mTimer::timerDelete()
{
    register long level;

    /* timer check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)(&timer_)) == M_OBJECT_CLASS_TIMER);
    MASSERT(!mObject::getInstance()->objectIsSystemobject((mObject_t*)(&timer_)));
    if(mObject::getInstance()->objectIsSystemobject((mObject_t*)this))
    {
        return M_RESULT_ERROR;
    }
    /* disable interrupt */
    level = HW::hwInterruptDisable();

    timerRemove(&timer_);
    /* stop timer */
    timer_.flag &= ~TIMER_STATE_FLAG_ACTIVATED;

    /* enable interrupt */
    HW::hwInterruptEnable(level);

    mObject::getInstance()->objectRemove((mObject_t*)this);
    delete this;
    return M_RESULT_EOK;
}
void mTimer::init(  const char *name,
            uint32_t   time,
            mTimerStateFlag   flag,
            const mTimerCallBack& cb)
{
    cb_ = std::move(cb);
    init(name, mTimerCommonCallbackFunc, this, time, flag);
}
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
void mTimer::init(  const char *name,
            void (*timeout)(void *parameter),
            void       *parameter,
            uint32_t   time,
            mTimerStateFlag   flag)
{
    /* timer object initialization */
    mObject::getInstance()->objectInit((mObject_t*)this, M_OBJECT_CLASS_TIMER, name);

    timerInit(timeout, parameter, time, flag);
}
/**
 * This function will detach a timer from timer management.
 *
 * @param timer the static timer object
 *
 * @return the operation status, RT_EOK on OK; RT_ERROR on error
 */
mResult mTimer::timerDetach()
{
    register long level;

    /* timer check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)(&timer_)) == M_OBJECT_CLASS_TIMER);
    MASSERT(mObject::getInstance()->objectIsSystemobject((mObject_t*)(&timer_)));

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    //_rt_timer_remove(timer);
    timerRemove(&timer_);
    /* stop timer */
    timer_.flag &= ~TIMER_STATE_FLAG_ACTIVATED;

    /* enable interrupt */
    HW::hwInterruptEnable(level);

    mObject::getInstance()->objectDetach((mObject_t*)(&timer_));

    return M_RESULT_EOK;
}
/**
 * This function will start the timer
 *
 * @param timer the timer to be started
 *
 * @return the operation status, RT_EOK on OK, -RT_ERROR on error
 */
mResult mTimer::start()
{
    unsigned int rowLvl;
    mList_t *timerList;
    register long level;
    mList_t *rowHead[TIMER_SKIP_LIST_LEVEL];
    unsigned int tstNr;
    static unsigned int randomNr;

    /* timer check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)(&timer_)) == M_OBJECT_CLASS_TIMER);

    /* stop timer firstly */
    level = HW::hwInterruptDisable();
    /* remove timer from list */
    //_rt_timer_remove(timer);
    timerRemove(&timer_);
    /* change status of timer */
    timer_.flag &= ~TIMER_STATE_FLAG_ACTIVATED;

    //RT_OBJECT_HOOK_CALL(rt_object_take_hook, (&(timer->parent)));

    /*
    * get timeout tick,
    * the max timeout tick shall not great than RT_TICK_MAX/2
    */
    MASSERT(timer_.initTick < TICK_MAX / 2);
    timer_.timeoutTick = mClock::getInstance()->tickGet() + timer_.initTick;

    /* insert timer to system timer list */
    timerList = timerListTotal;

    rowHead[0]  = &timerList[0];
    for (rowLvl = 0; rowLvl < TIMER_SKIP_LIST_LEVEL; rowLvl++)
    {
        for (; rowHead[rowLvl] != timerList[rowLvl].prev;
            rowHead[rowLvl]  = rowHead[rowLvl]->next)
        {
            struct mTimer_t *t;
            mList_t *p = rowHead[rowLvl]->next;

            /* fix up the entry pointer */
            t = listEntry(p, struct mTimer_t, row[rowLvl]);

            /* If we have two timers that timeout at the same time, it's
            * preferred that the timer inserted early get called early.
            * So insert the new timer to the end the the some-timeout timer
            * list.
            */
            if ((t->timeoutTick - timer_.timeoutTick) == 0)
            {
                continue;
            }
            else if ((t->timeoutTick - timer_.timeoutTick) < TICK_MAX / 2)
            {
                break;
            }
        }
        if (rowLvl != TIMER_SKIP_LIST_LEVEL - 1)
            rowHead[rowLvl + 1] = rowHead[rowLvl] + 1;
    }

    /* Interestingly, this super simple timer insert counter works very very
    * well on distributing the list height uniformly. By means of "very very
    * well", I mean it beats the randomness of timer->timeoutTick very easily
    * (actually, the timeoutTick is not random and easy to be attacked). */
    randomNr++;
    tstNr = randomNr;

    //rt_list_insert_after(rowHead[RT_TIMER_SKIP_LIST_LEVEL - 1],
    //                    &(timer->row[RT_TIMER_SKIP_LIST_LEVEL - 1]));
    timer_.row[TIMER_SKIP_LIST_LEVEL - 1].insertAfterTo(rowHead[TIMER_SKIP_LIST_LEVEL - 1]);
    for (rowLvl = 2; rowLvl <= TIMER_SKIP_LIST_LEVEL; rowLvl++)
    {
        if (!(tstNr & TIMER_SKIP_LIST_MASK))
        {
            timer_.row[TIMER_SKIP_LIST_LEVEL - rowLvl].insertAfterTo(rowHead[TIMER_SKIP_LIST_LEVEL - rowLvl]);
        }
        else
        {
            break;
        }
        /* Shift over the bits we have tested. Works well with 1 bit and 2
        * bits. */
        tstNr >>= (TIMER_SKIP_LIST_MASK + 1) >> 1;
    }

    timer_.flag |= TIMER_STATE_FLAG_ACTIVATED;

    /* enable interrupt */
    HW::hwInterruptEnable(level);
    return M_RESULT_EOK;
}

/**
 * This function will stop the timer
 *
 * @param timer the timer to be stopped
 *
 * @return the operation status, RT_EOK on OK, -RT_ERROR on error
 */
mResult mTimer::stop()
{
    register long level;

    /* timer check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)(&timer_)) == M_OBJECT_CLASS_TIMER);

    if (!(timer_.flag & TIMER_STATE_FLAG_ACTIVATED))
    {
        return M_RESULT_ERROR;
    }

    //RT_OBJECT_HOOK_CALL(rt_object_put_hook, (&(timer->parent)));

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    timerRemove(&timer_);
    /* change status */
    timer_.flag &= ~TIMER_STATE_FLAG_ACTIVATED;

    /* enable interrupt */
    HW::hwInterruptEnable(level);

    return M_RESULT_EOK;
}
/**
 * This function will get or set some options of the timer
 *
 * @param timer the timer to be get or set
 * @param cmd the control command
 * @param arg the argument
 *
 * @return RT_EOK
 */
mResult mTimer::timerControl(mTimerCtrl cmd, void *arg)
{
    register long level;

    /* timer check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)(&timer_)) == M_OBJECT_CLASS_TIMER);

    level = HW::hwInterruptDisable();
    switch (cmd)
    {
    case TIMER_CTRL_GET_TIME:
        *(uint32_t *)arg = timer_.initTick;
        break;

    case TIMER_CTRL_SET_TIME:
        timer_.initTick = *(uint32_t *)arg;
        break;

    case TIMER_CTRL_SET_ONESHOT:
        timer_.flag &= ~TIMER_FLAG_PERIODIC;
        break;

    case TIMER_CTRL_SET_PERIODIC:
        timer_.flag |= TIMER_FLAG_PERIODIC;
        break;

    case TIMER_CTRL_GET_STATE:
        if(timer_.flag & TIMER_STATE_FLAG_ACTIVATED)
        {
            /*timer is start and run*/
            *(uint32_t *)arg = TIMER_STATE_FLAG_ACTIVATED;
        }
        else
        {
            /*timer is stop*/
            *(uint32_t *)arg = TIMER_STATE_FLAG_DEACTIVATED;
        }
        break;

    default:
        break;
    }
    HW::hwInterruptEnable(level);
    return M_RESULT_EOK;
}

void mTimer::setTimerAndStart(uint32_t timout)
{
    timerControl(TIMER_CTRL_SET_TIME, (void*)&timout);
    start();
}
uint32_t mTimer::getTime()
{
    uint32_t time = 0;
    timerControl(TIMER_CTRL_GET_TIME, (void*)&time);
    return time;
}
bool mTimer::isTimerActived()
{
    uint32_t state = TIMER_STATE_FLAG_DEACTIVATED;
    timerControl(TIMER_CTRL_GET_STATE, (void*)&state);
    return state == TIMER_STATE_FLAG_ACTIVATED ? true : false;
}

/**
 * This function will check timer list, if a timeout event happens, the
 * corresponding timeout function will be invoked.
 *
 * @note this function shall be invoked in operating system timer interrupt.
 */
void mTimer::timerCheck(void)
{
    struct mTimer_t *t;
    uint32_t currentTick;
    register long level;
    mList_t list;

    //RT_DEBUG_LOG(RT_DEBUG_TIMER, ("timer check enter\n"));

    currentTick = mClock::getInstance()->tickGet();

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    while (!(timerListTotal[TIMER_SKIP_LIST_LEVEL - 1]).isEmpty())
    {
        t = listEntry(timerListTotal[TIMER_SKIP_LIST_LEVEL - 1].next,
                        struct mTimer_t, row[TIMER_SKIP_LIST_LEVEL - 1]);

        /*
        * It supposes that the new tick shall less than the half duration of
        * tick max.
        */
        if ((currentTick - t->timeoutTick) < TICK_MAX / 2)
        {
            //RT_OBJECT_HOOK_CALL(rt_timer_enter_hook, (t));

            /* remove timer from timer list firstly */
            timerRemove(t);

            /*one short flag*/
            if (!(t->flag & TIMER_FLAG_PERIODIC))
            {
                t->flag &= ~TIMER_STATE_FLAG_ACTIVATED;
            }
            /* add timer to temporary list  */
            t->row[TIMER_SKIP_LIST_LEVEL - 1].insertAfterTo(&list);
            /* call timeout function */
            t->timeoutFunc(t->parameter);

            /* re-get tick */
            currentTick = mClock::getInstance()->tickGet();

            //RT_OBJECT_HOOK_CALL(rt_timer_exit_hook, (t));
            //RT_DEBUG_LOG(RT_DEBUG_TIMER, ("current tick: %d\n", current_tick));

            /* Check whether the timer object is detached or started again */
            if (list.isEmpty())
            {
                continue;
            }
            t->row[TIMER_SKIP_LIST_LEVEL - 1].removeSelf();

            if ((t->flag & TIMER_FLAG_PERIODIC) &&
                (t->flag & TIMER_STATE_FLAG_ACTIVATED))
            {
                /* start it */
                t->flag &= ~TIMER_STATE_FLAG_ACTIVATED;
                reinterpret_cast<mTimer*>(t)->start();
            }
        }
        else break;
    }

    /* enable interrupt */
    HW::hwInterruptEnable(level);

    //RT_DEBUG_LOG(RT_DEBUG_TIMER, ("timer check leave\n"));
}


void mTimer::timerInit(void (*timeout)(void *parameter), void *parameter, uint32_t  time, mTimerStateFlag flag)
{
    /* set flag */
    timer_.flag  = flag;

    /* set deactivated */
    timer_.flag &= ~TIMER_STATE_FLAG_ACTIVATED;

    timer_.timeoutFunc = timeout;
    timer_.parameter   = parameter;

    timer_.timeoutTick = 0;
    timer_.initTick    = time;
}
/* the fist timer always in the last row */
uint32_t mTimer::timerListNextTimeout(mList_t timerList[])
{
    struct mTimer_t *timer;
    register long level;
    uint32_t timeoutTick = TICK_MAX;

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    //if (!rt_list_isempty(&timer_list[RT_TIMER_SKIP_LIST_LEVEL - 1]))
    if (!timerList[TIMER_SKIP_LIST_LEVEL - 1].isEmpty())
    {
        timer = listEntry(timerList[TIMER_SKIP_LIST_LEVEL - 1].next,
                            struct mTimer_t, row[TIMER_SKIP_LIST_LEVEL - 1]);
        timeoutTick = timer->timeoutTick;
    }

    /* enable interrupt */
    HW::hwInterruptEnable(level);

    return timeoutTick;
}

inline void mTimer::timerRemove(mTimer_t* timer)
{
    int i;

    for (i = 0; i < TIMER_SKIP_LIST_LEVEL; i++)
    {
        timer->row[i].removeSelf();
    }
}
