#include "mscheduler.hpp"
//#ifdef RT_USING_OVERFLOW_CHECK
void mSchedule::schedulerStackCheck(thread_t *thread)
{
    MASSERT(thread != nullptr);

#if defined(ARCH_CPU_STACK_GROWS_UPWARD)
    if (*((uint8_t *)((unsigned long)thread->stackAddr + thread->stackSize - 1)) != '#' ||
#else
    if (*((uint8_t *)thread->stackAddr) != '#' ||
#endif
        (unsigned long)thread->sp <= (unsigned long)thread->stackAddr ||
        (unsigned long)thread->sp >
        (unsigned long)thread->stackAddr + (unsigned long)thread->stackSize)
    {
        unsigned long level;

        rt_kprintf("thread:%s stack overflow\n", thread->name);
#ifdef RT_USING_FINSH //tony
        {
            extern long list_thread(void);
            list_thread();
        }
#endif
        level = HW::hwInterruptDisable();
        while (level);
    }
#if defined(ARCH_CPU_STACK_GROWS_UPWARD)
    else if ((unsigned long)thread->sp > ((unsigned long)thread->stackAddr + thread->stackSize))
    {
        rt_kprintf("warning: %s stack is close to the top of stack address.\n",
                thread->name);
    }
#else
    else if ((unsigned long)thread->sp <= ((unsigned long)thread->stackAddr + 32))
    {
        rt_kprintf("warning: %s stack is close to end of stack address.\n",
                thread->name);
    }
#endif
}
//#endif

/**
 * @ingroup SystemInit
 * This function will initialize the system scheduler
 */
void mSchedule::systemSchedulerInit(void)
{
    //register long offset;

    mIrq::getInstance()->interruptNestReset();

    //RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("start scheduler: max priority 0x%02x\n", THREAD_PRIORITY_MAX));

    /*for (offset = 0; offset < THREAD_PRIORITY_MAX; offset ++)
    {
        rt_list_init(&rt_thread_priority_table[offset]);
    }*/

    currentPriority_ = THREAD_PRIORITY_MAX - 1;
    currentThread_ = nullptr;

    /* initialize ready priority group */
    threadReadyPriorityGroup_ = 0;

#if THREAD_PRIORITY_MAX > 32
    /* initialize ready table */
    memset(threadReadyTable_, 0, sizeof(threadReadyTable_));
#endif

    /* initialize thread defunct */
    //rt_list_init(&rt_thread_defunct);
}
/**
 * @ingroup SystemInit
 * This function will startup scheduler. It will select one thread
 * with the highest priority level, then switch to it.
 */
void mSchedule::systemSchedulerStart(void)
{
    register thread_t *toThread;
    register unsigned long highestReadyPriority;

#if THREAD_PRIORITY_MAX > 32
    register unsigned long number;

    number = mKservice::getInstance()->__rt_ffs(threadReadyPriorityGroup_) - 1;
    highestReadyPriority = (number << 3) + mKservice::getInstance()->__rt_ffs(threadReadyTable_[number]) - 1;
#else
    highestReadyPriority = mKservice::getInstance()->__rt_ffs(threadReadyPriorityGroup_) - 1;
#endif

    /* get switch to thread */
    toThread = listEntry(threadPriorityTable_[highestReadyPriority].next,
                            struct thread_t,
                            tlist);

    currentThread_ = toThread;

    /* switch to new thread */
    HW::hwContextSwitchTo((uint32_t)&toThread->sp);

    /* never come back */
}
/**
 * This function will perform one schedule. It will select one thread
 * with the highest priority level, then switch to it.
 */
void mSchedule::schedule(void)
{
    long level;
    thread_t *toThread = nullptr;
    thread_t *fromThread = nullptr;

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    /* check the scheduler is enabled or not */
    if (schedulerLockNest_ == 0)
    {
        register unsigned long highestReadyPriority;

#if THREAD_PRIORITY_MAX <= 32
        highestReadyPriority = mKservice::getInstance()->__rt_ffs(threadReadyPriorityGroup_) - 1;
#else
        register unsigned long number;

        number = mKservice::getInstance()->__rt_ffs(threadReadyPriorityGroup_) - 1;
        highestReadyPriority = (number << 3) + mKservice::getInstance()->__rt_ffs(threadReadyTable_[number]) - 1;
#endif

        /* get switch to thread */
        if(threadPriorityTable_[highestReadyPriority].isEmpty())
        {
            printf("Error: all thread sleep\r\n");
            printf("Error: do not use threadSleep in idle thread\r\n");
            printf("Error: do not use ipc func in interrupt\r\n");
            while(1);
        }
        toThread = listEntry(threadPriorityTable_[highestReadyPriority].next, struct thread_t, tlist);

        /* if the destination thread is not the same as current thread */
        if (toThread != currentThread_)
        {
            currentPriority_ = (uint8_t)highestReadyPriority;
            fromThread       = currentThread_;
            currentThread_   = toThread;

            //RT_OBJECT_HOOK_CALL(rt_scheduler_hook, (from_thread, toThread));

            /* switch to new thread */
            /*RT_DEBUG_LOG(RT_DEBUG_SCHEDULER,
                        ("[%d]switch to priority#%d "
                        "thread:%.*s(sp:0x%p), "
                        "from thread:%.*s(sp: 0x%p)\n",
                        mIrq::getInstance()->interruptGetNest(), highestReadyPriority,
                        NAME_MAX, toThread->name, toThread->sp,
                        NAME_MAX, fromThread->name, fromThread->sp));*/

#ifdef RT_USING_OVERFLOW_CHECK
            _rt_scheduler_stack_check(toThread);
#endif

            if (mIrq::getInstance()->getInterruptNestWithoutDisableIsr() == 0)
            {
                HW::hwContextSwitch((unsigned long)&fromThread->sp, (unsigned long)&toThread->sp);

                /* enable interrupt */
                HW::hwInterruptEnable(level);

                return ;
            }
            else
            {
                //RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("switch in interrupt\n"));
                HW::hwContextSwitchInterrupt((unsigned long)&fromThread->sp, (unsigned long)&toThread->sp);
            }
        }
    }
    /* enable interrupt */
    HW::hwInterruptEnable(level);
}
/*
* This function will insert a thread to system ready queue. The state of
* thread will be set as READY and remove from suspend queue.
*
* @param thread the thread to be inserted
* @note Please do not invoke this function in user application.
*/
void mSchedule::scheduleInsertThread(thread_t *thread)
{
    register long temp;

    MASSERT(thread != nullptr);

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /* change stat */
    thread->stat = THREAD_READY | (thread->stat & ~THREAD_STAT_MASK);

    /* insert thread to ready list */
    thread->tlist.insertBeforeTo(&(threadPriorityTable_[thread->currentPriority]));

    /* set priority mask */
#if THREAD_PRIORITY_MAX <= 32
    /*RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("insert thread[%.*s], the priority: %d\n",
                                    NAME_MAX, thread->name, thread->currentPriority));*/
#else
    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER,
                ("insert thread[%.*s], the priority: %d 0x%x %d\n",
                RT_NAME_MAX,
                thread->name,
                thread->number,
                thread->number_mask,
                thread->high_mask));
#endif

#if THREAD_PRIORITY_MAX > 32
    threadReadyTable[thread->number] |= thread->highMask;
#endif
    threadReadyPriorityGroup_ |= thread->numberMask;

    /* enable interrupt */
    HW::hwInterruptEnable(temp);
}
/*
* This function will remove a thread from system ready queue.
*
* @param thread the thread to be removed
*
* @note Please do not invoke this function in user application.
*/
void mSchedule::scheduleRemoveThread(thread_t *thread)
{
    register long temp;

    MASSERT(thread != nullptr);

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

#if THREAD_PRIORITY_MAX <= 32
    /*RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("remove thread[%.*s], the priority: %d\n",
                                    NAME_MAX, thread->name,
                                    thread->currentPriority));*/
#else
    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER,
                ("remove thread[%.*s], the priority: %d 0x%x %d\n",
                NAME_MAX,
                thread->name,
                thread->number,
                thread->numberMask,
                thread->highMask));
#endif

    /* remove thread from ready list */
    thread->tlist.removeSelf();
    if ((threadPriorityTable_[thread->currentPriority].isEmpty()))
    {
#if THREAD_PRIORITY_MAX > 32
        threadPriorityTable_[thread->number] &= ~thread->highMask;
        if (threadPriorityTable_[thread->number] == 0)
        {
            threadReadyPriorityGroup_ &= ~thread->numberMask;
        }
#else
        threadReadyPriorityGroup_ &= ~thread->numberMask;
#endif
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);
}

/**
 * This function will lock the thread scheduler.
 */
void mSchedule::enterCritical(void)
{
    register long level;

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    /*
    * the maximal number of nest is RT_UINT16_MAX, which is big
    * enough and does not check here
    */
    schedulerLockNest_ ++;

    /* enable interrupt */
    HW::hwInterruptEnable(level);
}

/**
 * This function will unlock the thread scheduler.
 */
void mSchedule::exitCritical(void)
{
    register long level;

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    schedulerLockNest_ --;
    if (schedulerLockNest_ <= 0)
    {
        schedulerLockNest_ = 0;
        /* enable interrupt */
        HW::hwInterruptEnable(level);

        if (currentThread_)
        {
            /* if scheduler is started, do a schedule */
            schedule();
        }
    }
    else
    {
        /* enable interrupt */
        HW::hwInterruptEnable(level);
    }
}

/**
 * Get the scheduler lock level
 *
 * @return the level of the scheduler lock. 0 means unlocked.
 */
uint16_t mSchedule::criticalLevel(void)
{
    return schedulerLockNest_;
}

thread_t*& mSchedule::getCurrentThread()
{
    return currentThread_;
}
mList_t* mSchedule::getThreadDefunct()
{
    return &threadDefunct_;
}
mList_t* mSchedule::getThreadPriorityTable(uint8_t priority)
{
    return &threadPriorityTable_[priority];
}