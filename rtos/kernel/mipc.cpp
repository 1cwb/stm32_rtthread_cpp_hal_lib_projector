#include "mipc.hpp"
#include "mthread.hpp"

/**
 * This function will initialize an IPC object
 *
 * @param ipc the IPC object
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult mIpc::ipcObjectInit(mIpcObject_t* ipcObj)
{
    return M_RESULT_EOK;
}
/**
 * This function will suspend a thread to a specified list. IPC object or some
 * double-queue object (mailbox etc.) contains this kind of list.
 *
 * @param list the IPC suspended thread list
 * @param thread the thread object to be suspended
 * @param flag the IPC object flag,
 *        which shall be RT_IPC_FLAG_FIFO/RT_IPC_FLAG_PRIO.
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult  mIpc::ipcListSuspend(mIpcObject_t* ipcObj, struct thread_t *thread, mIpcFlag flag)
{
    /* suspend thread */
    mthread::threadSuspend(thread);
    switch (flag)
    {
    case IPC_FLAG_FIFO:
        thread->tlist.insertBeforeTo(&ipcObj->suspendThread);
        break;

    case IPC_FLAG_PRIO:
        {
            struct mList_t *n;
            struct thread_t *sthread;

            /* find a suitable position */
            for (n = ipcObj->suspendThread.next; n != &ipcObj->suspendThread; n = n->next)
            {
                sthread = listEntry(n, struct thread_t, tlist);

                /* find out */
                if (thread->currentPriority < sthread->currentPriority)
                {
                    /* insert this thread before the sthread */
                    thread->tlist.insertBeforeTo(&(sthread->tlist));
                    break;
                }
            }

            /*
            * not found a suitable position,
            * append to the end of suspend_thread list
            */
            if (n == &ipcObj->suspendThread)
            {
                thread->tlist.insertBeforeTo(&ipcObj->suspendThread);
            }
        }
        break;

    default:
        break;
    }

    return M_RESULT_EOK;
}
mResult  mIpc::ipcListSuspend(mList_t* list, struct thread_t *thread, mIpcFlag flag)
{
    /* suspend thread */
    mthread::threadSuspend(thread);

    switch (flag)
    {
    case IPC_FLAG_FIFO:
        thread->tlist.insertBeforeTo(list);
        break;

    case IPC_FLAG_PRIO:
        {
            struct mList_t *n;
            struct thread_t *sthread;

            /* find a suitable position */
            for (n = list->next; n != list; n = n->next)
            {
                sthread = listEntry(n, struct thread_t, tlist);

                /* find out */
                if (thread->currentPriority < sthread->currentPriority)
                {
                    /* insert this thread before the sthread */
                    thread->tlist.insertBeforeTo(&sthread->tlist);
                    break;
                }
            }

            /*
             * not found a suitable position,
             * append to the end of suspend_thread list
             */
            if (n == list)
            {
                thread->tlist.insertBeforeTo(list);
            }
        }
        break;

    default:
        break;
    }
    return M_RESULT_EOK;
}
/**
 * This function will resume the first thread in the list of a IPC object:
 * - remove the thread from suspend queue of IPC object
 * - put the thread into system ready queue
 *
 * @param list the thread list
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult  mIpc::ipcListResume(mIpcObject_t* ipcObj)
{
    struct thread_t *thread;

    /* get thread entry */
    thread = listEntry(ipcObj->suspendThread.next, struct thread_t, tlist);

    //RT_DEBUG_LOG(RT_DEBUG_IPC, ("resume thread:%s\n", thread->name));

    /* resume it */
    reinterpret_cast<mthread*>(thread)->threadResume();
    return M_RESULT_EOK;
}
mResult  mIpc::ipcListResume(mList_t* list)
{
    struct thread_t *thread;

    /* get thread entry */
    thread = listEntry(list->next, struct thread_t, tlist);

    //RT_DEBUG_LOG(RT_DEBUG_IPC, ("resume thread:%s\n", thread->name));

    /* resume it */
    reinterpret_cast<mthread*>(thread)->threadResume();
    return M_RESULT_EOK;
}
/**
 * This function will resume all suspended threads in a list, including
 * suspend list of IPC object and private list of mailbox etc.
 *
 * @param list of the threads to resume
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult mIpc::ipcListResumeAll(mIpcObject_t* ipcObj)
{
    struct thread_t *thread;
    register unsigned long temp;

    /* wakeup all suspended threads */
    while (!ipcObj->suspendThread.isEmpty())
    {
        /* disable interrupt */
        temp = HW::hwInterruptDisable();

        /* get next suspended thread */
        thread = listEntry(ipcObj->suspendThread.next, struct thread_t, tlist);
        /* set error code to RT_ERROR */
        thread->error = M_RESULT_ERROR;

        /*
        * resume thread
        * In rt_thread_resume function, it will remove current thread from
        * suspended list
        */
        reinterpret_cast<mthread*>(thread)->threadResume();

        /* enable interrupt */
        HW::hwInterruptEnable(temp);
    }

    return M_RESULT_EOK;
}

mResult mIpc::ipcListResumeAll(mList_t *list)
{
    struct thread_t *thread;
    register long temp;

    /* wakeup all suspended threads */
    while (!list->isEmpty())
    {
        /* disable interrupt */
        temp = HW::hwInterruptDisable();

        /* get next suspended thread */
        thread = listEntry(list->next, struct thread_t, tlist);
        /* set error code to RT_ERROR */
        thread->error = M_RESULT_ERROR;

        /*
        * resume thread
        * In rt_thread_resume function, it will remove current thread from
        * suspended list
        */
        reinterpret_cast<mthread*>(thread)->threadResume();

        /* enable interrupt */
        HW::hwInterruptEnable(temp);
    }

    return M_RESULT_EOK;
}
/**
 * This function will create a semaphore from system resource
 *
 * @param name the name of semaphore
 * @param value the initial value of semaphore
 * @param flag the flag of semaphore
 *
 * @return the created semaphore, nullptr on error happen
 *
 * @see rt_sem_init
 */
mResult mSemaphore::semCreateEx(const char *name, uint32_t value, mIpcFlag flag)
{
    //RT_DEBUG_NOT_IN_INTERRUPT;
    MASSERT(value < 0x10000U);

    /* initialize ipc object */
    mObject::getInstance()->objectAdd((mObject_t*)this, M_OBJECT_CLASS_SEMAPHORE, name);
    /* set initial value */
    sem_.value = value;

    /* set parent */
    sem_.flag = flag;

    return M_RESULT_EOK;
}

/**
 * This function will delete a semaphore object and release the memory
 *
 * @param sem the semaphore object
 *
 * @return the error code
 *
 * @see rt_sem_detach
 */
mResult mSemaphore::semDelete()
{
    //RT_DEBUG_NOT_IN_INTERRUPT;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_SEMAPHORE);
    MASSERT(!mObject::getInstance()->objectIsSystemobject((mObject_t*)this));

    if(mObject::getInstance()->objectIsSystemobject((mObject_t*)this))
    {
        return M_RESULT_ERROR;
    }
    /* wakeup all suspended threads */
    ipcListResumeAll(reinterpret_cast<mIpcObject_t*>(this));

    /* delete semaphore object */
    delete this;

    return M_RESULT_EOK;
}

    /**
 * This function will initialize a semaphore and put it under control of
 * resource management.
 *
 * @param sem the semaphore object
 * @param name the name of semaphore
 * @param value the initial value of semaphore
 * @param flag the flag of semaphore
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult  mSemaphore::init(const char *name, uint32_t value, mIpcFlag flag)
{
    MASSERT(value < 0x10000U);

    /* initialize object */
    mObject::getInstance()->objectInit((mObject_t*)(this),M_OBJECT_CLASS_SEMAPHORE, name);

    /* initialize ipc object */
    ipcObjectInit(reinterpret_cast<mIpcObject_t*>(&sem_));

    /* set initial value */
    sem_.value = (uint16_t)value;

    /* set parent */
    sem_.flag = flag;
    return M_RESULT_EOK;
}
/**
 * This function will detach a semaphore from resource management
 *
 * @param sem the semaphore object
 *
 * @return the operation status, M_RESULT_EOK on successful
 *
 * @see rt_sem_delete
 */
mResult  mSemaphore::detach()
{
    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)&sem_) == M_OBJECT_CLASS_SEMAPHORE);
    MASSERT(mObject::getInstance()->objectIsSystemobject((mObject_t*)&sem_));

    /* wakeup all suspended threads */
    ipcListResumeAll(reinterpret_cast<mIpcObject_t*>(this));

    /* detach semaphore object */
    mObject::getInstance()->objectDetach((mObject_t*)this);
    return M_RESULT_EOK;
}
/**
 * This function will take a semaphore, if the semaphore is unavailable, the
 * thread shall wait for a specified time.
 *
 * @param sem the semaphore object
 * @param time the waiting time
 *
 * @return the error code
 */
mResult  mSemaphore::semTake(int32_t time)
{
    register long temp;
    struct thread_t *thread;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)&sem_) == M_OBJECT_CLASS_SEMAPHORE);

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("thread %s take sem:%s, which value is: %d\n",
                                rt_thread_self()->name,
                                ((struct rt_object *)sem)->name,
                                sem->value));*/

    if (sem_.value > 0)
    {
        /* semaphore is available */
        sem_.value --;

        /* enable interrupt */
        HW::hwInterruptEnable(temp);
    }
    else
    {
        /* no waiting, return with timeout */
        if (time == 0)
        {
            HW::hwInterruptEnable(temp);

            return M_RESULT_ETIMEOUT;
        }
        else
        {
            /* current context checking */
            //RT_DEBUG_IN_THREAD_CONTEXT;

            /* semaphore is unavailable, push to suspend list */
            /* get current thread */
            thread = (thread_t*)mthread::threadSelf();

            /* reset thread error number */
            thread->error = M_RESULT_EOK;

            /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("sem take: suspend thread - %s\n",
                                        thread->name));*/

            /* suspend thread */
            ipcListSuspend(reinterpret_cast<mIpcObject_t*>(&sem_), thread, (mIpcFlag)(sem_.flag));

            /* has waiting time, start thread timer */
            if (time > 0)
            {
                /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("set thread:%s to timer list\n",
                                            thread->name));*/

                /* reset the timeout of thread timer and start it */
                reinterpret_cast<mthread*>(thread)->getThTimer()->setTimerAndStart(time);
            }

            /* enable interrupt */
            HW::hwInterruptEnable(temp);

            /* do schedule */
            mSchedule::getInstance()->schedule();

            if (thread->error != M_RESULT_EOK)
            {
                return (mResult)thread->error;
            }
        }
    }

    return M_RESULT_EOK;
}

/**
 * This function will try to take a semaphore and immediately return
 *
 * @param sem the semaphore object
 *
 * @return the error code
 */
mResult  mSemaphore::semTrytake()
{
    return semTake(0);
}

/**
 * This function will release a semaphore, if there are threads suspended on
 * semaphore, it will be waked up.
 *
 * @param sem the semaphore object
 *
 * @return the error code
 */
mResult  mSemaphore::semRelease()
{
    register long temp;
    register bool needSchedule;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)&sem_) == M_OBJECT_CLASS_SEMAPHORE);

    needSchedule = false;

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("thread %s releases sem:%s, which value is: %d\n",
                                rt_thread_self()->name,
                                ((struct rt_object *)sem)->name,
                                sem->value));*/

    if (!sem_.suspendThread.isEmpty())
    {
        /* resume the suspended thread */
        ipcListResume((mIpcObject_t*)&sem_);
        needSchedule = true;
    }
    else
    {
        if(sem_.value < SEM_VALUE_MAX)
        {
            sem_.value ++; /* increase value */
        }
        else
        {
            HW::hwInterruptEnable(temp); /* enable interrupt */
            return M_RESULT_EFULL; /* value overflowed */
        }
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    /* resume a thread, re-schedule */
    if (needSchedule)
    {
        mSchedule::getInstance()->schedule();
    }
    return M_RESULT_EOK;
}

/**
 * This function can get or set some extra attributions of a semaphore object.
 *
 * @param sem the semaphore object
 * @param cmd the execution command
 * @param arg the execution argument
 *
 * @return the error code
 */
mResult  mSemaphore::semControl(mIpcCmd cmd, void *arg)
{
    long level;

    /* parameter check */

    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)&sem_) == M_OBJECT_CLASS_SEMAPHORE);

    if (cmd == IPC_CMD_RESET)
    {
        unsigned long value;

        /* get value */
        value = (unsigned long)arg;
        /* disable interrupt */
        level = HW::hwInterruptDisable();

        /* resume all waiting thread */
        ipcListResumeAll((mIpcObject_t*)this);

        /* set new value */
        sem_.value = (uint16_t)value;

        /* enable interrupt */
        HW::hwInterruptEnable(level);

        mSchedule::getInstance()->schedule();

        return M_RESULT_EOK;
    }

    return M_RESULT_ERROR;
}

/**
 * This function will initialize a mutex and put it under control of resource
 * management.
 *
 * @param mutex the mutex object
 * @param name the name of mutex
 * @param flag the flag of mutex
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult mMutex::init(const char *name, mIpcFlag flag)
{
    /* initialize object */
    mObject::getInstance()->objectInit((mObject_t*)(this),M_OBJECT_CLASS_MUTEX, name);

    /* initialize ipc object */
    ipcObjectInit((mIpcObject_t*) (this));

    mutex_.value = 1;
    mutex_.owner = nullptr;
    mutex_.originalPriority = 0xFF;
    mutex_.hold = 0;

    /* set flag */
    mutex_.flag = flag;

    return M_RESULT_EOK;
}
/**
 * This function will detach a mutex from resource management
 *
 * @param mutex the mutex object
 *
 * @return the operation status, M_RESULT_EOK on successful
 *
 * @see rt_mutex_delete
 */
mResult mMutex::detach()
{
    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MUTEX);
    MASSERT(mObject::getInstance()->objectIsSystemobject((mObject_t*)this));

    /* wakeup all suspended threads */
    ipcListResumeAll((mIpcObject_t*)this);

    /* detach semaphore object */
    mObject::getInstance()->objectDetach((mObject_t*)(this));

    return M_RESULT_EOK;
}
/**
 * This function will take a mutex, if the mutex is unavailable, the
 * thread shall wait for a specified time.
 *
 * @param mutex the mutex object
 * @param time the waiting time
 *
 * @return the error code
 */
mResult mMutex::mutexTake(int32_t time)
{
    register long temp;
    struct thread_t *thread;

    /* this function must not be used in interrupt even if time = 0 */
    //RT_DEBUG_IN_THREAD_CONTEXT;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MUTEX);

    /* get current thread */
    thread = mthread::threadSelf();

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /*RT_DEBUG_LOG(RT_DEBUG_IPC,
                 ("mutex_take: current thread %s, mutex value: %d, hold: %d\n",
                  thread->name, mutex->value, mutex->hold));*/

    /* reset thread error */
    thread->error = M_RESULT_EOK;

    if (mutex_.owner == thread)
    {
        if(mutex_.hold < MUTEX_HOLD_MAX)
        {
            /* it's the same thread */
            mutex_.hold ++;
        }
        else
        {
            HW::hwInterruptEnable(temp); /* enable interrupt */
            return M_RESULT_EFULL; /* value overflowed */
        }
    }
    else
    {
        /* The value of mutex is 1 in initial status. Therefore, if the
         * value is great than 0, it indicates the mutex is avaible.
         */
        if (mutex_.value > 0)
        {
            /* mutex is available */
            mutex_.value --;

            /* set mutex owner and original priority */
            mutex_.owner             = thread;
            mutex_.originalPriority = thread->currentPriority;
            if(mutex_.hold < MUTEX_HOLD_MAX)
            {
                mutex_.hold ++;
            }
            else
            {
                HW::hwInterruptEnable(temp); /* enable interrupt */
                return M_RESULT_EFULL; /* value overflowed */
            }
        }
        else
        {
            /* no waiting, return with timeout */
            if (time == 0)
            {
                /* set error as timeout */
                thread->error = M_RESULT_ETIMEOUT;

                /* enable interrupt */
                HW::hwInterruptEnable(temp);

                return M_RESULT_EFULL;
            }
            else
            {
                /* mutex is unavailable, push to suspend list */
                /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("mutex_take: suspend thread: %s\n",
                                            thread->name));*/

                /* change the owner thread priority of mutex */
                if (thread->currentPriority < mutex_.owner->currentPriority)
                {
                    /* change the owner thread priority */
                    reinterpret_cast<mthread*>(mutex_.owner)->threadControl(THREAD_CTRL_CHANGE_PRIORITY, &thread->currentPriority);
                }

                /* suspend current thread */
                ipcListSuspend((mIpcObject_t*)this , thread, (mIpcFlag)mutex_.flag);
                
                /* has waiting time, start thread timer */
                if (time > 0)
                {
                    /*RT_DEBUG_LOG(RT_DEBUG_IPC,
                                 ("mutex_take: start the timer of thread:%s\n",
                                  thread->name));*/

                    /* reset the timeout of thread timer and start it */
                    reinterpret_cast<mthread*>(thread)->getThTimer()->setTimerAndStart(time);
                }

                /* enable interrupt */
                HW::hwInterruptEnable(temp);

                /* do schedule */
                mSchedule::getInstance()->schedule();

                if (thread->error != M_RESULT_EOK)
                {
                    /* return error */
                    return thread->error;
                }
                else
                {
                    /* the mutex is taken successfully. */
                    /* disable interrupt */
                    temp = HW::hwInterruptDisable();
                }
            }
        }
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    //RT_OBJECT_HOOK_CALL(rt_object_take_hook, (&(mutex->parent.parent)));

    return M_RESULT_EOK;
}

/**
 * This function will release a mutex, if there are threads suspended on mutex,
 * it will be waked up.
 *
 * @param mutex the mutex object
 *
 * @return the error code
 */
mResult mMutex::mutexRelease()
{
    register long temp;
    struct thread_t *thread;
    bool needSchedule;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MUTEX);

    needSchedule = false;

    /* only thread could release mutex because we need test the ownership */
    //RT_DEBUG_IN_THREAD_CONTEXT;

    /* get current thread */
    thread = mthread::threadSelf();

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /*RT_DEBUG_LOG(RT_DEBUG_IPC,
                 ("mutex_release:current thread %s, mutex value: %d, hold: %d\n",
                  thread->name, mutex->value, mutex->hold));*/

    //RT_OBJECT_HOOK_CALL(rt_object_put_hook, (&(mutex->parent.parent)));

    /* mutex only can be released by owner */
    if (thread != mutex_.owner)
    {
        thread->error = M_RESULT_ERROR;

        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        return M_RESULT_ERROR;
    }

    /* decrease hold */
    mutex_.hold --;
    /* if no hold */
    if (mutex_.hold == 0)
    {
        /* change the owner thread to original priority */
        if (mutex_.originalPriority != mutex_.owner->currentPriority)
        {
            reinterpret_cast<mthread*>(mutex_.owner)->threadControl(THREAD_CTRL_CHANGE_PRIORITY, &mutex_.originalPriority);
        }

        /* wakeup suspended thread */
        if (!mutex_.suspendThread.isEmpty())
        {
            /* get suspended thread */
            thread = listEntry(mutex_.suspendThread.next,
                                   struct thread_t,
                                   tlist);

            /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("mutex_release: resume thread: %s\n",
                                        thread->name));*/

            /* set new owner and priority */
            mutex_.owner             = thread;
            mutex_.originalPriority = thread->currentPriority;
            if(mutex_.hold < MUTEX_HOLD_MAX)
            {
                mutex_.hold ++;
            }
            else
            {
                HW::hwInterruptEnable(temp); /* enable interrupt */
                return M_RESULT_EFULL; /* value overflowed */
            }

            /* resume thread */
            ipcListResume((mIpcObject_t*)(this));

            needSchedule = true;
        }
        else
        {
            if(mutex_.value < MUTEX_VALUE_MAX)
            {
                /* increase value */
                mutex_.value ++;
            }
            else
            {
                HW::hwInterruptEnable(temp); /* enable interrupt */
                return M_RESULT_EFULL; /* value overflowed */
            }

            /* clear owner */
            mutex_.owner             = nullptr;
            mutex_.originalPriority = 0xff;
        }
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    /* perform a schedule */
    if (needSchedule == true)
    {
        mSchedule::getInstance()->schedule();
    }

    return M_RESULT_EOK;
}

/**
 * This function will initialize an event and put it under control of resource
 * management.
 *
 * @param event the event object
 * @param name the name of event
 * @param flag the flag of event
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult mEvent::init(const char *name, mIpcFlag flag)
{
    /* initialize object */
    mObject::getInstance()->objectInit((mObject_t*)this, M_OBJECT_CLASS_EVENT, name);

    /* set parent flag */
    event_.flag = flag;

    /* initialize ipc object */
    ipcObjectInit((mIpcObject_t*) this);

    /* initialize event */
    event_.set = 0;

    return M_RESULT_EOK;
}

/**
 * This function will detach an event object from resource management
 *
 * @param event the event object
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult mEvent::detach()
{
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_EVENT);
    MASSERT(mObject::getInstance()->objectIsSystemobject((mObject_t*)this));

    /* resume all suspended thread */
    ipcListResumeAll((mIpcObject_t*)this);
    /* detach event object */
    mObject::getInstance()->objectDetach((mObject_t*)this);

    return M_RESULT_EOK;
}

/**
 * This function will send an event to the event object, if there are threads
 * suspended on event object, it will be waked up.
 *
 * @param event the event object
 * @param set the event set
 *
 * @return the error code
 */
mResult mEvent::send(uint32_t set)
{
    struct mList_t *n;
    struct thread_t *thread;
    register unsigned long level;
    register long status;
    bool needSchedule;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_EVENT);

    if (set == 0)
    {
        return M_RESULT_ERROR;
    }

    needSchedule = false;

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    /* set event */
    event_.set |= set;

    //RT_OBJECT_HOOK_CALL(rt_object_put_hook, (&(event_.parent.parent)));

    if (!event_.suspendThread.isEmpty())
    {
        /* search thread list to resume thread */
        n = event_.suspendThread.next;
        while (n != &(event_.suspendThread))
        {
            /* get thread */
            thread = listEntry(n, struct thread_t, tlist);

            status = M_RESULT_ERROR;
            if (thread->eventInfo & EVENT_FLAG_AND)
            {
                if ((thread->eventSet & event_.set) == thread->eventSet)
                {
                    /* received an AND event */
                    status = M_RESULT_EOK;
                }
            }
            else if (thread->eventInfo & EVENT_FLAG_OR)
            {
                if (thread->eventSet & event_.set)
                {
                    /* save the received event set */
                    thread->eventSet = thread->eventSet & event_.set;

                    /* received an OR event */
                    status = M_RESULT_EOK;
                }
            }
            else
            {
                /* enable interrupt */
                HW::hwInterruptEnable(level);

                return M_RESULT_EINVAL;
            }

            /* move node to the next */
            n = n->next;

            /* condition is satisfied, resume thread */
            if (status == M_RESULT_EOK)
            {
                /* clear event */
                if (thread->eventInfo & EVENT_FLAG_CLEAR)
                {
                    event_.set &= ~thread->eventSet;
                }

                /* resume thread, and thread list breaks out */
                reinterpret_cast<mthread*>(thread)->threadResume();

                /* need do a scheduling */
                needSchedule = true;
            }
        }
    }

    /* enable interrupt */
    HW::hwInterruptEnable(level);

    /* do a schedule */
    if (needSchedule == true)
    {
        mSchedule::getInstance()->schedule();
    }

    return M_RESULT_EOK;
}

/**
 * This function will receive an event from event object, if the event is
 * unavailable, the thread shall wait for a specified time.
 *
 * @param event the fast event object
 * @param set the interested event set
 * @param option the receive option, either RT_EVENT_FLAG_AND or
 *        RT_EVENT_FLAG_OR should be set.
 * @param timeout the waiting time
 * @param recved the received event, if you don't care, nullptr can be set.
 *
 * @return the error code
 */
mResult mEvent::recv(uint32_t  set,
                    uint8_t   option,
                    int32_t   timeout,
                    uint32_t *recved)
{
    struct thread_t *thread;
    register long level;
    register long status;

    //RT_DEBUG_IN_THREAD_CONTEXT;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_EVENT);

    if (set == 0)
    {
        return M_RESULT_ERROR;
    }

    /* initialize status */
    status = M_RESULT_ERROR;
    /* get current thread */
    thread = mthread::threadSelf();
    /* reset thread error */
    thread->error = M_RESULT_EOK;

    //RT_OBJECT_HOOK_CALL(rt_object_trytake_hook, (&(event_.parent.parent)));

    /* disable interrupt */
    level = HW::hwInterruptDisable();

    /* check event set */
    if (option & EVENT_FLAG_AND)
    {
        if ((event_.set & set) == set)
        {
            status = M_RESULT_EOK;
        }
    }
    else if (option & EVENT_FLAG_OR)
    {
        if (event_.set & set)
        {
            status = M_RESULT_EOK;
        }
    }
    else
    {
        /* either RT_EVENT_FLAG_AND or RT_EVENT_FLAG_OR should be set */
        MASSERT(0);
    }

    if (status == M_RESULT_EOK)
    {
        /* set received event */
        if (recved)
        {
            *recved = (event_.set & set);
        }

        /* fill thread event info */
        thread->eventSet = (event_.set & set);
        thread->eventInfo = option;

        /* received event */
        if (option & EVENT_FLAG_CLEAR)
        {
            event_.set &= ~set;
        }
    }
    else if (timeout == 0)
    {
        /* no waiting */
        thread->error = M_RESULT_ETIMEOUT;

        /* enable interrupt */
        HW::hwInterruptEnable(level);

        return M_RESULT_ETIMEOUT;
    }
    else
    {
        /* fill thread event info */
        thread->eventSet  = set;
        thread->eventInfo = option;

        /* put thread to suspended thread list */
        ipcListSuspend((mIpcObject_t*)this,
                            thread,
                            (mIpcFlag)event_.flag);

        /* if there is a waiting timeout, active thread timer */
        if (timeout > 0)
        {
            /* reset the timeout of thread timer and start it */
            reinterpret_cast<mthread*>(thread)->getThTimer()->setTimerAndStart(timeout);
        }

        /* enable interrupt */
        HW::hwInterruptEnable(level);

        /* do a schedule */
        mSchedule::getInstance()->schedule();

        if (thread->error != M_RESULT_EOK)
        {
            /* return error */
            return thread->error;
        }

        /* received an event, disable interrupt to protect */
        level = HW::hwInterruptDisable();

        /* set received event */
        if (recved)
        {
            *recved = thread->eventSet;
        }
    }

    /* enable interrupt */
    HW::hwInterruptEnable(level);

    //RT_OBJECT_HOOK_CALL(rt_object_take_hook, (&(event_.parent.parent)));

    return thread->error;
}

/**
 * This function can get or set some extra attributions of an event object.
 *
 * @param event the event object
 * @param cmd the execution command
 * @param arg the execution argument
 *
 * @return the error code
 */
mResult mEvent::control(mIpcCmd cmd, void *arg)
{
    long level;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_EVENT);

    if (cmd == IPC_CMD_RESET)
    {
        /* disable interrupt */
        level = HW::hwInterruptDisable();

        /* resume all waiting thread */
        ipcListResumeAll((mIpcObject_t*)this);

        /* initialize event set */
        event_.set = 0;

        /* enable interrupt */
        HW::hwInterruptEnable(level);

        mSchedule::getInstance()->schedule();

        return M_RESULT_EOK;
    }

    return M_RESULT_ERROR;
}

/**
 * This function will initialize a message queue and put it under control of
 * resource management.
 *
 * @param mq the message object
 * @param name the name of message queue
 * @param msgpool the beginning address of buffer to save messages
 * @param msg_size the maximum size of message
 * @param pool_size the size of buffer to save messages
 * @param flag the flag of message queue
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult mMessagequeue::init(const char *name,
                uint32_t   msgSize,
                uint32_t   poolSize,
                mIpcFlag   flag)
{
    if(bInited_)
    {
        return M_RESULT_ERROR;
    }
    struct mqMessage_t *head;
    register long temp;

    /* initialize object */
    mObject::getInstance()->objectInit((mObject_t*)this, M_OBJECT_CLASS_MESSAGEQUEUE, name);

    /* set parent flag */
    msg_.flag = flag;

    /* initialize ipc object */
    ipcObjectInit((mIpcObject_t*)this);

    /* set message pool */
    msg_.msgPool = new uint8_t[poolSize];
    if(!msg_.msgPool)
    {
        return M_RESULT_ERROR;
    }
    /* get correct message size */
    msg_.msgSize = M_ALIGN(msgSize, M_ALIGN_SIZE);
    msg_.maxMsgs = poolSize / (msg_.msgSize + sizeof(struct mqMessage_t));

    /* initialize message list */
    msg_.msgQueueHead = nullptr;
    msg_.msgQueueTail = nullptr;

    /* initialize message empty list */
    msg_.msgQueueFree = nullptr;
    for (temp = 0; temp < msg_.maxMsgs; temp ++)
    {
        head = (struct mqMessage_t *)((uint8_t *)msg_.msgPool +
                                        temp * (msg_.msgSize + sizeof(struct mqMessage_t)));
        head->next = (struct mqMessage_t *)msg_.msgQueueFree;
        msg_.msgQueueFree = head;
    }

    /* the initial entry is zero */
    msg_.entry = 0;
    bInited_ = true;
    return M_RESULT_EOK;
}
/**
 * This function will detach a message queue object from resource management
 *
 * @param mq the message queue object
 *
 * @return the operation status, M_RESULT_EOK on successful
 */
mResult mMessagequeue::detach()
{
    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MESSAGEQUEUE);
    MASSERT(mObject::getInstance()->objectIsSystemobject((mObject_t*)this));

    /* resume all suspended thread */
    ipcListResumeAll((mIpcObject_t*)this);
    /* also resume all message queue private suspended thread */
    
    ipcListResumeAll(&msg_.suspendSenderThread);

    /* detach message queue object */
    mObject::getInstance()->objectDetach((mIpcObject_t*)this);
    if(msg_.msgPool)
    {
        delete [] msg_.msgPool;
        msg_.msgPool = nullptr;
    }
    bInited_ = false;
    return M_RESULT_EOK;
}

/**
 * This function will send a message to message queue object. If the message queue is full,
 * current thread will be suspended until timeout.
 *
 * @param mq the message queue object
 * @param buffer the message
 * @param size the size of buffer
 * @param timeout the waiting time
 *
 * @return the error code
 */
mResult mMessagequeue::sendWait(const void *buffer, uint32_t size, int32_t timeout)
{
    if(!bInited_)
    {
        return M_RESULT_ERROR;
    }
    register long temp;
    struct mqMessage_t *msg;
    uint32_t tickDelta;
    struct thread_t *thread;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MESSAGEQUEUE);
    MASSERT(buffer != nullptr);
    MASSERT(size != 0);

    /* greater than one message size */
    if (size > msg_.msgSize)
    {
        return M_RESULT_ERROR;
    }

    /* initialize delta tick */
    tickDelta = 0;
    /* get current thread */
    thread = mthread::threadSelf();

    //RT_OBJECT_HOOK_CALL(rt_object_put_hook, (&(msg_.parent.parent)));

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /* get a free list, there must be an empty item */
    msg = (struct mqMessage_t *)msg_.msgQueueFree;
    /* for non-blocking call */
    if (msg == nullptr && timeout == 0)
    {
        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        return M_RESULT_EFULL;
    }

    /* message queue is full */
    while ((msg = (struct mqMessage_t *)msg_.msgQueueFree) == nullptr)
    {
        /* reset error number in thread */
        thread->error = M_RESULT_EOK;

        /* no waiting, return timeout */
        if (timeout == 0)
        {
            /* enable interrupt */
            HW::hwInterruptEnable(temp);

            return M_RESULT_EFULL;
        }

        //RT_DEBUG_IN_THREAD_CONTEXT;
        /* suspend current thread */
        ipcListSuspend(&(msg_.suspendSenderThread), thread, (mIpcFlag)msg_.flag);

        /* has waiting time, start thread timer */
        if (timeout > 0)
        {
            /* get the start tick of timer */
            tickDelta = mClock::getInstance()->tickGet();

            /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("mq_send_wait: start timer of thread:%s\n",
                                        thread->name));*/

            /* reset the timeout of thread timer and start it */
            reinterpret_cast<mthread*>(thread)->getThTimer()->setTimerAndStart(timeout);
        }

        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        /* re-schedule */
        mSchedule::getInstance()->schedule();

        /* resume from suspend state */
        if (thread->error != M_RESULT_EOK)
        {
            /* return error */
            return thread->error;
        }

        /* disable interrupt */
        temp = HW::hwInterruptDisable();

        /* if it's not waiting forever and then re-calculate timeout tick */
        if (timeout > 0)
        {
            tickDelta = mClock::getInstance()->tickGet() - tickDelta;
            timeout -= tickDelta;
            if (timeout < 0)
                timeout = 0;
        }
    }

    /* move free list pointer */
    msg_.msgQueueFree = msg->next;

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    /* the msg is the new tailer of list, the next shall be NULL */
    msg->next = nullptr;
    /* copy buffer */
    memcpy(msg + 1, buffer, size);

    /* disable interrupt */
    temp = HW::hwInterruptDisable();
    /* link msg to message queue */
    if (msg_.msgQueueTail != nullptr)
    {
        /* if the tail exists, */
        ((struct mqMessage_t *)msg_.msgQueueTail)->next = msg;
    }

    /* set new tail */
    msg_.msgQueueTail = msg;
    /* if the head is empty, set head */
    if (msg_.msgQueueHead == nullptr)
    {
        msg_.msgQueueHead = msg;
    }

    if(msg_.entry < MQ_ENTRY_MAX)
    {
        /* increase message entry */
        msg_.entry ++;
    }
    else
    {
        HW::hwInterruptEnable(temp); /* enable interrupt */
        return M_RESULT_EFULL; /* value overflowed */
    }

    /* resume suspended thread */
    if (!msg_.suspendThread.isEmpty())
    {
        ipcListResume((mIpcObject_t*)this);

        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        mSchedule::getInstance()->schedule();

        return M_RESULT_EOK;
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    return M_RESULT_EOK;
}
/**
 * This function will send a message to message queue object, if there are
 * threads suspended on message queue object, it will be waked up.
 *
 * @param mq the message queue object
 * @param buffer the message
 * @param size the size of buffer
 *
 * @return the error code
 */
mResult mMessagequeue::send(const void *buffer, uint32_t size)
{
    return sendWait(buffer, size, 0);
}
/**
 * This function will send an urgent message to message queue object, which
 * means the message will be inserted to the head of message queue. If there
 * are threads suspended on message queue object, it will be waked up.
 *
 * @param mq the message queue object
 * @param buffer the message
 * @param size the size of buffer
 *
 * @return the error code
 */
mResult mMessagequeue::urgent(const void *buffer, uint32_t size)
{
    register long temp;
    struct mqMessage_t *msg;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MESSAGEQUEUE);
    MASSERT(buffer != nullptr);
    MASSERT(size != 0);

    /* greater than one message size */
    if (size > msg_.msgSize)
        return M_RESULT_ERROR;

    //RT_OBJECT_HOOK_CALL(rt_object_put_hook, (&(msg_.parent.parent)));

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /* get a free list, there must be an empty item */
    msg = (struct mqMessage_t *)msg_.msgQueueFree;
    /* message queue is full */
    if (msg == nullptr)
    {
        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        return M_RESULT_EFULL;
    }
    /* move free list pointer */
    msg_.msgQueueFree = msg->next;

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    /* copy buffer */
    memcpy(msg + 1, buffer, size);

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /* link msg to the beginning of message queue */
    msg->next = (struct mqMessage_t *)msg_.msgQueueHead;
    msg_.msgQueueHead = msg;

    /* if there is no tail */
    if (msg_.msgQueueTail == nullptr)
    {
        msg_.msgQueueTail = msg;
    }

    if(msg_.entry < MQ_ENTRY_MAX)
    {
        /* increase message entry */
        msg_.entry ++;
    }
    else
    {
        HW::hwInterruptEnable(temp); /* enable interrupt */
        return M_RESULT_EFULL; /* value overflowed */
    }

    /* resume suspended thread */
    if (!msg_.suspendThread.isEmpty())
    {
        ipcListResume((mIpcObject_t*)this);

        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        mSchedule::getInstance()->schedule();

        return M_RESULT_EOK;
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    return M_RESULT_EOK;
}

/**
 * This function will receive a message from message queue object, if there is
 * no message in message queue object, the thread shall wait for a specified
 * time.
 *
 * @param mq the message queue object
 * @param buffer the received message will be saved in
 * @param size the size of buffer
 * @param timeout the waiting time
 *
 * @return the error code
 */
mResult mMessagequeue::recv( void *buffer, uint32_t  size, int32_t timeout)
{
    struct thread_t *thread;
    register long temp;
    struct mqMessage_t *msg;
    uint32_t tickDelta;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MESSAGEQUEUE);
    MASSERT(buffer != nullptr);
    MASSERT(size != 0);

    /* initialize delta tick */
    tickDelta = 0;
    /* get current thread */
    thread = mthread::threadSelf();
    //RT_OBJECT_HOOK_CALL(rt_object_trytake_hook, (&(msg_.parent.parent)));

    /* disable interrupt */
    temp = HW::hwInterruptDisable();

    /* for non-blocking call */
    if (msg_.entry == 0 && timeout == 0)
    {
        HW::hwInterruptEnable(temp);

        return M_RESULT_ETIMEOUT;
    }

    /* message queue is empty */
    while (msg_.entry == 0)
    {
        //RT_DEBUG_IN_THREAD_CONTEXT;

        /* reset error number in thread */
        thread->error = M_RESULT_EOK;

        /* no waiting, return timeout */
        if (timeout == 0)
        {
            /* enable interrupt */
            HW::hwInterruptEnable(temp);

            thread->error = M_RESULT_ETIMEOUT;

            return M_RESULT_ETIMEOUT;
        }

        /* suspend current thread */
        ipcListSuspend((mIpcObject_t*)this, thread, (mIpcFlag)msg_.flag);

        /* has waiting time, start thread timer */
        if (timeout > 0)
        {
            /* get the start tick of timer */
            tickDelta = mClock::getInstance()->tickGet();

            /*RT_DEBUG_LOG(RT_DEBUG_IPC, ("set thread:%s to timer list\n",
                                        thread->name));*/

            /* reset the timeout of thread timer and start it */
            reinterpret_cast<mthread*>(thread)->getThTimer()->setTimerAndStart(timeout);
        }

        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        /* re-schedule */
        mSchedule::getInstance()->schedule();

        /* recv message */
        if (thread->error != M_RESULT_EOK)
        {
            /* return error */
            return thread->error;
        }

        /* disable interrupt */
        temp = HW::hwInterruptDisable();

        /* if it's not waiting forever and then re-calculate timeout tick */
        if (timeout > 0)
        {
            tickDelta = mClock::getInstance()->tickGet() - tickDelta;
            timeout -= tickDelta;
            if (timeout < 0)
            {
                timeout = 0;
            }
        }
    }

    /* get message from queue */
    msg = (struct mqMessage_t *)msg_.msgQueueHead;

    /* move message queue head */
    msg_.msgQueueHead = msg->next;
    /* reach queue tail, set to NULL */
    if (msg_.msgQueueTail == msg)
        msg_.msgQueueTail = nullptr;

    /* decrease message entry */
    if(msg_.entry > 0)
    {
        msg_.entry --;
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    /* copy message */
    memcpy(buffer, msg + 1, size > msg_.msgSize ? msg_.msgSize : size);

    /* disable interrupt */
    temp = HW::hwInterruptDisable();
    /* put message to free list */
    msg->next = (struct mqMessage_t *)msg_.msgQueueFree;
    msg_.msgQueueFree = msg;

    /* resume suspended thread */
    if (!msg_.suspendSenderThread.isEmpty())
    {
        ipcListResume(&(msg_.suspendSenderThread));

        /* enable interrupt */
        HW::hwInterruptEnable(temp);

        //RT_OBJECT_HOOK_CALL(rt_object_take_hook, (&(msg_.parent.parent)));

        mSchedule::getInstance()->schedule();

        return M_RESULT_EOK;
    }

    /* enable interrupt */
    HW::hwInterruptEnable(temp);

    //RT_OBJECT_HOOK_CALL(rt_object_take_hook, (&(msg_.parent.parent)));

    return M_RESULT_EOK;
}

/**
 * This function can get or set some extra attributions of a message queue
 * object.
 *
 * @param mq the message queue object
 * @param cmd the execution command
 * @param arg the execution argument
 *
 * @return the error code
 */
mResult mMessagequeue::control(int cmd, void *arg)
{
    long level;
    struct mqMessage_t *msg;

    /* parameter check */
    MASSERT(mObject::getInstance()->objectGetType((mObject_t*)this) == M_OBJECT_CLASS_MESSAGEQUEUE);

    if (cmd == IPC_CMD_RESET)
    {
        /* disable interrupt */
        level = HW::hwInterruptDisable();

        /* resume all waiting thread */
        ipcListResumeAll((mIpcObject_t*)this);
        /* also resume all message queue private suspended thread */
        ipcListResumeAll(&(msg_.suspendSenderThread));

        /* release all message in the queue */
        while (msg_.msgQueueHead != nullptr)
        {
            /* get message from queue */
            msg = (struct mqMessage_t *)msg_.msgQueueHead;

            /* move message queue head */
            msg_.msgQueueHead = msg->next;
            /* reach queue tail, set to NULL */
            if (msg_.msgQueueTail == msg)
                msg_.msgQueueTail = nullptr;

            /* put message to free list */
            msg->next = (struct mqMessage_t *)msg_.msgQueueFree;
            msg_.msgQueueFree = msg;
        }

        /* clean entry */
        msg_.entry = 0;

        /* enable interrupt */
        HW::hwInterruptEnable(level);

        mSchedule::getInstance()->schedule();

        return M_RESULT_EOK;
    }

    return M_RESULT_EOK;
}