#include "mIdle.hpp"
#include "rtoscommon.hpp"
ALIGN(M_ALIGN_SIZE) uint8_t mIdle::threadStack[IDLE_THREAD_STACK_SIZE];

void mIdle::threadIdleInit(void)
{
    /* initialize thread */
    idleThread_.init(
                "tidle",
                &threadStack[0],
                sizeof(threadStack),
                THREAD_PRIORITY_MAX - 1,
                32,
                [](void* p)
                {
                    while(true)
                    {
                        exec();
                    }
                },nullptr);

    /* startup */
    idleThread_.startup();
}
void mIdle::exec()
{
    /* Loop until there is no dead thread. So one call to rt_thread_idle_excute
    * will do all the cleanups. */
    /* disable interrupt */
    while (1)
    {
        long lock;
        thread_t* thread;

        lock = HW::hwInterruptDisable();

        /* check whether list is empty */
        if (mSchedule::getInstance()->getThreadDefunct()->isEmpty())
        {
            HW::hwInterruptEnable(lock);
            break;
        }
        /* get defunct thread */
        thread = listEntry(mSchedule::getInstance()->getThreadDefunct()->next, struct thread_t, tlist);
        /* remove defunct thread */
        thread->tlist.removeSelf();
        mObject::getInstance()->objectRemove((mObject_t*)thread);
        /* release thread's stack */
        delete [] reinterpret_cast<uint8_t*>(thread->stackAddr);
        /* delete thread object */
        delete reinterpret_cast<mthread*>(thread);
        HW::hwInterruptEnable(lock);
    }
}