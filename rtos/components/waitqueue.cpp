#include "waitqueue.hpp"


void mWqueue::add(wqueueNode_t* node)
{
    long level;
    level = HW::hwInterruptDisable();
    node->list.insertBeforeTo(&queue_.waitingList);
    HW::hwInterruptEnable(level);
}
void mWqueue::remove(wqueueNode_t* node)
{
    long level;
    level = HW::hwInterruptDisable();
    node->list.removeSelf();
    HW::hwInterruptEnable(level);
}
mResult mWqueue::wait(int condition, int timeout)
{
    int tick;
    thread_t* tid = nullptr;
    mTimer_t* tmr = nullptr;
    wqueueNode __wait(mWqueue::defaultWake);
    long level;

    tick = mClock::getInstance()->tickFromMillisecond(timeout);

    if(condition || tick == 0)
    {
        return M_RESULT_EOK;
    }
    level = HW::hwInterruptDisable();
    tid = mthread::threadSelf();
    tmr = reinterpret_cast<mthread*>(tid)->getThTimer_t();
    do 
    {
        if(queue_.flag == WQFLAGS::WQ_FLAG_WAKEUP)
        {
            //ALREADY WAKE UP;
            break;
        }
        add(&__wait);
        printf("+\r\n");
        mthread::threadSuspend(tid);
        if(tick != WAITING_FOREVER)
        {
            reinterpret_cast<mTimer*>(tmr)->timerControl(TIMER_CTRL_SET_TIME, &tick);
            reinterpret_cast<mTimer*>(tmr)->start();
        }
        HW::hwInterruptEnable(level);
        mSchedule::getInstance()->schedule();
        if (tid->error != M_RESULT_EOK)
        {
            /*if timeout return error */
            remove(&__wait);
            return tid->error;
        }
        level = HW::hwInterruptDisable();
    }while(0);
    queue_.flag = WQFLAGS::WQ_FLAG_CLEAN;
    HW::hwInterruptEnable(level);
    remove(&__wait);
    return M_RESULT_EOK;
}
void mWqueue::wakeup(void* key)
{
    long level;
    register bool needSchedule = false;

    mList_t* queueList = nullptr;
    mList_t* node = nullptr;
    wqueueNode_t* entry = nullptr;

    queueList = &(queue_.waitingList);

    level = HW::hwInterruptDisable();

    queue_.flag = WQFLAGS::WQ_FLAG_WAKEUP;

    if(!(queueList->isEmpty()))
    {
        for(node = queueList->next; node != queueList; node = node->next)
        {
            entry = listEntry(node, wqueueNode_t, list);
            if(entry->wakeup(entry, key) == 0)
            {
                reinterpret_cast<mthread*>(entry->pollingThread)->threadResume();
                needSchedule = true;
                remove(entry);
                break;
            }
        }
    }
    HW::hwInterruptEnable(level);
    if(needSchedule)
    {
        mSchedule::getInstance()->schedule();
    }
}
int mWqueue::defaultWake(struct wqueueNode *wait, void *key)
{
    return M_RESULT_EOK;
}