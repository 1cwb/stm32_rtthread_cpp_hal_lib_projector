#include "workqueue.hpp"
workQueue::workQueue()
{

}
workQueue::~workQueue()
{

}
mResult workQueue::init(const char* name, uint8_t size, uint16_t stackSize, uint8_t priority)
{
    MASSERT(size > 0);
    MASSERT(stackSize > 0);
    thread_ = mthread::create(name,stackSize,priority,10,workQueue::executor,this);
    if(!thread_)
    {
        return M_RESULT_ERROR;
    }
    queue_ = new workItem*[size];
    if(!queue_)
    {
        return M_RESULT_ENOMEM;
    }
    qsize_ = size;
    size_ = 0;
    if(lock.init(name,1,IPC_FLAG_FIFO) != M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    if(thread_->startup() != M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult workQueue::deinit()
{
    if(thread_->threadDelete() != M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    if(lock.detach() != M_RESULT_EOK)
    {
        return M_RESULT_ERROR;
    }
    delete []queue_;
    return  M_RESULT_EOK;
}
mResult workQueue::scheduleWork(workItem* item)
{
    if(!item)
    {
        return M_RESULT_ERROR;
    }
    cancelWork(item);

    lock.semTake(WAITING_FOREVER);
    if(size_ >= qsize_ - 1)
    {
        lock.semRelease();
        return M_RESULT_EFULL;
    }
    if(size_ == 0)
    {
        queue_[0] = item;
        size_ += 1;
    }
    else
    {
        queue_[size_] = item;
        size_ += 1;
        for(int i = size_ / 2 - 1; i >= 0; i--)
        {
            heapify(queue_, size_, i);
        }
    }
    lock.semRelease();
    thread_->threadResume();
    mSchedule::getInstance()->schedule();
    return M_RESULT_EOK;
}

mResult workQueue::cancelWork(workItem* item)
{
    if(!item)
    {
        return M_RESULT_EINVAL;
    }
    int idx;
    lock.semTake(WAITING_FOREVER);
    idx = findWorkItemIdx(item);
    if(idx < 0)
    {
        lock.semRelease();
        return M_RESULT_EEMPTY;
    }
    swapItem(&queue_[idx], &queue_[size_ - 1]);
    size_ -= 1;
    for(int i = size_ / 2 - 1; i >= 0; i--)
    {
        heapify(queue_, size_, i);
    }
    lock.semRelease();
    return M_RESULT_EOK;
}

void workQueue::swapItem(workItem** a, workItem** b)
{
    workItem* temp = *b;
    *b = *a;
    *a = temp;
}
void workQueue::heapify(workItem** queue, uint8_t size, int idx)
{
    if(size == 1)
    {
        return;
    }

    int smallest = idx;
    int l = 2 * idx + 1;
    int r = 2 * idx + 2;

    if(l < size && queue[l]->getScheduleTime() < queue[smallest]->getScheduleTime())
    {
        smallest = l;
    }
    if(r < size && queue[r]->getScheduleTime() < queue[smallest]->getScheduleTime())
    {
        smallest = r;
    }
    if(smallest != idx)
    {
        swapItem(&queue[idx], &queue[smallest]);
        heapify(queue, size, smallest);
    }
}
int workQueue::findWorkItemIdx(workItem* item)
{
    int index = -1;
    for(int i = 0; i < size_; i++)
    {
        if(item == queue_[i])
        {
            index = i;
            break;
        }
    }
    return index;
}
workItem* workQueue::pop()
{
    lock.semTake(WAITING_FOREVER);
    if(size_ == 0)
    {
        lock.semRelease();
        return nullptr;
    }
    workItem* dqItem = queue_[0];
    lock.semRelease();
    if(cancelWork(dqItem) != M_RESULT_EOK)
    {
        return nullptr;
    }
    return dqItem;
}
void workQueue::executor(void* param)
{
    if(param == nullptr)
    {
        return;
    }
    workQueue* workqueue = reinterpret_cast<workQueue*>(param);
    workItem* item = nullptr;
    uint32_t timenow, scheduleTime;

    while (true)
    {
        if(workqueue->getsize() == 0)
        {
            mthread::threadSuspend(mthread::threadSelf());
            mSchedule::getInstance()->schedule();
        }
        timenow = mClock::getInstance()->tickGet();
        scheduleTime = workqueue->getWorkItem()[0]->getScheduleTime();
        if(scheduleTime > timenow)
        {
            mthread::threadDelay(scheduleTime - timenow);
            continue;
        }
        item = workqueue->pop();
        if(item)
        {
            item->run();
            if(item->getPeriod() != 0 && workqueue->findWorkItemIdx(item) < 0)
            {
                item->setScheduleTime(SCHEDULE_DELAY(item->getPeriod()));
                workqueue->scheduleWork(item);
            }
        }
    }
}