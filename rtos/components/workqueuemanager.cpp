#include "workqueuemanager.hpp"
#include "mklog.hpp"

workQueueManager::workQueueManager()
{
    MASSERT(init() == M_RESULT_EOK);
}
workQueueManager::~workQueueManager()
{
    for(auto& it: wqList)
    {
        if(it)
        {
            it->deinit();
            it = nullptr;
        }
    }
}
mResult workQueueManager::init()
{
    do
    {
        wqList[0] = new workQueue();
        if(!wqList[0])
        {
            break;
        }
        if(wqList[0]->init(WORKQUEUE_LP_WORK, 20, 4096, 6) != M_RESULT_EOK)
        {
            break;
        }
        wqList[1] = new workQueue();
        if(!wqList[1])
        {
            break;
        }
        if(wqList[1]->init(WORKQUEUE_HP_WORK, 20, 4096, 4) != M_RESULT_EOK)
        {
            break;
        }
        return M_RESULT_EOK;
    } while (false);
    if(wqList[1])
    {
        wqList[1]->deinit();
        delete wqList[1];
    }
    if(wqList[0])
    {
        wqList[0]->deinit();
        delete wqList[0];
    }
    return M_RESULT_ERROR;
}
workQueue* workQueueManager::find(const char* name)
{
    for(auto& it : wqList)
    {
        if(strcmp(it->getThread()->name(), name) == 0)
        {
            return it;
        }
    }
    return nullptr;
}