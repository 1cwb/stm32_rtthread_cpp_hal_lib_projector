#pragma once
#include "workqueue.hpp"

#define WORKQUEUE_LP_WORK "lpWkQue"
#define WORKQUEUE_HP_WORK "hpWkQue"
class workQueueManager
{
public:
    static workQueueManager* getInstance()
    {
        static workQueueManager qma;
        return &qma;
    }
    workQueue* find(const char* name);
private:
    workQueueManager();
    ~workQueueManager();
    workQueueManager(const workQueueManager&) = delete;
    workQueueManager(workQueueManager&&) = delete;
    workQueueManager& operator=(const workQueueManager&) = delete;
    workQueueManager& operator=(workQueueManager&&) = delete;
    mResult init();
    static const int intQueueMaxLen_ = 2;
    workQueue* wqList[intQueueMaxLen_] = { nullptr };
};