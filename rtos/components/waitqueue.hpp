#pragma once
#include "rtoscommon.hpp"
#include "mthread.hpp"
#include <functional>

enum class WQFLAGS
{
    WQ_FLAG_CLEAN  =  0x00,
    WQ_FLAG_WAKEUP =  0x01
};

/**
 * WaitQueue structure
 */
struct wqueue
{
    WQFLAGS flag = WQFLAGS::WQ_FLAG_CLEAN;
    mList_t waitingList;
};
using wqueue_t = wqueue ;

struct wqueueNode;
using wqueueFunc = std::function<int(struct wqueueNode *wait, void *key)>;//int (*)(struct wqueueNode *wait, void *key);

struct wqueueNode
{
    thread_t* pollingThread;
    mList_t list;
    wqueueFunc wakeup;
    uint32_t key;
    wqueueNode(const wqueueFunc& func)
    :pollingThread(mthread::threadSelf()),
     wakeup(func),
     key(0)
    {}
};

using wqueueNode_t = wqueueNode;

class mWqueue
{
public:
    mWqueue() = default;
    mWqueue(const mWqueue& mq) = default;
    mWqueue(mWqueue&& mq) = default;
    ~mWqueue() = default;
    mWqueue& operator=(const mWqueue& mq) = default;
    mWqueue& operator=(mWqueue&& mq) = default;

    void add(wqueueNode_t* node);
    void remove(wqueueNode_t* node);
    mResult wait(int condition, int timeout);
    void wakeup(void* key);
    static int defaultWake(struct wqueueNode *wait, void *key);
private:
   wqueue_t queue_;   
};