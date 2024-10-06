#pragma once
#include "rtoscommon.hpp"
#include "mthread.hpp"
#include <functional>

#define SCHEDULE_DELAY(delayMs) (mClock::getInstance()->tickGet() + (delayMs))
using workItemCallback = std::function<void(void* param)>;
class workItem
{
public:
    workItem(const char* name, uint32_t scheduleTime, uint16_t period, workItemCallback run, void* param):
    _name(name),
    _scheduleTime(scheduleTime),
    _run(run),
    _param(param),
    _period(period) {}
    ~workItem(){}
    workItem(const workItem&) = delete;
    workItem(workItem&&) = delete;
    workItem& operator=(const workItem&) = delete;
    workItem& operator=(workItem&&) = delete;
    const char* getName() {return _name;}
    uint32_t getScheduleTime() const {return _scheduleTime;}
    void setScheduleTime(uint32_t scheduleTime) {_scheduleTime = scheduleTime;}
    void run() {if(_run){_run(_param);}}
    void* getParam(){return _param;}
    uint16_t getPeriod() const {return _period;}
    void setPeriod(uint16_t period) {_period = period;}
private:
    const char* _name;
    uint32_t _scheduleTime;
    workItemCallback _run;
    void* _param;
    uint16_t _period;
};

class workQueue
{
public:
    workQueue();
    ~workQueue();
    workQueue(const workQueue&) = delete;
    workQueue(workQueue&&) = delete;
    workQueue& operator=(const workQueue&) = delete;
    workQueue& operator=(workQueue&&) = delete;
    mResult scheduleWork(workItem* item);
    mResult cancelWork(workItem* item);
    uint8_t getsize() const {return size_;}
    uint8_t getQsize() const {return qsize_;}
    workItem** getWorkItem() {return queue_;}
    mthread* getThread() {return thread_;}
    mResult init(const char* name, uint8_t size, uint16_t stackSize, uint8_t priority);
    mResult deinit();
private:
    inline void swapItem(workItem** a, workItem** b);
    inline void heapify(workItem** queue, uint8_t size, int idx);
    inline int findWorkItemIdx(workItem* item);
    workItem* pop();
    static void executor(void* param);
private:
    mthread* thread_;
    uint8_t qsize_;
    uint8_t size_;
    workItem** queue_;
    mSemaphore lock;
};