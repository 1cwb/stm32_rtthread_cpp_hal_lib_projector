#pragma once
#include "rtoscommon.hpp"
#include "mirq.hpp"
#include "mkservice.hpp"
#include "mhw.hpp"
#include <functional>

class mSchedule
{
    using mScheduleHookCallbackFunc = std::function<void(thread_t*, thread_t*)>;
public:
    static mSchedule* getInstance()
    {
        static mSchedule sch;
        return &sch;
    }

    //#ifdef RT_USING_OVERFLOW_CHECK
    void schedulerStackCheck(thread_t *thread);
    //#endif

    /**
     * @ingroup SystemInit
     * This function will initialize the system scheduler
     */
    void systemSchedulerInit(void);
    /**
     * @ingroup SystemInit
     * This function will startup scheduler. It will select one thread
     * with the highest priority level, then switch to it.
     */
    void systemSchedulerStart(void);
    /**
     * This function will perform one schedule. It will select one thread
     * with the highest priority level, then switch to it.
     */
    void schedule(void);
    /*
    * This function will insert a thread to system ready queue. The state of
    * thread will be set as READY and remove from suspend queue.
    *
    * @param thread the thread to be inserted
    * @note Please do not invoke this function in user application.
    */
    void scheduleInsertThread(thread_t *thread);
    /*
    * This function will remove a thread from system ready queue.
    *
    * @param thread the thread to be removed
    *
    * @note Please do not invoke this function in user application.
    */
    void scheduleRemoveThread(thread_t *thread);

    /**
     * This function will lock the thread scheduler.
     */
    void enterCritical(void);

    /**
     * This function will unlock the thread scheduler.
     */
    void exitCritical(void);

    /**
     * Get the scheduler lock level
     *
     * @return the level of the scheduler lock. 0 means unlocked.
     */
    uint16_t criticalLevel(void);
    thread_t*& getCurrentThread();
    mList_t* getThreadDefunct();
    mList_t* getThreadPriorityTable(uint8_t priority);
    bool bScheduleStarted() const {return bScheduleStart_;}
    static void registerScheduleHookCallback(const mScheduleHookCallbackFunc scheduleHookCb){scheduleHookCb_ = scheduleHookCb;}
private:
    mSchedule():schedulerLockNest_(0),currentThread_(nullptr),currentPriority_(THREAD_PRIORITY_MAX-1),bScheduleStart_(false)
    {
    }
    ~mSchedule()
    {

    }
    mSchedule(const mSchedule&) = delete;
    mSchedule(mSchedule&&) = delete;
    mSchedule& operator=(const mSchedule&) = delete;
    mSchedule& operator=(mSchedule&&) = delete;

    int16_t schedulerLockNest_;
    thread_t *currentThread_;
    uint8_t currentPriority_;
    mList_t threadDefunct_;

    mList_t threadPriorityTable_[THREAD_PRIORITY_MAX];
    uint32_t threadReadyPriorityGroup_;

#if THREAD_PRIORITY_MAX > 32
    /* Maximum priority level, 256 */
    uint8_t threadReadyTable_[32];
#endif
    bool bScheduleStart_;
    static mScheduleHookCallbackFunc scheduleHookCb_;
};