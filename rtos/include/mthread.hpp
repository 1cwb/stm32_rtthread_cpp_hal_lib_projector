#pragma once
#include "rtoscommon.hpp"
#include "mobject.hpp"
#include "cpuport.hpp"
#include "mclock.hpp"
#include <functional>
#include <mtimer.hpp>
#include "mmem.hpp"

class mthread
{
    using mThreadCallbackFunc = std::function<void(void*)>;
public:
    mthread()
    {
        thData_.extiThread = threadExti;
        thData_.yieldThread = threadYield;
    }
    ~mthread()
    {

    }
    mthread(const mthread&) = delete;
    mthread(mthread&&) = delete;
    mthread& operator=(const mthread&) = delete;
    mthread& operator=(mthread&&) = delete;
    static void threadExti();
    static mthread* create(const char       *name,
                                    uint32_t         stackSize,
                                    uint8_t          priority,
                                    uint32_t         tick,
                                    const mThreadCallbackFunc& func,
                                    void* callbackParam);
    /**
     * This function will delete a thread. The thread object will be removed from
     * thread queue and deleted from system object management in the idle thread.
     *
     * @param thread the thread to be deleted
     *
     * @return the operation status, RT_EOK on OK, -RT_ERROR on error
     */
    mResult threadDelete();
    mResult init(const char       *name,
                 void             *stackStart,
                 uint32_t         stackSize,
                 uint8_t          priority,
                 uint32_t         tick,
                 const mThreadCallbackFunc& func,
                 void* callbackParam);
    mResult init(const char       *name,
                 void (*entry)(void *parameter),
                 void             *parameter,
                 void             *stackStart,
                 uint32_t         stackSize,
                 uint8_t          priority,
                 uint32_t         tick);
    /**
     * This function will return self thread object
     *
     * @return the self thread object
     */
    static thread_t* threadSelf(void);

    /**
     * This function will start a thread and put it to system ready queue
     *
     * @param thread the thread to be started
     *
     * @return the operation status, RT_EOK on OK, -RT_ERROR on error
     */
    mResult startup();
    /**
     * This function will let current thread yield processor, and scheduler will
     * choose a highest thread to run. After yield processor, the current thread
     * is still in READY state.
     *
     * @return RT_EOK
     */
    static mResult threadYield(void);

    /**
     * This function will let current thread sleep for some ticks.
     *
     * @param tick the sleep ticks
     *
     * @return RT_EOK
     */
    static mResult threadSleep(uint32_t tick);

    /**
     * This function will let current thread delay for some ticks.
     *
     * @param tick the delay ticks
     *
     * @return RT_EOK
     */
    static mResult threadDelay(uint32_t tick);

    /**
     * This function will let current thread delay until (*tick + inc_tick).
     *
     * @param tick the tick of last wakeup.
     * @param inc_tick the increment tick
     *
     * @return RT_EOK
     */
    static mResult threadDelayUntil(uint32_t *tick, uint32_t incTick);
    /**
     * This function will let current thread delay for some milliseconds.
     *
     * @param ms the delay ms time
     *
     * @return RT_EOK
     */
    static mResult threadMdelay(int32_t ms);

    /**
     * This function will control thread behaviors according to control command.
     *
     * @param thread the specified thread to be controlled
     * @param cmd the control command, which includes
     *  RT_THREAD_CTRL_CHANGE_PRIORITY for changing priority level of thread;
     *  RT_THREAD_CTRL_STARTUP for starting a thread;
     *  RT_THREAD_CTRL_CLOSE for delete a thread;
     *  RT_THREAD_CTRL_BIND_CPU for bind the thread to a CPU.
     * @param arg the argument of control command
     *
     * @return RT_EOK
     */
    mResult threadControl(mthreadCtrlCmd cmd, void *arg);

    /**
     * This function will suspend the specified thread.
     *
     * @param thread the thread to be suspended
     *
     * @return the operation status, RT_EOK on OK, -RT_ERROR on error
     *
     * @note if suspend self thread, after this function call, the
     * rt_schedule() must be invoked.
     */
    static mResult threadSuspend(thread_t* thread);
    /**
     * This function will resume a thread and put it to system ready queue.
     *
     * @param thread the thread to be resumed
     *
     * @return the operation status, RT_EOK on OK, -RT_ERROR on error
     */
    mResult threadResume();

    /**
     * This function is the timeout function for thread, normally which is invoked
     * when thread is timeout to wait some resource.
     *
     * @param parameter the parameter of thread timeout function
     */
    static void threadTimeout(void *parameter);

    /**
     * This function will find the specified thread.
     *
     * @param name the name of thread finding
     *
     * @return the found thread
     *
     * @note please don't invoke this function in interrupt status.
     */
    static thread_t* threadFind(char *name);
    /**
     * This function will detach a thread. The thread object will be removed from
     * thread queue and detached/deleted from system object management.
     *
     * @param thread the thread to be deleted
     *
     * @return the operation status, RT_EOK on OK, -RT_ERROR on error
     */
    mResult threadDetach();
    mTimer* getThTimer();
    mTimer_t* getThTimer_t();
    char* name() const;
    int32_t getTotalStackSize() const {return thData_.stackSize;}
    int32_t getFreeStackSize() {return (reinterpret_cast<uint8_t*>(thData_.sp) - reinterpret_cast<uint8_t*>(thData_.stackAddr));}

    static void showAllThreadStackSizeInfo();
private:
    mResult threadInit( const char       *name,
                        void (*entry)(void *parameter),
                        void             *parameter,
                        void             *stackStart,
                        uint32_t         stackSize,
                        uint8_t          priority,
                        uint32_t         tick);
    mResult threadCreate(const char       *name,
                 uint32_t         stackSize,
                 uint8_t          priority,
                 uint32_t         tick,
                 const mThreadCallbackFunc& func,
                 void* callbackParam);
    /**
     * This function will create a thread object and allocate thread object memory
     * and stack.
     *
     * @param name the name of thread, which shall be unique
     * @param entry the entry function of thread
     * @param parameter the parameter of thread enter function
     * @param stack_size the size of thread stack
     * @param priority the priority of thread
     * @param tick the time slice if there are same priority thread
     *
     * @return the created thread object
     */
    mResult threadCreate(const char *name,
                                void (*entry)(void *parameter),
                                void       *parameter,
                                uint32_t stackSize,
                                uint8_t  priority,
                                uint32_t tick);
    /* must be invoke witch rt_hw_interrupt_disable */
    static void threadCleanupExecute(thread_t* thread);
    static void threadFunc(void* p);
private:
    thread_t thData_;
    mTimer thTimer_;
    mThreadCallbackFunc cb_;
    void* callbackParam_;
};