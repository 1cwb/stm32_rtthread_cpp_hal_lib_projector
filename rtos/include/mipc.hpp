#pragma once
#include "rtoscommon.hpp"

class mIpc
{
public:
    mIpc()
    {
    }
    ~mIpc()
    {

    }
    mIpc(const mIpc&) = delete;
    mIpc(mIpc&&) = delete;
    mIpc& operator=(const mIpc&) = delete;
    mIpc& operator=(mIpc&&) = delete;
    /**
     * This function will initialize an IPC object
     *
     * @param ipc the IPC object
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult ipcObjectInit(mIpcObject_t* ipcObj);
    /**
     * This function will suspend a thread to a specified list. IPC object or some
     * double-queue object (mailbox etc.) contains this kind of list.
     *
     * @param list the IPC suspended thread list
     * @param thread the thread object to be suspended
     * @param flag the IPC object flag,
     *        which shall be RT_IPC_FLAG_FIFO/RT_IPC_FLAG_PRIO.
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult ipcListSuspend(mIpcObject_t* ipcObj, struct thread_t *thread, mIpcFlag flag);
    mResult ipcListSuspend(mList_t* list, struct thread_t *thread, mIpcFlag flag);
    /**
     * This function will resume the first thread in the list of a IPC object:
     * - remove the thread from suspend queue of IPC object
     * - put the thread into system ready queue
     *
     * @param list the thread list
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult ipcListResume(mIpcObject_t* ipcObj);
    mResult  ipcListResume(mList_t* list);
    /**
     * This function will resume all suspended threads in a list, including
     * suspend list of IPC object and private list of mailbox etc.
     *
     * @param list of the threads to resume
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult ipcListResumeAll(mIpcObject_t* ipcObj);
    mResult ipcListResumeAll(mList_t *list);
};

class mSemaphore : public mIpc
{
public:
    mSemaphore()
    {
    }
    ~mSemaphore()
    {

    }
    mSemaphore(const mSemaphore&) = delete;
    mSemaphore(mSemaphore&&) = delete;
    mSemaphore& operator=(const mSemaphore&) = delete;
    mSemaphore& operator=(mSemaphore&&) = delete;

    static mSemaphore* semCreate(const char *name, uint32_t value, mIpcFlag flag)
    {
        mSemaphore* sem = new mSemaphore;
        if(sem)
        {
            sem->semCreateEx(name, value, flag);
            return sem;
        }
        return nullptr;
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
    mResult semDelete();
        /**
     * This function will initialize a semaphore and put it under control of
     * resource management.
     *
     * @param sem the semaphore object
     * @param name the name of semaphore
     * @param value the initial value of semaphore
     * @param flag the flag of semaphore
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult init(const char *name, uint32_t value, mIpcFlag flag);
    /**
     * This function will detach a semaphore from resource management
     *
     * @param sem the semaphore object
     *
     * @return the operation status, RT_EOK on successful
     *
     * @see rt_sem_delete
     */
    mResult detach();
    /**
     * This function will take a semaphore, if the semaphore is unavailable, the
     * thread shall wait for a specified time.
     *
     * @param sem the semaphore object
     * @param time the waiting time
     *
     * @return the error code
     */
    mResult semTake(int32_t time);

    /**
     * This function will try to take a semaphore and immediately return
     *
     * @param sem the semaphore object
     *
     * @return the error code
     */
    mResult semTrytake();

    /**
     * This function will release a semaphore, if there are threads suspended on
     * semaphore, it will be waked up.
     *
     * @param sem the semaphore object
     *
     * @return the error code
     */
    mResult semRelease();

    /**
     * This function can get or set some extra attributions of a semaphore object.
     *
     * @param sem the semaphore object
     * @param cmd the execution command
     * @param arg the execution argument
     *
     * @return the error code
     */
    mResult semControl(mIpcCmd cmd, void *arg);
    mSemaphore_t* getSem() {return &sem_;}
private:

        /**
     * This function will create a semaphore from system resource
     *
     * @param name the name of semaphore
     * @param value the initial value of semaphore
     * @param flag the flag of semaphore
     *
     * @return the created semaphore, RT_NULL on error happen
     *
     * @see rt_sem_init
     */
    mResult semCreateEx(const char *name, uint32_t value, mIpcFlag flag);

private:
    mSemaphore_t sem_;
};

class mMutex : public mIpc
{
public:
    mMutex()
    {
    }
    mMutex(const char *name, mIpcFlag flag)
    {
        init(name, flag);
    }
    ~mMutex()
    {

    }
    mMutex(const mMutex&) = delete;
    mMutex(mMutex&&) = delete;
    mMutex& operator=(const mMutex&) = delete;
    mMutex& operator=(mMutex&&) = delete;

    /**
     * This function will initialize a mutex and put it under control of resource
     * management.
     *
     * @param mutex the mutex object
     * @param name the name of mutex
     * @param flag the flag of mutex
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult init(const char *name, mIpcFlag flag);

    /**
     * This function will detach a mutex from resource management
     *
     * @param mutex the mutex object
     *
     * @return the operation status, RT_EOK on successful
     *
     * @see rt_mutex_delete
     */
    mResult detach();
    /**
     * This function will take a mutex, if the mutex is unavailable, the
     * thread shall wait for a specified time.
     *
     * @param mutex the mutex object
     * @param time the waiting time
     *
     * @return the error code
     */
    mResult mutexTake(int32_t time);
    /**
     * This function will release a mutex, if there are threads suspended on mutex,
     * it will be waked up.
     *
     * @param mutex the mutex object
     *
     * @return the error code
     */
    mResult mutexRelease();
private:
    mMutex_t mutex_;
};

class mEvent : public mIpc
{
public:
    mEvent()
    {
    }
    ~mEvent()
    {

    }
    mEvent(const mEvent&) = delete;
    mEvent(mEvent&&) = delete;
    mEvent& operator=(const mEvent&) = delete;
    mEvent& operator=(mEvent&&) = delete;

    /**
     * This function will initialize an event and put it under control of resource
     * management.
     *
     * @param event the event object
     * @param name the name of event
     * @param flag the flag of event
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult init(const char *name, mIpcFlag flag);

    /**
     * This function will detach an event object from resource management
     *
     * @param event the event object
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult detach();
    /**
     * This function will send an event to the event object, if there are threads
     * suspended on event object, it will be waked up.
     *
     * @param event the event object
     * @param set the event set
     *
     * @return the error code
     */
    mResult send(uint32_t set);

    /**
     * This function will receive an event from event object, if the event is
     * unavailable, the thread shall wait for a specified time.
     *
     * @param event the fast event object
     * @param set the interested event set
     * @param option the receive option, either RT_EVENT_FLAG_AND or
     *        RT_EVENT_FLAG_OR should be set.
     * @param timeout the waiting time
     * @param recved the received event, if you don't care, RT_NULL can be set.
     *
     * @return the error code
     */
    mResult recv(uint32_t  set,
                    uint8_t   option,
                    int32_t   timeout,
                    uint32_t *recved);

    /**
     * This function can get or set some extra attributions of an event object.
     *
     * @param event the event object
     * @param cmd the execution command
     * @param arg the execution argument
     *
     * @return the error code
     */
    mResult control(mIpcCmd cmd, void *arg);
private:
    mEvent_t event_;
};


class mMessagequeue : public mIpc
{
    struct mqMessage_t
    {
        struct mqMessage_t *next;
    };
public:
    mMessagequeue() : bInited_(false)
    {
    }
    ~mMessagequeue()
    {

    }
    mMessagequeue(const mMessagequeue&) = delete;
    mMessagequeue(mMessagequeue&&) = delete;
    mMessagequeue& operator=(const mMessagequeue&) = delete;
    mMessagequeue& operator=(mMessagequeue&&) = delete;
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
     * @return the operation status, RT_EOK on successful
     */
    mResult init(const char *name,
                    uint32_t   msgSize,
                    uint32_t   poolSize,
                    mIpcFlag   flag);
    /**
     * This function will detach a message queue object from resource management
     *
     * @param mq the message queue object
     *
     * @return the operation status, RT_EOK on successful
     */
    mResult detach();
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
    mResult sendWait(const void *buffer, uint32_t size, int32_t timeout);
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
    mResult send(const void *buffer, uint32_t size);
    mResult urgent(const void *buffer, uint32_t size);
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
    mResult recv( void *buffer, uint32_t  size, int32_t timeout);
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
    mResult control(int cmd, void *arg);
private:
    mMessagequeue_t msg_;
    bool bInited_;
};