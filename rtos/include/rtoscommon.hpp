#pragma once
#include <stdio.h>
#include <stdint.h>
#include <cstring>
#include <limits>
#include "mbase.hpp"

#define NAME_MAX 8
#define THREAD_PRIORITY_MAX 8

#define TIMER_SKIP_LIST_LEVEL          1
#define TIMER_SKIP_LIST_MASK         0x3

#define WAITING_FOREVER              -1              /**< Block forever until get resource. */
#define WAITING_NO                   0               /**< Non-block. */

//  <i>Default: 1000   (1ms)
#define TICK_PER_SECOND  THREAD_TICK_PER_SECOND

/* RT-Thread error code definitions */
enum mResult
{
    M_RESULT_EOK                 =         0,               /**< There is no error */
    M_RESULT_ERROR               =         1,               /**< A generic error happens */
    M_RESULT_ETIMEOUT            =         2,               /**< Timed out */
    M_RESULT_EFULL               =         3,               /**< The resource is full */
    M_RESULT_EEMPTY              =         4,               /**< The resource is empty */
    M_RESULT_ENOMEM              =         5,               /**< No memory */
    M_RESULT_ENOSYS              =         6,               /**< No system */
    M_RESULT_EBUSY               =         7,               /**< Busy */
    M_RESULT_EIO                 =         8,               /**< IO error */
    M_RESULT_EINTR               =         9,               /**< Interrupted system call */
    M_RESULT_EINVAL              =         10,              /**< Invalid argument */
    M_RESULT_EXIST               =         11,
    M_RESULT_NODEV               =         12,
    M_RESULT_NOFILE              =         13,
    M_RESULT_NOTTY               =         14,
    M_RESULT_BADF                =         15,
};
/**@}*/

/*
 * thread state definitions
 */
enum mthreadStatus
{
    THREAD_INIT          =        0x00,                /**< Initialized status */
    THREAD_READY         =        0x01,                /**< Ready status */
    THREAD_SUSPEND       =        0x02,                /**< Suspend status */
    THREAD_RUNNING       =        0x03,                /**< Running status */
    THREAD_BLOCK         =        THREAD_SUSPEND,      /**< Blocked status */
    THREAD_CLOSE         =        0x04,                /**< Closed status */
    THREAD_STAT_MASK     =        0x0f,
};

/**
 * thread control command definitions
 */
enum mthreadCtrlCmd
{
    THREAD_CTRL_STARTUP         = 0x00,                /**< Startup thread. */
    THREAD_CTRL_CLOSE           = 0x01,                /**< Close thread. */
    THREAD_CTRL_CHANGE_PRIORITY = 0x02,                /**< Change thread priority. */
    THREAD_CTRL_INFO            = 0x03,                /**< Get thread information. */
};

/**
 * Double List structure
 */
struct mList_t
{
    struct mList_t *next;                          /**< point to next node. */
    struct mList_t *prev;                          /**< point to prev node. */
    mList_t()
    {
        next = prev = this;
    }
    ~mList_t()
    {
        next = prev = nullptr;
    }
    inline void insertAfterTo(mList_t* l)
    {
        l->next->prev = this;
        this->next = l->next;

        l->next = this;
        this->prev = l;
    }
    inline void insertBeforeTo(mList_t* l)
    {
        l->prev->next = this;
        this->prev = l->prev;

        l->prev = this;
        this->next = l;
    }
    inline void removeSelf()
    {
        this->next->prev = this->prev;
        this->prev->next = this->next;

        this->next = this->prev = this;
    }
    inline bool isEmpty()
    {
        return this->next == this;
    }
    inline uint32_t len()
    {
        uint32_t len = 0;
        const mList_t *p = this;
        while (p->next != this)
        {
            p = p->next;
            len ++;
        }
        return len;
    }
};
/**
 * Single List structure
 */
struct sList_t
{
    struct sList_t *next;                         /**< point to next node. */
    sList_t()
    {
        next = nullptr;
    }
    ~sList_t()
    {
        next = nullptr;
    }
    inline void appendTo(sList_t* l)
    {
        struct sList_t *node;

        node = l;
        while (node->next) node = node->next;

        /* append the node to the tail */
        node->next = this;
        this->next = nullptr;
    }
    inline void insertTo(sList_t* l)
    {
        this->next = l->next;
        l->next = this;
    }
    inline uint32_t len()
    {
        uint32_t len = 0;
        const sList_t *list = this->next;
        while (list != nullptr)
        {
            list = list->next;
            len ++;
        }

        return len;
    }
    inline sList_t *removeFrom(sList_t *l)
    {
        /* remove slist head */
        struct sList_t *node = l;
        while (node->next && node->next != this) node = node->next;

        /* remove node */
        if (node->next != (sList_t *)0) node->next = node->next->next;

        return l;
    }
    inline sList_t *first()
    {
        return this->next;
    }

    inline sList_t *tail()
    {
        sList_t* p = this;
        while (p->next) p = p->next;

        return p;
    }

    inline sList_t *getNext()
    {
        return this->next;
    }

    inline bool isEmpty()
    {
        return this->next == nullptr;
    }
};

/**
 * Base structure of Kernel object
 */
struct mObject_t
{
    char       name[NAME_MAX];                       /**< name of kernel object */
    uint8_t type;                                    /**< type of kernel object */
    uint8_t flag;                                    /**< flag of kernel object */

    mList_t list;                                    /**< list node of kernel object */
};
/**
 *  The object type can be one of the follows with specific
 *  macros enabled:
 *  - Thread
 *  - Semaphore
 *  - Mutex
 *  - Event
 *  - MailBox
 *  - MessageQueue
 *  - MemHeap
 *  - MemPool
 *  - Device
 *  - Timer
 *  - Unknown
 *  - Static
 */
enum mObjectClassType
{
    M_OBJECT_CLASS_NULL          = 0x00,      /**< The object is not used. */
    M_OBJECT_CLASS_THREAD        = 0x01,      /**< The object is a thread. */
    M_OBJECT_CLASS_SEMAPHORE     = 0x02,      /**< The object is a semaphore. */
    M_OBJECT_CLASS_MUTEX         = 0x03,      /**< The object is a mutex. */
    M_OBJECT_CLASS_EVENT         = 0x04,      /**< The object is a event. */
    M_OBJECT_CLASS_MAILBOX       = 0x05,      /**< The object is a mail box. */
    M_OBJECT_CLASS_MESSAGEQUEUE  = 0x06,      /**< The object is a message queue. */
    M_OBJECT_CLASS_MEMHEAP       = 0x07,      /**< The object is a memory heap. */
    M_OBJECT_CLASS_MEMPOLL       = 0x08,      /**< The object is a memory pool. */
    M_OBJECT_CLASS_DEVICE        = 0x09,      /**< The object is a device. */
    M_OBJECT_CLASS_TIMER         = 0x0a,      /**< The object is a timer. */
    M_OBJECT_CLASS_UNKNOWN       = 0x0c,      /**< The object is unknown. */
    M_OBJECT_CLASS_STATIC        = 0x80       /**< The object is a static object. */
};

/**
 * The information of the kernel object
 */
struct mObjectInformation_t
{
    enum mObjectClassType type;                     /**< object class type */
    mList_t                 objectList;              /**< object list */
    //size_t                  objectSize;              /**< object size */
};

/**
 * clock & timer macros
 */
enum mTimerStateFlag
{
    TIMER_STATE_FLAG_DEACTIVATED    =   0x0,             /**< timer is deactive */
    TIMER_STATE_FLAG_ACTIVATED      =   0x1,             /**< timer is active */
    TIMER_FLAG_ONE_SHOT       =   0x0,             /**< one shot timer */
    TIMER_FLAG_PERIODIC       =   0x2,             /**< periodic timer */
    TIMER_FLAG_HARD_TIMER     =   0x0,             /**< hard timer,the timer's callback function will be called in tick isr. */
    TIMER_FLAG_SOFT_TIMER     =   0x4,             /**< soft timer,the timer's callback function will be called in timer thread. */
};

enum mTimerCtrl
{
    TIMER_CTRL_SET_TIME       =   0x0,             /**< set timer control command */
    TIMER_CTRL_GET_TIME       =   0x1,             /**< get timer control command */
    TIMER_CTRL_SET_ONESHOT    =   0x2,             /**< change timer to one shot */
    TIMER_CTRL_SET_PERIODIC   =   0x3,             /**< change timer to periodic */
    TIMER_CTRL_GET_STATE      =   0x4,             /**< get timer run state active or deactive*/
};

/**
 * timer structure
 */
struct mTimer_t : public mObject_t
{
    //struct rt_object parent;                          /**< inherit from rt_object */
    mList_t        row[TIMER_SKIP_LIST_LEVEL];

    void (*timeoutFunc)(void *parameter);               /**< timeout function */
    void            *parameter;                         /**< timeout function's parameter */

    uint32_t        initTick;                           /**< timer timeout tick */
    uint32_t        timeoutTick;                        /**< timeout tick */
};

/**
 * Thread structure
 */
struct thread_t : public mObject_t
{
    mList_t   tlist;                                  /**< the thread list */

    /* stack point and entry */
    void       *sp;                                     /**< stack point */
    void       *entry;                                  /**< entry */
    void       *parameter;                              /**< parameter */
    void       *stackAddr;                             /**< stack address */
    uint32_t stackSize;                             /**< stack size */
    void (*extiThread)();
    mResult (*yieldThread)();

    /* error code */
    mResult    error;                                  /**< error code */

    uint8_t  stat;                                   /**< thread status */

    /* priority */
    uint8_t  currentPriority;                       /**< current priority */
    uint8_t  initPriority;                          /**< initialized priority */
#if THREAD_PRIORITY_MAX > 32
    uint8_t  number;
    uint8_t  high_mask;
#endif
    uint32_t numberMask;

    /* thread event */
    uint32_t eventSet;
    uint8_t  eventInfo;

    unsigned long  initTick;                              /**< thread's initialized tick */
    unsigned long  remainingTick;                         /**< remaining tick */

    void (*cleanup)(struct thread_t *tid);             /**< cleanup function when thread exit */

    uint32_t userData;                              /**< private user data beyond this thread */
};

/**
 * IPC flags and control command definitions
 */
enum mIpcFlag
{
    IPC_FLAG_FIFO        =        0x00,            /**< FIFOed IPC. @ref IPC. */
    IPC_FLAG_PRIO        =        0x01            /**< PRIOed IPC. @ref IPC. */
};

enum mIpcCmd
{
    IPC_CMD_UNKNOWN      =        0x00,            /**< unknown IPC command */
    IPC_CMD_RESET        =        0x01            /**< reset IPC object */
};

/**
 * Base structure of IPC object
 */
struct mIpcObject_t : public mObject_t
{
    mList_t      suspendThread;                    /**< threads pended on this resource */
};
/**
 * Semaphore structure
 */
struct mSemaphore_t : public mIpcObject_t
{
    uint16_t          value;                         /**< value of semaphore. */
    uint16_t          reserved;                      /**< reserved field */
};

/**
 * Mutual exclusion (mutex) structure
 */
struct mMutex_t : public mIpcObject_t
{
    uint16_t          value;                         /**< value of mutex */

    uint8_t           originalPriority;             /**< priority of last thread hold the mutex */
    uint8_t           hold;                          /**< numbers of thread hold the mutex */

    struct thread_t   *owner;                       /**< current owner of mutex */
};

/**
 * flag defintions in event
 */
enum mEventFlag
{
    EVENT_FLAG_AND         =      0x01,            /**< logic and */
    EVENT_FLAG_OR          =      0x02,            /**< logic or */
    EVENT_FLAG_CLEAR       =      0x04,            /**< clear flag */
};

/*
 * event structure
 */
struct mEvent_t : public mIpcObject_t
{
    uint32_t          set;                           /**< event set */
};

/**
 * message queue structure
 */
struct mMessagequeue_t : public mIpcObject_t
{
    uint8_t             *msgPool;                      /**< start address of message queue */

    uint16_t            msgSize;                      /**< message size of each message */
    uint16_t            maxMsgs;                      /**< max number of messages */

    uint16_t            entry;                         /**< index of messages in the queue */

    void                *msgQueueHead;                /**< list head */
    void                *msgQueueTail;                /**< list tail */
    void                *msgQueueFree;                /**< pointer indicated the free node of queue */

    mList_t             suspendSenderThread;         /**< sender thread suspended on this message queue */
};

#define rt_kprintf printf