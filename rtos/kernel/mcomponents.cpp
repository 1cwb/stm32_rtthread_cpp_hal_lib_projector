//#pragma once 
#include "rtoscommon.h"
#include "mthread.h"
#include "mirq.h"
#include "mobject.h"
#include "mscheduler.h"
#include "mIdle.h"
#include "board.h"
#include "sys.h"

#define MAIN_THREAD_STACK_SIZE 1024*10
#define MAIN_THREAD_PRIORITY 0
#define MAIN_THREAD_TICK_TIME 20

/* if there is not enable heap, we should use static thread and stack. */
DTCM_MEM_ALIGN(M_ALIGN_SIZE) static uint8_t mainStack[MAIN_THREAD_STACK_SIZE];

void mainThreadEntry(void *parameter)
{
    extern int main(void);
    main();
}

class mComponets
{
public:
    static mComponets* getInstance()
    {
        static mComponets com;
        return &com;
    }
    void startUp()
    {
        HW::hwInterruptDisable();
        mSchedule::getInstance()->systemSchedulerInit();
        applicationInit();
        mIdle::getInstance()->threadIdleInit();
        mSchedule::getInstance()->systemSchedulerStart();
    }
    void applicationInit(void)
    {
        mResult result;
        result = mainThread_.init("main", mainThreadEntry, nullptr,
                                mainStack, sizeof(mainStack), MAIN_THREAD_PRIORITY, MAIN_THREAD_TICK_TIME);
        MASSERT(result == M_RESULT_EOK);

        /* if not define RT_USING_HEAP, using to eliminate the warning */
        (void)result;
        mainThread_.startup();
    }
private:
    mComponets()
    {
    }
    ~mComponets()
    {

    }
    mComponets(const mComponets&) = delete;
    mComponets(mComponets&&) = delete;
    mComponets& operator=(const mComponets&) = delete;
    mComponets& operator=(mComponets&&) = delete;

    mthread mainThread_;
};

extern "C" int entry(void)
{
    boardInit();
    mComponets::getInstance()->startUp();
    return 0;
}