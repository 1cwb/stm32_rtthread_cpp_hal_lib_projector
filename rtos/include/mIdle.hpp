#pragma once
#include "mthread.hpp"
#include "mmem.hpp"
#define IDLE_THREAD_STACK_SIZE  1024

class mIdle
{
public:
    static mIdle* getInstance()
    {
        static mIdle idle;
        return &idle;
    }
    void threadIdleInit(void);
    static void exec();
private:
    mIdle()
    {

    }
    ~mIdle()
    {

    }
    mIdle(const mIdle&) = delete;
    mIdle(mIdle&&) = delete;
    mIdle& operator=(const mIdle&) = delete;
    mIdle& operator=(mIdle&&) = delete;

    mthread idleThread_;
    ALIGN(M_ALIGN_SIZE) static uint8_t threadStack[IDLE_THREAD_STACK_SIZE];
};
