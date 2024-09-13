#include "rtoscommon.hpp"
#include "mhw.hpp"
#include "mirq.hpp"
#include "mclock.hpp"
#include "sys.h"
#include "mmem.hpp"

#define MEM_HEAP_SIZE 50 * 1024 // 256K
D2_MEM_ALIGN(M_ALIGN_SIZE) static uint8_t memHeap[MEM_HEAP_SIZE];

extern "C" void SysTick_Handler(void)
{
    mIrq::getInstance()->interruptEnter();
    HAL_IncTick();
    mClock::getInstance()->tickIncrease();
    mIrq::getInstance()->interruptLeave();
}

void boardInit(void)
{
    hwInit();
    mMem::getInstance()->init(memHeap, memHeap + MEM_HEAP_SIZE);
}
