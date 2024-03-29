#include "rtoscommon.h"
#include "mhw.h"
#include "mirq.h"
#include "mclock.h"
#include "sys.h"
#include "mmem.h"

#define MEM_HEAP_SIZE 50 * 1024 // 256K
D2_MEM_ALIGN(M_ALIGN_SIZE) static uint8_t memHeap[MEM_HEAP_SIZE];

extern "C" void SysTick_Handler(void)
{
    mIrq::getInstance()->interruptEnter();

    mClock::getInstance()->tickIncrease();
    mIrq::getInstance()->interruptLeave();
}

void boardInit(void)
{
    hwInit();
    mMem::getInstance()->init(memHeap, memHeap + MEM_HEAP_SIZE);
}
