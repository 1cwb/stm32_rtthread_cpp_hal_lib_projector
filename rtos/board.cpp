#include "rtoscommon.hpp"
#include "mhw.hpp"
#include "mirq.hpp"
#include "mclock.hpp"
#include "sys.h"
#include "mmem.hpp"
#include "mklog.hpp"
#include "usart.h"

#define MEM_HEAP_SIZE 64 * 1024 // 256K
D2_MEM_ALIGN(M_ALIGN_SIZE) static uint8_t memHeap[MEM_HEAP_SIZE];

class LOG : public mKlog
{
public:
    static LOG* getInstance()
    {
        static LOG logx;
        return &logx;
    }
private:
    LOG(){registerSelf(this);}
    virtual ~LOG() = default;
    virtual mResult send(const uint8_t* data, uint32_t len)
    {
        HAL_UART_Transmit(&UART1_Handler,data,len,100);
        return M_RESULT_EOK;
    }
};

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
    LOG::getInstance()->setLevel(LOG_LEVEL_DEBUG);
    mMem::getInstance()->init(memHeap, memHeap + MEM_HEAP_SIZE);
}
