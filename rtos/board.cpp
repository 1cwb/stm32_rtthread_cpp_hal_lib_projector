#include "rtoscommon.hpp"
#include "mhw.hpp"
#include "mirq.hpp"
#include "mclock.hpp"
#include "sys.h"
#include "mmem.hpp"
#include "mklog.hpp"
#include "usart.h"
#include "fatfsff.hpp"

#if USE_SDRAM
#define MEM_HEAP_SIZE 16384 * 1024 // 16M
static uint8_t* memHeap = SDRAM_MEM_ADDR;
#else
#define MEM_HEAP_SIZE 256 * 1024 // 256K
D2_MEM_ALIGN(M_ALIGN_SIZE) static uint8_t memHeap[MEM_HEAP_SIZE];
#endif

class LOG : public mKlog
{
public:
    static LOG* getInstance()
    {
        static LOG logx;
        return &logx;
    }
    virtual void saveLogToFile(const std::string& fileName) override
    {
        FRESULT res;
        if((res = file.open(fileName.c_str(), FA_CREATE_ALWAYS | FA_WRITE | FA_OPEN_APPEND)) == FRESULT::FR_OK)
        {
            printf("open %s success\r\n", fileName.c_str());
            bFileOpen = true;
            setOutputToFile(true);
        }
        else
        {
            printf("open %s failed, Reason: (%s)\r\n", fileName.c_str(), mFatFs::errToStr(res));
            bFileOpen = false;
            setOutputToFile(false);
        }
    }
private:
    LOG():bFileOpen(false)
    {
        registerSelf(this);
    }
    virtual ~LOG() {file.close();}
    virtual mResult send(const uint8_t* data, uint32_t len) override
    {
        unsigned int bw;
        FRESULT res;
        if(isOutputToFile() && bFileOpen)
        {
            if((res = file.write(data, len, &bw)) == FRESULT::FR_OK)
            {
                file.sync();
            }
            else
            {
                bFileOpen = false;
                setOutputToFile(false);
            }
        }
        else
        {
            HAL_UART_Transmit(&UART1_Handler,data,len,100);
        }
        return M_RESULT_EOK;
    }
private:
    bool bFileOpen;
    mFile file;
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
