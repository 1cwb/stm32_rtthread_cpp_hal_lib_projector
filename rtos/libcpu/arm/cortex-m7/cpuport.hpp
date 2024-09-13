#pragma once
#include "rtoscommon.hpp"

#if               ( /* GNU */(defined ( __GNUC__ ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)))
#define USE_FPU   1
#else
#define USE_FPU   0
#endif
namespace CPUPORT{
extern "C"{

/* exception and interrupt handler table */
//uint32_t mInterruptFromThread;
//uint32_t mtInterruptToThread;
//uint32_t mtThreadSwitchInterruptFlag;


/* exception hook */
//static int32_t (*rt_exception_hook)(void *context) = nullptr;

struct exceptionStackFrame
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
};

struct stackFrame
{
#if USE_FPU
    uint32_t flag;
#endif /* USE_FPU */

    /* r4 ~ r11 register */
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;

    struct exceptionStackFrame exceptionStackFrame;
};

struct exceptionStackFrameFpu
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;

#if USE_FPU
    /* FPU register */
    uint32_t S0;
    uint32_t S1;
    uint32_t S2;
    uint32_t S3;
    uint32_t S4;
    uint32_t S5;
    uint32_t S6;
    uint32_t S7;
    uint32_t S8;
    uint32_t S9;
    uint32_t S10;
    uint32_t S11;
    uint32_t S12;
    uint32_t S13;
    uint32_t S14;
    uint32_t S15;
    uint32_t FPSCR;
    uint32_t NO_NAME;
#endif
};

struct stackFrameFpu
{
    uint32_t flag;

    /* r4 ~ r11 register */
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;

#if USE_FPU
    /* FPU register s16 ~ s31 */
    uint32_t s16;
    uint32_t s17;
    uint32_t s18;
    uint32_t s19;
    uint32_t s20;
    uint32_t s21;
    uint32_t s22;
    uint32_t s23;
    uint32_t s24;
    uint32_t s25;
    uint32_t s26;
    uint32_t s27;
    uint32_t s28;
    uint32_t s29;
    uint32_t s30;
    uint32_t s31;
#endif

    struct exceptionStackFrameFpu exceptionStackFrame;
};

uint8_t *hwStackInit( void    *tentry,
                        void    *parameter,
                        uint8_t *stackAddr,
                        void    *texit);
struct exceptionInfo_t
{
    uint32_t excReturn;
    struct stackFrame stackFrame;
};
void hwHardFaultException(exceptionInfo_t *exceptionInfo);

}//extern "C"
}//namespace CPUPORT