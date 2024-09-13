#include "rtoscommon.hpp"
#include "cpuport.hpp"
#include "mthread.hpp"

#if               ( /* GNU */(defined ( __GNUC__ ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)))
#define USE_FPU   1
#else
#define USE_FPU   0
#endif

/* exception and interrupt handler table */
uint32_t mInterruptFromThread = 0;
uint32_t mtInterruptToThread = 0;
uint32_t mtThreadSwitchInterruptFlag = 0;

uint8_t *CPUPORT::hwStackInit( void    *tentry,
                        void    *parameter,
                        uint8_t *stackAddr,
                        void    *texit)
{
    struct stackFrame *stackFrame;
    uint8_t         *stk;
    unsigned long       i;

    stk  = stackAddr + sizeof(uint32_t);
    stk  = (uint8_t *)M_ALIGN_DOWN((uint32_t)stk, 8);
    stk -= sizeof(struct stackFrame);

    stackFrame = (struct stackFrame *)stk;

    /* init all register */
    for (i = 0; i < sizeof(struct stackFrame) / sizeof(uint32_t); i ++)
    {
        ((uint32_t *)stackFrame)[i] = 0xdeadbeef;
    }

    stackFrame->exceptionStackFrame.r0  = (unsigned long)parameter; /* r0 : argument */
    stackFrame->exceptionStackFrame.r1  = 0;                        /* r1 */
    stackFrame->exceptionStackFrame.r2  = 0;                        /* r2 */
    stackFrame->exceptionStackFrame.r3  = 0;                        /* r3 */
    stackFrame->exceptionStackFrame.r12 = 0;                        /* r12 */
    stackFrame->exceptionStackFrame.lr  = (unsigned long)(((thread_t*)texit)->extiThread);     /* lr */
    stackFrame->exceptionStackFrame.pc  = (unsigned long)tentry;    /* entry point, pc */
    stackFrame->exceptionStackFrame.psr = 0x01000000L;              /* PSR */

#if USE_FPU
    stackFrame->flag = 0;
#endif /* USE_FPU */

    /* return task's current stack address */
    return stk;
}
void CPUPORT::hwHardFaultException(exceptionInfo_t *exceptionInfo)
{
    extern long list_thread(void);
    //struct exceptionStackFrame *exceptionStack = &exceptionInfo->stackFrame.exceptionStackFrame;
    struct stackFrame *context = &exceptionInfo->stackFrame;

    printf("psr: 0x%08lx\r\n", context->exceptionStackFrame.psr);

    printf("r00: 0x%08lx\r\n", context->exceptionStackFrame.r0);
    printf("r01: 0x%08lx\r\n", context->exceptionStackFrame.r1);
    printf("r02: 0x%08lx\r\n", context->exceptionStackFrame.r2);
    printf("r03: 0x%08lx\r\n", context->exceptionStackFrame.r3);
    printf("r04: 0x%08lx\r\n", context->r4);
    printf("r05: 0x%08lx\r\n", context->r5);
    printf("r06: 0x%08lx\r\n", context->r6);
    printf("r07: 0x%08lx\r\n", context->r7);
    printf("r08: 0x%08lx\r\n", context->r8);
    printf("r09: 0x%08lx\r\n", context->r9);
    printf("r10: 0x%08lx\r\n", context->r10);
    printf("r11: 0x%08lx\r\n", context->r11);
    printf("r12: 0x%08lx\r\n", context->exceptionStackFrame.r12);
    printf(" lr: 0x%08lx\r\n", context->exceptionStackFrame.lr);
    printf(" pc: 0x%08lx\r\n", context->exceptionStackFrame.pc);

    if (exceptionInfo->excReturn & (1 << 2))
    {
        printf("hard fault on thread: %s\r\n\r\n", mthread::threadSelf()->name);
    }
    else
    {
        printf("hard fault on handler\r\n\r\n");
    }

    if ( (exceptionInfo->excReturn & 0x10) == 0)
    {
        printf("FPU active!\r\n");
    }
    printf("you can use the command to find whitch line caused this fault: \r\n");
    printf("arm-none-eabi-addr2line -e xxx.elf -a -f %lx\r\n",context->exceptionStackFrame.pc);
    while (1);
}
