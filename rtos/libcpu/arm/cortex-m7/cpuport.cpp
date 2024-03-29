#include "rtoscommon.h"
#include "cpuport.h"

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
    #if 0
    extern long list_thread(void);
    struct exceptionStackFrame *exceptionStack = &exceptionInfo->stackFrame.exceptionStackFrame;
    struct stackFrame *context = &exceptionInfo->stackFrame;

    if (rt_exception_hook != RT_NULL)
    {
        rt_err_t result;

        result = rt_exception_hook(exception_stack);
        if (result == RT_EOK) return;
    }

    rt_kprintf("psr: 0x%08x\n", context->exceptionStackFrame.psr);

    rt_kprintf("r00: 0x%08x\n", context->exceptionStackFrame.r0);
    rt_kprintf("r01: 0x%08x\n", context->exceptionStackFrame.r1);
    rt_kprintf("r02: 0x%08x\n", context->exceptionStackFrame.r2);
    rt_kprintf("r03: 0x%08x\n", context->exceptionStackFrame.r3);
    rt_kprintf("r04: 0x%08x\n", context->r4);
    rt_kprintf("r05: 0x%08x\n", context->r5);
    rt_kprintf("r06: 0x%08x\n", context->r6);
    rt_kprintf("r07: 0x%08x\n", context->r7);
    rt_kprintf("r08: 0x%08x\n", context->r8);
    rt_kprintf("r09: 0x%08x\n", context->r9);
    rt_kprintf("r10: 0x%08x\n", context->r10);
    rt_kprintf("r11: 0x%08x\n", context->r11);
    rt_kprintf("r12: 0x%08x\n", context->exceptionStackFrame.r12);
    rt_kprintf(" lr: 0x%08x\n", context->exceptionStackFrame.lr);
    rt_kprintf(" pc: 0x%08x\n", context->exceptionStackFrame.pc);

    if (exceptionInfo->excReturn & (1 << 2))
    {
        rt_kprintf("hard fault on thread: %s\r\n\r\n", rt_thread_self()->name);

#ifdef RT_USING_FINSH
        list_thread();
#endif
    }
    else
    {
        rt_kprintf("hard fault on handler\r\n\r\n");
    }

    if ( (exception_info->excReturn & 0x10) == 0)
    {
        rt_kprintf("FPU active!\r\n");
    }

#ifdef RT_USING_FINSH
    hard_fault_track();
#endif /* RT_USING_FINSH */

    while (1);
    #endif
}
