/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-10-11     Bernard      first version
 * 2012-01-01     aozima       support context switch load/store FPU register.
 * 2013-06-18     aozima       add restore MSP feature.
 * 2013-06-23     aozima       support lazy stack optimized.
 * 2018-07-24     aozima       enhancement hard fault exception handler.
 */

/**
 * @addtogroup cortex-m4
 */
/*@{*/

.cpu cortex-m4
.syntax unified
.thumb
.text

.equ    SCB_VTOR,           0xE000ED08              /* Vector Table Offset Register */
.equ    NVIC_INT_CTRL,      0xE000ED04              /* interrupt control state register */
.equ    NVIC_SYSPRI2,       0xE000ED20              /* system priority register (2) */
.equ    NVIC_PENDSV_PRI,    0x00FF0000              /* PendSV priority value (lowest) */
.equ    NVIC_PENDSVSET,     0x10000000              /* value to trigger PendSV exception */

/*
 * rt_base_t hwInterruptDisable();
 */
.global hwInterruptDisable
.type hwInterruptDisable, %function
hwInterruptDisable:
    MRS     r0, PRIMASK
    CPSID   I
    BX      LR

/*
 * void hwInterruptEnable(rt_base_t level);
 */
.global hwInterruptEnable
.type hwInterruptEnable, %function
hwInterruptEnable:
    MSR     PRIMASK, r0
    BX      LR

/*
 * void hwContextSwitch(rt_uint32 from, rt_uint32 to);
 * r0 --> from
 * r1 --> to
 */
.global hwContextSwitchInterrupt
.type hwContextSwitchInterrupt, %function
.global hwContextSwitch
.type hwContextSwitch, %function

hwContextSwitchInterrupt:
hwContextSwitch:
    /* set mtThreadSwitchInterruptFlag to 1 */
    LDR     r2, =mtThreadSwitchInterruptFlag
    LDR     r3, [r2]
    CMP     r3, #1
    BEQ     _reswitch
    MOV     r3, #1
    STR     r3, [r2]

    LDR     r2, =mInterruptFromThread   /* set mInterruptFromThread */
    STR     r0, [r2]

_reswitch:
    LDR     r2, =mtInterruptToThread     /* set mtInterruptToThread */
    STR     r1, [r2]

    LDR r0, =NVIC_INT_CTRL              /* trigger the PendSV exception (causes context switch) */
    LDR r1, =NVIC_PENDSVSET
    STR r1, [r0]
    BX  LR

/* r0 --> switch from thread stack
 * r1 --> switch to thread stack
 * psr, pc, lr, r12, r3, r2, r1, r0 are pushed into [from] stack
 */
.global PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:
    /* disable interrupt to protect context switch */
    MRS r2, PRIMASK
    CPSID   I

    /* get mtThreadSwitchInterruptFlag */
    LDR r0, =mtThreadSwitchInterruptFlag
    LDR r1, [r0]
    CBZ r1, pendsv_exit         /* pendsv already handled */

    /* clear mtThreadSwitchInterruptFlag to 0 */
    MOV r1, #0x00
    STR r1, [r0]

    LDR r0, =mInterruptFromThread
    LDR r1, [r0]
    CBZ r1, switch_to_thread    /* skip register save at the first time */

    MRS r1, psp                 /* get from thread stack pointer */
    
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    TST     lr, #0x10           /* if(!EXC_RETURN[4]) */
    VSTMDBEQ r1!, {d8 - d15}    /* push FPU register s16~s31 */
#endif
    
    STMFD   r1!, {r4 - r11}     /* push r4 - r11 register */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    MOV     r4, #0x00           /* flag = 0 */

    TST     lr, #0x10           /* if(!EXC_RETURN[4]) */
    MOVEQ   r4, #0x01           /* flag = 1 */

    STMFD   r1!, {r4}           /* push flag */
#endif

    LDR r0, [r0]
    STR r1, [r0]                /* update from thread stack pointer */

switch_to_thread:
    LDR r1, =mtInterruptToThread
    LDR r1, [r1]
    LDR r1, [r1]                /* load thread stack pointer */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    LDMFD   r1!, {r3}           /* pop flag */
#endif

    LDMFD   r1!, {r4 - r11}     /* pop r4 - r11 register */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    CMP     r3,  #0             /* if(flag_r3 != 0) */
    VLDMIANE  r1!, {d8 - d15}   /* pop FPU register s16~s31 */
#endif

    MSR psp, r1                 /* update stack pointer */

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    ORR     lr, lr, #0x10       /* lr |=  (1 << 4), clean FPCA. */
    CMP     r3,  #0             /* if(flag_r3 != 0) */
    BICNE   lr, lr, #0x10       /* lr &= ~(1 << 4), set FPCA. */
#endif

pendsv_exit:
    /* restore interrupt */
    MSR PRIMASK, r2

    ORR lr, lr, #0x04
    BX  lr

/*
 * void hwContextSwitchTo(rt_uint32 to);
 * r0 --> to
 */
.global hwContextSwitchTo
.type hwContextSwitchTo, %function
hwContextSwitchTo:
    LDR r1, =mtInterruptToThread
    STR r0, [r1]

#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    /* CLEAR CONTROL.FPCA */
    MRS     r2, CONTROL         /* read */
    BIC     r2, #0x04           /* modify */
    MSR     CONTROL, r2         /* write-back */
#endif

    /* set from thread to 0 */
    LDR r1, =mInterruptFromThread
    MOV r0, #0x0
    STR r0, [r1]

    /* set interrupt flag to 1 */
    LDR     r1, =mtThreadSwitchInterruptFlag
    MOV     r0, #1
    STR     r0, [r1]

    /* set the PendSV exception priority */
    LDR r0, =NVIC_SYSPRI2
    LDR r1, =NVIC_PENDSV_PRI
    LDR.W   r2, [r0,#0x00]       /* read       */
    ORR     r1,r1,r2             /* modify     */
    STR     r1, [r0]             /* write-back */

    LDR r0, =NVIC_INT_CTRL      /* trigger the PendSV exception (causes context switch) */
    LDR r1, =NVIC_PENDSVSET
    STR r1, [r0]

    /* restore MSP */
    LDR     r0, =SCB_VTOR
    LDR     r0, [r0]
    LDR     r0, [r0]
    NOP
    MSR     msp, r0

    /* enable interrupts at processor level */
    CPSIE   F
    CPSIE   I

    /* never reach here! */

.global HardFault_Handler
.type HardFault_Handler, %function
HardFault_Handler:
    /* get current context */
    MRS     r0, msp                 /* get fault context from handler. */
    TST     lr, #0x04               /* if(!EXC_RETURN[2]) */
    BEQ     _get_sp_done
    MRS     r0, psp                 /* get fault context from thread. */
_get_sp_done:

    STMFD   r0!, {r4 - r11}         /* push r4 - r11 register */
#if defined (__VFP_FP__) && !defined(__SOFTFP__)
    STMFD   r0!, {lr}               /* push dummy for flag */
#endif
    STMFD   r0!, {lr}               /* push exec_return register */

    TST     lr, #0x04               /* if(!EXC_RETURN[2]) */
    BEQ     _update_msp
    MSR     psp, r0                 /* update stack pointer to PSP. */
    B       _update_done
_update_msp:
    MSR     msp, r0                 /* update stack pointer to MSP. */
_update_done:

    PUSH    {LR}
    BL      hwHardFaultException
    POP     {LR}

    ORR     lr, lr, #0x04
    BX      lr
