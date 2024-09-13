#pragma once
#include "rtoscommon.hpp"

namespace HW
{
#ifdef __cplusplus
extern "C" {
#endif

/*
 * Some macros define
 */
#ifndef HWREG32
#define HWREG32(x)          (*((volatile rt_uint32_t *)(x)))
#endif
#ifndef HWREG16
#define HWREG16(x)          (*((volatile rt_uint16_t *)(x)))
#endif
#ifndef HWREG8
#define HWREG8(x)           (*((volatile rt_uint8_t *)(x)))
#endif

#ifndef RT_CPU_CACHE_LINE_SZ
#define RT_CPU_CACHE_LINE_SZ    32
#endif

enum RT_HW_CACHE_OPS
{
    RT_HW_CACHE_FLUSH      = 0x01,
    RT_HW_CACHE_INVALIDATE = 0x02,
};

/*
 * CPU interfaces
 */
#if 0
void rt_hw_cpu_icache_enable(void);
void rt_hw_cpu_icache_disable(void);
long rt_hw_cpu_icache_status(void);
void rt_hw_cpu_icache_ops(int ops, void* addr, int size);

void rt_hw_cpu_dcache_enable(void);
void rt_hw_cpu_dcache_disable(void);
long rt_hw_cpu_dcache_status(void);
void rt_hw_cpu_dcache_ops(int ops, void* addr, int size);

void rt_hw_cpu_reset(void);
void rt_hw_cpu_shutdown(void);

/*
 * Interrupt handler definition
 */
typedef void (*isrhandler_t)(int vector, void *param);

struct rt_irq_desc
{
    isrhandler_t handler;
    void            *param;

#ifdef USING_INTERRUPT_INFO
    char             name[RT_NAME_MAX];
    uint32_t      counter;
#endif
};

/*
 * Interrupt interfaces
 */
void hwInterruptInit(void);
void hwInterruptMask(int vector);
void hwInterruptUmask(int vector);
isrhandler_t hwInterruptInstall(int vector, isrhandler_t handler, void *param, const char *name);
#endif


long hwInterruptDisable(void);
void hwInterruptEnable(long level);

/*
 * Context interfaces
 */
void hwContextSwitch(unsigned long from, unsigned long to);
void hwContextSwitchTo(unsigned long to);
void hwContextSwitchInterrupt(unsigned long from, unsigned long to);

void hwConsoleOutput(const char *str);

void hwBacktrace(uint32_t *fp, unsigned long thread_entry);
void hwShowMemory(uint32_t addr, size_t size);

/*
 * Exception interfaces
 */
//void hwExceptionInstall(mResult (*exception_handle)(void *context));

/*
 * delay interfaces
 */
void hwUsDelay(uint32_t us);

#define RT_DEFINE_SPINLOCK(x)
#define RT_DECLARE_SPINLOCK(x)    unsigned long x

#define rt_hw_spin_lock(lock)     *(lock) = hwInterruptDisable()
#define rt_hw_spin_unlock(lock)   hwInterruptEnable(*(lock))

#ifdef __cplusplus
}
#endif
}; // namespace HW
