#ifndef __MBASE_H__
#define __MBASE_H__

/* maximum value of base type */
#define TICK_MAX                     UINT32_MAX   /**< Maxium number of tick */

/* maximum value of ipc type */
#define SEM_VALUE_MAX                UINT16_MAX   /**< Maxium number of semaphore .value */
#define MUTEX_VALUE_MAX              UINT16_MAX   /**< Maxium number of mutex .value */
#define MUTEX_HOLD_MAX               UINT8_MAX    /**< Maxium number of mutex .hold */
#define MB_ENTRY_MAX                 UINT16_MAX   /**< Maxium number of mailbox .entry */
#define MQ_ENTRY_MAX                 UINT16_MAX   /**< Maxium number of message queue .entry */

#define containerof(ptr, type, member) \
    ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))
/**
 * @brief get the struct for this entry
 * @param node the entry point
 * @param type the type of structure
 * @param member the name of list in structure
 */
#define listEntry(node, type, member) \
    containerof(node, type, member)

#define M_ALIGN_SIZE  4

/**
 * @ingroup BasicDef
 *
 * @def ALIGN(size, align)
 * Return the most contiguous size aligned at specified width. ALIGN(13, 4)
 * would return 16.
 */
#define M_ALIGN(size, align)           (((size) + (align) - 1) & ~((align) - 1))

/**
 * @ingroup BasicDef
 *
 * @def ALIGN_DOWN(size, align)
 * Return the down number of aligned at specified width. ALIGN_DOWN(13, 4)
 * would return 12.
 */
#define M_ALIGN_DOWN(size, align)      ((size) & ~((align) - 1))


/* the version of GNU GCC must be greater than 4.x */
typedef __builtin_va_list       __gnuc_va_list;
typedef __gnuc_va_list          va_list;
#define va_start(v,l)           __builtin_va_start(v,l)
#define va_end(v)               __builtin_va_end(v)
#define va_arg(v,l)             __builtin_va_arg(v,l)


#define SECTION(x)                  __attribute__((section(x)))
#define M_UNUSED                    __attribute__((unused))
#define M_USED                      __attribute__((used))
#define ALIGN(n)                    __attribute__((aligned(n)))
#define M_WEAK                      __attribute__((weak))

typedef int (*initFunc)(void);
struct initDesc_t
{
    const char* fnName; //函数名称
    const initFunc fn;  //函数指针
};

#define INIT_EXPORT(fn, level)                                                       \
    const char __init_##fn##_name[] = #fn;                                            \
    M_USED const struct initDesc_t __initDesc_t_##fn SECTION(".initfunc." level) = \
    { __init_##fn##_name, fn};

#endif