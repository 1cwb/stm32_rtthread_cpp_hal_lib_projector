#pragma once
#include "rtoscommon.hpp"
#include "mhw.hpp"
class mIrq
{
public:
    static mIrq* getInstance()
    {
        static mIrq irq;
        return &irq;
    }
    uint8_t getInterruptNestWithoutDisableIsr() const;
    void interruptNestReset();
    /**
     * This function will be invoked by BSP, when enter interrupt service routine
     *
     * @note please don't invoke this routine in application
     *
     * @see interruptEnter
     */
    void interruptEnter(void);

    /**
     * This function will be invoked by BSP, when leave interrupt service routine
     *
     * @note please don't invoke this routine in application
     *
     * @see interruptLeave
     */
    void interruptLeave(void);

    /**
     * This function will return the nest of interrupt.
     *
     * User application can invoke this function to get whether current
     * context is interrupt context.
     *
     * @return the number of nested interrupts.
     */
    uint8_t interruptGetNest(void) const;

private:
    mIrq():interruptNest_(0)
    {
    }
    ~mIrq()
    {

    }
    mIrq(const mIrq&) = delete;
    mIrq(mIrq&&) = delete;
    mIrq& operator=(const mIrq&) = delete;
    mIrq& operator=(mIrq&&) = delete;

    volatile uint8_t interruptNest_;
};