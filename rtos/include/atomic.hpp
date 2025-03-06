#pragma once
#include "mhw.hpp"
template <typename T>
class mAtomic
{
#if defined (__GNUC__)                /* GNU GCC Compiler */
public:
    mAtomic() = default;
    explicit mAtomic(T value) : _value(value) {}
    inline T load()
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval, &_value)) != 0U);
        return oldval;
    }
    /**
	 * Atomically store a value
	 */
	inline void store(T value)
	{
        do
        {
            __LDREXW(&_value);
        } while ((__STREXW(value, &_value)) != 0U);
	}
	/**
	 * Atomically add a number and return the previous value.
	 * @return value prior to the addition
	 */
	inline void fetchAdd(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval + value, &_value)) != 0U);
    }
	/**
	 * Atomically substract a number and return the previous value.
	 * @return value prior to the substraction
	 */
	inline void fetchSub(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval - value, &_value)) != 0U);
    }
	/**
	 * Atomic AND with a number
	 * @return value prior to the operation
	 */
	inline void fetchAnd(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval & value, &_value)) != 0U);
    }
	/**
	 * Atomic XOR with a number
	 * @return value prior to the operation
	 */
	inline void fetchXor(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval ^ value, &_value)) != 0U);
    }
	/**
	 * Atomic OR with a number
	 * @return value prior to the operation
	 */
	inline void fetchOr(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval | value, &_value)) != 0U);
    }
	/**
	 * Atomic compare and exchange operation.
	 * This compares the contents of _value with the contents of *expected. If
	 * equal, the operation is a read-modify-write operation that writes desired
	 * into _value. If they are not equal, the operation is a read and the current
	 * contents of _value are written into *expected.
	 * @return If desired is written into _value then true is returned
	 */
	inline T compareExchange(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(value, &_value)) != 0U);
        return oldval;
    }
    inline void flagClear()
    {
        do
        {
            __LDREXW(&_value);
        } while ((__STREXW(0, &_value)) != 0U);
    }
    inline void flagTestAndSet()
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(1, &_value)) != 0U);
    }
    inline bool exchangeStrong(T *old, T newval)
    {
        T result;
        T temp = *old;
        do
        {
            result = __LDREXW(&_value);
            if (result != temp)
            {
                *old = result;
                __STREXW(result, &_value);
                break;
            }
        } while ((__STREXW(newval, &_value)) != 0U);
        return (result == temp);
    }
    inline void operator++()
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval + 1, &_value)) != 0U);
    }
    inline void operator++(int)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval + 1, &_value)) != 0U);
    }
    inline void operator--()
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval - 1, &_value)) != 0U);
    }
    inline void operator--(int)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval - 1, &_value)) != 0U);
    }
    inline void operator|=(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval | value, &_value)) != 0U);
    }
    inline void operator&=(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval & value, &_value)) != 0U);
    }
    inline void operator^=(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval ^ value, &_value)) != 0U);
    }
    inline void operator+=(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval + value, &_value)) != 0U);
    }
    inline void operator-=(T value)
    {
        T oldval;
        do
        {
            oldval = __LDREXW(&_value);
        } while ((__STREXW(oldval - value, &_value)) != 0U);
    }
private:
    __attribute__((always_inline))     static inline T __LDREXW(volatile T *addr)
    {
        T result;

        __asm volatile ("ldrex %0, %1" : "=r" (result) : "Q" (*addr) );
        return result;
    }
    __attribute__((always_inline))     static inline T __STREXW(volatile T value, volatile T *addr)
    {
        T result;

        __asm volatile ("strex %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" (value) );
        return result;
    }
private:
    volatile T _value;

#endif
};