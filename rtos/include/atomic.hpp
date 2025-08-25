#pragma once
#include "mhw.hpp"
#include <type_traits>

// 内存序枚举（简化版）
enum memory_order {
    relaxed = __ATOMIC_RELAXED,
    consume = __ATOMIC_CONSUME,
    acquire = __ATOMIC_ACQUIRE,
    release = __ATOMIC_RELEASE,
    acq_rel = __ATOMIC_ACQ_REL,
    seq_cst = __ATOMIC_SEQ_CST
};

template <typename T>
class mAtomic
{
    static_assert(std::is_integral<T>::value || std::is_pointer<T>::value,
                 "mAtomic only supports integral and pointer types");
    static_assert(sizeof(T) <= sizeof(void*),
                 "mAtomic does not support types larger than pointer size");

public:
    mAtomic() noexcept = default;
    explicit mAtomic(T value) noexcept : _value(value) {}
    
    // 禁止拷贝和赋值
    mAtomic(const mAtomic&) = delete;
    mAtomic& operator=(const mAtomic&) = delete;

    // 加载操作
    template <memory_order order = seq_cst>
    T load() noexcept {
        T oldval;
        do {
            oldval = __LDREXW(&_value);
        } while (__STREXW(oldval, &_value) != 0U);
        return oldval;
    }

    // 存储操作
    template <memory_order order = seq_cst>
    void store(T value) noexcept {
        do {
            __LDREXW(&_value);
        } while (__STREXW(value, &_value) != 0U);
    }

    // 交换操作
    template <memory_order order = seq_cst>
    T exchange(T value) noexcept {
        T oldval;
        do {
            oldval = __LDREXW(&_value);
        } while (__STREXW(value, &_value) != 0U);
        return oldval;
    }

    // 比较交换操作
    template <memory_order order = seq_cst>
    bool compare_exchange_strong(T& expected, T desired) noexcept {
        T result = expected;
        do {
            result = __LDREXW(&_value);
            if (result != expected) {
                expected = result;
                __STREXW(result, &_value); // 释放锁
                return false;
            }
        } while (__STREXW(desired, &_value) != 0U);
        return true;
    }

    // 原子加法并返回之前的值
    template <memory_order order = seq_cst>
    T fetch_add(T value) noexcept {
        T oldval;
        do {
            oldval = __LDREXW(&_value);
        } while (__STREXW(oldval + value, &_value) != 0U);
        return oldval;
    }

    // 原子减法并返回之前的值
    template <memory_order order = seq_cst>
    T fetch_sub(T value) noexcept {
        T oldval;
        do {
            oldval = __LDREXW(&_value);
        } while (__STREXW(oldval - value, &_value) != 0U);
        return oldval;
    }

    // 原子与操作并返回之前的值
    template <memory_order order = seq_cst>
    T fetch_and(T value) noexcept {
        T oldval;
        do {
            oldval = __LDREXW(&_value);
        } while (__STREXW(oldval & value, &_value) != 0U);
        return oldval;
    }

    // 原子或操作并返回之前的值
    template <memory_order order = seq_cst>
    T fetch_or(T value) noexcept {
        T oldval;
        do {
            oldval = __LDREXW(&_value);
        } while (__STREXW(oldval | value, &_value) != 0U);
        return oldval;
    }

    // 原子异或操作并返回之前的值
    template <memory_order order = seq_cst>
    T fetch_xor(T value) noexcept {
        T oldval;
        do {
            oldval = __LDREXW(&_value);
        } while (__STREXW(oldval ^ value, &_value) != 0U);
        return oldval;
    }

    // 标志操作
    void clear_flag() noexcept {
        store(0);
    }

    bool test_and_set_flag() noexcept {
        return exchange(1) != 0;
    }

    // 运算符重载
    T operator=(T value) noexcept { store(value); return value; }
    operator T() noexcept { return load(); }

    mAtomic& operator++() noexcept { fetch_add(1); return *this; }
    T operator++(int) noexcept { return fetch_add(1); }
    mAtomic& operator--() noexcept { fetch_sub(1); return *this; }
    T operator--(int) noexcept { return fetch_sub(1); }

    mAtomic& operator+=(T value) noexcept { fetch_add(value); return *this; }
    mAtomic& operator-=(T value) noexcept { fetch_sub(value); return *this; }
    mAtomic& operator&=(T value) noexcept { fetch_and(value); return *this; }
    mAtomic& operator|=(T value) noexcept { fetch_or(value); return *this; }
    mAtomic& operator^=(T value) noexcept { fetch_xor(value); return *this; }

private:
    // ARM独占访问指令封装
    __attribute__((always_inline)) 
    static inline T __LDREXW(volatile T* addr) {
        T result;
        __asm volatile ("ldrex %0, %1" : "=r" (result) : "Q" (*addr));
        return result;
    }

    __attribute__((always_inline)) 
    static inline T __STREXW(T value, volatile T* addr) {
        T result;
        __asm volatile ("strex %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" (value));
        return result;
    }

private:
    alignas(sizeof(T)) volatile T _value;
};