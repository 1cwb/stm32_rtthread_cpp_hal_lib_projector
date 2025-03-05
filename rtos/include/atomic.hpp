// atomic.hpp
#pragma once
#include <cstdint>

// 内存顺序模型（简化版，支持常用选项）
enum memory_order {
    relaxed = 0,
    acquire,
    release,
    acq_rel,
    seq_cst
};

// 内存屏障
inline void atomic_thread_fence(memory_order order) {
    switch (order) {
        case acquire:
        case release:
        case acq_rel:
        case seq_cst:
            // DMB 指令：数据内存屏障（确保屏障前的内存访问先于屏障后的访问）
            asm volatile ("dmb ish" ::: "memory");
            break;
        case relaxed:
            break;
    }
}

// 原子类型模板基类
template <typename T>
class atomic_base {
protected:
    volatile T value;

    // 检查类型是否支持（仅限整型和指针）
    static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                  "Unsupported type for atomic operation");

    // 使用 LDREX/STREX 实现原子操作
    T load_impl() const {
        return __atomic_load_n(&value, __ATOMIC_RELAXED);
    }

    void store_impl(T desired) {
        __atomic_store_n(&value, desired, __ATOMIC_RELAXED);
    }

    bool compare_exchange_impl(T& expected, T desired) {
        return __atomic_compare_exchange_n(&value, &expected, desired, false,
                                          __ATOMIC_RELAXED, __ATOMIC_RELAXED);
    }
};

// 特化原子类（支持 32 位整型）
template <>
class atomic_base<uint32_t> {
protected:
    alignas(4) volatile uint32_t value; // 强制对齐到 4 字节

    uint32_t load_impl(memory_order order) const {
        uint32_t result;
        asm volatile (
            "ldrex %0, [%1] \n"
            : "=r" (result)
            : "r" (&value)
            : "memory"
        );
        atomic_thread_fence(order);
        return result;
    }

    void store_impl(uint32_t desired, memory_order order) {
        atomic_thread_fence(order);
        uint32_t status;
        do {
            asm volatile (
                "strex %0, %1, [%2] \n"
                : "=r" (status)
                : "r" (desired), "r" (&value)
                : "memory"
            );
        } while (status != 0);
    }

    bool compare_exchange_impl(uint32_t& expected, uint32_t desired, memory_order order) {
        atomic_thread_fence(order);
        uint32_t old_val = expected;
        uint32_t status;
        asm volatile (
            "ldrex %0, [%3]       \n"
            "cmp   %0, %1         \n"
            "bne   1f             \n"
            "strex %2, %4, [%3]   \n"
            "b     2f             \n"
            "1:                   \n"
            "mov   %2, #1         \n"
            "2:                   \n"
            : "=&r" (expected), "+r" (old_val), "=&r" (status)
            : "r" (&value), "r" (desired)
            : "memory", "cc"
        );
        if (order == memory_order::acq_rel || order == memory_order::seq_cst) {
            asm volatile ("dmb ish" ::: "memory");
        }
        return status == 0;
    }
};

// 新增：特化 uint8_t（支持 bool）
template <>
class atomic_base<uint8_t> {
protected:
    volatile uint8_t value;

    uint8_t load_impl(memory_order order) const {
        uint8_t result;
        asm volatile (
            "ldrexb %0, [%1] \n"   // 8 位独占加载
            : "=r" (result)
            : "r" (&value)
            : "memory"
        );
        atomic_thread_fence(order);
        return result;
    }

    void store_impl(uint8_t desired, memory_order order) {
        atomic_thread_fence(order);
        uint32_t status;
        do {
            asm volatile (
                "strexb %0, %1, [%2] \n"  // 8 位独占存储
                : "=r" (status)
                : "r" (desired), "r" (&value)
                : "memory"
            );
        } while (status != 0);
    }

    bool compare_exchange_impl(uint8_t& expected, uint8_t desired, memory_order order) {
        atomic_thread_fence(order); // 前屏障
        uint8_t old_val = expected;
        uint32_t status;
        asm volatile (
            "ldrexb %0, [%3]       \n"
            "cmp    %0, %1         \n"
            "bne    1f             \n"
            "strexb %2, %4, [%3]   \n"
            "b      2f             \n"
            "1:                     \n"
            "mov    %2, #1         \n"
            "2:                     \n"
            : "=&r" (expected), "+r" (old_val), "=&r" (status)
            : "r" (&value), "r" (desired)
            : "memory", "cc"
        );
        // 后屏障（acq_rel/seq_cst）
        if (order == memory_order::acq_rel || order == memory_order::seq_cst) {
            asm volatile ("dmb ish" ::: "memory");
        }
        return status == 0;
    }
};

// 原子类型接口
template <typename T>
class atomic : public atomic_base<T> {
public:
    atomic() = default;
    explicit atomic(T val) { this->value = val; }

    T load(memory_order order = seq_cst) const {
        return this->load_impl(order);
    }

    void store(T desired, memory_order order = seq_cst) {
        this->store_impl(desired, order);
    }

    bool compare_exchange_weak(T& expected, T desired, memory_order order = seq_cst) {
        return this->compare_exchange_impl(expected, desired, order);
    }
    // 扩展：原子算术操作
    T fetch_add(T arg, memory_order order = seq_cst) {
        T old_val = load(order);
        while (!compare_exchange_weak(old_val, old_val + arg, order)) {}
        return old_val;
    }

    T fetch_sub(T arg, memory_order order = seq_cst) {
        return fetch_add(-arg, order);
    }

    // 运算符重载
    // 前缀自增（++obj）
    T operator++() { 
        return fetch_add(1) + 1; 
    }

    // 后缀自增（obj++）
    T operator++(int) { 
        return fetch_add(1);
    }

    // 前缀自减（--obj）
    T operator--() { 
        return fetch_sub(1) - 1;
    }

    // 后缀自减（obj--）
    T operator--(int) { 
        return fetch_sub(1);
    }

    // 加法赋值（obj += arg）
    T operator+=(T arg) { 
        return fetch_add(arg) + arg;
    }

    // 减法赋值（obj -= arg）
    T operator-=(T arg) { 
        return fetch_sub(arg);
    }

    // 隐式类型转换（允许直接当 T 使用）
    operator T() const { 
        return load();
    }

    // 赋值运算符（obj = desired）
    atomic& operator=(T desired) { 
        store(desired);
        return *this;
    }
};

// 新增：特化 bool 类型（基于 uint8_t）
template <>
class atomic<bool> : public atomic_base<uint8_t> {
public:
    atomic() = default;
    explicit atomic(bool val) { this->value = val ? 1 : 0; }

    bool load(memory_order order = seq_cst) const {
        return (atomic_base<uint8_t>::load_impl(order) != 0);
    }

    void store(bool desired, memory_order order = seq_cst) {
        atomic_base<uint8_t>::store_impl(desired ? 1 : 0, order);
    }

    bool compare_exchange_weak(bool& expected, bool desired, memory_order order = seq_cst) {
        uint8_t expected_raw = expected ? 1 : 0;
        bool success = atomic_base<uint8_t>::compare_exchange_impl(expected_raw, desired ? 1 : 0, order);
        expected = (expected_raw != 0);
        return success;
    }

    // 禁用不适用运算符（如 +=、++）
    void operator++() = delete;
    void operator+=(bool) = delete;

    // 赋值运算符（obj = desired）
    atomic& operator=(bool desired) { 
        store(desired);
        return *this;
    }
};