#pragma once
#include "mmem.hpp"
#include <list>

void* operator new(size_t size);
void* operator new[](size_t size);
void operator delete(void* ptr) noexcept;
void operator delete[](void* ptr) noexcept;


template<typename T>
class mMemAllocator
{
public:
    using value_type = T;

    mMemAllocator() = default;

    template <typename U>
    mMemAllocator(const mMemAllocator<U>&) noexcept {}

    T* allocate(std::size_t n)
    {
        return static_cast<T*>(mMem::getInstance()->malloc(sizeof(T) * n));
    }

    void deallocate(T* p, std::size_t n)
    {
        mMem::getInstance()->free(p);
    }
};

