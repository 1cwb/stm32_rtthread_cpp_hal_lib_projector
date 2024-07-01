#pragma once
#include "mmem.h"
#include <list>

void* operator new(size_t size) 
{
    return mMem::getInstance()->malloc(size);
}

void* operator new[](size_t size) 
{
    return mMem::getInstance()->malloc(size);
}

void operator delete(void* ptr) noexcept 
{
    mMem::getInstance()->free(ptr);
}

void operator delete[](void* ptr) noexcept 
{
    mMem::getInstance()->free(ptr);
}
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

