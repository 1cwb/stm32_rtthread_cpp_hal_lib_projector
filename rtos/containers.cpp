#include "containers.hpp"
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