#include "mmem.hpp"

// 前向声明C接口
extern "C" {
    void mmem_init(void* beginAddr, void* endAddr);
    void* mmem_malloc(size_t size);
    void* mmem_realloc(void* rmem, size_t newsize);
    void* mmem_calloc(size_t count, size_t size);
    void mmem_free(void* rmem);
    size_t mmem_total(void);
    size_t mmem_used(void);
    size_t mmem_used_max(void);
}

// 全局实例
static mMem* get_instance() {
    static mMem* instance = nullptr;
    if (!instance) {
        instance = mMem::getInstance();
    }
    return instance;
}

void mmem_init(void* beginAddr, void* endAddr) {
    mMem* instance = get_instance();
    if (instance) {
        instance->init(beginAddr, endAddr);
    }
}

void* mmem_malloc(size_t size) {
    mMem* instance = get_instance();
    return instance ? instance->malloc(size) : nullptr;
}

void* mmem_realloc(void* rmem, size_t newsize) {
    mMem* instance = get_instance();
    return instance ? instance->realloc(rmem, newsize) : nullptr;
}

void* mmem_calloc(size_t count, size_t size) {
    mMem* instance = get_instance();
    return instance ? instance->calloc(count, size) : nullptr;
}

void mmem_free(void* rmem) {
    mMem* instance = get_instance();
    if (instance) {
        instance->free(rmem);
    }
}

size_t mmem_total() {
    mMem* instance = get_instance();
    return instance ? instance->total() : 0;
}

size_t mmem_used() {
    mMem* instance = get_instance();
    return instance ? instance->used() : 0;
}

size_t mmem_used_max() {
    mMem* instance = get_instance();
    return instance ? instance->usedMaxMem() : 0;
}