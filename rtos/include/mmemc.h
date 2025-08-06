#ifndef MMEM_C_H
#define MMEM_C_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void mmem_init(void* beginAddr, void* endAddr);
void* mmem_malloc(size_t size);
void* mmem_realloc(void* rmem, size_t newsize);
void* mmem_calloc(size_t count, size_t size);
void mmem_free(void* rmem);
size_t mmem_total(void);
size_t mmem_used(void);
size_t mmem_used_max(void);

#ifdef __cplusplus
}
#endif

#endif // MMEM_C_H