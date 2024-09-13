#pragma once
#include "rtoscommon.hpp"
#include "mipc.hpp"

struct heapMem_t
{
    /* magic and used flag */
    uint16_t magic;
    uint16_t used;
#ifdef ARCH_CPU_64BIT
    uint32_t resv;
#endif
    unsigned long next, prev;
};

#define M_MEM_STATS

#define HEAP_MAGIC 0x1ea0

#ifdef ARCH_CPU_64BIT
#define MIN_SIZE 24
#else
#define MIN_SIZE 12
#endif

#define MIN_SIZE_ALIGNED     M_ALIGN(MIN_SIZE, M_ALIGN_SIZE)
#define SIZEOF_STRUCT_MEM    M_ALIGN(sizeof(struct heapMem_t), M_ALIGN_SIZE)

class mMem
{
public:
    static mMem* getInstance()
    {
        static mMem mem;
        return &mem;
    }
    /**
     * @ingroup SystemInit
     *
     * This function will initialize system heap memory.
     *
     * @param beginAddr the beginning address of system heap memory.
     * @param endAddr the end address of system heap memory.
     */
    void init(void *beginAddr, void *endAddr);
    /**
     * Allocate a block of memory with a minimum of 'size' bytes.
     *
     * @param size is the minimum size of the requested block in bytes.
     *
     * @return pointer to allocated memory or NULL if no free memory was found.
     */
    void *malloc(unsigned long size);
    /**
     * This function will change the previously allocated memory block.
     *
     * @param rmem pointer to memory allocated by rt_malloc
     * @param newsize the required new size
     *
     * @return the changed memory block address
     */
    void *realloc(void *rmem, unsigned long newsize);
    /**
     * This function will contiguously allocate enough space for count objects
     * that are size bytes of memory each and returns a pointer to the allocated
     * memory.
     *
     * The allocated memory is filled with bytes of value zero.
     *
     * @param count number of objects to allocate
     * @param size size of the objects to allocate
     *
     * @return pointer to allocated memory / NULL pointer if there is an error
     */
    void *calloc(unsigned long count, unsigned long size);
    /**
     * This function will release the previously allocated memory block by
     * rt_malloc. The released memory block is taken back to system heap.
     *
     * @param rmem the address of memory which will be released
     */
    void free(void *rmem);

    unsigned long total() const
    {
        return memSizeAligned_;
    }
    unsigned long used() const
    {
        #ifdef M_MEM_STATS
        return usedMem_;
        #else
        return 0;
        #endif
    }
    unsigned long usedMaxMem() const
    {
        #ifdef M_MEM_STATS
        return maxMem_;
        #else
        return 0;
        #endif
    }
private:
    void plugHoles(struct heapMem_t *mem);

    mMem() : bInit_(false)
    {
        #ifdef M_MEM_STATS
        usedMem_ = 0;
        maxMem_ = 0;
        #endif
    }
    ~mMem()
    {

    }
    mMem(const mMem&) = delete;
    mMem(mMem&&) = delete;
    mMem& operator=(const mMem&) = delete;
    mMem& operator=(mMem&&) = delete;

    /** the last entry, always unused! */
    heapMem_t *heapEnd_;
    uint8_t *heapPtr_;

    heapMem_t *plfree_;   /* pointer to the lowest free block */

    unsigned long memSizeAligned_;

#ifdef M_MEM_STATS
    unsigned long usedMem_, maxMem_;
#endif
    bool bInit_;
    mSemaphore memSem_;
};
