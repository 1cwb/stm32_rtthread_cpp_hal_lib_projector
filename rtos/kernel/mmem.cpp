#include "mmem.hpp"

/**
 * @ingroup SystemInit
 *
 * This function will initialize system heap memory.
 *
 * @param beginAddr the beginning address of system heap memory.
 * @param endAddr the end address of system heap memory.
 */
void mMem::init(void *beginAddr, void *endAddr)
{
    if(bInit_)
    {
        return;
    }
    struct heapMem_t *mem;
    unsigned long beginAlign = M_ALIGN((unsigned long)beginAddr, M_ALIGN_SIZE);
    unsigned long endAlign   = M_ALIGN_DOWN((unsigned long)endAddr, M_ALIGN_SIZE);

    //RT_DEBUG_NOT_IN_INTERRUPT;

    /* alignment addr */
    if ((endAlign > (2 * SIZEOF_STRUCT_MEM)) &&
        ((endAlign - 2 * SIZEOF_STRUCT_MEM) >= beginAlign))
    {
        /* calculate the aligned memory size */
        memSizeAligned_ = endAlign - beginAlign - 2 * SIZEOF_STRUCT_MEM;
    }
    else
    {
        rt_kprintf("mem init, error begin address 0x%lx, and end address 0x%lx\n",
                (unsigned long)beginAddr, (unsigned long)endAddr);

        return;
    }

    /* point to begin address of heap */
    heapPtr_ = (uint8_t *)beginAlign;

    //RT_DEBUG_LOG(RT_DEBUG_MEM, ("mem init, heap begin address 0x%x, size %d\n",
    //                            (unsigned long)heapPtr_, memSizeAligned_));

    /* initialize the start of the heap */
    mem        = (struct heapMem_t *)heapPtr_;
    mem->magic = HEAP_MAGIC;
    mem->next  = memSizeAligned_ + SIZEOF_STRUCT_MEM;
    mem->prev  = 0;
    mem->used  = 0;

    /* initialize the end of the heap */
    heapEnd_        = (struct heapMem_t *)&heapPtr_[mem->next];
    heapEnd_->magic = HEAP_MAGIC;
    heapEnd_->used  = 1;
    heapEnd_->next  = memSizeAligned_ + SIZEOF_STRUCT_MEM;
    heapEnd_->prev  = memSizeAligned_ + SIZEOF_STRUCT_MEM;

    memSem_.init("memsem",1,IPC_FLAG_FIFO);

    /* initialize the lowest-free pointer to the start of the heap */
    plfree_ = (struct heapMem_t *)heapPtr_;
    bInit_ = true;
}
/**
 * Allocate a block of memory with a minimum of 'size' bytes.
 *
 * @param size is the minimum size of the requested block in bytes.
 *
 * @return pointer to allocated memory or NULL if no free memory was found.
 */
void *mMem::malloc(unsigned long size)
{
    unsigned long ptr, ptr2;
    struct heapMem_t *mem, *mem2;

    if (size == 0 || !bInit_)
    {
        return nullptr;
    }

    //RT_DEBUG_NOT_IN_INTERRUPT;

    if (size != M_ALIGN(size, M_ALIGN_SIZE))
    {
        /*RT_DEBUG_LOG(RT_DEBUG_MEM, ("malloc size %d, but align to %d\n",
                                    size, M_ALIGN(size, M_ALIGN_SIZE)));*/
    }
    else
    {
        //RT_DEBUG_LOG(RT_DEBUG_MEM, ("malloc size %d\n", size));
    }

    /* alignment size */
    size = M_ALIGN(size, M_ALIGN_SIZE);

    if (size > memSizeAligned_)
    {
        //RT_DEBUG_LOG(RT_DEBUG_MEM, ("no memory\n"));

        return nullptr;
    }

    /* every data block must be at least MIN_SIZE_ALIGNED long */
    if (size < MIN_SIZE_ALIGNED)
    {
        size = MIN_SIZE_ALIGNED;
    }

    /* take memory semaphore */
    memSem_.semTake(WAITING_FOREVER);

    for (ptr = (uint8_t *)plfree_ - heapPtr_;
        ptr < memSizeAligned_ - size;
        ptr = ((struct heapMem_t *)&heapPtr_[ptr])->next)
    {
        mem = (struct heapMem_t *)&heapPtr_[ptr];

        if ((!mem->used) && (mem->next - (ptr + SIZEOF_STRUCT_MEM)) >= size)
        {
            /* mem is not used and at least perfect fit is possible:
            * mem->next - (ptr + SIZEOF_STRUCT_MEM) gives us the 'user data size' of mem */

            if (mem->next - (ptr + SIZEOF_STRUCT_MEM) >=
                (size + SIZEOF_STRUCT_MEM + MIN_SIZE_ALIGNED))
            {
                /* (in addition to the above, we test if another struct heapMem_t (SIZEOF_STRUCT_MEM) containing
                * at least MIN_SIZE_ALIGNED of data also fits in the 'user data space' of 'mem')
                * -> split large block, create empty remainder,
                * remainder must be large enough to contain MIN_SIZE_ALIGNED data: if
                * mem->next - (ptr + (2*SIZEOF_STRUCT_MEM)) == size,
                * struct heapMem_t would fit in but no data between mem2 and mem2->next
                * @todo we could leave out MIN_SIZE_ALIGNED. We would create an empty
                *       region that couldn't hold data, but when mem->next gets freed,
                *       the 2 regions would be combined, resulting in more free memory
                */
                ptr2 = ptr + SIZEOF_STRUCT_MEM + size;

                /* create mem2 struct */
                mem2       = (struct heapMem_t *)&heapPtr_[ptr2];
                mem2->magic = HEAP_MAGIC;
                mem2->used = 0;
                mem2->next = mem->next;
                mem2->prev = ptr;

                /* and insert it between mem and mem->next */
                mem->next = ptr2;
                mem->used = 1;

                if (mem2->next != memSizeAligned_ + SIZEOF_STRUCT_MEM)
                {
                    ((struct heapMem_t *)&heapPtr_[mem2->next])->prev = ptr2;
                }
#ifdef M_MEM_STATS
                usedMem_ += (size + SIZEOF_STRUCT_MEM);
                if (maxMem_ < usedMem_)
                    maxMem_ = usedMem_;
#endif
            }
            else
            {
                /* (a mem2 struct does no fit into the user data space of mem and mem->next will always
                * be used at this point: if not we have 2 unused structs in a row, plug_holes should have
                * take care of this).
                * -> near fit or excact fit: do not split, no mem2 creation
                * also can't move mem->next directly behind mem, since mem->next
                * will always be used at this point!
                */
                mem->used = 1;
#ifdef M_MEM_STATS
                usedMem_ += mem->next - ((uint8_t *)mem - heapPtr_);
                if (maxMem_ < usedMem_)
                    maxMem_ = usedMem_;
#endif
            }
            /* set memory block magic */
            mem->magic = HEAP_MAGIC;

            if (mem == plfree_)
            {
                /* Find next free block after mem and update lowest free pointer */
                while (plfree_->used && plfree_ != heapEnd_)
                {
                    plfree_ = (struct heapMem_t *)&heapPtr_[plfree_->next];
                }
                MASSERT(((plfree_ == heapEnd_) || (!plfree_->used)));
            }
            memSem_.semRelease();
            MASSERT((unsigned long)mem + SIZEOF_STRUCT_MEM + size <= (unsigned long)heapEnd_);
            MASSERT((unsigned long)((uint8_t *)mem + SIZEOF_STRUCT_MEM) % M_ALIGN_SIZE == 0);
            MASSERT((((unsigned long)mem) & (M_ALIGN_SIZE - 1)) == 0);

            /*RT_DEBUG_LOG(RT_DEBUG_MEM,
                        ("allocate memory at 0x%x, size: %d\n",
                        (unsigned long)((uint8_t *)mem + SIZEOF_STRUCT_MEM),
                        (unsigned long)(mem->next - ((uint8_t *)mem - heapPtr_))));*/

            /* return the memory data except mem struct */
            return (uint8_t *)mem + SIZEOF_STRUCT_MEM;
        }
    }

    memSem_.semRelease();
    return nullptr;
}
/**
 * This function will change the previously allocated memory block.
 *
 * @param rmem pointer to memory allocated by rt_malloc
 * @param newsize the required new size
 *
 * @return the changed memory block address
 */
void *mMem::realloc(void *rmem, unsigned long newsize)
{
    unsigned long size;
    unsigned long ptr, ptr2;
    struct heapMem_t *mem, *mem2;
    void *nmem;
    if(!bInit_)
    {
        return nullptr;
    }
    //RT_DEBUG_NOT_IN_INTERRUPT;

    /* alignment size */
    newsize = M_ALIGN(newsize, M_ALIGN_SIZE);
    if (newsize > memSizeAligned_)
    {
        //RT_DEBUG_LOG(RT_DEBUG_MEM, ("realloc: out of memory\n"));
        return nullptr;
    }
    else if (newsize == 0)
    {
        this->free(rmem);
        return nullptr;
    }

    /* allocate a new memory block */
    if (rmem == nullptr)
    {
        return this->malloc(newsize);
    }

    memSem_.semTake(WAITING_FOREVER);

    if ((uint8_t *)rmem < (uint8_t *)heapPtr_ ||
        (uint8_t *)rmem >= (uint8_t *)heapEnd_)
    {
        /* illegal memory */
        memSem_.semRelease();
        return rmem;
    }

    mem = (struct heapMem_t *)((uint8_t *)rmem - SIZEOF_STRUCT_MEM);

    ptr = (uint8_t *)mem - heapPtr_;
    size = mem->next - ptr - SIZEOF_STRUCT_MEM;
    if (size == newsize)
    {
        /* the size is the same as */
        memSem_.semRelease();
        return rmem;
    }

    if (newsize + SIZEOF_STRUCT_MEM + MIN_SIZE < size)
    {
        /* split memory block */
#ifdef M_MEM_STATS
        usedMem_ -= (size - newsize);
#endif

        ptr2 = ptr + SIZEOF_STRUCT_MEM + newsize;
        mem2 = (struct heapMem_t *)&heapPtr_[ptr2];
        mem2->magic = HEAP_MAGIC;
        mem2->used = 0;
        mem2->next = mem->next;
        mem2->prev = ptr;

        mem->next = ptr2;
        if (mem2->next != memSizeAligned_ + SIZEOF_STRUCT_MEM)
        {
            ((struct heapMem_t *)&heapPtr_[mem2->next])->prev = ptr2;
        }

        if (mem2 < plfree_)
        {
            /* the splited struct is now the lowest */
            plfree_ = mem2;
        }

        plugHoles(mem2);
        memSem_.semRelease();

        return rmem;
    }
    memSem_.semRelease();

    /* expand memory */
    nmem = this->malloc(newsize);
    if (nmem != nullptr) /* check memory */
    {
        memcpy(nmem, rmem, size < newsize ? size : newsize);
        this->free(rmem);
    }

    return nmem;
}
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
void *mMem::calloc(unsigned long count, unsigned long size)
{
    void *p;
    if(!bInit_)
    {
        return nullptr;
    }
    /* allocate 'count' objects of size 'size' */
    p = this->malloc(count * size);

    /* zero the memory */
    if (p)
    {
        memset(p, 0, count * size);
    }
    return p;
}
/**
 * This function will release the previously allocated memory block by
 * rt_malloc. The released memory block is taken back to system heap.
 *
 * @param rmem the address of memory which will be released
 */
void mMem::free(void *rmem)
{
    struct heapMem_t *mem;

    if (rmem == nullptr || !bInit_)
    {
        return;
    }

    //RT_DEBUG_NOT_IN_INTERRUPT;

    MASSERT((((unsigned long)rmem) & (M_ALIGN_SIZE - 1)) == 0);
    MASSERT((uint8_t *)rmem >= (uint8_t *)heapPtr_ &&
            (uint8_t *)rmem < (uint8_t *)heapEnd_);

    if ((uint8_t *)rmem < (uint8_t *)heapPtr_ ||
        (uint8_t *)rmem >= (uint8_t *)heapEnd_)
    {
        //RT_DEBUG_LOG(RT_DEBUG_MEM, ("illegal memory\n"));
        return;
    }

    /* Get the corresponding struct heapMem_t ... */
    mem = (struct heapMem_t *)((uint8_t *)rmem - SIZEOF_STRUCT_MEM);

    /*RT_DEBUG_LOG(RT_DEBUG_MEM,
                ("release memory 0x%x, size: %d\n",
                (unsigned long)rmem,
                (unsigned long)(mem->next - ((uint8_t *)mem - heapPtr_))));*/


    /* protect the heap from concurrent access */
    memSem_.semTake(WAITING_FOREVER);

    /* ... which has to be in a used state ... */
    if (!mem->used || mem->magic != HEAP_MAGIC)
    {
        rt_kprintf("to free a bad data block:\n");
        rt_kprintf("mem: 0x%08lx, used flag: %d, magic code: 0x%04x\n", (unsigned long)mem, mem->used, mem->magic);
    }
    MASSERT(mem->used);
    MASSERT(mem->magic == HEAP_MAGIC);
    /* ... and is now unused. */
    mem->used  = 0;
    mem->magic = HEAP_MAGIC;

    if (mem < plfree_)
    {
        /* the newly freed struct is now the lowest */
        plfree_ = mem;
    }

#ifdef M_MEM_STATS
    usedMem_ -= (mem->next - ((uint8_t *)mem - heapPtr_));
#endif

    /* finally, see if prev or next are free also */
    plugHoles(mem);
    memSem_.semRelease();
}

void mMem::plugHoles(struct heapMem_t *mem)
{
    struct heapMem_t *nmem;
    struct heapMem_t *pmem;

    MASSERT((uint8_t *)mem >= heapPtr_);
    MASSERT((uint8_t *)mem < (uint8_t *)heapEnd_);
    MASSERT(mem->used == 0);

    /* plug hole forward */
    nmem = (struct heapMem_t *)&heapPtr_[mem->next];
    if (mem != nmem &&
        nmem->used == 0 &&
        (uint8_t *)nmem != (uint8_t *)heapEnd_)
    {
        /* if mem->next is unused and not end of heapPtr_,
        * combine mem and mem->next
        */
        if (plfree_ == nmem)
        {
            plfree_ = mem;
        }
        mem->next = nmem->next;
        ((struct heapMem_t *)&heapPtr_[nmem->next])->prev = (uint8_t *)mem - heapPtr_;
    }

    /* plug hole backward */
    pmem = (struct heapMem_t *)&heapPtr_[mem->prev];
    if (pmem != mem && pmem->used == 0)
    {
        /* if mem->prev is unused, combine mem and mem->prev */
        if (plfree_ == mem)
        {
            plfree_ = pmem;
        }
        pmem->next = mem->next;
        ((struct heapMem_t *)&heapPtr_[mem->next])->prev = (uint8_t *)pmem - heapPtr_;
    }
}