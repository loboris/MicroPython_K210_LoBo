/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <iomem.h>
//#include <printf.h>
#include <atomic.h>
#include <sys/lock.h>
#include "FreeRTOS.h"
#include "task.h"
#include "syslog.h"

// LoBo: some fixes in data types and unused variables

typedef struct _iomem_malloc_t
{
    void (*init)();
    uint32_t (*unused)(); // LoBo: set as uint32_t
    uint8_t *membase;
    uint32_t memsize;
    uint32_t memtblsize;
    uint16_t *memmap;
    uint8_t  memrdy;
    _lock_t *lock;
} iomem_malloc_t;

static _lock_t iomem_lock;

static void iomem_init();
static uint32_t k_unused();

extern char *_ioheap_line;
extern char *_heap_line;
// Lobo: commented
//extern char _heap_start[];
//extern char *_heap_cur;

iomem_malloc_t malloc_controll = {
    iomem_init,
    k_unused,
    NULL,
    0,
    0,
    NULL,
    0,
    &iomem_lock
};

static void iomem_set(void *s, uint8_t c, uint32_t num)
{
    uint8_t *xs = s;
    while(num--)
        *xs++=c;
}

static void iomem_init()
{
    malloc_controll.membase = (uint8_t *)((uintptr_t)_heap_line-0x40000000);
    malloc_controll.memsize = (uintptr_t)_ioheap_line - (uintptr_t)malloc_controll.membase;

    malloc_controll.memtblsize = malloc_controll.memsize / IOMEM_BLOCK_SIZE;
    malloc_controll.memmap = (uint16_t *)malloc(malloc_controll.memtblsize * 2);
    mb();

    malloc_controll.membase = (uint8_t *)((uintptr_t)_heap_line-0x40000000);
    malloc_controll.memsize = (uintptr_t)_ioheap_line - (uintptr_t)malloc_controll.membase;
    malloc_controll.memtblsize = malloc_controll.memsize / IOMEM_BLOCK_SIZE;

    iomem_set(malloc_controll.memmap, 0, malloc_controll.memtblsize * 2);
    iomem_set(malloc_controll.membase, 0, malloc_controll.memsize);
    malloc_controll.memrdy = 1;
}

static uint32_t k_unused()
{
    uint32_t unused=0;
    //uint32_t i;
    unused = (uintptr_t)_ioheap_line + 0x40000000 - (uintptr_t)_heap_line;

    return unused;
}

#if FIX_CACHE
static uint32_t k_malloc(uint32_t size)
{
    signed long offset = 0;
    uint32_t xmemb;
    uint32_t kmemb = 0;
    //uint32_t i;
    if(!malloc_controll.memrdy)
        malloc_controll.init();
    if(size==0)
        return 0XFFFFFFFF;
    xmemb=size / IOMEM_BLOCK_SIZE;
    if(size % IOMEM_BLOCK_SIZE)
        xmemb++;

    for(offset=malloc_controll.memtblsize-1; offset>=0; offset--)
    {
        if(!malloc_controll.memmap[offset])
        {
            kmemb++;
        }
        else 
        {
            offset = offset - malloc_controll.memmap[offset] + 1;
            kmemb=0;
        }
        if(kmemb==xmemb)
        {
            malloc_controll.memmap[offset] = xmemb;
            malloc_controll.memmap[offset+xmemb-1] = xmemb;
            return (offset * IOMEM_BLOCK_SIZE);
        }
    }
    return 0XFFFFFFFF;
}

static uint8_t k_free(uint32_t offset)
{
    //int i;
    if(!malloc_controll.memrdy)
    {
        malloc_controll.init();
        return 1;
    }  
    if(offset < malloc_controll.memsize)
    {  
        int index=offset / IOMEM_BLOCK_SIZE;
        int nmemb=malloc_controll.memmap[index];

        malloc_controll.memmap[index] = 0;
        malloc_controll.memmap[index+nmemb-1] = 0;

        if((uintptr_t)_ioheap_line == (uintptr_t)malloc_controll.membase + offset)
        {
            _ioheap_line = (char *)((uintptr_t)_ioheap_line + nmemb * IOMEM_BLOCK_SIZE);
        }
        return 0;
    }
    else 
        return 2;
}  

//static volatile uint32_t mstatus_t = 0;
#endif

void iomem_free(void *paddr)
{
    if(paddr == NULL)
        return;
#if FIX_CACHE
    uint32_t offset;
    _lock_acquire_recursive(malloc_controll.lock);
    offset=(uintptr_t)paddr - (uintptr_t)malloc_controll.membase;
    k_free(offset);
    _lock_release_recursive(malloc_controll.lock);
#else
    free(paddr);
#endif
}

void iomem_free_isr(void *paddr)
{
    if(paddr == NULL)
        return;
#if FIX_CACHE
    uint32_t offset;
    offset=(uintptr_t)paddr - (uintptr_t)malloc_controll.membase;
    k_free(offset);
#endif
}

void *iomem_malloc(uint32_t size)
{
#if FIX_CACHE
    _lock_acquire_recursive(malloc_controll.lock);

    uint32_t offset;
    offset=k_malloc(size);
    if(offset == 0XFFFFFFFF)
    {
        LOGW("[IOMEM]", "IOMEM malloc OUT of MEMORY!\r\n");
        _lock_release_recursive(malloc_controll.lock);
         return NULL;
    }
    else 
    {
        if((uintptr_t)_ioheap_line > (uintptr_t)malloc_controll.membase + offset)
        {
            _ioheap_line = (char *)((uintptr_t)malloc_controll.membase + offset);
            if((uintptr_t)_ioheap_line < (uintptr_t)_heap_line-0x40000000)
            {
                LOGW("[IOMEM]", "WARNING: iomem heap line (%p) < cache heap line (%p) !\r\n", _ioheap_line, _heap_line-0x40000000);
            }
        };
        _lock_release_recursive(malloc_controll.lock);
        return (void*)((uintptr_t)malloc_controll.membase + offset);
    }
#else
    return malloc(size);
#endif
}

uint32_t iomem_unused()
{
    return malloc_controll.unused();
}

uint32_t is_memory_cache(uintptr_t address)
{
    #define MEM_CACHE_LEN (6 * 1024 * 1024)

    return ((address >= 0x80000000) && (address < 0x80000000 + MEM_CACHE_LEN));
}

