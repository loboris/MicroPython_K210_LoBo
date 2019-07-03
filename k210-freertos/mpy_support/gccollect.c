/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Damien P. George
 * Copyright (c) 2017 Pycom Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include "syslog.h"
#include <setjmp.h>

#include "py/mpconfig.h"
#include "py/mpstate.h"
#include "py/gc.h"
#include "py/mpthread.h"
#include "mphalport.h"
#include "gccollect.h"

/*
uintptr_t get_sp(void) {
    uintptr_t result;
    __asm__ ("la %0, _sp0\n" : "=r" (result) );
    return result;
}
*/

typedef jmp_buf regs_t;

static void gc_helper_get_regs(regs_t arr) {
    setjmp(arr);
}

// Explicitly mark this as noinline to make sure the regs variable
// is effectively at the top of the stack: otherwise, in builds where
// LTO is enabled and a lot of inlining takes place we risk a stack
// layout where regs is lower on the stack than pointers which have
// just been allocated but not yet marked, and get incorrectly sweeped.
MP_NOINLINE void gc_collect_regs_and_stack(void) {
    regs_t regs;
    gc_helper_get_regs(regs);
    // GC stack (and regs because we captured them)
    void **regs_ptr = (void**)(void*)&regs;
    gc_collect_root(regs_ptr, ((uintptr_t)MP_STATE_THREAD(stack_top) - (uintptr_t)&regs) / sizeof(uintptr_t));
}

/*
static void *inner_sp = NULL;
static void **inner_ptrs;

//-------------------------------------
static void gc_collect_inner(int level)
{
    // ToDo: is this really necessary ?
    // This function is called recursively 8 times
    // so that all K210 registers are pushed onto the stack
    // we than scan the stack for heap pointers
    if (level < sizeof(void*)) {
        gc_collect_inner(level + 1);
        if (level != 0) return;
    }

    if (level == sizeof(void*)) {
        // collect on stack
        inner_sp = (void *)pxTaskGetStackTop(NULL);
        inner_ptrs = (void**)(void*)inner_sp;
        gc_collect_root(inner_ptrs, ((void *)MP_STATE_THREAD(stack_top) - inner_sp) / sizeof(void*));
        return;
    }
}
*/

//-------------------
void gc_collect(void)
{
    // start the GC
    gc_collect_start();

    DEBUG_GC_printf("[GC_COLLECT] stacks\r\n");
    //gc_collect_inner(0);
    gc_collect_regs_and_stack();

    // Finally, trace root pointers from other threads
#if MICROPY_PY_THREAD
    mp_thread_gc_others();
#endif
    // end the GC
    gc_collect_end();
}

