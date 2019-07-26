/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
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

#include <string.h>
#include "py/runtime.h"
#include "extmod/machine_mem.h"

#if MICROPY_PY_MACHINE

// If you wish to override the functions for mapping the machine_mem read/write
// address, then add a #define for MICROPY_MACHINE_MEM_GET_READ_ADDR and/or
// MICROPY_MACHINE_MEM_GET_WRITE_ADDR in your mpconfigport.h. Since the
// prototypes are identical, it is allowable for both of the macros to evaluate
// the to same function.
//
// It is expected that the modmachine.c file for a given port will provide the
// implementations, if the default implementation isn't used.

#if !defined(MICROPY_MACHINE_MEM_GET_READ_ADDR) || !defined(MICROPY_MACHINE_MEM_GET_WRITE_ADDR)
STATIC uintptr_t machine_mem_get_addr(mp_obj_t addr_o, uint align) {
    uintptr_t addr = mp_obj_int_get_truncated(addr_o);
    if ((align > 0) && ((addr & (align - 1)) != 0)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "address %08x is not aligned to %d bytes", addr, align));
    }
    return addr;
}
#if !defined(MICROPY_MACHINE_MEM_GET_READ_ADDR)
#define MICROPY_MACHINE_MEM_GET_READ_ADDR machine_mem_get_addr
#endif
#if !defined(MICROPY_MACHINE_MEM_GET_WRITE_ADDR)
#define MICROPY_MACHINE_MEM_GET_WRITE_ADDR machine_mem_get_addr
#endif
#endif

STATIC void machine_mem_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    (void)kind;
    machine_mem_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if ((self->elem_size > 0) && (self->elem_size <= 8)) mp_printf(print, "<%u-bit memory>", 8 * self->elem_size);
    else if (self->elem_size > 8) mp_printf(print, "<%u bytes memory>", self->elem_size);
    else mp_printf(print, "<string memory>");
}

STATIC mp_obj_t machine_mem_subscr(mp_obj_t self_in, mp_obj_t index, mp_obj_t value) {
    // TODO support slice index to read/write multiple values at once
    machine_mem_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (value == MP_OBJ_NULL) {
        // delete
        return MP_OBJ_NULL; // op not supported
    } else if (value == MP_OBJ_SENTINEL) {
        // load
        size_t val_len = 0;
        uint64_t val = 0;
        uint8_t strval[MYCROPY_SYS_RAMBUF_SIZE+1] = {0};
        uintptr_t addr = MICROPY_MACHINE_MEM_GET_READ_ADDR(index, self->elem_size);
        uintptr_t ram_addr = addr + K210_SRAM_START_ADDRESS;
        int max_len = K210_SRAM_SIZE - addr;

        if ((self->elem_size > 0) && (self->elem_size <= 8)) val_len = self->elem_size * 8;
        else if (self->elem_size > 8) {
            // membytes
            memcpy(strval, (uint8_t*)ram_addr, (MYCROPY_SYS_RAMBUF_SIZE > max_len) ? max_len : MYCROPY_SYS_RAMBUF_SIZE);
            val_len = (self->elem_size <= max_len) ? self->elem_size : max_len;
        }
        else {
            // memstr
            memcpy(strval, (uint8_t*)ram_addr, (MYCROPY_SYS_RAMBUF_SIZE > max_len) ? max_len : MYCROPY_SYS_RAMBUF_SIZE);
            val_len = strlen((const char *)strval);
        }

        // ==== Allow reading from all of RAM area ====
        if (addr > (K210_SRAM_SIZE - val_len)) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "reading %lu bytes from address %08x outside RAM area", val_len, addr));
        }
        switch (self->elem_size) {
            case 0: return mp_obj_new_str((const char *)strval, val_len); break;
            case 1: val = (*(uint8_t*)ram_addr); break;
            case 2: val = (*(uint16_t*)ram_addr); break;
            case 4: val = (*(uint32_t*)ram_addr); break;
            case 8: val = (*(uint64_t*)ram_addr); break;
            default: return mp_obj_new_str((const char *)strval, val_len);
        }
        return mp_obj_new_int(val);
    } else {
        // store
        uintptr_t addr = MICROPY_MACHINE_MEM_GET_WRITE_ADDR(index, self->elem_size);
        addr += K210_SRAM_START_ADDRESS;
        uint64_t val = 0;
        size_t val_len = 0;
        const char *strval = NULL;
        if ((self->elem_size > 0) && (self->elem_size <= 8)) {
            val = mp_obj_get_int_truncated(value);
            val_len = self->elem_size* 8;
        }
        else if (self->elem_size > 8) {
            strval = mp_obj_str_get_data(value, &val_len);
        }
        else {
            strval = mp_obj_str_get_str(value);
            val_len = strlen(strval) + 1;
        }
        // ==== Only allow writing to system RAM buffer ! ====
        if ((sys_rambuf_ptr == 0) || (addr < sys_rambuf_ptr) || (addr > (int64_t)(sys_rambuf_ptr + MYCROPY_SYS_RAMBUF_SIZE - val_len))) {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "writing %lu bytes to address %08x outside of RAM buffer", val_len, addr));
        }
        switch (self->elem_size) {
            case 0: strcpy((char *)addr, strval); break;
            case 1: (*(uint8_t*)addr) = val; break;
            case 2: (*(uint16_t*)addr) = val; break;
            case 4: (*(uint32_t*)addr) = val; break;
            case 8: (*(uint64_t*)addr) = val; break;
            default: memcpy((uint8_t *)addr, strval, (val_len <= self->elem_size) ? val_len : self->elem_size); break;
        }
        return mp_const_none;
    }
}

const mp_obj_type_t machine_mem_type = {
    { &mp_type_type },
    .name = MP_QSTR_mem,
    .print = machine_mem_print,
    .subscr = machine_mem_subscr,
};

const machine_mem_obj_t machine_mem8_obj = {{&machine_mem_type}, 1};
const machine_mem_obj_t machine_mem16_obj = {{&machine_mem_type}, 2};
const machine_mem_obj_t machine_mem32_obj = {{&machine_mem_type}, 4};
const machine_mem_obj_t machine_mem64_obj = {{&machine_mem_type}, 8};
const machine_mem_obj_t machine_memstr_obj = {{&machine_mem_type}, 0};

#endif // MICROPY_PY_MACHINE
