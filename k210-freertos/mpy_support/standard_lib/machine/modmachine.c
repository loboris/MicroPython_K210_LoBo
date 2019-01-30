/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sysctl.h"
#include "syslog.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "py/mpprint.h"
#include <stdio.h>
#include "hal.h"
#include "modmachine.h"
#include "extmod/machine_mem.h"

#if MICROPY_PY_MACHINE

handle_t gpiohs_handle = 0;
uint32_t mp_used_gpiohs = 0;
machine_pin_def_t mp_used_pins[FPIOA_NUM_IO] = {0};

const char *gpiohs_funcs[10] = {
        "Not used",
        "Flash",
        "SD Card",
        "Display",
        "Pin",
        "UART",
        "I2C",
        "SPI",
        "PWM",
        "ISP_UART",
};

//------------------------
bool machine_init_gpiohs()
{
    bool res = true;
    if (gpiohs_handle == 0) {
        gpiohs_handle = io_open("/dev/gpio0");
        if (gpiohs_handle == 0) res = false;
    }
    return res;
}

//--------------------------------
void gpiohs_set_used(uint8_t gpio)
{
    mp_used_gpiohs |= (1 << gpio);
}

//--------------------------------
void gpiohs_set_free(uint8_t gpio)
{
    mp_used_gpiohs &= ~(1 << gpio);
}

//-----------------------------------
int gpiohs_get_free()
{
    int res = -1;
    uint32_t used_gpiohs = mp_used_gpiohs;
    for (int i=0; i<32; i++) {
        if ((used_gpiohs & 1) == 0) {
            mp_used_gpiohs |= (1 << i); // set pin used
            res = i;
            break;
        }
        used_gpiohs >>= 1;
    }
    return res;
}

//------------------------------------------------------------
void fpioa_setup_pins(int n, mp_fpioa_cfg_item_t functions[n])
{
    for (int i = 0; i < n; i++) {
        fpioa_set_function(functions[i].number, functions[i].function);
        LOGD("[PINS]", "Set pin %d to function %d", functions[i].number, functions[i].function);
    }
}

//----------------------------------------------------------------------
bool fpioa_check_pins(int n, mp_fpioa_cfg_item_t functions[n], int func)
{
    bool res = true;
    int pin;
    for (int i = 0; i < n; i++) {
        pin = functions[i].number;
        if ((mp_used_pins[pin].func != GPIO_FUNC_NONE) && (func != mp_used_pins[pin].func)) {
            res = false;
            LOGE("PIN CHECK", "Pin %d used by %s", pin, gpiohs_funcs[mp_used_pins[pin].func]);
            break;
        }
    }
    return res;
}

//------------------------------------------------------------------------
void fpioa_setused_pins(int n, mp_fpioa_cfg_item_t functions[n], int func)
{
    for (int i = 0; i < n; i++) {
        mp_used_pins[functions[i].number].func = func;
        mp_used_pins[functions[i].number].fpioa_func = functions[i].function;
        mp_used_pins[functions[i].number].gpio = functions[i].gpio;
    }
}

//---------------------------------------------------------------
void fpioa_freeused_pins(int n, mp_fpioa_cfg_item_t functions[n])
{
    for (int i = 0; i < n; i++) {
        mp_used_pins[functions[i].number].func = GPIO_FUNC_NONE;
        mp_used_pins[functions[i].number].fpioa_func = FUNC_DEBUG31;
        mp_used_pins[functions[i].number].gpio = -1;
    }
}

//-----------------------------------------------------------------
STATIC mp_obj_t machine_freq(size_t n_args, const mp_obj_t *args) {
    if (n_args == 0) {
        // get CPU frequency
        return mp_obj_new_int(uxPortGetCPUClock());
    }
    else {
        // set CPU frequency
        mp_int_t freq = mp_obj_get_int(args[0]);
        if (freq <= 800) freq *= 1000000;
        if ((freq < 200000000) || (freq > 800000000)) {
            mp_raise_ValueError("CPU frequency must be in 200 - 800 MHz range");
        }
        system_set_cpu_frequency(freq / 2);
        return mp_obj_new_int(uxPortGetCPUClock());
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_freq_obj, 0, 1, machine_freq);

// Assumes 0 <= max <= RAND_MAX
// Returns in the closed interval [0, max]
//-------------------------------------
uint64_t random_at_most(uint32_t max) {
    uint64_t    // max <= RAND_MAX < ULONG_MAX, so this is okay.
    num_bins = (uint64_t) max + 1,
    num_rand = (uint64_t) 0xFFFFFFFF + 1,
    bin_size = num_rand / num_bins,
    defect   = num_rand % num_bins;

    uint32_t x;
    do {
        x = rand();
    }
    while (num_rand - defect <= (uint64_t)x); // This is carefully written not to overflow

    // Truncated division is intentional
    return x/bin_size;
}

//-----------------------------------------------------------------
STATIC mp_obj_t machine_random(size_t n_args, const mp_obj_t *args)
{
    if (n_args == 1) {
        uint32_t rmax = mp_obj_get_int(args[0]);
        return mp_obj_new_int_from_uint(random_at_most(rmax));
    }
    uint32_t rmin = mp_obj_get_int(args[0]);
    uint32_t rmax = mp_obj_get_int(args[1]);
    return mp_obj_new_int_from_uint(rmin + random_at_most(rmax - rmin));
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_random_obj, 1, 2, machine_random);

//---------------------------------
STATIC mp_obj_t machine_reset(void)
{
    sysctl->soft_reset.soft_reset = 1; // This function does not return.

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_reset_obj, machine_reset);

//-----------------------------------
STATIC mp_obj_t machine_pinstat(void)
{
    int i;
    char sgpio[8] = {'\0'};
    printf(" Pin  GpioHS     Used by  Fpioa\n");
    printf("-------------------------------\n");
    for (i=0; i<FPIOA_NUM_IO; i++) {
        if (mp_used_pins[i].func != GPIO_FUNC_NONE) {
            if (mp_used_pins[i].gpio < 0) sprintf(sgpio, "%s", "-");
            else sprintf(sgpio, "%d", mp_used_pins[i].gpio);
            printf("%4d%8s%12s%7d\n", i, sgpio, gpiohs_funcs[mp_used_pins[i].func], mp_used_pins[i].fpioa_func);
        }
    }
    printf("-------------------------------\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_pinstat_obj, machine_pinstat);


//===========================================================
STATIC const mp_map_elem_t machine_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_machine) },

    { MP_ROM_QSTR(MP_QSTR_mem8),            MP_ROM_PTR(&machine_mem8_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem16),           MP_ROM_PTR(&machine_mem16_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem32),           MP_ROM_PTR(&machine_mem32_obj) },

    { MP_ROM_QSTR(MP_QSTR_freq),            MP_ROM_PTR(&machine_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_random),          MP_ROM_PTR(&machine_random_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),           MP_ROM_PTR(&machine_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_pinstat),         MP_ROM_PTR(&machine_pinstat_obj) },

    { MP_ROM_QSTR(MP_QSTR_Pin),             MP_ROM_PTR(&machine_pin_type) },
    //{ MP_ROM_QSTR(MP_QSTR_UART),            MP_ROM_PTR(&machine_uart_type) },
};

//===========================
STATIC MP_DEFINE_CONST_DICT (
    machine_module_globals,
    machine_module_globals_table
);

const mp_obj_module_t machine_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&machine_module_globals,
};

#endif // MICROPY_PY_MACHINE
