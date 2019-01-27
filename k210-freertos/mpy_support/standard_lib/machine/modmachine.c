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
#include <fpioa.h>
#include <devices.h>
#include "sysctl.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include <stdio.h>
#include "hal.h"
#include "modmachine.h"
#include "extmod/machine_mem.h"

#if MICROPY_PY_MACHINE

static handle_t gpiohs_handle = 0;

uint8_t mp_used_gpios[32] = {0};

// RGB LEDs pins configuretion
static const fpioa_cfg_t leds_pins_cfg =
{
    .version = PIN_CFG_VERSION,
    .functions_count = 3,
    // RGB LED
    .functions[0] = {12, FUNC_GPIOHS0 + LED12_GPIONUM},
    .functions[1] = {13, FUNC_GPIOHS0 + LED13_GPIONUM},
    .functions[2] = {14, FUNC_GPIOHS0 + LED14_GPIONUM},
};

//-----------------------------------------------
void fpioa_setup_pins(const fpioa_cfg_t *pin_cfg)
{
    configASSERT(pin_cfg->version == PIN_CFG_VERSION);

    uint32_t i;
    for (i = 0; i < pin_cfg->functions_count; i++)
    {
        fpioa_cfg_item_t item = pin_cfg->functions[i];
        fpioa_set_function(item.number, item.function);
    }
}

//----------------------------------
STATIC mp_obj_t machine_setup_leds()
{
    bool res = false;
    fpioa_setup_pins(&leds_pins_cfg);   // Configure leds pins
    if (gpiohs_handle == 0) {
        gpiohs_handle = io_open("/dev/gpio0");
        if (gpiohs_handle) {
            gpio_set_drive_mode(gpiohs_handle, LED12_GPIONUM, GPIO_DM_OUTPUT);
            gpio_set_pin_value(gpiohs_handle, LED12_GPIONUM, GPIO_PV_HIGH);
            gpio_set_drive_mode(gpiohs_handle, LED13_GPIONUM, GPIO_DM_OUTPUT);
            gpio_set_pin_value(gpiohs_handle, LED13_GPIONUM, GPIO_PV_HIGH);
            gpio_set_drive_mode(gpiohs_handle, LED14_GPIONUM, GPIO_DM_OUTPUT);
            gpio_set_pin_value(gpiohs_handle, LED14_GPIONUM, GPIO_PV_HIGH);
            mp_used_gpios[LED12_GPIONUM] = 1;
            mp_used_gpios[LED13_GPIONUM] = 1;
            mp_used_gpios[LED14_GPIONUM] = 1;
            res = true;
        }
    }
    return mp_obj_new_bool(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_setup_leds_obj, machine_setup_leds);

//---------------------------------------------------------------
STATIC mp_obj_t machine_set_led(mp_obj_t led_in, mp_obj_t val_in)
{
    if (gpiohs_handle) {
        int led = mp_obj_get_int(led_in);
        if ((led < LED12_GPIONUM) || (led > LED14_GPIONUM)) {
            mp_raise_ValueError("Unsupported LED pin");
        }
        int val = (mp_obj_is_true(val_in) ? GPIO_PV_LOW : GPIO_PV_HIGH);
        gpio_set_pin_value(gpiohs_handle, led, val);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_set_led_obj, machine_set_led);

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



//===========================================================
STATIC const mp_map_elem_t machine_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_machine) },

    { MP_ROM_QSTR(MP_QSTR_mem8),            MP_ROM_PTR(&machine_mem8_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem16),           MP_ROM_PTR(&machine_mem16_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem32),           MP_ROM_PTR(&machine_mem32_obj) },

    { MP_ROM_QSTR(MP_QSTR_freq),            MP_ROM_PTR(&machine_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_random),          MP_ROM_PTR(&machine_random_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),           MP_ROM_PTR(&machine_reset_obj) },

    { MP_ROM_QSTR(MP_QSTR_initleds),        MP_ROM_PTR(&machine_setup_leds_obj) },
    { MP_ROM_QSTR(MP_QSTR_setled),          MP_ROM_PTR(&machine_set_led_obj) },
    //{ MP_ROM_QSTR(MP_QSTR_UART),            MP_ROM_PTR(&machine_uart_type) },

    // class constants
    { MP_ROM_QSTR(MP_QSTR_LEDR),            MP_ROM_INT(LED14_GPIONUM) },
    { MP_ROM_QSTR(MP_QSTR_LEDG),            MP_ROM_INT(LED13_GPIONUM) },
    { MP_ROM_QSTR(MP_QSTR_LEDB),            MP_ROM_INT(LED12_GPIONUM) },
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
