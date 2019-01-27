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

#ifndef MICROPY_MODMACHINE_H
#define MICROPY_MODMACHINE_H

#include "FreeRTOS.h"
#include "task.h"
#include "pin_cfg.h"

#include "py/obj.h"

#define LED12_GPIONUM     (5)
#define LED13_GPIONUM     (6)
#define LED14_GPIONUM     (7)


typedef struct _machine_pin_obj_t {
    mp_obj_base_t base;
    uint8_t id;
    uint8_t mode;
    uint8_t pull;
    uint8_t irq_type;
    int8_t irq_level;
    uint8_t irq_expected;
    int8_t irq_repeat;
    int32_t irq_debounce;
    int32_t irq_active_time;
    uint32_t irq_num;
    uint32_t irq_missed;
    uint32_t irq_scheduled;
    mp_obj_t irq_handler;
    TaskHandle_t debounce_task;
} machine_pin_obj_t;


/*
// RGB LEDs pins configuretion
const fpioa_cfg_t leds_pin_cfg =
{
    .version = PIN_CFG_VERSION,
    .functions_count = 3,
    // RGB LED
    .functions[0] = {12, FUNC_GPIOHS6},
    .functions[1] = {13, FUNC_GPIOHS7},
    .functions[2] = {14, FUNC_GPIOHS8},
};

*/

extern uint8_t mp_used_gpios[32];

void fpioa_setup_pins(const fpioa_cfg_t *pin_cfg);
uint64_t random_at_most(uint32_t max);

//extern const mp_obj_type_t machine_uart_type;

#endif // MICROPY_MODMACHINE_H
