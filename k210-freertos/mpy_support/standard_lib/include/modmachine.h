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
#include <fpioa.h>
#include <devices.h>

#include "py/obj.h"

#define LED12_GPIONUM     (5)
#define LED13_GPIONUM     (6)
#define LED14_GPIONUM     (7)

typedef enum _gpio_func
{
    GPIO_FUNC_NONE,
    GPIO_FUNC_FLASH,
    GPIO_FUNC_SDCARD,
    GPIO_FUNC_DISP,
    GPIO_FUNC_PIN,
    GPIO_FUNC_UART,
    GPIO_FUNC_I2C,
    GPIO_FUNC_SPI,
    GPIO_FUNC_PWM,
    GPIO_FUNC_ISP_UART
} gpio_pin_func_t;

typedef struct _mp_fpioa_cfg_item
{
    int8_t gpio;
    int number;
    fpioa_function_t function;
} mp_fpioa_cfg_item_t;

typedef struct _machine_pin_def_t {
    int8_t gpio;
    gpio_pin_func_t func;
    fpioa_function_t fpioa_func;
} machine_pin_def_t;

typedef struct _machine_pin_obj_t {
    mp_obj_base_t base;
    int8_t pin;
    int8_t gpio;
    uint8_t mode;
    uint8_t pull;
    uint8_t irq_type;
    uint8_t irq_level;
    int8_t irq_lastlevel;
    uint8_t irq_dbcproc;
    uint32_t irq_num;
    uint32_t irq_missed;
    uint32_t irq_scheduled;
    int32_t irq_debounce;
    int32_t irq_dbcpulses;
    uint64_t irq_time;
    mp_obj_t irq_handler;
    TaskHandle_t debounce_task;
} machine_pin_obj_t;

extern int MainTaskProc;
extern handle_t gpiohs_handle;
extern uint32_t mp_used_gpiohs;
extern machine_pin_def_t mp_used_pins[FPIOA_NUM_IO];
extern const char *gpiohs_funcs[10];

void fpioa_setused_pins(int n, mp_fpioa_cfg_item_t functions[n], int func);
void fpioa_freeused_pins(int n, mp_fpioa_cfg_item_t functions[n]);
bool fpioa_check_pins(int n, mp_fpioa_cfg_item_t functions[n], int func);
void fpioa_setup_pins(int n, mp_fpioa_cfg_item_t functions[n]);
bool machine_init_gpiohs();
void gpiohs_set_used(uint8_t gpio);
void gpiohs_set_free(uint8_t gpio);
int gpiohs_get_free();

uint64_t random_at_most(uint32_t max);

extern const mp_obj_type_t machine_pin_type;
//extern const mp_obj_type_t machine_uart_type;

#endif // MICROPY_MODMACHINE_H
