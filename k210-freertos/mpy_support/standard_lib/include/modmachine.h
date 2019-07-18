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

#define MAIN_TASK_PROC    0

#define mp_obj_is_meth(o) (mp_obj_is_obj(o) && (((mp_obj_base_t*)MP_OBJ_TO_PTR(o))->type->name == MP_QSTR_bound_method))

#define PLL0_MAX_OUTPUT_FREQ    988000000UL
#define PLL1_MAX_OUTPUT_FREQ    400000000UL
#define PLL2_MAX_OUTPUT_FREQ    45100000UL
#define CPU_MAX_FREQ            (PLL0_MAX_OUTPUT_FREQ / 2)
#define KPU_MAX_FREQ            PLL1_MAX_OUTPUT_FREQ
#define I2S_MAX_FREQ            0

#define SYS_RESET_REASON_NONE   0
#define SYS_RESET_REASON_NLR    1
#define SYS_RESET_REASON_WDT    2
#define SYS_RESET_REASON_SOFT   3
#define SYS_RESET_REASON_PWRON  4

typedef enum _gpio_func_t
{
    GPIO_FUNC_NONE,
    GPIO_FUNC_FLASH,
    GPIO_FUNC_SDCARD,
    GPIO_FUNC_DISP,
    GPIO_FUNC_PIN,
    GPIO_FUNC_UART,
    GPIO_FUNC_I2C,
    GPIO_FUNC_SPI0,
    GPIO_FUNC_SPI1,
    GPIO_FUNC_SPI_SLAVE,
    GPIO_FUNC_PWM,
    GPIO_FUNC_ISP_UART,
    GPIO_FUNC_GSM_UART,
    GPIO_FUNC_WIFI_UART,
} gpio_pin_func_t;

typedef enum _gpio_func_as_t
{
    GPIO_USEDAS_NONE,
    GPIO_USEDAS_TX,
    GPIO_USEDAS_RX,
    GPIO_USEDAS_MOSI,
    GPIO_USEDAS_MISO,
    GPIO_USEDAS_CLK,
    GPIO_USEDAS_CS,
    GPIO_USEDAS_SDA,
    GPIO_USEDAS_SCL,
    GPIO_USEDAS_INPUT,
    GPIO_USEDAS_OUTPUT,
    GPIO_USEDAS_IO,
    GPIO_USEDAS_WR,
    GPIO_USEDAS_RD,
    GPIO_USEDAS_DCX,
    GPIO_USEDAS_RST,
    GPIO_USEDAS_DATA0,
    GPIO_USEDAS_DATA1,
    GPIO_USEDAS_DATA2,
    GPIO_USEDAS_DATA3,
} gpio_pin_func_as_t;

typedef struct _mp_fpioa_cfg_item
{
    int8_t gpio;
    int number;
    gpio_pin_func_as_t usedas;
    fpioa_function_t function;
} __attribute__((aligned(8))) mp_fpioa_cfg_item_t;

typedef struct _machine_pin_def_t {
    int8_t gpio;
    gpio_pin_func_t func;
    gpio_pin_func_as_t usedas;
    fpioa_function_t fpioa_func;
} __attribute__((aligned(8))) machine_pin_def_t;

typedef struct _machine_pin_obj_t {
    mp_obj_base_t base;
    int8_t pin;
    int8_t gpio;
    uint8_t mode;
    uint8_t value;
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
} __attribute__((aligned(8))) machine_pin_obj_t;

typedef struct _mpy_flash_config_t {
    uint32_t    ver;
    bool        use_two_main_tasks;
    bool        pystack_enabled;
    uint32_t    heap_size1;
    uint32_t    heap_size2;
    uint32_t    pystack_size;
    uint32_t    main_task_stack_size;
    uint32_t    cpu_clock;
    uint32_t    repl_bdr;
    int32_t     boot_menu_pin;
    uint32_t    log_level;
    uint32_t    vm_divisor;
    bool        log_color;
} __attribute__((aligned(8))) mpy_flash_config_t;

typedef struct _mpy_config_t {
    mpy_flash_config_t config;
    uint32_t           crc;
} __attribute__((aligned(8))) mpy_config_t;

enum term_colors_t {
    BLACK = 0,
    RED,
    GREEN,
    BROWN,
    BLUE,
    PURPLE,
    CYAN,
    DEFAULT,
};

extern handle_t flash_spi;
extern handle_t gpiohs_handle;
extern uint32_t mp_used_gpiohs;
extern machine_pin_def_t mp_used_pins[FPIOA_NUM_IO];
extern const char *gpiohs_funcs[14];
extern const char *gpiohs_funcs_in_use[14];
extern const char *reset_reason[8];
extern const char *term_colors[8];
extern mpy_config_t mpy_config;

const char *term_color(enum term_colors_t color);
bool mpy_config_crc(bool set);
bool mpy_read_config();
void mpy_config_set_default();

mp_obj_t exec_code_from_str(const char *strdata);
void fpioa_setused_pins(int n, mp_fpioa_cfg_item_t functions[n], int func);
void fpioa_freeused_pins(int n, mp_fpioa_cfg_item_t functions[n]);
bool fpioa_check_pins(int n, mp_fpioa_cfg_item_t functions[n], int func);
void fpioa_setup_pins(int n, mp_fpioa_cfg_item_t functions[n]);
bool machine_init_gpiohs();
void gpiohs_set_used(uint8_t gpio);
void gpiohs_set_free(uint8_t gpio);
int gpiohs_get_free();

uint64_t random_at_most(uint32_t max);
time_t _get_time(bool systm);
void _set_sys_time(struct tm *tm_inf, int zone);

extern const mp_obj_type_t machine_pin_type;
extern const mp_obj_type_t machine_uart_type;
extern const mp_obj_type_t machine_hw_i2c_type;
extern const mp_obj_type_t machine_hw_spi_type;

#endif // MICROPY_MODMACHINE_H
