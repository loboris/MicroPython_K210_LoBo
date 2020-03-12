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

#define PLL0_MAX_OUTPUT_FREQ        988000000UL
#define PLL1_MAX_OUTPUT_FREQ        800000000UL
#define PLL2_DEFAULT_OUTPUT_FREQ    45158400UL
#define CPU_MAX_FREQ                (PLL0_MAX_OUTPUT_FREQ / 2)
#define KPU_MAX_FREQ                (PLL1_MAX_OUTPUT_FREQ / 2)
#define I2S_MAX_FREQ                0
#define CYCLES_PER_US               (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000)

#define SYS_RESET_REASON_NONE       0
#define SYS_RESET_REASON_NLR        1
#define SYS_RESET_REASON_WDT        2
#define SYS_RESET_REASON_SOFT       3
#define SYS_RESET_REASON_PWRON      4

#define TIMER_MAX_TIMERS            12

#define SPI_MASTER_0            0
#define SPI_MASTER_1            1
#define SPI_MASTER_WS2812_0     2
#define SPI_MASTER_WS2812_1     3
#define SPI_SLAVE               4
#define SPI_INTERFACE_MAX       5


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
    GPIO_FUNC_TIMER,
    GPIO_FUNC_1WIRE,
    GPIO_FUNC_DVP
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
    GPIO_USEDAS_1WIRE,
    GPIO_USEDAS_DVP_RST,
    GPIO_USEDAS_DVP_PWDN,
    GPIO_USEDAS_DVP_XCLK,
    GPIO_USEDAS_DVP_VSYNC,
    GPIO_USEDAS_DVP_HREF,
    GPIO_USEDAS_DVP_PCLK,
    GPIO_USEDAS_DVP_SCLK,
    GPIO_USEDAS_DVP_SDA,
    GPIO_USEDAS_HANDSHAKE
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


typedef struct _machine_timer_obj_t {
    mp_obj_base_t   base;
    uint8_t         id;         // timer number (0~11)
    uint8_t         state;      // current timer state
    uint8_t         type;       // timer type
    int8_t          pin;        // timer pin, if not used: -1
    int8_t          pin_gpio;
    int8_t          pin_mode;
    int8_t          pin_pull;
    uint8_t         reserved;
    handle_t        handle;     // hw timer handle
    bool            repeat;     // true for periodic type
    uint64_t        period;     // timer period in us
    int64_t         remain;     // remaining us until timer event
    uint32_t        interval;   // hw timer interval in nanoseconds
    double          resolution; // hw timer resolution in nanoseconds
    uint64_t        event_num;  // number of timer events
    uint64_t        cb_num;     // number of scheduled timer callbacks
    mp_obj_t        callback;   // timer callback function
} __attribute__((aligned(8))) machine_timer_obj_t;


typedef struct _machine_pwm_obj_t {
    mp_obj_base_t   base;
    handle_t        handle;     // hw pwm handle
    uint8_t         channel;
    int8_t          pin;
    bool            active;
    bool            invert;
    uint32_t        perc;
    uint32_t        periods;
    double          freq;
    double          dperc;
} __attribute__((aligned(8))) machine_pwm_obj_t;


typedef union _ws2812b_rgb {
    struct
    {
        uint32_t blue  : 8;
        uint32_t red   : 8;
        uint32_t green : 8;
        uint32_t white : 8;
    };
    uint32_t rgb;
} ws2812b_rgb;

typedef struct _ws2812b_buffer_t
{
    size_t      num_pix;
    ws2812b_rgb *rgb_buffer;
} ws2812b_buffer_t;

typedef struct _machine_hw_spi_obj_t {
    mp_obj_base_t       base;
    int8_t              spi_num;            // SPI device used (0-1 spi master, 2-3 ws2812, 4 spi slave)
    int8_t              sck;
    int8_t              mosi;
    int8_t              miso;
    int8_t              cs;
    int8_t              handshake;
    int8_t              handshake_gpio;
    int8_t              mode;
    int8_t              firstbit;
    int8_t              nbits;
    bool                duplex;
    bool                reused_spi;
    uint32_t            baudrate;
    uint32_t            freq;
    handle_t            handle;             // handle to SPI driver
    handle_t            spi_device;
    uint8_t             *slave_buffer;
    uint32_t            buffer_size;
    uint32_t            slave_ro_size;
    bool                slave_buffer_allocated;
    mp_obj_t            slave_cb;           // slave callback function
    TaskHandle_t        slave_task;
    QueueHandle_t       slave_queue;
    ws2812b_buffer_t    ws2812_buffer;
    uint16_t            ws2812_lo;
    uint16_t            ws2812_hi;
    uint32_t            ws2812_rst;
    uint16_t            ws_lo;
    uint16_t            ws_hi;
    uint32_t            ws_rst;
    uint32_t            ws_needed_buf_size;
    bool                ws2812_white;
    enum {
        MACHINE_HW_SPI_STATE_NONE,
        MACHINE_HW_SPI_STATE_INIT,
        MACHINE_HW_SPI_STATE_DEINIT
    } state;
} machine_hw_spi_obj_t;


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
extern const char *gpiohs_funcs[17];
extern const char *gpiohs_funcs_in_use[17];
extern const char *reset_reason[8];
extern const char *term_colors[8];
extern mpy_config_t mpy_config;
extern void *mpy_timers_used[TIMER_MAX_TIMERS];
extern QueueHandle_t spi_slave_task_queue;

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

int spi_master_hard_init(uint8_t mosi, int8_t miso, uint8_t sck, int8_t cs, gpio_pin_func_t func);

extern const mp_obj_type_t machine_pin_type;
extern const mp_obj_type_t machine_uart_type;
extern const mp_obj_type_t machine_hw_i2c_type;
extern const mp_obj_type_t machine_hw_spi_type;
extern const mp_obj_type_t machine_timer_type;
extern const mp_obj_type_t machine_pwm_type;
extern const mp_obj_type_t machine_onewire_type;
extern const mp_obj_type_t machine_ds18x20_type;

#endif // MICROPY_MODMACHINE_H
