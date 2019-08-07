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
#include "uarths.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "py/mpprint.h"
#include "py/compile.h"
#include "hal.h"
#include "modmachine.h"
#include "mphalport.h"
#include "extmod/machine_mem.h"
#include "w25qxx.h"
#include "platform_k210.h"

#if MICROPY_PY_MACHINE

#define INCLUDE_FLASH_TEST  (0)

#define CRC32_INIT (0)

mpy_config_t mpy_config;

handle_t gpiohs_handle = 0;
uint32_t mp_used_gpiohs = 0;
machine_pin_def_t mp_used_pins[FPIOA_NUM_IO] = {0};
handle_t flash_spi = 0;

uintptr_t sys_rambuf_ptr = 0;

const char *gpiohs_funcs[16] = {
        "Not used",
        "Flash",
        "SD Card",
        "Display",
        "Pin",
        "UART",
        "I2C",
        "SPI0",
        "SPI1",
        "SPI_SLAVE",
        "PWM",
        "ISP_UART",
        "GSM_UART",
        "WiFi_UART",
        "TIMER",
        "I/O"
};

const char *gpiohs_funcs_in_use[16] = {
        "pin not used",
        "pin used by Flash",
        "pin used by SD Card",
        "pin used by Display",
        "pin used by Pin",
        "pin used by UART",
        "pin used by I2C",
        "pin used by SPI0",
        "pin used by SPI1",
        "pin used by SPI_SLAVE",
        "pin used by PWM",
        "pin used by ISP_UART",
        "pin used by GSM_UART",
        "pin used by WiFi_UART",
        "pin used by Timer",
        "pin used by 1Wire"
};

const char *gpiohs_usedas[21] = {
        "--",
        "Tx",
        "Rx",
        "mosi",
        "miso",
        "clk",
        "cs",
        "sda",
        "scl",
        "input",
        "output",
        "io",
        "wr",
        "rd",
        "dcx",
        "rst",
        "data0",
        "data1",
        "data2",
        "data3",
        "1wire"
};

const char *reset_reason[8] = {
        "Unknown",
        "Soft reset",
        "WDT0 reset",
        "WDT1 (NLR) reset",
        "External pin reset",
        "Unknown",
        "Unknown",
        "Unknown",
};

static const char *log_levels[6] = {
        "LOG_NONE",       /*!< No log output */
        "LOG_ERROR",      /*!< Critical errors, software module can not recover on its own */
        "LOG_WARN",       /*!< Error conditions from which recovery measures have been taken */
        "LOG_INFO",       /*!< Information messages which describe normal flow of events */
        "LOG_DEBUG",      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
        "LOG_VERBOSE"     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
};

const char *term_colors[8] = {
        LOG_BOLD(LOG_COLOR_BLACK),
        LOG_BOLD(LOG_COLOR_RED),
        LOG_BOLD(LOG_COLOR_GREEN),
        LOG_BOLD(LOG_COLOR_BROWN),
        LOG_BOLD(LOG_COLOR_BLUE),
        LOG_BOLD(LOG_COLOR_PURPLE),
        LOG_BOLD(LOG_COLOR_CYAN),
        LOG_RESET_COLOR,
};

//----------------------------------------------
const char *term_color(enum term_colors_t color)
{
    if (mpy_config.config.log_color) return term_colors[color];
    else return "";
}

//---------------------------
bool mpy_config_crc(bool set)
{
    uint32_t ccrc = mp_hal_crc32((const uint8_t *)&mpy_config.config, sizeof(mpy_flash_config_t));
    if (set) {
        mpy_config.crc = ccrc;
        int res = w25qxx_write_data(MICRO_PY_FLASH_CONFIG_START, (uint8_t *)&mpy_config, sizeof(mpy_config_t));
        return (res == W25QXX_OK);
    }
    // check only
    return (mpy_config.crc == ccrc);
}

//--------------------
bool mpy_read_config()
{
    bool ret = false;
    mpy_config_t config;
    // read config from flash
    int res = w25qxx_read_data(MICRO_PY_FLASH_CONFIG_START, (uint8_t *)&config, sizeof(mpy_config_t));
    if (res == W25QXX_OK) {
        uint32_t ccrc = mp_hal_crc32((const uint8_t *)&config.config, sizeof(mpy_flash_config_t));
        if (config.crc == ccrc) {
            if (config.config.ver == MICROPY_PY_LOBO_VERSION_NUM) {
                // read config's crc ok, copy to current config
                memcpy((void *)&mpy_config, (void *)&config, sizeof(mpy_config_t));
                ret = mpy_config_crc(false);
            }
            else {
                LOGW("CONFIG", "New MicroPython version");
            }
        }
        else {
            LOGW("CONFIG", "Error reading configuration (crc)");
        }
    }
    else {
        LOGW("CONFIG", "Error reading configuration (flash read)");
    }
    return ret;
}

//---------------------------
void mpy_config_set_default()
{
    mpy_config.config.ver = MICROPY_PY_LOBO_VERSION_NUM;
    mpy_config.config.use_two_main_tasks = MICROPY_USE_TWO_MAIN_TASKS;
    mpy_config.config.pystack_enabled = MICROPY_ENABLE_PYSTACK;
    mpy_config.config.heap_size1 = MICROPY_HEAP_SIZE;
    mpy_config.config.heap_size2 = MICROPY_HEAP_SIZE2;
    mpy_config.config.pystack_size = MICROPY_PYSTACK_SIZE;
    mpy_config.config.main_task_stack_size = MICROPY_TASK_STACK_LEN;
    mpy_config.config.cpu_clock = MICRO_PY_DEFAULT_CPU_CLOCK;
    mpy_config.config.repl_bdr = MICRO_PY_DEFAULT_BAUDRATE;
    mpy_config.config.boot_menu_pin = MICRO_PY_BOOT_MENU_PIN;
    mpy_config.config.log_level = LOG_WARN;
    mpy_config.config.vm_divisor = MICROPY_PY_THREAD_GIL_VM_DIVISOR;
    mpy_config.config.log_color = MICROPY_PY_USE_LOG_COLORS;

    bool res = mpy_config_crc(true);
    LOGM("CONFIG", "Default flash configuration set (%d)", res);
}

//----------------------------------------------
mp_obj_t exec_code_from_str(const char *strdata)
{
    // ==== execute python code from string ====
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, strdata, strlen(strdata), 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, MP_PARSE_FILE_INPUT);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // uncaught exception
        return (mp_obj_t)nlr.ret_val;
    }
    return mp_const_true;
}
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
        mp_used_pins[functions[i].number].usedas = functions[i].usedas;
        mp_used_pins[functions[i].number].fpioa_func = functions[i].function;
        mp_used_pins[functions[i].number].gpio = functions[i].gpio;
    }
}

//---------------------------------------------------------------
void fpioa_freeused_pins(int n, mp_fpioa_cfg_item_t functions[n])
{
    for (int i = 0; i < n; i++) {
        mp_used_pins[functions[i].number].func = GPIO_FUNC_NONE;
        mp_used_pins[functions[i].number].usedas = GPIO_USEDAS_NONE;
        mp_used_pins[functions[i].number].fpioa_func = FUNC_DEBUG31;
        mp_used_pins[functions[i].number].gpio = -1;
    }
}

//-----------------------------------
STATIC mp_obj_t machine_pinstat(void)
{
    int i;
    char sgpio[8] = {'\0'};
    mp_printf(&mp_plat_print, " Pin  GpioHS     Used by      as  Fpioa\n");
    mp_printf(&mp_plat_print, "---------------------------------------\n");
    for (i=0; i<FPIOA_NUM_IO; i++) {
        if (mp_used_pins[i].func != GPIO_FUNC_NONE) {
            if (mp_used_pins[i].gpio < 0) sprintf(sgpio, "%s", "-");
            else sprintf(sgpio, "%d", mp_used_pins[i].gpio);
            mp_printf(&mp_plat_print, "%4d%8s%12s%8s%7d\n", i, sgpio, gpiohs_funcs[mp_used_pins[i].func],
                    gpiohs_usedas[mp_used_pins[i].usedas], mp_used_pins[i].fpioa_func);
        }
    }
    mp_printf(&mp_plat_print, "---------------------------------------\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_pinstat_obj, machine_pinstat);

//---------------------------------------------------------------
STATIC mp_obj_t machine_freq(size_t n_args, const mp_obj_t *args)
{
    uint32_t pll0 = 0;
    if (n_args > 0) {
        if (n_args > 1) {
            // Set PLL0
            pll0 = (uint32_t)mp_obj_get_int(args[1]);
            if ((pll0 != 800) && (pll0 != 1000)) {
                mp_raise_ValueError("Allowed PLL0 frequencies: 800 & 1000 MHz");
            }
            pll0 *= 1000000;
        }
        // set CPU frequency
        mp_int_t freq = mp_obj_get_int(args[0]);
        freq = (freq / 10) * 10;
        if ((freq != 100) && (freq != 200) && (freq != 400)) {
            mp_raise_ValueError("Allowed CPU frequencies: 100, 200, 400 MHz");
        }
        freq *= 1000000;
        if (pll0 > 0) sysctl_pll_set_freq(SYSCTL_PLL0, pll0);

        mp_hal_set_cpu_frequency(freq);
    }

    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));
    tuple[1] = mp_obj_new_int(sysctl_clock_get_freq(SYSCTL_CLOCK_PLL0));
    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_freq_obj, 0, 2, machine_freq);

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

//------------------------------------------------------
STATIC mp_obj_t mod_machine_log_level(mp_obj_t level_in)
{
    int32_t level = mp_obj_get_int(level_in);
    if ((level < LOG_NONE) || (level > LOG_VERBOSE)) {
        mp_raise_ValueError("Log level 0~5 expected");
    }

    user_log_level = level;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_log_level_obj, mod_machine_log_level);

//------------------------------------------------
STATIC mp_obj_t mod_machine_crc16(mp_obj_t buf_in)
{
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    uint16_t crc = mp_hal_crc16((uint8_t *)bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(crc);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_crc16_obj, mod_machine_crc16);

//------------------------------------------------
STATIC mp_obj_t mod_machine_crc32(mp_obj_t buf_in)
{
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    uint32_t crc = mp_hal_crc32((uint8_t *)bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(crc);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_crc32_obj, mod_machine_crc32);

//-------------------------------------------------
STATIC mp_obj_t mod_machine_base64(mp_obj_t buf_in)
{
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    int out_len = ((bufinfo.len + ((bufinfo.len % 3) ? (3 - (bufinfo.len % 3)) : 0)) * 4 / 3) + 2;
    unsigned char *out_str = pvPortMalloc(out_len);
    if (out_str == NULL) {
        mp_raise_msg(&mp_type_OSError, "Error allocating string buffer");
    }
    if (!base64_encode((unsigned char *)bufinfo.buf, bufinfo.len, out_str, &out_len)) {
        mp_raise_msg(&mp_type_OSError, "Base64 encode error");
    }

    mp_obj_t res = mp_obj_new_str((char *)out_str, out_len);
    vPortFree(out_str);
    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_base64_obj, mod_machine_base64);

//-----------------------------------------------------------------------
STATIC mp_obj_t mod_machine_baudrate(size_t n_args, const mp_obj_t *args)
{
    if (n_args > 0) {
        int bdr = mp_obj_get_int(args[0]);
        if ((bdr < 115200) || (bdr > 4000000)) {
            mp_raise_ValueError("Baudrate out of range");
        }
        uarths_baudrate = uarths_init(bdr);
    }

    return mp_obj_new_int(uarths_baudrate);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_machine_baudrate_obj, 0, 1, mod_machine_baudrate);

//---------------------------------
STATIC mp_obj_t machine_reset(void)
{
    sysctl->soft_reset.soft_reset = 1; // This function does not return.

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_reset_obj, machine_reset);

//--------------------------------------------------
STATIC mp_obj_t mod_machine_fsdebug(mp_obj_t dbg_in)
{
    w25qxx_debug = mp_obj_is_true(dbg_in);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_fsdebug_obj, mod_machine_fsdebug);

//----------------------------------------------------------
STATIC mp_obj_t mod_machine_flash_spi_check(mp_obj_t dbg_in)
{
    w25qxx_spi_check = mp_obj_is_true(dbg_in);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_flash_spi_check_obj, mod_machine_flash_spi_check);

//--------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_mpy_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_twotasks, ARG_pystacken, ARG_heap, ARG_pyssize, ARG_mainssize, ARG_freq, ARG_bdr, ARG_pin, ARG_logl, ARG_logcolor, ARG_vmd, ARG_print };
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_two_tasks_enable,  MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_pystack_enable,    MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_heap,              MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_pystack_size,      MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_main_stack_size,   MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_cpu_freq,          MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_repl_baudrate,     MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_bootmenu_pin,      MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_log_level,         MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_log_color,         MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_vm_divisor,        MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_print,             MP_ARG_KW_ONLY | MP_ARG_BOOL, { .u_bool = true } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool barg;
    int iarg;
    bool changed=false;
    mpy_config_t config;
    memcpy((void *)&config, (void *)&mpy_config, sizeof(mpy_config_t));

    if (args[ARG_twotasks].u_obj != mp_const_none) {
        barg = mp_obj_is_true(args[ARG_twotasks].u_obj);
        config.config.use_two_main_tasks = barg;
    }
    if (args[ARG_pystacken].u_obj != mp_const_none) {
        barg = mp_obj_is_true(args[ARG_pystacken].u_obj);
        config.config.pystack_enabled = barg;
    }
    if (args[ARG_heap].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_heap].u_obj);
        iarg = (iarg / 1024) * 1024;
    }
    else iarg = config.config.heap_size1;
    // Check heap(s)
    if (config.config.use_two_main_tasks) {
        if ((iarg < MICRO_PY_MIN_HEAP_SIZE) || (iarg > (MICRO_PY_MAX_HEAP_SIZE - MICRO_PY_MIN_HEAP_SIZE))) {
            LOGE("CONFIG", "Heap #1 size out of range (%dKB - %luKB)", MICRO_PY_MIN_HEAP_SIZE/1024, (MICRO_PY_MAX_HEAP_SIZE - MICRO_PY_MIN_HEAP_SIZE)/1024);
            mp_raise_ValueError("Heap #1 size out of range");
        }
        config.config.heap_size1 = iarg;
        config.config.heap_size2 = MICRO_PY_MAX_HEAP_SIZE - config.config.heap_size1;
    }
    else {
        config.config.heap_size1 = MICRO_PY_MAX_HEAP_SIZE;
        config.config.heap_size2 = 0;
    }

    if (args[ARG_pyssize].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_pyssize].u_obj);
        iarg = (iarg / 1024) * 1024;
        if ((iarg < MICRO_PY_MIN_PYSTACK_SIZE) || (iarg > MICRO_PY_MAX_PYSTACK_SIZE)) {
            mp_raise_ValueError("PyStack size out of range");
        }
        config.config.pystack_size = iarg;
    }
    if (args[ARG_mainssize].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_mainssize].u_obj);
        iarg = (iarg / 1024) * 1024;
        if ((iarg < 8192) || (iarg > 65536)) {
            mp_raise_ValueError("Main task stack size out of range");
        }
        iarg /= sizeof(StackType_t);
        config.config.main_task_stack_size = iarg;
    }
    if (args[ARG_freq].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_freq].u_obj);
        iarg = (iarg / 50) * 50;
        if ((iarg < 200) || (iarg > MICRO_PY_CPU_MAX_FREQ)) {
            mp_raise_ValueError("CPU frequency out of range");
        }

        iarg *= 1000000;
        config.config.cpu_clock = iarg;
    }
    if (args[ARG_bdr].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_bdr].u_obj);
        if ((iarg < 115200) || (iarg > 4000000)) {
            mp_raise_ValueError("Baudrate out of range");
        }
        config.config.repl_bdr = iarg;
    }
    if (args[ARG_pin].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_pin].u_obj);
        if ((iarg < 0) || (iarg >= FPIOA_NUM_IO)) {
            mp_raise_ValueError("Invalid boot menu pin");
        }
        config.config.boot_menu_pin = iarg;
        changed = true;
    }
    if (args[ARG_logl].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_logl].u_obj);
        if ((iarg < LOG_NONE) || (iarg > LOG_VERBOSE)) {
            mp_raise_ValueError("Log level 0~5 expected");
        }
        config.config.log_level = iarg;
    }
    if (args[ARG_vmd].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[ARG_vmd].u_obj);
        if ((iarg < 1) || (iarg > 128)) {
            mp_raise_ValueError("VM divisor out of range");
        }
        config.config.vm_divisor = iarg;
    }
    if (args[ARG_logcolor].u_obj != mp_const_none) {
        barg = mp_obj_is_true(args[ARG_logcolor].u_obj);
        config.config.log_color = barg;
    }

    // Check configuration values
    if (mpy_config.config.use_two_main_tasks != config.config.use_two_main_tasks) changed = true;
    if (mpy_config.config.heap_size1 != config.config.heap_size1) changed = true;
    if (mpy_config.config.heap_size2 != config.config.heap_size2) changed = true;
    if (mpy_config.config.pystack_enabled != config.config.pystack_enabled) changed = true;
    if (mpy_config.config.pystack_size != config.config.pystack_size) changed = true;
    if (mpy_config.config.main_task_stack_size != config.config.main_task_stack_size) changed = true;
    if (mpy_config.config.cpu_clock != config.config.cpu_clock) changed = true;
    if (mpy_config.config.repl_bdr != config.config.repl_bdr) changed = true;
    if (mpy_config.config.boot_menu_pin != config.config.boot_menu_pin) changed = true;
    if (mpy_config.config.log_level != config.config.log_level) changed = true;
    if (mpy_config.config.log_color != config.config.log_color) changed = true;
    if (mpy_config.config.vm_divisor != config.config.vm_divisor) changed = true;

    if (args[ARG_print].u_bool) {
        mp_printf(&mp_plat_print, "\r\n%sMicroPython configuration:\r\n--------------------------%s\r\n", term_color(CYAN), term_color(DEFAULT));
        mp_printf(&mp_plat_print, "   MPy version code: %06X\r\n", config.config.ver);
        mp_printf(&mp_plat_print, "  Two MPy instances: %s\r\n", (config.config.use_two_main_tasks) ? "True" : "False");
        mp_printf(&mp_plat_print, "       PyStack used: %s\r\n", (config.config.pystack_enabled) ? "True" : "False");
        mp_printf(&mp_plat_print, "Available heap size: %u KB\r\n", MICRO_PY_MAX_HEAP_SIZE / 1024);
        if (config.config.use_two_main_tasks) {
            mp_printf(&mp_plat_print, "    MPy#1 heap size: %u KB\r\n", config.config.heap_size1 / 1024);
            mp_printf(&mp_plat_print, "    MPy#2 heap size: %u KB\r\n", config.config.heap_size2 / 1024);
        }
        else {
            mp_printf(&mp_plat_print, "      MPy heap size: %u KB\r\n", config.config.heap_size1 / 1024);
        }
        mp_printf(&mp_plat_print, "       PyStack size: %u B\r\n", config.config.pystack_size);
        mp_printf(&mp_plat_print, "     MPy stack size: %u B\r\n", config.config.main_task_stack_size * sizeof(StackType_t));
        mp_printf(&mp_plat_print, "      CPU frequency: %u MHz\r\n", config.config.cpu_clock / 1000000);
        mp_printf(&mp_plat_print, "      REPL baudrate: %u bd\r\n", config.config.repl_bdr);
        mp_printf(&mp_plat_print, "      Boot menu pin: %d\r\n", config.config.boot_menu_pin);
        mp_printf(&mp_plat_print, "  Default log level: %u (%s)\r\n", config.config.log_level, log_levels[config.config.log_level]);
        mp_printf(&mp_plat_print, "     Use log colors: %u (%s)\r\n", config.config.log_color, (config.config.log_color) ? "True" : "False");
        mp_printf(&mp_plat_print, "         VM divisor: %u bytecodes\r\n", config.config.vm_divisor);
        if (changed) {
            mp_printf(&mp_plat_print, "\r\nPress %sY%s to save", term_color(BROWN), term_color(DEFAULT));
            char key = '\0';
            key = wait_key("", 10000);
            if ((key == 'y') || (key == 'Y')) {
                memcpy((void *)&mpy_config, (void *)&config, sizeof(mpy_config_t));
                bool res = mpy_config_crc(true);
                if (!res) {
                    mp_printf(&mp_plat_print, "\r\n");
                    mp_raise_msg(&mp_type_OSError, "Error saving configuration to Flash");
                }
                mp_printf(&mp_plat_print, "%s\rNew configuration saved\r\n%s", term_color(CYAN), term_color(DEFAULT));
            }
            else {
                mp_printf(&mp_plat_print, "%s\rNew configuration not saved\r\n%s", term_color(CYAN), term_color(DEFAULT));
            }
        }
    }
    mp_obj_t cfg_tuple[13];
    cfg_tuple[0] = (mpy_config.config.use_two_main_tasks) ? mp_const_true : mp_const_false;
    cfg_tuple[1] = (mpy_config.config.pystack_enabled) ? mp_const_true : mp_const_false;
    cfg_tuple[2] = mp_obj_new_int(MICRO_PY_MAX_HEAP_SIZE);
    cfg_tuple[3] = mp_obj_new_int(mpy_config.config.heap_size1);
    cfg_tuple[4] = mp_obj_new_int(mpy_config.config.heap_size2);
    cfg_tuple[5] = mp_obj_new_int(mpy_config.config.pystack_size);
    cfg_tuple[6] = mp_obj_new_int(mpy_config.config.main_task_stack_size * sizeof(StackType_t));
    cfg_tuple[7] = mp_obj_new_int(mpy_config.config.cpu_clock);
    cfg_tuple[8] = mp_obj_new_int(mpy_config.config.repl_bdr);
    cfg_tuple[9] = mp_obj_new_int(mpy_config.config.boot_menu_pin);
    cfg_tuple[10] = mp_obj_new_int(mpy_config.config.log_level);
    cfg_tuple[11] = (mpy_config.config.log_color) ? mp_const_true : mp_const_false;
    cfg_tuple[12] = mp_obj_new_int(mpy_config.config.vm_divisor);

    return mp_obj_new_tuple(13, cfg_tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_mpy_config_obj, 0, machine_mpy_config);

//-----------------------------------------------
STATIC mp_obj_t mod_machine_wdt (mp_obj_t tmo_in)
{
    int tmo = mp_obj_get_int(tmo_in);
    if (tmo <= 0) mp_hal_wtd_enable(false, 6);
    else if (tmo > 120) {
        mp_raise_ValueError("Timeout range is 0 ~ 120 seconds");
    }
    else mp_hal_wtd_enable(true, tmo);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_wdt_obj, mod_machine_wdt);

//-------------------------------------
STATIC mp_obj_t mod_machine_wdt_reset()
{
    mp_hal_wdt_reset();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_machine_wdt_reset_obj, mod_machine_wdt_reset);

//--------------------------------------------------
STATIC mp_obj_t mod_machine_vm_hook (mp_obj_t state)
{
    bool f = mp_obj_is_true(state);
    use_vm_hook = f;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_vm_hook_obj, mod_machine_vm_hook);

//---------------------------------------------------------------
STATIC mp_obj_t mod_machine_wdt_reset_in_vm_hook (mp_obj_t state)
{
    bool f = mp_obj_is_true(state);
    wdt_reset_in_vm_hook = f;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_wdt_reset_in_vm_hook_obj, mod_machine_wdt_reset_in_vm_hook);

//--------------------------------------
STATIC mp_obj_t mod_machine_get_rambuf()
{
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(sys_rambuf_ptr - K210_SRAM_START_ADDRESS);
    tuple[1] = mp_obj_new_int(MYCROPY_SYS_RAMBUF_SIZE);
    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_machine_get_rambuf_obj, mod_machine_get_rambuf);

//----------------------------------------
STATIC mp_obj_t mod_machine_reset_reason()
{
    uint8_t rst_stat = system_status & 0x0F;
    uint8_t rst = 0;
    for (uint8_t i=0; i<4; i++) {
        if ((rst_stat >> i) & 1) {
            rst = i+1;
            break;
        }
    }
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(rst_stat);
    tuple[1] = mp_obj_new_str(reset_reason[rst], strlen(reset_reason[rst]));
    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_machine_reset_reason_obj, mod_machine_reset_reason);

//------------------------------------------------------
STATIC mp_obj_t mod_machine_flash_read(mp_obj_t addr_in)
{
    uint32_t addr = mp_obj_get_int(addr_in);
    uint8_t buf[512] = {0};
    enum w25qxx_status_t res;

    res = w25qxx_read_data(addr, buf, 512);
    if (res == W25QXX_OK) {
        for (int k=0; k<512; k++) {
            if ((k & 0x1f) == 0) mp_printf(&mp_plat_print, "\n[%08X] ", addr+k);
            mp_printf(&mp_plat_print, "%02X ", buf[k]);
        }
        mp_printf(&mp_plat_print, "\n");
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_flash_read_obj, mod_machine_flash_read);

//----------------------------------------------------
STATIC mp_obj_t mod_machine_membytes(mp_obj_t size_in)
{
    uint16_t size = mp_obj_get_int(size_in);
    if ((size < 9) || (size > MYCROPY_SYS_RAMBUF_SIZE)) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Out pf membytes size range (9 ~ %d)", MYCROPY_SYS_RAMBUF_SIZE));
    }
    machine_mem_obj_t *self = m_new_obj(machine_mem_obj_t);
    self->base.type = &machine_mem_type;
    self->elem_size = size;

    return self;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_membytes_obj, mod_machine_membytes);


//===========================================================
STATIC const mp_map_elem_t machine_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_machine) },

    { MP_ROM_QSTR(MP_QSTR_mem8),            MP_ROM_PTR(&machine_mem8_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem16),           MP_ROM_PTR(&machine_mem16_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem32),           MP_ROM_PTR(&machine_mem32_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem64),           MP_ROM_PTR(&machine_mem64_obj) },
    { MP_ROM_QSTR(MP_QSTR_memstr),          MP_ROM_PTR(&machine_memstr_obj) },
    { MP_ROM_QSTR(MP_QSTR_membytes),        MP_ROM_PTR(&mod_machine_membytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_rambuf),          MP_ROM_PTR(&mod_machine_get_rambuf_obj) },

    { MP_ROM_QSTR(MP_QSTR_freq),            MP_ROM_PTR(&machine_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_random),          MP_ROM_PTR(&machine_random_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),           MP_ROM_PTR(&machine_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_reason),    MP_ROM_PTR(&mod_machine_reset_reason_obj) },
    { MP_ROM_QSTR(MP_QSTR_pinstat),         MP_ROM_PTR(&machine_pinstat_obj) },
    { MP_ROM_QSTR(MP_QSTR_loglevel),        MP_ROM_PTR(&mod_machine_log_level_obj) },
    { MP_ROM_QSTR(MP_QSTR_crc16),           MP_ROM_PTR(&mod_machine_crc16_obj) },
    { MP_ROM_QSTR(MP_QSTR_crc32),           MP_ROM_PTR(&mod_machine_crc32_obj) },
    { MP_ROM_QSTR(MP_QSTR_base64enc),       MP_ROM_PTR(&mod_machine_base64_obj) },
    { MP_ROM_QSTR(MP_QSTR_repl_baudrate),   MP_ROM_PTR(&mod_machine_baudrate_obj) },
    { MP_ROM_QSTR(MP_QSTR_wdt),             MP_ROM_PTR(&mod_machine_wdt_obj) },
    { MP_ROM_QSTR(MP_QSTR_wdt_reset),       MP_ROM_PTR(&mod_machine_wdt_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_fsdebug),         MP_ROM_PTR(&mod_machine_fsdebug_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_spicheck),     MP_ROM_PTR(&mod_machine_flash_spi_check_obj) },
    { MP_ROM_QSTR(MP_QSTR_mpy_config),      MP_ROM_PTR(&machine_mpy_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_vm_hook),         MP_ROM_PTR(&mod_machine_vm_hook_obj) },
    { MP_ROM_QSTR(MP_QSTR_vm_hook_wdt),     MP_ROM_PTR(&mod_machine_wdt_reset_in_vm_hook_obj) },
    { MP_ROM_QSTR(MP_QSTR_flash_read),      MP_ROM_PTR(&mod_machine_flash_read_obj) },

    { MP_ROM_QSTR(MP_QSTR_Pin),             MP_ROM_PTR(&machine_pin_type) },
    { MP_ROM_QSTR(MP_QSTR_UART),            MP_ROM_PTR(&machine_uart_type) },
    { MP_ROM_QSTR(MP_QSTR_I2C),             MP_ROM_PTR(&machine_hw_i2c_type) },
    { MP_ROM_QSTR(MP_QSTR_SPI),             MP_ROM_PTR(&machine_hw_spi_type) },
    { MP_ROM_QSTR(MP_QSTR_Timer),           MP_ROM_PTR(&machine_timer_type) },
    { MP_ROM_QSTR(MP_QSTR_PWM),             MP_ROM_PTR(&machine_pwm_type) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_Onewire),     MP_ROM_PTR(&machine_onewire_type) },

    { MP_ROM_QSTR(MP_QSTR_RAM_START),       MP_ROM_INT(K210_SRAM_START_ADDRESS) },
    { MP_ROM_QSTR(MP_QSTR_LOG_NONE),        MP_ROM_INT(LOG_NONE) },
    { MP_ROM_QSTR(MP_QSTR_LOG_ERROR),       MP_ROM_INT(LOG_ERROR) },
    { MP_ROM_QSTR(MP_QSTR_LOG_WARN),        MP_ROM_INT(LOG_WARN) },
    { MP_ROM_QSTR(MP_QSTR_LOG_INFO),        MP_ROM_INT(LOG_INFO) },
    { MP_ROM_QSTR(MP_QSTR_LOG_DEBUG),       MP_ROM_INT(LOG_DEBUG) },
    { MP_ROM_QSTR(MP_QSTR_LOG_VERBOSE),     MP_ROM_INT(LOG_VERBOSE) },
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
