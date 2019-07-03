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

const char *gpiohs_funcs[14] = {
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
};

const char *gpiohs_funcs_in_use[14] = {
        "pin not used",
        "pin used by Flash",
        "pin used by SD Card",
        "pin used by Display",
        "pin used by pin",
        "pin used by UART",
        "pin used by I2C",
        "pin used by SPI0",
        "pin used by SPI1",
        "pin used by SPI_SLAVE",
        "pin used by PWM",
        "pin used by ISP_UART",
        "pin used by GSM_UART",
        "pin used by WiFi_UART",
};

const char *gpiohs_usedas[20] = {
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
    mp_obj_t tuple[2];
    if (n_args == 0) {
        // get CPU frequency and Flash SPI speed
        tuple[0] = mp_obj_new_int(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));
        tuple[1] = mp_obj_new_int(w25qxx_actual_speed);
    }
    else {
        // set CPU frequency
        mp_int_t freq = mp_obj_get_int(args[0]);
        freq = (freq / 50) * 50;
        if ((freq < 200) || (freq > MICRO_PY_CPU_MAX_FREQ)) {
            #if MICRO_PY_ALLOW_OVERCLOCK
            mp_raise_ValueError("Allowed CPU frequencies: 200 - 750 MHz");
            #else
            mp_raise_ValueError("Allowed CPU frequencies: 200 - 500 MHz");
            #endif
        }

        freq *= 1000000;

        mp_hal_set_cpu_frequency(freq);

        tuple[0] = mp_obj_new_int(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));
        tuple[1] = mp_obj_new_int(w25qxx_actual_speed);
    }
    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_freq_obj, 0, 1, machine_freq);

//-------------------------------------------------------------------
STATIC mp_obj_t machine_setflash(size_t n_args, const mp_obj_t *args)
{
    mp_obj_t tuple[2];
    if (n_args < 2) {
        // get CPU frequency
        tuple[0] = mp_obj_new_int(work_trans_mode);
        tuple[1] = mp_obj_new_int(w25qxx_flash_speed);
    }
    else {
        // set Flash frequency
        mp_int_t mode = mp_obj_get_int(args[0]);
        if ((mode < 0) || (mode > 2)) {
            mp_raise_ValueError("Allowed modes: 0, 1, 2");
        }
        mp_int_t flash_speed = mp_obj_get_int(args[1]);
        if (flash_speed <= 30) flash_speed *= 1000000;
        if ((flash_speed < 1000000) || (flash_speed > 30000000)) {
            mp_raise_ValueError("Flash speed must be in 1 - 30 MHz range");
        }

        flash_speed = w25qxx_init(flash_spi, mode, flash_speed);

        tuple[0] = mp_obj_new_int(work_trans_mode);
        tuple[1] = mp_obj_new_int(flash_speed);
    }
    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_setflash_obj, 0, 2, machine_setflash);

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

//---------------------------------------------------
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

#if INCLUDE_FLASH_TEST
//---------------------------------------------------------------------------------------
STATIC mp_obj_t machine_test_flash(mp_obj_t mode_in, mp_obj_t speed_in, mp_obj_t type_in)
{
    uint32_t oldmode = work_trans_mode;
    uint32_t oldspeed =w25qxx_flash_speed;
    int mode = mp_obj_get_int(mode_in);
    if ((mode < 0) || (mode > 2)) mode = 2;
    int speed = mp_obj_get_int(speed_in);
    int type = mp_obj_get_int(type_in);
    if ((speed < 1000000) || (speed > 100000000)) speed = 10000000;
    uint32_t real_speed = w25qxx_init(flash_spi, mode, speed);
    uint32_t spi_speed = sysctl_clock_get_freq(SYSCTL_CLOCK_SPI3);

    mp_printf(&mp_plat_print, "\nFLASH CPU: %u, PLL0=%u, test: mode=%d, speed=%d (%u, spi_clk=%u)\n",
            sysctl_clock_get_freq(SYSCTL_CLOCK_CPU), sysctl_clock_get_freq(SYSCTL_CLOCK_PLL0), mode, speed, real_speed, spi_speed);
    mp_printf(&mp_plat_print, "-----------\n\n");
    if (type == 0) {
        w25qxx_init(flash_spi, oldmode, oldspeed);
        return mp_const_none;
    }

    uint32_t rd;
    uint32_t wr;
    uint32_t er;
    uint32_t sector;
    uint32_t start_addr = MICRO_PY_FLASH_USED_END;
    uint32_t end_addr = MICRO_PY_FLASH_SIZE - start_addr;
    // use 1 MB maximum for testing
    if ((end_addr-start_addr) > 0x100000) end_addr = start_addr + 0x100000;

    int count = 0;
    enum w25qxx_status_t res;
    uint64_t start_time, end_time, op_time;
    uint8_t buf[4096];
    uint8_t rdbuf[4096];

    w25qxx_clear_counters();
    if (type > 1) {
        mp_printf(&mp_plat_print, "ERASE test\n");
        count = 0;
        start_time = mp_hal_ticks_us();
        for (sector = start_addr; sector < end_addr; sector+=4096) {
            memset(rdbuf, 0, 4096);
            w25qxx_sector_erase(sector);
            res = w25qxx_wait_busy();
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "Erase error at %x\n", sector);
                goto exit;
            }
            count++;
            mp_hal_wdt_reset();
        }
        end_time = mp_hal_ticks_us();
        mp_printf(&mp_plat_print, "Erase time: %lu, %lu/sector\n", end_time-start_time, (end_time-start_time) / count);
        w25qxx_get_counters(&rd, &wr, &er, NULL);
        mp_printf(&mp_plat_print, "Flash counters: reads: %u, writes: %u, erases: %u\n", rd, wr, er);

        w25qxx_clear_counters();
        start_time = mp_hal_ticks_us();
        for (sector = start_addr; sector < end_addr; sector+=4096) {
            res = w25qxx_read_data(sector, rdbuf, 4096);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "Erase test error (compare) at sector %x\n", sector);
                goto exit;
            }
            for (int i=0; i<4096; i++) {
                if (rdbuf[i] != 0xFF) {
                    mp_printf(&mp_plat_print, "Erase test error at sector %x, pos %d (%2x)\n", sector, i, rdbuf[i]);
                    goto exit;
                }
            }
            count++;
        }
        end_time = mp_hal_ticks_us();
        mp_printf(&mp_plat_print, "\nRead time: %lu, %lu/sector\n", end_time-start_time, (end_time-start_time) / count);
        w25qxx_get_counters(&rd, &wr, &er, NULL);
        mp_printf(&mp_plat_print, "Flash counters: reads: %u, writes: %u, erases: %u\n", rd, wr, er);
    }

    mp_hal_wdt_reset();
    mp_printf(&mp_plat_print, "FLASH test\n");
    for (int n=0; n<4096; n++) {
        if (n & 1) buf[n] = 0x55;
        else buf[n] = 0xaa;
    }
    w25qxx_clear_counters();
    count = 0;
    start_time = mp_hal_ticks_us();
    for (sector = start_addr; sector < end_addr; sector+=4096) {
        // write sector
        res = w25qxx_write_data(sector, buf, 4096);
        if (res != W25QXX_OK) {
            mp_printf(&mp_plat_print, "Write error at %x\n", sector);
            goto exit;
        }
        count++;
    }
    end_time = mp_hal_ticks_us();
    mp_printf(&mp_plat_print, "Read test, buffers written in %lu, %lu/sector\n", end_time-start_time, (end_time-start_time) / count);

    bool old_spicheck = w25qxx_spi_check;
    w25qxx_spi_check = true;
    count = 0;
    op_time = 0;
    for (int n=0; n<20; n++) {
        for (sector = start_addr; sector < end_addr; sector+=4096) {
            start_time = mp_hal_ticks_us();
            res = w25qxx_read_data(sector, rdbuf, 4096);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "Read test error at sector %x, pass=%d\n", sector, n);
                w25qxx_spi_check = old_spicheck;
                goto exit;
            }
            op_time += (mp_hal_ticks_us() - start_time);
            // read sector and compare
            res = w25qxx_read_data(sector, rdbuf, 4096);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "Read test error (compare) at sector %x, pass=%d\n", sector, n);
                w25qxx_spi_check = old_spicheck;
                goto exit;
            }
            for (int i=0; i<4096; i++) {
                if (buf[i] != rdbuf[i]) {
                    mp_printf(&mp_plat_print, "Compare error at sector %x, pos %d (%2x <> %2x)\n", sector, i, rdbuf[i], buf[i]);
                    w25qxx_spi_check = old_spicheck;
                    goto exit;
                }
            }
            count++;
        }
        mp_hal_wdt_reset();
    }
    end_time = mp_hal_ticks_us();
    w25qxx_spi_check = old_spicheck;
    mp_printf(&mp_plat_print, "\nRead time: %lu, %lu/sector\n", op_time, op_time / count);
    w25qxx_get_counters(&rd, &wr, &er, NULL);
    mp_printf(&mp_plat_print, "Flash counters: reads: %u, writes: %u, erases: %u\n", rd, wr, er);

exit:
    w25qxx_init(flash_spi, oldmode, oldspeed);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(machine_test_flash_obj, machine_test_flash);
#endif


//--------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_mpy_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_two_tasks_enable,  MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_pystack_enable,    MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_heap1,             MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_heap2,             MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_pystack_size,      MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_main_stack_size,   MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_cpu_freq,          MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_repl_baudrate,     MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_bootmenu_pin,      MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_log_level,         MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_vm_divisor,        MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_print,             MP_ARG_KW_ONLY | MP_ARG_BOOL, { .u_bool = true } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool barg;
    int iarg;
    bool changed = false;
    mpy_config_t config;
    memcpy((void *)&config, (void *)&mpy_config, sizeof(mpy_config_t));

    if (args[0].u_obj != mp_const_none) {
        barg = mp_obj_is_true(args[0].u_obj);
        config.config.use_two_main_tasks = barg;
    }
    if (args[1].u_obj != mp_const_none) {
        barg = mp_obj_is_true(args[1].u_obj);
        config.config.pystack_enabled = barg;
    }
    if (args[2].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[2].u_obj);
        iarg = (iarg / 1024) * 1024;
        if ((iarg < 0x80000) || (iarg > (MICRO_PY_MAX_HEAP_SIZE - ((config.config.use_two_main_tasks) ? 0x80000 : 0)))) {
            mp_raise_ValueError("Heap #1 size out of range");
        }
        config.config.heap_size1 = iarg;
    }
    if (args[3].u_obj != mp_const_none) {
        if (config.config.use_two_main_tasks) {
            iarg = mp_obj_get_int(args[3].u_obj);
            iarg = (iarg / 1024) * 1024;
            if ((iarg < 0x80000) || (iarg > (MICRO_PY_MAX_HEAP_SIZE - config.config.heap_size1))) {
                mp_raise_ValueError("Heap #2 size out of range");
            }
        }
        else iarg = 0;
        config.config.heap_size2 = iarg;
    }
    if (args[4].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[4].u_obj);
        iarg = (iarg / 1024) * 1024;
        if ((iarg < 2048) || (iarg > 65536)) {
            mp_raise_ValueError("PyStack size out of range");
        }
        config.config.pystack_size = iarg;
    }
    if (args[5].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[5].u_obj);
        iarg = (iarg / 1024) * 1024;
        if ((iarg < 8192) || (iarg > 65536)) {
            mp_raise_ValueError("Main task stack size out of range");
        }
        iarg /= sizeof(StackType_t);
        config.config.main_task_stack_size = iarg;
    }
    if (args[6].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[6].u_obj);
        iarg = (iarg / 50) * 50;
        if ((iarg < 200) || (iarg > MICRO_PY_CPU_MAX_FREQ)) {
            mp_raise_ValueError("CPU frequency out of range");
        }

        iarg *= 1000000;
        config.config.cpu_clock = iarg;
    }
    if (args[7].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[7].u_obj);
        if ((iarg < 115200) || (iarg > 4000000)) {
            mp_raise_ValueError("Baudrate out of range");
        }
        config.config.repl_bdr = iarg;
    }
    if (args[8].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[8].u_obj);
        if ((iarg < 0) || (iarg >= FPIOA_NUM_IO)) {
            mp_raise_ValueError("Invalid boot menu pin");
        }
        config.config.boot_menu_pin = iarg;
        changed = true;
    }
    if (args[9].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[9].u_obj);
        if ((iarg < LOG_NONE) || (iarg > LOG_VERBOSE)) {
            mp_raise_ValueError("Log level 0~5 expected");
        }
        config.config.log_level = iarg;
    }
    if (args[10].u_obj != mp_const_none) {
        iarg = mp_obj_get_int(args[10].u_obj);
        if ((iarg < 1) || (iarg > 128)) {
            mp_raise_ValueError("VM divisor out of range");
        }
        config.config.vm_divisor = iarg;
    }
    // Check configuration values
    if (mpy_config.config.use_two_main_tasks != config.config.use_two_main_tasks) changed = true;
    if (mpy_config.config.pystack_enabled != config.config.pystack_enabled) changed = true;
    if (mpy_config.config.heap_size1 != config.config.heap_size1) changed = true;
    if (mpy_config.config.heap_size2 != config.config.heap_size2) changed = true;
    if (mpy_config.config.pystack_size != config.config.pystack_size) changed = true;
    if (mpy_config.config.main_task_stack_size != config.config.main_task_stack_size) changed = true;
    if (mpy_config.config.cpu_clock != config.config.cpu_clock) changed = true;
    if (mpy_config.config.repl_bdr != config.config.repl_bdr) changed = true;
    if (mpy_config.config.boot_menu_pin != config.config.boot_menu_pin) changed = true;
    if (mpy_config.config.log_level != config.config.log_level) changed = true;
    if (mpy_config.config.vm_divisor != config.config.vm_divisor) changed = true;

    if (changed) {
        memcpy((void *)&mpy_config, (void *)&config, sizeof(mpy_config_t));
        bool res = mpy_config_crc(true);
        if (!res) {
            mp_raise_msg(&mp_type_OSError, "Error saving configuration to Flash");
        }
        LOGM("CONFIG", "New flash configuration saved");
        if (args[11].u_bool) mp_printf(&mp_plat_print, "\r\nNew MicroPython configuration:\r\n------------------------------\r\n");
    }
    else if (args[11].u_bool) mp_printf(&mp_plat_print, "\r\nCurrent MicroPython configuration:\r\n----------------------------------\r\n");

    if (args[11].u_bool) {
        mp_printf(&mp_plat_print, " MPy version code: %06X\r\n", mpy_config.config.ver);
        mp_printf(&mp_plat_print, "Two MPy instances: %s\r\n", (mpy_config.config.use_two_main_tasks) ? "True" : "False");
        mp_printf(&mp_plat_print, "     PyStack used: %s\r\n", (mpy_config.config.pystack_enabled) ? "True" : "False");
        mp_printf(&mp_plat_print, "  MPy#1 heap size: %u KB\r\n", mpy_config.config.heap_size1 / 1024);
        mp_printf(&mp_plat_print, "  MPy#2 heap size: %u KB\r\n", mpy_config.config.heap_size2 / 1024);
        mp_printf(&mp_plat_print, "     PyStack size: %u B\r\n", mpy_config.config.pystack_size);
        mp_printf(&mp_plat_print, "   MPy stack size: %u B\r\n", mpy_config.config.main_task_stack_size * sizeof(StackType_t));
        mp_printf(&mp_plat_print, "    CPU frequency: %u MHz\r\n", mpy_config.config.cpu_clock / 1000000);
        mp_printf(&mp_plat_print, "    REPL baudrate: %u bd\r\n", mpy_config.config.repl_bdr);
        mp_printf(&mp_plat_print, "     But menu pin: %d\r\n", mpy_config.config.boot_menu_pin);
        mp_printf(&mp_plat_print, "Default log level: %u (%s)\r\n", mpy_config.config.log_level, log_levels[mpy_config.config.log_level]);
        mp_printf(&mp_plat_print, "       VM divisor: %u\r\n", mpy_config.config.vm_divisor);
    }
    mp_obj_t cfg_tuple[11];
    cfg_tuple[0] = (mpy_config.config.use_two_main_tasks) ? mp_const_true : mp_const_false;
    cfg_tuple[1] = (mpy_config.config.pystack_enabled) ? mp_const_true : mp_const_false;
    cfg_tuple[2] = mp_obj_new_int(mpy_config.config.heap_size1);
    cfg_tuple[3] = mp_obj_new_int(mpy_config.config.heap_size2);
    cfg_tuple[4] = mp_obj_new_int(mpy_config.config.pystack_size);
    cfg_tuple[5] = mp_obj_new_int(mpy_config.config.main_task_stack_size * sizeof(StackType_t));
    cfg_tuple[6] = mp_obj_new_int(mpy_config.config.cpu_clock);
    cfg_tuple[7] = mp_obj_new_int(mpy_config.config.repl_bdr);
    cfg_tuple[8] = mp_obj_new_int(mpy_config.config.boot_menu_pin);
    cfg_tuple[9] = mp_obj_new_int(mpy_config.config.log_level);
    cfg_tuple[10] = mp_obj_new_int(mpy_config.config.vm_divisor);

    return mp_obj_new_tuple(11, cfg_tuple);
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

//------------------------------------------------
STATIC mp_obj_t mod_machine_test (mp_obj_t obj_in)
{
    printf("OBJ: [%016lX]\r\n", (uint64_t)obj_in);
    return obj_in;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_test_obj, mod_machine_test);

//--------------------------------------
STATIC mp_obj_t mod_machine_get_rambuf()
{
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(sys_rambuf_ptr);
    tuple[1] = mp_obj_new_int(SYS_RAMBUF_SIZE);
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


//===========================================================
STATIC const mp_map_elem_t machine_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_machine) },

    { MP_ROM_QSTR(MP_QSTR_mem8),            MP_ROM_PTR(&machine_mem8_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem16),           MP_ROM_PTR(&machine_mem16_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem32),           MP_ROM_PTR(&machine_mem32_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem64),           MP_ROM_PTR(&machine_mem64_obj) },
    { MP_ROM_QSTR(MP_QSTR_rambuf),          MP_ROM_PTR(&mod_machine_get_rambuf_obj) },

    { MP_ROM_QSTR(MP_QSTR_freq),            MP_ROM_PTR(&machine_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_flash_setup),     MP_ROM_PTR(&machine_setflash_obj) },
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
    { MP_ROM_QSTR(MP_QSTR_test),            MP_ROM_PTR(&mod_machine_test_obj) },
    { MP_ROM_QSTR(MP_QSTR_mpy_config),      MP_ROM_PTR(&machine_mpy_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_vm_hook),         MP_ROM_PTR(&mod_machine_vm_hook_obj) },
    { MP_ROM_QSTR(MP_QSTR_vm_hook_wdt),     MP_ROM_PTR(&mod_machine_wdt_reset_in_vm_hook_obj) },
    #if INCLUDE_FLASH_TEST
    { MP_ROM_QSTR(MP_QSTR_test_flash),      MP_ROM_PTR(&machine_test_flash_obj) },
    #endif

    { MP_ROM_QSTR(MP_QSTR_Pin),             MP_ROM_PTR(&machine_pin_type) },
    { MP_ROM_QSTR(MP_QSTR_UART),            MP_ROM_PTR(&machine_uart_type) },
    { MP_ROM_QSTR(MP_QSTR_I2C),             MP_ROM_PTR(&machine_hw_i2c_type) },
    { MP_ROM_QSTR(MP_QSTR_SPI),             MP_ROM_PTR(&machine_hw_spi_type) },

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
