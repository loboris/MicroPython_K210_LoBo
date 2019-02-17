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
#include <stdio.h>
#include "hal.h"
#include "modmachine.h"
#include "mphalport.h"
#include "extmod/machine_mem.h"
#include "w25qxx.h"
#include "spiffs_config.h"

#if MICROPY_PY_MACHINE

#define CRC32_INIT (0)

extern handle_t flash_spi;
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

//---------------------------------------------------------------
STATIC mp_obj_t machine_freq(size_t n_args, const mp_obj_t *args)
{
    mp_obj_t tuple[2];
    if (n_args == 0) {
        // get CPU frequency
        tuple[0] = mp_obj_new_int(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));
        tuple[1] = mp_obj_new_int(w25qxx_flash_speed);
    }
    else {
        // set CPU frequency
        mp_int_t freq = mp_obj_get_int(args[0]);
        if ((freq != 200) && (freq != 300) && (freq != 400) && (freq != 450) && (freq != 500)) {
            mp_raise_ValueError("Allowed CPU frequencies: 200, 300, 400, 450 and 500 MHz");
        }
        freq *= 1000000;

        system_set_cpu_frequency(freq);
        log_divisor = (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);

        uint32_t flash_speed = 16000000;
        if (freq == 200000000) flash_speed = 8000000;
        else if (freq == 300000000) flash_speed = 12000000;
        else if (freq == 400000000) flash_speed = 16000000;
        else if (freq == 450000000) flash_speed = 18000000;
        else if (freq == 500000000) flash_speed = 20000000;

        flash_speed = w25qxx_init(flash_spi, SPI_FF_QUAD, flash_speed);

        tuple[0] = mp_obj_new_int(uxPortGetCPUClock());
        tuple[1] = mp_obj_new_int(flash_speed);
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
        if (flash_speed <= 50) flash_speed *= 1000000;
        if ((flash_speed < 1000000) || (flash_speed > 20000000)) {
            mp_raise_ValueError("Flash speed must be in 1 - 20 MHz range");
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
    mp_printf(&mp_plat_print, " Pin  GpioHS     Used by  Fpioa\n");
    mp_printf(&mp_plat_print, "-------------------------------\n");
    for (i=0; i<FPIOA_NUM_IO; i++) {
        if (mp_used_pins[i].func != GPIO_FUNC_NONE) {
            if (mp_used_pins[i].gpio < 0) sprintf(sgpio, "%s", "-");
            else sprintf(sgpio, "%d", mp_used_pins[i].gpio);
            mp_printf(&mp_plat_print, "%4d%8s%12s%7d\n", i, sgpio, gpiohs_funcs[mp_used_pins[i].func], mp_used_pins[i].fpioa_func);
        }
    }
    mp_printf(&mp_plat_print, "-------------------------------\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_pinstat_obj, machine_pinstat);

/*
//---------------------------------------------------------------------
STATIC mp_obj_t machine_test_flash(mp_obj_t mode_in, mp_obj_t speed_in)
{
    uint32_t oldmode = work_trans_mode;
    uint32_t oldspeed =w25qxx_flash_speed;
    int mode = mp_obj_get_int(mode_in);
    int speed = mp_obj_get_int(speed_in);
    uint32_t real_speed = w25qxx_init(flash_spi, mode, speed);
    uint32_t spi_speed = sysctl_clock_get_freq(SYSCTL_CLOCK_SPI3);

    uint32_t rd;
    uint32_t wr;
    uint32_t er;
    uint32_t sector;
    uint32_t start_addr = 0xA00000;
    int count = 0, retry;
    enum w25qxx_status_t res;
    uint64_t start_time, end_time;
    uint8_t buf[4096];
    uint8_t rdbuf[4096];

    w25qxx_clear_counters();

    mp_printf(&mp_plat_print, "\nFLASH test: mode=%d, speed=%d (%u, %u)\n", mode, speed, real_speed, spi_speed);
    mp_printf(&mp_plat_print, "-----------\n\n");
    mp_printf(&mp_plat_print, "ERASE test\n");
    count = 0;
    start_time = mp_hal_ticks_us();
    for (sector = start_addr; sector < 0xB00000; sector+=4096) {
        memset(rdbuf, 0, 4096);
        w25qxx_sector_erase(sector);
        res = w25qxx_wait_busy();
        if (res != W25QXX_OK) {
            mp_printf(&mp_plat_print, "Erase error at %x\n", sector);
            goto exit;
        }
        res = w25qxx_read_data(sector, rdbuf, 4096);
        for (int i=0; i<4096; i++) {
            if (rdbuf[i] != 0xFF) {
                mp_printf(&mp_plat_print, "Erase test error at sector %x, pos %d (%2x)\n", sector, i, rdbuf[i]);
                goto exit;
            }
        }
        count++;
    }
    end_time = mp_hal_ticks_us();
    mp_printf(&mp_plat_print, "Erase time: %lu, %lu/sector\n", end_time-start_time, (end_time-start_time) / count);

    for (int n=0; n<2; n++) {
        mp_printf(&mp_plat_print, "\nWRITE/READ test, pass %d\n", n+1);
        memset(buf, ((n == 0) ? 0x55 : 0xaa), 4096);
        count = 0;
        start_time = mp_hal_ticks_us();
        for (sector = start_addr; sector < 0xB00000; sector+=4096) {
            memset(rdbuf, 0, 4096);
            retry = 0;
restart:
            // write sector
            res = w25qxx_write_data(sector, buf, 4096);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "Write error at %x (%d)\n", sector, retry);
                retry++;
                if (retry < 3) goto restart;
                goto endtest;
            }
            // read sector and compare
            res = w25qxx_read_data(sector, rdbuf, 4096);
            for (int i=0; i<4096; i++) {
                if (buf[i] != rdbuf[i]) {
                    mp_printf(&mp_plat_print, "Compare error at sector %x, pos %d (%2x <> %2x) (%d)\n", sector, i, rdbuf[i], buf[i], retry);
                    retry++;
                    if (retry < 3) goto restart;
                    goto endtest;
                }
            }
            count++;
        }
endtest:
        end_time = mp_hal_ticks_us();
        mp_printf(&mp_plat_print, "Write/read/compare time: %lu, %lu/sector (%d)\n", end_time-start_time, (end_time-start_time) / count, retry);
        w25qxx_get_counters(&rd, &wr, &er, NULL);
        mp_printf(&mp_plat_print, "reads: %u, writes: %u, erases: %u\n", rd, wr, er);
    }
exit:
    w25qxx_get_counters(&rd, &wr, &er, NULL);
    mp_printf(&mp_plat_print, "reads: %u, writes: %u, erases: %u\n", rd, wr, er);
    w25qxx_init(flash_spi, oldmode, oldspeed);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_test_flash_obj, machine_test_flash);
*/

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

//------------------------------------------------------------
STATIC mp_obj_t mod_machine_log_timestamp(mp_obj_t time_us_in)
{
    if (mp_obj_is_true(time_us_in)) log_divisor = (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
    else log_divisor = 1;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_log_timestamp_obj, mod_machine_log_timestamp);

//------------------------------------------------
STATIC mp_obj_t mod_machine_crc16(mp_obj_t buf_in)
{
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    uint16_t crc = mp_hal_crc16((uint8_t *)bufinfo.buf, bufinfo.len);
    return mp_obj_new_int(crc);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_machine_crc16_obj, mod_machine_crc16);

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


//===========================================================
STATIC const mp_map_elem_t machine_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_machine) },

    { MP_ROM_QSTR(MP_QSTR_mem8),            MP_ROM_PTR(&machine_mem8_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem16),           MP_ROM_PTR(&machine_mem16_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem32),           MP_ROM_PTR(&machine_mem32_obj) },

    { MP_ROM_QSTR(MP_QSTR_freq),            MP_ROM_PTR(&machine_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_flash_setup),     MP_ROM_PTR(&machine_setflash_obj) },
    { MP_ROM_QSTR(MP_QSTR_random),          MP_ROM_PTR(&machine_random_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),           MP_ROM_PTR(&machine_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_pinstat),         MP_ROM_PTR(&machine_pinstat_obj) },
    //{ MP_ROM_QSTR(MP_QSTR_test_flash),      MP_ROM_PTR(&machine_test_flash_obj) },
    { MP_ROM_QSTR(MP_QSTR_loglevel),        MP_ROM_PTR(&mod_machine_log_level_obj) },
    { MP_ROM_QSTR(MP_QSTR_log_us),          MP_ROM_PTR(&mod_machine_log_timestamp_obj) },
    { MP_ROM_QSTR(MP_QSTR_crc16),           MP_ROM_PTR(&mod_machine_crc16_obj) },
    { MP_ROM_QSTR(MP_QSTR_repl_baudrate),   MP_ROM_PTR(&mod_machine_baudrate_obj) },

    { MP_ROM_QSTR(MP_QSTR_Pin),             MP_ROM_PTR(&machine_pin_type) },
    //{ MP_ROM_QSTR(MP_QSTR_UART),            MP_ROM_PTR(&machine_uart_type) },

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
