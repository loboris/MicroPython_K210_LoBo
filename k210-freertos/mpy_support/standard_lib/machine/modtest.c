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
#include <math.h>
#include "uarths.h"

#include "py/runtime.h"
#include "py/mpprint.h"
#include "modmachine.h"
#include "mphalport.h"
#include "w25qxx.h"

#if MICROPY_PY_USE_TEST_MODULE

//------------------------------------------------------------------------------------
STATIC mp_obj_t test_flash(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_mode, ARG_type, ARG_pll0, ARG_cpufreq, ARG_speed };
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_mode,      MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_type,      MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_pll0,      MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_cpufreq,   MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_speed,     MP_ARG_KW_ONLY | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t type = 0;
    bool freq_changed = false;

    uint32_t old_pll0 = sysctl_clock_get_freq(SYSCTL_CLOCK_PLL0);
    uint32_t old_cpufreq = sysctl_clock_get_freq(SYSCTL_CLOCK_CPU);
    uint8_t oldmode = work_trans_mode;
    uint32_t oldspeed = w25qxx_flash_speed;

    uint32_t pll0 = sysctl_clock_get_freq(SYSCTL_CLOCK_PLL0);
    uint32_t cpufreq = sysctl_clock_get_freq(SYSCTL_CLOCK_CPU);
    uint8_t mode = work_trans_mode;
    uint32_t speed = w25qxx_flash_speed;

    if (args[ARG_type].u_obj != mp_const_none) type = (uint8_t)mp_obj_get_int(args[ARG_type].u_obj);
    if (args[ARG_mode].u_obj != mp_const_none) mode = mp_obj_get_int(args[ARG_mode].u_obj) & 0x03;
    if (args[ARG_pll0].u_obj != mp_const_none) pll0 = (uint32_t)mp_obj_get_int(args[ARG_pll0].u_obj);
    if (args[ARG_cpufreq].u_obj != mp_const_none) cpufreq = (uint32_t)mp_obj_get_int(args[ARG_cpufreq].u_obj);
    if (args[ARG_speed].u_obj != mp_const_none) speed = (uint32_t)mp_obj_get_int(args[ARG_speed].u_obj);

    if (mode > 2) mode = 2;
    if (speed > 85000000) speed = oldspeed;
    if ((pll0 < 247000000) || (pll0 > 1000000000)) pll0 = old_pll0;
    if ((cpufreq < 30000000) || (cpufreq > 500000000)) cpufreq = old_cpufreq;

    if ((cpufreq != old_cpufreq) || (pll0 != old_pll0)) {
        freq_changed = true;
        sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_MAX_OUTPUT_FREQ);
        mp_hal_set_cpu_frequency(124000000);
        sysctl_pll_set_freq(SYSCTL_PLL0, pll0);
        mp_hal_set_cpu_frequency(cpufreq);
        uint64_t count_us = mp_hal_ticks_us();
        sys_us_counter_cpu = read_csr64(mcycle);
        sys_us_counter = count_us;
        uarths_init(uarths_baudrate);
    }

    uint32_t real_speed = w25qxx_init(flash_spi, mode, speed);
    uint32_t spi_speed = sysctl_clock_get_freq(SYSCTL_CLOCK_SPI3);

    mp_printf(&mp_plat_print, "\nFLASH_TEST: CPU_freq=%u, PLL0=%u, flash_mode=%d, flash_speed=%d (%u, spi_clk=%u)\n",
            sysctl_clock_get_freq(SYSCTL_CLOCK_CPU), sysctl_clock_get_freq(SYSCTL_CLOCK_PLL0), mode, speed, real_speed, spi_speed);
    mp_printf(&mp_plat_print, "-----------\n");
    vTaskDelay(50);

    if (type == 0) {
        if (freq_changed) {
            sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_MAX_OUTPUT_FREQ);
            mp_hal_set_cpu_frequency(124000000);
            sysctl_pll_set_freq(SYSCTL_PLL0, old_pll0);
            mp_hal_set_cpu_frequency(old_cpufreq);
            vTaskDelay(10);
        }
        w25qxx_init(flash_spi, oldmode, oldspeed);
        return mp_const_none;
    }

    uint32_t rd = 0;
    uint32_t wr = 0;
    uint32_t er = 0;
    uint64_t wqtime = 0;
    uint32_t sector;
    uint32_t start_addr = MICRO_PY_FLASH_USED_END;
    uint32_t end_addr = MICRO_PY_FLASH_SIZE - start_addr;
    // use 1 MB maximum for testing
    if ((end_addr-start_addr) > 0x100000) end_addr = start_addr + 0x100000;

    int count = 0;
    enum w25qxx_status_t res;
    uint64_t start_time, end_time;
    uint8_t __attribute__((aligned(8))) buf[4096];
    uint8_t __attribute__((aligned(8))) rdbuf[4096];

    bool old_spicheck = w25qxx_spi_check;
    w25qxx_spi_check = false;

    if ((type & 0x0f) > 1) {
        mp_printf(&mp_plat_print, "\nERASE flash\n");
        count = 0;
        w25qxx_clear_counters();
        start_time = mp_hal_ticks_us();
        for (sector = start_addr; sector < end_addr; sector+=4096) {
            memset(rdbuf, 0, 4096);
            res = w25qxx_sector_erase(sector);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "  Erase error at %x\n", sector);
                goto exit;
            }
            count++;
            mp_hal_wdt_reset();
        }
        end_time = mp_hal_ticks_us();
        mp_printf(&mp_plat_print, "  Erase time: %luus, %luus/sector\n", end_time-start_time, (end_time-start_time) / count);
        w25qxx_get_counters(&rd, &wr, &er, &wqtime);
        mp_printf(&mp_plat_print, "  Flash counters: reads: %u, writes: %u, erases: %u, time=%lu\n", rd, wr, er, wqtime);

        count = 0;
        w25qxx_clear_counters();
        start_time = mp_hal_ticks_us();
        for (sector = start_addr; sector < end_addr; sector+=4096) {
            res = w25qxx_read_data(sector, rdbuf, 4096);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "  Erase read back error at sector %x\n", sector);
                goto exit;
            }
            for (int i=0; i<4096; i++) {
                if (rdbuf[i] != 0xFF) {
                    mp_printf(&mp_plat_print, "Erase compare error at sector %x, pos %d (%2x)\n", sector, i, rdbuf[i]);
                    goto exit;
                }
            }
            count++;
        }
        end_time = mp_hal_ticks_us();
        w25qxx_get_counters(&rd, &wr, &er, &wqtime);
        mp_printf(&mp_plat_print, "  Read/compare time: %luus, %luus/sector\n", end_time-start_time, (end_time-start_time) / count);
        mp_printf(&mp_plat_print, "  Flash counters: reads: %u, writes: %u, erases: %u, time=%lu\n", rd, wr, er, wqtime);
    }

    mp_hal_wdt_reset();
    for (int n=0; n<4096; n++) {
        if (n & 1) buf[n] = 0x55;
        else buf[n] = 0xaa;
    }

    if (type & 0xf0) {
        mp_printf(&mp_plat_print, "\nFLASH program\n");
        count = 0;
        w25qxx_clear_counters();
        start_time = mp_hal_ticks_us();
        for (sector = start_addr; sector < end_addr; sector+=4096) {
            // write sector with known pattern
            buf[0] = 0x3c;
            buf[1] = (uint8_t)(count & 0xff);
            buf[2] = (uint8_t)((count>>8) & 0xff);
            res = w25qxx_write_data(sector, buf, 4096);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "  Write error at %x\n", sector);
                goto exit;
            }
            count++;
        }
        end_time = mp_hal_ticks_us();
        w25qxx_get_counters(&rd, &wr, &er, &wqtime);
        mp_printf(&mp_plat_print, "  Program time for %d sectors %luus, %luus/sector\n", count, end_time-start_time, (end_time-start_time) / count);
        mp_printf(&mp_plat_print, "  Flash counters: reads: %u, writes: %u, erases: %u, time=%lu\n", rd, wr, er, wqtime);
    }

    mp_printf(&mp_plat_print, "\nFLASH read\n");
    w25qxx_clear_counters();
    for (int n=0; n<16; n++) {
        count = 0;
        for (sector = start_addr; sector < end_addr; sector+=4096) {
            res = w25qxx_read_data(sector, rdbuf, 4096);
            if (res != W25QXX_OK) {
                mp_printf(&mp_plat_print, "  Read error at sector %08X, pass=%d\n", sector, n);
                w25qxx_spi_check = old_spicheck;
                goto exit;
            }
            if (type & 0xf0) {
                buf[0] = 0x3c;
                buf[1] = (uint8_t)(count & 0xff);
                buf[2] = (uint8_t)((count>>8) & 0xff);
                // compare sector
                for (int i=0; i<4096; i++) {
                    if (buf[i] != rdbuf[i]) {
                        mp_printf(&mp_plat_print, "  Compare error at sector %d, addr=%08X, pos=%d, pass=%d\n", count, sector, i, n);
                        for (int k=0; k<16; k++) {
                            mp_printf(&mp_plat_print, "%02X ", rdbuf[k]);
                        }
                        mp_printf(&mp_plat_print, " [");
                        for (int k=0; k<16; k++) {
                            mp_printf(&mp_plat_print, "%02X ", buf[k]);
                        }
                        mp_printf(&mp_plat_print, "]\n");
                        w25qxx_spi_check = old_spicheck;
                        goto exit;
                    }
                }
            }
            else if ((count == 8) && (n == 0)) {
                buf[0] = 0x3c;
                buf[1] = (uint8_t)(count & 0xff);
                buf[2] = (uint8_t)((count>>8) & 0xff);
                mp_printf(&mp_plat_print, "  Sector %d (addr=%08X) content:\n  ", count, sector);
                for (int k=0; k<16; k++) {
                    mp_printf(&mp_plat_print, "%02X ", rdbuf[k]);
                }
                mp_printf(&mp_plat_print, " [");
                for (int k=0; k<16; k++) {
                    mp_printf(&mp_plat_print, "%02X ", buf[k]);
                }
                mp_printf(&mp_plat_print, "]\n");
            }
            count++;
        }
        mp_hal_wdt_reset();
    }
    end_time = mp_hal_ticks_us();
    w25qxx_get_counters(&rd, &wr, &er, &wqtime);
    w25qxx_spi_check = old_spicheck;
    mp_printf(&mp_plat_print, "  Read time for %d sectors: %luus, %luus/sector\n", rd, wqtime, wqtime / rd);
    mp_printf(&mp_plat_print, "  Flash counters: reads: %u, writes: %u, erases: %u, time=%lu\n\n", rd, wr, er, wqtime);

exit:
    vTaskDelay(50);
    if (freq_changed) {
        sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_MAX_OUTPUT_FREQ);
        mp_hal_set_cpu_frequency(124000000);
        sysctl_pll_set_freq(SYSCTL_PLL0, old_pll0);
        mp_hal_set_cpu_frequency(old_cpufreq);
        uint64_t count_us = mp_hal_ticks_us();
        sys_us_counter_cpu = read_csr64(mcycle);
        sys_us_counter = count_us;
        uarths_init(uarths_baudrate);
        vTaskDelay(10);
    }
    w25qxx_init(flash_spi, oldmode, oldspeed);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(test_flash_obj, 0, test_flash);


// double / float test
//-----------------------------------------------
STATIC mp_obj_t ftest(mp_obj_t in1, mp_obj_t in2)
{
    double d1, d2, d3, d4;
    float f1, f2, f3, f4;
    uint64_t start_time, end_time;

    d1 = (double)mp_obj_get_float(in1);
    d2 = (double)mp_obj_get_float(in2);
    f1 = (float)mp_obj_get_float(in1);
    f2 = (float)mp_obj_get_float(in2);

    d3 = d2 / d1;
    d4 = sqrt(d3);
    printf("%.22f %.22f\r\n", d3, d4);
    start_time = mp_hal_ticks_us();
    for (int i=0; i<10000; i++) {
        d3 = d2 / d1;
        d4 = sqrt(d3);
        d1 += (double)0.1;
    }
    end_time = mp_hal_ticks_us();
    printf("%.22f %.22f (%lu us)\r\n", d3, d4, end_time-start_time);
    mp_printf(&mp_plat_print, "%.22f %.22f (%lu us)\r\n\r\n", d3, d4, end_time-start_time);

    f3 = f2 / f1;
    f4 = sqrt(f3);
    printf("%.22f %.22f\r\n", f3, f4);
    start_time = mp_hal_ticks_us();
    for (int i=0; i<10000; i++) {
        f3 = f2 / f1;
        f4 = sqrt(f3);
        f1 += (float)0.1;
    }
    end_time = mp_hal_ticks_us();
    printf("%.22f %.22f (%lu us)\r\n", f3, f4, end_time-start_time);
    mp_printf(&mp_plat_print, "%.22f %.22f (%lu us)\r\n", f3, f4, end_time-start_time);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(ftest_obj, ftest);

//------------------------------------
STATIC mp_obj_t test (mp_obj_t obj_in)
{
    printf("OBJ: [%016lX]\r\n", (uint64_t)obj_in);
    return obj_in;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(test_obj, test);


static handle_t timer_dev = 0;
static size_t timer_counter = 0;

//----------------------------------------
STATIC void test_timer_isr(void *userdata)
{
    timer_counter++;
    //timer_set_enable(timer_dev, false);
}

//-----------------------------------------
STATIC mp_obj_t test_timer(mp_obj_t obj_in)
{
    // max interval = 8694265779 ns @ 494 MHz clock
    size_t res = 0;
    if (mp_obj_is_true(obj_in)) {
        timer_dev = io_open("/dev/timer0");
        timer_set_on_tick(timer_dev, test_timer_isr, NULL);
        res = timer_set_interval(timer_dev, 1000000);

        timer_set_enable(timer_dev, true);
        vTaskDelay(20);
    }
    else {
        double resolution = 0;
        uint32_t load = 0;
        uint32_t val = timer_get_value(timer_dev, &resolution, &load);
        res = timer_counter;
        mp_obj_t tuple[4];
        tuple[0] = mp_obj_new_int(res);
        tuple[1] = mp_obj_new_float(resolution);
        tuple[2] = mp_obj_new_int(load);
        tuple[3] = mp_obj_new_int(val);
        return mp_obj_new_tuple(4, tuple);
    }

    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(test_timer_obj, test_timer);


//===========================================================
STATIC const mp_map_elem_t test_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_test) },

    { MP_ROM_QSTR(MP_QSTR_test_flash),      MP_ROM_PTR(&test_flash_obj) },
    { MP_ROM_QSTR(MP_QSTR_ftest),           MP_ROM_PTR(&ftest_obj) },
    { MP_ROM_QSTR(MP_QSTR_test),            MP_ROM_PTR(&test_obj) },
    { MP_ROM_QSTR(MP_QSTR_timer),           MP_ROM_PTR(&test_timer_obj) },
};

//===========================
STATIC MP_DEFINE_CONST_DICT (
    test_module_globals,
    test_module_globals_table
);

const mp_obj_module_t mp_test_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&test_module_globals,
};

#endif // MICROPY_PY_USE_TEST_MODULE
