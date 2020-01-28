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


#include "mpconfigport.h"

#if MICROPY_PY_USE_TEST_MODULE

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
#include "camera/dvp_camera.h"
#include "../display/tftspi.h"
#include "py/objstr.h"

static bool camera_is_init = false;
static sensor_t sensor = {0};
static int pio_num = 0;
static handle_t timer_dev = 0;
static handle_t timer_dev1 = 0;
static size_t timer_counter = 0;

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
    printf("size_t: %lu\r\n", sizeof(size_t));
    return obj_in;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(test_obj, test);


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
        timer_dev1 = io_open("/dev/timer1");
        timer_set_on_tick(timer_dev, test_timer_isr, NULL);
        res = timer_set_interval(timer_dev, 1000000000); // 1 second
        printf("Interval #1: %lu\r\n", res);
        res = timer_set_interval(timer_dev1, 20000000000); // 20 seconds
        printf("Interval #2: %lu\r\n", res);

        timer_set_enable(timer_dev, true);
        timer_set_enable(timer_dev1, true);
        vTaskDelay(20);
    }
    else {
        double resolution = 0;
        size_t runtime1 = 1;
        size_t runtime2 = 1;
        size_t val1 = timer_get_value(timer_dev, &resolution, &runtime1);
        size_t val2 = timer_get_value(timer_dev1, NULL, &runtime2);
        res = timer_counter;
        mp_obj_t tuple[6];
        tuple[0] = mp_obj_new_int(res);
        tuple[1] = mp_obj_new_float(resolution);
        tuple[2] = mp_obj_new_int(val1);
        tuple[3] = mp_obj_new_int(val2);
        tuple[4] = mp_obj_new_float(((double)runtime1 * resolution) / 1e9);
        tuple[5] = mp_obj_new_float(((double)runtime2 * resolution) / 1e9);
        return mp_obj_new_tuple(6, tuple);
    }

    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(test_timer_obj, test_timer);

//===========================================================


static SemaphoreHandle_t dvp_semaphore = NULL;

//-------------------------------------------------------------
static void on_dvp_irq(dvp_frame_event_t event, void* userdata)
{
    sensor_t *sensor = (sensor_t *)userdata;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (event)
    {
        case VIDEO_FE_BEGIN:
            if (!sensor->dvp_finish_flag) {
                dvp_enable_frame(sensor->dvp_handle);
                gpio_set_pin_value(gpiohs_handle, pio_num, 1);
            }
            break;
        case VIDEO_FE_END:
            // Frame complete
            if (sensor->frame_count) sensor->frame_count--;
            sensor->gram_mux ^= 0x01;
            if (sensor->frame_count) {
                dvp_set_output_attributes(sensor->dvp_handle, DATA_FOR_DISPLAY, VIDEO_FMT_RGB565, sensor->gram_mux ? sensor->gram1 : sensor->gram0);
            }
            else {
                sensor->dvp_finish_flag = true;
                dvp_disable(sensor);
            }
            gpio_set_pin_value(gpiohs_handle, pio_num, 0);
            xSemaphoreGiveFromISR(dvp_semaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken)
            {
                portYIELD_FROM_ISR();
            }
            break;
        default:
            sensor->frame_count = 0;
            dvp_disable(sensor);
            sensor->dvp_finish_flag = true;
            gpio_set_pin_value(gpiohs_handle, pio_num, 0);
            gpio_set_pin_value(gpiohs_handle, pio_num, 1);
            gpio_set_pin_value(gpiohs_handle, pio_num, 0);
            gpio_set_pin_value(gpiohs_handle, pio_num, 1);
            gpio_set_pin_value(gpiohs_handle, pio_num, 0);
            gpio_set_pin_value(gpiohs_handle, pio_num, 1);
    }
}

#include "../display/moddisplay.h"

//-------------------------------------------------------------------------------------
STATIC mp_obj_t test_camera(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_n, ARG_bar, ARG_mode, ARG_size, ARG_qs, ARGS_type, ARG_test, ARG_fb, ARG_effect, ARG_night, ARG_contrast, ARG_width, ARG_height };
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_n,         MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = 10 } },
       { MP_QSTR_bar,       MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = 0 } },
       { MP_QSTR_mode,      MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = PIXFORMAT_RGB565 } },
       { MP_QSTR_size,      MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = FRAMESIZE_QVGA } },
       { MP_QSTR_qs,        MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = -1 } },
       { MP_QSTR_type,      MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = SENSORTYPE_OV2640 } },
       { MP_QSTR_test,      MP_ARG_KW_ONLY | MP_ARG_BOOL, { .u_bool = false } },
       { MP_QSTR_fb,        MP_ARG_KW_ONLY | MP_ARG_BOOL, { .u_bool = true } },
       { MP_QSTR_effect,    MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = -1 } },
       { MP_QSTR_night,     MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = -1 } },
       { MP_QSTR_contrast,  MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = -5 } },
       { MP_QSTR_width,     MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = 0 } },
       { MP_QSTR_height,    MP_ARG_KW_ONLY | MP_ARG_INT,  { .u_int = 0 } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_obj_t buff_obj0 = mp_const_none, buff_obj1 = mp_const_none;
    int n = args[ARG_n].u_int;
    if (n < 1) n = 1;
    int bar = args[ARG_bar].u_int & 0xff;
    int mode = args[ARG_mode].u_int;
    int size = args[ARG_size].u_int;
    bool test = args[ARG_test].u_bool;
    //if (mode == PIXFORMAT_JPEG) n = 1;
    mp_uint_t tstart, tend;
    //if (mode != PIXFORMAT_JPEG) size = FRAMESIZE_QVGA;

    if (pio_num == 0) {
        pio_num = gpiohs_get_free();
        if (pio_num) {
            if (fpioa_set_function(18, FUNC_GPIOHS0 + pio_num) == 0) {
                gpio_set_drive_mode(gpiohs_handle, pio_num, GPIO_DM_OUTPUT);
                gpio_set_pin_value(gpiohs_handle, pio_num, 0);
            }
        }
    }
    else gpio_set_pin_value(gpiohs_handle, pio_num, 0);
    vTaskDelay(10);

    if (!camera_is_init) {
        fpioa_set_function(42, FUNC_CMOS_RST);
        fpioa_set_function(44, FUNC_CMOS_PWDN);
        fpioa_set_function(46, FUNC_CMOS_XCLK);
        fpioa_set_function(43, FUNC_CMOS_VSYNC);
        fpioa_set_function(45, FUNC_CMOS_HREF);
        fpioa_set_function(47, FUNC_CMOS_PCLK);
        fpioa_set_function(41, FUNC_SCCB_SCLK);
        fpioa_set_function(40, FUNC_SCCB_SDA);

        sysctl_set_spi0_dvp_data(1);
        memset(&sensor, 0, sizeof(sensor_t));
        camera_is_init = true;
    }

    dvp_deinit(&sensor);

    memset(&sensor, 0, sizeof(sensor_t));
    sensor.sensor_type = args[ARGS_type].u_int;
    sensor.pixformat = mode;
    sensor.framesize = size;

    sensor.irq_func = on_dvp_irq;
    sensor.irq_func_data = (void *)&sensor;
    sensor.frame_count = n;
    sensor.dvp_finish_flag = true;

    if (!dvp_init(&sensor)) {
        mp_raise_msg(&mp_type_OSError, "Error initializing DVP");
        return mp_const_false;
    }

    buff_obj0 = mp_obj_new_frame_buffer(sensor.gram_size+8);
    if (buff_obj0 == mp_const_none) {
        dvp_deinit(&sensor);
        mp_raise_msg(&mp_type_OSError, "Error creating camera frame buffer #0.");
    }
    mp_obj_array_t *gram0 = (mp_obj_array_t *)buff_obj0;
    sensor.gram0 = gram0->items + 8;

    buff_obj1 = mp_obj_new_frame_buffer(sensor.gram_size+8);
    if (buff_obj1 == mp_const_none) {
        dvp_deinit(&sensor);
        if (buff_obj0 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj0);
        if (buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj1);
        mp_raise_msg(&mp_type_OSError, "Error creating camera frame buffer #1.");
    }
    mp_obj_array_t *gram1 = (mp_obj_array_t *)buff_obj1;
    sensor.gram1 = gram1->items + 8;

    if (!dvp_semaphore) dvp_semaphore = xSemaphoreCreateBinary();
    if (!dvp_semaphore) {
        dvp_deinit(&sensor);
        if (buff_obj0 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj0);
        if (buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj1);
        mp_raise_msg(&mp_type_OSError, "Error creating camera semaphore.");
    }

    //sensor.set_pixformat(mode);
    sensor.set_colorbar(bar);
    if (args[ARG_effect].u_int >= 0) sensor.set_special_effect((uint8_t)args[ARG_effect].u_int);
    if (args[ARG_night].u_int >= 0) sensor.set_night_mode((uint8_t)args[ARG_night].u_int);
    if (args[ARG_contrast].u_int > -5) sensor.set_contrast((uint8_t)args[ARG_contrast].u_int);
    if ((mode == PIXFORMAT_JPEG) && (args[ARG_qs].u_int > 0)) sensor.set_quality(args[ARG_qs].u_int);
    //if (mode == PIXFORMAT_JPEG) vTaskDelay(500);

    uint16_t *fb = tft_frame_buffer;
    memset(fb, 0, 320*240*2);
    int fb_ok = 0;
    int fb_ok_n = -1;
    mp_obj_t buffer = mp_const_true;

    gpio_set_pin_value(gpiohs_handle, pio_num, 1);
    vTaskDelay(4);
    gpio_set_pin_value(gpiohs_handle, pio_num, 0);
    vTaskDelay(8);

    mp_hal_wdt_reset();
    tstart = mp_hal_ticks_ms();
    sensor.dvp_finish_flag = false;
    dvp_enable(&sensor);

    while (sensor.frame_count) {
        if (xSemaphoreTake(dvp_semaphore, 750/portTICK_PERIOD_MS ) != pdTRUE) {
            sensor.frame_count = 0;
            dvp_deinit(&sensor);
            if (buff_obj0 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj0);
            if (buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj1);
            if (dvp_semaphore) vSemaphoreDelete(dvp_semaphore);
            dvp_semaphore = NULL;
            mp_raise_msg(&mp_type_OSError, "Error waiting for frame.");
        }

        if (!test) {
            if (mode != PIXFORMAT_JPEG) {
                uint16_t *cbuf = (uint16_t *)((sensor.gram_mux) ? sensor.gram0 : sensor.gram1);
                uint16_t cbufinc = 1;
                if (dvp_cam_resolution[sensor.framesize][0] > 320) cbufinc = dvp_cam_resolution[sensor.framesize][0] / 320;
                int x, y;
                for (y=0; y<dvp_cam_resolution[sensor.framesize][1]; y+=cbufinc) {
                    for (x=0; x<dvp_cam_resolution[sensor.framesize][0]; x += (cbufinc*2)) {
                        fb[(y/cbufinc*320) + (x/cbufinc)] = cbuf[(y*dvp_cam_resolution[sensor.framesize][0]) + x + 1];
                        fb[(y/cbufinc*320) + (x/cbufinc) + 1] = cbuf[(y*dvp_cam_resolution[sensor.framesize][0]) + x];
                    }
                }
                tft_frame_buffer = fb;
                send_frame_buffer();
            }
            else {
                uint8_t *frame_buffer = sensor.gram_mux ? sensor.gram0 : sensor.gram1;
                // jpeg data have swapped 4-byte values
                uint8_t tmpb0, tmpb1;
                for (int i = 0; i < sensor.gram_size; i+=4) {
                    tmpb0 = frame_buffer[i];
                    tmpb1 = frame_buffer[i+1];
                    frame_buffer[i] = frame_buffer[i+3];
                    frame_buffer[i+1] = frame_buffer[i+2];
                    frame_buffer[i+2] = tmpb1;
                    frame_buffer[i+3] = tmpb0;
                }
                uint32_t jpeg_buffer_idx = 0;
                if ((frame_buffer[0] == 0xff) && (frame_buffer[1] == 0xd8) && (memcmp(frame_buffer+6, "JFIF", 4) == 0)) {
                    for (int i = 0; i < (sensor.gram_size-4); i++) {
                        if ((frame_buffer[i] == 0xff) && (frame_buffer[i+1] == 0xd9)) {
                            jpeg_buffer_idx = i+2;
                            break;
                        }
                    }
                }
                if (jpeg_buffer_idx > 0) {
                    uint8_t scale = 0;
                    if (TFT_jpg_image(0, 0, scale, mp_const_none, frame_buffer, jpeg_buffer_idx+2)) {
                        fb_ok++;
                        fb_ok_n = sensor.frame_count;
                        if (buffer == mp_const_true) buffer = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)frame_buffer, jpeg_buffer_idx+2);
                        send_frame_buffer();
                    }
                }
            }
        }
        mp_hal_wdt_reset();
        if (mode != PIXFORMAT_JPEG) {
            sensor.set_pixformat(sensor.pixformat);
        }
    }
    tend = mp_hal_ticks_ms();

    uint8_t *frame_buffer = sensor.gram_mux ? sensor.gram0 : sensor.gram1;

    mp_hal_wdt_reset();
    vTaskDelay(1000);
    mp_hal_wdt_reset();

    dvp_deinit(&sensor);
    vTaskDelay(10);
    gpio_set_pin_value(gpiohs_handle, pio_num, 1);
    vTaskDelay(4);
    gpio_set_pin_value(gpiohs_handle, pio_num, 0);

    printf("Time: %lu ms, framerate=%0.2f, jpegOK=%d (%d)\r\n", tend-tstart, 1.0 / ((double)(tend-tstart) / 1000.0 / (double)n), fb_ok, fb_ok_n);

    mp_hal_wdt_reset();
    if (test) {
        if (mode != PIXFORMAT_JPEG) {
            if ((dvp_cam_resolution[sensor.framesize][0] != 320) || (dvp_cam_resolution[sensor.framesize][1] != 240)) {
                uint8_t cbufinc = 1;
                uint16_t *cbuf = (uint16_t *)frame_buffer;
                if (dvp_cam_resolution[sensor.framesize][0] > 320) cbufinc = dvp_cam_resolution[sensor.framesize][0] / 320;
                int x, y;
                for (y=0; y<dvp_cam_resolution[sensor.framesize][1]; y+=cbufinc) {
                    for (x=0; x<dvp_cam_resolution[sensor.framesize][0]; x += (cbufinc*2)) {
                        fb[(y/cbufinc*320) + (x/cbufinc)] = cbuf[(y*dvp_cam_resolution[sensor.framesize][0]) + x + 1];
                        fb[(y/cbufinc*320) + (x/cbufinc) + 1] = cbuf[(y*dvp_cam_resolution[sensor.framesize][0]) + x];
                    }
                }
                tft_frame_buffer = fb;
            }
            else {
                if (mode == PIXFORMAT_RGB565) {
                    tft_frame_buffer = (uint16_t *)frame_buffer;
                    memcpy(fb, tft_frame_buffer, sensor.gram_size);
                    tft_frame_buffer = fb;
                }
                else {
                    uint16_t clr;
                    for (int i=0; i<sensor.gram_size; i+=2) {
                        clr = (frame_buffer[i] & 0xf8) << 8;
                        clr |= (frame_buffer[i] & 0xfc) << 3;
                        clr |= (frame_buffer[i] & 0xf8) >> 3;
                        fb[i] = clr;
                    }
                }
            }
            send_frame_buffer();
        }

        for (int i= 0; i<1280; i++) {
            if ((i % 32) == 0) printf("\r\n%04x: ", i);
            printf("%02X ", frame_buffer[i]);
        }
        printf("\r\n");
    }
    else if (mode == PIXFORMAT_JPEG) {
        // jpeg data have swapped 4-byte values
        /*printf("Swap jpeg\r\n");
        for (int i= 0; i<32; i++) {
            printf("%02X ", frame_buffer[i]);
        }
        printf("\r\n");
        uint8_t tmpb0, tmpb1;
        for (int i = 0; i < sensor.gram_size; i+=4) {
            tmpb0 = frame_buffer[i];
            tmpb1 = frame_buffer[i+1];
            frame_buffer[i] = frame_buffer[i+3];
            frame_buffer[i+1] = frame_buffer[i+2];
            frame_buffer[i+2] = tmpb1;
            frame_buffer[i+3] = tmpb0;
        }*/
        printf("Find jpeg size\r\n");
        for (int i= 0; i<32; i++) {
            printf("%02X ", frame_buffer[i]);
        }
        printf("\r\n");
        uint32_t jpeg_buffer_idx = 0;
        if ((frame_buffer[0] == 0xff) && (frame_buffer[1] == 0xd8) && (memcmp(frame_buffer+6, "JFIF", 4) == 0)) {
            for (int i = 0; i < (sensor.gram_size-4); i++) {
                if ((frame_buffer[i] == 0xff) && (frame_buffer[i+1] == 0xd9)) {
                    printf("jpeg end marker at %d\r\n", i);
                    jpeg_buffer_idx = i+2;
                    break;
                }
            }
        }
        if (jpeg_buffer_idx > 0) {
            printf("jpeg -> string object\r\n");
            buffer = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)frame_buffer, jpeg_buffer_idx+2);
            printf("jpeg done.\r\n");
        }
        else {
            printf("jpeg error\r\n");
            if (buffer == mp_const_true) buffer = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)frame_buffer, sensor.gram_size);
        }
    }

    if (dvp_semaphore) vSemaphoreDelete(dvp_semaphore);
    dvp_semaphore = NULL;

    tft_frame_buffer = fb;
    if (buff_obj0 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj0);
    if (buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)buff_obj1);

    return buffer;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(test_camera_obj, 0, test_camera);


//------------------------
STATIC mp_obj_t test_mem()
{
    uint8_t *xbuf = (uint8_t *)0x40600000;
    for (int i= 0; i<256; i++) {
        xbuf[i] = i;
    }
    for (int i= 0; i<256; i++) {
        if ((i % 32) == 0) printf("\r\n");
        printf("%02X ", xbuf[i]);
    }
    printf("\r\n");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(test_mem_obj, test_mem);



//===========================================================
STATIC const mp_map_elem_t test_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_test) },

    { MP_ROM_QSTR(MP_QSTR_test_flash),      MP_ROM_PTR(&test_flash_obj) },
    { MP_ROM_QSTR(MP_QSTR_ftest),           MP_ROM_PTR(&ftest_obj) },
    { MP_ROM_QSTR(MP_QSTR_test),            MP_ROM_PTR(&test_obj) },
    { MP_ROM_QSTR(MP_QSTR_timer),           MP_ROM_PTR(&test_timer_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera),          MP_ROM_PTR(&test_camera_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem),             MP_ROM_PTR(&test_mem_obj) },
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
