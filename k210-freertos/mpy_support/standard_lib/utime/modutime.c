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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#include "encoding.h"
#include "sysctl.h"
#include "stdio.h"
#include "sleep.h"
#include "rtc.h"
#include <devices.h>
#include "syslog.h"
#include "lwip/apps/sntp.h"

#include "py/runtime.h"
#include "extmod/utime_mphal.h"
#include "lib/timeutils/timeutils.h"
#include "mphalport.h"

handle_t mp_rtc_rtc0;

static time_t time_base = 0;

//---------------------------------------------
void _set_sys_time(struct tm *tm_inf, int zone)
{
    int tz = 0;
    char tzs[16];

    // time zone
    if ((zone >= -11) && (zone <= 12)) {
        tz = zone * -1;
        if (tz >= 0) sprintf(tzs, "UTC+%d", tz);
        else sprintf(tzs, "UTC%d", tz);
    }
    else {
        sprintf(tzs, "UTC+0");
    }

    time_t seconds = mktime(tm_inf);
    seconds += ((tz * -1) * 3600);
    setenv("TZ", tzs, 1);
    //tzset();

    // Set system time
    rtc_set_datetime(mp_rtc_rtc0, tm_inf); // set RTC time

    /*
    struct timeval tv;
    struct timezone tzz;
    tv.tv_sec = seconds;
    tv.tv_usec = 0;
    tzz.tz_minuteswest = tz * 60;
    tzz.tz_dsttime = 0;
    settimeofday(&tv, &tzz);
    */

    time_t rtc_seconds;
    time(&rtc_seconds);                 // get system time
    time_base = seconds-rtc_seconds;    // set time correction variable
}

/*
 * The time can be get either using the POSIX 'time()' function
 * or reading the time from RTC timer
 * While testing which method is better, we use both
 * and all functions have an optional parameter to select which one to use
 * It looks like the system time runs too fast, so it is probably better
 * to use RTC timer
 */

//----------------------------
time_t _get_time(bool systm) {
    time_t seconds;
    if (systm) {
        time(&seconds); // get system time
        seconds += time_base;
    }
    else {
        struct tm now;
        rtc_get_datetime(mp_rtc_rtc0, &now);
        seconds = mktime(&now);
    }
    return seconds;
}

//--------------------------------------------------------------
STATIC mp_obj_t time_time(size_t n_args, const mp_obj_t *args) {
    bool systm = false;
    if (n_args > 0) {
        systm = mp_obj_is_true(args[0]);
    }
    return mp_obj_new_int(_get_time(systm));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(time_time_obj, 0, 1, time_time);

//------------------------------------------------------------------
static mp_obj_t _time_tuple(time_t sec_in, bool local, bool systm) {
    time_t seconds;
    struct tm *tm_info;

    if (sec_in >= 0) seconds = sec_in;
    else seconds = _get_time(systm);

    if (local) tm_info = localtime(&seconds);
    else tm_info = gmtime(&seconds);
    mp_obj_t tuple[8] = {
        mp_obj_new_int(tm_info->tm_year + 1900),
        mp_obj_new_int(tm_info->tm_mon + 1),
        mp_obj_new_int(tm_info->tm_mday),
        mp_obj_new_int(tm_info->tm_hour),
        mp_obj_new_int(tm_info->tm_min),
        mp_obj_new_int(tm_info->tm_sec),
        mp_obj_new_int(tm_info->tm_wday + 1),
        mp_obj_new_int(tm_info->tm_yday + 1)
    };
    return mp_obj_new_tuple(8, tuple);
}

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t time_localtime(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_secs,  MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_systm, MP_ARG_BOOL, { .u_bool = false } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    return _time_tuple(args[0].u_int, true, args[1].u_bool);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(time_localtime_obj, 0, time_localtime);

//------------------------------------------------------------------------------------------
STATIC mp_obj_t time_gmtime(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_secs,  MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_systm, MP_ARG_BOOL, { .u_bool = false } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    return _time_tuple(args[0].u_int, false, args[1].u_bool);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(time_gmtime_obj, 0, time_gmtime);

//------------------------------------------------------------------------------------------
STATIC mp_obj_t time_strftime(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_format, MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_time,                     MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_local,                    MP_ARG_OBJ,  { .u_bool = true } },
        { MP_QSTR_systm,                    MP_ARG_BOOL, { .u_bool = false } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *fmt = (char *)mp_obj_str_get_str(args[0].u_obj);
    char str_time[128];
    struct tm *tm_info;
    struct tm tm_inf;

    if (args[1].u_obj != mp_const_none) {
        mp_obj_t *time_items;

        mp_obj_get_array_fixed_n(args[1].u_obj, 8, &time_items);

        tm_inf.tm_year = mp_obj_get_int(time_items[0]) - 1900;
        tm_inf.tm_mon = mp_obj_get_int(time_items[1]) - 1;
        tm_inf.tm_mday = mp_obj_get_int(time_items[2]);
        tm_inf.tm_hour = mp_obj_get_int(time_items[3]);
        tm_inf.tm_min = mp_obj_get_int(time_items[4]);
        tm_inf.tm_sec = mp_obj_get_int(time_items[5]);
        tm_inf.tm_wday = mp_obj_get_int(time_items[6]) - 1;
        tm_inf.tm_yday = mp_obj_get_int(time_items[7]) - 1;
        tm_info = &tm_inf;
    }
    else {
        time_t seconds = _get_time(args[3].u_bool);
        if (args[2].u_bool) tm_info = localtime(&seconds);
        else tm_info = gmtime(&seconds);
    }

    strftime(str_time, 127, fmt, tm_info);

    return mp_obj_new_str(str_time, strlen(str_time));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(time_strftime_obj, 1, time_strftime);

//------------------------------------------------------------------------------------------
STATIC mp_obj_t time_mktime(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_time,   MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_tz,                       MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_setrtc,                   MP_ARG_BOOL, { .u_bool = false } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!mp_obj_is_type(args[0].u_obj, &mp_type_tuple)) {
        mp_raise_TypeError("expected time tuple");
    }

    struct tm tm_inf;
    mp_obj_t *time_items;
    size_t n_items = 0;
    int tz = 0;

    mp_obj_get_array(args[0].u_obj, &n_items, &time_items);
    if (n_items < 6) {
        mp_raise_ValueError("expected at least 6-item time tuple");
    }

    if (mp_obj_is_int(args[1].u_obj)) {
        // time zone
        tz = mp_obj_get_int(args[1].u_obj);
    }

    tm_inf.tm_year = mp_obj_get_int(time_items[0]) - 1900;
    tm_inf.tm_mon = mp_obj_get_int(time_items[1]) - 1;
    tm_inf.tm_mday = mp_obj_get_int(time_items[2]);
    tm_inf.tm_hour = mp_obj_get_int(time_items[3]);
    tm_inf.tm_min = mp_obj_get_int(time_items[4]);
    tm_inf.tm_sec = mp_obj_get_int(time_items[5]);
    tm_inf.tm_wday = 0;
    tm_inf.tm_yday = 0;

    if (args[2].u_bool) {
        // Set system time
        _set_sys_time(&tm_inf, tz);
    }

    time_t seconds = mktime(&tm_inf);
    return mp_obj_new_int(seconds);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(time_mktime_obj, 1, time_mktime);

/*
//------------------------------------------------------------------------------------------
STATIC mp_obj_t time_mktime(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_time,   MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_tz,                       MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_setrtc,                   MP_ARG_BOOL, { .u_bool = false } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!mp_obj_is_type(args[0].u_obj, &mp_type_tuple)) {
        mp_raise_TypeError("expected time tuple");
    }

    struct tm tm_inf;
    mp_obj_t *time_items;
    size_t n_items = 0;
    const char *tz = NULL;

    mp_obj_get_array(args[0].u_obj, &n_items, &time_items);
    if (n_items < 6) {
        mp_raise_ValueError("expected at least 6-item time tuple");
    }

    if (mp_obj_is_str(args[1].u_obj)) {
        tz = mp_obj_str_get_str(args[1].u_obj);
    }
    tm_inf.tm_year = mp_obj_get_int(time_items[0]) - 1900;
    tm_inf.tm_mon = mp_obj_get_int(time_items[1]) - 1;
    tm_inf.tm_mday = mp_obj_get_int(time_items[2]);
    tm_inf.tm_hour = mp_obj_get_int(time_items[3]);
    tm_inf.tm_min = mp_obj_get_int(time_items[4]);
    tm_inf.tm_sec = mp_obj_get_int(time_items[5]);
    tm_inf.tm_wday = 0;
    tm_inf.tm_yday = 0;

    time_t seconds = mktime(&tm_inf);
    if (args[2].u_bool) {
        // Set system time
        rtc_set_datetime(mp_rtc_rtc0, &tm_inf); // set RTC time

        time_t rtc_seconds;
        time(&rtc_seconds);             // get system time
        time_base = seconds-rtc_seconds;// set time correction variable

        if (tz) {
            // Set timezone, e.g. "CET-1CEST,M3.5.0,M10.5.0/3"
            setenv("TZ", tz, 1);
            //tzset();
        }
    }

    return mp_obj_new_int(seconds);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(time_mktime_obj, 1, time_mktime);
*/
//----------------------------------------------
STATIC mp_obj_t time_sleep_r(mp_obj_t seconds_o)
{
    mp_int_t res = _mp_hal_delay_us((mp_uint_t)(1000000 * mp_obj_get_float(seconds_o)));
    #if MICROPY_PY_BUILTINS_FLOAT
    return mp_obj_new_float((double)res / 1000000.0);
    #else
    return mp_obj_new_int(res / 1000000);
    #endif
}
MP_DEFINE_CONST_FUN_OBJ_1(mp_utime_sleep_r_obj, time_sleep_r);

//-------------------------------------------
STATIC mp_obj_t time_sleep_ms_r(mp_obj_t arg)
{
    mp_int_t ms = mp_obj_get_int(arg);
    if (ms > 0) {
        return mp_obj_new_int(_mp_hal_delay_us(ms * 1000) / 1000);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mp_utime_sleep_ms_r_obj, time_sleep_ms_r);

//-------------------------------------------
STATIC mp_obj_t time_sleep_us_r(mp_obj_t arg)
{
    mp_int_t us = mp_obj_get_int(arg);
    if (us > 0) {
        return mp_obj_new_int(_mp_hal_delay_us(us));
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mp_utime_sleep_us_r_obj, time_sleep_us_r);

//============================================================
STATIC const mp_rom_map_elem_t time_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_utime) },

    { MP_ROM_QSTR(MP_QSTR_time),        MP_ROM_PTR(&time_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_mktime),      MP_ROM_PTR(&time_mktime_obj) },
    { MP_ROM_QSTR(MP_QSTR_localtime),   MP_ROM_PTR(&time_localtime_obj) },
    { MP_ROM_QSTR(MP_QSTR_gmtime),      MP_ROM_PTR(&time_gmtime_obj) },
    { MP_ROM_QSTR(MP_QSTR_strftime),    MP_ROM_PTR(&time_strftime_obj) },

    { MP_ROM_QSTR(MP_QSTR_sleep),       MP_ROM_PTR(&mp_utime_sleep_obj) },
    { MP_ROM_QSTR(MP_QSTR_sleep_ms),    MP_ROM_PTR(&mp_utime_sleep_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_sleep_us),    MP_ROM_PTR(&mp_utime_sleep_us_obj) },

    { MP_ROM_QSTR(MP_QSTR_rsleep),      MP_ROM_PTR(&mp_utime_sleep_r_obj) },
    { MP_ROM_QSTR(MP_QSTR_rsleep_ms),   MP_ROM_PTR(&mp_utime_sleep_ms_r_obj) },
    { MP_ROM_QSTR(MP_QSTR_rsleep_us),   MP_ROM_PTR(&mp_utime_sleep_us_r_obj) },

    { MP_ROM_QSTR(MP_QSTR_ticks_ms),    MP_ROM_PTR(&mp_utime_ticks_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_us),    MP_ROM_PTR(&mp_utime_ticks_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_cpu),   MP_ROM_PTR(&mp_utime_ticks_cpu_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_add),   MP_ROM_PTR(&mp_utime_ticks_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_diff),  MP_ROM_PTR(&mp_utime_ticks_diff_obj) },
};

STATIC MP_DEFINE_CONST_DICT(time_module_globals, time_module_globals_table);

//====================================
const mp_obj_module_t utime_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&time_module_globals,
};

