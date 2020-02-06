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

#if MICROPY_USE_DISPLAY

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sysctl.h"
#include "syslog.h"

#include "moddisplay.h"
#include "modmachine.h"

#include "py/runtime.h"
#include "py/objstr.h"
#include "extmod/vfs.h"
#include "py/stream.h"
#include "lodepng.h"

display_settings_t tft_display_settings = {0};

//-------------------------------------------------------
static color_t rgb2color(uint8_t r, uint8_t g, uint8_t b)
{
    color_t color;
    color = (uint16_t)(r & 0xF8) << 8;
    color |= (uint16_t)(g & 0xFC) << 3;
    color |= (uint16_t)(b & 0xF8) >> 3;

    return color;
}

//-----------------------------------------
static color_t get_color(mp_obj_t color_in)
{
    if (mp_obj_is_int(color_in)) return mp_obj_get_int(color_in);

    if (!mp_obj_is_type(color_in, &mp_type_tuple)) {
        mp_raise_TypeError("Wrong color argument");
    }
    mp_obj_t *color_items;
    size_t n_items = 0;

    mp_obj_get_array(color_in, &n_items, &color_items);
    if (n_items != 3) {
        mp_raise_ValueError("expected 3-item color tuple");
    }
    color_t color = rgb2color(mp_obj_get_int(color_items[0]) & 0xFF, mp_obj_get_int(color_items[1]) & 0xFF, mp_obj_get_int(color_items[2]) & 0xFF);
    return color;
}

// Select currently active display device
//-------------------------------------------
static void setupDevice(mp_obj_t disp_dev_in)
{
    if mp_obj_is_type(disp_dev_in, &display_tft_type) {
        active_dstate = &tft_display_settings;
        active_dstate->tft_active_mode = TFT_MODE_TFT;
    }
    #if MICROPY_USE_EPD
    else if mp_obj_is_type(disp_dev_in, &display_epd_type) {
        display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(disp_dev_in);
        if (self->handle == 0) {
            mp_raise_ValueError("Device not initialized");
        }
        active_dstate = &epd_display_settings;
        active_dstate->tft_active_mode = TFT_MODE_EPD;
    }
    #endif
    #if MICROPY_USE_EVE
    else if mp_obj_is_type(disp_dev_in, &display_eve_type) {
        active_dstate = &eve_display_settings;
        tft_active_mode = TFT_MODE_EVE;
    }
    #endif
}

// constructor(id, ...)
//---------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    display_tft_obj_t *self = m_new_obj(display_tft_obj_t);
    self->base.type = &display_tft_type;

    return MP_OBJ_FROM_PTR(self);
}

//-----------------------------------------------------------------------------------------------
STATIC void display_tft_printinfo(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    display_tft_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_obj_array_t *fbuf = (mp_obj_array_t *)self->buff_obj0;
    char fb[32] = {'\0'};
    if (self->buff_obj0) sprintf(fb, "used (%s%lu B)", (self->buff_obj0) ? "2 x " : "", fbuf->len);
    mp_printf(print, "TFT Display (Speed=%u Hz, FB: %s)\r\n", self->dconfig.speed, fb);
}

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_type, ARG_width, ARG_height, ARG_speed, ARG_rot, ARG_bgr, ARG_splash, ARG_useFB };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_type,      MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = DISP_TYPE_ST7789V } },
        { MP_QSTR_width,     MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = DEFAULT_TFT_DISPLAY_WIDTH } },
        { MP_QSTR_height,    MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = DEFAULT_TFT_DISPLAY_HEIGHT } },
        { MP_QSTR_speed,     MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = SPI_DEFAULT_SPEED } },
        { MP_QSTR_rot,       MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_bgr,       MP_ARG_KW_ONLY  | MP_ARG_BOOL, { .u_bool = false } },
        { MP_QSTR_splash,    MP_ARG_KW_ONLY  | MP_ARG_BOOL, { .u_bool = true } },
        { MP_QSTR_useFB,     MP_ARG_KW_ONLY  | MP_ARG_BOOL, { .u_int = 1 } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    display_tft_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    int ret;

    if ((args[ARG_type].u_int < 0) || (args[ARG_type].u_int >= DISP_TYPE_MAX)) {
        mp_raise_ValueError("Unsupported display type");
    }

    active_dstate = &tft_display_settings;
    memset(active_dstate, 0, sizeof(display_settings_t));
    active_dstate->tft_active_mode = TFT_MODE_TFT;

    self->dconfig.type = args[ARG_type].u_int;

    self->dconfig.gamma = 0;
    self->dconfig.width = args[ARG_width].u_int;   // smaller dimension
    self->dconfig.height = args[ARG_height].u_int; // larger dimension
    self->dconfig.invrot = 1;
    self->buff_obj0 = mp_const_none;
    self->buff_obj1 = mp_const_none;
    self->active_fb = 0;

    self->dconfig.bgr = args[ARG_bgr].u_bool ? 0x08 : 0;

    int orient = LANDSCAPE;
    if ((args[ARG_rot].u_int >= 0) && (args[ARG_rot].u_int <= 3)) orient = args[ARG_rot].u_int;

    tft_display_settings.tft_active_mode = TFT_MODE_TFT;
    // ================================
    // ==== Initialize the Display ====

    ret = TFT_display_init(&self->dconfig);
    if (ret != 0) {
        mp_raise_msg(&mp_type_OSError, "Error initializing display");
    }

    active_dstate->use_frame_buffer = false;

    tft_set_speed(SPI_DEFAULT_SPEED);
    self->dconfig.speed = tft_get_speed();

    // Set requested speed
    uint32_t speed = args[ARG_speed].u_int;
    if (speed <= 24) speed *= 1000000;
    if ((speed < 1000000) || (speed > 24000000)) speed = SPI_DEFAULT_SPEED;
    if ((sysctl_clock_get_freq(SYSCTL_CLOCK_CPU) < 200000000) && (speed > SPI_DEFAULT_SPEED)) speed = SPI_DEFAULT_SPEED;
    self->dconfig.speed = speed;
    tft_set_speed(speed);
    self->dconfig.speed = tft_get_speed();

    // Set frame buffer(s)
    active_dstate->_tft_frame_buffer = NULL;
    active_dstate->tft_frame_buffer = NULL;
    active_dstate->use_frame_buffer = (args[ARG_useFB].u_int > 0);
    if (active_dstate->use_frame_buffer) {
        self->buff_obj0 = mp_obj_new_frame_buffer((active_dstate->_width * active_dstate->_height * 2) + 8);
        if (self->buff_obj0 == mp_const_none) active_dstate->use_frame_buffer = false;
        else if (args[ARG_useFB].u_int > 1) {
            self->buff_obj1 = mp_obj_new_frame_buffer((active_dstate->_width * active_dstate->_height * 2) + 8);
        }
        mp_obj_array_t *fbuf = (mp_obj_array_t *)self->buff_obj0;
        active_dstate->_tft_frame_buffer = fbuf->items;
        active_dstate->tft_frame_buffer = (uint16_t *)(active_dstate->_tft_frame_buffer + 8);
    }
    vTaskDelay(200);

    active_dstate->font_rotate = 0;
    active_dstate->text_wrap = 0;
    active_dstate->font_transparent = 0;
    active_dstate->font_forceFixed = 0;
    active_dstate->gray_scale = 0;
    TFT_setRotation(orient);
    TFT_setGammaCurve(0);
    TFT_setFont(DEJAVU18_FONT, 0, false);
    TFT_resetclipwin();
    TFT_fillScreen(0x2104);

    if (args[ARG_splash].u_bool) {
        active_dstate->_bg = 0x2104;
        TFT_drawRect(0, 0, active_dstate->_width, active_dstate->_height, TFT_CYAN);
        int fhight = TFT_getfontheight();
        active_dstate->_fg = TFT_GREEN;
        TFT_print("MicroPython", CENTER, (active_dstate->_height/2) - (fhight/2));
        active_dstate->_fg = TFT_BLUE;
        TFT_print("MicroPython", CENTER, (active_dstate->_height/2) + (fhight/2));
        active_dstate->_fg = TFT_RED;
        TFT_print("MicroPython", CENTER, (active_dstate->_height/2) - fhight - (fhight/2));
        active_dstate->_fg = TFT_GREEN;
        active_dstate->_bg = TFT_BLACK;
    }
    vTaskDelay(500);
    if (active_dstate->use_frame_buffer) send_frame_buffer();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_init_obj, 0, display_tft_init);

//--------------------------------------------------
STATIC mp_obj_t display_tft_deinit(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("Not implemented");
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(display_tft_deinit_obj, display_tft_deinit);

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawPixel(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_color,               MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    if (args[2].u_obj != mp_const_none) color = get_color(args[2].u_obj);

    TFT_drawPixel(x, y, color);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawPixel_obj, 2, display_tft_drawPixel);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawLine(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x1,    MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1,    MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_color,                   MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x0 = args[0].u_int;
    mp_int_t y0 = args[1].u_int;
    mp_int_t x1 = args[2].u_int;
    mp_int_t y1 = args[3].u_int;
    if (args[4].u_obj != mp_const_none) color = get_color(args[4].u_obj);

    TFT_drawLine(x0, y0, x1, y1, color);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawLine_obj, 4, display_tft_drawLine);

//-----------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawLineByAngle(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_start,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_length, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_angle,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t start = args[2].u_int;
    mp_int_t len = args[3].u_int;
    mp_int_t angle = args[4].u_int;
    if (args[5].u_obj != mp_const_none) color = get_color(args[5].u_obj);

    TFT_drawLineByAngle(x, y, start, len, angle, color);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawLineByAngle_obj, 5, display_tft_drawLineByAngle);

//--------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawTriangle(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x1,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x2,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y2,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_fillcolor,                MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x0 = args[0].u_int;
    mp_int_t y0 = args[1].u_int;
    mp_int_t x1 = args[2].u_int;
    mp_int_t y1 = args[3].u_int;
    mp_int_t x2 = args[4].u_int;
    mp_int_t y2 = args[5].u_int;
    if (args[6].u_obj != mp_const_none) color = get_color(args[6].u_obj);
    if (args[7].u_obj != mp_const_none) {
        TFT_fillTriangle(x0, y0, x1, y1, x2, y2, get_color(args[7].u_obj));
    }
    TFT_drawTriangle(x0, y0, x1, y1, x2, y2, color);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawTriangle_obj, 6, display_tft_drawTriangle);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawCircle(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_r,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_fillcolor,                MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t radius = args[2].u_int;
    if (args[3].u_obj != mp_const_none) color = get_color(args[3].u_obj);
    if (args[4].u_obj != mp_const_none) {
        TFT_fillCircle(x, y, radius, get_color(args[4].u_obj));
        if (args[3].u_int != args[4].u_int) TFT_drawCircle(x, y, radius, color);
    }
    else TFT_drawCircle(x, y, radius, color);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawCircle_obj, 3, display_tft_drawCircle);

//-------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawEllipse(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_rx,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_ry,     MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_opt,                      MP_ARG_INT, { .u_int = 15 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_fillcolor,                MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t rx = args[2].u_int;
    mp_int_t ry = args[3].u_int;
    mp_int_t opt = args[4].u_int & 0x0F;
    if (args[5].u_obj != mp_const_none) color = get_color(args[5].u_obj);
    if (args[6].u_obj != mp_const_none) {
        TFT_fillEllipse(x, y, rx, ry, get_color(args[6].u_obj), opt);
    }
    TFT_drawEllipse(x, y, rx, ry, color, opt);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawEllipse_obj, 4, display_tft_drawEllipse);

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawArc(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_r,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_thick,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_start,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_end,    MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 15 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_fillcolor,                MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    color_t fill_color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t r = args[2].u_int;
    mp_int_t th = args[3].u_int;
    mp_int_t start = args[4].u_int;
    mp_int_t end = args[5].u_int;
    if (args[6].u_obj != mp_const_none) color = get_color(args[6].u_obj);
    if (args[7].u_obj != mp_const_none) fill_color = get_color(args[7].u_obj);
    TFT_drawArc(x, y, r, th, start, end, color, fill_color);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawArc_obj, 6, display_tft_drawArc);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawPoly(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_r,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_sides,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_thick,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 1 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_fillcolor,                MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_rotate,                   MP_ARG_INT, { .u_int = 0 } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    color_t fill_color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t r = args[2].u_int;
    mp_int_t sides = args[3].u_int;
    mp_int_t th = args[4].u_int;
    if (args[5].u_obj != mp_const_none) color = get_color(args[5].u_obj);
    if (args[6].u_obj != mp_const_none) fill_color = get_color(args[6].u_obj);
    TFT_drawPolygon(x, y, sides, r, color, fill_color, args[7].u_int, th);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawPoly_obj, 5, display_tft_drawPoly);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawRect(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_width,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_height, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_fillcolor,                MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t w = args[2].u_int;
    mp_int_t h = args[3].u_int;
    if (args[4].u_obj != mp_const_none) color = get_color(args[4].u_obj);
    if (args[5].u_obj != mp_const_none) {
        TFT_fillRect(x, y, w, h, get_color(args[5].u_obj));
    }
    TFT_drawRect(x, y, w, h, color);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawRect_obj, 4, display_tft_drawRect);

//-------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_writeScreen(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_width,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_height, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_buffer, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t w = args[2].u_int;
    mp_int_t h = args[3].u_int;

    // clipping
    if ((x >= active_dstate->_width) || (y > active_dstate->_height)) {
        mp_raise_ValueError("Point (x,y) outside the display area");
    }

    if (x < 0) {
        w -= (0 - x);
        x = 0;
    }
    if (y < 0) {
        h -= (0 - y);
        y = 0;
    }
    if (w < 0) w = 0;
    if (h < 0) h = 0;

    if ((x + w) > (active_dstate->_width+1)) w = active_dstate->_width - x + 1;
    if ((y + h) > (active_dstate->_height+1)) h = active_dstate->_height - y + 1;
    if (w == 0) w = 1;
    if (h == 0) h = 1;

    int clr_len = h*w;
    int buf_len = (clr_len*3) + 1;

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[4].u_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len != buf_len) {
        mp_raise_ValueError("Wrong buffer length");
    }

    send_data(x, y, x+w+1, y+h+1, (uint32_t)clr_len, (color_t *)bufinfo.buf + 1);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_writeScreen_obj, 5, display_tft_writeScreen);

//---------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_drawRoundRect(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_width,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_height, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_r,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_fillcolor,                MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_fg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    mp_int_t w = args[2].u_int;
    mp_int_t h = args[3].u_int;
    mp_int_t r = args[4].u_int;
    if (args[5].u_obj != mp_const_none) color = get_color(args[5].u_obj);
    if (args[6].u_obj != mp_const_none) {
        TFT_fillRoundRect(x, y, w, h, r, get_color(args[6].u_obj));
    }
    TFT_drawRoundRect(x, y, w, h, r, color);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_drawRoundRect_obj, 5, display_tft_drawRoundRect);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_fillScreen(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_bg;
    if (args[0].u_obj != mp_const_none) color = get_color(args[0].u_obj);
    TFT_fillScreen(color);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_fillScreen_obj, 0, display_tft_fillScreen);

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_fillWin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_color,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t color = active_dstate->_bg;
    if (args[0].u_obj != mp_const_none) color = get_color(args[0].u_obj);
    TFT_fillWindow(color);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_fillWin_obj, 0, display_tft_fillWin);

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_setFont(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_font, ARG_rot, ARG_transp, ARG_fixedw,ARG_info };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_font,         MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_rotate,       MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_transparent,  MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_fixedwidth,   MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_info,         MP_ARG_KW_ONLY  | MP_ARG_BOOL, { .u_bool = false } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t font = DEFAULT_FONT;
    mp_obj_t fontf = mp_const_none;

    if (mp_obj_is_str(args[ARG_font].u_obj)) {
        font = USER_FONT;
        fontf = args[ARG_font].u_obj;
    }
    else {
        font = mp_obj_get_int(args[ARG_font].u_obj);
    }
    bool res = TFT_setFont(font, fontf, args[ARG_info].u_bool);

    if (args[ARG_rot].u_int >= 0) active_dstate->font_rotate = args[ARG_rot].u_int;
    if (args[ARG_transp].u_int >= 0) active_dstate->font_transparent = args[ARG_transp].u_int & 1;
    if (args[ARG_fixedw].u_int >= 0) active_dstate->font_forceFixed = args[ARG_fixedw].u_int & 1;

    return (res) ? mp_const_true : mp_const_false;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_setFont_obj, 1, display_tft_setFont);



//-------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_getFont(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    setupDevice(pos_args[0]);

    uint8_t type = 0;
    mp_obj_t tuple[2];

    tuple[0] = mp_obj_new_int(getFontInfo(&type));
    tuple[1] = mp_obj_new_int(type);

    return mp_obj_new_tuple(2, tuple);
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_getFont_obj, 0, display_tft_getFont);

//-------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_getFontSize(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    setupDevice(pos_args[0]);
    int width, height;
    TFT_getfontsize(&width, &height);

    mp_obj_t tuple[2];

    tuple[0] = mp_obj_new_int(width);
    tuple[1] = mp_obj_new_int(height);

    return mp_obj_new_tuple(2, tuple);
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_getFontSize_obj, 0, display_tft_getFontSize);

//--------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_setRot(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_rot, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = -1 } },
    };
    setupDevice(pos_args[0]);
    display_tft_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t rot = LANDSCAPE;
    if (args[0].u_int >= 0) rot = args[0].u_int;
    if (rot > 255) rot = 0;

    TFT_setRotation(rot);
    self->dconfig.width = active_dstate->_width;
    self->dconfig.height = active_dstate->_height;

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_setRot_obj, 1, display_tft_setRot);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_7segAttrib(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_width, ARG_height, ARG_bw, ARG_space, ARG_color };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_width,     MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_height,    MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_bar_width, MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_space,     MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_color,     MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (active_dstate->cfont.bitmap == 2) {
        mp_int_t width = active_dstate->cfont.x_size;
        mp_int_t height = active_dstate->cfont.y_size;
        mp_int_t bwidth = active_dstate->cfont.offset;
        mp_int_t space = active_dstate->cfont.numchars;
        mp_int_t color = active_dstate->cfont.color;
        if (args[ARG_width].u_int >= 0) width = args[ARG_width].u_int;
        if (args[ARG_height].u_int >= 0) height = args[ARG_height].u_int;
        if (args[ARG_bw].u_int >= 0) bwidth = args[ARG_bw].u_int;
        if (args[ARG_space].u_int >= 0) space = args[ARG_space].u_int;
        if (args[ARG_color].u_obj != mp_const_none) color = get_color(args[ARG_color].u_obj);

        set_7seg_font_atrib(width, height, bwidth, space, color);
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_7segAttrib_obj, 1, display_tft_7segAttrib);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_vectAttrib(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_scale, ARG_rot, ARG_grot, ARG_haspect, ARG_vaspect, ARG_bcolor, ARG_bsize, ARG_btype, ARG_bstep, ARG_shrx, ARG_shry, ARG_pad, ARG_def };
    const mp_arg_t allowed_args[] = {
            { MP_QSTR_scale,        MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_rotate,       MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_grotate,      MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_haspect,      MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_vaspect,      MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_bcolor,       MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_bsize,        MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_btype,        MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_bstep,        MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_shearx,       MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_sheary,       MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_padding,      MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_default,      MP_ARG_KW_ONLY  | MP_ARG_BOOL,{ .u_bool= false } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (active_dstate->cfont.bitmap == 0) {
        mp_obj_t tuple[12];
        if (args[ARG_def].u_bool) {
            // Set defaults
            setAspect(&active_dstate->cfont.context, 1.0f, 1.0f);
            setGlyphPadding(&active_dstate->cfont.context, 4.0f);
            setGlyphScale(&active_dstate->cfont.context, 1.0f);
            setBrush(&active_dstate->cfont.context, BRUSH_DISK);
            setBrushSize(&active_dstate->cfont.context, 1.0f);
            setBrushStep(&active_dstate->cfont.context, 10.0f);
            setBrushColor(&active_dstate->cfont.context, TFT_CYAN);
            setRenderFilter(&active_dstate->cfont.context, RENDEROP_NONE);
            setRotationAngle(&active_dstate->cfont.context, 0.0f, 0.0f);
            goto exit;
        }
        bool f;
        float haspect = active_dstate->cfont.context.scale.horizontal;
        float vaspect = active_dstate->cfont.context.scale.vertical;
        float grot = active_dstate->cfont.context.rotate.glyph.angle;
        float rot = active_dstate->cfont.context.rotate.string.angle;
        float shearx = active_dstate->cfont.context.shear.angleX;
        float sheary = active_dstate->cfont.context.shear.angleY;
        uint32_t render = RENDEROP_NONE;

        if (args[ARG_scale].u_obj != mp_const_none) {
            float scale = mp_obj_get_float(args[ARG_scale].u_obj);
            if ((scale >= 0.1f) && (scale <= 10.0f)) setGlyphScale(&active_dstate->cfont.context, scale);
        }

        f = false;
        if (args[ARG_haspect].u_obj != mp_const_none) {
            haspect = mp_obj_get_float(args[ARG_haspect].u_obj);
            if ((haspect >= 0.05f) || (haspect <= -0.05f)) f = true;
        }
        if (args[ARG_vaspect].u_obj != mp_const_none) {
            vaspect = mp_obj_get_float(args[ARG_vaspect].u_obj);
            if ((vaspect >= 0.05f) || (vaspect <= -0.05f)) f = true;
        }
        if (f) setAspect(&active_dstate->cfont.context, haspect, vaspect);

        if (args[ARG_bcolor].u_obj != mp_const_none) setBrushColor(&active_dstate->cfont.context, get_color(args[ARG_bcolor].u_obj));

        if (args[ARG_rot].u_obj != mp_const_none) rot = mp_obj_get_float(args[ARG_rot].u_obj);
        if (args[ARG_grot].u_obj != mp_const_none) grot = mp_obj_get_float(args[ARG_grot].u_obj);
        if ((rot >= 0.0f) || (grot >= 0.0f)) {
            if (grot > 0.0f) render |= RENDEROP_ROTATE_GLYPHS;
            if (rot > 0.0f) render |= RENDEROP_ROTATE_STRING;
            setRotationAngle(&active_dstate->cfont.context, grot, rot);
        }

        if (args[ARG_bsize].u_obj != mp_const_none) setBrushSize(&active_dstate->cfont.context, mp_obj_get_float(args[ARG_bsize].u_obj));
        if (args[ARG_btype].u_obj != mp_const_none) {
            int btype = mp_obj_get_int(args[ARG_btype].u_obj);
            if ((btype >= 0) && (btype < BRUSH_TOTAL)) setBrush(&active_dstate->cfont.context, btype);
        }
        if (args[ARG_bstep].u_obj != mp_const_none) {
            float step = mp_obj_get_float(args[ARG_bstep].u_obj);
            setBrushStep(&active_dstate->cfont.context, step);
        }

        f = false;
        if (args[ARG_shrx].u_obj != mp_const_none) {
            shearx = mp_obj_get_float(args[ARG_shrx].u_obj);
            if ((shearx >= 0.0f) && (shearx <= 360.0f)) {
                f = true;
                render |= RENDEROP_SHEAR_X;
            }
        }
        if (args[ARG_shry].u_obj != mp_const_none) {
            sheary = mp_obj_get_float(args[ARG_shry].u_obj);
            if ((sheary >= 0.0f) && (sheary <= 360.0f)) {
                f = true;
                render |= RENDEROP_SHEAR_Y;
            }
        }
        if (f) setShearAngle(&active_dstate->cfont.context, shearx, sheary);

        if (args[ARG_pad].u_obj != mp_const_none) {
            float pad = mp_obj_get_float(args[ARG_pad].u_obj);
            if ((pad >= -0.5f) && (pad <= 50.0f)) setGlyphPadding(&active_dstate->cfont.context, pad);
        }

        setRenderFilter(&active_dstate->cfont.context, render);
exit:
        tuple[0] = mp_obj_new_float(active_dstate->cfont.context.scale.glyph);
        tuple[1] = mp_obj_new_float(active_dstate->cfont.context.scale.horizontal);
        tuple[2] = mp_obj_new_float(active_dstate->cfont.context.scale.vertical);
        tuple[3] = mp_obj_new_float(active_dstate->cfont.context.rotate.string.angle);
        tuple[4] = mp_obj_new_float(active_dstate->cfont.context.rotate.glyph.angle);
        tuple[5] = mp_obj_new_float(active_dstate->cfont.context.shear.angleX);
        tuple[6] = mp_obj_new_float(active_dstate->cfont.context.shear.angleY);
        tuple[7] = mp_obj_new_float(active_dstate->cfont.context.brush.size);
        tuple[8] = mp_obj_new_int(active_dstate->cfont.context.brush.type);
        tuple[9] = mp_obj_new_float(active_dstate->cfont.context.brush.step);
        tuple[10] = mp_obj_new_int(active_dstate->cfont.context.brush.color);
        tuple[11] = mp_obj_new_float(active_dstate->cfont.context.xypad);

        return mp_obj_new_tuple(12, tuple);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_vectAttrib_obj, 1, display_tft_vectAttrib);

//-------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_print(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_x, ARG_y, ARG_text, ARG_color, ARG_bgcolor, ARG_rotate, ARG_transp, ARG_fixedw, ARG_wrap };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,            MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,            MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_text,         MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_color,                          MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_bgcolor,                        MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_rotate,       MP_ARG_KW_ONLY  | MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_transparent,  MP_ARG_KW_ONLY  | MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_fixedwidth,   MP_ARG_KW_ONLY  | MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_wrap,         MP_ARG_KW_ONLY  | MP_ARG_INT, { .u_int = -1 } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x = args[ARG_x].u_int;
    mp_int_t y = args[ARG_y].u_int;
    unsigned char *st = (unsigned char *)mp_obj_str_get_str(args[ARG_text].u_obj);

    if ((x <= CENTER) || (y <= CENTER)) {
        // Aligned position requested
        if ((active_dstate->cfont.bitmap != 0) && (active_dstate->font_rotate != 0)) {
            mp_raise_ValueError("Rotated strings cannot be aligned");
        }
        if ((active_dstate->cfont.bitmap == 0) &&
            ((active_dstate->cfont.context.rotate.string.angle != 0.0f) ||
             (active_dstate->cfont.context.rotate.glyph.angle != 0.0f))) {
            mp_raise_ValueError("Rotated strings cannot be aligned");
        }
    }

    color_t old_fg = active_dstate->_fg;
    color_t old_bg = active_dstate->_bg;
    int old_rot = active_dstate->font_rotate;
    int old_transp = active_dstate->font_transparent;
    int old_fixed = active_dstate->font_forceFixed;
    int old_wrap = active_dstate->text_wrap;

    if (args[ARG_color].u_obj != mp_const_none) active_dstate->_fg = get_color(args[ARG_color].u_obj);
    if (args[ARG_bgcolor].u_obj != mp_const_none) active_dstate->_bg = get_color(args[ARG_bgcolor].u_obj);
    if (args[ARG_rotate].u_int >= 0) active_dstate->font_rotate = args[ARG_rotate].u_int;
    if (args[ARG_transp].u_int >= 0) active_dstate->font_transparent = args[ARG_transp].u_int & 1;
    if (args[ARG_fixedw].u_int >= 0) active_dstate->font_forceFixed = args[ARG_fixedw].u_int & 1;
    if (args[ARG_wrap].u_int >= 0) active_dstate->text_wrap = args[ARG_wrap].u_int & 1;

    if (active_dstate->cfont.bitmap == 0) {
        // Vector font
        if ((x <= CENTER) || (x >= LASTX) || (y <= CENTER) || (y >= LASTX)) {
            // Set x & y for special coordinates codes

            int tmpw, tmph;
            TFT_getStringSize((char *)st, &tmpw, &tmph);
            box_t box;
            getStringMetrics(&active_dstate->cfont.context, st, &box);

            // Calculate coordinates for LASTX and/or LASTY
            if ((x >= LASTX) && (x < LASTY)) x = active_dstate->TFT_X + (x-LASTX);
            if (y >= LASTY) y = active_dstate->TFT_Y + (y-LASTY);

            // Calculate CENTER, RIGHT or BOTTOM position
            if (x == RIGHT) x = active_dstate->dispWin.x2 - tmpw + active_dstate->dispWin.x1;
            else if (x == CENTER) x = (((active_dstate->dispWin.x2 - active_dstate->dispWin.x1 + 1) - tmpw) / 2) + active_dstate->dispWin.x1;

            if (y == BOTTOM) y = active_dstate->dispWin.y2 - tmph + active_dstate->dispWin.y1;
            else if (y==CENTER) y = (((active_dstate->dispWin.y2 - active_dstate->dispWin.y1 + 1) - (tmph/2)) / 2) + active_dstate->dispWin.y1;
        }
        else {
            x += active_dstate->dispWin.x1;
            y += active_dstate->dispWin.y1;
        }
        // Check string position is on screen
        if (x < active_dstate->dispWin.x1) x = active_dstate->dispWin.x1;
        if (y < active_dstate->dispWin.y1) y = active_dstate->dispWin.y1;
        if ((x > active_dstate->dispWin.x2) || (y > active_dstate->dispWin.y2)) {
            active_dstate->_fg = old_fg;
            active_dstate->_bg = old_bg;
            active_dstate->font_rotate = old_rot;
            active_dstate->font_transparent = old_transp;
            active_dstate->font_forceFixed = old_fixed;
            active_dstate->text_wrap = old_wrap;
            mp_raise_ValueError("x or y out of display window");
        }

        vfontDrawString(&active_dstate->cfont.context, st, x, y);
    }
    else {
        TFT_print((char *)st, x, y);
    }

    active_dstate->_fg = old_fg;
    active_dstate->_bg = old_bg;
    active_dstate->font_rotate = old_rot;
    active_dstate->font_transparent = old_transp;
    active_dstate->font_forceFixed = old_fixed;
    active_dstate->text_wrap = old_wrap;

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_print_obj, 3, display_tft_print);

//-------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_stringWidth(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_text,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *st = (char *)mp_obj_str_get_str(args[0].u_obj);

    mp_int_t w = TFT_getStringWidth(st);

    return mp_obj_new_int(w);
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_stringWidth_obj, 1, display_tft_stringWidth);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_stringSize(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_text,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *st = (char *)mp_obj_str_get_str(args[0].u_obj);

    int w, h;
    TFT_getStringSize(st, &w, &h);
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(w);
    tuple[1] = mp_obj_new_int(h);

    return mp_obj_new_tuple(2, tuple);
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_stringSize_obj, 1, display_tft_stringSize);

//-----------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_clearStringRect(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,       MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,       MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_text,    MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_color,                     MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    color_t old_bg = active_dstate->_bg;
    mp_int_t x = args[0].u_int;
    mp_int_t y = args[1].u_int;
    char *st = (char *)mp_obj_str_get_str(args[2].u_obj);

    if (args[3].u_obj != mp_const_none) active_dstate->_bg = get_color(args[3].u_obj);

    TFT_clearStringRect(x, y, st);

    active_dstate->_bg = old_bg;

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_clearStringRect_obj, 3, display_tft_clearStringRect);

//-------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_Image(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_x, ARG_y, ARG_file, ARG_scale, ARG_type, ARG_debug };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_file,   MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_scale,                    MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_type,                     MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_debug,  MP_ARG_KW_ONLY  | MP_ARG_INT, { .u_int = 0 } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const char *filename = NULL;
    mp_buffer_info_t bufinfo;
    bool is_buffer = false;
    if (!mp_obj_is_str(args[ARG_file].u_obj)) {
        if (mp_obj_is_type(args[ARG_file].u_obj, &mp_type_bytes)) {
            mp_get_buffer_raise(args[ARG_file].u_obj, &bufinfo, MP_BUFFER_READ);
            if (bufinfo.len < 16) {
                mp_raise_ValueError("Wrong source size");
            }
            is_buffer = true;
        }
        else mp_raise_msg(&mp_type_OSError, "File name or image buffer expected");
    }
    else {
        filename = mp_obj_str_get_str(args[ARG_file].u_obj);
    }

    int img_type = args[ARG_type].u_int;
    int x = args[ARG_x].u_int;
    int y = args[ARG_y].u_int;

    if ((img_type <= IMAGE_TYPE_MIN) || (img_type >= IMAGE_TYPE_MAX)) {
        if (!is_buffer) {
            // try to determine file image type
            char *fname = (char *)mp_obj_str_get_str(args[ARG_file].u_obj);
            char upr_fname[strlen(fname)+1];
            strcpy(upr_fname, fname);
            for (int i=0; i < strlen(upr_fname); i++) {
              upr_fname[i] = toupper((unsigned char) upr_fname[i]);
            }
            if (strstr(upr_fname, ".JPG") != NULL) img_type = IMAGE_TYPE_JPG;
            else if (strstr(upr_fname, ".BMP") != NULL) img_type = IMAGE_TYPE_BMP;
            else if (strstr(upr_fname, ".RAW") != NULL) img_type = IMAGE_TYPE_RAW;
            else if (strstr(upr_fname, ".PNG") != NULL) img_type = IMAGE_TYPE_PNG;
            else {
                // Open the file and try to determine image type
                mp_obj_t fargs[2];
                fargs[0] = args[ARG_file].u_obj;
                fargs[1] = mp_obj_new_str("rb", 2);
                mp_obj_t ffd = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);
                if (ffd) {
                    uint8_t buf[16] = {0};
                    if ( mp_stream_posix_read((void *)ffd, buf, 11) == 11) {
                        buf[10] = 0;
                        if (strstr((char *)(buf+6), "JFIF") != NULL) img_type = IMAGE_TYPE_JPG;
                        else if ((buf[0] == 0x42) && (buf[1] == 0x4d)) img_type = IMAGE_TYPE_BMP;
                        else if (memcmp(buf+4, "rawB", 4) == 0) img_type = IMAGE_TYPE_RAW;
                        else if ((buf[0] == 0x89) && (memcmp(buf+1, "PNG", 3) == 0)) img_type = IMAGE_TYPE_PNG;
                    }
                    mp_stream_close(ffd);
                }
            }
        }
        else {
            // try to determine buffer image type
            uint8_t buf[16] = {0};
            memcpy(buf, bufinfo.buf, 15);
            if (strstr((char *)(buf+6), "JFIF") != NULL) img_type = IMAGE_TYPE_JPG;
            else if ((buf[0] == 0x42) && (buf[1] == 0x4d)) img_type = IMAGE_TYPE_BMP;
            else if (memcmp(buf+4, "rawB", 4) == 0) img_type = IMAGE_TYPE_RAW;
            else if ((buf[0] == 0x89) && (memcmp(buf+1, "PNG", 3) == 0)) img_type = IMAGE_TYPE_PNG;
        }
    }
    if ((img_type <= IMAGE_TYPE_MIN) || (img_type >= IMAGE_TYPE_MAX)) {
        mp_raise_msg(&mp_type_OSError, "Cannot determine image type");
    }

    active_dstate->image_debug = (uint8_t)args[ARG_debug].u_bool;

    if (img_type == IMAGE_TYPE_BMP) {
        if (active_dstate->tft_active_mode == TFT_MODE_EPD) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "BMP not supported on EPD"));
        }
        if (is_buffer) TFT_bmp_image(x, y, args[ARG_scale].u_int, mp_const_none, (uint8_t *)bufinfo.buf, bufinfo.len);
        else TFT_bmp_image(x, y, args[ARG_scale].u_int, args[ARG_file].u_obj, NULL, 0);
    }
    else if (img_type == IMAGE_TYPE_JPG) {
        if (is_buffer) TFT_jpg_image(x, y, args[ARG_scale].u_int, mp_const_none, (uint8_t *)bufinfo.buf, bufinfo.len);
        else TFT_jpg_image(x, y, args[ARG_scale].u_int, args[ARG_file].u_obj, NULL, 0);
    }
    else if (img_type == IMAGE_TYPE_PNG) {
        if (active_dstate->tft_active_mode == TFT_MODE_EPD) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "PNG not supported on EPD"));
        }
        if (is_buffer) TFT_png_image(x, y, args[ARG_scale].u_int, NULL, (uint8_t *)bufinfo.buf, bufinfo.len);
        else TFT_png_image(x, y, args[ARG_scale].u_int, filename, NULL, 0);
    }
    else if (img_type == IMAGE_TYPE_RAW) {
        if (active_dstate->tft_active_mode == TFT_MODE_EPD) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "RAW not supported on EPD"));
        }
        uint16_t width = 0;
        uint16_t height = 0;
        if (is_buffer) {
            if (memcmp(bufinfo.buf+4, "rawB", 4) != 0) {
                mp_raise_ValueError("Not a valid raw image");
            }
            width = *(uint16_t *)bufinfo.buf;
            height = *(uint16_t *)(bufinfo.buf+2);
            if ((bufinfo.len-8) != (width*height*2)) {
                mp_raise_ValueError("Wrong source size");
            }

            send_data_scale(x, y, width, height, (color_t *)(bufinfo.buf+8), args[ARG_scale].u_int);
        }
        else {
            // Open the file
            char rawb_id[4] = {0};
            mp_obj_t fargs[2];
            fargs[0] = args[ARG_file].u_obj;
            fargs[1] = mp_obj_new_str("rb", 2);
            mp_obj_t ffd = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);
            if (ffd) {
                int res;
                // Get file size
                int fsize = mp_stream_posix_lseek((void *)ffd, 0, SEEK_END);
                int at_start = mp_stream_posix_lseek((void *)ffd, 0, SEEK_SET);
                if ((fsize < 16) || (at_start != 0)) {
                    mp_stream_close(ffd);
                    mp_raise_ValueError("Wrong file size");
                }
                // Get width and height
                res = mp_stream_posix_read((void *)ffd, &width, 2);
                if (res != 2) {
                    mp_stream_close(ffd);
                    mp_raise_msg(&mp_type_OSError, "Error reading file");
                }
                res = mp_stream_posix_read((void *)ffd, &height, 2);
                if (res != 2) {
                    mp_stream_close(ffd);
                    mp_raise_msg(&mp_type_OSError, "Error reading file");
                }
                res = mp_stream_posix_read((void *)ffd, rawb_id, 4);
                if (res != 4) {
                    mp_stream_close(ffd);
                    mp_raise_msg(&mp_type_OSError, "Error reading file");
                }
                if (memcmp(rawb_id, "rawB", 4) != 0) {
                    mp_stream_close(ffd);
                    mp_raise_ValueError("not a valid raw image");
                }
                if ((fsize-8) != (width*height*2)) {
                    mp_stream_close(ffd);
                    mp_raise_ValueError("Wrong file size");
                }

                if ((x == 0) && (y == 0) && (width == active_dstate->_width) && (height == active_dstate->_height)) {
                    // Full frame buffer
                    fsize = mp_stream_posix_read((void *)ffd, active_dstate->tft_frame_buffer, fsize-8);
                    if (fsize != (active_dstate->_width * active_dstate->_height*2)) {
                        mp_stream_close(ffd);
                        mp_raise_msg(&mp_type_OSError, "Error reading file");
                    }
                }
                else {
                    uint8_t *img_buf = pvPortMalloc(width*height*2);
                    if (img_buf) {
                        res = mp_stream_posix_read((void *)ffd, img_buf, fsize);
                        if (res != (width*height*2)) {
                            vPortFree(img_buf);
                            mp_stream_close(ffd);
                            mp_raise_msg(&mp_type_OSError, "Error reading file");
                        }
                        send_data_scale(x, y, width, height, (color_t *)(img_buf), args[ARG_scale].u_int);
                        vPortFree(img_buf);
                    }
                    else {
                        mp_stream_close(ffd);
                        mp_raise_msg(&mp_type_OSError, "Error allocating file buffer");
                    }
                }
                mp_stream_close(ffd);
            }
        }
    }
    else {
        mp_raise_msg(&mp_type_OSError, "Unsupported image type");
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_Image_obj, 3, display_tft_Image);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_HSBtoRGB(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_hue,         MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_saturation,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_brightness,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_float_t hue = mp_obj_get_float(args[0].u_obj);
    mp_float_t sat = mp_obj_get_float(args[1].u_obj);
    mp_float_t bri = mp_obj_get_float(args[2].u_obj);

    color_t color = HSBtoRGB(hue, sat, bri);

    return mp_obj_new_int(color);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_HSBtoRGB_obj, 3, display_tft_HSBtoRGB);

//-------------------------------------------------------------------------
STATIC mp_obj_t display_tft_RGBtoColor(mp_obj_t self_in, mp_obj_t color_in)
{
    setupDevice(self_in);
    color_t color = get_color(color_in);

    return mp_obj_new_int(color);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_tft_RGBtoColor_obj, display_tft_RGBtoColor);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_setclipwin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y,  MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x0 = args[0].u_int;
    mp_int_t y0 = args[1].u_int;
    mp_int_t x1 = args[2].u_int;
    mp_int_t y1 = args[3].u_int;

    TFT_setclipwin(x0, y0, x1, y1);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_setclipwin_obj, 4, display_tft_setclipwin);

//--------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_resetclipwin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    setupDevice(pos_args[0]);
    TFT_resetclipwin();

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_resetclipwin_obj, 0, display_tft_resetclipwin);

//-------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_saveclipwin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    setupDevice(pos_args[0]);
    TFT_saveClipWin();

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_saveclipwin_obj, 0, display_tft_saveclipwin);

//----------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_restoreclipwin(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    setupDevice(pos_args[0]);
    TFT_restoreClipWin();

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_restoreclipwin_obj, 0, display_tft_restoreclipwin);

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_getSize(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    setupDevice(pos_args[0]);
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(active_dstate->_width);
    tuple[1] = mp_obj_new_int(active_dstate->_height);

    return mp_obj_new_tuple(2, tuple);
}
MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_getSize_obj, 0, display_tft_getSize);

//------------------------------------------------------
STATIC mp_obj_t display_tft_getWinSize(mp_obj_t self_in)
{
    setupDevice(self_in);
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(active_dstate->dispWin.x2 - active_dstate->dispWin.x1 + 1);
    tuple[1] = mp_obj_new_int(active_dstate->dispWin.y2 - active_dstate->dispWin.y1 + 1);

    return mp_obj_new_tuple(2, tuple);
}
MP_DEFINE_CONST_FUN_OBJ_1(display_tft_getWinSize_obj, display_tft_getWinSize);

//--------------------------------------------------
STATIC mp_obj_t display_tft_get_bg(mp_obj_t self_in)
{
    setupDevice(self_in);
    return mp_obj_new_int(active_dstate->_bg);
}
MP_DEFINE_CONST_FUN_OBJ_1(display_tft_get_bg_obj, display_tft_get_bg);

//--------------------------------------------------
STATIC mp_obj_t display_tft_get_fg(mp_obj_t self_in)
{
    setupDevice(self_in);
    return mp_obj_new_int(active_dstate->_fg);
}
MP_DEFINE_CONST_FUN_OBJ_1(display_tft_get_fg_obj, display_tft_get_fg);

//---------------------------------------------------------------------
STATIC mp_obj_t display_tft_set_bg(mp_obj_t self_in, mp_obj_t color_in)
{
    setupDevice(self_in);
    active_dstate->_bg = get_color(color_in);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_tft_set_bg_obj, display_tft_set_bg);

//---------------------------------------------------------------------
STATIC mp_obj_t display_tft_set_fg(mp_obj_t self_in, mp_obj_t color_in)
{
    setupDevice(self_in);
    active_dstate->_fg = get_color(color_in);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_tft_set_fg_obj, display_tft_set_fg);

//-------------------------------------------------
STATIC mp_obj_t display_tft_get_X(mp_obj_t self_in)
{
    setupDevice(self_in);
    int x = active_dstate->TFT_X;
    return mp_obj_new_int(x);
}
MP_DEFINE_CONST_FUN_OBJ_1(display_tft_get_X_obj, display_tft_get_X);

//-------------------------------------------------
STATIC mp_obj_t display_tft_get_Y(mp_obj_t self_in)
{
    setupDevice(self_in);
    int y = active_dstate->TFT_Y;
    return mp_obj_new_int(y);
}
MP_DEFINE_CONST_FUN_OBJ_1(display_tft_get_Y_obj, display_tft_get_Y);

//------------------------------------------------------------------------
STATIC mp_obj_t display_tft_set_speed(size_t n_args, const mp_obj_t *args)
{
    setupDevice(args[0]);
    display_tft_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args > 1) {
        uint32_t speed = mp_obj_get_int(args[1]);
        if (speed <= 24) speed *= 1000000;
        if ((speed < 1000000) || (speed > 24000000)) {
            mp_raise_ValueError("Unsupported tft speed (1 - 24 Mhz)");
        }
        if ((sysctl_clock_get_freq(SYSCTL_CLOCK_CPU) < 200000000) && (speed > SPI_DEFAULT_SPEED)) {
            mp_raise_ValueError("Maximal tft speed @100MHz CPU is 16 MHz");
        }

        // Set SPI clock used for display operations
        self->dconfig.speed = speed;
        tft_set_speed(speed);
        self->dconfig.speed = tft_get_speed();
    }
    return mp_obj_new_int(tft_get_speed());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(display_tft_set_speed_obj, 1, 2, display_tft_set_speed);

//------------------------------------------------
STATIC mp_obj_t display_tft_show(mp_obj_t self_in)
{
    setupDevice(self_in);
    send_frame_buffer();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(display_tft_show_obj, display_tft_show);

//-------------------------------------------------------------------------
STATIC mp_obj_t display_tft_use_tft_fb(size_t n_args, const mp_obj_t *args)
{
    setupDevice(args[0]);
    display_tft_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    int n_fb = 0;
    if (n_args > 1) {
        n_fb = mp_obj_get_int(args[1]);
        if (n_fb < 0) n_fb = 0;
        if (n_fb > 2) n_fb = 2;
        active_dstate->use_frame_buffer = (n_fb > 0);
        if (active_dstate->use_frame_buffer) {
            if (self->buff_obj0 == mp_const_none) {
                // create 1st buffer
                self->buff_obj0 = mp_obj_new_frame_buffer((active_dstate->_width * active_dstate->_height * 2) + 8);
                if (self->buff_obj0 == mp_const_none) {
                    if (self->buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)self->buff_obj1);
                    active_dstate->use_frame_buffer = false;
                    n_fb = 0;
                }
            }
            if (n_fb > 1) {
                if (self->buff_obj1 == mp_const_none) {
                    self->buff_obj1 = mp_obj_new_frame_buffer((active_dstate->_width * active_dstate->_height * 2) + 8);
                    if (self->buff_obj1 == mp_const_none) n_fb = 1;
                }
            }
            if (active_dstate->use_frame_buffer) {
                mp_obj_array_t *fbuf = (mp_obj_array_t *)self->buff_obj0;
                active_dstate->_tft_frame_buffer = fbuf->items;
                active_dstate->tft_frame_buffer = (uint16_t *)(active_dstate->_tft_frame_buffer + 8);
                self->active_fb = 0;
            }
        }
        else {
            // delete buffers if exists
            if (self->buff_obj0 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)self->buff_obj0);
            if (self->buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)self->buff_obj1);
            self->buff_obj0 = mp_const_none;
            self->buff_obj1 = mp_const_none;
            n_fb = 0;
        }
    }
    else {
        if (self->buff_obj0 != mp_const_none) n_fb++;
        if (self->buff_obj1 != mp_const_none) n_fb++;
    }
    return mp_obj_new_int(n_fb);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(display_tft_use_tft_fb_obj, 1, 2, display_tft_use_tft_fb);

//----------------------------------------------------------------------------
STATIC mp_obj_t display_tft_active_tft_fb(size_t n_args, const mp_obj_t *args)
{
    setupDevice(args[0]);
    display_tft_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (!active_dstate->use_frame_buffer) {
        mp_raise_msg(&mp_type_OSError, "Framebuffer not used");
    }
    if (n_args > 1) {
        int act_fb = mp_obj_get_int(args[1]);
        if (act_fb < 0) act_fb = 0;
        if (act_fb > 1) act_fb = 1;
        mp_obj_array_t *fbuf = (mp_obj_array_t *)self->buff_obj0;
        if (act_fb == 1) fbuf = (mp_obj_array_t *)self->buff_obj1;
        active_dstate->_tft_frame_buffer = fbuf->items;
        active_dstate->tft_frame_buffer = (uint16_t *)(active_dstate->_tft_frame_buffer + 8);
        self->active_fb = act_fb;
    }

    return mp_obj_new_int(self->active_fb);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(display_tft_active_tft_fb_obj, 1, 2, display_tft_active_tft_fb);

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t display_tft_fb_read(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_x, ARG_y, ARG_width, ARG_height, ARG_dest, ARG_png };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_INT,  { .u_int = 0 } },
        { MP_QSTR_y,      MP_ARG_INT,  { .u_int = 0 } },
        { MP_QSTR_width,  MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_height, MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_dest,   MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_png,    MP_ARG_BOOL, { .u_bool = false } },
    };

    setupDevice(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (!active_dstate->use_frame_buffer) {
        mp_raise_msg(&mp_type_OSError, "Framebuffer not used");
    }
    int x = args[ARG_x].u_int;
    int y = args[ARG_y].u_int;
    uint16_t width = (args[ARG_width].u_int > 0) ? (uint16_t)args[ARG_width].u_int : active_dstate->_width;
    uint16_t height = (args[ARG_height].u_int > 0) ? (uint16_t)args[ARG_height].u_int : active_dstate->_height;
    if ((x < 0) || (x > active_dstate->_width) || (y < 0) || (y > active_dstate->_height) ||
        (width < 1) || (width > active_dstate->_width) || (height < 1) || (height > active_dstate->_height)) {
        mp_raise_ValueError("Wrong coordinates");
    }

    bool to_png = args[ARG_png].u_bool;
    const char *filename = NULL;
    if (mp_obj_is_str(args[ARG_dest].u_obj)) filename = mp_obj_str_get_str(args[ARG_dest].u_obj);

    if ((!to_png) && (filename)) {
        char upr_fname[strlen(filename)+1];
        strcpy(upr_fname, filename);
        for (int i=0; i < strlen(upr_fname); i++) {
          upr_fname[i] = toupper((unsigned char) upr_fname[i]);
        }
        if (strstr(upr_fname, ".PNG") != NULL) to_png = true;
    }

    if (mp_obj_is_str(args[ARG_dest].u_obj)) {
        // === read to file ===
        if ((to_png) && (filename)) {
            // === Export to PNG file ===
            mp_obj_t png_buf = mp_obj_new_frame_buffer(width*height*3);
            if (png_buf != mp_const_none) {
                mp_obj_array_t *fbuf = (mp_obj_array_t *)png_buf;
                memset(fbuf->items, 0, width*height*3);
                int len = get_framebuffer_RGB888(x, y, x+width, y+height, width*height*3, (uint8_t *)fbuf->items);
                if (len < 0) {
                    mp_obj_delete_frame_buffer(fbuf);
                    mp_raise_msg(&mp_type_OSError, "Error encoding to RGB888");
                }
                unsigned error = lodepng_encode24_file(filename, (const unsigned char *)fbuf->items, width, height);
                mp_obj_delete_frame_buffer(fbuf);
                if (error) {
                    mp_raise_msg(&mp_type_OSError, lodepng_error_text(error));
                }
            }
            else {
                mp_raise_msg(&mp_type_OSError, "Error allocating buffer");
            }
            return mp_const_none;
        }

        // === Export to RAW file ===
        mp_obj_t fargs[2];
        fargs[0] = args[ARG_dest].u_obj;
        fargs[1] = mp_obj_new_str("wb", 2);

        // Open the file
        mp_obj_t ffd = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);
        if (ffd) {
            int written_bytes;
            int len = active_dstate->_width * active_dstate->_height*2;
            written_bytes = mp_stream_posix_write((void *)ffd, &width, 2);
            written_bytes = mp_stream_posix_write((void *)ffd, &height, 2);
            written_bytes = mp_stream_posix_write((void *)ffd, "rawB", 4);

            if ((x == 0) && (y == 0) && (width == active_dstate->_width) && (height == active_dstate->_height)) {
                written_bytes = mp_stream_posix_write((void *)ffd, active_dstate->tft_frame_buffer, len);
            }
            else {
                mp_obj_t raw_buf = mp_obj_new_frame_buffer(len);
                if (raw_buf != mp_const_none) {
                    mp_obj_array_t *fbuf = (mp_obj_array_t *)raw_buf;
                    len = get_framebuffer(x, y, x+width, y+height, width*height, (color_t *)fbuf->items);
                    len *= 2;
                    written_bytes = mp_stream_posix_write((void *)ffd, (const void *)fbuf->items, len);
                    mp_obj_delete_frame_buffer(fbuf);
                }
                else {
                    mp_stream_close(ffd);
                    mp_raise_msg(&mp_type_OSError, "Error allocating buffer");
                }
            }
            mp_stream_close(ffd);
            if (written_bytes != len) {
                mp_raise_msg(&mp_type_OSError, "Error writing to file");
            }
        }
        else {
            mp_raise_msg(&mp_type_OSError, "Error opening file for writing.");
        }
        return mp_const_none;
    }

    // === read to buffer ===
    mp_obj_t buffer = mp_const_none;
    if (to_png) {
        // === Export to PNG buffer ===
        mp_obj_t png_buf = mp_obj_new_frame_buffer(width*height*3);
        if (png_buf != mp_const_none) {
            mp_obj_array_t *fbuf = (mp_obj_array_t *)png_buf;
            memset(fbuf->items, 0, width*height*3);
            unsigned char* png = NULL;
            size_t pngsize;
            int len = get_framebuffer_RGB888(x, y, x+width, y+height, width*height*3, (uint8_t *)fbuf->items);
            if (len < 0) {
                mp_obj_delete_frame_buffer(fbuf);
                mp_raise_msg(&mp_type_OSError, "Error encoding to RGB888");
            }
            unsigned error = lodepng_encode24(&png, &pngsize, (const unsigned char *)fbuf->items, width, height);
            mp_obj_delete_frame_buffer(fbuf);
            if (error) {
                if (png) lodepng_free(png);
                mp_raise_msg(&mp_type_OSError, lodepng_error_text(error));
            }
            buffer = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)png, pngsize);
            lodepng_free(png);
        }
        else {
            mp_raise_msg(&mp_type_OSError, "Error allocating buffer");
        }
    }
    else {
        // === Export to RAW buffer ===
        if ((x == 0) && (y == 0) && (width == active_dstate->_width) && (height == active_dstate->_height)) {
            *(uint16_t *)(active_dstate->tft_frame_buffer-8) = active_dstate->_width;
            *(uint16_t *)(active_dstate->tft_frame_buffer-6) = active_dstate->_height;
            memcpy(active_dstate->tft_frame_buffer, "rawB", 4);
            buffer = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)active_dstate->_tft_frame_buffer, (active_dstate->_width * active_dstate->_height * 2)+8);
        }
        else {
            color_t *buf = pvPortMalloc((width*height*2)+8);
            if (buf) {
                int len = get_framebuffer(x, y, x+width, y+height, width*height, buf+8);
                if (len == (width*height)) {
                    *(uint16_t *)(buf) = active_dstate->_width;
                    *(uint16_t *)(buf+2) = active_dstate->_height;
                    memcpy(buf+4, "rawB", 4);
                    buffer = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)buf, (len*2)+8);
                }
                vPortFree(buf);
            }
            else {
                mp_raise_msg(&mp_type_OSError, "Error allocating buffer");
            }
        }
    }
    return buffer;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(display_tft_fb_read_obj, 0, display_tft_fb_read);


//================================================================
STATIC const mp_rom_map_elem_t display_tft_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_init),                MP_ROM_PTR(&display_tft_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),              MP_ROM_PTR(&display_tft_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_pixel),               MP_ROM_PTR(&display_tft_drawPixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_line),                MP_ROM_PTR(&display_tft_drawLine_obj) },
    { MP_ROM_QSTR(MP_QSTR_lineByAngle),         MP_ROM_PTR(&display_tft_drawLineByAngle_obj) },
    { MP_ROM_QSTR(MP_QSTR_triangle),            MP_ROM_PTR(&display_tft_drawTriangle_obj) },
    { MP_ROM_QSTR(MP_QSTR_circle),              MP_ROM_PTR(&display_tft_drawCircle_obj) },
    { MP_ROM_QSTR(MP_QSTR_ellipse),             MP_ROM_PTR(&display_tft_drawEllipse_obj) },
    { MP_ROM_QSTR(MP_QSTR_arc),                 MP_ROM_PTR(&display_tft_drawArc_obj) },
    { MP_ROM_QSTR(MP_QSTR_polygon),             MP_ROM_PTR(&display_tft_drawPoly_obj) },
    { MP_ROM_QSTR(MP_QSTR_rect),                MP_ROM_PTR(&display_tft_drawRect_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeScreen),         MP_ROM_PTR(&display_tft_writeScreen_obj) },
    { MP_ROM_QSTR(MP_QSTR_roundrect),           MP_ROM_PTR(&display_tft_drawRoundRect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),               MP_ROM_PTR(&display_tft_fillScreen_obj) },
    { MP_ROM_QSTR(MP_QSTR_clearwin),            MP_ROM_PTR(&display_tft_fillWin_obj) },
    { MP_ROM_QSTR(MP_QSTR_font),                MP_ROM_PTR(&display_tft_setFont_obj) },
    { MP_ROM_QSTR(MP_QSTR_getFont),             MP_ROM_PTR(&display_tft_getFont_obj) },
    { MP_ROM_QSTR(MP_QSTR_fontSize),            MP_ROM_PTR(&display_tft_getFontSize_obj) },
    { MP_ROM_QSTR(MP_QSTR_text),                MP_ROM_PTR(&display_tft_print_obj) },
    { MP_ROM_QSTR(MP_QSTR_orient),              MP_ROM_PTR(&display_tft_setRot_obj) },
    { MP_ROM_QSTR(MP_QSTR_textWidth),           MP_ROM_PTR(&display_tft_stringWidth_obj) },
    { MP_ROM_QSTR(MP_QSTR_textSize),            MP_ROM_PTR(&display_tft_stringSize_obj) },
    { MP_ROM_QSTR(MP_QSTR_textClear),           MP_ROM_PTR(&display_tft_clearStringRect_obj) },
    { MP_ROM_QSTR(MP_QSTR_attrib7seg),          MP_ROM_PTR(&display_tft_7segAttrib_obj) },
    { MP_ROM_QSTR(MP_QSTR_attribVfont),         MP_ROM_PTR(&display_tft_vectAttrib_obj) },
    { MP_ROM_QSTR(MP_QSTR_image),               MP_ROM_PTR(&display_tft_Image_obj) },
    { MP_ROM_QSTR(MP_QSTR_hsb2rgb),             MP_ROM_PTR(&display_tft_HSBtoRGB_obj) },
    { MP_ROM_QSTR(MP_QSTR_rgb2color),           MP_ROM_PTR(&display_tft_RGBtoColor_obj) },
    { MP_ROM_QSTR(MP_QSTR_setwin),              MP_ROM_PTR(&display_tft_setclipwin_obj) },
    { MP_ROM_QSTR(MP_QSTR_resetwin),            MP_ROM_PTR(&display_tft_resetclipwin_obj) },
    { MP_ROM_QSTR(MP_QSTR_savewin),             MP_ROM_PTR(&display_tft_saveclipwin_obj) },
    { MP_ROM_QSTR(MP_QSTR_restorewin),          MP_ROM_PTR(&display_tft_restoreclipwin_obj) },
    { MP_ROM_QSTR(MP_QSTR_screensize),          MP_ROM_PTR(&display_tft_getSize_obj) },
    { MP_ROM_QSTR(MP_QSTR_winsize),             MP_ROM_PTR(&display_tft_getWinSize_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fg),              MP_ROM_PTR(&display_tft_get_fg_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_bg),              MP_ROM_PTR(&display_tft_get_bg_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fg),              MP_ROM_PTR(&display_tft_set_fg_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_bg),              MP_ROM_PTR(&display_tft_set_bg_obj) },
    { MP_ROM_QSTR(MP_QSTR_text_x),              MP_ROM_PTR(&display_tft_get_X_obj) },
    { MP_ROM_QSTR(MP_QSTR_text_y),              MP_ROM_PTR(&display_tft_get_Y_obj) },
    { MP_ROM_QSTR(MP_QSTR_setspeed),            MP_ROM_PTR(&display_tft_set_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_show),                MP_ROM_PTR(&display_tft_show_obj) },
    { MP_ROM_QSTR(MP_QSTR_useFB),               MP_ROM_PTR(&display_tft_use_tft_fb_obj) },
    { MP_ROM_QSTR(MP_QSTR_activeFB),            MP_ROM_PTR(&display_tft_active_tft_fb_obj) },
    { MP_ROM_QSTR(MP_QSTR_readFB),              MP_ROM_PTR(&display_tft_fb_read_obj) },

    // class constants
    { MP_ROM_QSTR(MP_QSTR_CENTER),              MP_ROM_INT(CENTER) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT),               MP_ROM_INT(RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_BOTTOM),              MP_ROM_INT(BOTTOM) },
    { MP_ROM_QSTR(MP_QSTR_LASTX),               MP_ROM_INT(LASTX) },
    { MP_ROM_QSTR(MP_QSTR_LASTY),               MP_ROM_INT(LASTY) },

    { MP_ROM_QSTR(MP_QSTR_PORTRAIT),            MP_ROM_INT(PORTRAIT) },
    { MP_ROM_QSTR(MP_QSTR_LANDSCAPE),           MP_ROM_INT(LANDSCAPE) },
    { MP_ROM_QSTR(MP_QSTR_PORTRAIT_FLIP),       MP_ROM_INT(PORTRAIT_FLIP) },
    { MP_ROM_QSTR(MP_QSTR_LANDSCAPE_FLIP),      MP_ROM_INT(LANDSCAPE_FLIP) },

    { MP_ROM_QSTR(MP_QSTR_FONT_Default),        MP_ROM_INT(DEFAULT_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_DejaVu18),       MP_ROM_INT(DEJAVU18_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_DejaVu24),       MP_ROM_INT(DEJAVU24_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_Ubuntu),         MP_ROM_INT(UBUNTU16_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_Comic),          MP_ROM_INT(COMIC24_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_Minya),          MP_ROM_INT(MINYA24_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_Tooney),         MP_ROM_INT(TOONEY32_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_Small),          MP_ROM_INT(SMALL_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_DefaultSmall),   MP_ROM_INT(DEF_SMALL_FONT) },
    { MP_ROM_QSTR(MP_QSTR_FONT_7seg),           MP_ROM_INT(FONT_7SEG) },
    { MP_ROM_QSTR(MP_QSTR_FONT_USER1),          MP_ROM_INT(USER_FONT_1) },
    { MP_ROM_QSTR(MP_QSTR_FONT_USER2),          MP_ROM_INT(USER_FONT_2) },
    { MP_ROM_QSTR(MP_QSTR_FONT_USER3),          MP_ROM_INT(USER_FONT_3) },
    { MP_ROM_QSTR(MP_QSTR_FONT_USER4),          MP_ROM_INT(USER_FONT_4) },

    { MP_ROM_QSTR(MP_QSTR_VFONT_FUTURAL),         MP_ROM_INT(VFONT_FUTURAL) },

    { MP_ROM_QSTR(MP_QSTR_BLACK),               MP_ROM_INT(cTFT_BLACK) },
    { MP_ROM_QSTR(MP_QSTR_NAVY),                MP_ROM_INT(cTFT_NAVY) },
    { MP_ROM_QSTR(MP_QSTR_DARKGREEN),           MP_ROM_INT(cTFT_DARKGREEN) },
    { MP_ROM_QSTR(MP_QSTR_DARKCYAN),            MP_ROM_INT(cTFT_DARKCYAN) },
    { MP_ROM_QSTR(MP_QSTR_MAROON),              MP_ROM_INT(cTFT_MAROON) },
    { MP_ROM_QSTR(MP_QSTR_PURPLE),              MP_ROM_INT(cTFT_PURPLE) },
    { MP_ROM_QSTR(MP_QSTR_OLIVE),               MP_ROM_INT(cTFT_OLIVE) },
    { MP_ROM_QSTR(MP_QSTR_LIGHTGREY),           MP_ROM_INT(cTFT_LIGHTGREY) },
    { MP_ROM_QSTR(MP_QSTR_GREY),                MP_ROM_INT(cTFT_GREY) },
    { MP_ROM_QSTR(MP_QSTR_DARKGREY),            MP_ROM_INT(cTFT_DARKGREY) },
    { MP_ROM_QSTR(MP_QSTR_BLUE),                MP_ROM_INT(cTFT_BLUE) },
    { MP_ROM_QSTR(MP_QSTR_GREEN),               MP_ROM_INT(cTFT_GREEN) },
    { MP_ROM_QSTR(MP_QSTR_CYAN),                MP_ROM_INT(cTFT_CYAN) },
    { MP_ROM_QSTR(MP_QSTR_RED),                 MP_ROM_INT(cTFT_RED) },
    { MP_ROM_QSTR(MP_QSTR_MAGENTA),             MP_ROM_INT(cTFT_MAGENTA) },
    { MP_ROM_QSTR(MP_QSTR_YELLOW),              MP_ROM_INT(cTFT_YELLOW) },
    { MP_ROM_QSTR(MP_QSTR_WHITE),               MP_ROM_INT(cTFT_WHITE) },
    { MP_ROM_QSTR(MP_QSTR_ORANGE),              MP_ROM_INT(cTFT_ORANGE) },
    { MP_ROM_QSTR(MP_QSTR_GREENYELLOW),         MP_ROM_INT(cTFT_GREENYELLOW) },
    { MP_ROM_QSTR(MP_QSTR_PINK),                MP_ROM_INT(cTFT_PINK) },

    { MP_ROM_QSTR(MP_QSTR_JPG),                 MP_ROM_INT(IMAGE_TYPE_JPG) },
    { MP_ROM_QSTR(MP_QSTR_BMP),                 MP_ROM_INT(IMAGE_TYPE_BMP) },
    { MP_ROM_QSTR(MP_QSTR_RAW),                 MP_ROM_INT(IMAGE_TYPE_RAW) },
    { MP_ROM_QSTR(MP_QSTR_PNG),                 MP_ROM_INT(IMAGE_TYPE_PNG) },
};
STATIC MP_DEFINE_CONST_DICT(display_tft_locals_dict, display_tft_locals_dict_table);

//======================================
const mp_obj_type_t display_tft_type = {
    { &mp_type_type },
    .name = MP_QSTR_TFT,
    .print = display_tft_printinfo,
    .make_new = display_tft_make_new,
    .locals_dict = (mp_obj_dict_t*)&display_tft_locals_dict,
};

#endif // MICROPY_USE_DISPLAY

