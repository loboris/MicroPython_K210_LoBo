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

#ifndef _MODDISPLAY_TFT_H_
#define _MODDISPLAY_TFT_H_

#include "mpconfigport.h"

#ifdef MICROPY_USE_DISPLAY

#include "py/obj.h"
#include "tftspi.h"
#include "tft.h"
#ifdef MICROPY_USE_EVE
#include "eve/FT8.h"
#endif

extern uint8_t disp_used_spi_host;

typedef struct _display_tft_obj_t {
    mp_obj_base_t base;
    display_config_t dconfig;
    uint32_t tp_calx;
    uint32_t tp_caly;
} display_tft_obj_t;

extern const mp_obj_type_t display_tft_type;

typedef struct _display_epd_obj_t {
    mp_obj_base_t base;
    display_config_t dconfig;
    uint8_t *drawBuff;
    uint8_t *gs_drawBuff;
} display_epd_obj_t;

extern const mp_obj_type_t display_epd_type;

#ifdef MICROPY_USE_EVE

typedef struct _display_eve_obj_t {
    mp_obj_base_t base;
    machine_hw_spi_obj_t *spi;
    eve_config_t dconfig;
    exspi_device_handle_t disp_spi_dev;
    exspi_device_handle_t *disp_spi;
    uint16_t width;
    uint16_t height;
    uint8_t in_list;
} display_eve_obj_t;

extern const mp_obj_type_t display_eve_type;

#endif

// Used by EVE & EPD
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawPixel_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawCircle_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawLine_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawLineByAngle_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawTriangle_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawEllipse_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawArc_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawPoly_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawRect_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_drawRoundRect_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_setFont_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_getFontSize_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_print_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_getSize_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_setclipwin_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_resetclipwin_obj);

// Used only by EPD
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_fillScreen_obj);
MP_DECLARE_CONST_FUN_OBJ_KW(display_tft_Image_obj);
MP_DECLARE_CONST_FUN_OBJ_1(display_tft_deinit_obj);

#endif

#endif
