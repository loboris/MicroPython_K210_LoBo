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

#if (MICROPY_USE_EPD == 1) && (MICROPY_USE_TFT == 1)

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "syslog.h"

#include "epdspi.h"
#include "epd_4_2.h"
#include "epd_2_9.h"
#include "moddisplay.h"
#include "modmachine.h"
#include "py/runtime.h"
#include "py/objstr.h"

#define EPD42_PLL_RATES_NUM 11
static const uint8_t epd42_pll_rates[][2] = {
        //{ 0x39, 200 },
        { 0x31, 170 },
        { 0x29, 150 },
        { 0x3A, 100 },
        { 0x3B,  67 },
        { 0x3C,  50 },
        { 0x3D,  40 },
        { 0x3F,  30 },
        { 0x2F,  20 },
        { 0x0B,  10 },
        { 0x0B,  10 },
        { 0x00,   0 },
};

static const char* const display_types[] = {
    "2.9\" Black (GDEH)",
    "2.9\" Black (DEPG)",
    "Waveshare 4.2\" Black",
    "Waveshare 4.2\" Color",
};

display_settings_t epd_display_settings = {0};

static const char TAG[] = "[MODEPD]";

//-----------------------
static void setupDevice()
{
    active_dstate = &epd_display_settings;
    active_dstate->tft_active_mode = TFT_MODE_EPD;
}

//------------------------------------------------------------------------------------------------
STATIC void display_epd_printinfo(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    display_epd_obj_t *self = self_in;
    if (self->handle) {
        mp_printf(print, "EPD   (%dx%d%s, Type=%s, Ready: %s, Clk=%u Hz)\n",
                epd_display_settings._width, epd_display_settings._height, (epd_display_settings.orientation & 2) ? " flip" : "",
                display_types[self->type-DISP_TYPE_EPD_2_9_GDEH], (self->handle) ? "yes" : "no", self->speed);
        mp_printf(print, "Pins  (miso=%d, clk=%d, cs=%d, dc=%d, busy=%d, reset=%d, power=%d)",
                self->mosi, self->sck, self->cs, self->dc, self->busy, self->reset, self->pwr);
    }
    else {
        mp_printf(print, "EPD (Not initialized)");
    }
}


// constructor(id, ...)
//-----------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t display_epd_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {

    display_epd_obj_t *self = m_new_obj(display_epd_obj_t);
    self->base.type = &display_epd_type;
    self->cs = -1;
    self->dc = -1;
    self->drawBuff = NULL;
    self->drawBuff_col = NULL;
    self->handle = 0;
    drawBuff = NULL;
    drawBuff42 = NULL;
    drawBuff42y = NULL;

    return MP_OBJ_FROM_PTR(self);
}

/*
//------------------------------------------------------
STATIC void spi_deinit_internal(display_epd_obj_t *self)
{
    if (self->disp_spi->handle) {
        int ret;
        // Deinitialize display spi device(s)
        ret = remove_extspi_device(self->disp_spi);
        if (ret != ESP_OK) {
            mp_raise_msg(&mp_type_OSError, "Error removing display device");
        }

        //gpio_pad_select_gpio(self->dconfig.miso);
        gpio_pad_select_gpio(self->dconfig.mosi);
        gpio_pad_select_gpio(self->dconfig.sck);
    }
}
*/

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t display_epd_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_type, ARG_id, ARG_speed, ARG_mosi, ARG_clk, ARG_cs,
           ARG_dc, ARG_busy, ARG_rst, ARG_pwr, ARG_rot, ARG_splash, ARG_mode };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_type,      MP_ARG_REQUIRED                   | MP_ARG_INT,  { .u_int = DISP_TYPE_EPD_2_9_GDEH } },
        { MP_QSTR_id,                          MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = 1 } },
        { MP_QSTR_speed,                       MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = 6000000 } },
        { MP_QSTR_mosi,      MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_clk,       MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_cs,        MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_dc,        MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_busy,                        MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_rst_pin,                     MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_pwr_pin,                     MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_rot,                         MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = LANDSCAPE } },
        { MP_QSTR_splash,                      MP_ARG_KW_ONLY  | MP_ARG_BOOL, { .u_bool = true } },
        { MP_QSTR_mode,                        MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    display_epd_obj_t *self = pos_args[0];
    int ret;

    // === deinitialize display spi device if it was initialized ===
    //if (self->disp_spi->handle) spi_deinit_internal(self);

    // === Get arguments ===
    if ((args[ARG_id].u_int < 0) && (args[ARG_id].u_int > 1)) {
        mp_raise_ValueError("SPI id must be either 0 or 1");
    }

    if ((args[ARG_type].u_int != DISP_TYPE_EPD_2_9_GDEH) && (args[ARG_type].u_int != DISP_TYPE_EPD_2_9_DEPG) &&
            (args[ARG_type].u_int != DISP_TYPE_EPD_4_2) && (args[ARG_type].u_int != DISP_TYPE_EPD_4_2_C)) {
        mp_raise_ValueError("Unsupported EPD display type");
    }

    self->type = args[ARG_type].u_int;
    self->spi_num = args[ARG_id].u_int;
    if ((self->type == DISP_TYPE_EPD_2_9_GDEH) || (self->type == DISP_TYPE_EPD_2_9_DEPG)) {
        self->width = EPD_2_9_DISPLAY_WIDTH;
        self->height = EPD_2_9_DISPLAY_HEIGHT;
    }
    else {
        self->width = EPD_4_2_DISPLAY_WIDTH;
        self->height = EPD_4_2_DISPLAY_HEIGHT;
    }
    self->reset = args[ARG_rst].u_int;
    self->busy = args[ARG_busy].u_int;
    if ((self->type == DISP_TYPE_EPD_2_9_GDEH) && (self->busy < 0)) {
        mp_raise_ValueError("GDEH029A1 requires BUSY pin!");
    }
    self->pwr = args[ARG_pwr].u_int;
    self->mosi = args[ARG_mosi].u_int;
    self->sck = args[ARG_clk].u_int;
    self->cs = args[ARG_cs].u_int;
    self->dc = args[ARG_dc].u_int;
    self->speed = args[ARG_speed].u_int;
    if ((self->speed < 1000000) || (self->speed > 20000000)) self->speed = 4000000;
    self->drawBuff = NULL;
    self->drawBuff_col = NULL;
    drawBuff = NULL;
    drawBuff42 = NULL;
    drawBuff42y = NULL;
    uint8_t mode42 = 0x0F;
    if (args[ARG_mode].u_int > 0) mode42 = args[ARG_mode].u_int & 0x3F;

    int orient = args[ARG_rot].u_int & 3 ;
    //if ((orient != LANDSCAPE) && (orient != LANDSCAPE_FLIP)) orient = LANDSCAPE;
    epd_display_settings._width = self->width;
    epd_display_settings._height = self->height;

    // Allocate frame buffers
    if ((self->type == DISP_TYPE_EPD_2_9_GDEH) || (self->type == DISP_TYPE_EPD_2_9_DEPG)) {
        self->drawBuff = pvPortMalloc(EPD_2_9_BUFFER_SIZE);
        if (self->drawBuff == NULL) {
            self->drawBuff = NULL;
            mp_raise_ValueError("Error allocating display buffer");
        }
        drawBuff = self->drawBuff;
        epd_type = EPD_TYPE_2_9;
    }
    else {
        self->drawBuff = pvPortMalloc(EPD42_BUFFER_SIZE);
        self->drawBuff_col = pvPortMalloc(EPD42_BUFFER_SIZE);
        if ((self->drawBuff == NULL) || (self->drawBuff_col == NULL)) {
            if (self->drawBuff) vPortFree(self->drawBuff);
            if (self->drawBuff_col) vPortFree(self->drawBuff_col);
            self->drawBuff = NULL;
            self->drawBuff_col = NULL;
            mp_raise_ValueError("Error allocating display buffers");
        }
        drawBuff42 = self->drawBuff;
        drawBuff42y = self->drawBuff_col;
        if (self->type == DISP_TYPE_EPD_4_2_C) epd_type = EPD_TYPE_4_2_C;
        else epd_type = EPD_TYPE_4_2;
    }

    // ================================
    // ==== Initialize the Display ====
    ret = EPD_display_init(self);
    if (ret != 0) {
        if (self->drawBuff) vPortFree(self->drawBuff);
        if (self->drawBuff_col) vPortFree(self->drawBuff_col);
        self->drawBuff = NULL;
        self->drawBuff_col = NULL;
        drawBuff = NULL;
        drawBuff42 = NULL;
        drawBuff42y = NULL;
        mp_raise_msg(&mp_type_OSError, "Error initializing display");
    }

    if ((self->type == DISP_TYPE_EPD_2_9_GDEH) || (self->type == DISP_TYPE_EPD_2_9_DEPG)) EPD_display_reset(self->type, 0);
    else if (self->type == DISP_TYPE_EPD_4_2) EPD_display_reset(self->type, 0x3F);
    else if (self->type == DISP_TYPE_EPD_4_2_C) EPD_display_reset(self->type, mode42);

    active_dstate = &epd_display_settings;
    active_dstate->tft_active_mode = TFT_MODE_EPD;

    gs_used_shades = 0;
    dotted_fil = 0;
    active_dstate->font_rotate = 0;
    active_dstate->text_wrap = 0;
    active_dstate->font_transparent = 0;
    active_dstate->font_forceFixed = 0;
    TFT_setRotation(orient);
    self->width = active_dstate->_width;
    self->height = active_dstate->_height;
    active_dstate->_fg = EPD_BLACK;
    active_dstate->_bg = EPD_WHITE;

    LOGD(TAG, "Display initialized");
    TFT_fillScreen(EPD_WHITE);
    TFT_setFont(TOONEY32_FONT, NULL, false);
    TFT_resetclipwin();
    if (args[ARG_splash].u_bool) {
        LOGD(TAG, "Show splash");
        TFT_drawRect(8, 8, active_dstate->_width-16, active_dstate->_height-16, EPD_BLACK);
        int fheight = TFT_getfontheight();
        TFT_print("MicroPython", CENTER, (active_dstate->_height/2) - fheight - 4);
        TFT_setFont(DEJAVU24_FONT, NULL, false);
        fheight = TFT_getfontheight();
        TFT_print("MicroPython for K210", CENTER, active_dstate->_height/2);
        TFT_setFont(DEFAULT_FONT, NULL, false);
        active_dstate->_bg = EPD_CBLACK;
        active_dstate->_fg = EPD_CWHITE;
        TFT_print(" Welcome to MicroPython e-Display ", CENTER, active_dstate->_height/2 + fheight + 4);
        active_dstate->_fg = EPD_BLACK;
        active_dstate->_bg = EPD_WHITE;
        if (epd_type != EPD_TYPE_2_9) {
            for (int n=50; n<=(active_dstate->_width-50); n+=10) {
                TFT_drawCircle(n, active_dstate->_height - 40, 20, EPD_BLACK);
            }
        }
    }

    EPD_UpdateScreen(self);
    TFT_setFont(DEFAULT_FONT, NULL, false);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(display_epd_init_obj, 0, display_epd_init);

//------------------------------------------------------------------------------------------
STATIC mp_obj_t display_epd_show(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x,      MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_y,      MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_width,  MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_height, MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_full,   MP_ARG_INT,  { .u_int = 0 } },
    };

    setupDevice();
    display_epd_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (epd_type == EPD_TYPE_2_9) {
        if (args[4].u_int > 0) {
            EPD_2_9_display(self->drawBuff, (args[4].u_int-1) & 0x3ff);
            return mp_const_none;
        }
    }
    else {
        if (args[4].u_int > 0) {
            Epd42_SendFrame();
            return mp_const_none;
        }
    }

    mp_int_t x, y, xend, yend, tmpx, tmpy, tmpxend, tmpyend, w, h;
    bool have_coords = true;
    for (int i=0; i<4; i++) {
        if (args[i].u_int < 0) {
            have_coords = false;
            break;
        }
    }
    if (have_coords) {
        x = args[0].u_int & 0x1ff;
        y = args[1].u_int & 0x1ff;
        xend = x + (args[2].u_int & 0x1ff) - 1;
        yend = y + (args[3].u_int & 0x1ff) - 1;
    }
    else {
        x = 0;
        y = 0;
        xend = active_dstate->_width-1;
        yend = active_dstate->_height-1;
    }

    tmpx = x;
    tmpxend = xend;
    tmpy = y;
    tmpyend = yend;

    //LOGD(TAG, "Update partial: %ld, %ld, %ld, %ld", x, y, xend, yend);
    if ((x < active_dstate->_width) && (y < active_dstate->_height) && (xend < active_dstate->_width) && (yend < active_dstate->_height) && (xend > x) && (yend > y)) {
        // adjust coordinates based on current orientation
        if (epd_type == EPD_TYPE_2_9) {
            w = 296;
            h = 128;
        }
        else {
            w = 400;
            h = 300;
        }
        switch(active_dstate->orientation) {
            case LANDSCAPE_FLIP:
                y = h - tmpyend - 1;
                yend = h - tmpy -1 ;
                x = w - tmpxend - 1;
                xend = w - tmpx -1 ;
                break;
            case PORTRAIT:
                y = h - tmpxend -1;
                yend = h - tmpx - 1;
                x = tmpy;
                xend = tmpyend;
                break;
            case PORTRAIT_FLIP:
                y = tmpx;
                yend = tmpxend;
                x = tmpy;
                xend = tmpyend;
                break;
        }

        //LOGD(TAG, "Update partial: %ld, %ld, %ld, %ld", x, y, xend, yend);
        EPD_UpdatePartial(self, x, y, xend, yend);
    }
    else mp_raise_ValueError("Wrong coordinates");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(display_epd_show_obj, 0, display_epd_show);

//-------------------------------------------------------------------
STATIC mp_obj_t display_epd_reset(mp_obj_t self_in, mp_obj_t mode_in)
{
    setupDevice();
    display_epd_obj_t *self = (display_epd_obj_t *)self_in;
    uint8_t mode = (uint8_t)mp_obj_get_int(mode_in) & 0x3f;
    mode |= 0x0F;
    if ((self->type != DISP_TYPE_EPD_2_9_GDEH) && (self->type != DISP_TYPE_EPD_2_9_DEPG)) {
        if (mode & 0x10) self->type = DISP_TYPE_EPD_4_2;
        else self->type = DISP_TYPE_EPD_4_2_C;
        EPD_display_reset(self->type, mode);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_epd_reset_obj, display_epd_reset);

//--------------------------------------------------------
STATIC mp_obj_t display_epd_get_upd_time(mp_obj_t self_in)
{
    setupDevice();
    //display_epd_obj_t *self = (display_epd_obj_t *)self_in;
    return mp_obj_new_int(EPD_ready_timer);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(display_epd_get_upd_time_obj, display_epd_get_upd_time);

//--------------------------------------------------------------------
STATIC mp_obj_t display_epd_clear(size_t n_args, const mp_obj_t *args)
{
    setupDevice();
    display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(args[0]);
    bool show = false;
    if (n_args > 1) show = mp_obj_is_true(args[1]);

    if ((self->type == DISP_TYPE_EPD_2_9_GDEH) || (self->type == DISP_TYPE_EPD_2_9_DEPG)) {
        memset(drawBuff, 0xFF, EPD_2_9_BUFFER_SIZE);
        gs_used_shades = 0;
    }
    else if (self->type == DISP_TYPE_EPD_4_2) {
        memset(drawBuff42, 0, EPD42_BUFFER_SIZE);
    }
    else {
        memset(drawBuff42, 0x00, EPD42_BUFFER_SIZE);
        memset(drawBuff42y, 0, EPD42_BUFFER_SIZE);
    }

    if (show) EPD_UpdateScreen(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(display_epd_clear_obj, 1, 2, display_epd_clear);

//---------------------------------------------------------------------
STATIC mp_obj_t display_epd_set_bg(mp_obj_t self_in, mp_obj_t color_in)
{
    setupDevice();
    active_dstate->_bg = mp_obj_get_int(color_in) & 0x1f;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_epd_set_bg_obj, display_epd_set_bg);

//---------------------------------------------------------------------
STATIC mp_obj_t display_epd_set_fg(mp_obj_t self_in, mp_obj_t color_in)
{
    setupDevice();
    active_dstate->_fg = mp_obj_get_int(color_in) & 0x1f;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_epd_set_fg_obj, display_epd_set_fg);

//--------------------------------------------------
STATIC mp_obj_t display_epd_deinit(mp_obj_t self_in)
{
    setupDevice();
    display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(self_in);

    EPD_display_deinit(self);

    if (self->drawBuff) vPortFree(self->drawBuff);
    if (self->drawBuff_col) vPortFree(self->drawBuff_col);
    self->drawBuff = NULL;
    self->drawBuff_col = NULL;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(display_epd_deinit_obj, display_epd_deinit);

//------------------------------------------------------------------
STATIC mp_obj_t display_epd_PLL(mp_obj_t self_in, mp_obj_t pll_in)
{
    setupDevice();
    display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(self_in);
    if ((self->type != DISP_TYPE_EPD_2_9_GDEH) && (self->type != DISP_TYPE_EPD_2_9_DEPG)) {
        uint8_t pll_hz = (uint8_t)(mp_obj_get_int(pll_in) & 0xff);
        uint8_t pll = 0;
        for (int i=0; i < EPD42_PLL_RATES_NUM; i++) {
            if (epd42_pll_rates[i][1] == pll_hz) {
                pll = epd42_pll_rates[i][0];
                break;
            }
        }
        if (pll) Epd42_setpll(pll);
        else mp_raise_ValueError("Not supported refresh rate");
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_epd_PLL_obj, display_epd_PLL);

//-------------------------------------------------------------------
STATIC mp_obj_t display_epd_partMode(mp_obj_t self_in, mp_obj_t mode)
{
    setupDevice();
    display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(self_in);
    if ((self->type != DISP_TYPE_EPD_2_9_GDEH) && (self->type != DISP_TYPE_EPD_2_9_DEPG)) {
        epd42_part_mode = (uint8_t)(mp_obj_get_int(mode) & 0x01);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_epd_partMode_obj, display_epd_partMode);

//---------------------------------------------------
STATIC mp_obj_t display_epd_gettemp(mp_obj_t self_in)
{
    setupDevice();
    display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(self_in);

    if ((self->type != DISP_TYPE_EPD_2_9_GDEH) && (self->type != DISP_TYPE_EPD_2_9_DEPG)) {
        int t = Epd42_ReadTemp();
        return mp_obj_new_int(t);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(display_epd_gettemp_obj, display_epd_gettemp);

//----------------------------------------------------------------------
STATIC mp_obj_t display_epd_readOTP(size_t n_args, const mp_obj_t *args)
{
    setupDevice();
    display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(args[0]);

    if ((self->type != DISP_TYPE_EPD_2_9_GDEH) && (self->type != DISP_TYPE_EPD_2_9_DEPG)) {
        bool prn = false;
        int temper = 20;
        if (n_args > 1) prn = mp_obj_is_true(args[1]);
        if (n_args > 2) temper = mp_obj_get_int(args[2]);

        uint8_t *buf = pvPortMalloc(4096+8);
        if (buf) {
            Epd42_ReadOTP(buf, temper, prn);
            if (buf[0] == 0) {
                vPortFree(buf);
                mp_raise_msg(&mp_type_OSError, "Error reading OTP memory");
            }
            mp_obj_t ret = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)(buf+1), 4096);
            vPortFree(buf);
            return ret;
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(display_epd_readOTP_obj, 1, 3, display_epd_readOTP);

//----------------------------------------------------------------------
STATIC mp_obj_t display_epd_dotted(mp_obj_t self_in, mp_obj_t dotted_in)
{
    setupDevice();
    //display_epd_obj_t *self = (display_epd_obj_t *)MP_OBJ_TO_PTR(self_in);

    dotted_fil = mp_obj_is_true(dotted_in);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(display_epd_dotted_obj, display_epd_dotted);


//----------------------------------------------------------------
STATIC const mp_rom_map_elem_t display_epd_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_init),                MP_ROM_PTR(&display_epd_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),              MP_ROM_PTR(&display_epd_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_show),                MP_ROM_PTR(&display_epd_show_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),               MP_ROM_PTR(&display_epd_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_updtime),             MP_ROM_PTR(&display_epd_get_upd_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),               MP_ROM_PTR(&display_epd_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fg),              MP_ROM_PTR(&display_epd_set_fg_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_bg),              MP_ROM_PTR(&display_epd_set_bg_obj) },
    { MP_ROM_QSTR(MP_QSTR_rate),                MP_ROM_PTR(&display_epd_PLL_obj) },
    { MP_ROM_QSTR(MP_QSTR_partMode),            MP_ROM_PTR(&display_epd_partMode_obj) },
    { MP_ROM_QSTR(MP_QSTR_temper),              MP_ROM_PTR(&display_epd_gettemp_obj) },
    { MP_ROM_QSTR(MP_QSTR_readOTP),             MP_ROM_PTR(&display_epd_readOTP_obj) },
    { MP_ROM_QSTR(MP_QSTR_dottedFill),          MP_ROM_PTR(&display_epd_dotted_obj) },
    // common with TFT module
    { MP_ROM_QSTR(MP_QSTR_pixel),               MP_ROM_PTR(&display_tft_drawPixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_line),                MP_ROM_PTR(&display_tft_drawLine_obj) },
    { MP_ROM_QSTR(MP_QSTR_lineByAngle),         MP_ROM_PTR(&display_tft_drawLineByAngle_obj) },
    { MP_ROM_QSTR(MP_QSTR_triangle),            MP_ROM_PTR(&display_tft_drawTriangle_obj) },
    { MP_ROM_QSTR(MP_QSTR_circle),              MP_ROM_PTR(&display_tft_drawCircle_obj) },
    { MP_ROM_QSTR(MP_QSTR_ellipse),             MP_ROM_PTR(&display_tft_drawEllipse_obj) },
    { MP_ROM_QSTR(MP_QSTR_arc),                 MP_ROM_PTR(&display_tft_drawArc_obj) },
    { MP_ROM_QSTR(MP_QSTR_polygon),             MP_ROM_PTR(&display_tft_drawPoly_obj) },
    { MP_ROM_QSTR(MP_QSTR_rect),                MP_ROM_PTR(&display_tft_drawRect_obj) },
    { MP_ROM_QSTR(MP_QSTR_roundrect),           MP_ROM_PTR(&display_tft_drawRoundRect_obj) },
    { MP_ROM_QSTR(MP_QSTR_font),                MP_ROM_PTR(&display_tft_setFont_obj) },
    { MP_ROM_QSTR(MP_QSTR_getFont),             MP_ROM_PTR(&display_tft_getFont_obj) },
    { MP_ROM_QSTR(MP_QSTR_fontSize),            MP_ROM_PTR(&display_tft_getFontSize_obj) },
    { MP_ROM_QSTR(MP_QSTR_text),                MP_ROM_PTR(&display_tft_print_obj) },
    { MP_ROM_QSTR(MP_QSTR_screensize),          MP_ROM_PTR(&display_tft_getSize_obj) },
    { MP_ROM_QSTR(MP_QSTR_setwin),              MP_ROM_PTR(&display_tft_setclipwin_obj) },
    { MP_ROM_QSTR(MP_QSTR_resetwin),            MP_ROM_PTR(&display_tft_resetclipwin_obj) },
    { MP_ROM_QSTR(MP_QSTR_image),               MP_ROM_PTR(&display_tft_Image_obj) },
    { MP_ROM_QSTR(MP_QSTR_orient),              MP_ROM_PTR(&display_tft_setRot_obj) },
    { MP_ROM_QSTR(MP_QSTR_attrib7seg),          MP_ROM_PTR(&display_tft_7segAttrib_obj) },
    { MP_ROM_QSTR(MP_QSTR_attribVfont),         MP_ROM_PTR(&display_tft_vectAttrib_obj) },
    { MP_ROM_QSTR(MP_QSTR_textWidth),           MP_ROM_PTR(&display_tft_stringWidth_obj) },
    { MP_ROM_QSTR(MP_QSTR_textSize),            MP_ROM_PTR(&display_tft_stringSize_obj) },
    { MP_ROM_QSTR(MP_QSTR_textClear),           MP_ROM_PTR(&display_tft_clearStringRect_obj) },
    { MP_ROM_QSTR(MP_QSTR_text_x),              MP_ROM_PTR(&display_tft_get_X_obj) },
    { MP_ROM_QSTR(MP_QSTR_text_y),              MP_ROM_PTR(&display_tft_get_Y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fg),              MP_ROM_PTR(&display_tft_get_fg_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_bg),              MP_ROM_PTR(&display_tft_get_bg_obj) },

    // Constants
    { MP_ROM_QSTR(MP_QSTR_WS_2_9B_GDEH),        MP_ROM_INT(DISP_TYPE_EPD_2_9_GDEH) },
    { MP_ROM_QSTR(MP_QSTR_WS_2_9B_DEPG),        MP_ROM_INT(DISP_TYPE_EPD_2_9_DEPG) },
    { MP_ROM_QSTR(MP_QSTR_WS_4_2B),             MP_ROM_INT(DISP_TYPE_EPD_4_2) },
    { MP_ROM_QSTR(MP_QSTR_WS_4_2C),             MP_ROM_INT(DISP_TYPE_EPD_4_2_C) },

    { MP_ROM_QSTR(MP_QSTR_CENTER),              MP_ROM_INT(CENTER) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT),               MP_ROM_INT(RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_BOTTOM),              MP_ROM_INT(BOTTOM) },
    { MP_ROM_QSTR(MP_QSTR_LASTX),               MP_ROM_INT(LASTX) },
    { MP_ROM_QSTR(MP_QSTR_LASTY),               MP_ROM_INT(LASTY) },

    { MP_ROM_QSTR(MP_QSTR_PORTRAIT),            MP_ROM_INT(PORTRAIT) },
    { MP_ROM_QSTR(MP_QSTR_LANDSCAPE),           MP_ROM_INT(LANDSCAPE) },
    { MP_ROM_QSTR(MP_QSTR_PORTRAIT_FLIP),       MP_ROM_INT(PORTRAIT_FLIP) },
    { MP_ROM_QSTR(MP_QSTR_LANDSCAPE_FLIP),      MP_ROM_INT(LANDSCAPE_FLIP) },

    { MP_ROM_QSTR(MP_QSTR_VFONT_FUTURAL),       MP_ROM_INT(VFONT_FUTURAL) },

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

    { MP_ROM_QSTR(MP_QSTR_BLACK),               MP_ROM_INT(EPD_BLACK) },
    { MP_ROM_QSTR(MP_QSTR_WHITE),               MP_ROM_INT(EPD_WHITE) },
    { MP_ROM_QSTR(MP_QSTR_COLOR),               MP_ROM_INT(EPD_CBLACK) },
    { MP_ROM_QSTR(MP_QSTR_CWHITE),              MP_ROM_INT(EPD_CWHITE) },
    { MP_ROM_QSTR(MP_QSTR_GRAY1),               MP_ROM_INT(EPD_GRAY1) },
    { MP_ROM_QSTR(MP_QSTR_GRAY2),               MP_ROM_INT(EPD_GRAY2) },
    { MP_ROM_QSTR(MP_QSTR_GRAY3),               MP_ROM_INT(EPD_GRAY3) },
    { MP_ROM_QSTR(MP_QSTR_GRAY4),               MP_ROM_INT(EPD_GRAY4) },
    { MP_ROM_QSTR(MP_QSTR_GRAY5),               MP_ROM_INT(EPD_GRAY5) },
    { MP_ROM_QSTR(MP_QSTR_GRAY6),               MP_ROM_INT(EPD_GRAY6) },
    { MP_ROM_QSTR(MP_QSTR_GRAY7),               MP_ROM_INT(EPD_GRAY7) },
};
STATIC MP_DEFINE_CONST_DICT(display_epd_locals_dict, display_epd_locals_dict_table);


//======================================
const mp_obj_type_t display_epd_type = {
    { &mp_type_type },
    .name = MP_QSTR_TFT,
    .print = display_epd_printinfo,
    .make_new = display_epd_make_new,
    .locals_dict = (mp_obj_t)&display_epd_locals_dict,
};

#endif // CONFIG_MICROPY_USE_TFT















