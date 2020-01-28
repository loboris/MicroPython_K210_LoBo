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

#if MICROPY_USE_TFT

#include "tftspi.h"

#if MICROPY_USE_EPD

#include <string.h>
#include "epd_4_2.h"
#include "epdspi.h"

static const char *TAG = "[EPD_4.2]";

// ================ Waveshare 4.2" display ===================================================

// ==== LUTs ===============================================================
/*
static const uint8_t GxGDEW042T2_lut_20_vcom0_full[] =
{
0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
0x00, 0x17, 0x17, 0x00, 0x00, 0x02,
0x00, 0x0A, 0x01, 0x00, 0x00, 0x01,
0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_21_ww_full[] =
{
0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_22_bw_full[] =
{
0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_23_wb_full[] =
{
0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_24_bb_full[] =
{
0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const epd_lut_t lut_full = { (uint8_t *)GxGDEW042T2_lut_20_vcom0_full, (uint8_t *)GxGDEW042T2_lut_21_ww_full, (uint8_t *)GxGDEW042T2_lut_22_bw_full,
        (uint8_t *)GxGDEW042T2_lut_24_bb_full, (uint8_t *)GxGDEW042T2_lut_23_wb_full };
*/

#define TP0A  2 // sustain phase for bb and ww, change phase for bw and wb
#define TP0B 45 // change phase for bw and wb

static const uint8_t GxGDEW042T2_lut_20_vcom0_partial[] =
{
  0x00,
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_21_ww_partial[] =
{
  0x80, // 10 00 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_22_bw_partial[] =
{
  0xA0, // 10 10 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_23_wb_partial[] =
{
  0x50, // 01 01 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t GxGDEW042T2_lut_24_bb_partial[] =
{
  0x40, // 01 00 00 00
  TP0A, TP0B, 0x01, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const epd_lut_t lut_partial = { (uint8_t *)GxGDEW042T2_lut_20_vcom0_partial, (uint8_t *)GxGDEW042T2_lut_21_ww_partial, (uint8_t *)GxGDEW042T2_lut_22_bw_partial,
                                       (uint8_t *)GxGDEW042T2_lut_24_bb_partial, (uint8_t *)GxGDEW042T2_lut_23_wb_partial };



const uint8_t GxGDEW042T2_C_vcom[] = {
    0x00, 0x2C, 0x2C, 0x05, 0x24, 0x00,
    0x00, 0x0D, 0x01, 0x1C, 0x02, 0x03,
    0x00, 0x0A, 0x0A, 0x05, 0x05, 0x0C,
    0x00, 0x05, 0x05, 0x03, 0x03, 0x0F,
    0x00, 0x0D, 0x0A, 0x1C, 0x12, 0x03,
    0x00, 0x03, 0x01, 0x1E, 0x03, 0x06,
    0x00, 0x02, 0x19, 0x01, 0x03, 0x0C,
    0x00, 0x00,
};
const uint8_t GxGDEW042T2_C_ww[] = {
    0x90, 0x2C, 0x2C, 0x05, 0x24, 0x00,
    0x40, 0x0D, 0x01, 0x1C, 0x02, 0x03,
    0x66, 0x0A, 0x0A, 0x05, 0x05, 0x0C,
    0x99, 0x05, 0x05, 0x03, 0x03, 0x0F,
    0x80, 0x0D, 0x0A, 0x1C, 0x12, 0x03,
    0x00, 0x03, 0x01, 0x1E, 0x03, 0x06,
    0x00, 0x02, 0x19, 0x01, 0x03, 0x0C,
};
const uint8_t GxGDEW042T2_C_bw[] = {
    0xA8, 0x2C, 0x2C, 0x05, 0x24, 0x00,
    0x48, 0x0D, 0x01, 0x1C, 0x02, 0x03,
    0x66, 0x0A, 0x0A, 0x05, 0x05, 0x0C,
    0x99, 0x05, 0x05, 0x03, 0x03, 0x0F,
    0x06, 0x0D, 0x0A, 0x1C, 0x12, 0x03,
    0x8E, 0x03, 0x01, 0x1E, 0x03, 0x06,
    0xB0, 0x02, 0x19, 0x01, 0x03, 0x0C,
};
const uint8_t GxGDEW042T2_C_bb[] = {
    0x62, 0x2C, 0x2C, 0x05, 0x24, 0x00,
    0x09, 0x0D, 0x01, 0x1C, 0x02, 0x03,
    0x66, 0x0A, 0x0A, 0x05, 0x05, 0x0C,
    0x99, 0x05, 0x05, 0x03, 0x03, 0x0F,
    0x11, 0x0D, 0x12, 0x1C, 0x0A, 0x03,
    0x00, 0x03, 0x01, 0x1E, 0x03, 0x06,
    0x09, 0x02, 0x19, 0x01, 0x03, 0x0C,
};
const uint8_t GxGDEW042T2_C_wb[] = {
    0x90, 0x2C, 0x2C, 0x05, 0x24, 0x00,
    0x40, 0x0D, 0x01, 0x1C, 0x02, 0x03,
    0x66, 0x0A, 0x0A, 0x05, 0x05, 0x0C,
    0x99, 0x05, 0x05, 0x03, 0x03, 0x0F,
    0x80, 0x0D, 0x0A, 0x1C, 0x12, 0x03,
    0x00, 0x03, 0x01, 0x1E, 0x03, 0x06,
    0x00, 0x02, 0x19, 0x01, 0x03, 0x0C,
};

static const epd_lut_t lut_partial_c = { (uint8_t *)GxGDEW042T2_C_vcom, (uint8_t *)GxGDEW042T2_C_ww, (uint8_t *)GxGDEW042T2_C_bw,
                                       (uint8_t *)GxGDEW042T2_C_bb, (uint8_t *)GxGDEW042T2_C_wb };

static const uint8_t default_resolution[4] = {0x01, 0x90, 0x01, 0x2c};

// ==== Color LUTs =====================================

static uint8_t epd42_otp_buf[4096+8];

static int8_t epd42_otp_temperatures[15];

// default values for registers, starts with a5
// PSR, PFS, BTST(3), TSE, CDI, TCON, TRES(2x2), GSST(2x2), CCSET, PWS, TSSET (19 bytes + 1 byte (enable=a5))
static uint8_t *epd42_otp_default_regs;

// PLL | VCOM_HV VCOM_LV | VDH | VDL | VDHR | VCOM_DC
static uint8_t *epd42_otp_lut_pwr;
static const uint8_t lut_default_pwr[6] = {0x3c, 0x00, 0x2b, 0x2b, 0x09, 0x12};

static uint8_t *epd42_otp_lut_vcom;
static uint8_t *epd42_otp_lut_ww;
static uint8_t *epd42_otp_lut_bw;
static uint8_t *epd42_otp_lut_wb;
static uint8_t *epd42_otp_lut_bb;
static epd_lut_t epd42_otp_lut;

static bool first_update = true;
static uint8_t epd42_mode = 0;
static uint8_t disp_mode = 0x0f;

uint8_t epd42_part_mode = 1;

//----------------------------------------------
static void _print_lut(uint8_t *buf, char *info)
{
    printf("const uint8_t lut_otp_%s[] = {\n", info);
    for (int y=0; y < 7; y++) {
        printf("   ");
        for (int x=0; x<6; x++) {
            printf(" 0x%02X,", buf[y*6 + x]);
        }
        printf("\n");
    }
    if (strstr(info, "vcom")) printf("    0x%02X, 0x%02X,\n", buf[42], buf[43]);
    printf("};\n");
}

//===========================================================
void Epd42_ReadOTP(uint8_t *otp_buffer, int temper, bool prn)
{
    EPD_WriteCMD(READ_OTP);
    EPD_WRITE_DATA(); // data write
    io_read(spi_device, otp_buffer, 4097);
    SPI_DESELECT(); // CS=1
    mp_hal_delay_us(500);
    LOGD(TAG, "Read OTP LUTS for %d'C", temper);

    otp_buffer[0] = 0;
    if ((otp_buffer[1] != 0xa5) || (otp_buffer[0x21] != 0xa5) || (otp_buffer[0xf1] != 0xa5)) {
        LOGW(TAG, "Error reading OTP LUTS");
        otp_buffer[0] = 0;
        return;
    }
    memcpy(epd42_otp_temperatures, (int8_t *)otp_buffer + 2, 15);
    // find the nearest temperature
    int temp_idx = 15;
    int temp_diff = 999;
    for (int i=0; i<15; i++) {
        if (epd42_otp_temperatures[i] >= 0x7f) break;
        if (abs(temper - epd42_otp_temperatures[i]) < temp_diff) {
            temp_diff = abs(temper - epd42_otp_temperatures[i]);
            temp_idx = i;
        }
    }
    if (temp_idx >= 14) {
        LOGW(TAG, "OTP LUT temperature (%d) not found", temper);
        otp_buffer[0] = 0;
        return;
    }
    if (epd42_otp_temperatures[temp_idx] != temper) {
        LOGD(TAG, "Nearest temperature: %d'C", epd42_otp_temperatures[temp_idx]);
    }

    epd42_otp_default_regs = otp_buffer + 0x21;
    epd42_otp_lut_pwr = (uint8_t *)otp_buffer + 0x101 + (temp_idx*0x100); // LUT for requested temperature
    epd42_otp_lut_vcom = epd42_otp_lut_pwr + 6;
    epd42_otp_lut_ww = epd42_otp_lut_vcom + 44;
    epd42_otp_lut_bw = epd42_otp_lut_ww + 42;
    epd42_otp_lut_wb = epd42_otp_lut_bw + 42;
    epd42_otp_lut_bb = epd42_otp_lut_wb + 42;
    epd42_otp_lut.vcom0 = epd42_otp_lut_vcom;
    epd42_otp_lut.ww = epd42_otp_lut_ww;
    epd42_otp_lut.bw = epd42_otp_lut_bw;
    epd42_otp_lut.bb = epd42_otp_lut_bb;
    epd42_otp_lut.wb = epd42_otp_lut_wb;

    if (prn) {
        printf("\n// OTP LUTs for temperatures:");
        for (int i=0; i<15; i++) {
            if (epd42_otp_temperatures[i] == 127) break;
            printf(" %d", epd42_otp_temperatures[i]);
        }
        printf(" };\n");

        printf("// OTP default regs:\n");
        printf("const uint8_t epd42_otp_default_regs[%d] = {\n   ", epd42_otp_temperatures[temp_idx]);
        for (int i=0; i<20; i++) {
            printf(" 0x%02X,", epd42_otp_default_regs[i]);
        }
        printf("\n};\n");

        printf("// OTP LUT for %d'C\n", epd42_otp_temperatures[temp_idx]);
        printf("const uint8_t lut_pwr_settings[6] = {");
        for (int i=0; i<6; i++) {
            printf(" 0x%02X,", epd42_otp_lut_pwr[i]);
        }
        printf(" };\n");
        _print_lut(epd42_otp_lut_vcom, "vcom");
        _print_lut(epd42_otp_lut_ww, "ww");
        _print_lut(epd42_otp_lut_bw, "bw");
        _print_lut(epd42_otp_lut_bb, "bb");
        _print_lut(epd42_otp_lut_wb, "wb");
    }

    otp_buffer[0] = 1;
}


/**
 *  @brief: set the look-up table
 */
//-------------------------------------------
static void Epd42_SetLut(const epd_lut_t lut)
{
    if ((disp_mode & 0x20) == 0) return; // not needed in OTP mode
    uint8_t n_lut;
    const uint8_t *lutn = NULL;
    for (n_lut=LUT_FOR_VCOM; n_lut<=LUT_BLACK_TO_BLACK; n_lut++) {
        uint8_t n = 42;
        if (n_lut == LUT_FOR_VCOM) n = 44;
        switch (n_lut) {
            case LUT_FOR_VCOM:
                lutn = lut.vcom0;
                break;
            case LUT_WHITE_TO_WHITE:
                lutn = lut.ww;
                break;
            case LUT_BLACK_TO_WHITE:
                lutn = lut.bw;
                break;
            case LUT_WHITE_TO_BLACK:
                lutn = lut.wb;
                break;
            case LUT_BLACK_TO_BLACK:
                lutn = lut.bb;
                break;
        }
        EPD_WriteCMD(n_lut);
        EPD_WRITE_DATA(); // data write
        io_write(spi_device, lutn, n);
        SPI_DESELECT(); // CS=1
    }
}

//============================
void Epd42_setpll(uint8_t pll)
{
    EPD_WriteCMD_p1(PLL_CONTROL, pll);
}

/* Mode:
   0x0F LUT from OTP, B/W/C
   0x1F LUT from OTP, B/W
   0x2F LUT from REG, B/W/C
   0x3F LUT from REG, B/W

   PLL (refresh rate):
   0x39 200Hz; 0x3A 100Hz; 0x3B 67Hz; 0x3C 50Hz (default); 0x3D 40Hz; 0x3E 33HZ; 0x3F 29Hz;
   0x29 150Hz; 0x31 171Hz; 0x2F 20Hz; 0x0B 10Hz
 */
//---------------------------
void Epd42_Init(uint8_t mode)
{
    EPD_Reset();

    LOGD(TAG, "Init: mode %02X", mode);
    epd42_mode = mode & 0x3F;

    // Booster soft start
    EPD_WriteCMD_p3(BOOSTER_SOFT_START, 0x17, 0x17, 0x17);

    if (epd42_mode & 0x20) {
        // LUT from REG: Power settings
        uint8_t lut_pwr[5];
        if (epd42_otp_buf[0]) memcpy(lut_pwr, epd42_otp_lut_pwr, 5);
        else memcpy(lut_pwr, lut_default_pwr, 5);
        lut_pwr[0] = 0x03;

        EPD_WriteCMD(POWER_SETTING);
        EPD_WRITE_DATA(); // data write
        io_write(spi_device, lut_pwr, 5);
        SPI_DESELECT(); // CS=1
    }

    // Power on
    EPD_WriteCMD(POWER_ON);
    EPD_WaitReady();

    // Panel & PLL setting
    EPD_WriteCMD_p1(PANEL_SETTING, epd42_mode);
    EPD_WriteCMD_p1(PLL_CONTROL, (epd42_otp_buf[0]) ? epd42_otp_lut_pwr[0] : lut_default_pwr[0]);

    // Resolution settings (400x300)
    EPD_WriteCMD(RESOLUTION_SETTING);
    EPD_WRITE_DATA(); // data write
    io_write(spi_device, default_resolution, 4);
    SPI_DESELECT(); // CS=1

    if (epd42_mode & 0x20) {
        // LUT from REG: VCM_DC setting
        EPD_WriteCMD_p1(VCM_DC_SETTING, (epd42_otp_buf[0]) ? epd42_otp_lut_pwr[5] : lut_default_pwr[5]);
    }

    // VCOM and data interval setting
    EPD_WriteCMD_p1(VCOM_AND_DATA_INTERVAL_SETTING, 0x87);
}

/**
 * @brief: This displays the frame data from SRAM
 */
//----------------------------------
static void Epd42_DisplayFrame(void)
{
    EPD_WriteCMD(DISPLAY_REFRESH);

    SPI_DESELECT(); // CS=1
    mp_hal_delay_ms(10);
    EPD_WaitReady();
}

//--------------------
void Epd42_SendFrame()
{
    if (epd42_mode & 0x20) {
        LOGD(TAG, "Set full LUT");
        Epd42_SetLut(epd42_otp_lut);
    }

    mp_hal_wdt_reset();
    if (epd_type == EPD_TYPE_4_2) {
        LOGD(TAG, "Send frame: B/W mode");
        // this command writes “OLD” data to SRAM
        EPD_WriteCMD(DATA_START_TRANSMISSION_1);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        io_write(spi_device, drawBuff42y, EPD42_BUFFER_SIZE);

        // this command writes “NEW” data to SRAM
        mp_hal_delay_ms(3);
        EPD_WriteCMD(DATA_START_TRANSMISSION_2);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        io_write(spi_device, drawBuff42, EPD42_BUFFER_SIZE);
        // set old data buffer
        memcpy(drawBuff42y, drawBuff42, EPD42_BUFFER_SIZE);
    }
    else {
        LOGD(TAG, "Send frame: color mode");
        // this command writes “B/W” data to SRAM
        EPD_WriteCMD(DATA_START_TRANSMISSION_1);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        io_write(spi_device, drawBuff42, EPD42_BUFFER_SIZE);

        // this command writes “COLOR” data to SRAM
        mp_hal_delay_ms(3);
        EPD_WriteCMD(DATA_START_TRANSMISSION_2);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        io_write(spi_device, drawBuff42y, EPD42_BUFFER_SIZE);
    }
    SPI_DESELECT(); // CS=1

    mp_hal_wdt_reset();
    mp_hal_delay_us(500);
    Epd42_DisplayFrame();

    first_update = false;
}

/**
 *  @brief: transmit partial data to the SRAM
 *  xstart should be the multiple of 8, the last 3 bits will always be ignored
 */
//----------------------------------------------------------------------
void Epd42_SendPartialWindow(int xstart, int ystart, int xend, int yend)
{
    //if ((first_update) || (epd_type == EPD_TYPE_4_2_C)) {
    if (first_update) {
        Epd42_SendFrame();
        return;
    }

    if (epd42_mode & 0x20) {
        LOGD(TAG, "Set partial LUT");
        if (epd_type == EPD_TYPE_4_2) Epd42_SetLut(lut_partial);
        else Epd42_SetLut(lut_partial_c);
    }

    xstart &= 0x01ff;
    xend &= 0x01ff;
    ystart &= 0x01ff;
    yend &= 0x01ff;

    EPD_WriteCMD(PARTIAL_IN);
    SPI_DESELECT(); // CS=1
    mp_hal_delay_ms(3);

    EPD_WriteCMD(PARTIAL_WINDOW);
    EPD_WRITE_DATA(); // data write
    EPD_SPI_Write(xstart >> 8);
    EPD_SPI_Write(xstart & 0x0f8);
    EPD_SPI_Write(xend >> 8);
    EPD_SPI_Write(xend | 0x007);
    EPD_SPI_Write(ystart >> 8);
    EPD_SPI_Write(ystart & 0xff);
    EPD_SPI_Write(yend >> 8);
    EPD_SPI_Write(yend & 0xff);
    // 1 - Gates scan both inside and outside of the partial window. (default)
    // 0 - Gates scan inside of the partial window .
    EPD_SPI_Write(epd42_part_mode);
    SPI_DESELECT(); // CS=1
    mp_hal_delay_ms(3);

    // Send frame
    if (epd_type == EPD_TYPE_4_2) {
        LOGD(TAG, "Send partial frame: B/W mode");
        // this command writes “OLD” data to SRAM
        EPD_WriteCMD(DATA_START_TRANSMISSION_1);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        for (int dy=ystart; dy <= yend; dy++) {
            for (int dx=xstart; dx <= xend; dx+=8) {
                EPD_SPI_Write(drawBuff42y[(dy * (active_dstate->_width >> 3)) + (dx>>3)]);
            }
        }

        // this command writes “NEW” data to SRAM
        mp_hal_delay_ms(3);
        EPD_WriteCMD(DATA_START_TRANSMISSION_2);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        for (int dy=ystart; dy <= yend; dy++) {
            for (int dx=xstart; dx <= xend; dx+=8) {
                EPD_SPI_Write(drawBuff42[(dy * (active_dstate->_width >> 3)) + (dx>>3)]);
            }
        }
        // set old data buffer
        memcpy(drawBuff42y, drawBuff42, EPD42_BUFFER_SIZE);
    }
    else {
        LOGD(TAG, "Send partial frame: color mode");
        // this command writes “B/W” data to SRAM
        EPD_WriteCMD(DATA_START_TRANSMISSION_1);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        for (int dy=ystart; dy <= yend; dy++) {
            for (int dx=xstart; dx <= xend; dx+=8) {
                EPD_SPI_Write(drawBuff42[(dy * (active_dstate->_width >> 3)) + (dx>>3)] /*^ 0xff*/);
            }
        }
        // this command writes “COLOR” data to SRAM
        mp_hal_delay_ms(3);
        EPD_WriteCMD(DATA_START_TRANSMISSION_2);
        EPD_WRITE_DATA(); // data write
        //mp_hal_delay_us(500);
        for (int dy=ystart; dy <= yend; dy++) {
            for (int dx=xstart; dx <= xend; dx+=8) {
                EPD_SPI_Write(drawBuff42y[(dy * (active_dstate->_width >> 3)) + (dx>>3)]);
            }
        }
    }
    SPI_DESELECT(); // CS=1
    mp_hal_delay_us(500);
    Epd42_DisplayFrame();

    mp_hal_delay_ms(3);
    EPD_WriteCMD(PARTIAL_OUT);
    SPI_DESELECT(); // CS=1
}

//======================
int Epd42_ReadTemp(void)
{
    EPD_WriteCMD(TEMPERATURE_SENSOR_COMMAND);
    mp_hal_delay_ms(3);
    EPD_WaitReady();
    EPD_WRITE_DATA(); // data write
    uint8_t th = EPD_SPI_Read();
    uint8_t tl = EPD_SPI_Read();
    SPI_DESELECT(); // CS=1

    int t = (tl != 0) ? (th / (tl >> 5)) : (int)th;
    LOGD(TAG, "Temp: 0x%02X, 0x%02X, %d", th, tl, t);

    return t;
}

//============================================
void EPD_4_2_reset(uint8_t type, uint8_t mode)
{
    // Initialize display
    first_update = true;
    disp_mode = mode;
    epd42_otp_buf[0] = 0;
    if (type == DISP_TYPE_EPD_4_2) {
        LOGD(TAG, "INIT: B/W mode");
        if (mode & 0x20) Epd42_ReadOTP(epd42_otp_buf, Epd42_ReadTemp(), false);
        epd_type = EPD_TYPE_4_2;
        if (epd42_otp_buf[0] == 0) mode = 0x1f;         // OTP read error, set OTP mode
        Epd42_Init(mode);
        if ((mode & 0x2f) && (epd42_otp_buf[0])) {
            LOGD(TAG, "Set full LUT");
            Epd42_SetLut(epd42_otp_lut);                // default LUT
        }
        memset(drawBuff42, 0, EPD42_BUFFER_SIZE);       // clear screen
        memset(drawBuff42y, 0xFF, EPD42_BUFFER_SIZE);   // clear screen
        //Epd42_SendFrame();
    }
    else if (type == DISP_TYPE_EPD_4_2_C) {
        LOGD(TAG, "INIT: color mode");
        if (mode & 0x20) Epd42_ReadOTP(epd42_otp_buf, Epd42_ReadTemp(), false);
        epd_type = EPD_TYPE_4_2_C;
        if (epd42_otp_buf[0] == 0) mode = 0x0f;         // OTP read error, set OTP mode
        Epd42_Init(mode);
        if ((mode & 0x2f) && (epd42_otp_buf[0])) {
            LOGD(TAG, "Set full LUT and 171 MHz rate");
            EPD_WriteCMD_p1(PLL_CONTROL, 0x31);         // 171 MHz
            Epd42_SetLut(epd42_otp_lut);
        }
        memset(drawBuff42, 0x00, EPD42_BUFFER_SIZE);  // clear screen
        memset(drawBuff42y, 0x00, EPD42_BUFFER_SIZE); // clear screen
        //Epd42_SendFrame();
    }
}

// ^^^^^^^^^^^^^^^^ Waveshare 4.2" display ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#endif

#endif
