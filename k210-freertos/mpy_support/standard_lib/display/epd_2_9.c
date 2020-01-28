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

#define CONFIG_SHA_EPD_2_9_LUT_DEBUG

#include <string.h>
#include "epd_2_9.h"
#include "epdspi.h"

static const char *TAG = "[EPD_2.9]";

// currently initialized display type
uint8_t EPD_2_9_dev_type;

uint32_t *EPD_2_9_tmpbuf = NULL;
uint32_t *EPD_2_9_oldbuf = NULL;

static bool EPD_2_9_have_oldbuf = false;


//--------------------------------------
static const uint8_t xlat_curve[256] = {
    0x00,0x01,0x01,0x02,0x02,0x03,0x03,0x03,0x04,0x04,0x05,0x05,
    0x06,0x06,0x07,0x07,0x08,0x08,0x09,0x09,0x0a,0x0a,0x0a,0x0b,
    0x0b,0x0c,0x0c,0x0d,0x0d,0x0e,0x0e,0x0f,0x0f,0x10,0x10,0x11,
    0x11,0x12,0x12,0x13,0x13,0x14,0x15,0x15,0x16,0x16,0x17,0x17,
    0x18,0x18,0x19,0x19,0x1a,0x1a,0x1b,0x1b,0x1c,0x1d,0x1d,0x1e,
    0x1e,0x1f,0x1f,0x20,0x20,0x21,0x22,0x22,0x23,0x23,0x24,0x25,
    0x25,0x26,0x26,0x27,0x27,0x28,0x29,0x29,0x2a,0x2a,0x2b,0x2c,
    0x2c,0x2d,0x2e,0x2e,0x2f,0x2f,0x30,0x31,0x31,0x32,0x33,0x33,
    0x34,0x35,0x35,0x36,0x37,0x37,0x38,0x39,0x39,0x3a,0x3b,0x3b,
    0x3c,0x3d,0x3e,0x3e,0x3f,0x40,0x40,0x41,0x42,0x43,0x43,0x44,
    0x45,0x46,0x46,0x47,0x48,0x49,0x49,0x4a,0x4b,0x4c,0x4c,0x4d,
    0x4e,0x4f,0x50,0x50,0x51,0x52,0x53,0x54,0x55,0x55,0x56,0x57,
    0x58,0x59,0x5a,0x5b,0x5b,0x5c,0x5d,0x5e,0x5f,0x60,0x61,0x62,
    0x63,0x64,0x65,0x66,0x66,0x67,0x68,0x69,0x6a,0x6b,0x6c,0x6d,
    0x6e,0x6f,0x70,0x71,0x72,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,
    0x7b,0x7c,0x7d,0x7e,0x80,0x81,0x82,0x83,0x84,0x86,0x87,0x88,
    0x89,0x8a,0x8c,0x8d,0x8e,0x90,0x91,0x92,0x93,0x95,0x96,0x98,
    0x99,0x9a,0x9c,0x9d,0x9f,0xa0,0xa2,0xa3,0xa5,0xa6,0xa8,0xa9,
    0xab,0xac,0xae,0xb0,0xb1,0xb3,0xb5,0xb6,0xb8,0xba,0xbc,0xbe,
    0xbf,0xc1,0xc3,0xc5,0xc7,0xc9,0xcb,0xcd,0xcf,0xd1,0xd3,0xd6,
    0xd8,0xda,0xdc,0xdf,0xe1,0xe3,0xe6,0xe8,0xeb,0xed,0xf0,0xf3,
    0xf5,0xf8,0xfb,0xfe,
};

// full, includes inverting
const struct EPD_2_9_lut_entry EPD_2_9_lut_full[] = {
    { .length = 23, .voltages = 0x02, },
    { .length =  4, .voltages = 0x01, },
    { .length = 11, .voltages = 0x11, },
    { .length =  4, .voltages = 0x12, },
    { .length =  6, .voltages = 0x22, },
    { .length =  5, .voltages = 0x66, },
    { .length =  4, .voltages = 0x69, },
    { .length =  5, .voltages = 0x59, },
    { .length =  1, .voltages = 0x58, },
    { .length = 14, .voltages = 0x99, },
    { .length =  1, .voltages = 0x88, },
    { .length = 0 }
};

// full, no inversion
const struct EPD_2_9_lut_entry EPD_2_9_lut_normal[] = {
    { .length =  3, .voltages = 0x10, },
    { .length =  5, .voltages = 0x18, },
    { .length =  1, .voltages = 0x08, },
    { .length =  8, .voltages = 0x18, },
    { .length =  2, .voltages = 0x08, },
    { .length = 0 }
};

// full, no inversion, needs 2 updates for full update
const struct EPD_2_9_lut_entry EPD_2_9_lut_faster[] = {
    { .length =  1, .voltages = 0x10, },
    { .length =  8, .voltages = 0x18, },
    { .length =  1, .voltages = 0x08, },
    { .length = 0 }
};

// full, no inversion, needs 4 updates for full update
const struct EPD_2_9_lut_entry EPD_2_9_lut_fastest[] = {
    { .length =  1, .voltages = 0x10, },
    { .length =  5, .voltages = 0x18, },
    { .length =  1, .voltages = 0x08, },
    { .length = 0 }
};

//-----------------------------------------------------------------------------
static uint8_t EPD_2_9_lut_conv(uint8_t voltages, enum EPD_2_9_lut_flags flags)
{
    if (flags & LUT_FLAG_FIRST) {
        voltages |= voltages >> 4;
        voltages &= 15;
        if ((voltages & 3) == 3) // reserved
            voltages ^= 2; // set to '1': VSH (black)
        if ((voltages & 12) == 12) // reserved
            voltages ^= 4; // set to '2': VSL (white)
        voltages |= voltages << 4;
    }

    if (flags & LUT_FLAG_PARTIAL) voltages &= 0x3c; // only keep 0->1 and 1->0
    if (flags & LUT_FLAG_WHITE) voltages &= 0xcc; // only keep 0->1 and 1->1
    if (flags & LUT_FLAG_BLACK) voltages &= 0x33; // only keep 0->0 and 1->0

    return voltages;
}


// GDEH029A1
//------------------------------------------------------------------------------------------------------------------
int EPD_2_9_lut_generate_gdeh029a1(const struct EPD_2_9_lut_entry *list, enum EPD_2_9_lut_flags flags, uint8_t *lut)
{
    LOGD(TAG, "GDEH flags = %d.", flags);

    memset(lut, 0, 30);

    int pos = 0;
    while (list->length != 0) {
        uint8_t voltages = EPD_2_9_lut_conv(list->voltages, flags);
        int len = list->length;
        while (len > 0) {
            int plen = len > 15 ? 15 : len;
            if (pos == 20) {
                LOGE(TAG, "lut overflow.");
                return -1; // full
            }
            lut[pos] = voltages;
            if ((pos & 1) == 0) lut[20+(pos >> 1)] = plen;
            else lut[20+(pos >> 1)] |= plen << 4;
            len -= plen;
            pos++;
        }

        list = &list[1];
    }

    // the GDEH029A01 needs an empty update cycle at the end.
    if (pos == 20) {
        LOGE(TAG, "lut overflow.");
        return -1; // full
    }
    if ((pos & 1) == 0) lut[20+(pos >> 1)] = 1;
    else lut[20+(pos >> 1)] |= 1 << 4;

#ifdef CONFIG_SHA_EPD_2_9_LUT_DEBUG
    {
        LOGD(TAG, "LUT dump:");
        char line[10*3 + 1];
        char *lptr = line;
        int i;
        for (i=0; i<30; i++) {
            sprintf(lptr, " %02x", lut[i]);
            lptr = &lptr[3];
            if ((i % 10) == 9) {
                LOGD(TAG, "%s", line);
                lptr = line;
            }
        }
    }
#endif // CONFIG_SHA_EPD_2_9_LUT_DEBUG

    return 30;
}

// DEPG0290B01
// ------------------------------------------------------------------------------------------------------------------
int EPD_2_9_lut_generate_depg0290b1(const struct EPD_2_9_lut_entry *list, enum EPD_2_9_lut_flags flags, uint8_t *lut)
{
    LOGD(TAG, "DEPG flags = %d.", flags);

    memset(lut, 0, 70);

    int pos = 0;
    int spos = 0;
    while (list->length != 0) {
        int len = list->length;
        if (pos == 7) {
            LOGE(TAG, "lut overflow.");
            return -1; // full
        }
        uint8_t voltages = EPD_2_9_lut_conv(list->voltages, flags);

        lut[0*7 + pos] |= ((voltages >> 0) & 3) << ((3-spos)*2);
        lut[1*7 + pos] |= ((voltages >> 2) & 3) << ((3-spos)*2);
        lut[2*7 + pos] |= ((voltages >> 4) & 3) << ((3-spos)*2);
        lut[3*7 + pos] |= ((voltages >> 6) & 3) << ((3-spos)*2);
        lut[5*7 + pos*5 + spos] = len;
        lut[5*7 + pos*5 + spos] = len;

        spos++;
        if (spos == 2) {
            spos = 0;
            pos++;
        }

        list = &list[1];
    }

#ifdef CONFIG_SHA_EPD_2_9_LUT_DEBUG
    {
        LOGD(TAG, "LUT dump:");
        char line[3*7+1];
        char *lptr = line;
        int i;
        for (i=0; i<35; i++) {
            sprintf(lptr, " %02x", lut[i]);
            lptr = &lptr[3];
            if ((i % 7) == 6) {
                LOGD(TAG, "%s", line);
                lptr = line;
            }
        }
        for (; i<70; i++) {
            sprintf(lptr, " %02x", lut[i]);
            lptr = &lptr[3];
            if ((i % 5) == 4) {
                LOGD(TAG, "%s", line);
                lptr = line;
            }
        }
    }
#endif // CONFIG_SHA_EPD_2_9_LUT_DEBUG

    return 70;
}

//--------------------------------------------------------------------------------------------------------
int EPD_2_9_lut_generate(const struct EPD_2_9_lut_entry *list, enum EPD_2_9_lut_flags flags, uint8_t *lut)
{
    LOGD(TAG, "LUT generate: flags = %01X", flags);
    if (EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_GDEH) return EPD_2_9_lut_generate_gdeh029a1(list, flags, lut);
    if (EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) return EPD_2_9_lut_generate_depg0290b1(list, flags, lut);
    return 0;
}

//---------------------------------------------------------------------
static void memcpy_u32(uint32_t *dst, const uint32_t *src, size_t size)
{
    while (size-- > 0) {
        *dst++ = *src++;
    }
}

//----------------------------------------------------------------
static void memset_u32(uint32_t *dst, uint32_t value, size_t size)
{
    while (size-- > 0) {
        *dst++ = value;
    }
}

//----------------------------------------------------------------------------------------------------
static void EPD_2_9_create_bitplane(const uint8_t *img, uint32_t *buf, int bit, EPD_2_9_flags_t flags)
{
#ifdef EPD_ROTATED_180
    flags ^= DISPLAY_FLAG_ROTATE_180;
#endif
    int x, y;
    int pos, dx, dy;
    if (flags & DISPLAY_FLAG_ROTATE_180) {
        pos = DISP_SIZE_Y-1;
        dx = DISP_SIZE_Y;
        dy = -DISP_SIZE_Y*DISP_SIZE_X - 1;
    }
    else {
        pos = (DISP_SIZE_X-1)*DISP_SIZE_Y;
        dx = -DISP_SIZE_Y;
        dy = DISP_SIZE_Y*DISP_SIZE_X + 1;
    }
    if (!(flags & DISPLAY_FLAG_8BITPIXEL)) {
        memcpy((uint8_t *)buf, img, DISP_SIZE_Y*DISP_SIZE_X/8);
        return;
    }
    for (y = 0; y < DISP_SIZE_Y; y++) {
        for (x = 0; x < DISP_SIZE_X;) {
            int x_bits;
            uint32_t res = 0;
            for (x_bits=0; x_bits<32; x_bits++)
            {
                res <<= 1;
                if (flags & DISPLAY_FLAG_8BITPIXEL) {
                    uint8_t pixel = img[pos];
                    pos += dx;
                    int j = xlat_curve[pixel];
                    if ((j & bit) != 0)
                        res++;
                }
                else {
                    uint8_t pixel = img[pos >> 3] >> (pos & 7);
                    pos += dx;
                    if ((pixel & 1) != 0)
                        res++;
                }
                x++;
            }
            *buf++ = res;
        }
        pos += dy;
    }
}

//------------------------------------------------------------------------------------------------
static void EPD_2_9_set_ram_area(uint8_t x_start, uint8_t x_end, uint16_t y_start, uint16_t y_end)
{
    // set RAM X - address Start / End position
    EPD_WriteCMD_p2(0x44, x_start, x_end);
    // set RAM Y - address Start / End position
    EPD_WriteCMD_p4(0x45, y_start & 0xff, y_start >> 8, y_end & 0xff, y_end >> 8);
}

//------------------------------------------------------------------
static void EPD_2_9_set_ram_pointer(uint8_t x_addr, uint16_t y_addr)
{
    // set RAM X address counter
    EPD_WriteCMD_p1(0x4e, x_addr);
    // set RAM Y address counter
    EPD_WriteCMD_p2(0x4f, y_addr & 0xff, y_addr >> 8);
}

//-----------------------------------------------------
static void EPD_2_9_write_bitplane(const uint32_t *buf)
{
    EPD_2_9_set_ram_area(0, DISP_SIZE_X_B - 1, 0, DISP_SIZE_Y - 1);
    EPD_2_9_set_ram_pointer(0, 0);
    EPD_Write_command_stream32(0x24, buf, DISP_SIZE_X_B * DISP_SIZE_Y/4);
    EPD_WaitReady();
}

//-----------------------------------------------------------------------------
void EPD_2_9_update(const uint32_t *buf, const struct EPD_2_9_update *upd_conf)
{
    // generate lut data
    const struct EPD_2_9_lut_entry *lut_entries;

    if (upd_conf->lut == EPD_2_9_LUT_CUSTOM) {
        LOGD(TAG, "update: custom LUT");
        lut_entries = upd_conf->lut_custom;
    }
    else if ((upd_conf->lut >= 0) && (upd_conf->lut <= EPD_2_9_LUT_MAX)) {
        LOGD(TAG, "update: LUT %d", upd_conf->lut);
        const struct EPD_2_9_lut_entry *lut_lookup[EPD_2_9_LUT_MAX + 1] = {
            EPD_2_9_lut_full,
            EPD_2_9_lut_normal,
            EPD_2_9_lut_faster,
            EPD_2_9_lut_fastest,
        };
        lut_entries = lut_lookup[upd_conf->lut];
    }
    else {
        LOGD(TAG, "update: default LUT (full)");
        lut_entries = EPD_2_9_lut_full;
    }

    uint8_t lut[EPD_2_9_LUT_MAX_SIZE];
    int lut_len = EPD_2_9_lut_generate(lut_entries, upd_conf->lut_flags, lut);
    if (lut_len < 0 ) {
        LOGE(TAG, "Error generating LUT");
        return;
    }

    EPD_Write_command_stream(0x32, lut, lut_len);
    EPD_WaitReady();

    if (buf == NULL) {
        buf = EPD_2_9_tmpbuf;
    }

    // Write the new display data
    EPD_2_9_write_bitplane(buf);

    if ((EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) && EPD_2_9_have_oldbuf) {
        // For DEFG display write the old display data
        EPD_Write_command_stream32(0x26, EPD_2_9_oldbuf, DISP_SIZE_X_B * DISP_SIZE_Y/4);
    }

    // ==== Execute Display update ====

    // write number of overscan lines
    EPD_WriteCMD_p1(0x3a, upd_conf->reg_0x3a);
    // write time to write every line
    EPD_WriteCMD_p1(0x3b, upd_conf->reg_0x3b);
    uint16_t y_len = upd_conf->y_end - upd_conf->y_start;
    // configure length of update
    EPD_WriteCMD_p3(0x01, y_len & 0xff, y_len >> 8, 0x00);
    // configure starting-line of update
    EPD_WriteCMD_p2(0x0f, upd_conf->y_start & 0xff, upd_conf->y_start >> 8);
    // bit mapped enabled phases of the update: (in this order)
    //   80 - enable clock signal
    //   40 - enable CP
    //   20 - load temperature value
    //   10 - load LUT
    //   08 - initial display
    //   04 - pattern display
    //   02 - disable CP
    //   01 - disable clock signal
    EPD_WriteCMD_p1(0x22, 0xc7);

    // start update
    EPD_WriteCMD(0x20);

    if (EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) {
        // For DEFG display set the old display data
        memcpy_u32(EPD_2_9_oldbuf, buf, DISP_SIZE_X_B * DISP_SIZE_Y/4);
    }
    EPD_2_9_have_oldbuf = true;
    EPD_WaitReady();
}

//-------------------------------------------------------------
void EPD_2_9_display(const uint8_t *img, EPD_2_9_flags_t flags)
{
    int lut_mode = (flags >> DISPLAY_FLAG_LUT_BIT) & ((1 << DISPLAY_FLAG_LUT_SIZE)-1);

    uint32_t *buf = EPD_2_9_tmpbuf;
    if (img == NULL) {
        memset_u32(buf, 0, DISP_SIZE_X_B * DISP_SIZE_Y/4);
    }
    else {
        //EPD_2_9_create_bitplane(img, buf, 0x80, flags);
        buf = (uint32_t *)img;
    }

    if ((flags & DISPLAY_FLAG_NO_UPDATE) != 0) return;

    int lut_flags = 0;
    if (!EPD_2_9_have_oldbuf || (flags & DISPLAY_FLAG_FULL_UPDATE)) {
        // old image not known (or full update requested); do full update
        lut_flags |= LUT_FLAG_FIRST;
    }
    else if (lut_mode - 1 != EPD_2_9_LUT_FULL) {
        // old image is known; prefer to do a partial update
        lut_flags |= LUT_FLAG_PARTIAL;
    }

    struct EPD_2_9_update eink_upd = {
        .lut       = lut_mode > 0 ? lut_mode - 1 : EPD_2_9_LUT_DEFAULT,
        .lut_flags = lut_flags,
        .reg_0x3a  = 26,   // 26 dummy lines per gate
        .reg_0x3b  = 0x08, // 62us per line
        .y_start   = 0,
        .y_end     = 295,
    };
    EPD_2_9_update(buf, &eink_upd);
}

//-----------------------------------------------------------------------------------
void EPD_2_9_display_greyscale(const uint8_t *img, EPD_2_9_flags_t flags, int layers)
{
    // start with black.
    EPD_2_9_display(NULL, flags | DISPLAY_FLAG_FULL_UPDATE);

    // the max. number of layers. more layers will result in more ghosting
    if ((EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) && (layers > 5)) {
        layers = 5;
    }
    else if ((EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_GDEH) && (layers > 7)) {
        layers = 7;
    }

    int p_ini = (EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) ? 4 : 16;

    EPD_2_9_have_oldbuf = false;

    int layer;
    uint8_t *imgbuf;
    for (layer = 0; layer < layers; layer++) {
        imgbuf = (uint8_t *)img + (layer * 4756);
        int bit = 128 >> layer;
        int t = bit;
        // gdeh: 128, 64, 32, 16, 8, 4, 2
        // depg: 128, 64, 32, 16, 8

        int p = p_ini;

        while ((t & 1) == 0 && (p > 1)) {
            t >>= 1;
            p >>= 1;
        }

        if ((EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) && (EPD_2_9_have_oldbuf == false) && (p == 1) && (t > 1) && ((layer+1) < layers)) {
            //EPD_2_9_create_bitplane(img, EPD_2_9_oldbuf, bit, flags);
            memcpy((uint8_t *)EPD_2_9_oldbuf, imgbuf, 4756);
            EPD_2_9_have_oldbuf = true;
            continue;
        }

        int j;
        for (j = 0; j < p; j++) {
            int y_start = 0 + j * (DISP_SIZE_Y / p);
            int y_end = y_start + (DISP_SIZE_Y / p) - 1;

            uint32_t *buf = EPD_2_9_tmpbuf;
            //EPD_2_9_create_bitplane(img, buf, bit, flags);
            memcpy((uint8_t *)buf, imgbuf, 4756);

            // clear borders
            memset_u32(buf, 0, y_start * DISP_SIZE_X_B/4);
            memset_u32(&buf[(y_end+1) * DISP_SIZE_X_B/4], 0, (DISP_SIZE_Y-y_end-1) * DISP_SIZE_X_B/4);

            struct EPD_2_9_lut_entry lut[4];

            if (EPD_2_9_have_oldbuf) {
                // LUT:
                //   Use old state as previous layer;
                //   Do nothing when bits are not set;
                //   Make pixel whiter when bit is set;
                //   Duration is <t> cycles.
                lut[0].length = t;
                lut[0].voltages = 0x80;
                lut[1].length = t;
                lut[1].voltages = 0xa0;
                lut[2].length = t;
                lut[2].voltages = 0xa8;
                lut[3].length = 0;

            }
            else {
                // LUT:
                //   Ignore old state;
                //   Do nothing when bit is not set;
                //   Make pixel whiter when bit is set;
                //   Duration is <t> cycles.
                lut[0].length = t;
                lut[0].voltages = 0x88;
                lut[1].length = 0;
            }

            // update display
            struct EPD_2_9_update eink_upd = {
                .lut        = EPD_2_9_LUT_CUSTOM,
                .lut_custom = lut,
                .reg_0x3a   = 0, // no dummy lines per gate
                .reg_0x3b   = 0, // 30us per line
                .y_start    = y_start,
                .y_end      = y_end + 1,
            };
            EPD_2_9_update(buf, &eink_upd);
            EPD_2_9_have_oldbuf = false;
        }
    }
}

//---------------------------
void EPD_2_9_deep_sleep(void)
{
    // enter deep sleep
    EPD_WriteCMD_p1(0x10, 0x01);
}

//-----------------------
void EPD_2_9_wakeup(void)
{
    // leave deep sleep
    EPD_WriteCMD_p1(0x10, 0x00);
}

//================================
int EPD_2_9_init(uint8_t dev_type)
{
    EPD_2_9_dev_type = dev_type;
    // allocate buffers
    if (EPD_2_9_tmpbuf == NULL) {
        EPD_2_9_tmpbuf = pvPortMalloc(DISP_SIZE_X_B * DISP_SIZE_Y);
        if (EPD_2_9_tmpbuf == NULL) return -1;
    }

    if (EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) {
        if (EPD_2_9_oldbuf == NULL) {
            EPD_2_9_oldbuf = pvPortMalloc(DISP_SIZE_X_B * DISP_SIZE_Y);
            if (EPD_2_9_oldbuf == NULL) return -1;
        }
    }
    LOGD(TAG, "HW init");

    // Hardware reset
    EPD_Reset();
    // Software reset
    EPD_WriteCMD(0x12);
    EPD_WaitReady();
    if (EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_GDEH) {
        // initialize GDEH029A1
        // 0C: booster soft start control
        EPD_WriteCMD_p3(0x0c, 0xd7, 0xd6, 0x9d);
        // 2C: write VCOM register
        EPD_WriteCMD_p1(0x2c, 0xa8); // VCOM 7c
        // 11: data entry mode setting
        EPD_WriteCMD_p1(0x11, 0x03); // X inc, Y dec
    }
    if (EPD_2_9_dev_type == DISP_TYPE_EPD_2_9_DEPG) {
        // initialize DEPG0290B01
        // Set analog block control
        EPD_WriteCMD_p1(0x74, 0x54);
        // Set digital block control
        EPD_WriteCMD_p1(0x7E, 0x3B);
        // Set display size and driver output control
        EPD_WriteCMD_p3(0x01, 0x27, 0x01, 0x00);
        // 11: data entry mode setting
        EPD_WriteCMD_p1(0x11, 0x03); // X inc, Y dec
        // Set RAM X address (00h to 0Fh)
        EPD_WriteCMD_p2(0x44, 0x00, 0x0F);
        // Set RAM Y address (0127h to 0000h)
        EPD_WriteCMD_p4(0x45, 0x00, 0x00, 0x27, 0x01);
        // Set border waveform for VBD (see datasheet)
        EPD_WriteCMD_p1(0x3C, 0x01);
        // SET VOLTAGE
        // Set VCOM value
        EPD_WriteCMD_p1(0x2C, 0x26);
        // Gate voltage setting (17h = 20 Volt, ranges from 10v to 21v)
        EPD_WriteCMD_p1(0x03, 0x17);
        // Source voltage setting (15volt, 0 volt and -15 volt)
        EPD_WriteCMD_p3(0x04, 0x41, 0x00, 0x32);
    }
    EPD_WaitReady();

    LOGD(TAG, "HW init done");

    return 0;
}

#endif

#endif
