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

#ifndef _EPD_2_9_H_
#define _EPD2_9_H_

#include "mpconfigport.h"

#if MICROPY_USE_TFT

#if MICROPY_USE_EPD

#define EPD_2_9_DISPLAY_WIDTH  296
#define EPD_2_9_DISPLAY_HEIGHT 128
#define EPD_2_9_BUFFER_SIZE    (EPD_2_9_DISPLAY_WIDTH * EPD_2_9_DISPLAY_HEIGHT)

// the max number of layers used for grey scale
#define EPD_2_9_MAX_LAYERS 16

extern uint32_t *EPD_2_9_tmpbuf;
extern uint32_t *EPD_2_9_oldbuf;

int EPD_2_9_init(uint8_t dev_type);

// EPD_2_9_update 'lut' settings
enum EPD_2_9_lut
{
    EPD_2_9_LUT_CUSTOM  = -1,
    EPD_2_9_LUT_FULL    =  0,
    EPD_2_9_LUT_NORMAL  =  1,
    EPD_2_9_LUT_FASTER  =  2,
    EPD_2_9_LUT_FASTEST =  3,
    EPD_2_9_LUT_DEFAULT = EPD_2_9_LUT_FULL,
    EPD_2_9_LUT_MAX     = EPD_2_9_LUT_FASTEST,
};

// config-settings structure
struct EPD_2_9_update {
    /** lut index */
    int lut;
    /** optional lut flags */
    int lut_flags;
    /** the raw lut data if EPD_2_9_LUT_CUSTOM is selected */
    const struct EPD_2_9_lut_entry *lut_custom;
    /** raw setting for the number of dummy lines */
    int reg_0x3a;
    /** raw setting for the time per line */
    int reg_0x3b;
    /** the start column for partial-screen-updates */
    int y_start;
    /** the end column for partial-screen-updates */
    int y_end;
};

/** refresh the eink display with given config-settings
 * @param buf the raw buffer to write to the screen
 * @param upd_conf the config-settings to use
 */
void EPD_2_9_update(const uint32_t *buf, const struct EPD_2_9_update *upd_conf);

/** EPD_2_9_display flags settings */
typedef int EPD_2_9_flags_t;

// bitmapped flags:
/** the input buffer is 8 bits per pixel instead of 1 bit per pixel */
#define DISPLAY_FLAG_8BITPIXEL    1
/** rotate the output 180 degrees */
#define DISPLAY_FLAG_ROTATE_180   2
/** update internal buffer; use EPD_2_9_update() to push update to screen */
#define DISPLAY_FLAG_NO_UPDATE    4
/** ensure the screen gets a full update */
#define DISPLAY_FLAG_FULL_UPDATE  8

// fields and sizes:
/** the lut is stored in bits 8-11 */
#define DISPLAY_FLAG_LUT_BIT      8
/** the lut is stored in bits 8-11 */
#define DISPLAY_FLAG_LUT_SIZE     4

/** macro for specifying EPD_2_9_flags_t lut type */
#define DISPLAY_FLAG_LUT(x) ((1+(x)) << DISPLAY_FLAG_LUT_BIT)

/**
 * display image on badge
 *
 * @param img contains the image in 1 bit per pixel or 8 bits per pixel
 * @param flags see `EPD_2_9_flags_t`
 */
void EPD_2_9_display(const uint8_t *img, EPD_2_9_flags_t flags);

/**
 * display greyscale image on badge (hack)
 *
 * @param img contains the image in 1 bit per pixel or 8 bits per pixel
 * @param flags see `EPD_2_9_flags_t`
 * @param layers the number of layers used. (the max number of layers depends on te
 *    display type)
 */
void EPD_2_9_display_greyscale(const uint8_t *img, EPD_2_9_flags_t flags, int layers);

/**
 * go in deep sleep mode. this disables the ram in the eink chip. a wake-up is needed
 * to continue using the eink display
 */
void EPD_2_9_deep_sleep(void);

/**
 * wake-up from deep sleep mode.
 */
void EPD_2_9_wakeup(void);


//==============================================================

/** the needed size of the buffer used in EPD_2_9_generate_lut() */
#define EPD_2_9_LUT_MAX_SIZE 70

/** specification of display update instruction */
struct EPD_2_9_lut_entry {
    /** the number of cycles the voltages are held; 0 = end of list */
    uint8_t length;

    /** bitmapped value containing voltages for every (old-bit, new-bit) pair:
     * - bits 0,1: from 0 to 0
     * - bits 2,3: from 0 to 1
     * - bits 4,5: from 1 to 0
     * - bits 6,7: from 1 to 1
     *
     * allowed values:
     * - 0: VSS
     * - 1: VSH
     * - 2: VSL
     */
    uint8_t voltages;
};

/** filters to use on a EPD_2_9_lut_entry structure */
enum EPD_2_9_lut_flags {
    LUT_FLAG_FIRST    = 1, // do not depend on previous image
    LUT_FLAG_PARTIAL  = 2, // do not touch already correct pixels
    LUT_FLAG_WHITE    = 4, // white only
    LUT_FLAG_BLACK    = 8, // black only
};


/**
 * Generate LUT data for specific eink display.
 *
 * @param list screen updata data in 'generic' format.
 * @param flags optional alterations on generated lut data.
 * @param lut output data buffer. should be of size EPD_2_9_LUT_MAX_SIZE.
 * @return lut length. returns -1 on error.
 */
int EPD_2_9_lut_generate(const struct EPD_2_9_lut_entry *list, enum EPD_2_9_lut_flags flags, uint8_t *lut);

/* pre-defined lookup-table display-updates. */

/** full screen update with inverse updates */
extern const struct EPD_2_9_lut_entry EPD_2_9_lut_full[];
/** screen update which just sets the black and white */
extern const struct EPD_2_9_lut_entry EPD_2_9_lut_normal[];
/** same as EPD_2_9_lut_normal but with shorter timings */
extern const struct EPD_2_9_lut_entry EPD_2_9_lut_faster[];
/** same as EPD_2_9_lut_faster but with shorter timings */
extern const struct EPD_2_9_lut_entry EPD_2_9_lut_fastest[];


//=============================================================================================================

/** the number of horizontal pixels
 * @note the display is rotated 90 degrees
 */
#define DISP_SIZE_X 128

/** the number of vertical pixels
 * @note the display is rotated 90 degrees
 */
#define DISP_SIZE_Y 296

/** the number of bytes in a pixel row
 * @note the display is rotated 90 degrees
 */
#define DISP_SIZE_X_B ((DISP_SIZE_X + 7) >> 3)


/** write command with `datalen` data bytes */
void EPD_2_9_dev_write_command_stream(uint8_t command, const uint8_t *data,
                                         unsigned int datalen);

/** write command with `datalen` data dwords */
void EPD_2_9_dev_write_command_stream_u32(uint8_t command, const uint32_t *data,
                                         unsigned int datalen);


#endif // MICROPY_USE_EPD

#endif // MICROPY_USE_TFT

#endif
