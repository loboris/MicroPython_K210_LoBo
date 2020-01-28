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

#ifndef _DVP_CAMERA_H
#define _DVP_CAMERA_H

#include <stdio.h>
#include "mphalport.h"

#define DVP_JPEG_BUF_AISRAM_SIZE    0x200000
#define DVP_JPEG_BUF_SIZE           (512*1024)

#define CAMERA_XCLK_FREQUENCY       (26000000)

enum _data_for
{
    DATA_FOR_AI = 0,
    DATA_FOR_DISPLAY = 1,
} ;

extern const int resolution[][2];

/* ===== Sensor abstraction layer ===================================================== */

#define OV7725_SLV_ADDR       (0x42)
#define OV2640_SLV_ADDR       (0x60)
#define MT9V034_SLV_ADDR      (0xB8)
#define LEPTON_SLV_ADDR       (0x54)
#define OV5640_SLV_ADDR       (0x78)

#define OV_CHIP_ID      (0x0A)
#define OV5640_CHIP_ID  (0x300A)
#define ON_CHIP_ID      (0x00)

#define OV9650_ID       (0x96)
#define OV2640_ID       (0x26)
#define OV7725_ID       (0x77)
#define OV5640_ID       (0x56)
#define MT9V034_ID      (0x13)
#define LEPTON_ID       (0x54)

#define IM_MAX(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define IM_MIN(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define IM_DIV(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a / _b) : 0; })
#define IM_MOD(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a % _b) : 0; })

typedef enum {
    SENSORTYPE_INVALID = 0,
    SENSORTYPE_AUTO,
    SENSORTYPE_OV2640,
    SENSORTYPE_OV5640,
} sensortypes_t;

typedef enum {
    PIXFORMAT_INVALID = 0,
    PIXFORMAT_RGB565,    // 2BPP/RGB565
    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
} pixformat_t;

typedef enum {
    FRAMESIZE_INVALID = 0,
    // C/SIF Resolutions
    FRAMESIZE_QQCIF,    // 88x72 x
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_CIF,      // 352x288 x
    FRAMESIZE_QQSIF,    // 88x60 x
    FRAMESIZE_QSIF,     // 176x120
    FRAMESIZE_SIF,      // 352x240 x
    // VGA Resolutions
    FRAMESIZE_QQQQVGA,  // 40x30 x
    FRAMESIZE_QQQVGA,   // 80x60 x
    FRAMESIZE_QQVGA,    // 160x120 x
    FRAMESIZE_QVGA,     // 320x240 *
    FRAMESIZE_VGA,      // 640x480 *
    FRAMESIZE_HQQQVGA,  // 60x40
    FRAMESIZE_HQQVGA,   // 120x80 x
    FRAMESIZE_HQVGA,    // 240x160
    FRAMESIZE_HVGA,     // 480x320
    // FFT Resolutions
    FRAMESIZE_64X32,    // 64x32 x
    FRAMESIZE_64X64,    // 64x64 x
    FRAMESIZE_128X64,   // 128x64 x
    FRAMESIZE_128X128,  // 128x128
    // Other
    FRAMESIZE_LCD,      // 128x160
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_WVGA,     // 720x480 x
    FRAMESIZE_WVGA2,    // 752x480 x
    FRAMESIZE_SVGA,     // 800x600 x
    FRAMESIZE_XGA,      // 1024x768 x
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_720P,     // 1280x720 *
    FRAMESIZE_1080P,    // 1920x1080 *
    FRAMESIZE_960P,     // 1280x960 *
    FRAMESIZE_5MPP,     // 2592x1944 *
} framesize_t;

typedef struct _sensor sensor_t;
typedef struct _sensor {
    uint8_t  sensor_type;           // Sensor type
    uint8_t  slv_addr;              // Sensor I2C slave address.
    void     *gram0;                // pointer to frame buffer #1
    void     *gram1;                // pointer to frame buffer #2
    uint32_t gram_size;             // frame buffer size
    uint8_t  gram_mux;              // active frame buffer switcher
    handle_t dvp_handle;            // handle to DVP device
    handle_t scpp_handle;           // handle to SCCP
    handle_t scpp_dev_handle;       // handle to SCCP device
    uint32_t frame_count;
    void     *irq_func;             // pointer to the DVP interrupt handler
    void     *irq_func_data;        // DVP interrupt handler data
    const uint16_t *color_palette;  // Color palette used for color lookup.

    // Sensor state
    uint16_t  chip_id;              // Sensor ID.
    uint16_t  manuf_id;             // Sensor manufacturer ID.
    pixformat_t pixformat;          // Pixel format
    framesize_t framesize;          // Frame size
    uint8_t night_mode;             //
    bool is_init;                   // true if the sensor was initialized
    bool dvp_is_enabled;            // true if DVP operation is active
    bool dvp_finish_flag;           // set to true when the frame was received
    uint32_t xclk;                  // actual XCLK frequency

    // Sensor function pointers
    int  (*reset)               ();
    int  (*sleep)               (int enable);
    int  (*read_reg)            (uint16_t reg_addr);
    int  (*read_id)             (uint16_t *manuf_id, uint16_t *device_id);
    int  (*write_reg)           (uint16_t reg_addr, uint16_t reg_data);
    int  (*set_pixformat)       (pixformat_t pixformat);
    int  (*set_framesize)       (uint8_t framesize);
    int  (*check_framesize)     (uint8_t framesize);
    int  (*set_contrast)        (int level);
    int  (*set_brightness)      (int level);
    int  (*set_saturation)      (int level);
    int  (*set_quality)         (int quality);
    int  (*set_colorbar)        (int enable);
    int  (*set_hmirror)         (int enable);
    int  (*set_exposure)        (int exposure);
    int  (*set_vflip)           (int enable);
    int  (*set_special_effect)  (uint8_t sde);
    int  (*set_light_mode)      (uint8_t mode);
    int  (*set_night_mode)      (int enable);
    int  (*deinit)              ();
} sensor_t;

extern const uint16_t dvp_cam_resolution[][2];

int ov2640_init(sensor_t *sensor);
int ov2640_check_framesize(uint8_t framesize);
int ov5640_init(sensor_t *sensor);
int ov5640_check_framesize(uint8_t framesize);

bool dvp_init(sensor_t *sensor);
void dvp_config_size(sensor_t *sensor);
void dvp_deinit(sensor_t *sensor);
void dvp_enable(sensor_t *sensor);
void dvp_disable(sensor_t *sensor);
int dvp_check_framesize(uint8_t type, uint8_t framesize);

#endif
