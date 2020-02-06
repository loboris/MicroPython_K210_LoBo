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

#include <sys/unistd.h>
#include <stdlib.h>
#include <string.h>
#include <devices.h>
#include "syslog.h"
#include "dvp_camera.h"

static sensor_t *last_sensor = NULL;

static const char *TAG = "[DVP]";

// Resolution table
//----------------------------------------
const uint16_t dvp_cam_resolution[][2] = {
    {0,    0   },
    // C/SIF Resolutions
    {88,   72  },    /* QQCIF     */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {88,   60  },    /* QQSIF     */
    {176,  120 },    /* QSIF      */
    {352,  240 },    /* SIF       */
    // VGA Resolutions
    {40,   30  },    /* QQQQVGA   */
    {80,   60  },    /* QQQVGA    */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {640,  480 },    /* VGA       */
    {60,   40  },    /* HQQQVGA   */
    {120,  80  },    /* HQQVGA    */
    {240,  160 },    /* HQVGA     */
    {480,  320 },    /* HVGA      */
    // FFT Resolutions
    {64,   32  },    /* 64x32     */
    {64,   64  },    /* 64x64     */
    {128,  64  },    /* 128x64    */
    {128,  128 },    /* 128x64    */
    // Other
    {128,  160 },    /* LCD       */
    {128,  160 },    /* QQVGA2    */
    {720,  480 },    /* WVGA      */
    {752,  480 },    /* WVGA2     */
    {800,  600 },    /* SVGA      */
    {1024, 768 },    /* XGA       */
    {1280, 1024},    /* SXGA      */
    {1600, 1200},    /* UXGA      */
    {1280, 720},     /* 720P      */
    {1920, 1080},    /* 1080P     */
    {1280, 960},     /* 960P      */
    {2592, 1944},    /* 5MP       */
};

//------------------------------------------
static void sensor_restart(sensor_t *sensor)
{
    dvp_set_signal(sensor->dvp_handle, DVP_SIG_POWER_DOWN, 1);
    dvp_set_signal(sensor->dvp_handle, DVP_SIG_RESET, 0);
    mp_hal_delay_ms(20);
    dvp_set_signal(sensor->dvp_handle, DVP_SIG_RESET, 1);
    dvp_set_signal(sensor->dvp_handle, DVP_SIG_POWER_DOWN, 0);
    // Delay from Reset pull high to SCCB initialization
    mp_hal_delay_ms(30);
}

//-------------------------------
void dvp_deinit(sensor_t *sensor)
{
    /*if ((last_sensor != NULL) && (last_sensor != sensor)) {
        LOGD(TAG, "Deinit last sensor");
        dvp_deinit(last_sensor);
        last_sensor = NULL;
    }*/

    dvp_disable(sensor);
    mp_hal_delay_ms(10);
    if (sensor->deinit) sensor->deinit();
    sensor->is_init = false;
    if (sensor->dvp_handle) {
        LOGD(TAG, "DVP device deinit");
        dvp_set_on_frame_event(sensor->dvp_handle, NULL, NULL);
        io_close(sensor->dvp_handle);
        sensor->dvp_handle = 0;
    }
}

//------------------------------------
void dvp_config_size(sensor_t *sensor)
{
    // === Configure DVP  frame buffer(s) ===
    uint16_t _width = dvp_cam_resolution[sensor->framesize][0];
    uint16_t _height = dvp_cam_resolution[sensor->framesize][1];
    sensor->gram_size = _width * _height;

    if (sensor->pixformat == PIXFORMAT_JPEG) {
        // in JPEG mode '_height' represents the maximal expected jpeg file size
        if (sensor->sensor_type == SENSORTYPE_OV5640) _width /= 2;
        else _height /= 2;
        //sensor->gram_size = _width * _height;
        if (dvp_cam_resolution[sensor->framesize][0] < 640) sensor->gram_size = 128 * 1024;
        else if (dvp_cam_resolution[sensor->framesize][0] < 1280) sensor->gram_size = 256 * 1024;
        else if (dvp_cam_resolution[sensor->framesize][0] < 2000) sensor->gram_size = 512 * 1024;
        else sensor->gram_size = 1024 * 1024;
    }
    else sensor->gram_size *= 2; // RGB565 uses two bytes per pixel
    if (sensor->gram_size < 65536) sensor->gram_size = 65536;
    sensor->gram_size = ((sensor->gram_size / 8) * 8) + 8; // 8 bytes aligned
    sensor->gram_size -= 8;

    // Configure DVP
    dvp_config(sensor->dvp_handle, _width, _height, false);
    LOGD(TAG, "Frame buffer size: %u", sensor->gram_size);
}

// Initialize DVP hardware interface and camera sensor.
//-----------------------------
bool dvp_init(sensor_t *sensor)
{
    // === Open DVP device ===
    if (sensor->dvp_handle == 0) {
        sensor->dvp_handle = io_open("/dev/dvp0");
        if (sensor->dvp_handle == 0) {
            LOGE(TAG, "Error opening DVP device");
            dvp_deinit(sensor);
            return false;
        }
    }

    // Disable DVP output for now
    dvp_set_output_enable(sensor->dvp_handle, DATA_FOR_AI, false);
    dvp_set_output_enable(sensor->dvp_handle, DATA_FOR_DISPLAY, false);
    // Disable DVP events
    dvp_set_on_frame_event(sensor->dvp_handle, NULL, NULL);
    dvp_set_frame_event_enable(sensor->dvp_handle, VIDEO_FE_END, false);
    dvp_set_frame_event_enable(sensor->dvp_handle, VIDEO_FE_BEGIN, false);

    sensor->dvp_is_enabled = false;

    // hard reset camera
    sensor_restart(sensor);

    // === Initialize the camera ===
    if (!sensor->is_init) {
        switch (sensor->sensor_type)
        {
            case SENSORTYPE_OV2640:
                if (ov2640_init(sensor) != 0) {
                    dvp_deinit(sensor);
                    return false;
                }
                break;
            case SENSORTYPE_OV5640:
                if (ov5640_init(sensor) != 0) {
                    dvp_deinit(sensor);
                    return false;
                }
                break;
            case SENSORTYPE_AUTO:
                LOGD(TAG, "Camera auto detect");
                if (ov2640_init(sensor) != 0) {
                    if (ov5640_init(sensor) != 0) {
                        dvp_deinit(sensor);
                        return false;
                    }
                    LOGD(TAG, "OV5640");
                    sensor->sensor_type = SENSORTYPE_OV5640;
                }
                else {
                    LOGD(TAG, "OV2640");
                    sensor->sensor_type = SENSORTYPE_OV2640;
                }
                break;
            default:
                LOGE(TAG, "Unsupported camera type");
                dvp_deinit(sensor);
                return false;
        }
    }

    // Set DVP clock
    if (sensor->sensor_type == SENSORTYPE_OV5640)
        sensor->xclk = (uint32_t)dvp_xclk_set_clock_rate(sensor->dvp_handle, CAMERA_XCLK_FREQUENCY);
    else
        sensor->xclk = (uint32_t)dvp_xclk_set_clock_rate(sensor->dvp_handle, CAMERA_XCLK_FREQUENCY1);
    LOGD(TAG, "DVP clock: %u", sensor->xclk);

    // === Reset and initialize the camera ===
    sensor->reset();
    if (dvp_check_framesize(sensor->sensor_type, sensor->framesize) != 0) {
        LOGE(TAG, "Unsupported frame size");
        dvp_deinit(sensor);
        return false;
    }
    sensor->set_pixformat(sensor->pixformat);
    //sensor->set_framesize(sensor->framesize);

    // Configure DVP frame size
    dvp_config_size(sensor);
    // Assign DVP on event interrupt function
    if (sensor->irq_func) {
        // IRQ function defined for sensor
        dvp_set_on_frame_event(sensor->dvp_handle, sensor->irq_func, sensor->irq_func_data);
    }
    else dvp_set_on_frame_event(sensor->dvp_handle, NULL, NULL);
    LOGI(TAG, "Manuf_id: 0x%04x, Device_id: 0x%04x", sensor->manuf_id, sensor->chip_id);

    last_sensor = sensor;

    return true;
}

//-------------------------------
void dvp_enable(sensor_t *sensor)
{
    if (!sensor->dvp_is_enabled) {
        sensor->gram_mux = 0;
        sensor->dvp_finish_flag = false;
        if (sensor->dvp_handle) {
            // Set initial DVP attributes
            dvp_set_output_attributes(sensor->dvp_handle, DATA_FOR_DISPLAY, VIDEO_FMT_RGB565, sensor->gram0);
            //dvp_set_output_enable(sensor->dvp_handle, DATA_FOR_AI, true);
            dvp_set_output_enable(sensor->dvp_handle, DATA_FOR_DISPLAY, true);
            dvp_set_frame_event_enable(sensor->dvp_handle, VIDEO_FE_END, true);
            dvp_set_frame_event_enable(sensor->dvp_handle, VIDEO_FE_BEGIN, true);
        }
        sensor->dvp_is_enabled = true;
    }
}

//--------------------------------
void dvp_disable(sensor_t *sensor)
{
    if (sensor->dvp_is_enabled) {
        sensor->dvp_finish_flag = true;
        if (sensor->dvp_handle) {
            dvp_set_frame_event_enable(sensor->dvp_handle, VIDEO_FE_END, false);
            dvp_set_frame_event_enable(sensor->dvp_handle, VIDEO_FE_BEGIN, false);
            dvp_set_output_enable(sensor->dvp_handle, DATA_FOR_AI, false);
            dvp_set_output_enable(sensor->dvp_handle, DATA_FOR_DISPLAY, false);
        }
        sensor->dvp_is_enabled = false;
    }
}

//------------------------------------------------------
int dvp_check_framesize(uint8_t type, uint8_t framesize)
{
    switch (type)
    {
        case SENSORTYPE_OV2640:
            return ov2640_check_framesize(framesize);
            break;
        case SENSORTYPE_OV5640:
            return ov5640_check_framesize(framesize);
            break;
        default:
            return -1;
    }
}
