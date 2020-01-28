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

#if MICROPY_USE_CAMERA

#include <stdio.h>
#include <string.h>
#include "syslog.h"

#include "py/nlr.h"
#include "py/runtime.h"
#include "py/objstr.h"
#include "py/stream.h"
#include "extmod/vfs.h"
#include "modmachine.h"
#include "dvp_camera.h"
#include "../display/moddisplay.h"

#define USE_DEBUG_PIN   1
#define DEBUG_PIN_NUM   18

#define DEST_BUFFER     0
#define DEST_TFT        1
#define DEST_FILE       2

typedef struct _mod_camera_obj_t {
    mp_obj_base_t       base;
    mp_obj_t            buff_obj0;
    mp_obj_t            buff_obj1;
    sensor_t            sensor;
    SemaphoreHandle_t   semaphore;
    TaskHandle_t        preview_task;
} mod_camera_obj_t;

typedef struct _task_params_t {
    void        *cam_obj;
    void        *thread_handle;
    uint32_t    max_frames;
    uint8_t     disp_fps;
} task_params_t;


extern const mp_obj_type_t mod_camera_type;

#if USE_DEBUG_PIN
static int debug_pin = 0;
#endif

static bool camera_pins_init = false;
static bool camera_is_init = false;
static task_params_t task_params;
static const char *TAG = "[MOD_CAMERA]";

//-----------------------------
static bool cam_pins_init(void)
{
    // Configure display pins
    if (!camera_pins_init) {
        mp_fpioa_cfg_item_t camera_pin_func[8];
        // FPIOA configuration
        camera_pin_func[0] = (mp_fpioa_cfg_item_t){-1, 42, GPIO_USEDAS_DVP_RST, FUNC_CMOS_RST};
        camera_pin_func[1] = (mp_fpioa_cfg_item_t){-1, 44, GPIO_USEDAS_DVP_PWDN, FUNC_CMOS_PWDN};
        camera_pin_func[2] = (mp_fpioa_cfg_item_t){-1, 46, GPIO_USEDAS_DVP_XCLK, FUNC_CMOS_XCLK};
        camera_pin_func[3] = (mp_fpioa_cfg_item_t){-1, 43, GPIO_USEDAS_DVP_VSYNC, FUNC_CMOS_VSYNC};
        camera_pin_func[4] = (mp_fpioa_cfg_item_t){-1, 45, GPIO_USEDAS_DVP_HREF, FUNC_CMOS_HREF};
        camera_pin_func[5] = (mp_fpioa_cfg_item_t){-1, 47, GPIO_USEDAS_DVP_PCLK, FUNC_CMOS_PCLK};
        camera_pin_func[6] = (mp_fpioa_cfg_item_t){-1, 41, GPIO_USEDAS_DVP_SCLK, FUNC_SCCB_SCLK};
        camera_pin_func[7] = (mp_fpioa_cfg_item_t){-1, 40, GPIO_USEDAS_DVP_SDA, FUNC_SCCB_SDA};

        if (!fpioa_check_pins(8, camera_pin_func, GPIO_FUNC_DVP)) {
            return false;
        }
        // Setup and mark used pins
        fpioa_setup_pins(8, camera_pin_func);
        fpioa_setused_pins(8, camera_pin_func, GPIO_FUNC_DVP);

        sysctl_set_spi0_dvp_data(1);

        camera_pins_init = true;
    }

    return true;
}

//-------------------------------
static bool cam_pins_deinit(void)
{
    // Configure display pins
    if (camera_pins_init) {
        mp_fpioa_cfg_item_t camera_pin_func[8];
        // FPIOA configuration
        camera_pin_func[0] = (mp_fpioa_cfg_item_t){-1, 42, GPIO_USEDAS_NONE, FUNC_RESV0};
        camera_pin_func[1] = (mp_fpioa_cfg_item_t){-1, 44, GPIO_USEDAS_NONE, FUNC_RESV0};
        camera_pin_func[2] = (mp_fpioa_cfg_item_t){-1, 46, GPIO_USEDAS_NONE, FUNC_RESV0};
        camera_pin_func[3] = (mp_fpioa_cfg_item_t){-1, 43, GPIO_USEDAS_NONE, FUNC_RESV0};
        camera_pin_func[4] = (mp_fpioa_cfg_item_t){-1, 45, GPIO_USEDAS_NONE, FUNC_RESV0};
        camera_pin_func[5] = (mp_fpioa_cfg_item_t){-1, 47, GPIO_USEDAS_NONE, FUNC_RESV0};
        camera_pin_func[6] = (mp_fpioa_cfg_item_t){-1, 41, GPIO_USEDAS_NONE, FUNC_RESV0};
        camera_pin_func[7] = (mp_fpioa_cfg_item_t){-1, 40, GPIO_USEDAS_NONE, FUNC_RESV0};

        if (!fpioa_check_pins(8, camera_pin_func, GPIO_FUNC_DVP)) {
            return false;
        }
        // Setup and mark used pins
        fpioa_setup_pins(8, camera_pin_func);
        fpioa_freeused_pins(8, camera_pin_func);

        camera_pins_init = false;
    }

    return true;
}

//----------------------------------------------------
static void cam_delete_buffers(mod_camera_obj_t *self)
{
    if (self->buff_obj0 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)self->buff_obj0);
    if (self->buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)self->buff_obj1);
    self->buff_obj0 = mp_const_none;
    self->buff_obj1 = mp_const_none;
}

// Create one frame buffer for capture mode
// Preview mode requires two frame buffers, the 2nd buffer will be created when needed
//---------------------------------------------------
static bool cam_create_buffer(mod_camera_obj_t *self)
{
    mp_obj_array_t *gram0;
    uint32_t old_size = 0;
    if (self->buff_obj0 != mp_const_none) {
        gram0 = (mp_obj_array_t *)self->buff_obj0;
        old_size = gram0->len;
    }

    cam_delete_buffers(self);

    self->buff_obj0 = mp_obj_new_frame_buffer(self->sensor.gram_size+8);
    if (self->buff_obj0 == mp_const_none) {
        if (old_size) {
            self->buff_obj0 = mp_obj_new_frame_buffer(old_size);
            if (self->buff_obj0 != mp_const_none) {
                self->sensor.gram0 = gram0->items + 8;
                self->sensor.gram1 = self->sensor.gram0;
                return false;
            }
        }
        dvp_deinit(&self->sensor);
        m_del_obj(mod_camera_obj_t, self);
        mp_raise_msg(&mp_type_OSError, "Error creating camera frame buffer.");
    }
    gram0 = (mp_obj_array_t *)self->buff_obj0;
    self->sensor.gram0 = gram0->items + 8;
    self->sensor.gram1 = self->sensor.gram0;
    LOGD(TAG, "FB created at %p (buf at %p, size=%lu)", (void *)self->buff_obj0, gram0->items, gram0->len);
    return true;
}

//----------------------------------------------
static void check_camera(mod_camera_obj_t *self)
{
    if (!camera_is_init) {
        mp_raise_msg(&mp_type_OSError, "Camera not initialized");
    }
}

//-----------------------------------------------
static void check_preview(mod_camera_obj_t *self)
{
    check_camera(self);
    if (self->preview_task) {
        mp_raise_msg(&mp_type_OSError, "Cannot execute while in preview mode");
    }
}

//---------------------------------------------
// This function is executed from dvp interrupt
//-------------------------------------------------------------
static void on_dvp_irq(dvp_frame_event_t event, void* userdata)
{
    mod_camera_obj_t *self = (mod_camera_obj_t *)userdata;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (event)
    {
        case VIDEO_FE_BEGIN:
            // Frame start (vsync detected)
            if (!self->sensor.dvp_finish_flag) {
                dvp_enable_frame(self->sensor.dvp_handle);
                #if USE_DEBUG_PIN
                gpio_set_pin_value(gpiohs_handle, debug_pin, 1);
                #endif
            }
            break;
        case VIDEO_FE_END:
            // Frame complete
            if (self->sensor.frame_count) self->sensor.frame_count--;
            self->sensor.gram_mux ^= 0x01; // select next (active) frame buffer
            if (self->sensor.frame_count) {
                dvp_set_output_attributes(self->sensor.dvp_handle, DATA_FOR_DISPLAY, VIDEO_FMT_RGB565, self->sensor.gram_mux ? self->sensor.gram1 : self->sensor.gram0);
            }
            else {
                self->sensor.dvp_finish_flag = true;
                dvp_disable(&self->sensor);
            }

            #if USE_DEBUG_PIN
            gpio_set_pin_value(gpiohs_handle, debug_pin, 0);
            #endif
            if (self->semaphore) {
                xSemaphoreGiveFromISR(self->semaphore, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken)
                {
                    portYIELD_FROM_ISR();
                }
            }
            break;
        default:
            self->sensor.dvp_finish_flag = true;
            self->sensor.frame_count = 0;
            dvp_disable(&self->sensor);
    }
}

//------------------------------------------------
static void _camera_deinit(mod_camera_obj_t *self)
{
    dvp_deinit(&self->sensor);
    cam_delete_buffers(self);
    cam_pins_deinit();

    if (self->semaphore) vSemaphoreDelete(self->semaphore);
    self->semaphore = NULL;
    camera_is_init = false;
}

//------------------------------------------------
static void swap_pixels(uint16_t *cbuff, int size)
{
    uint16_t tmpclr;
    for (int i=0; i<size; i+=2) {
        tmpclr = cbuff[i];
        cbuff[i] = cbuff[i+1];
        cbuff[i+1] = tmpclr;
    }
}

// After initialization capture frames to stabilize the operation
//----------------------------------------------------------
static void init_frames(mod_camera_obj_t *self, bool delete)
{
    self->sensor.frame_count = 10;
    xSemaphoreTake(self->semaphore, 0);

    LOGD(TAG, "Capture %u frames", self->sensor.frame_count);
    vTaskDelay(100/portTICK_PERIOD_MS);
    // Start capturing frames
    dvp_enable(&self->sensor);
    while (self->sensor.frame_count > 0) {
        if (xSemaphoreTake(self->semaphore, 500/portTICK_PERIOD_MS ) != pdTRUE) {
            self->sensor.frame_count = 0;
            dvp_disable(&self->sensor);
            if (delete) {
                _camera_deinit(self);
                m_del_obj(mod_camera_obj_t, self);
            }
            mp_raise_msg(&mp_type_OSError, "Error waiting for initialization frames.");
        }

        mp_hal_wdt_reset();
    }
    dvp_disable(&self->sensor);

    bool show = true;
    if (self->sensor.pixformat == PIXFORMAT_JPEG) show = false;
    if (active_dstate->tft_frame_buffer == NULL) show = false;
    if ((dvp_cam_resolution[self->sensor.framesize][0] > active_dstate->_width) || (dvp_cam_resolution[self->sensor.framesize][1] > active_dstate->_height)) show = false;
    if (show) {
        // Display the last captured frame
        color_t *cbuff = (uint16_t *)((self->sensor.gram_mux) ? self->sensor.gram0 : self->sensor.gram1);
        swap_pixels(cbuff, dvp_cam_resolution[self->sensor.framesize][0] * dvp_cam_resolution[self->sensor.framesize][1]);
        send_data_scale(0, 0, dvp_cam_resolution[self->sensor.framesize][0], dvp_cam_resolution[self->sensor.framesize][1], cbuff, 0);
        send_frame_buffer();
    }
}

//------------------------------------------
static void preview_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    // if the task uses some MicroPython functions, we have to save
    // MicroPython state in local storage pointers
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)task_params->thread_handle, THREAD_LSP_STATE));
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)task_params->thread_handle, THREAD_LSP_ARGS));

    mod_camera_obj_t *self = (mod_camera_obj_t *)task_params->cam_obj;
    uint64_t notify_val = 0;

    self->sensor.frame_count = task_params->max_frames;
    xSemaphoreTake(self->semaphore, 0);

    active_dstate->font_rotate = 0;
    active_dstate->text_wrap = 0;
    active_dstate->font_forceFixed = 0;
    active_dstate->gray_scale = 0;
    _tft_setRotation(LANDSCAPE);
    TFT_resetclipwin();
    active_dstate->_fg = TFT_CYAN;
    TFT_setFont(UBUNTU16_FONT, 0, false);
    active_dstate->font_transparent = 1;
    active_dstate->_bg = TFT_BLACK;
    if (task_params->disp_fps > 1) {
        active_dstate->font_transparent = 0;
        active_dstate->_bg = TFT_DARKGREY;
    }
    char str_tft[32];

    // Start capturing frames
    uint64_t tstart = mp_hal_ticks_ms();
    uint64_t frame_count = 0;
    double fps = 0;

    dvp_enable(&self->sensor);
    while (self->sensor.frame_count > 0) {
        if (xSemaphoreTake(self->semaphore, 500/portTICK_PERIOD_MS ) != pdTRUE) {
            self->sensor.frame_count = 0;
            break;
        }
        frame_count++;
        fps = 1.0 / ((double)(mp_hal_ticks_ms()-tstart) / 1000.0 / (double)frame_count);

        if (active_dstate->tft_frame_buffer) {
            // Display captured frame
            color_t *cbuff = (uint16_t *)((self->sensor.gram_mux) ? self->sensor.gram0 : self->sensor.gram1);
            swap_pixels(cbuff, dvp_cam_resolution[self->sensor.framesize][0] * dvp_cam_resolution[self->sensor.framesize][1]);
            send_data_scale(0, 0, dvp_cam_resolution[self->sensor.framesize][0], dvp_cam_resolution[self->sensor.framesize][1], cbuff, 0);
            if (task_params->disp_fps) {
                sprintf(str_tft, "%0.2f fps", fps);
                TFT_print(str_tft, 5, 5);
                sprintf(str_tft, "%lu", frame_count);
                TFT_print(str_tft, RIGHT, 5);
            }
            send_frame_buffer();
        }

        mp_hal_wdt_reset();

        // Check notification
        notify_val = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notify_val, 0) == pdPASS) {
            if (notify_val == 0xA500) {
                // Terminate task requested
                self->sensor.frame_count = 0;
                break;
            }
        }
    } // task's main loop

    /*
    if ((active_dstate->tft_frame_buffer) && (task_params->disp_fps == 0)) {
        sprintf(str_tft, "%0.2f fps", fps);
        TFT_print(str_tft, 5, 5);
        sprintf(str_tft, "%lu", frame_count);
        TFT_print(str_tft, RIGHT, 5);
        send_frame_buffer();
    }
    */
    printf("Frame rate: %0.2f fps\r\n", fps);

    dvp_disable(&self->sensor);
    // Terminate debounce task
    self->preview_task = NULL;
    vTaskDelete(NULL);
}

//---------------------------------------------------
static void stop_preview_task(mod_camera_obj_t *self)
{
    if (self->preview_task == NULL) return;

    // Terminate preview task
    xTaskNotify(self->preview_task, 0xA500, eSetValueWithoutOverwrite);
    vTaskDelay(10);
    while (self->preview_task) {
        // Terminate the debounce task
        xTaskNotify(self->preview_task, 0xA500, eSetValueWithoutOverwrite);
        vTaskDelay(10);
        // if res == pdPASS, the task has received the notification
        //if (res == pdPASS) break;
    }
    // Free the 2nd frame buffer
    if (self->buff_obj1 != mp_const_none) mp_obj_delete_frame_buffer((mp_obj_array_t *)self->buff_obj1);
    self->buff_obj1 = mp_const_none;
    self->sensor.gram1 = self->sensor.gram0;
}

//--------------------------------------
static void tft_setup(uint8_t time_disp)
{
    active_dstate->font_rotate = 0;
    active_dstate->text_wrap = 0;
    active_dstate->font_forceFixed = 0;
    active_dstate->gray_scale = 0;
    //_tft_setRotation(LANDSCAPE);
    TFT_resetclipwin();

    if (time_disp) {
        active_dstate->font_transparent = 1;
        active_dstate->_bg = TFT_BLACK;
        TFT_setFont(UBUNTU16_FONT, 0, false);

        char str_time[32];
        struct tm *tm_info;
        time_t seconds = _get_time(false);
        tm_info = localtime(&seconds);
        strftime(str_time, 127, "%F %R", tm_info);

        if (time_disp == 1) active_dstate->_fg = TFT_BLACK;
        else {
            active_dstate->_fg = TFT_CYAN;
            active_dstate->_bg = TFT_DARKGREY;
            active_dstate->font_transparent = 0;
        }
        TFT_print(str_time, 5, 5);
        if (time_disp == 1) {
            active_dstate->_fg = TFT_CYAN;
            TFT_print(str_time, 7, 7);
        }
    }
}


// ===== Camera MicroPython bindings =====

//-------------------------------------------------------------------------------------------
STATIC void mod_camera_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    mod_camera_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (camera_is_init) {
        mp_printf(print, "Camera(%s) %dx%d, buffer: %s%u, status: %s",
                (self->sensor.sensor_type == SENSORTYPE_OV2640) ? "OV2640" : "OV5640",
                dvp_cam_resolution[self->sensor.framesize][0], dvp_cam_resolution[self->sensor.framesize][1],
                (self->preview_task) ? "2*" : "", self->sensor.gram_size+8,
                (self->preview_task) ? "Preview" : "Capture");
    }
    else {
        mp_printf(print, "Camera( deinitialized )");
    }
}

//------------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_camera_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    enum { ARG_type };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_type,  MP_ARG_INT,  {.u_int = SENSORTYPE_AUTO} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    //mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    if (camera_is_init) {
        mp_raise_msg(&mp_type_OSError, "Only one camera object can be created");
    }
    if ((args[ARG_type].u_int != SENSORTYPE_AUTO) && (args[ARG_type].u_int != SENSORTYPE_OV2640) && (args[ARG_type].u_int != SENSORTYPE_OV5640)) {
        mp_raise_ValueError("Unsupported camera type.");
    }

    mod_camera_obj_t *self = m_new_obj_with_finaliser(mod_camera_obj_t);
    self->base.type = &mod_camera_type;
    memset(&self->sensor, 0, sizeof(sensor_t));

    if (!cam_pins_init()) {
        m_del_obj(mod_camera_obj_t, self);
        mp_raise_msg(&mp_type_OSError, "Error setting DVP pins.");
    }

    #if USE_DEBUG_PIN
    if (debug_pin == 0) {
        debug_pin = gpiohs_get_free();
        if (debug_pin) {
            if (fpioa_set_function(DEBUG_PIN_NUM, FUNC_GPIOHS0 + debug_pin) == 0) {
                gpio_set_drive_mode(gpiohs_handle, debug_pin, GPIO_DM_OUTPUT);
                gpio_set_pin_value(gpiohs_handle, debug_pin, 0);
            }
            else debug_pin = 0;
        }
    }
    #endif

    // Set configuration
    self->sensor.sensor_type = args[ARG_type].u_int;
    self->sensor.pixformat = PIXFORMAT_RGB565;
    self->sensor.framesize = FRAMESIZE_QVGA;
    self->sensor.irq_func = on_dvp_irq;
    self->sensor.irq_func_data = (void *)self;
    self->sensor.frame_count = 1;
    self->preview_task = NULL;

    LOGD(TAG, "Camera deinit");
    dvp_deinit(&self->sensor);
    mp_hal_delay_ms(20);

    self->semaphore = xSemaphoreCreateBinary();
    if (!self->semaphore) {
        dvp_deinit(&self->sensor);
        m_del_obj(mod_camera_obj_t, self);
        mp_raise_msg(&mp_type_OSError, "Error creating camera semaphore.");
    }

    LOGD(TAG, "Camera init");
    if (!dvp_init(&self->sensor)) {
        _camera_deinit(self);
        m_del_obj(mod_camera_obj_t, self);
        mp_raise_msg(&mp_type_OSError, "Error initializing DVP.");
    }

    // Camera frame buffer is not allocated yet
    dvp_config_size(&self->sensor);
    if (!cam_create_buffer(self)) {
        _camera_deinit(self);
        m_del_obj(mod_camera_obj_t, self);
        mp_raise_msg(&mp_type_OSError, "Error initializing DVP.");
    }

    self->sensor.set_pixformat(PIXFORMAT_RGB565);
    self->sensor.set_framesize(FRAMESIZE_QVGA);

    init_frames(self, true);
    camera_is_init = true;

    return MP_OBJ_FROM_PTR(self);
}

//-------------------------------------------------
STATIC mp_obj_t mod_camera_deinit(mp_obj_t self_in)
{
    mod_camera_obj_t *self = MP_OBJ_TO_PTR(self_in);

    check_camera(self);
    stop_preview_task(self);
    _camera_deinit(self);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_camera_deinit_obj, mod_camera_deinit);

//------------------------------------------------
STATIC mp_obj_t mod_camera_reset(mp_obj_t self_in)
{
    mod_camera_obj_t *self = MP_OBJ_TO_PTR(self_in);

    check_camera(self);
    stop_preview_task(self);

    LOGD(TAG, "Deinitialize camera");
    dvp_deinit(&self->sensor);
    cam_delete_buffers(self);

    LOGD(TAG, "Init with new settings");
    mp_hal_delay_ms(20);

    if (!dvp_init(&self->sensor)) {
        dvp_deinit(&self->sensor);
        cam_pins_deinit();
        if (self->semaphore) vSemaphoreDelete(self->semaphore);
        self->semaphore = NULL;
        camera_is_init = false;
        mp_raise_msg(&mp_type_OSError, "Error initializing DVP.");
    }

    LOGD(TAG, "Create buffers");
    dvp_config_size(&self->sensor);
    if (!cam_create_buffer(self)) {
        _camera_deinit(self);
        m_del_obj(mod_camera_obj_t, self);
        mp_raise_msg(&mp_type_OSError, "Error initializing DVP.");
    }

    self->sensor.set_pixformat(PIXFORMAT_RGB565);
    self->sensor.set_framesize(FRAMESIZE_QVGA);

    init_frames(self, false);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_camera_reset_obj, mod_camera_reset);

//---------------------------------------------------------------------
STATIC mp_obj_t mod_camera_set_size(mp_obj_t self_in, mp_obj_t size_in)
{
    mod_camera_obj_t *self = MP_OBJ_TO_PTR(self_in);

    int size = mp_obj_get_int(size_in);
    if (size != self->sensor.framesize) {
        if (self->sensor.check_framesize(size) != 0) {
            mp_raise_ValueError("Unsupported size.");
        }
        self->sensor.framesize = size;
        dvp_config_size(&self->sensor);
        if (!cam_create_buffer(self)) return mp_const_false;
        self->sensor.set_framesize(size);
    }

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_set_size_obj, mod_camera_set_size);

//---------------------------------------------------------------------------------
static void _restore_mode_size(mod_camera_obj_t *self, int size, uint32_t buf_size)
{
    if ((self->sensor.pixformat!= PIXFORMAT_RGB565) || (self->sensor.framesize != size)) {
        // Change mode or size or both
        self->sensor.pixformat = PIXFORMAT_RGB565;
        self->sensor.framesize = size;

        dvp_config_size(&self->sensor);
        cam_create_buffer(self);

        self->sensor.set_pixformat(PIXFORMAT_RGB565);
        self->sensor.set_framesize(size);
    }
    self->sensor.set_exposure(0); // set auto exposure on
}

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_camera_capture(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_dest, ARG_time, ARG_mode, ARG_size, ARG_qs, ARG_exposure };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_dest,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_time,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_mode,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = PIXFORMAT_RGB565} },
        { MP_QSTR_size,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = FRAMESIZE_INVALID} },
        { MP_QSTR_quality,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_exposure, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    mod_camera_obj_t *self = pos_args[0];

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    check_camera(self);
    stop_preview_task(self);

    #if USE_DEBUG_PIN
    gpio_set_pin_value(gpiohs_handle, debug_pin, 1);
    vTaskDelay(2);
    gpio_set_pin_value(gpiohs_handle, debug_pin, 0);
    #endif

    int mode = args[ARG_mode].u_int;
    int size = self->sensor.framesize;
    if (args[ARG_size].u_int > FRAMESIZE_INVALID) size = args[ARG_size].u_int;
    int exposure = args[ARG_exposure].u_int;

    uint32_t buf_size = self->sensor.gram_size;
    int orig_size = self->sensor.framesize;

    if ((mode != PIXFORMAT_JPEG) && (mode != PIXFORMAT_RGB565)) {
        mp_raise_ValueError("Unsupported mode.");
    }
    //if ((mode == PIXFORMAT_JPEG) && (self->sensor.sensor_type == SENSORTYPE_OV5640) && (size != FRAMESIZE_QVGA)) {
    //    mp_raise_ValueError("Unsupported size for OV5640 jpeg mode.");
    //}
    if (size != self->sensor.framesize) {
        if (self->sensor.check_framesize(size) != 0) {
            mp_raise_ValueError("Unsupported size.");
        }
    }

    if (mode == PIXFORMAT_JPEG) {
        int qs = args[ARG_qs].u_int;
        if ((qs >= 3) && (qs <= 50)) self->sensor.set_quality((uint8_t)qs);
    }

    if (exposure > 0) self->sensor.set_exposure(exposure);
    else {
        exposure = self->sensor.set_exposure(-1); // get current exposure
        if (self->sensor.sensor_type == SENSORTYPE_OV2640) {
            if (dvp_cam_resolution[size][0] > dvp_cam_resolution[FRAMESIZE_SVGA][0]) {
                self->sensor.set_exposure(exposure * 4);
            }
            else self->sensor.set_exposure(-2); // disable auto exposure
        }
        else {
            self->sensor.set_exposure(-2); // disable auto exposure
        }
        LOGD(TAG, "Capture exposure: %d", exposure);
    }

    if ((mode != self->sensor.pixformat) || (size != self->sensor.framesize)) {
        // Change mode or size or both
        self->sensor.pixformat = mode;
        self->sensor.framesize = size;
        dvp_config_size(&self->sensor);
        if (self->sensor.gram_size != buf_size) {
            if (!cam_create_buffer(self)) {
                _restore_mode_size(self, orig_size, buf_size);
                return mp_const_none;
            }
        }
        /*if ((size != orig_size) && (exposure <= 1)) {
            // set size and wait for AE to stabilize
            self->sensor.set_framesize(size);
            mp_hal_delay_ms(160);
        }*/
        if (mode == PIXFORMAT_JPEG) self->sensor.set_pixformat(mode); // this will set the frame size to
        else if (size != orig_size) self->sensor.set_framesize(size); // set frame size for RGB565 mode
    }

    // Get capture dimensions
    uint16_t width = dvp_cam_resolution[self->sensor.framesize][0];
    uint16_t height = dvp_cam_resolution[self->sensor.framesize][1];

    mp_uint_t tstart = mp_hal_ticks_ms();
    // === Start capture ===
    mp_hal_wdt_reset();
    self->sensor.frame_count = 1;
    xSemaphoreTake(self->semaphore, 0);
    dvp_enable(&self->sensor);

    while (self->sensor.frame_count > 0) {
        // wait for frame
        if (xSemaphoreTake(self->semaphore, 2000/portTICK_PERIOD_MS ) != pdTRUE) {
            self->sensor.frame_count = 0;
            dvp_disable(&self->sensor);
            mp_hal_wdt_reset();
            /*_restore_mode_size(self, orig_size, buf_size);*/
            #if USE_DEBUG_PIN
            gpio_set_pin_value(gpiohs_handle, debug_pin, 1);
            vTaskDelay(5);
            gpio_set_pin_value(gpiohs_handle, debug_pin, 0);
            #endif
            //mp_raise_msg(&mp_type_OSError, "Error waiting for frame.");
            break;
        }
        mp_hal_wdt_reset();
    }
    dvp_disable(&self->sensor);
    mp_hal_wdt_reset();

    // === Process captured frame ===
    uint8_t *frame_buffer = (self->sensor.gram_mux) ? self->sensor.gram0 : self->sensor.gram1;
    mp_obj_t result = mp_const_none;
    uint8_t dest = DEST_TFT;
    mp_obj_t ffd = mp_const_none;
    LOGD(TAG, "Frame (%ux%u) captured (%lu ms), processing", width, height, mp_hal_ticks_ms()-tstart);

    if (mp_obj_is_str(args[ARG_dest].u_obj)) {
        dest = DEST_FILE;
        // === capture to file ===
        mp_obj_t fargs[2];
        fargs[0] = args[ARG_dest].u_obj;
        fargs[1] = mp_obj_new_str("wb", 2);

        // Open the file
        ffd = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);
        if (!ffd) {
            _restore_mode_size(self, orig_size, buf_size);
            mp_raise_msg(&mp_type_OSError, "Error opening capture file.");
        }
    }
    else if (args[ARG_dest].u_obj != mp_const_none) {
        int dst = mp_obj_get_int(args[ARG_dest].u_obj);
        if (dst == DEST_BUFFER) dest = DEST_BUFFER;
    }

    if (dest == DEST_TFT) {
        if (active_dstate->tft_frame_buffer == NULL) {
            _restore_mode_size(self, orig_size, buf_size);
            mp_raise_msg(&mp_type_OSError, "TFT frame buffer not initialized");
        }
        memset(active_dstate->tft_frame_buffer, 0, active_dstate->_width * active_dstate->_height * 2);
    }

    if (self->sensor.pixformat == PIXFORMAT_JPEG) {
        // === Process JPEG data ===
        // jpeg data are written to frame buffer as 32-bit values with swapped endianness!
        mp_obj_array_t *gram0;
        gram0 = (mp_obj_array_t *)self->buff_obj0;
        LOGD(TAG, "Swap JPEG frame buffer");
        uint8_t tmpb0, tmpb1;
        for (int i = 0; i < (gram0->len); i+=4) {
            tmpb0 = frame_buffer[i];
            tmpb1 = frame_buffer[i+1];
            frame_buffer[i] = frame_buffer[i+3];
            frame_buffer[i+1] = frame_buffer[i+2];
            frame_buffer[i+2] = tmpb1;
            frame_buffer[i+3] = tmpb0;
        }
        LOGD(TAG, "Check JPEG signature");
        // check jpeg start marker and find jpeg end marker
        uint32_t jpeg_size = 0;
        if ((frame_buffer[0] == 0xff) && (frame_buffer[1] == 0xd8) && (memcmp(frame_buffer+6, "JFIF", 4) == 0)) {
            for (int i = 0; i < ((gram0->len)-1); i++) {
                if ((frame_buffer[i] == 0xff) && (frame_buffer[i+1] == 0xd9)) {
                    jpeg_size = i+2;
                    break;
                }
            }
        }
        else {
            if (ffd != mp_const_none) mp_stream_close(ffd);
            _restore_mode_size(self, orig_size, buf_size);
            mp_raise_msg(&mp_type_OSError, "No jpeg header detected.");
        }
        if (jpeg_size == 0) {
            mp_hal_wdt_reset();
            _restore_mode_size(self, orig_size, buf_size);
            mp_raise_msg(&mp_type_OSError, "No jpeg end marker detected.");
        }
        LOGD(TAG, "JPEG ok, size=%u", jpeg_size);

        if (dest == DEST_BUFFER) {
            result = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)frame_buffer, jpeg_size);
        }
        else if (dest == DEST_TFT) {
            int scale = (width / active_dstate->_width) - 1;
            if ((scale > 0) && ((width % active_dstate->_width) >= (active_dstate->_width/4))) scale++;
            LOGD(TAG, "JPEG -> tft (scale=%d)", scale);
            if (TFT_jpg_image(0, 0, scale, mp_const_none, frame_buffer, jpeg_size)) {
                tft_setup((uint8_t)(args[ARG_time].u_int & 3));
                // show image
                send_frame_buffer();
            }
            else LOGD(TAG, "JPEG decode error");
        }
        else if ((dest == DEST_FILE) && (ffd != mp_const_none)) {
            int written_bytes = mp_stream_posix_write((void *)ffd, frame_buffer, jpeg_size);
            mp_stream_close(ffd);
            if (written_bytes != jpeg_size) {
                _restore_mode_size(self, orig_size, buf_size);
                mp_raise_msg(&mp_type_OSError, "Error writing capture to file");
            }
        }
        else if (ffd != mp_const_none) mp_stream_close(ffd);
    }
    else {
        // === Process RGB565 data ===
        // Captured data have swapped even and odd pixels, fix it
        // (Data are written to buffer as 32-bit values)
        swap_pixels((uint16_t *)frame_buffer, width*height);

        if (dest == DEST_BUFFER) {
            *(uint16_t *)(frame_buffer-8) = width;
            *(uint16_t *)(frame_buffer-6) = height;
            memcpy(frame_buffer-4, "rawB", 4);
            result = mp_obj_new_str_copy(&mp_type_bytes, (const byte*)(frame_buffer-8), (width*height*2)+8);
        }
        else if (dest == DEST_TFT) {
            send_data_scale(0, 0, width, height, (color_t *)frame_buffer, 0);

            tft_setup((uint8_t)(args[ARG_time].u_int & 3));
            send_frame_buffer();
        }
        else if ((dest == DEST_FILE) && (ffd != mp_const_none)) {
            *(uint16_t *)(frame_buffer-8) = width;
            *(uint16_t *)(frame_buffer-6) = height;
            memcpy(frame_buffer-4, "rawB", 4);
            int written_bytes = mp_stream_posix_write((void *)ffd, (void *)(frame_buffer-8), (width*height*2)+8);
            mp_stream_close(ffd);
            if (written_bytes != ((width*height*2)+8)) {
                _restore_mode_size(self, orig_size, buf_size);
                mp_raise_msg(&mp_type_OSError, "Error writing to capture file");
            }
        }
        else if (ffd != mp_const_none) mp_stream_close(ffd);
    }

    _restore_mode_size(self, orig_size, buf_size);
    return result;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mod_camera_capture_obj, 1, mod_camera_capture);

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_camera_preview(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_stop, ARG_frames, ARG_fps };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_stop,                      MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_frames,   MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 10000000} },
        { MP_QSTR_fps,      MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 0} },
    };

    mod_camera_obj_t *self = pos_args[0];

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    check_camera(self);

    if (self->preview_task) {
        if (args[ARG_stop].u_bool == true) stop_preview_task(self);
        return mp_const_none;
    }

    // Check conditions to run preview
    if (self->sensor.pixformat == PIXFORMAT_JPEG) {
        mp_raise_msg(&mp_type_OSError, "preview not supported when in JPEG mode.");
    }
    if (active_dstate->tft_frame_buffer == NULL) {
        mp_raise_msg(&mp_type_OSError, "display framebuffer not available.");
    }
    /*if (self->sensor.framesize != FRAMESIZE_QVGA) {
        self->sensor.framesize = FRAMESIZE_QVGA;
        dvp_config_size(&self->sensor);
        cam_create_buffer(self);
        self->sensor.set_framesize(FRAMESIZE_QVGA);
    }*/

    // === Preview requires two frame buffers ===
    self->buff_obj1 = mp_obj_new_frame_buffer(self->sensor.gram_size+8);
    if (self->buff_obj1 == mp_const_none) {
        mp_raise_msg(&mp_type_OSError, "Error creating 2nd frame buffer.");
    }
    mp_obj_array_t *gram1 = (mp_obj_array_t *)self->buff_obj1;
    self->sensor.gram1 = gram1->items + 8;
    LOGD(TAG, "2nd buffer created: %p (buf at %p, size=%lu)", (void *)self->buff_obj1, gram1->items, gram1->len);

    // Create the preview task
    task_params.max_frames = 10000000;
    if (args[ARG_frames].u_int > 100) task_params.max_frames = args[ARG_frames].u_int;
    task_params.disp_fps = (uint8_t)(args[ARG_fps].u_int & 3);;
    task_params.cam_obj = (void *)self;
    task_params.thread_handle = xTaskGetCurrentTaskHandle();

    BaseType_t res = xTaskCreate(
            preview_task,               // function entry
            "Camera_preview",           // task name
            configMINIMAL_STACK_SIZE,   // stack_deepth
            (void *)&task_params,       // function argument
            MICROPY_TASK_PRIORITY+1,    // task priority
            &self->preview_task);       // task handle

    if (res != pdPASS) self->preview_task = NULL;
    if (self->preview_task == NULL) {
        mp_raise_ValueError("error starting preview task");
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mod_camera_preview_obj, 1, mod_camera_preview);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_camera_orient(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_hmirror, ARG_vflip };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_hmirror,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_vflip,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    mod_camera_obj_t *self = pos_args[0];

    check_preview(self);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[ARG_hmirror].u_obj != mp_const_none) self->sensor.set_hmirror(mp_obj_is_true(args[ARG_hmirror].u_obj));
    if (args[ARG_vflip].u_obj != mp_const_none) self->sensor.set_vflip(mp_obj_is_true(args[ARG_vflip].u_obj));

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mod_camera_orient_obj, 1, mod_camera_orient);

//-------------------------------------------------------------------
STATIC mp_obj_t mod_camera_effects(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_get_int(arg_in);
    if ((arg < 0) || (self->sensor.set_special_effect((uint8_t)arg) != 0)) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_effects_obj, mod_camera_effects);

//----------------------------------------------------------------------
STATIC mp_obj_t mod_camera_brightness(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_get_int(arg_in);
    if (self->sensor.set_brightness(arg) != 0) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_brightness_obj, mod_camera_brightness);

//--------------------------------------------------------------------
STATIC mp_obj_t mod_camera_contrast(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_get_int(arg_in);
    if (self->sensor.set_contrast(arg) != 0) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_contrast_obj, mod_camera_contrast);

//----------------------------------------------------------------------
STATIC mp_obj_t mod_camera_saturation(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_get_int(arg_in);
    if (self->sensor.set_saturation(arg) != 0) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_saturation_obj, mod_camera_saturation);

//---------------------------------------------------------------------
STATIC mp_obj_t mod_camera_lightmode(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_get_int(arg_in);
    if ((arg < 0) || (self->sensor.set_light_mode((uint8_t)arg) != 0)) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_lightmode_obj, mod_camera_lightmode);

//---------------------------------------------------------------------
STATIC mp_obj_t mod_camera_nightmode(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_is_true(arg_in);
    if ((arg < 0) || (self->sensor.set_night_mode(arg) != 0)) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_nightmode_obj, mod_camera_nightmode);

//---------------------------------------------------------------
STATIC mp_obj_t mod_camera_bar(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_get_int(arg_in);
    if (self->sensor.set_colorbar(arg) != 0) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_bar_obj, mod_camera_bar);

//-------------------------------------------------------------------
STATIC mp_obj_t mod_camera_quality(mp_obj_t self_in, mp_obj_t arg_in)
{
    mod_camera_obj_t *self = self_in;

    check_preview(self);
    int arg = mp_obj_get_int(arg_in);
    if (self->sensor.pixformat != PIXFORMAT_JPEG) {
        mp_raise_ValueError("not in JPEG mode");
    }
    if ((arg < 3) || (arg > 50) || (self->sensor.set_quality((uint8_t)arg) != 0)) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_camera_quality_obj, mod_camera_quality);

//----------------------------------------------------------------------
STATIC mp_obj_t mod_camera_exposure(size_t n_args, const mp_obj_t *args)
{
    mod_camera_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    check_preview(self);
    if (n_args > 1) {
        int exp = mp_obj_get_int(args[1]);
        self->sensor.set_exposure(exp);
        return mp_const_none;
    }
    return mp_obj_new_int(self->sensor.set_exposure(-1));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_camera_exposure_obj, 1, 2, mod_camera_exposure);


//==============================================================
STATIC const mp_rom_map_elem_t mod_camera_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__),         MP_ROM_PTR(&mod_camera_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),           MP_ROM_PTR(&mod_camera_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&mod_camera_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture),         MP_ROM_PTR(&mod_camera_capture_obj) },
    { MP_ROM_QSTR(MP_QSTR_preview),         MP_ROM_PTR(&mod_camera_preview_obj) },
    { MP_ROM_QSTR(MP_QSTR_orient),          MP_ROM_PTR(&mod_camera_orient_obj) },
    { MP_ROM_QSTR(MP_QSTR_effect),          MP_ROM_PTR(&mod_camera_effects_obj) },
    { MP_ROM_QSTR(MP_QSTR_brightness),      MP_ROM_PTR(&mod_camera_brightness_obj) },
    { MP_ROM_QSTR(MP_QSTR_contrast),        MP_ROM_PTR(&mod_camera_contrast_obj) },
    { MP_ROM_QSTR(MP_QSTR_saturation),      MP_ROM_PTR(&mod_camera_saturation_obj) },
    { MP_ROM_QSTR(MP_QSTR_lightmode),       MP_ROM_PTR(&mod_camera_lightmode_obj) },
    { MP_ROM_QSTR(MP_QSTR_nightmode),       MP_ROM_PTR(&mod_camera_nightmode_obj) },
    { MP_ROM_QSTR(MP_QSTR_bar),             MP_ROM_PTR(&mod_camera_bar_obj) },
    { MP_ROM_QSTR(MP_QSTR_quality),         MP_ROM_PTR(&mod_camera_quality_obj) },
    { MP_ROM_QSTR(MP_QSTR_size),            MP_ROM_PTR(&mod_camera_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_exposure),        MP_ROM_PTR(&mod_camera_exposure_obj) },

    { MP_ROM_QSTR(MP_QSTR_MODE_RGB565),     MP_ROM_INT(PIXFORMAT_RGB565) },
    { MP_ROM_QSTR(MP_QSTR_MODE_JPEG),       MP_ROM_INT(PIXFORMAT_JPEG) },

    { MP_ROM_QSTR(MP_QSTR_SIZE_CIF),        MP_ROM_INT(FRAMESIZE_CIF) },      // 352x288
    { MP_ROM_QSTR(MP_QSTR_SIZE_SIF),        MP_ROM_INT(FRAMESIZE_SIF) },      // 352x240
    { MP_ROM_QSTR(MP_QSTR_SIZE_QQVGA),      MP_ROM_INT(FRAMESIZE_QQVGA) },    // 160x120
    { MP_ROM_QSTR(MP_QSTR_SIZE_128X64),     MP_ROM_INT(FRAMESIZE_128X64) },   // 128x64
    { MP_ROM_QSTR(MP_QSTR_SIZE_QVGA),       MP_ROM_INT(FRAMESIZE_QVGA) },     // 320x240
    { MP_ROM_QSTR(MP_QSTR_SIZE_HVGA),       MP_ROM_INT(FRAMESIZE_HVGA) },     // 480x320
    { MP_ROM_QSTR(MP_QSTR_SIZE_VGA),        MP_ROM_INT(FRAMESIZE_VGA) },      // 640x480
    { MP_ROM_QSTR(MP_QSTR_SIZE_WVGA2),      MP_ROM_INT(FRAMESIZE_WVGA2) },    // 752x480
    { MP_ROM_QSTR(MP_QSTR_SIZE_SVGA),       MP_ROM_INT(FRAMESIZE_SVGA) },     // 800x600
    { MP_ROM_QSTR(MP_QSTR_SIZE_XGA),        MP_ROM_INT(FRAMESIZE_XGA) },      // 1024x768
    { MP_ROM_QSTR(MP_QSTR_SIZE_SXGA),       MP_ROM_INT(FRAMESIZE_SXGA) },     // 1280x1024
    { MP_ROM_QSTR(MP_QSTR_SIZE_UXGA),       MP_ROM_INT(FRAMESIZE_UXGA) },     // 1600x1200
    { MP_ROM_QSTR(MP_QSTR_SIZE_720P),       MP_ROM_INT(FRAMESIZE_720P) },     // 1280x720 *
    { MP_ROM_QSTR(MP_QSTR_SIZE_1080P),      MP_ROM_INT(FRAMESIZE_1080P) },    // 1920x1080 *
    { MP_ROM_QSTR(MP_QSTR_SIZE_960P),       MP_ROM_INT(FRAMESIZE_960P) },     // 1280x960 *
    { MP_ROM_QSTR(MP_QSTR_SIZE_5MP),        MP_ROM_INT(FRAMESIZE_5MPP) },     // 2592x1944 *

    { MP_ROM_QSTR(MP_QSTR_LIGHT_AUTO),      MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_LIGHT_SUNNY),     MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_LIGHT_OFFICE),    MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_LIGHT_CLUUDY),    MP_ROM_INT(3) },
    { MP_ROM_QSTR(MP_QSTR_LIGHT_HOME),      MP_ROM_INT(4) },

    { MP_ROM_QSTR(MP_QSTR_DEST_BUFFER),     MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_DEST_DISP),       MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_DEST_FILE),       MP_ROM_INT(2) },
};
STATIC MP_DEFINE_CONST_DICT(mod_camera_locals_dict, mod_camera_locals_dict_table);

//=====================================
const mp_obj_type_t mod_camera_type = {
    { &mp_type_type },
    .name = MP_QSTR_camera,
    .print = mod_camera_print,
    .make_new = mod_camera_make_new,
    .locals_dict = (mp_obj_dict_t*)&mod_camera_locals_dict,
};


//==========================================================
STATIC const mp_map_elem_t camera_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_camera) },

    { MP_ROM_QSTR(MP_QSTR_cam),             MP_ROM_PTR(&mod_camera_type) },

    { MP_ROM_QSTR(MP_QSTR_AUTO),            MP_ROM_INT(SENSORTYPE_AUTO) },
    { MP_ROM_QSTR(MP_QSTR_OV2640),          MP_ROM_INT(SENSORTYPE_OV2640) },
    { MP_ROM_QSTR(MP_QSTR_OV5640),          MP_ROM_INT(SENSORTYPE_OV5640) },
};

//===========================
STATIC MP_DEFINE_CONST_DICT (
    camera_module_globals,
    camera_module_globals_table
);

const mp_obj_module_t mp_module_camera = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&camera_module_globals,
};

#endif
