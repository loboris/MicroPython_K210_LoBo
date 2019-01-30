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
/*
 *  Author: LoBo (loboris@gmail.com, loboris.github)
 *
 *  Module supporting MAIX TFT displays
 * 
 * HIGH SPEED LOW LEVEL DISPLAY FUNCTIONS
 * USING DIRECT or DMA SPI TRANSFER MODEs
 *
*/

#include "mpconfigport.h"

#if MICROPY_USE_DISPLAY

#define USE_DISPLAY_TASK    0

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <devices.h>
#include "tftspi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "syslog.h"
//#include "project_cfg.h"
#include "mphalport.h"

#define WAIT_CYCLE      4U
#define LCD_X_MAX   240
#define LCD_Y_MAX   320
#define TFT_BGR     8
// RGB to GRAYSCALE constants
// 0.2989  0.5870  0.1140
#define GS_FACT_R 0.2989
#define GS_FACT_G 0.4870
#define GS_FACT_B 0.2140

#define SPI_CHANNEL     (0)
#define DCX_IO          (38)
#define TFT_RST         (37)

enum _instruction_length
{
    INSTRUCTION_LEN_0 = 0,
    INSTRUCTION_LEN_8 = 8,
    INSTRUCTION_LEN_16 = 16,
    INSTRUCTION_LEN_32 = 32,
} ;

enum _address_length
{
    ADDRESS_LEN_0 = 0,
    ADDRESS_LEN_8 = 8,
    ADDRESS_LEN_16 = 16,
    ADDRESS_LEN_32 = 32,
} ;

enum _frame_length
{
    FRAME_LEN_0 = 0,
    FRAME_LEN_8 = 8,
    FRAME_LEN_16 = 16,
    FRAME_LEN_32 = 32,
} ;

static handle_t gio;
static handle_t spi0;
static handle_t spi_dfs8;
static handle_t spi_dfs16;
static handle_t spi_dfs32;

typedef struct {
    uint8_t dc;
    uint8_t type;
    uint16_t len;
    void *data;
} disp_msg_t;

#define DISP_NUM_FUNC           4
#define DISP_SPI_SLAVE_SELECT   3  // FUNC_SPI0_SS3

static bool display_pins_init = false;
static int tft_rst_gpionum = 0;
static int tft_dc_gpionum = 0;
static mp_fpioa_cfg_item_t disp_pin_func[DISP_NUM_FUNC];

// ====================================================
// ==== Global variables, default values ==============

// Converts colors to grayscale if set to 1
uint8_t gray_scale = 0;

// Display dimensions
int _width = DEFAULT_TFT_DISPLAY_WIDTH;
int _height = DEFAULT_TFT_DISPLAY_HEIGHT;

// Display type
uint8_t tft_disp_type = DEFAULT_DISP_TYPE;
uint8_t tft_touch_type = TOUCH_TYPE_NONE;

uint8_t bits_per_color = 16;
uint8_t TFT_RGB_BGR = 0;
uint8_t gamma_curve = 0;
uint32_t spi_speed = 4000000;
// ====================================================

#if USE_DISPLAY_TASK
extern int MainTaskProc;
static QueueHandle_t dispQueue = NULL;
static TaskHandle_t disp_task_handle = 0;
#endif

//static const char TAG[] = "[TFTSPI]";
static uint8_t invertrot = 1;

uint8_t spibus_is_init = 0;

// ==== Functions =====================


//---------------------------
static void set_dcx_control()
{
    gpio_set_pin_value(gio, tft_dc_gpionum, GPIO_PV_LOW);
}

//------------------------
static void set_dcx_data()
{
    gpio_set_pin_value(gio, tft_dc_gpionum, GPIO_PV_HIGH);
}

//----------------------
uint32_t tft_set_speed()
{
    spi_dev_set_clock_rate(spi_dfs8, 3000000);
    uint32_t speed = (uint32_t)spi_dev_set_clock_rate(spi_dfs16, spi_speed);
    spi_dev_set_clock_rate(spi_dfs32, spi_speed);
    return speed;
}

//-----------------------------
static bool tft_hard_init(void)
{
    // Configure display pins
    if (!display_pins_init) {
        // FPIOA configuration
        tft_dc_gpionum = gpiohs_get_free();
        if (tft_dc_gpionum < 0) return false;
        tft_rst_gpionum = gpiohs_get_free();
        if (tft_rst_gpionum < 0) {
            gpiohs_set_free(tft_dc_gpionum);
            return false;
        }

        disp_pin_func[0] = (mp_fpioa_cfg_item_t){tft_dc_gpionum, DCX_IO, FUNC_GPIOHS0 + tft_dc_gpionum};
        disp_pin_func[1] = (mp_fpioa_cfg_item_t){-1, 36, FUNC_SPI0_SS3};
        disp_pin_func[2] = (mp_fpioa_cfg_item_t){-1, 39, FUNC_SPI0_SCLK};
        disp_pin_func[3] = (mp_fpioa_cfg_item_t){tft_rst_gpionum, TFT_RST, FUNC_GPIOHS0 + tft_rst_gpionum};

        if (!fpioa_check_pins(DISP_NUM_FUNC, disp_pin_func, GPIO_FUNC_DISP)) {
            gpiohs_set_free(tft_dc_gpionum);
            gpiohs_set_free(tft_rst_gpionum);
            return false;
        }
        // Setup and mark used pins
        fpioa_setup_pins(DISP_NUM_FUNC, disp_pin_func);
        fpioa_setused_pins(DISP_NUM_FUNC, disp_pin_func, GPIO_FUNC_DISP);
        display_pins_init = true;
    }

    // open gpiohs device
    gio = io_open("/dev/gpio0");
    if (gio == 0) return false;

    gpio_set_drive_mode(gio, tft_dc_gpionum, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gio, tft_dc_gpionum, GPIO_PV_HIGH);

    gpio_set_drive_mode(gio, tft_rst_gpionum, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gio, tft_rst_gpionum, GPIO_PV_HIGH);

    // Set SPI0_D0-D7 DVP_D0-D7 as spi and dvp data pin
    sysctl_set_spi0_dvp_data(1);

    // openSPI device
    spi0 = io_open("/dev/spi0");
    if (spi0 == 0) return false;

    spi_dfs8 = spi_get_device(spi0, SPI_MODE_0, SPI_FF_OCTAL, 1 << DISP_SPI_SLAVE_SELECT, FRAME_LEN_8);
    if (spi_dfs8 == 0) return false;
    spi_dev_config_non_standard(spi_dfs8, INSTRUCTION_LEN_8, ADDRESS_LEN_0, WAIT_CYCLE, SPI_AITM_AS_FRAME_FORMAT);

    spi_dfs16 = spi_get_device(spi0, SPI_MODE_0, SPI_FF_OCTAL, 1 << DISP_SPI_SLAVE_SELECT, FRAME_LEN_16);
    if (spi_dfs16 == 0) return false;
    spi_dev_config_non_standard(spi_dfs16, INSTRUCTION_LEN_16, ADDRESS_LEN_0, WAIT_CYCLE, SPI_AITM_AS_FRAME_FORMAT);

    spi_dfs32 = spi_get_device(spi0, SPI_MODE_0, SPI_FF_OCTAL, 1 << DISP_SPI_SLAVE_SELECT, FRAME_LEN_32);
    if (spi_dfs32 == 0) return false;
    spi_dev_config_non_standard(spi_dfs32, INSTRUCTION_LEN_0, ADDRESS_LEN_32, WAIT_CYCLE, SPI_AITM_AS_FRAME_FORMAT);

    tft_set_speed();

    return true;
}

#if USE_DISPLAY_TASK
//--------------------------------------
static void disp_task(void *pvParameter)
{
    disp_msg_t msg;

    while (1) {
        if (xQueueReceive(dispQueue, &msg, portMAX_DELAY) == pdPASS) {
            if (msg.dc > 0) set_dcx_data();
            else set_dcx_control();

            switch (msg.type) {
                case 1:
                    io_write(spi_dfs8, (const uint8_t *)(msg.data), (size_t)msg.len);
                    break;
                case 2:
                    io_write(spi_dfs16, (const uint8_t *)(msg.data), (size_t)msg.len);
                    break;
                case 3:
                    spi_dev_fill(spi_dfs32, 0, *((uint32_t *)msg.data), *((uint32_t *)msg.data), msg.len);
                    break;
                default:
                    LOGE(TAG, "Unknown msg tyte");
            }
        }
        else {
            LOGE(TAG, "Queue receive error");
        }
    }
    if (dispQueue) {
        vQueueDelete(dispQueue);
        dispQueue = NULL;
    }
    disp_task_handle = 0;
    vTaskDelete(NULL);
}
#endif

//----------------------------------------
static void tft_write_command(uint8_t cmd)
{
#if USE_DISPLAY_TASK
    disp_msg_t msg;
    msg.dc = 0;
    msg.type = 1;
    msg.len = 1;
    msg.data = (void *)&cmd;
    xQueueSend(dispQueue, &msg, 1000/portTICK_PERIOD_MS);
#else
    set_dcx_control();
    io_write(spi_dfs8, (const uint8_t *)(&cmd), 1);
#endif
}

//------------------------------------------------------------
static void tft_write_byte(uint8_t* data_buf, uint32_t length)
{
#if USE_DISPLAY_TASK
    disp_msg_t msg;
    msg.dc = 1;
    msg.type = 1;
    msg.len = length;
    msg.data = (void *)data_buf;
    xQueueSend(dispQueue, &msg, 1000/portTICK_PERIOD_MS);
#else
    set_dcx_data();
    io_write(spi_dfs8, (const uint8_t *)(data_buf), length);
#endif
}

//-------------------------------------------------------------
static void tft_write_half(uint16_t* data_buf, uint32_t length)
{
#if USE_DISPLAY_TASK
    disp_msg_t msg;
    msg.dc = 1;
    msg.type = 2;
    msg.len = length*2;
    msg.data = (void *)data_buf;
    xQueueSend(dispQueue, &msg, 1000/portTICK_PERIOD_MS);
#else
    set_dcx_data();
    io_write(spi_dfs16, (const uint8_t *)(data_buf), length * 2);
#endif
}

/*
//-------------------------------------------------------------
static void tft_write_word(uint32_t* data_buf, uint32_t length)
{
#if USE_DISPLAY_TASK
    disp_msg_t msg;
    msg.dc = 1;
    msg.type = 3;
    msg.len = length*4;
    msg.data = (void *)data_buf;
    xQueueSend(dispQueue, &msg, 1000/portTICK_PERIOD_MS);
#else
    set_dcx_data();
    io_write(spi_dfs32, (const uint8_t *)data_buf, length * 4);
#endif
}
*/

//-------------------------------------------------------
static void tft_fill_data(uint32_t data, uint32_t length)
{
#if USE_DISPLAY_TASK
    disp_msg_t msg;
    msg.dc = 1;
    msg.type = 3;
    msg.len = length*4-1;
    msg.data = (void *)&data;
    xQueueSend(dispQueue, &msg, 1000/portTICK_PERIOD_MS);
#else
    set_dcx_data();
    spi_dev_fill(spi_dfs32, 0, data, data, length - 1);
#endif
}


//---------------------------------
static bool tft_init(uint8_t hw_sw)
{
    uint8_t data;

    if (!tft_hard_init()) return false;

    if (hw_sw & 0x02) {
        // hard reset
        gpio_set_pin_value(gio, tft_rst_gpionum, GPIO_PV_LOW);
        mp_hal_delay_ms(120);
        gpio_set_pin_value(gio, tft_rst_gpionum, GPIO_PV_HIGH);
    }
    if (hw_sw & 0x01) {
        // soft reset
        tft_write_command(SOFTWARE_RESET);
        mp_hal_delay_ms(120);
    }
    // exit sleep
    tft_write_command(SLEEP_OFF);
    mp_hal_delay_ms(120);
    // pixel format
    tft_write_command(PIXEL_FORMAT_SET);
    data = 0x55;
    tft_write_byte(&data, 1);
    tft_write_command(DISPALY_ON);
    return true;
}

// Send command with data to display, display must be selected
//----------------------------------------------------------------------
void disp_spi_transfer_cmd_data(int8_t cmd, uint8_t *data, uint32_t len)
{
    tft_write_command(cmd);

	if ((len == 0) || (data == NULL)) return;

	tft_write_byte(data, len);
}

// Send 1 byte display command, display must be selected
//------------------------------------
void disp_spi_transfer_cmd(int8_t cmd)
{
    tft_write_command(cmd);
}

// Set the address window for display write & read commands, display must be selected
//-----------------------------------------------------------------------------------------
static void disp_spi_transfer_addrwin(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2) {
    uint8_t data[4];

    data[0] = (uint8_t)(x1 >> 8);
    data[1] = (uint8_t)(x1);
    data[2] = (uint8_t)(x2 >> 8);
    data[3] = (uint8_t)(x2);
    tft_write_command(HORIZONTAL_ADDRESS_SET);
    tft_write_byte(data, 4);
    data[0] = (uint8_t)(y1 >> 8);
    data[1] = (uint8_t)(y1);
    data[2] = (uint8_t)(y2 >> 8);
    data[3] = (uint8_t)(y2);
    tft_write_command(VERTICAL_ADDRESS_SET);
    tft_write_byte(data, 4);
    tft_write_command(MEMORY_WRITE);
    //mp_hal_delay_us(50);
}

// Convert color to gray scale
//------------------------------------
static color_t color2gs(color_t color)
{
	color_t _color;
    float gs_clr = GS_FACT_R * color.r + GS_FACT_G * color.g + GS_FACT_B * color.b;
    if (gs_clr > 255) gs_clr = 255;

    _color.r = (uint8_t)gs_clr;
    _color.g = (uint8_t)gs_clr;
    _color.b = (uint8_t)gs_clr;

    return _color;
}

// Convert color to 16-bit 565 RGB value
//------------------------------------
static uint16_t color16(color_t color)
{
	uint16_t _color;
	_color = (uint16_t)(color.b & 0xF8) << 8;
	_color |= (uint16_t)(color.g & 0xFC) << 3;
	_color |= (uint16_t)(color.r & 0xF8) >> 3;
    return _color;
}

//============================================
int wait_trans_finish(uint8_t free_line)
{
    return 0;
}

// Set display pixel at given coordinates to given color
//=================================================
void drawPixel(int16_t x, int16_t y, color_t color)
{
    color_t _color = color;
	if (gray_scale) _color = color2gs(color);

	disp_spi_transfer_addrwin(x, x, y, y);

	if (bits_per_color == 16) {
		uint16_t _color16 = color16(_color);
	    tft_write_half(&_color16, 1);
	}
	else {
	}
}

// Write 'len' color data to TFT 'window' (x1,y2),(x2,y2)
//================================================================================
void TFT_pushColorRep(int x1, int y1, int x2, int y2, color_t color, uint32_t len)
{
    color_t _color;
    uint16_t  _color16 = 0;

    // Prepare fill color
    if (gray_scale) _color = color2gs(color);
    else _color = color;
    _color16 = color16(_color);

    uint32_t data = ((uint32_t)_color16 << 16) | (uint32_t)_color16;

    // ** Send address window **
	disp_spi_transfer_addrwin(x1, x2, y1, y2);

	tft_fill_data(data, len);
}

// Write 'len' color data to TFT 'window' (x1,y2),(x2,y2) from given buffer
// === Device must already be selected ===
//========================================================================
void send_data(int x1, int y1, int x2, int y2, uint32_t len, color_t *buf)
{
    color_t _color;
    uint16_t  _color16 = 0;
    // prepare color buffer
    uint16_t *cbuf = malloc(len*2);

    for (uint32_t i=0; i<len; i++) {
        if (gray_scale) _color = color2gs(buf[i]);
        else _color = buf[i];
        _color16 = color16(_color);
        cbuf[i] = _color16;
    }

    // ** Send address window **
    if (x2 > x1) x2 -= 1;
    if (y2 > y1) y2 -= 1;
    disp_spi_transfer_addrwin(x1, x2, y1, y2);

    // Send color buffer
    tft_write_half(cbuf, len);
    free(cbuf);
}

//---------------------------------------------------------------------------
void send_data16(int x1, int y1, int x2, int y2, uint32_t len, uint16_t *buf)
{
    // ** Send address window **
    if (x2 > x1) x2 -= 1;
    if (y2 > y1) y2 -= 1;
    disp_spi_transfer_addrwin(x1, x2, y1, y2);

    // Send color buffer
    tft_write_half(buf, len);
}

//==================================
void _tft_setRotation(uint8_t rot) {
	uint8_t rotation = rot & 3; // can't be higher than 3
	uint8_t send = 1;
	uint8_t madctl = 0;
	uint16_t tmp;

    if ((rotation & 1)) {
        // in landscape modes must be width > height
        if (_width < _height) {
            tmp = _width;
            _width  = _height;
            _height = tmp;
        }
    }
    else {
        // in portrait modes must be width < height
        if (_width > _height) {
            tmp = _width;
            _width  = _height;
            _height = tmp;
        }
    }
    if (invertrot == 2) {
        switch (rotation) {
            case PORTRAIT:
            madctl = (MADCTL_MV | TFT_RGB_BGR);
            break;
            case LANDSCAPE:
            madctl = (MADCTL_MX | TFT_RGB_BGR);
            break;
            case PORTRAIT_FLIP:
            madctl = (MADCTL_MV | TFT_RGB_BGR);
            break;
            case LANDSCAPE_FLIP:
            madctl = (MADCTL_MY | TFT_RGB_BGR);
            break;
        }
    }
    else if (invertrot == 3) {
        switch (rotation) {
            case PORTRAIT:
            madctl = (MADCTL_MX | MADCTL_MV | TFT_RGB_BGR);
            break;
            case LANDSCAPE:
            madctl = (TFT_RGB_BGR);
            break;
            case PORTRAIT_FLIP:
            madctl = (MADCTL_MY | MADCTL_MV | TFT_RGB_BGR);
            break;
            case LANDSCAPE_FLIP:
            madctl = (MADCTL_MY | MADCTL_MX | TFT_RGB_BGR);
            break;
        }
    }
    else if (invertrot == 1) {
        switch (rotation) {
            case PORTRAIT:
            madctl = (MADCTL_MY | MADCTL_MX | TFT_RGB_BGR);
            break;
            case LANDSCAPE:
            madctl = (MADCTL_MY | MADCTL_MV | TFT_RGB_BGR);
            break;
            case PORTRAIT_FLIP:
            madctl = (TFT_RGB_BGR);
            break;
            case LANDSCAPE_FLIP:
            madctl = (MADCTL_MX | MADCTL_MV | TFT_RGB_BGR);
            break;
        }
    }
    else {
        switch (rotation) {
            case PORTRAIT:
            madctl = (MADCTL_MX | TFT_RGB_BGR);
            break;
            case LANDSCAPE:
            madctl = (MADCTL_MV | TFT_RGB_BGR);
            break;
            case PORTRAIT_FLIP:
            madctl = (MADCTL_MY | TFT_RGB_BGR);
            break;
            case LANDSCAPE_FLIP:
            madctl = (MADCTL_MX | MADCTL_MY | MADCTL_MV | TFT_RGB_BGR);
            break;
        }
    }
	if (send) {
    	disp_spi_transfer_cmd_data(MEMORY_ACCESS_CTL, &madctl, 1);
	}

}

//---------------------------------------
void bcklOff(display_config_t *dconfig) {
}

//--------------------------------------
void bcklOn(display_config_t *dconfig) {
}

//===========================================
void _tft_setBitsPerColor(uint8_t bitsperc) {
}

//----------------------------------------------
int TFT_spiInit(display_config_t *dconfig)
{
	return 0;
}

//=================================================
void TFT_display_setvars(display_config_t *dconfig)
{
    // === SET GLOBAL VARIABLES ==========================
    tft_disp_type = dconfig->type;
    _width = dconfig->width;
    _height = dconfig->height;
    TFT_RGB_BGR = dconfig->bgr;
    gamma_curve = dconfig->gamma;
    spi_speed = dconfig->speed;
    bits_per_color = dconfig->color_bits;
    invertrot = dconfig->invrot;
    // ===================================================
}

// Initialize the display
// ==================================================
int TFT_display_init(display_config_t *dconfig)
{
#if USE_DISPLAY_TASK
    if (dispQueue == NULL) {
        dispQueue = xQueueCreate( 32, sizeof(disp_msg_t) );
        configASSERT(dispQueue);
    }

    xTaskCreateAtProcessor(
            MainTaskProc ^ 1,           // processor
            disp_task,                  // function entry
            "display_task",             // task name
            configMINIMAL_STACK_SIZE,   // stack_deepth
            NULL,                       // function argument
            10,                         // task priority
            &disp_task_handle);         // task handle
    configASSERT(disp_task_handle);
#endif

    TFT_display_setvars(dconfig);

    if (!tft_init(2)) return -1;
    vTaskDelay(100);

    // Clear screen
    //_tft_setRotation(PORTRAIT);
    // TFT_pushColorRep(0, 0, _width-1, _height-1, (color_t){0,0,0}, (uint32_t)(_height*_width));

    ///Enable backlight
    return 0;
}

#endif // CONFIG_MICROPY_USE_DISPLAY

