/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 LoBo (https://github.com/loboris)
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
 * Common EPD SPI interface functions
 */

#include "mpconfigport.h"

#if MICROPY_USE_TFT

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "epdspi.h"
#include "tftspi.h"
#include "epd_4_2.h"
#include "epd_2_9.h"

#if MICROPY_USE_EPD

int epd_busy_pin = -1;
int epd_busy_level;
int epd_cs_pin;
int epd_dc_pin;
int epd_rst_pin = -1;
int epd_pwr_pin = -1;
int EPD_ready_timer = 0;
handle_t spi_device = 0;
handle_t spi_handle = 0;

static int ready_tmo = 4000;
static const char TAG[] = "[EPD_SPI]";
static uint8_t status_cmd = 0x2F;
static uint8_t busy_bit = 2;

// -----------------------------------
// Check display busy line or register
// Wait until ready or timeout
// -----------------------------------
//------------------
void EPD_WaitReady()
{
    uint8_t val = 0;
    EPD_ready_timer = 0;
    while (EPD_ready_timer < ready_tmo) {
        if (epd_busy_pin >= 0) {
            // Check busy status by monitoring the busy pin
            if (gpio_get_pin_value(gpiohs_handle, epd_busy_pin) != epd_busy_level) break;
        }
        else {
            // Check busy status by reading the status register
            SPI_SELECT(); // CS=0
            EPD_WRITE_COMMAND(); // command write
            io_write(spi_device, &status_cmd, 1);
            EPD_WRITE_DATA(); // data read
            io_read(spi_device, &val, 1);
            SPI_DESELECT(); // CS=1
            if (((val >> busy_bit) & 1) != epd_busy_level) break;
        }
        vTaskDelay(2);
        mp_hal_wdt_reset();
        EPD_ready_timer += 2;
    }
    if (EPD_ready_timer >= ready_tmo) LOGW(TAG, "NOT ready in %d ms", EPD_ready_timer);
    else LOGD(TAG, "Ready in %d ms", EPD_ready_timer);
}

// Reset the display if the reset pin is provided
//--------------
void EPD_Reset()
{
    if (epd_rst_pin>=0) {
        mp_hal_wdt_reset();
        gpio_set_pin_value(gpiohs_handle, epd_rst_pin, GPIO_PV_LOW);
        mp_hal_delay_ms(20);
        gpio_set_pin_value(gpiohs_handle, epd_rst_pin, GPIO_PV_HIGH);
        mp_hal_delay_ms(20);
        EPD_WaitReady();
    }
}

// Send one byte to display
// Device must already be selected
//-------------------------------
void EPD_SPI_Write(uint8_t value)
{
    io_write(spi_device, &value, 1);
}

//--------------------
uint8_t EPD_SPI_Read()
{
    uint8_t value = 0;
    io_read(spi_device, &value, 1);
    return value;
}

// Write one command without parameters
// doesn't deselect the spi device
//--------------------------------
void EPD_WriteCMD(uint8_t command)
{
    SPI_SELECT(); // CS=0
    EPD_WRITE_COMMAND(); // command write
    EPD_SPI_Write(command);
}

// Write data
// doesn't deselect the spi device
//------------------------------
void EPD_WriteDATA(uint8_t data)
{
    EPD_WRITE_DATA(); // data write
    EPD_SPI_Write(data);
}

// Write command with one parameter, deselect spi
//-------------------------------------------------
void EPD_WriteCMD_p1(uint8_t command, uint8_t para)
{
    EPD_WriteCMD(command);
    EPD_WriteDATA(para);
    SPI_DESELECT(); // CS=1
}

/*
// Write data buffer
// doesn't deselect the spi device
//--------------------------------------------------
static void EPD_WriteDATAbuf(uint8_t *data, int len)
{
    EPD_WRITE_DATA(); // data write
    io_write(spi_device, data, len);
}
*/

// Write command with 2 parameters, deselect spi
//-----------------------------------------------------------------
void EPD_WriteCMD_p2(uint8_t command, uint8_t para1, uint8_t para2)
{
    EPD_WriteCMD(command);
    EPD_WriteDATA(para1);
    EPD_SPI_Write(para2);
    SPI_DESELECT(); // CS=1
}

// Write command with 3 parameters, deselect spi
//--------------------------------------------------------------------------------
void EPD_WriteCMD_p3(uint8_t command, uint8_t para1, uint8_t para2, uint8_t para3)
{
    EPD_WriteCMD(command);
    EPD_WriteDATA(para1);
    EPD_SPI_Write(para2);
    EPD_SPI_Write(para3);
    SPI_DESELECT(); // CS=1
}

// Write command with 4 parameters, deselect spi
//-----------------------------------------------------------------------------------------------
void EPD_WriteCMD_p4(uint8_t command, uint8_t para1, uint8_t para2, uint8_t para3, uint8_t para4)
{
    EPD_WriteCMD(command);
    EPD_WriteDATA(para1);
    EPD_SPI_Write(para2);
    EPD_SPI_Write(para3);
    EPD_SPI_Write(para4);
    SPI_DESELECT(); // CS=1
}

// Send command with multiple parameters, deselect spi
//---------------------------------------
void EPD_Write(uint8_t *buf, int datalen)
{
    uint8_t *ptemp = buf;

    // The first byte is written as command value
    EPD_WriteCMD(*ptemp++);
    // the rest are sent as data
    EPD_WRITE_DATA();
    io_write(spi_device, ptemp, datalen-1);
    SPI_DESELECT(); // CS=1
}

// Send command and data stream
//----------------------------------------------------------------------------------------
void EPD_Write_command_stream(uint8_t command, const uint8_t *data,  unsigned int datalen)
{
    LOGD(TAG, "EPD_Write_command_stream (%02X, %u)", command, datalen);
    EPD_WriteCMD(command);
    EPD_WRITE_DATA();
    io_write(spi_device, data, datalen);
    SPI_DESELECT(); // CS=1
    EPD_WaitReady();
}

//-------------------------------------------------------------------------------------------
void EPD_Write_command_stream32(uint8_t command, const uint32_t *data,  unsigned int datalen)
{
    LOGD(TAG, "EPD_Write_command_stream32 (%02X, %u)", command, datalen*4);
    EPD_WriteCMD(command);
    EPD_WRITE_DATA();
    io_write(spi_device, (uint8_t *)data, datalen*4);
    SPI_DESELECT(); // CS=1
    EPD_WaitReady();
}

// ================================================================
// === Main function to send data to display ======================
// If  rep==true:  repeat sending data to display 'len' times
// If rep==false:  send 'len' data bytes from buffer to display
// ================================================================
//--------------------------------------------------------------
void EPD_SPI_send_data(uint8_t *data, uint32_t len, uint8_t rep)
{
    SPI_SELECT(); // CS=0
    if (!rep) {
        io_write(spi_device, data, len);
    }
    else {
        uint8_t *buf = pvPortMalloc(len);
        if (buf) {
            memset(buf, data[0], len);
            io_write(spi_device, buf, len);
            vPortFree(buf);
        }
        else LOGW(TAG, "send data: buffer error");
    }
    SPI_DESELECT(); // CS=1
}


// =====================================
// ==== EPD SPI interface functions ====
// =====================================

//--------------------------------------------------------------------------
static int EPD_SpiPinsInit(display_epd_obj_t *epd_dev, gpio_pin_func_t func)
{
    // mosi & sck are mandatory, miso & cs are unused
    int spi_offset = 0;
    if (func == GPIO_FUNC_SPI1) {
        spi_offset = FUNC_SPI1_D0 - FUNC_SPI0_D0;
    }
    uint8_t same_pins = 0;
    mp_fpioa_cfg_item_t spi_pin_func[2];

    if ((mp_used_pins[epd_dev->mosi].func == func) && (mp_used_pins[epd_dev->mosi].usedas == GPIO_USEDAS_MISO)) same_pins++;
    if ((mp_used_pins[epd_dev->sck].func == func) && (mp_used_pins[epd_dev->sck].usedas == GPIO_USEDAS_CLK)) same_pins++;

    if (same_pins == 2) {
        // === same mosi & sck
        return 1;
    }

    // === mosi, sck must be set, check if already used
    if (mp_used_pins[epd_dev->mosi].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "MOSI %s", gpiohs_funcs_in_use[mp_used_pins[epd_dev->mosi].func]);
        return -1;
    }
    if (mp_used_pins[epd_dev->sck].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "SCK %s", gpiohs_funcs_in_use[mp_used_pins[epd_dev->mosi].func]);
        return -2;
    }

    // === Configure pins ===
    spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, epd_dev->mosi, GPIO_USEDAS_MOSI, FUNC_SPI0_D0+spi_offset};
    spi_pin_func[1] = (mp_fpioa_cfg_item_t){-1, epd_dev->sck, GPIO_USEDAS_CLK, FUNC_SPI0_SCLK+spi_offset};

    // === Setup and mark used pins ===
    fpioa_setup_pins(2, spi_pin_func);
    fpioa_setused_pins(2, spi_pin_func, func);

    return 0;
}

// Configure the pins used by EPD display
//----------------------------------------------------
static int EPD_EpdPinsInit(display_epd_obj_t *epd_dev)
{
    if (mp_used_pins[epd_dev->cs].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "CS %s", gpiohs_funcs_in_use[mp_used_pins[epd_dev->cs].func]);
        return -1;
    }
    if (epd_dev->busy >= 0) {
        if (mp_used_pins[epd_dev->busy].func != GPIO_FUNC_NONE) {
            LOGW(TAG, "BUSY %s", gpiohs_funcs_in_use[mp_used_pins[epd_dev->busy].func]);
            return -2;
        }
    }
    if (mp_used_pins[epd_dev->dc].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "DC %s", gpiohs_funcs_in_use[mp_used_pins[epd_dev->dc].func]);
        return -3;
    }
    if (epd_dev->reset >= 0) {
        if (mp_used_pins[epd_dev->reset].func != GPIO_FUNC_NONE) {
            LOGW(TAG, "RESET %s", gpiohs_funcs_in_use[mp_used_pins[epd_dev->reset].func]);
            return -3;
        }
    }
    if (epd_dev->pwr >= 0) {
        if (mp_used_pins[epd_dev->pwr].func != GPIO_FUNC_NONE) {
            LOGW(TAG, "PWR %s", gpiohs_funcs_in_use[mp_used_pins[epd_dev->pwr].func]);
            return -3;
        }
    }

    int n_pins = 2;
    mp_fpioa_cfg_item_t pin_func[5];

    epd_cs_pin = gpiohs_get_free();
    if (epd_cs_pin < 0) {
        return -4;
    }
    epd_dc_pin = gpiohs_get_free();
    if (epd_dc_pin < 0) {
        gpiohs_set_free(epd_cs_pin);
        return -5;
    }
    if (epd_dev->busy >= 0) {
        epd_busy_pin = gpiohs_get_free();
        if (epd_busy_pin < 0) {
            gpiohs_set_free(epd_cs_pin);
            gpiohs_set_free(epd_dc_pin);
            return -6;
        }
        pin_func[n_pins] = (mp_fpioa_cfg_item_t){epd_busy_pin, epd_dev->busy, GPIO_USEDAS_INPUT, FUNC_GPIOHS0 + epd_busy_pin};
        n_pins++;
    }
    if (epd_dev->reset >= 0) {
        epd_rst_pin = gpiohs_get_free();
        if (epd_rst_pin < 0) {
            gpiohs_set_free(epd_cs_pin);
            gpiohs_set_free(epd_dc_pin);
            gpiohs_set_free(epd_busy_pin);
            return -7;
        }
        pin_func[n_pins] = (mp_fpioa_cfg_item_t){epd_rst_pin, epd_dev->reset, GPIO_USEDAS_RST, FUNC_GPIOHS0 + epd_rst_pin};
        n_pins++;
    }
    if (epd_dev->pwr >= 0) {
        epd_pwr_pin = gpiohs_get_free();
        if (epd_pwr_pin < 0) {
            gpiohs_set_free(epd_cs_pin);
            gpiohs_set_free(epd_dc_pin);
            gpiohs_set_free(epd_busy_pin);
            if (epd_rst_pin >= 0) gpiohs_set_free(epd_rst_pin);
            return -8;
        }
        pin_func[n_pins] = (mp_fpioa_cfg_item_t){epd_pwr_pin, epd_dev->pwr, GPIO_USEDAS_OUTPUT, FUNC_GPIOHS0 + epd_pwr_pin};
        n_pins++;
    }

    pin_func[0] = (mp_fpioa_cfg_item_t){epd_cs_pin, epd_dev->cs, GPIO_USEDAS_CS, FUNC_GPIOHS0 + epd_cs_pin};
    pin_func[1] = (mp_fpioa_cfg_item_t){epd_dc_pin, epd_dev->dc, GPIO_USEDAS_DCX, FUNC_GPIOHS0 + epd_dc_pin};

    // === Setup and mark used pins ===
    fpioa_setup_pins(n_pins, pin_func);
    fpioa_setused_pins(n_pins, pin_func, GPIO_FUNC_DISP);

    gpio_set_drive_mode(gpiohs_handle, epd_cs_pin, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gpiohs_handle, epd_cs_pin, GPIO_PV_HIGH);
    gpio_set_drive_mode(gpiohs_handle, epd_dc_pin, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gpiohs_handle, epd_dc_pin, GPIO_PV_HIGH);
    if (epd_dev->busy >= 0) {
        gpio_set_drive_mode(gpiohs_handle, epd_busy_pin, GPIO_DM_INPUT_PULL_UP);
    }
    if (epd_dev->reset >= 0) {
        gpio_set_drive_mode(gpiohs_handle, epd_rst_pin, GPIO_DM_OUTPUT);
        gpio_set_pin_value(gpiohs_handle, epd_rst_pin, GPIO_PV_HIGH);
    }
    if (epd_dev->pwr >= 0) {
        gpio_set_drive_mode(gpiohs_handle, epd_pwr_pin, GPIO_DM_OUTPUT);
        gpio_set_pin_value(gpiohs_handle, epd_pwr_pin, GPIO_PV_HIGH);
    }

    return 0;
}

//-----------------------------------------------------
static void spi_hard_deinit(display_epd_obj_t *epd_dev)
{
    // Deconfigure used pins
    mp_fpioa_cfg_item_t spi_pin_func[2];

    spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, epd_dev->mosi, GPIO_USEDAS_NONE, FUNC_RESV0};
    spi_pin_func[1] = (mp_fpioa_cfg_item_t){-1, epd_dev->sck, GPIO_USEDAS_NONE, FUNC_RESV0};
    fpioa_setup_pins(2, spi_pin_func);
    fpioa_freeused_pins(2, spi_pin_func);
}

//-----------------------------------------------------
static void epd_hard_deinit(display_epd_obj_t *epd_dev)
{
    // Deconfigure used pins
    mp_fpioa_cfg_item_t epd_pin_func[5];
    int n_pins = 2;

    gpiohs_set_free(epd_cs_pin);
    gpiohs_set_free(epd_dc_pin);
    if (epd_dev->busy >= 0) {
        gpiohs_set_free(epd_busy_pin);
        epd_pin_func[n_pins] = (mp_fpioa_cfg_item_t){-1, epd_dev->busy, GPIO_USEDAS_NONE, FUNC_RESV0};
        n_pins++;
    }
    if (epd_dev->reset >= 0) {
        gpiohs_set_free(epd_rst_pin);
        epd_pin_func[n_pins] = (mp_fpioa_cfg_item_t){-1, epd_dev->reset, GPIO_USEDAS_NONE, FUNC_RESV0};
        n_pins++;
    }
    if (epd_dev->pwr >= 0) {
        gpiohs_set_free(epd_pwr_pin);
        epd_pin_func[n_pins] = (mp_fpioa_cfg_item_t){-1, epd_dev->pwr, GPIO_USEDAS_NONE, FUNC_RESV0};
        n_pins++;
    }
    epd_rst_pin = -1;
    epd_pwr_pin = -1;
    epd_busy_pin = -1;

    epd_pin_func[0] = (mp_fpioa_cfg_item_t){-1, epd_dev->cs, GPIO_USEDAS_NONE, FUNC_RESV0};
    epd_pin_func[1] = (mp_fpioa_cfg_item_t){-1, epd_dev->dc, GPIO_USEDAS_NONE, FUNC_RESV0};

    fpioa_setup_pins(n_pins, epd_pin_func);
    fpioa_freeused_pins(n_pins, epd_pin_func);
}


// =========================================================
// ==== SPI device and Display initialization functions ====
// =========================================================


//================================================
void EPD_display_reset(uint8_t type, uint8_t mode)
{
    // Initialize display
    if ((type == DISP_TYPE_EPD_2_9_GDEH) || (type == DISP_TYPE_EPD_2_9_DEPG)) {
        EPD_2_9_init(type);
    }
    else EPD_4_2_reset(type, mode);
}

//Initialize the display SPI interface
//==============================================
int EPD_display_init(display_epd_obj_t *epd_dev)
{
    // Check display type
    if ((epd_dev->type == DISP_TYPE_EPD_2_9_GDEH) || (epd_dev->type == DISP_TYPE_EPD_2_9_DEPG)) {
        epd_busy_level = 1;
        status_cmd = 0x2F;
        busy_bit = 2;
        ready_tmo = 4000;
    }
    else if ((epd_dev->type == DISP_TYPE_EPD_4_2) || (epd_dev->type == DISP_TYPE_EPD_4_2_C)) {
        epd_busy_level = 0;
        status_cmd = 0x71;
        busy_bit = 0;
        if (epd_dev->type == DISP_TYPE_EPD_4_2) ready_tmo = 12000;
        else ready_tmo = 35000;
    }
    else {
        LOGE(TAG, "Unsupported display type");
        return -9;
    }

    char spidev[16] = { 0 };
    int spi_func = GPIO_FUNC_SPI0;

    sprintf(spidev, "/dev/spi%d", epd_dev->spi_num & 1);
    if ((epd_dev->spi_num & 1) == 1) spi_func = GPIO_FUNC_SPI1;

    if (EPD_SpiPinsInit(epd_dev, spi_func) != 0) {
        return -1;
    }

    spi_device = 0;
    spi_handle = io_open(spidev);
    if (spi_handle == 0) {
        LOGE(TAG, "Error opening SPI device");
        return -2;
    }

    spi_device = spi_get_device(spi_handle, 0, SPI_FF_STANDARD, 1 << (epd_dev->spi_num & 1), 8);
    if (!spi_device) {
        io_close(spi_handle);
        spi_hard_deinit(epd_dev);
        LOGE(TAG, "Error opening SPI device");
        return -3;
    }

    spi_dev_master_config_half_duplex(spi_device, epd_dev->mosi, -1);
    epd_dev->speed = (uint32_t)spi_dev_set_clock_rate(spi_device, epd_dev->speed);


    if (EPD_EpdPinsInit(epd_dev) != 0) {
        io_close(spi_handle);
        spi_hard_deinit(epd_dev);
        LOGE(TAG, "Error configuring pins");
        return -4;
    }

    epd_dev->speed = spi_dev_set_clock_rate(spi_device, epd_dev->speed);
    LOGI(TAG, "Speed set to: %u", epd_dev->speed);

    epd_dev->handle = spi_device;
    LOGD(TAG, "EPD Initialized");

    return 0;
}

// Deinitialize the display SPI interface, free pins
//=================================================
void EPD_display_deinit(display_epd_obj_t *epd_dev)
{
    if (epd_dev->handle) {
        io_close(spi_device);
        io_close(spi_handle);
        spi_hard_deinit(epd_dev);
        epd_hard_deinit(epd_dev);
        spi_device = 0;
        spi_handle = 0;
        epd_dev->handle = 0;
    }
}


// ==============================================
// ===== Display update functions ===============
// ==============================================


//===============================================
void EPD_UpdateScreen(display_epd_obj_t *epd_dev)
{
    if (epd_type == EPD_TYPE_2_9) {
        LOGD(TAG, "2.9\" Full update (%02X)", gs_used_shades);
        if (gs_used_shades > 1) {
            EPD_2_9_display_greyscale(epd_dev->drawBuff, DISPLAY_FLAG_FULL_UPDATE | DISPLAY_FLAG_8BITPIXEL, 7);
        }
        else {
            EPD_2_9_display(epd_dev->drawBuff, DISPLAY_FLAG_FULL_UPDATE);
        }
    }
    else Epd42_SendFrame();
}

//============================================================================================
void EPD_UpdatePartial(display_epd_obj_t *epd_dev, int xstart, int ystart, int xend, int yend)
{
    if (epd_type == EPD_TYPE_2_9) {
        LOGD(TAG, "2.9\" Partial update (%02X)", gs_used_shades);
        if (gs_used_shades > 1) {
            EPD_2_9_display_greyscale(epd_dev->drawBuff, DISPLAY_FLAG_FULL_UPDATE | DISPLAY_FLAG_8BITPIXEL, 7);
        }
        else {
            EPD_2_9_display(epd_dev->drawBuff, DISPLAY_FLAG_FULL_UPDATE);
        }
    }
    else Epd42_SendPartialWindow(xstart, ystart, xend, yend);
}


#endif // MICROPY_USE_EPD


#endif // MICROPY_USE_TFT

