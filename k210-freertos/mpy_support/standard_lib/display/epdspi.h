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

#ifndef _EPDSPI_H_
#define _EPDSPI_H_

#include "mpconfigport.h"

#if MICROPY_USE_TFT


#include "moddisplay.h"

#if MICROPY_USE_EPD

#include "syslog.h"
#include "devices.h"
#include "mphalport.h"

extern handle_t spi_device;
extern handle_t spi_handle;
extern int EPD_ready_timer;

extern int epd_busy_pin;
extern int epd_busy_level;
extern int epd_cs_pin;
extern int epd_dc_pin;
extern int epd_rst_pin;
extern int epd_pwr_pin;

#define SPI_DESELECT()      gpio_set_pin_value(gpiohs_handle, epd_cs_pin, GPIO_PV_HIGH)
#define SPI_SELECT()        gpio_set_pin_value(gpiohs_handle, epd_cs_pin, GPIO_PV_LOW)
#define EPD_WRITE_DATA()    gpio_set_pin_value(gpiohs_handle, epd_dc_pin, GPIO_PV_HIGH)
#define EPD_WRITE_COMMAND() gpio_set_pin_value(gpiohs_handle, epd_dc_pin, GPIO_PV_LOW)



// ===== EPD2IN9 =======================================

extern uint8_t *drawBuff;
extern uint16_t gs_used_shades;
extern uint8_t *drawBuff42;
extern uint8_t *drawBuff42y;
extern uint8_t epd_type;
extern uint8_t dotted_fil;


void EPD_Reset();
void EPD_WaitReady();
//int EPD_get_ready_timer();
void EPD_SPI_Write(uint8_t value);
uint8_t EPD_SPI_Read();
void EPD_WriteCMD(uint8_t command);
void EPD_WriteDATA(uint8_t data);
void EPD_WriteCMD_p1(uint8_t command, uint8_t para);
void EPD_WriteCMD_p2(uint8_t command, uint8_t para1, uint8_t para2);
void EPD_WriteCMD_p3(uint8_t command, uint8_t para1, uint8_t para2, uint8_t para3);
void EPD_WriteCMD_p4(uint8_t command, uint8_t para1, uint8_t para2, uint8_t para3, uint8_t para4);
void EPD_Write(uint8_t *buf, int datalen);
void EPD_SPI_send_data(uint8_t *data, uint32_t len, uint8_t rep);
void EPD_UpdateScreen(display_epd_obj_t *epd_dev);
void EPD_UpdatePartial(display_epd_obj_t *epd_dev, int xstart, int ystart, int xend, int yend);
void EPD_SPI_send_data(uint8_t *data, uint32_t len, uint8_t rep);
void EPD_Write_command_stream(uint8_t command, const uint8_t *data,  unsigned int datalen);
void EPD_Write_command_stream32(uint8_t command, const uint32_t *data,  unsigned int datalen);
int EPD_display_init(display_epd_obj_t *epd_dev);
void EPD_display_reset(uint8_t type, uint8_t mode);
void EPD_display_deinit(display_epd_obj_t *epd_dev);

#endif

#endif // MICROPY_USE_TFT

#endif
