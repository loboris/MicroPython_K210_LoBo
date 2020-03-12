/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 LoBo (https://github.com/loboris)
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

#if MICROPY_PY_USE_ESP32

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "syslog.h"

#include "py/runtime.h"
#include "py/objstr.h"

#include "modmachine.h"
#include "mphalport.h"

/*
 * ESP32 communication format:
 * ---------------------------
 * status|command   int16_t
 * length           uint16_t
 * payload          uint8_t[length]
 * crc32            uint32_t
 */

// ----------------------------------------------------
// Those defines must be the same as in ESP32 firmware!
// ----------------------------------------------------
#define USE_SPI_MASTER              1
#define SPI_MASTER_POOLING          1
#define DUMMY_BYTES                 4

#define SPI_BUFFER_SIZE             4096
#define SPI_FRAME_SIZE              (SPI_BUFFER_SIZE+8)
#define REQUESTS_URL_MAX_SIZE       256

#define ESP_COMMAND_NONE            0
#define ESP_COMMAND_ECHO            1
#define ESP_COMMAND_GETTIME         2
#define ESP_COMMAND_SETTIME         3
#define ESP_COMMAND_GETVOLTAGE      4
#define ESP_COMMAND_GETKEYS         5
#define ESP_COMMAND_DEEPSLEEP       6
#define ESP_COMMAND_LOGENABLE       7
#define ESP_COMMAND_RQGET           100
#define ESP_COMMAND_RQNEXT          101

#define ESP_COMMAND_WIFIINIT        200
#define ESP_COMMAND_WIFISTATUS      201
#define ESP_COMMAND_READ            255

#define ESP_ERROR_OK                0x0000
#define ESP_ERROR_COMMAND_UNKNOWN   0x0100
#define ESP_ERROR_CRC               0x0200
#define ESP_ERROR_LENGTH            0x0300
#define ESP_ERROR_NOCMD             0x0400
#define ESP_ERROR_FRAME             0x0500
#define ESP_ERROR_PROCESS           0x0600
#define ESP_ERROR_TIMEOUT           0x0700
#define ESP_ERROR_NOTCONNECTED      0x0800
#define ESP_ERROR_SPISLAVE          0x0900

#define ESP_STATUS_RQFINISH         0x6000
#define ESP_STATUS_MREQUEST         0x6100
// ----------------------------------------------------


static const char *esp32_errors[9] = {
        "Ok",
        "Cmd unknown",
        "CRC Error",
        "Wrong length",
        "No command",
        "Frame Error",
        "Process error"
        "Timeout",
        "Not connected",
        "SPI Slave error",
};


typedef struct _esp32_obj_t {
    mp_obj_base_t           base;
    machine_hw_spi_obj_t    *spi_instance;
    QueueHandle_t           slave_queue;
    int8_t                  rdy;
    int                     rdy_pin;
    bool                    double_wrspeed;
    int16_t                 esp_cmdstat;
    uint16_t                esp_len;
    uint32_t                esp_writetime;
    uint32_t                esp_readtime;
    uint32_t                write_time;
    uint32_t                read_time;
    uint32_t                timeout;
} esp32_obj_t;

#if !USE_SPI_MASTER
static uint8_t __attribute__((aligned(8))) esp_buffer[SPI_FRAME_SIZE];
#endif

const mp_obj_type_t esp32_type;
static const char TAG[] = "[ESP32]";


//-----------------------------------------------
int urlencode(char *url, char *dest_url, int len)
{
    char c, code0, code1;
    int idx = 0;
    bool is_qstr = false;

    memset(dest_url, 0, len);
    // Loop through url, if we find an encode char, replace with % and add hex as ASCII chars
    // If we reach the query string (?), mark the query string encoding
    while (*url) {
        c = *url++;
        if ((!is_qstr) && (c == '?')) is_qstr = true;
        if (c == ' ') {
            if (is_qstr) {
                if (idx < len) dest_url[idx++]= '+';
                else idx++;
            }
            else {
                if (idx < (len-3)) {
                    dest_url[idx++] = '%';
                    dest_url[idx++] = '2';
                    dest_url[idx++] = '0';
                }
                else idx +=3;
            }
        }
        else if (isalnum(c)) {
            if (idx < len) dest_url[idx++] = c;
            else idx++;
        }
        else {
            code1 = (c & 0xf) + '0';
            if ((c & 0xf) > 9) code1 = (c & 0xf) - 10 + 'A';
            c = (c >> 4) & 0xf;
            code0 = c +'0';
            if (c > 9) code0 = c - 10 + 'A';
            if (idx < (len-3)) {
                dest_url[idx++] = '%';
                dest_url[idx++] = code0;
                dest_url[idx++] = code1;
            }
            else idx +=3;
        }
    }
    return idx;

}

#if !USE_SPI_MASTER
//------------------------------------------------------------
static void esp32_spi_send(esp32_obj_t *self, uint8_t *buf_in)
{
    if (self->double_wrspeed) {
        self->spi_instance->baudrate *= 2;
        self->spi_instance->freq = (uint32_t)spi_dev_set_clock_rate(self->spi_instance->spi_device, self->spi_instance->baudrate);
    }
    if (self->esp_len > SPI_BUFFER_SIZE) {
        self->esp_cmdstat |= ESP_ERROR_LENGTH;
        return;
    }

    // Wait for ready signal from ESP32
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 0) {
    }
    uint64_t t_start = mp_hal_ticks_us();

    // Prepare frame
    memset(esp_buffer, 0, SPI_FRAME_SIZE);
    if (buf_in) memcpy(esp_buffer + 4, buf_in, self->esp_len);
    else self->esp_len = 0;
    *(uint16_t *)(&esp_buffer[0]) = self->esp_cmdstat;
    *(uint16_t *)(&esp_buffer[2]) = self->esp_len;
    uint32_t crc = hal_crc32((const void *)esp_buffer, self->esp_len+4, 0);
    memcpy(esp_buffer + self->esp_len + 4, (void *)&crc, 4);

    // and send it, the write length must be ALIGNED to 4 bytes!
    int wrlen = self->esp_len+8;
    if (wrlen & 3) wrlen = (wrlen & 0xFFFC) + 4;
    int written = io_write(self->spi_instance->spi_device, esp_buffer, wrlen);
    self->write_time = mp_hal_ticks_us() - t_start;

    // Wait until request processed by ESP32
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 1) {
    }
    t_start = mp_hal_ticks_us();
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 0) {
    }
    self->esp_writetime = mp_hal_ticks_us() - t_start;

    if (wrlen != written) {
        self->esp_cmdstat |= ESP_ERROR_FRAME;
        LOGW(TAG, "Write: sent %d, expected %d", written, wrlen);
    }
    if (self->double_wrspeed) {
        self->spi_instance->baudrate /= 2;
        self->spi_instance->freq = (uint32_t)spi_dev_set_clock_rate(self->spi_instance->spi_device, self->spi_instance->baudrate);
    }
    return;
}

//-------------------------------------------------
static void esp32_spi_getStatLen(esp32_obj_t *self)
{
    // Wait for ready signal from ESP32
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 0) {
    }

    self->esp_cmdstat = 0;
    int rdlen = io_read(self->spi_instance->spi_device, esp_buffer, 8);

    // Wait until request processed by ESP32
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 1) {
    }
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 0) {
    }

    if (rdlen == 8) {
        self->esp_cmdstat = *(uint16_t *)(&esp_buffer[0]);
        self->esp_len = *(uint16_t *)(&esp_buffer[2]);
    }
    else {
        self->esp_cmdstat = ESP_ERROR_LENGTH;
        self->esp_len = 0;
    }
}

//-------------------------------------------
static void esp32_spi_read(esp32_obj_t *self)
{
    // Wait for ready signal from ESP32
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 0) {
    }
    uint64_t t_start = mp_hal_ticks_us();

    self->esp_cmdstat = 0;
    // Align receive length to 4 bytes
    int rdlen = self->esp_len+8;
    if (rdlen & 3) rdlen = (rdlen & 0xFFFC) + 4;
    rdlen = io_read(self->spi_instance->spi_device, esp_buffer, rdlen);
    self->read_time = mp_hal_ticks_us() - t_start;

    // Wait until request processed by ESP32
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 1) {
    }
    t_start = mp_hal_ticks_us();
    while (gpio_get_pin_value(gpiohs_handle, self->rdy_pin) == 0) {
    }
    self->esp_readtime = mp_hal_ticks_us() - t_start;

    if (rdlen >= 8) {
        self->esp_cmdstat = *(uint16_t *)(&esp_buffer[0]);
        uint16_t length = *(uint16_t *)(&esp_buffer[2]);
        if (length <= SPI_BUFFER_SIZE) {
            uint32_t crc = 0;
            memcpy((void *)&crc, esp_buffer + length + 4, 4);
            uint32_t calc_crc = hal_crc32((const void *)esp_buffer, length+4, 0);
            if (crc == calc_crc) {
                if (self->esp_len != length) {
                    LOGD(TAG, "Read: received length (%u) <> expected length (%u)", length, self->esp_len);
                }
                self->esp_len = length;
            }
            else {
                LOGD(TAG, "Read: CRC Error,  (len=%u)", length);
                self->esp_cmdstat |= ESP_ERROR_CRC;
            }
        }
        else {
            LOGD(TAG, "Read: Length error (%u)", length);
            self->esp_cmdstat |= ESP_ERROR_LENGTH;
        }
    }
    else {
        self->esp_cmdstat |= ESP_ERROR_FRAME;
    }
}
#else

// Request command processing from ESP32
//---------------------------------------------------------------
static void esp32_spi_command(esp32_obj_t *self, uint8_t *buf_in)
{
    if (self->esp_len > SPI_BUFFER_SIZE) {
        self->esp_cmdstat |= ESP_ERROR_LENGTH;
        return;
    }

    spi_slave_command_t slv_cmd = {0};
    uint8_t *esp_buffer = self->spi_instance->slave_buffer+DUMMY_BYTES;

    // Clean previous slave messages
    while (xQueueReceive(self->slave_queue, &slv_cmd, 0) == pdTRUE) {
        ;
    }

    // Prepare command frame in slave buffer
    memset(esp_buffer, 0, SPI_FRAME_SIZE);
    if (buf_in) memcpy(esp_buffer + 4, buf_in, self->esp_len);
    else self->esp_len = 0;
    *(uint16_t *)(&esp_buffer[0]) = self->esp_cmdstat;
    *(uint16_t *)(&esp_buffer[2]) = self->esp_len;
    uint32_t crc = hal_crc32((const void *)esp_buffer, self->esp_len+4, 0);
    /*if (spi_slave_mutex) {
        if (xSemaphoreTake(spi_slave_mutex, SPI_SLAVE_MUTEX_WAIT_TIME) != pdTRUE) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Slave buffer access timeout"));
        }
    }*/
    memcpy(esp_buffer + self->esp_len + 4, (void *)&crc, 4);

    int tmo = 1000;
    if (self->spi_instance->handshake >= 0) {
        while (tmo) {
            // Pulse handshake line, request ESP32 to check the command
            if (spi_slave_set_handshake(self->spi_instance->handle, 10)) break;
            vTaskDelay(50);
            tmo -= 50;
            if (tmo == 0) {
                self->esp_cmdstat |= ESP_ERROR_TIMEOUT;
                self->esp_len = 0;
                return;
            }
        }
    }

    LOGD(TAG, "Command: Wait for ESP32 response...");
    uint64_t t_start = mp_hal_ticks_us();

    // Wait until ESP has processed the command
    tmo = 1000;
    while (1) {
        if (xQueueReceive(self->slave_queue, &slv_cmd, tmo / portTICK_RATE_MS) == pdTRUE) {
            if (slv_cmd.err != SPI_CMD_ERR_OK) {
                LOGD(TAG, "Command: Slave error %d", slv_cmd.err);
                self->esp_cmdstat |= ESP_ERROR_SPISLAVE;
                self->esp_len = 0;
                break;
            }
            if (slv_cmd.cmd == SPI_CMD_WRITE_DATA_BLOCK) {
                // ESP32 write, this should be the response to the command
                self->read_time = mp_hal_ticks_us() - t_start;

                // The response frame is in the slave buffer
                self->esp_cmdstat = *(uint16_t *)(&esp_buffer[0]);
                uint16_t length = *(uint16_t *)(&esp_buffer[2]);
                if (length <= SPI_BUFFER_SIZE) {
                    uint32_t crc = 0;
                    memcpy((void *)&crc, esp_buffer + length + 4, 4);
                    uint32_t calc_crc = hal_crc32((const void *)esp_buffer, length+4, 0);
                    if (crc == calc_crc) {
                        if (self->esp_len != length) {
                            LOGD(TAG, "Response: received length (%u) <> expected length (%u)", length, self->esp_len);
                        }
                        self->esp_len = length;
                    }
                    else {
                        LOGD(TAG, "Response: CRC Error,  (len=%u)", length);
                        self->esp_cmdstat |= ESP_ERROR_CRC;
                    }
                }
                else {
                    LOGD(TAG, "Response: Length error (%u)", length);
                    self->esp_cmdstat |= ESP_ERROR_LENGTH;
                }
                break;
            }
            else {
                // ESP32 read, first status(command&length) then full frame
                if (slv_cmd.len == 4) continue;  // status requested
                // full command frame requested
                self->write_time = mp_hal_ticks_us() - t_start;
                t_start = mp_hal_ticks_us();
                tmo = self->timeout;
            }
        }
        else {
            self->esp_cmdstat |= ESP_ERROR_TIMEOUT;
            self->esp_len = 0;
            break;
        }
    }
    memset(esp_buffer, 0, 4);
    return;
}
#endif

//-------------------------------------
static void checkSPI(esp32_obj_t *self)
{
    if (self->spi_instance->state != MACHINE_HW_SPI_STATE_INIT) {
        mp_raise_msg(&mp_type_OSError, "SPI not initialized");
    }
}

//----------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t modesp32_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    enum { ARG_spi, ARG_rdy, ARG_dwrspeed };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi,           MP_ARG_REQUIRED                   | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        #if !USE_SPI_MASTER
        { MP_QSTR_rdy,                             MP_ARG_KW_ONLY  | MP_ARG_INT,  { .u_int = -1 } },
        { MP_QSTR_doublewrspeed,                   MP_ARG_KW_ONLY  | MP_ARG_BOOL, { .u_bool = false } },
        #endif
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    esp32_obj_t *self = m_new_obj(esp32_obj_t);
    mp_obj_t spi_inst = args[ARG_spi].u_obj;
    if (!mp_obj_is_type(spi_inst, &machine_hw_spi_type)) {
        mp_raise_ValueError("SPI instance expected");
    }
    self->spi_instance = (machine_hw_spi_obj_t *)args[ARG_spi].u_obj;
    #if USE_SPI_MASTER
    if (self->spi_instance->spi_num != SPI_SLAVE) {
        mp_raise_ValueError("SPI SLAVE instance expected");
    }
    #else
    if (self->spi_num > SPI_MASTER_1) {
        mp_raise_ValueError("SPI MASTER instance expected");
    }
    #endif

    self->base.type = &esp32_type;

    if (!machine_init_gpiohs()) {
        mp_raise_ValueError("Cannot initialize gpiohs");
    }

    // === Get arguments ===

    #if !USE_SPI_MASTER
    self->double_wrspeed = args[ARG_dwrspeed].u_bool;

    // Configure RDY pin
    self->rdy = args[ARG_rdy].u_int;
    if (mp_used_pins[self->rdy].func != GPIO_FUNC_NONE) {
        mp_raise_ValueError(gpiohs_funcs_in_use[mp_used_pins[self->rdy].func]);
    }
    self->rdy_pin = gpiohs_get_free();
    if (self->rdy_pin < 0) {
        mp_raise_ValueError("Error configuring RDY pin");
    }
    LOGD(TAG, "Configure RDY pin (%d)", self->rdy);
    if (fpioa_set_function(self->rdy, FUNC_GPIOHS0 + self->rdy_pin) < 0) {
        gpiohs_set_free(self->rdy_pin);
        mp_raise_ValueError("Error configuring RDY pin");
    }
    gpio_set_drive_mode(gpiohs_handle, self->rdy_pin, GPIO_DM_INPUT_PULL_UP);
    mp_used_pins[self->rdy].func = GPIO_FUNC_PIN;
    mp_used_pins[self->rdy].usedas = GPIO_USEDAS_HANDSHAKE;
    mp_used_pins[self->rdy].gpio = self->rdy_pin;
    mp_used_pins[self->rdy].fpioa_func = FUNC_GPIOHS0 + self->rdy_pin;
    #else

    self->slave_queue = xQueueCreate(8, sizeof(spi_slave_command_t));
    if (self->spi_instance->slave_queue) {
        self->spi_instance->slave_queue = NULL;
        vQueueDelete(self->spi_instance->slave_queue);
    }
    self->spi_instance->slave_queue = self->slave_queue;
    #endif

    return MP_OBJ_FROM_PTR(self);
}

//--------------------------------------------------------------
STATIC mp_obj_t modesp32_echo(mp_obj_t self_in, mp_obj_t wr_buf)
{
    esp32_obj_t *self = self_in;
    checkSPI(self);

    self->esp_cmdstat = ESP_COMMAND_ECHO;
    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);

    uint8_t *srcbuf = (uint8_t *)src.buf;
    self->esp_len = src.len;

    #if !USE_SPI_MASTER
    esp32_spi_send(self, srcbuf);

    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }

    esp32_spi_read(self);
    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }
    LOGD(TAG, "Timings: write=(%u, %u), read=(%u, %u)", self->esp_writetime, self->write_time, self->esp_readtime, self->read_time);
    #else
    esp32_spi_command(self, srcbuf);
    LOGD(TAG, "Timings: write=%u), read=%u", self->write_time, self->read_time);
    uint8_t *esp_buffer = self->spi_instance->slave_buffer;
    #endif

    if (memcmp(srcbuf, esp_buffer+4, self->esp_len) != 0) {
        LOGD(TAG, "Error: echoed data does not match");
        return mp_const_false;
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(modesp32_echo_obj, modesp32_echo);

//------------------------------------------------
STATIC mp_obj_t modesp32_gettime(mp_obj_t self_in)
{
    esp32_obj_t *self = self_in;
    checkSPI(self);

    self->esp_cmdstat = ESP_COMMAND_GETTIME;
    self->esp_len = 0;

    #if !USE_SPI_MASTER
    esp32_spi_send(self, NULL);

    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }

    self->esp_len = 4;
    esp32_spi_read(self);
    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }
    LOGD(TAG, "Timings: write=(%u, %u), read=(%u, %u)", self->esp_writetime, self->write_time, self->esp_readtime, self->read_time);
    #else
    esp32_spi_command(self, NULL);
    LOGD(TAG, "Timings: write=%u), read=%u", self->write_time, self->read_time);
    uint8_t *esp_buffer = self->spi_instance->slave_buffer+4;
    #endif
    uint32_t seconds = 0;
    memcpy(&seconds, esp_buffer+4, 4);
    return mp_obj_new_int(seconds);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(modesp32_gettime_obj, modesp32_gettime);

//------------------------------------------------
STATIC mp_obj_t modesp32_getvoltage(mp_obj_t self_in)
{
    esp32_obj_t *self = self_in;
    checkSPI(self);

    self->esp_cmdstat = ESP_COMMAND_GETVOLTAGE;
    self->esp_len = 0;

    #if !USE_SPI_MASTER
    esp32_spi_send(self, NULL);

    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }

    self->esp_len = 8;
    esp32_spi_read(self);
    #else
    esp32_spi_command(self, NULL);
    LOGD(TAG, "Timings: write=%u), read=%u", self->write_time, self->read_time);
    uint8_t *esp_buffer = self->spi_instance->slave_buffer+4;
    #endif
    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }
    uint32_t v1, v2;
    memcpy(&v1, esp_buffer+4, 4);
    memcpy(&v2, esp_buffer+8, 4);
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(v1);
    tuple[1] = mp_obj_new_int(v2);

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(modesp32_getvoltage_obj, modesp32_getvoltage);

//---------------------------------------------------------------
STATIC mp_obj_t modesp32_rqGET(mp_obj_t self_in, mp_obj_t wr_buf)
{
    esp32_obj_t *self = self_in;
    checkSPI(self);

    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);

    if (src.len >= REQUESTS_URL_MAX_SIZE) {
        mp_raise_ValueError("Max url length exceeded");
    }

    self->esp_cmdstat = ESP_COMMAND_RQGET;
    uint8_t *srcbuf = (uint8_t *)src.buf;
    self->esp_len = src.len;

    #if !USE_SPI_MASTER
    esp32_spi_send(self, srcbuf);

    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }

    // Get command status and response length
    esp32_spi_getStatLen(self);
    if (self->esp_cmdstat != ESP_COMMAND_RQGET) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }

    esp32_spi_read(self);
    if ((self->esp_cmdstat >> 8) != ESP_ERROR_OK) {
        LOGD(TAG, "Error %d, %s", self->esp_cmdstat >> 8, esp32_errors[self->esp_cmdstat >> 8]);
        return mp_const_false;
    }
    LOGD(TAG, "Timings: write=(%u, %u), read=(%u, %u)", self->esp_writetime, self->write_time, self->esp_readtime, self->read_time);
    #else
    esp32_spi_command(self, srcbuf);
    LOGD(TAG, "Timings: write=%u), read=%u", self->write_time, self->read_time);
    uint8_t *esp_buffer = self->spi_instance->slave_buffer+4;
    #endif
    return mp_obj_new_str((const char *)(esp_buffer+4), self->esp_len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(modesp32_rqGET_obj, modesp32_rqGET);


//=============================================================
STATIC const mp_rom_map_elem_t modesp32_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_echo),                MP_ROM_PTR(&modesp32_echo_obj) },
    { MP_ROM_QSTR(MP_QSTR_gettime),             MP_ROM_PTR(&modesp32_gettime_obj) },
    { MP_ROM_QSTR(MP_QSTR_getvoltage),          MP_ROM_PTR(&modesp32_getvoltage_obj) },
    { MP_ROM_QSTR(MP_QSTR_rqGET),               MP_ROM_PTR(&modesp32_rqGET_obj) },
};
STATIC MP_DEFINE_CONST_DICT(modesp32_locals_dict, modesp32_locals_dict_table);

//==============================
const mp_obj_type_t esp32_type = {
    { &mp_type_type },
    .name = MP_QSTR_esp32,
    .make_new = modesp32_make_new,
    .locals_dict = (mp_obj_dict_t*)&modesp32_locals_dict,
};

#endif
