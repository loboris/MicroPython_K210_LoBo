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
 * ==== SPI SLAVE DEVICE =========================
 * SPI slave device works in half-duplex mode only
 * and uses single pin for mosi & miso !
 * ===============================================
 *
 * Data buffer of 256 B ~ 512 KB is used the data area
 * Read only area can be defined at the end of the data area
 *
 * After the command is received, SPI slave driver sends data to master
 * or reads data from master, depending on command type
 * DMA is used for transfers
 *
 * SPI Slave mode command structure:
 * ---------------------------------
   CMD     - 1-byte command code
   ADDR    - 3-byte address, least significant byte first, from/to which the data are read/written by the master
   DATALEN - 3-byte data length, least significant byte first
   CSUM    - 1-byte checksum, sum of previous 7 bytes, XOR checksum is used
 *
 * DATA    - DATALEN bytes are read or written by the master after the 8-byte command sequence
 *
 * CMD values:
   -----------
   0 - SPI_CMD_NO_COMMAND,              no command
   1 - SPI_CMD_TEST_COMMAND,            test command, sends requested number of the same byte (1st byte of the address field)
   2 - SPI_CMD_READ_INFO,               sends the slave info, 18 bytes (10-byte version; 24-bit buffer size; 24-bit ro size; 2-byte csum)
   3 - SPI_CMD_LAST_STATUS,             sends the last command status, 18 bytes (32-bit values: command, error, address, length; 2-byte csum)
   4 - SPI_CMD_WRITE_DATA_BLOCK,        receives the data and saves to spi buffer
   5 - SPI_CMD_WRITE_DATA_BLOCK_CSUM,   receives the data and saves to spi buffer, uses checksum for data verification
   6 - SPI_CMD_READ_DATA_BLOCK,         send the data from spi buffer
   7 - SPI_CMD_READ_DATA_BLOCK_CSUM,    send the data from spi buffer, 2-byte checksum is append to the data

  *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "mpconfigport.h"

#include "devices.h"
#include "hal.h"
#include "syslog.h"

#include "py/mpstate.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"

#include "modmachine.h"
#include "mphalport.h"

#define SLAVE_BUFFER_MIN_SIZE   256
#define SLAVE_BUFFER_MAX_SIZE   (1024*1024)
#define SPI_MASTER_0            0
#define SPI_MASTER_1            1
#define SPI_MASTER_WS2812_0     2
#define SPI_MASTER_WS2812_1     3
#define SPI_SLAVE               4

typedef union _ws2812b_rgb {
    struct
    {
        uint32_t blue  : 8;
        uint32_t red   : 8;
        uint32_t green : 8;
        uint32_t white : 8;
    };
    uint32_t rgb;
} ws2812b_rgb;

typedef struct _ws2812b_buffer_t
{
    size_t      num_pix;
    ws2812b_rgb *rgb_buffer;
} ws2812b_buffer_t;

typedef struct _machine_hw_spi_obj_t {
    mp_obj_base_t base;
    int8_t      spi_num;            // SPI device used (0-1 spi master, 2-3 ws2812, 4 spi slave)
    int8_t      sck;
    int8_t      mosi;
    int8_t      miso;
    int8_t      cs;
    int8_t      mode;
    int8_t      firstbit;
    int8_t      nbits;
    bool        duplex;
    bool        reused_spi;
    uint32_t    baudrate;
    uint32_t    freq;
    handle_t    handle;             // handle to SPI driver
    handle_t    spi_device;
    uint8_t     *slave_buffer;
    uint32_t    buffer_size;
    uint32_t    slave_ro_size;
    bool        slave_biffer_allocated;
    uint32_t    *slave_cb;          // slave callback function
    ws2812b_buffer_t ws2812_buffer;
    uint16_t    ws2812_lo;
    uint16_t    ws2812_hi;
    uint32_t    ws2812_rst;
    uint16_t    ws_lo;
    uint16_t    ws_hi;
    uint32_t    ws_rst;
    uint32_t    ws_needed_buf_size;
    bool        ws2812_white;
    enum {
        MACHINE_HW_SPI_STATE_NONE,
        MACHINE_HW_SPI_STATE_INIT,
        MACHINE_HW_SPI_STATE_DEINIT
    } state;
} machine_hw_spi_obj_t;

static const char *slave_err[8] = {
    "Ok",
    "Wrong command",
    "Cmd CSUM error",
    "Data CRC error",
    "Wrong address",
    "Wrong length",
    "Timeout",
    "Error",
};

static const char *TAG = "[SPI]";
static machine_hw_spi_obj_t *slave_obj = NULL;

static bool neopixel_show(machine_hw_spi_obj_t *self, bool test);

//-----------------------------------------
static unsigned char reverse_lookup[16] = {
    0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
    0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf,
};

/*
//-----------------------------------
static uint8_t reverse_byte(uint8_t n)
{
   // Reverse the top and bottom nibble then swap them.
   return (reverse_lookup[n&0b1111] << 4) | reverse_lookup[n>>4];
}
*/

//-----------------------------------------
static void reverse(uint8_t *buff, int len)
{
    for (int i=0; i<len; i++) {
        buff[i] = (reverse_lookup[buff[i] & 0b1111] << 4) | reverse_lookup[buff[i] >> 4];
    }
}

//-----------------------------------------------------------------------------------------------
STATIC void machine_hw_spi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    machine_hw_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->state != MACHINE_HW_SPI_STATE_INIT) {
        mp_printf(print, "SPI( DEINITIALIZED )");
        return;
    }

    char spi_type[16];
    char bdrate[32];
    if (self->spi_num == SPI_SLAVE) {
        sprintf(spi_type, "SPI SLAVE");
        sprintf(bdrate, "from master");
    }
    else if (self->spi_num <= SPI_MASTER_1) {
        sprintf(spi_type, "SPI MASTER %d", self->spi_num);
        sprintf(bdrate, "%u (%u)", self->baudrate, self->freq);
    }
    else {
        sprintf(spi_type, "SPI WS2812 %d", self->spi_num & 1);
        sprintf(bdrate, "%u (%u)", self->baudrate, self->freq);
    }

    mp_printf(print, "%s\r\n  ( baudrate=%s, SPI_MODE%d, nbits=%d, bit order=%s, %s\r\n    Pins: mosi=%d, miso=%d, sck=%d, cs=%d", spi_type,
            bdrate, self->mode, self->nbits, (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) ? "LSB" : "MSB",
            (self->duplex) ? "Duplex" : "Half-duplex",
            self->mosi, self->miso, self->sck, self->cs);
    mp_printf(print, "\r\n          (pin value of -1 means the pin is not used)");

    if (self->spi_num == SPI_SLAVE) mp_printf(print, "\r\n    buffer_size=%u (%u read_only), callback: %s",
            self->buffer_size, self->slave_ro_size, (self->slave_cb == NULL) ? "False" : "True");

    if ((self->spi_num == SPI_MASTER_WS2812_0) || (self->spi_num == SPI_MASTER_WS2812_1)) {
        mp_printf(print, "\r\n    ws2812: pixels=%u, buffer size=%u (needs=%u)", self->ws2812_buffer.num_pix, self->buffer_size, self->ws_needed_buf_size);
        mp_printf(print, "\r\n    timing: lo=%u (%u), hi=%u (%u), rst=%u (%u)",
                self->ws2812_lo, self->ws_lo, self->ws2812_hi, self->ws_hi, self->ws2812_rst, self->ws_rst);
        mp_printf(print, "\r\n      RGBW: %s", (self->ws2812_white) ? "yes" : "no");
    }
    mp_printf(print, "\r\n  )\r\n");
}

//-----------------------------------------------------
static bool spi_hard_deinit(machine_hw_spi_obj_t *self)
{
    // Deconfigure used pins
    mp_fpioa_cfg_item_t spi_pin_func[4];
    if (self->spi_num == SPI_SLAVE) {
        // === SPI slave
        int n_func = 3;
        spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, self->mosi, GPIO_USEDAS_NONE, FUNC_RESV0};
        spi_pin_func[1] = (mp_fpioa_cfg_item_t){-1, self->sck, GPIO_USEDAS_NONE, FUNC_RESV0};
        spi_pin_func[2] = (mp_fpioa_cfg_item_t){-1, self->cs, GPIO_USEDAS_NONE, FUNC_RESV0};
        if (self->miso >= 0) {
            n_func = 4;
            spi_pin_func[3] = (mp_fpioa_cfg_item_t){-1, self->miso, GPIO_USEDAS_NONE, FUNC_RESV0};
        }

        fpioa_setup_pins(n_func, spi_pin_func);
        fpioa_freeused_pins(n_func, spi_pin_func);
    }
    else if (self->spi_num <= SPI_MASTER_1) {
        // === SPI master
        if (self->reused_spi) {
            if (self->cs >= 0) {
                spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, self->cs, GPIO_USEDAS_NONE, FUNC_RESV0};
                fpioa_setup_pins(1, spi_pin_func);
                fpioa_freeused_pins(1, spi_pin_func);
            }
        }
        else {
            int n_func = 0;
            if (self->cs >= 0) {
                spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, self->cs, GPIO_USEDAS_NONE, FUNC_RESV0};
                n_func++;
            }
            if (self->miso >= 0) {
                spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, self->miso, GPIO_USEDAS_NONE, FUNC_RESV0};
                n_func++;
            }
            spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, self->mosi, GPIO_USEDAS_NONE, FUNC_RESV0};
            n_func++;
            spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, self->sck, GPIO_USEDAS_NONE, FUNC_RESV0};
            n_func++;
            fpioa_setup_pins(n_func, spi_pin_func);
            fpioa_freeused_pins(n_func, spi_pin_func);
        }
    }
    else {
        // === SPI WS2812
        spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, self->mosi, GPIO_USEDAS_NONE, FUNC_RESV0};

        fpioa_setup_pins(1, spi_pin_func);
        fpioa_freeused_pins(1, spi_pin_func);
    }

    return true;
}

//------------------------------------------------------------------------------------------------------
static int spi_master_hard_init(uint8_t mosi, int8_t miso, uint8_t sck, int8_t cs, gpio_pin_func_t func)
{
    // mosi & sck are mandatory, miso & cs can be unused
    int spi_offset = 0, cs_offset = 0;
    if (func == GPIO_FUNC_SPI1) {
        spi_offset = FUNC_SPI1_D0 - FUNC_SPI0_D0;
        cs_offset = 1;
    }
    int n_func = 0;
    uint8_t same_pins = 0;
    mp_fpioa_cfg_item_t spi_pin_func[4];
    bool set_cs = true, set_miso = true;

    if (cs < 0) set_cs = false;
    else if (mp_used_pins[cs].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "CS %s", gpiohs_funcs_in_use[mp_used_pins[cs].func]);
        return -2;
    }

    if (miso < 0) set_miso = false;
    else if ((mp_used_pins[miso].func == func) && (mp_used_pins[miso].usedas == GPIO_USEDAS_MISO)) same_pins++;

    if ((mp_used_pins[mosi].func == func) && (mp_used_pins[mosi].usedas == GPIO_USEDAS_MISO)) same_pins++;
    if ((mp_used_pins[sck].func == func) && (mp_used_pins[sck].usedas == GPIO_USEDAS_MISO)) same_pins++;

    if ((!set_cs) && (!set_miso) && (same_pins == 2)) {
        // === no cs, no miso, same mosi & sck, no pins to set
        return 1;
    }
    else if ((!set_cs) && (same_pins == 3)) {
        // === no cs, same mosi, miso & sck, no pins to set
        return 1;
    }
    else if (same_pins == 3) {
        // === same mosi, miso & sck, set only cs
        spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, cs, GPIO_USEDAS_CS, FUNC_SPI0_SS0+spi_offset+cs_offset};
        fpioa_setup_pins(1, spi_pin_func);
        fpioa_setused_pins(1, spi_pin_func, func);
        return 1;
    }

    // === mosi, miso & sck must be set, check if already used
    if ((set_miso) && (mp_used_pins[miso].func != GPIO_FUNC_NONE)) {
        LOGW(TAG, "MISO %s", gpiohs_funcs_in_use[mp_used_pins[miso].func]);
        return -3;
    }
    if (mp_used_pins[mosi].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "MOSI %s", gpiohs_funcs_in_use[mp_used_pins[mosi].func]);
        return -4;
    }
    if (mp_used_pins[mosi].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "MOSI %s", gpiohs_funcs_in_use[mp_used_pins[mosi].func]);
        return -5;
    }

    // === Configure pins ===
    if (set_cs) {
        spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, cs, GPIO_USEDAS_CS, FUNC_SPI0_SS0+spi_offset+cs_offset};
        n_func++;
    }
    if (set_miso) {
        spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, miso, GPIO_USEDAS_MISO, FUNC_SPI0_D1+spi_offset};
        n_func++;
    }
    spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, mosi, GPIO_USEDAS_MOSI, FUNC_SPI0_D0+spi_offset};
    n_func++;
    spi_pin_func[n_func] = (mp_fpioa_cfg_item_t){-1, sck, GPIO_USEDAS_CLK, FUNC_SPI0_SCLK+spi_offset};
    n_func++;

    // === Setup and mark used pins ===
    fpioa_setup_pins(n_func, spi_pin_func);
    fpioa_setused_pins(n_func, spi_pin_func, func);

    return 0;
}

//---------------------------------------------------------------------------------------------------
static int spi_slave_hard_init(uint8_t mosi, int miso, uint8_t sck, uint8_t cs, gpio_pin_func_t func)
{
    if (mp_used_pins[mosi].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "MOSI %s", gpiohs_funcs_in_use[mp_used_pins[mosi].func]);
        return -1;
    }
    if (mp_used_pins[sck].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "SCK %s", gpiohs_funcs_in_use[mp_used_pins[sck].func]);
        return -3;
    }
    if (mp_used_pins[cs].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "CS %s", gpiohs_funcs_in_use[mp_used_pins[cs].func]);
        return -4;
    }
    int n_func = 3;
    if (miso >= 0) n_func = 4;
    // Configure pins
    mp_fpioa_cfg_item_t spi_pin_func[n_func];

    spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, cs, GPIO_USEDAS_CS, FUNC_SPI_SLAVE_SS};
    spi_pin_func[1] = (mp_fpioa_cfg_item_t){-1, mosi, GPIO_USEDAS_MOSI, FUNC_SPI_SLAVE_D0};
    spi_pin_func[2] = (mp_fpioa_cfg_item_t){-1, sck, GPIO_USEDAS_CLK, FUNC_SPI_SLAVE_SCLK};
    if (n_func == 4) spi_pin_func[3] = (mp_fpioa_cfg_item_t){-1, miso, GPIO_USEDAS_MISO, FUNC_RESV0};

    // Setup and mark used pins
    fpioa_setup_pins(n_func, spi_pin_func);
    fpioa_setused_pins(n_func, spi_pin_func, func);

    return 0;
}

//-----------------------------------------------------------------
static int spi_ws2812_hard_init(uint8_t mosi, gpio_pin_func_t func)
{
    int spi_offset = 0;
    if (func == GPIO_FUNC_SPI1) spi_offset = FUNC_SPI1_D0 - FUNC_SPI0_D0;

    if (mp_used_pins[mosi].func != GPIO_FUNC_NONE) {
        LOGW(TAG, "MOSI %s", gpiohs_funcs_in_use[mp_used_pins[mosi].func]);
        return -1;
    }
    // Configure pins
    mp_fpioa_cfg_item_t spi_pin_func[1];

    spi_pin_func[0] = (mp_fpioa_cfg_item_t){-1, mosi, GPIO_USEDAS_MOSI, FUNC_SPI0_D0+spi_offset};

    // Setup and mark used pins
    fpioa_setup_pins(1, spi_pin_func);
    fpioa_setused_pins(1, spi_pin_func, func);

    return 0;
}

//----------------------------------------------------
STATIC void checkSPImaster(machine_hw_spi_obj_t *self)
{
    if (self->state != MACHINE_HW_SPI_STATE_INIT) {
        mp_raise_msg(&mp_type_OSError, "SPI not initialized");
    }
    if (self->spi_num == SPI_SLAVE) {
        mp_raise_msg(&mp_type_OSError, "SPI not in master mode");
    }
}

//---------------------------------------------------
STATIC void checkSPIslave(machine_hw_spi_obj_t *self)
{
    if (self->state != MACHINE_HW_SPI_STATE_INIT) {
        mp_raise_msg(&mp_type_OSError, "SPI not initialized");
    }
    if (self->spi_num != SPI_SLAVE) {
        mp_raise_msg(&mp_type_OSError, "SPI not in slave mode");
    }
    if (self->slave_buffer == NULL) {
        mp_raise_msg(&mp_type_OSError, "No slave buffer");
    }
}

//----------------------------------------------------
STATIC void checkSPIws2812(machine_hw_spi_obj_t *self)
{
    if (self->state != MACHINE_HW_SPI_STATE_INIT) {
        mp_raise_msg(&mp_type_OSError, "SPI not initialized");
    }
    if (self->spi_num < SPI_MASTER_WS2812_0) {
        mp_raise_msg(&mp_type_OSError, "SPI not in ws2812 mode");
    }
}

//-----------------------------------
int spi_slave_receive_hook(void *ctx)
{
    spi_slave_command_t *slv_cmd = (spi_slave_command_t *)ctx;

    if ((slave_obj) && (slave_obj->slave_cb)) {
        // schedule callback function
        mp_obj_t tuple[4];
        tuple[0] = mp_obj_new_int(slv_cmd->cmd);
        tuple[1] = mp_obj_new_int(slv_cmd->err);
        tuple[2] = mp_obj_new_int(slv_cmd->addr);
        tuple[3] = mp_obj_new_int(slv_cmd->len);

        mp_sched_schedule(slave_obj->slave_cb, mp_obj_new_tuple(4, tuple));
    }
    else {
        LOGD("[SPI_SLAVE]", "cmd=%u, err=%u (%s), addr=%u, len=%u, time=%lu",
                slv_cmd->cmd, slv_cmd->err, slave_err[slv_cmd->err], slv_cmd->addr, slv_cmd->len, slv_cmd->time);
    }
    return 0;
}


//---------------------------------------------------------------------------------------------------------------
mp_obj_t machine_hw_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    enum { ARG_id, ARG_baudrate, ARG_mode, ARG_polarity, ARG_phase, ARG_firstbit, ARG_sck, ARG_mosi, ARG_miso, ARG_cs,
           ARG_duplex, ARG_bits, ARG_buffer, ARG_rolen, ARG_wsnum, ARG_wslo, ARG_wshi, ARG_wsrst, ARG_wswhite };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id,                                                   MP_ARG_INT,  {.u_int = 1} },
        { MP_QSTR_baudrate,         MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_mode,             MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_polarity,         MP_ARG_KW_ONLY                    | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_phase,            MP_ARG_KW_ONLY                    | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_firstbit,         MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = MICROPY_PY_MACHINE_SPI_MSB} },
        { MP_QSTR_sck,              MP_ARG_KW_ONLY  | MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_mosi,             MP_ARG_KW_ONLY  | MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_miso,             MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_cs,               MP_ARG_KW_ONLY  | MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_duplex,           MP_ARG_KW_ONLY                    | MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_bits,             MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = 8} },
        { MP_QSTR_slave_buffer,     MP_ARG_KW_ONLY                    | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_slave_rolen,      MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = 0} },
        { MP_QSTR_ws2812_num,       MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = 1} },
        { MP_QSTR_ws2812_lo,        MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = 400} },
        { MP_QSTR_ws2812_hi,        MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = 850} },
        { MP_QSTR_ws2812_reset,     MP_ARG_KW_ONLY                    | MP_ARG_INT,  {.u_int = 80000} },
        { MP_QSTR_ws2812_white,     MP_ARG_KW_ONLY                    | MP_ARG_BOOL, {.u_bool = false} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_hw_spi_obj_t *self = m_new_obj(machine_hw_spi_obj_t);
    self->base.type = &machine_hw_spi_type;
    self->state = MACHINE_HW_SPI_STATE_NONE;

    self->spi_num = args[ARG_id].u_int;
    if ((self->spi_num < 0) || (self->spi_num > 4)) {
        mp_raise_ValueError("Wrong SPI interface selected");
    }

    int mode = 0;
    if (args[ARG_mode].u_int < 0) {
        if ((args[ARG_polarity].u_bool == 0) && (args[ARG_phase].u_bool == 0)) mode = 0;
        else if ((args[ARG_polarity].u_bool == 0) && (args[ARG_phase].u_bool != 0)) mode = 1;
        else if ((args[ARG_polarity].u_bool != 0) && (args[ARG_phase].u_bool == 0)) mode = 2;
        else if ((args[ARG_polarity].u_bool != 0) && (args[ARG_phase].u_bool != 0)) mode = 3;
    }

    self->state = MACHINE_HW_SPI_STATE_DEINIT;

    if (args[ARG_baudrate].u_int < 0) {
        self->baudrate = 1000000;
        if ((self->spi_num == SPI_MASTER_WS2812_0) || (self->spi_num == SPI_MASTER_WS2812_1)) self->baudrate = 20000000;
    }
    else {
        if ((args[ARG_baudrate].u_int < 1000000) || (args[ARG_baudrate].u_int > 40000000)) {
            mp_raise_ValueError("SPI baudrate out of range (1000000 ~ 40000000)");
        }
        self->baudrate = args[ARG_baudrate].u_int;
    }

    if ((args[ARG_bits].u_int < 4) || (args[ARG_bits].u_int > 32)) {
        mp_raise_ValueError("SPI bits out of range (4 ~ 32)");
    }
    if ((self->spi_num == SPI_MASTER_WS2812_0) || (self->spi_num == SPI_MASTER_WS2812_1)) self->nbits = 32;
    else self->nbits = args[ARG_bits].u_int;

    bool flag = true;
    if ((args[ARG_mosi].u_int < 0) || (args[ARG_mosi].u_int >= FPIOA_NUM_IO)) flag = false;
    else if (self->spi_num == SPI_SLAVE) {
        if ((args[ARG_sck].u_int < 0) || (args[ARG_sck].u_int >= FPIOA_NUM_IO)) flag = false;
        else if ((args[ARG_cs].u_int < 0) || (args[ARG_cs].u_int >= FPIOA_NUM_IO)) flag = false;
        else if ((args[ARG_miso].u_int < -1) || (args[ARG_miso].u_int >= FPIOA_NUM_IO)) flag = false;
    }
    else if (self->spi_num <= SPI_MASTER_1) {
        if ((args[ARG_sck].u_int < 0) || (args[ARG_sck].u_int >= FPIOA_NUM_IO)) flag = false;
        else if ((args[ARG_cs].u_int < -1) || (args[ARG_cs].u_int >= FPIOA_NUM_IO)) flag = false;
        else if ((args[ARG_miso].u_int < -1) || (args[ARG_miso].u_int >= FPIOA_NUM_IO)) flag = false;
    }
    if (!flag) {
        mp_raise_ValueError("not all pins given or out of range");
    }

    char spidev[16] = { 0 };
    int spi_func = GPIO_FUNC_SPI0;

    if (self->spi_num == SPI_SLAVE) {
        sprintf(spidev, "/dev/spi_slave");
        spi_func = GPIO_FUNC_SPI_SLAVE;
    }
    else {
        sprintf(spidev, "/dev/spi%d", self->spi_num & 1);
        if ((self->spi_num & 1) == 1) spi_func = GPIO_FUNC_SPI1;
    }

    flag = true;
    if (self->spi_num <= SPI_MASTER_1) {
        // Init spi master pins
        int ret = spi_master_hard_init(args[ARG_mosi].u_int, args[ARG_miso].u_int, args[ARG_sck].u_int, args[ARG_cs].u_int, spi_func);
        if (ret < 0) flag = false;
        self->reused_spi = (ret == 1);
    }
    else if (self->spi_num == SPI_SLAVE) {
        // Init spi slave pins
        if (spi_slave_hard_init(args[ARG_mosi].u_int, args[ARG_miso].u_int, args[ARG_sck].u_int, args[ARG_cs].u_int, spi_func) != 0) flag = false;
        self->reused_spi = false;
    }
    else {
        // Init ws2812 pin
        if (spi_ws2812_hard_init(args[ARG_mosi].u_int, spi_func) != 0) flag = false;
        self->reused_spi = false;
    }
    if (!flag) {
        mp_raise_ValueError("Error configuring SPI slave device");
    }

    // === SPI peripheral initialization ===
    self->handle = io_open(spidev);
    if (self->handle == 0) {
        mp_raise_ValueError("Error opening SPI device");
    }

    // Set configuration
    self->mode = mode;
    self->firstbit = args[ARG_firstbit].u_int & 1;
    self->mosi = args[ARG_mosi].u_int;
    self->miso = args[ARG_miso].u_int;
    self->duplex = args[ARG_duplex].u_bool;
    self->sck = args[ARG_sck].u_int;
    self->cs = args[ARG_cs].u_int;
    self->slave_buffer = NULL;
    self->slave_cb = NULL;
    if (args[ARG_rolen].u_int < 0) self->slave_ro_size = 0;
    else self->slave_ro_size = args[ARG_rolen].u_int;

    self->ws2812_buffer.rgb_buffer = NULL;
    self->ws2812_hi = args[ARG_wshi].u_int;
    self->ws2812_lo = args[ARG_wslo].u_int;
    self->ws2812_rst = args[ARG_wsrst].u_int;
    self->ws_hi = 0;
    self->ws_lo = 0;
    self->ws_rst = 0;
    self->ws2812_rst = args[ARG_wsrst].u_int;
    self->ws2812_white = args[ARG_wswhite].u_bool;

    if (self->spi_num != SPI_SLAVE) {
        // SPI master or SPI WS2812
        self->spi_device = spi_get_device(self->handle, mode, SPI_FF_STANDARD, 1 << (self->spi_num & 1), self->nbits);
        if (!self->spi_device) {
            io_close(self->handle);
            spi_hard_deinit(self);
            mp_raise_ValueError("Error opening SPI device");
        }

        if ((self->duplex == false) && (self->miso < 0))
            spi_dev_master_config_half_duplex(self->spi_device, self->mosi, self->miso);
        self->freq = (uint32_t)spi_dev_set_clock_rate(self->spi_device, args[ARG_baudrate].u_int);
        self->buffer_size = 0;

        if ((self->spi_num == SPI_MASTER_WS2812_0) || (self->spi_num == SPI_MASTER_WS2812_1)) {
            // Init WS2812
            if ((args[ARG_wsnum].u_int < 1) || (args[ARG_wsnum].u_int > 1024)) {
                mp_raise_ValueError("WS2812 pixel number out of range (1 ~ 1024)");
            }
            self->ws2812_buffer.num_pix = args[ARG_wsnum].u_int;
            self->ws2812_buffer.rgb_buffer = pvPortMalloc(self->ws2812_buffer.num_pix * sizeof(ws2812b_rgb));
            if (self->ws2812_buffer.rgb_buffer == NULL) {
                io_close(self->handle);
                spi_hard_deinit(self);
                mp_raise_ValueError("Error allocating WS2812 buffer");
            }
            self->buffer_size = self->ws2812_buffer.num_pix * sizeof(ws2812b_rgb);
            memset(self->ws2812_buffer.rgb_buffer, 0x0, self->ws2812_buffer.num_pix * sizeof(ws2812b_rgb));
            neopixel_show(self, true);
        }
    }
    else {
        // SPI Slave
        self->freq = 0;
        self->duplex = false;

        if ((mp_obj_is_int(args[ARG_buffer].u_obj)) || (args[ARG_buffer].u_obj == mp_const_none)) {
            if (args[ARG_buffer].u_obj == mp_const_none) self->buffer_size = 1024;
            else self->buffer_size = mp_obj_get_int(args[ARG_buffer].u_obj);
            if ((self->buffer_size < SLAVE_BUFFER_MIN_SIZE) || (self->buffer_size > SLAVE_BUFFER_MAX_SIZE)) {
                io_close(self->handle);
                spi_hard_deinit(self);
                mp_raise_ValueError("SPI slave buffer size out of range");
            }
            self->slave_buffer = pvPortMalloc(self->buffer_size+8);
            if (self->slave_buffer == NULL) {
                io_close(self->handle);
                spi_hard_deinit(self);
                mp_raise_ValueError("Error allocating SPI slave buffer");
            }
            self->slave_biffer_allocated = true;
        }
        else {
            mp_buffer_info_t bufinfo;
            mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_RW);
            if (bufinfo.len < SLAVE_BUFFER_MIN_SIZE) {
                io_close(self->handle);
                spi_hard_deinit(self);
                mp_raise_ValueError("SPI slave buffer size out of range");
            }
            self->buffer_size = bufinfo.len - 8;
            self->slave_buffer = (uint8_t *)bufinfo.buf;
            self->slave_biffer_allocated = false;
        }
        if (self->slave_ro_size > (self->buffer_size / 2)) self->slave_ro_size = self->buffer_size / 2;

        memset(self->slave_buffer, 0, self->buffer_size);
        spi_slave_config(self->handle, self->nbits, self->slave_buffer, self->buffer_size, self->slave_ro_size,
                spi_slave_receive_hook, mp_hal_crc16, MICROPY_TASK_PRIORITY, self->mosi, self->miso);

        slave_obj = self;
    }

    self->state = MACHINE_HW_SPI_STATE_INIT;

    return MP_OBJ_FROM_PTR(self);
}

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_hw_spi_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_baudrate, ARG_firstbit, ARG_duplex, ARG_wsnum, ARG_wslo, ARG_wshi, ARG_wsrst, ARG_wswhite };

    machine_hw_spi_obj_t *self = pos_args[0];
    checkSPImaster(self);

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate,         MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_firstbit,         MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_duplex,           MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_ws2812_num,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_ws2812_lo,        MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_ws2812_hi,        MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_ws2812_reset,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_ws2812_white,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if ((self->spi_num != SPI_SLAVE) && (args[ARG_baudrate].u_int >= 1000000) && (args[ARG_baudrate].u_int <= 40000000)) {
        self->baudrate = args[ARG_baudrate].u_int;
        self->freq = (uint32_t)spi_dev_set_clock_rate(self->spi_device, args[ARG_baudrate].u_int);
    }
    if ((args[ARG_firstbit].u_int == MICROPY_PY_MACHINE_SPI_MSB) || (args[ARG_firstbit].u_int == MICROPY_PY_MACHINE_SPI_MSB)) {
        self->firstbit = args[ARG_firstbit].u_int;
    }
    if ((args[ARG_duplex].u_int == 0) || (args[ARG_duplex].u_int == 1)) {
        bool duplex = args[ARG_duplex].u_bool;
        if ((self->spi_num <= SPI_MASTER_1) && (duplex != self->duplex)) {
            self->duplex = args[ARG_duplex].u_bool;
            if ((self->duplex == false) && (self->miso < 0))
                spi_dev_master_config_half_duplex(self->spi_device, self->mosi, self->miso);
        }
    }

    if ((args[ARG_wsnum].u_int > 0) && (args[ARG_wsnum].u_int <= 1024) && (args[ARG_wsnum].u_int != self->ws2812_buffer.num_pix)) {
        ws2812b_rgb *old_buf = self->ws2812_buffer.rgb_buffer;
        uint16_t old_num = self->ws2812_buffer.num_pix;
        self->ws2812_buffer.num_pix = args[ARG_wsnum].u_int;
        self->ws2812_buffer.rgb_buffer = pvPortMalloc(self->ws2812_buffer.num_pix * sizeof(ws2812b_rgb));
        if (self->ws2812_buffer.rgb_buffer == NULL) {
            self->ws2812_buffer.rgb_buffer = old_buf;
            self->ws2812_buffer.num_pix = old_num;
        }
        else {
            if (old_buf) vPortFree(old_buf);
            self->buffer_size = self->ws2812_buffer.num_pix * sizeof(ws2812b_rgb);
            memset(self->ws2812_buffer.rgb_buffer, 0x0, self->ws2812_buffer.num_pix * sizeof(ws2812b_rgb));
        }
    }
    if (args[ARG_wshi].u_int > 100) self->ws2812_hi = args[ARG_wshi].u_int;
    if (args[ARG_wslo].u_int > 100) self->ws2812_lo = args[ARG_wslo].u_int;
    if (args[ARG_wsrst].u_int > 10000) self->ws2812_rst = args[ARG_wsrst].u_int;
    if (args[ARG_wswhite].u_int > 0) self->ws2812_white = args[ARG_wswhite].u_int & 1;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_spi_init_obj, 0, machine_hw_spi_init);

//-----------------------------------------------------
STATIC mp_obj_t machine_hw_spi_deinit(mp_obj_t self_in)
{
    machine_hw_spi_obj_t *self = self_in;
    if (self->state != MACHINE_HW_SPI_STATE_INIT) {
        mp_raise_msg(&mp_type_OSError, "SPI not initialized");
    }

    if (self->spi_num == SPI_SLAVE) spi_slave_deinit(self->handle);

    io_close(self->handle);
    self->handle = 0;
    spi_hard_deinit(self);

    if ((self->slave_biffer_allocated) && (self->slave_buffer)) vPortFree(self->slave_buffer);
    if (self->ws2812_buffer.rgb_buffer) vPortFree(self->ws2812_buffer.rgb_buffer);

    self->state = MACHINE_HW_SPI_STATE_NONE;
    slave_obj = NULL;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_spi_deinit_obj, machine_hw_spi_deinit);


//----------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_read(size_t n_args, const mp_obj_t *args)
{
    machine_hw_spi_obj_t *self = args[0];
    checkSPImaster(self);

    vstr_t vstr;
    vstr_init_len(&vstr, mp_obj_get_int(args[1]));
    memset(vstr.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, vstr.len);

    int rdlen = io_read(self->spi_device, (uint8_t *)vstr.buf, vstr.len);

    if (rdlen == vstr.len) {
        if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) reverse((uint8_t *)vstr.buf, vstr.len);
        return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_spi_read_obj, 2, 3, mp_machine_spi_read);

//--------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_readinto(size_t n_args, const mp_obj_t *args)
{
    machine_hw_spi_obj_t *self = args[0];
    checkSPImaster(self);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_WRITE);
    memset(bufinfo.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, bufinfo.len);

    int rdlen = io_read(self->spi_device, (uint8_t *)bufinfo.buf, bufinfo.len);

    if (rdlen == bufinfo.len) {
        if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) reverse((uint8_t *)bufinfo.buf, bufinfo.len);
        return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_spi_readinto_obj, 2, 3, mp_machine_spi_readinto);

//---------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_write(mp_obj_t self_in, mp_obj_t wr_buf)
{
    machine_hw_spi_obj_t *self = self_in;
    checkSPImaster(self);

    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);

    uint8_t *srcbuf = (uint8_t *)src.buf;

    if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) {
        srcbuf = pvPortMalloc(src.len);
        memcpy(srcbuf, src.buf, src.len);
        reverse((uint8_t *)srcbuf, src.len);
    }
    int wrlen = io_write(self->spi_device, srcbuf, src.len);

    if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) vPortFree(srcbuf);
    if (wrlen > 0) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_spi_write_obj, mp_machine_spi_write);

//----------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_write_readinto(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    machine_hw_spi_obj_t *self = pos_args[0];
    checkSPImaster(self);

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_source,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_dest,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_delay,                     MP_ARG_INT, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t src;
    mp_get_buffer_raise(args[0].u_obj, &src, MP_BUFFER_READ);
    mp_buffer_info_t dest;
    mp_get_buffer_raise(args[1].u_obj, &dest, MP_BUFFER_WRITE);
    int delay = args[2].u_int;
    if ((delay < 0) || (delay >= 0x10000)) {
        mp_raise_ValueError("delay out of range (0 ~ 65535)");
    }

    uint8_t *srcbuf = (uint8_t *)src.buf;
    if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) {
        srcbuf = pvPortMalloc(src.len);
        memcpy(srcbuf, src.buf, src.len);
        reverse((uint8_t *)src.buf, src.len);
    }
    int rdlen = 0;
    if (self->duplex) {
        rdlen = spi_dev_transfer_full_duplex(self->spi_device, srcbuf, src.len, (uint8_t *)dest.buf, dest.len);
    }
    else {
        if (delay) rdlen = spi_dev_transfer_sequential_with_delay(self->spi_device, srcbuf, src.len, (uint8_t *)dest.buf, dest.len, (uint16_t)delay);
        else rdlen = spi_dev_transfer_sequential(self->spi_device, srcbuf, src.len, (uint8_t *)dest.buf, dest.len);
    }

    if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) vPortFree(srcbuf);
    if (rdlen > 0) {
        if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) reverse((uint8_t *)dest.buf, dest.len);
        return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_spi_write_readinto_obj, 0, mp_machine_spi_write_readinto);

//---------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_read_from_mem(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    machine_hw_spi_obj_t *self = pos_args[0];
    checkSPImaster(self);

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_address,  MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_length,   MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_addrlen,                    MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_lsb,                        MP_ARG_BOOL, {.u_bool = true} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t wrbuf[4];
    uint8_t *rdbuf = NULL;
    if (args[1].u_int > 0) {
        rdbuf = pvPortMalloc(args[1].u_int);
    }
    if (rdbuf == NULL) {
        return mp_const_false;
    }
    uint32_t addr = (uint32_t)args[0].u_int;

    int adrlen = args[2].u_int;
    if (adrlen <= 0) {
        adrlen = 1;
        if (addr > 0xFF) adrlen++;
        if (addr > 0xFFFF) adrlen++;
        if (addr > 0xFFFFFF) adrlen++;
    }
    if (adrlen < 1) adrlen = 1;
    if (adrlen > 4) adrlen = 4;

    if (args[3].u_bool) {
        wrbuf[0] = addr & 0xFF;
        wrbuf[1] = (addr >> 8) & 0xFF;
        wrbuf[2] = (addr >> 16) & 0xFF;
        wrbuf[3] = (addr >> 24) & 0xFF;
    }
    else {
        wrbuf[3] = addr & 0xFF;
        wrbuf[2] = (addr >> 8) & 0xFF;
        wrbuf[1] = (addr >> 16) & 0xFF;
        wrbuf[0] = (addr >> 24) & 0xFF;
    }

    if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) reverse(wrbuf, adrlen);
    int rdlen = 0;
    if (self->duplex) {
        rdlen = spi_dev_transfer_full_duplex(self->spi_device, (const uint8_t *)wrbuf, adrlen, rdbuf, args[1].u_int);
    }
    else {
        rdlen = spi_dev_transfer_sequential(self->spi_device, (const uint8_t *)wrbuf, adrlen, rdbuf, args[1].u_int);
    }

    mp_obj_t res = mp_const_none;
    if (rdlen > 0) {
        if (self->firstbit == MICROPY_PY_MACHINE_SPI_LSB) reverse((uint8_t *)rdbuf, args[1].u_int);
        res = mp_obj_new_bytes(rdbuf, args[1].u_int);
    }
    else res = mp_const_empty_bytes;

    vPortFree(rdbuf);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_spi_read_from_mem_obj, 0, mp_machine_spi_read_from_mem);

//----------------------------------------------------------------------
STATIC mp_obj_t _spi_slavecmd(uint8_t cmd, uint32_t addr, uint32_t size)
{
    char buf[8];
    buf[0] = cmd;
    buf[1] = addr & 0xff;
    buf[2] = (addr >> 8) & 0xff;
    buf[3] = (addr >> 16) & 0xff;
    buf[4] = size & 0xff;
    buf[5] = (size >> 8) & 0xff;
    buf[6] = (size >> 16) & 0xff;
    buf[7] = 0;
    for (int i=0; i<7; i++) {
        buf[7] ^= buf[i];
    }
    return mp_obj_new_str_of_type(&mp_type_bytes, (const byte*)buf, 8);
}
//--------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_slavecmd(size_t n_args, const mp_obj_t *args)
{
    machine_hw_spi_obj_t *self = args[0];
    checkSPImaster(self);

    return _spi_slavecmd((uint8_t)mp_obj_get_int(args[1]), (uint32_t)mp_obj_get_int(args[2]), (uint32_t)mp_obj_get_int(args[3]));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_spi_slavecmd_obj, 4, 4, mp_machine_spi_slavecmd);


//------------------------------------------------------------------------
static void _check_addr_len(machine_hw_spi_obj_t *self, int addr, int len)
{
    if ((len < 1) || (len > self->buffer_size)) {
        mp_raise_ValueError("Length out of range");
    }
    if (addr >= self->buffer_size) {
        mp_raise_ValueError("Address not in slave data buffer");
    }
    if ((addr + len) > self->buffer_size) {
        mp_raise_ValueError("Data outside buffer");
    }
}

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_slave_setdata(mp_obj_t self_in, mp_obj_t buf_in, mp_obj_t addr_in)
{
    machine_hw_spi_obj_t *self = self_in;
    checkSPIslave(self);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    int addr = mp_obj_get_int(addr_in);
    _check_addr_len(self, addr, bufinfo.len);

    memcpy(self->slave_buffer + addr, bufinfo.buf, bufinfo.len);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_spi_slave_setdata_obj, mp_machine_spi_slave_setdata);

//--------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_slave_setbuffer(mp_obj_t self_in, mp_obj_t byte_in)
{
    machine_hw_spi_obj_t *self = self_in;
    checkSPIslave(self);

    uint8_t fill_byte = (uint8_t)mp_obj_get_int(byte_in);

    memset(self->slave_buffer, fill_byte, self->buffer_size);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_spi_slave_setbuffer_obj, mp_machine_spi_slave_setbuffer);

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_slave_getdata(mp_obj_t self_in, mp_obj_t addr_in, mp_obj_t len_in)
{
    machine_hw_spi_obj_t *self = self_in;
    checkSPIslave(self);

    int addr = mp_obj_get_int(addr_in);
    int len = mp_obj_get_int(len_in);
    _check_addr_len(self, addr, len);

    uint8_t *databuf = pvPortMalloc(len);
    if (databuf == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating data buffer"));
    }

    mp_obj_t data;
    memcpy(databuf, self->slave_buffer + addr, len);
    data = mp_obj_new_bytes(databuf, len);

    vPortFree(databuf);
    // Return buffer data as byte array
    return data;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_spi_slave_getdata_obj, mp_machine_spi_slave_getdata);

//----------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_spi_slave_callback(mp_obj_t self_in, mp_obj_t func)
{
    machine_hw_spi_obj_t *self = self_in;
    checkSPIslave(self);

    if ((!mp_obj_is_fun(func)) && (!mp_obj_is_meth(func)) && (func != mp_const_none)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Function argument required"));
    }

    if (func == mp_const_none) self->slave_cb = NULL;
    else self->slave_cb = func;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_spi_slave_callback_obj, mp_machine_spi_slave_callback);

// ==== WS2812 functions =================================================================================

//--------------------------------------------------------------
static bool neopixel_show(machine_hw_spi_obj_t *self, bool test)
{
    uint32_t longbit;
    uint32_t shortbit;
    uint32_t resbit;
    uint32_t i = 0;
    size_t ws_cnt = self->ws2812_buffer.num_pix;
    uint32_t *ws_data = (uint32_t *)self->ws2812_buffer.rgb_buffer;
    uint32_t clk_time = 1e9 / self->freq; // nanosecond per SPI clock

    longbit = (self->ws2812_hi + clk_time - 1) / clk_time;
    shortbit = (self->ws2812_lo + clk_time - 1) / clk_time;
    resbit = (self->ws2812_rst / clk_time);
    uint32_t reset_cnt = ((resbit + 7) / 8 + 3) / 4;
    self->ws_hi = longbit*clk_time;
    self->ws_lo = shortbit*clk_time;
    self->ws_rst = resbit*clk_time;

    uint32_t ws_mask = 0x800000;
    uint32_t spi_send_cnt = (((ws_cnt * 24 * (longbit + shortbit) + resbit + 7) / 8) + 3) / 4;
    if (self->ws2812_white) {
        spi_send_cnt = (((ws_cnt * 32 * (longbit + shortbit) + resbit + 7) / 8) + 3) / 4;
        ws_mask = 0x80000000;
    }
    uint32_t temp_buf_size = (spi_send_cnt + reset_cnt) * 4;
    self->ws_needed_buf_size = temp_buf_size;

    LOGD(TAG, "WS2812: ns/clk=%u, LongBit=%u ns, ShortBit=%u ns, Rst=%u ns, RstCnt=%u", clk_time, longbit*clk_time, shortbit*clk_time, resbit*clk_time, reset_cnt);

    if (test) return true;

    if (clk_time > (self->ws2812_hi / 2)) {
        LOGE(TAG, "WS2812: clock time to long (%d)", clk_time);
        return false;
    }
    uint32_t *tmp_spi_data = (uint32_t *)pvPortMalloc(temp_buf_size);
    if (tmp_spi_data == NULL) {
        LOGE(TAG, "WS2812: error allocating temporary buffer of %u bytes", temp_buf_size);
        return false;
    }

    const uint8_t *ws2812b_spi_send = (const uint8_t *)tmp_spi_data;

    memset(tmp_spi_data, 0, temp_buf_size);
    uint32_t *spi_data = tmp_spi_data;
    spi_data += reset_cnt;
    int pos = 31;
    uint32_t long_cnt = longbit;
    uint32_t short_cnt = shortbit;
    for (i = 0; i < ws_cnt; i++)
    {
        for (uint32_t mask = ws_mask; mask > 0; mask >>= 1)
        {
            long_cnt = longbit;
            short_cnt = shortbit;

            if (ws_data[i] & mask)
            {
                while (long_cnt--)
                {
                    *(spi_data) |= (1 << (pos--));
                    if (pos < 0)
                    {
                        spi_data++;
                        pos = 31;
                    }
                }
                while (short_cnt--)
                {
                    *(spi_data) &= ~(1 << (pos--));
                    if (pos < 0)
                    {
                        spi_data++;
                        pos = 31;
                    }
                }
            }
            else
            {
                while (short_cnt--)
                {
                    *(spi_data) |= (1 << (pos--));
                    if (pos < 0)
                    {
                        spi_data++;
                        pos = 31;
                    }
                }
                while (long_cnt--)
                {
                    *(spi_data) &= ~(1 << (pos--));
                    if (pos < 0)
                    {
                        spi_data++;
                        pos = 31;
                    }
                }
            }
        }
    }

    int res = io_write(self->spi_device, ws2812b_spi_send, temp_buf_size);
    LOGD(TAG, "WS2812: sent %d (%u) bytes", res, temp_buf_size);
    vPortFree(tmp_spi_data);
    return true;
}

//------------------------------------
static float Min(double a, double b) {
    return a <= b ? a : b;
}

//------------------------------------
static float Max(double a, double b) {
    return a >= b ? a : b;
}

// Convert 24-bit color to HSB representation
//--------------------------------------------------------------------------
static void rgb_to_hsb( uint32_t color, float *hue, float *sat, float *bri )
{
    float delta, min;
    float h = 0, s, v;
    uint8_t green = (color >> 16) & 0xFF;
    uint8_t red = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;

    min = Min(Min(red, green), blue);
    v = Max(Max(red, green), blue);
    delta = v - min;

    if (v == 0.0) s = 0;
    else s = delta / v;

    if (s == 0) h = 0.0;
    else
    {
        if (red == v)
            h = (green - blue) / delta;
        else if (green == v)
            h = 2 + (blue - red) / delta;
        else if (blue == v)
            h = 4 + (red - green) / delta;

        h *= 60;

        if (h < 0.0) h = h + 360;
    }

    *hue = h;
    *sat = s;
    *bri = v / 255;
}

// Convert HSB color to 24-bit color representation
//-------------------------------------------------------------------
static uint32_t hsb_to_rgb(float _hue, float _sat, float _brightness)
{
    float red = 0.0;
    float green = 0.0;
    float blue = 0.0;

    if (_sat == 0.0) {
        red = _brightness;
        green = _brightness;
        blue = _brightness;
    }
    else {
        if (_hue >= 360.0) _hue = fmod(_hue, 360);

        int slice = (int)(_hue / 60.0);
        float hue_frac = (_hue / 60.0) - slice;

        float aa = _brightness * (1.0 - _sat);
        float bb = _brightness * (1.0 - _sat * hue_frac);
        float cc = _brightness * (1.0 - _sat * (1.0 - hue_frac));

        switch(slice) {
            case 0:
                red = _brightness;
                green = cc;
                blue = aa;
                break;
            case 1:
                red = bb;
                green = _brightness;
                blue = aa;
                break;
            case 2:
                red = aa;
                green = _brightness;
                blue = cc;
                break;
            case 3:
                red = aa;
                green = bb;
                blue = _brightness;
                break;
            case 4:
                red = cc;
                green = aa;
                blue = _brightness;
                break;
            case 5:
                red = _brightness;
                green = aa;
                blue = bb;
                break;
            default:
                red = 0.0;
                green = 0.0;
                blue = 0.0;
                break;
        }
    }

    return (uint32_t)((uint8_t)(green * 255.0) << 16) | ((uint8_t)(red * 255.0) << 8) | ((uint8_t)(blue * 255.0));
}

//-----------------------------------------------------
STATIC mp_obj_t machine_neopixel_show(mp_obj_t self_in)
{
    machine_hw_spi_obj_t *self = self_in;
    checkSPIws2812(self);

    MP_THREAD_GIL_EXIT();
    neopixel_show(self, false);
    MP_THREAD_GIL_ENTER();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_neopixel_show_obj, machine_neopixel_show);


//------------------------------------------------------
STATIC mp_obj_t machine_neopixel_clear(mp_obj_t self_in)
{
    machine_hw_spi_obj_t *self = self_in;
    checkSPIws2812(self);

    memset(self->ws2812_buffer.rgb_buffer, 0x0, self->ws2812_buffer.num_pix * sizeof(ws2812b_rgb));
    MP_THREAD_GIL_EXIT();
    neopixel_show(self, false);
    MP_THREAD_GIL_ENTER();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_neopixel_clear_obj, machine_neopixel_clear);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_neopixel_set(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_pos,   MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = 1} },
        { MP_QSTR_color, MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = 0} },
        { MP_QSTR_num,                     MP_ARG_INT,  {.u_int = 1} },
        { MP_QSTR_update,                  MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_brightness,              MP_ARG_INT,  { .u_int = 255 } },
        { MP_QSTR_white,                   MP_ARG_INT,  { .u_int = 0 } },
    };
    machine_hw_spi_obj_t *self = pos_args[0];
    checkSPIws2812(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t bcolor;
    int pos = args[0].u_int;
    int cnt = args[2].u_int;
    uint32_t color = (uint32_t)args[1].u_int;
    uint8_t brightness = args[4].u_int & 0xFF;
    if (brightness < 255) {
        bcolor = color;
        color = 0;
        color |= (bcolor & 0xFF) * brightness / 255;
        color |= (((bcolor >> 8) & 0xFF) * brightness / 255) << 8;
        color |= (((bcolor >> 16) & 0xFF) * brightness / 255) << 16;
    }
    if (self->ws2812_white) {
        color <<= 8;
        color |= args[5].u_int && 0xFF;      // white
    }

    if (pos < 1) pos = 1;
    if (pos > self->ws2812_buffer.num_pix) pos = self->ws2812_buffer.num_pix;
    if (cnt < 1) cnt = 1;
    if ((cnt + pos - 1) > self->ws2812_buffer.num_pix) cnt = self->ws2812_buffer.num_pix - pos + 1;

    ws2812b_rgb *pixel;
    for (uint16_t i = 0; i < cnt; i++) {
        pixel = self->ws2812_buffer.rgb_buffer + i+pos-1;
        pixel->rgb = color;
    }

    if (args[3].u_bool) {
        MP_THREAD_GIL_EXIT();
        neopixel_show(self, false);
        MP_THREAD_GIL_ENTER();
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(machine_neopixel_set_obj, 3, machine_neopixel_set);

//---------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_neopixel_setHSB(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    const mp_arg_t allowed_args[] = {
        { MP_QSTR_pos,         MP_ARG_REQUIRED | MP_ARG_INT,  { .u_int = 0} },
        { MP_QSTR_hue,         MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_saturation,  MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_brightness,  MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_num,                           MP_ARG_INT,  { .u_int = 1} },
        { MP_QSTR_update,                        MP_ARG_BOOL, { .u_bool = true} },
    };
    machine_hw_spi_obj_t *self = pos_args[0];
    checkSPIws2812(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_float_t hue = mp_obj_get_float(args[1].u_obj);
    mp_float_t sat = mp_obj_get_float(args[2].u_obj);
    mp_float_t bri = mp_obj_get_float(args[3].u_obj);

    uint32_t color = hsb_to_rgb(hue, sat, bri);
    if (self->ws2812_white) {
        color <<= 8;
    }

    int pos = args[0].u_int;
    int cnt = args[4].u_int;

    if (pos < 1) pos = 1;
    if (pos > self->ws2812_buffer.num_pix) pos = self->ws2812_buffer.num_pix;
    if (cnt < 1) cnt = 1;
    if ((cnt + pos - 1) > self->ws2812_buffer.num_pix) cnt = self->ws2812_buffer.num_pix - pos + 1;

    ws2812b_rgb *pixel;
    for (uint16_t i = 0; i < cnt; i++) {
        pixel = self->ws2812_buffer.rgb_buffer + i+pos-1;
        pixel->rgb = color;
    }

    if (args[5].u_bool) {
        MP_THREAD_GIL_EXIT();
        neopixel_show(self, false);
        MP_THREAD_GIL_ENTER();
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_neopixel_setHSB_obj, 5, machine_neopixel_setHSB);

//-----------------------------------------------------------------------
STATIC mp_obj_t machine_neopixel_get(size_t n_args, const mp_obj_t *args)
{
    machine_hw_spi_obj_t *self = args[0];
    checkSPIws2812(self);

    uint32_t color;
    int pos = mp_obj_get_int(args[1]);
    if (pos < 1) pos = 1;
    if (pos > self->ws2812_buffer.num_pix) pos = self->ws2812_buffer.num_pix;
    int cnt = 0;
    if (n_args > 2) {
        cnt  = mp_obj_get_int(args[2]);
        if (cnt < 1) cnt = 1;
        if ((cnt + pos - 1) > self->ws2812_buffer.num_pix) cnt = self->ws2812_buffer.num_pix - pos + 1;
    }
    if (cnt < 2) {
        ws2812b_rgb *pixel = self->ws2812_buffer.rgb_buffer + pos-1;
        color = pixel->rgb;
        return mp_obj_new_int(color);
    }
    else {
        mp_obj_t pix_tuple[cnt];
        ws2812b_rgb *pixel;
        for (int i=0; i<cnt; i++) {
            pixel = self->ws2812_buffer.rgb_buffer + pos+i-1;
            color = pixel->rgb;
            pix_tuple[i] = mp_obj_new_int(color);
        }
        return mp_obj_new_tuple(cnt, pix_tuple);
    }
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_neopixel_get_obj, 2, 3, machine_neopixel_get);

//-----------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_neopixel_HSBtoRGB(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    const mp_arg_t allowed_args[] = {
        { MP_QSTR_hue,         MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_saturation,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_brightness,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };
    machine_hw_spi_obj_t *self = pos_args[0];
    checkSPIws2812(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_float_t hue = mp_obj_get_float(args[0].u_obj);
    mp_float_t sat = mp_obj_get_float(args[1].u_obj);
    mp_float_t bri = mp_obj_get_float(args[2].u_obj);

    uint32_t color = hsb_to_rgb(hue, sat, bri);

    return mp_obj_new_int(color);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_neopixel_HSBtoRGB_obj, 4, machine_neopixel_HSBtoRGB);

//------------------------------------------------------------------------------
STATIC mp_obj_t machine_neopixel_RGBtoHSB(mp_obj_t self_in, mp_obj_t color_in) {

    machine_hw_spi_obj_t *self = self_in;
    checkSPIws2812(self);

    uint32_t color = mp_obj_get_int(color_in);

    float hue, sat, bri;

    rgb_to_hsb(color, &hue, &sat, &bri);

    mp_obj_t tuple[3];

    tuple[0] = mp_obj_new_float(hue);
    tuple[1] = mp_obj_new_float(sat);
    tuple[2] = mp_obj_new_float(bri);

    return mp_obj_new_tuple(3, tuple);
}
MP_DEFINE_CONST_FUN_OBJ_2(machine_neopixel_RGBtoHSB_obj, machine_neopixel_RGBtoHSB);

//-------------------------------------------------------
STATIC mp_obj_t machine_neopixel_test(mp_obj_t self_in) {

    machine_hw_spi_obj_t *self = self_in;
    checkSPIws2812(self);

    int mindiff = 10000;
    uint16_t best_wslo = self->ws2812_lo;
    uint16_t best_wshi = self->ws2812_hi;

    uint16_t wslo = self->ws2812_lo;
    for (self->ws2812_lo = 100; self->ws2812_lo < 700; self->ws2812_lo += 10) {
        neopixel_show(self, true);
        if (abs(wslo - self->ws_lo) < mindiff) {
            mindiff = abs(wslo - self->ws_lo);
            best_wslo = self->ws_lo;
        }
    }
    self->ws2812_lo = best_wslo;

    uint16_t wshi = self->ws2812_hi;
    mindiff = 10000;
    for (self->ws2812_hi = 400; self->ws2812_hi < 1200; self->ws2812_hi += 10) {
        neopixel_show(self, true);
        if (abs(wshi - self->ws_hi) < mindiff) {
            mindiff = abs(wshi - self->ws_hi);
            best_wshi = self->ws_hi;
        }
    }
    self->ws2812_hi = best_wshi;
    self->ws_lo = best_wslo;
    self->ws_hi = best_wshi;

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_neopixel_test_obj, machine_neopixel_test);


//================================================================
STATIC const mp_rom_map_elem_t machine_spi_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init),                (mp_obj_t)&machine_hw_spi_init_obj },
    { MP_ROM_QSTR(MP_QSTR_deinit),              (mp_obj_t)&machine_hw_spi_deinit_obj },
    { MP_ROM_QSTR(MP_QSTR_read),                (mp_obj_t)&mp_machine_spi_read_obj },
    { MP_ROM_QSTR(MP_QSTR_readinto),            (mp_obj_t)&mp_machine_spi_readinto_obj },
    { MP_ROM_QSTR(MP_QSTR_readfrom_mem),        (mp_obj_t)&mp_machine_spi_read_from_mem_obj },
    { MP_ROM_QSTR(MP_QSTR_write),               (mp_obj_t)&mp_machine_spi_write_obj },
    { MP_ROM_QSTR(MP_QSTR_write_readinto),      (mp_obj_t)&mp_machine_spi_write_readinto_obj },
    { MP_ROM_QSTR(MP_QSTR_slave_cmd),           (mp_obj_t)&mp_machine_spi_slavecmd_obj },

    // Slave methods
    { MP_ROM_QSTR(MP_QSTR_setdata),             (mp_obj_t)&mp_machine_spi_slave_setdata_obj },
    { MP_ROM_QSTR(MP_QSTR_fillbuffer),          (mp_obj_t)&mp_machine_spi_slave_setbuffer_obj },
    { MP_ROM_QSTR(MP_QSTR_getdata),             (mp_obj_t)&mp_machine_spi_slave_getdata_obj },
    { MP_ROM_QSTR(MP_QSTR_callback),            (mp_obj_t)&mp_machine_spi_slave_callback_obj },

    // WS2812 methods
    { MP_ROM_QSTR(MP_QSTR_ws_clear),            (mp_obj_t)&machine_neopixel_clear_obj },
    { MP_ROM_QSTR(MP_QSTR_ws_set),              (mp_obj_t)&machine_neopixel_set_obj },
    { MP_ROM_QSTR(MP_QSTR_ws_setHSB),           (mp_obj_t)&machine_neopixel_setHSB_obj },
    { MP_ROM_QSTR(MP_QSTR_ws_get),              (mp_obj_t)&machine_neopixel_get_obj },
    { MP_ROM_QSTR(MP_QSTR_ws_show),             (mp_obj_t)&machine_neopixel_show_obj },
    { MP_ROM_QSTR(MP_QSTR_ws_HSBtoRGB),         (mp_obj_t)&machine_neopixel_HSBtoRGB_obj },
    { MP_ROM_QSTR(MP_QSTR_ws_RGBtoHSB),         (mp_obj_t)&machine_neopixel_RGBtoHSB_obj },
    { MP_ROM_QSTR(MP_QSTR_ws_test),             (mp_obj_t)&machine_neopixel_test_obj },

    // Constants
    { MP_ROM_QSTR(MP_QSTR_MSB),                 MP_ROM_INT(MICROPY_PY_MACHINE_SPI_MSB) },
    { MP_ROM_QSTR(MP_QSTR_LSB),                 MP_ROM_INT(MICROPY_PY_MACHINE_SPI_LSB) },
    { MP_ROM_QSTR(MP_QSTR_SPI0),                MP_ROM_INT(SPI_MASTER_0) },
    { MP_ROM_QSTR(MP_QSTR_SPI1),                MP_ROM_INT(SPI_MASTER_1) },
    { MP_ROM_QSTR(MP_QSTR_WS2812_0),            MP_ROM_INT(SPI_MASTER_WS2812_0) },
    { MP_ROM_QSTR(MP_QSTR_WS2812_1),            MP_ROM_INT(SPI_MASTER_WS2812_1) },
    { MP_ROM_QSTR(MP_QSTR_SPI_SLAVE),           MP_ROM_INT(SPI_SLAVE) },

    { MP_ROM_QSTR(MP_QSTR_SLAVE_CMD_TEST),      MP_ROM_INT(SPI_CMD_TEST_COMMAND) },
    { MP_ROM_QSTR(MP_QSTR_SLAVE_CMD_INFO),      MP_ROM_INT(SPI_CMD_READ_INFO) },
    { MP_ROM_QSTR(MP_QSTR_SLAVE_CMD_STATUS),    MP_ROM_INT(SPI_CMD_LAST_STATUS) },
    { MP_ROM_QSTR(MP_QSTR_SLAVE_CMD_WRITE),     MP_ROM_INT(SPI_CMD_WRITE_DATA_BLOCK) },
    { MP_ROM_QSTR(MP_QSTR_SLAVE_CMD_WRITE_CSUM),MP_ROM_INT(SPI_CMD_WRITE_DATA_BLOCK_CSUM) },
    { MP_ROM_QSTR(MP_QSTR_SLAVE_CMD_READ),      MP_ROM_INT(SPI_CMD_READ_DATA_BLOCK) },
    { MP_ROM_QSTR(MP_QSTR_SLAVE_CMD_READ_CSUM), MP_ROM_INT(SPI_CMD_READ_DATA_BLOCK_CSUM) },

    { MP_ROM_QSTR(MP_QSTR_BLACK),               MP_ROM_INT(0x00000000) },
    { MP_ROM_QSTR(MP_QSTR_WHITE),               MP_ROM_INT(0x00FFFFFF) },
    { MP_ROM_QSTR(MP_QSTR_RED),                 MP_ROM_INT(0x0000FF00) },
    { MP_ROM_QSTR(MP_QSTR_BLUE),                MP_ROM_INT(0x000000FF) },
    { MP_ROM_QSTR(MP_QSTR_GREEN),               MP_ROM_INT(0x00FF0000) },
    { MP_ROM_QSTR(MP_QSTR_YELLOW),              MP_ROM_INT(0x00FFFF00) },
    { MP_ROM_QSTR(MP_QSTR_CYAN),                MP_ROM_INT(0x00FF00FF) },
    { MP_ROM_QSTR(MP_QSTR_MAGENTA),             MP_ROM_INT(0x0000FFFF) },
    { MP_ROM_QSTR(MP_QSTR_ORANGE),              MP_ROM_INT(0x0080FF00) },
};
MP_DEFINE_CONST_DICT(mp_machine_spi_locals_dict, machine_spi_locals_dict_table);

//=========================================
const mp_obj_type_t machine_hw_spi_type = {
    { &mp_type_type },
    .name = MP_QSTR_SPI,
    .print = machine_hw_spi_print,
    .make_new = machine_hw_spi_make_new,
    .locals_dict = (mp_obj_dict_t *) &mp_machine_spi_locals_dict,
};
