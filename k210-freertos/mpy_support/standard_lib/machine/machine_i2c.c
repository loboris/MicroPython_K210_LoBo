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

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "mpconfigport.h"

#include "devices.h"
#include "hal.h"
#include "syslog.h"

#include "py/mpstate.h"
#include "py/runtime.h"
#include "py/obj.h"

#include "modmachine.h"

#define I2C_MODE_MASTER             1
#define I2C_MODE_SLAVE              2
#define I2C_MAX_DEVICES             3

#define I2C_SLAVE_DRIVER_DELETED    (0xffffa5a5)
#define I2C_SLAVE_START_EVENT       (0xffff005a)
#define I2C_SLAVE_STOP_EVENT        (0xffff00a5)
#define I2C_SLAVE_MAX_BUFFER_LENGTH (0x1000)    // minimal slave buffer size (128 bytes)
#define I2C_SLAVE_MIN_BUFFER_LENGTH (0x0080)    // maximal slave buffer size (4096 bytes)
#define I2C_SLAVE_DEFAULT_BUFF_LEN  256         // default slave buffer size
#define I2C_SLAVE_ADDR_DEFAULT      32          // default slave address

#define I2C_SLAVE_STATUS_ADDR_RX    1
#define I2C_SLAVE_STATUS_DATA_RX    2
#define I2C_SLAVE_STATUS_DATA_TX    4

#define I2C_SLAVE_DUMMY_BYTE        0xFE
#define I2C_SLAVE_FLAG_BUSY         0x80

#define I2C_SLAVE_MUTEX_TIMEOUT     (500 / portTICK_PERIOD_MS)
#define I2C_SLAVE_TASK_STACK_SIZE   configMINIMAL_STACK_SIZE

#define I2C_SLAVE_CBTYPE_NONE       0
#define I2C_SLAVE_CBTYPE_ADDR       1
#define I2C_SLAVE_CBTYPE_DATA_RX    2
#define I2C_SLAVE_CBTYPE_DATA_TX    4

typedef struct {
    uint16_t rxaddr;        // current read address in slave buffer (from master)
    uint16_t rxptr;         // slave buffer rx pointer
    uint16_t rxcount;       // number of received bytes (from master) in current transaction
    uint16_t txaddr;        // current write address in slave buffer (to master)
    uint16_t txptr;         // slave buffer tx pointer
    uint16_t rxovf;         // slave buffer rx overflow count
    uint16_t txovf;         // slave buffer tx overflow count
    uint16_t ro_len;        // slave buffer read-only area length (from the end of buffer)
    uint16_t tmpaddr;       // temporary buffer address in 2-bytes address mode
    uint8_t  status;        // slave transaction status
    uint8_t  dummy;         // reserved
} i2c_slave_state_t;

typedef struct _mp_machine_i2c_obj_t {
    mp_obj_base_t           base;
    uint32_t                speed;              // I2C speed
    uint8_t                 mode;               // I2C mode; master or slave
    handle_t                i2c_handle;         // handle to I2C driver
    uint8_t                 scl;                // SCL pin number
    uint8_t                 sda;                // SDA pin number
    int8_t                  i2c_num;            // I2C device used (0-2)
    // slave only variables
    i2c_slave_handler_t     slave_handler;      // slave only, interrupt handlers
    QueueHandle_t           slave_task_mutex;   // slave only, slave buffer mutex
    QueueHandle_t           slave_mutex;        // slave only, slave buffer mutex
    int8_t                  slave_addr;         // slave only, slave 7-bit address
    uint8_t                 *slave_buffer;      // slave only, data buffer pointer
    uint16_t                slave_buflen;       // slave only, data buffer length
    uint16_t                slave_rolen;        // slave only, read only buffer area length
    bool                    slave_busy;         // use slave busy register
    uint32_t                *slave_cb;          // slave only, slave callback function
    uint8_t                 slave_cbtype;       // bit-mapped callback type
} mp_machine_i2c_obj_t;

typedef struct _slave_task_params_t {
    void *i2c_obj;
    void *thread_handle;
} slave_task_params_t;

slave_task_params_t task_params;

const mp_obj_type_t machine_hw_i2c_type;

static uint8_t i2c_used[I2C_MAX_DEVICES] = { 0 };
static TaskHandle_t i2c_slave_task_handle = NULL;
static i2c_slave_state_t slave_state;
static i2c_slave_state_t task_state;
static mp_machine_i2c_obj_t *i2c_slave_device = NULL;

static const char *TAG = "[I2C]";


// ===== I2C Slave functions ==================================================================

/*
  For i2c slave mode, we are using the slave buffer to send/receive data to/from master
  I2C device with 128-4096 bytes of memory is emulated.
  For buffer sizes of 128-256 bytes, 8-bit (1 byte) buffer addressing is used.
  For buffer sizes of >256 bytes, 16-bit (2 bytes) buffer addressing is used.

  Master can read from any memory location in the buffer by sending the 1 or 2 bytes address first.
    After the address is received, master can read the data in the same transaction (issuing repeated start)
    or in the new read transaction.

  Master can write to any memory location, except for the optional RO (read only) area,
    in the buffer by sending the 1 or 2 bytes address first, then continuing to write the data in the same transaction.

  Functions for reading from/writing to the slave buffer from the main application are provided.
*/

// IRQ handler; one data byte received from master
//------------------------------------------
static void i2c_slave_receive(uint32_t data)
{
    if (i2c_slave_device == NULL) return;

    slave_state.rxcount++;
    if ((slave_state.rxcount == 1) && (i2c_slave_device->slave_buflen <= 256)) {
        // === address byte received in 1-byte address mode ===
        // 8-bit addressing, set the buffer address
        slave_state.rxaddr = (uint16_t)data;
        slave_state.rxptr = 0;
        slave_state.txaddr = slave_state.rxaddr;
        slave_state.txptr = 0;
        slave_state.status |= I2C_SLAVE_STATUS_ADDR_RX;
    }
    else if ((slave_state.rxcount == 1) && (i2c_slave_device->slave_buflen > 256)) {
        // === 1st address byte received in 2-byte address mode ===
        // 2-byte address is expected, save high address byte
        slave_state.tmpaddr = (uint16_t)data << 8;
        // set the buffer addresses outside buffer for now
        slave_state.rxaddr = i2c_slave_device->slave_buflen;
    }
    else if ((i2c_slave_device->slave_buflen > 256) && (slave_state.rxcount == 2)) {
        // === 2nd address byte received, set the buffer address ===
        slave_state.rxaddr = slave_state.tmpaddr | (uint16_t)data;
        slave_state.rxptr = 0;
        slave_state.txaddr = slave_state.rxaddr;
        slave_state.txptr = 0;
        slave_state.status |= I2C_SLAVE_STATUS_ADDR_RX;
    }
    else {
        // === data byte received, save to buffer ===
        uint16_t addr = slave_state.rxaddr + slave_state.rxptr;
        if (addr < (i2c_slave_device->slave_buflen - slave_state.ro_len)) {
            i2c_slave_device->slave_buffer[addr] = (uint8_t)data;
            slave_state.rxptr++;
        }
        else {
            slave_state.rxovf++;
        }
        //slave_state.status &= ~I2C_SLAVE_STATUS_ADDR_RX;
        slave_state.status |= I2C_SLAVE_STATUS_DATA_RX;
    }
}

// IRQ handler; master requests data byte from slave
//----------------------------------
static uint32_t i2c_slave_transmit()
{
    if (i2c_slave_device == NULL) return I2C_SLAVE_DUMMY_BYTE;

    uint32_t ret = 0;
    // get data byte from slave buffer at current address
    uint16_t addr = slave_state.txaddr + slave_state.txptr;
    if (addr < i2c_slave_device->slave_buflen) {
        ret =  (uint32_t)i2c_slave_device->slave_buffer[addr];
        slave_state.txptr++;
    }
    else {
        // address outside the slave buffer, send dummy byte
        ret = I2C_SLAVE_DUMMY_BYTE;
        slave_state.txovf++;
    }
    slave_state.status |= I2C_SLAVE_STATUS_DATA_TX;

    return ret;
}

// IRQ handler; I2C stop condition, transaction completed
//--------------------------------------------
static void i2c_slave_event(i2c_event_t event)
{

    if(I2C_EV_STOP == event) {
        if (i2c_slave_device == NULL) return;
        portBASE_TYPE HPTaskAwoken = pdFALSE;
        // set busy flag if needed
        if ((i2c_slave_device->slave_busy) && (slave_state.status & I2C_SLAVE_STATUS_DATA_RX))
            i2c_slave_device->slave_buffer[i2c_slave_device->slave_buflen] |= I2C_SLAVE_FLAG_BUSY;

        if (i2c_slave_task_handle) {
            // Send notification to I2C task if running
            // copy the status data to the task's status to ensure it is not changed if the new transaction starts
            memcpy((void *)&task_state.rxaddr, (void *)&slave_state.rxaddr, sizeof(i2c_slave_state_t));
            // send the notification
            xTaskNotifyFromISR(i2c_slave_task_handle, I2C_SLAVE_STOP_EVENT, eSetValueWithOverwrite, &HPTaskAwoken);
        }
        // next rx transaction needs to set the address first
        slave_state.rxcount = 0;
        slave_state.status = 0;

        // prepare tx address & ptr for next transaction
        if (slave_state.txptr) {
            slave_state.txaddr += slave_state.txptr;
            slave_state.txptr = 0;
        }

        if (HPTaskAwoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    // ** I2C_EV_START event is received on EVERY start condition, ignore it
    //else if(I2C_EV_START == event) {
    //    return;
    //}
}

// ==== Slave buffer functions, disable the interrupt during execution ==========

//--------------------------------------------------------------------------------------
static int i2c_slave_reset_busy(mp_machine_i2c_obj_t *i2c_obj, TickType_t ticks_to_wait)
{

    if (!i2c_obj->slave_busy) return -1;

    portBASE_TYPE res = xSemaphoreTake(i2c_obj->slave_mutex, ticks_to_wait);
    if (res == pdFALSE) return 0;

    //pic_set_irq_enable(IRQN_I2C0_INTERRUPT + i2c_obj->i2c_num, 0);
    taskENTER_CRITICAL();
    i2c_obj->slave_buffer[i2c_obj->slave_buflen-1] &= 0x7F;
    //pic_set_irq_enable(IRQN_I2C0_INTERRUPT + i2c_obj->i2c_num, 1);
    taskEXIT_CRITICAL();

    xSemaphoreGive(i2c_obj->slave_mutex);
    return 1;
}

//---------------------------------------------------------------------------------------------------
static int i2c_slave_set_buffer(mp_machine_i2c_obj_t *i2c_obj, uint8_t val, TickType_t ticks_to_wait)
{
    portBASE_TYPE res = xSemaphoreTake(i2c_obj->slave_mutex, ticks_to_wait);
    if (res == pdFALSE) return 0;

    taskENTER_CRITICAL();
    memset(i2c_obj->slave_buffer, val, i2c_obj->slave_buflen);
    if (i2c_obj->slave_busy) i2c_obj->slave_buffer[i2c_obj->slave_buflen-1] = 0;
    taskEXIT_CRITICAL();

    xSemaphoreGive(i2c_obj->slave_mutex);
    return 0;
}

//---------------------------------------------------------------------------------------------------------------------------
static int i2c_slave_write_buffer(mp_machine_i2c_obj_t *i2c_obj, uint8_t* data, int addr, int size, TickType_t ticks_to_wait)
{
    if (addr > i2c_obj->slave_buflen) return 0;
    // correct the size if necessary
    if ((addr+size) >= i2c_obj->slave_buflen) size = i2c_obj->slave_buflen - addr;

    if (size > 0) {
        portBASE_TYPE res = xSemaphoreTake(i2c_obj->slave_mutex, ticks_to_wait);
        if (res == pdFALSE) return 0;

        taskENTER_CRITICAL();
        // Copy data to slave buffer
        memcpy(i2c_obj->slave_buffer + addr, data, size);
        taskEXIT_CRITICAL();

        xSemaphoreGive(i2c_obj->slave_mutex);
    }
    return size;
}

//--------------------------------------------------------------------------------------------------------------------------
static int i2c_slave_read_buffer(mp_machine_i2c_obj_t *i2c_obj, uint8_t* data, int addr, int size, TickType_t ticks_to_wait)
{
    if (addr > i2c_obj->slave_buflen) return 0;

    if (size > 0) {
        // correct the size if necessary
        if ((addr+size) >= i2c_obj->slave_buflen) size = i2c_obj->slave_buflen - addr;

        portBASE_TYPE res = xSemaphoreTake(i2c_obj->slave_mutex, ticks_to_wait);
        if (res == pdFALSE) return 0;

        taskENTER_CRITICAL();
        // Copy data from slave buffer
        memcpy(data, i2c_obj->slave_buffer + addr, size);
        taskEXIT_CRITICAL();

        xSemaphoreGive(i2c_obj->slave_mutex);
    }
    return size;
}

// ==============================================================================

//----------------------------------------------------------------------------------------
static int i2c_hard_init(uint32_t i2c_num, uint8_t sda, uint8_t scl, gpio_pin_func_t func)
{
    if (mp_used_pins[sda].func != GPIO_FUNC_NONE) {
        LOGD(TAG, "SDA %s", gpiohs_funcs_in_use[mp_used_pins[sda].func]);
        return -1;
    }
    if (mp_used_pins[scl].func != GPIO_FUNC_NONE) {
        LOGD(TAG, "SCL %s", gpiohs_funcs_in_use[mp_used_pins[scl].func]);
        return -2;
    }

    // Configure sda & scl pins
    mp_fpioa_cfg_item_t i2c_pin_func[2];
    i2c_pin_func[0] = (mp_fpioa_cfg_item_t){-1, sda, GPIO_USEDAS_SDA, FUNC_I2C0_SDA + (i2c_num *2)};
    i2c_pin_func[1] = (mp_fpioa_cfg_item_t){-1, scl, GPIO_USEDAS_SCL, FUNC_I2C0_SCLK + (i2c_num *2)};
    // Setup and mark used pins
    fpioa_setup_pins(2, i2c_pin_func);
    fpioa_setused_pins(2, i2c_pin_func, func);

    return 0;
}

//--------------------------------------------------------
static bool i2c_hard_deinit(mp_machine_i2c_obj_t *i2c_obj)
{
    // Deconfigure sda & scl pins
    mp_fpioa_cfg_item_t i2c_pin_func[2];
    i2c_pin_func[0] = (mp_fpioa_cfg_item_t){-1, i2c_obj->sda, GPIO_USEDAS_NONE, FUNC_RESV0};
    i2c_pin_func[1] = (mp_fpioa_cfg_item_t){-1, i2c_obj->scl, GPIO_USEDAS_NONE, FUNC_RESV0};
    fpioa_setup_pins(2, i2c_pin_func);
    fpioa_freeused_pins(2, i2c_pin_func);

    return true;
}

//------------------------------------------------------------
static void _mp_machine_i2c_deinit(mp_machine_i2c_obj_t *self)
{
    if ((i2c_used[self->i2c_num] == I2C_MODE_SLAVE) && (i2c_slave_task_handle)) {
        xTaskNotify(i2c_slave_task_handle, I2C_SLAVE_DRIVER_DELETED, eSetValueWithOverwrite);
        int tmo = 120;
        while ((tmo) && (i2c_slave_task_handle)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            tmo--;
        }
    }
    if (self->slave_task_mutex) {
        vSemaphoreDelete(self->slave_task_mutex);
        self->slave_task_mutex = NULL;
    }
    if (self->slave_mutex) {
        vSemaphoreDelete(self->slave_mutex);
        self->slave_mutex = NULL;
    }

    if (self->i2c_handle) {
        io_close(self->i2c_handle);
        self->i2c_handle = 0;

        i2c_hard_deinit(self);
    }
    if (self->slave_buffer) {
        vPortFree(self->slave_buffer);
        self->slave_buffer = NULL;
    }
}

//-----------------------------------
static void _checkAddr(uint16_t addr)
{
    if (addr < 256) {
        if ((addr < 0x08) || (addr > 0x77)) {
            mp_raise_ValueError("Wrong 7-bit i2c address (0x08 - 0x77 allowed)");
        }
    }
    else {
        if (addr > 0x3FF) {
            mp_raise_ValueError("Wrong 10-bit i2c address (0x100 - 0x3FF allowed)");
        }
    }
}

//-----------------------------------------------------------------
static uint8_t getMemAdrLen(int memlen, int membits, uint32_t addr)
{
    if (membits > 0) memlen = membits / 8;
    if ((memlen < 1) || (memlen > 4)) {
        mp_raise_ValueError("Memory address length error, 1 - 4 allowed");
    }
    uint8_t len = 1;
    if (addr > 0xFF) len++;
    if (addr > 0xFFFF) len++;
    if (addr > 0xFFFFFF) len++;
    if (memlen > len) len = memlen;
    return len;
}

//--------------------------------------------------
STATIC void _checkMaster(mp_machine_i2c_obj_t *self)
{
    if (self->mode != I2C_MODE_MASTER) {
        mp_raise_ValueError("I2C not in MASTER mode)");
    }
}

//-------------------------------------------------
STATIC void _checkSlave(mp_machine_i2c_obj_t *self)
{
    if (self->mode != I2C_MODE_SLAVE) {
        mp_raise_ValueError("I2C not in SLAVE mode)");
    }
}

//============================================
static void i2c_slave_task(void *pvParameters)
{
    slave_task_params_t *task_params = (slave_task_params_t *)pvParameters;
    // if the task uses some MicroPython functions, we have to save
    // MicroPython state in local storage pointers
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)task_params->thread_handle, THREAD_LSP_STATE));
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)task_params->thread_handle, THREAD_LSP_ARGS));

    mp_machine_i2c_obj_t *i2c_obj = (mp_machine_i2c_obj_t *)task_params->i2c_obj;

    int len, ovf, rdlen, addr;
    uint8_t cb_type;
    uint64_t notify_val = 0;
    int notify_res = 0;
    bool flag = false;
    uint8_t *data;

    if (i2c_obj->slave_buflen == 0) goto exit;

    LOGD(TAG, "Slave task started");
    while (1) {
        // === Wait for notification from I2C interrupt routine ===
        notify_val = 0;
        notify_res = xTaskNotifyWait(0, ULONG_MAX, &notify_val, 1000 / portTICK_RATE_MS);

        if (i2c_obj->slave_task_mutex) xSemaphoreTake(i2c_obj->slave_task_mutex, I2C_SLAVE_MUTEX_TIMEOUT);
        if (notify_res != pdPASS) {
            flag = false;
            for (int i=0; i<I2C_MAX_DEVICES; i++) {
                if (i2c_used[i] == I2C_MODE_SLAVE) flag = true;
            }
            if (!flag) {
                LOGD(TAG, "i2c slave instance deleted");
                if (i2c_obj->slave_task_mutex) xSemaphoreGive(i2c_obj->slave_task_mutex);
                break;
            }
            if (i2c_obj->slave_task_mutex) xSemaphoreGive(i2c_obj->slave_task_mutex);
            continue;
        }

        // ==== notification received ====

        // Check if task exit requested or all i2c instances are deinitialized
        if (notify_val == I2C_SLAVE_DRIVER_DELETED) {
            LOGD(TAG, "i2c device #%d deleted", i2c_obj->i2c_num);
            if (i2c_obj->slave_task_mutex) xSemaphoreGive(i2c_obj->slave_task_mutex);
            break;
        }
        flag = false;
        for (int i=0; i<I2C_MAX_DEVICES; i++) {
            if (i2c_used[i] == I2C_MODE_SLAVE) flag = true;
        }
        if (!flag) {
            LOGD(TAG, "i2c slave instance deleted");
            if (i2c_obj->slave_task_mutex) xSemaphoreGive(i2c_obj->slave_task_mutex);
            break;
        }

        if (notify_val != I2C_SLAVE_STOP_EVENT) {
            LOGD(TAG, "Unhandled event (%04X)", (uint16_t)notify_val);
            if (i2c_obj->slave_task_mutex) xSemaphoreGive(i2c_obj->slave_task_mutex);
            continue;
        }
        LOGD(TAG, "Stop condition: status=%02X; rx(%d, %d); tx(%d,%d)",
                task_state.status, task_state.rxaddr, task_state.rxptr, task_state.txaddr, task_state.txptr);

        // handle notification
        if (i2c_obj->slave_cb) {
            cb_type = 0;
            ovf = 0;
            len = 0;
            addr = 0;
            if (task_state.status & I2C_SLAVE_STATUS_ADDR_RX) {
                // only address received from master
                cb_type |= I2C_SLAVE_CBTYPE_ADDR;
                addr = task_state.rxaddr;
            }
            if (task_state.status & I2C_SLAVE_STATUS_DATA_TX) {
                // read transaction, data sent to master
                cb_type |= I2C_SLAVE_CBTYPE_DATA_TX;
                ovf = task_state.txovf;
                len = task_state.txptr;
                addr = task_state.txaddr;
            }
            else if (task_state.status & I2C_SLAVE_STATUS_DATA_RX) {
                // write transaction, data received from master
                cb_type |= I2C_SLAVE_CBTYPE_DATA_RX;
                ovf = task_state.rxovf;
                len = task_state.rxptr;
                addr = task_state.rxaddr;
            }
            cb_type &= i2c_obj->slave_cbtype; // mask allowed callback types

            if (cb_type) {
                data = NULL;
                if (len > 0) {
                   data = pvPortMalloc(len);
                   if (data) {
                       rdlen = i2c_slave_read_buffer(i2c_obj, data, addr, len, I2C_SLAVE_MUTEX_TIMEOUT);
                       if (rdlen != len) {
                           vPortFree(data);
                           data = NULL;
                       }
                   }
                }
                // schedule callback function
                mp_obj_t tuple[5];
                tuple[0] = mp_obj_new_int(cb_type);
                tuple[1] = mp_obj_new_int(addr);
                tuple[2] = mp_obj_new_int(len);
                tuple[3] = mp_obj_new_int(ovf);
                if (data) tuple[4] = mp_obj_new_bytes((const byte*)data, len);
                else tuple[4] = mp_const_none;

                mp_sched_schedule(i2c_obj->slave_cb, mp_obj_new_tuple(5, tuple));
                if (data) vPortFree(data);
            }
        }
        if (i2c_obj->slave_task_mutex) xSemaphoreGive(i2c_obj->slave_task_mutex);
    }

exit:
    LOGD(TAG, "Slave task finished");
    i2c_slave_task_handle = NULL;
    vTaskDelete(NULL);
}


// ============================================================================================
// === I2C MicroPython bindings ===============================================================
// ============================================================================================

enum { ARG_id, ARG_mode, ARG_speed, ARG_freq, ARG_sda, ARG_scl, ARG_slaveaddr, ARG_slavebuflen, ARG_rolen, ARG_busy };

// Arguments for new object and init method
//----------------------------------------------------------
STATIC const mp_arg_t mp_machine_i2c_init_allowed_args[] = {
        { MP_QSTR_id,                                MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_mode,                              MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_speed,            MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_freq,             MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_sda,              MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_scl,              MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_slave_addr,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_slave_bufflen,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_slave_rolen,      MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_slave_busy,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
};

//------------------------------------------------------
static void _init_i2c_object(mp_machine_i2c_obj_t *self)
{
    char i2cdev[16];
    if (self->mode == I2C_MODE_MASTER) {
        // Setup I2C master
        sprintf(i2cdev, "/dev/i2c%d", self->i2c_num);
        self->i2c_handle = io_open(i2cdev);
        if (self->i2c_handle == 0) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error opening I2C device"));
        }
        int res = i2c_hard_init(self->i2c_num, self->sda, self->scl, GPIO_FUNC_I2C);
        if (res < 0) {
            mp_raise_ValueError("Error initializing I2C hardware");
        }
    }
    else {
        if (self->slave_task_mutex == NULL) self->slave_task_mutex = xSemaphoreCreateMutex();
        if (self->slave_mutex == NULL) self->slave_mutex = xSemaphoreCreateMutex();

        self->slave_buffer = pvPortMalloc(self->slave_buflen);
        if (self->slave_buffer == NULL) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating I2C slave buffer"));
        }
        memset(self->slave_buffer, 0, self->slave_buflen);

        if ((self->slave_busy) && (self->slave_rolen == 0)) self->slave_rolen = 1;

        // Setup I2C slave
        memset(&slave_state, 0, sizeof(i2c_slave_state_t));
        memset(&task_state, 0, sizeof(i2c_slave_state_t));

        sprintf(i2cdev, "/dev/i2c%d", self->i2c_num);
        self->i2c_handle = io_open(i2cdev);
        if (self->i2c_handle == 0) {
            vPortFree(self->slave_buffer);
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error opening I2C device"));
        }
        int res = i2c_hard_init(self->i2c_num, self->sda, self->scl, GPIO_FUNC_I2C);
        if (res < 0) {
            vPortFree(self->slave_buffer);
            mp_raise_ValueError("Error initializing I2C hardware");
        }

        self->slave_handler.on_event = i2c_slave_event;
        self->slave_handler.on_receive = i2c_slave_receive;
        self->slave_handler.on_transmit = i2c_slave_transmit,
        i2c_config_as_slave(self->i2c_handle, self->slave_addr, 7, &self->slave_handler);

        //Create a task to handle I2c slave events from I2C ISR
        if (i2c_slave_task_handle == NULL) {
            task_params.i2c_obj = (void *)self;
            task_params.thread_handle = xTaskGetCurrentTaskHandle();

            BaseType_t res = xTaskCreate(
                i2c_slave_task,             // function entry
                "i2c_slave_task",           // task name
                I2C_SLAVE_TASK_STACK_SIZE,  // stack_deepth
                (void *)&task_params,       // function argument
                MICROPY_TASK_PRIORITY+1,    // task priority
                &i2c_slave_task_handle);    // task handle
            if (res != pdPASS) {
                LOGE("UART", "Event task not started");
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (i2c_slave_task_handle == NULL) {
                vPortFree(self->slave_buffer);
                nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C slave task not started"));
            }
        }
    }
}

//-----------------------------------------------------------------------------------------------
STATIC void mp_machine_i2c_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    mp_machine_i2c_obj_t *self = self_in;
    if (i2c_used[self->i2c_num] > 0) {
        if (self->mode == I2C_MODE_MASTER)
            mp_printf(print, "I2C (Device=%u, Mode=MASTER, Speed=%u Hz, sda=%d, scl=%d)", self->i2c_num, self->speed, self->sda, self->scl);
        else {
            mp_printf(print, "I2C (Device=%u, Mode=SLAVE, Speed=%u Hz, sda=%d, scl=%d, addr=%d (0x%02X), buffer=%d B, read-only=%d B)",
                    self->i2c_num, self->speed, self->sda, self->scl, self->slave_addr, self->slave_addr, self->slave_buflen, self->slave_rolen);
            mp_printf(print, "\n     Callback=%s (%d)", self->slave_cb ? "True" : "False", self->slave_cbtype);
            if (i2c_slave_task_handle) {
                mp_printf(print, "\n     I2C task minimum free stack: %u", uxTaskGetStackHighWaterMark(i2c_slave_task_handle));
            }
        }
    }
    else {
        mp_printf(print, "I2C (Deinitialized)");
    }
}

//----------------------------------------------------------------------------------------------------------------------
static mp_obj_t mp_machine_i2c_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    mp_arg_val_t args[MP_ARRAY_SIZE(mp_machine_i2c_init_allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(mp_machine_i2c_init_allowed_args), mp_machine_i2c_init_allowed_args, args);

    int8_t sda;
    int8_t scl;
    int32_t speed;
    int i2c_num = args[ARG_id].u_int;
    if ((i2c_num < 0) || (i2c_num >= I2C_MAX_DEVICES)) i2c_num = 0; // set default

    // Check speed
    if (args[ARG_freq].u_int > 0) speed = args[ARG_freq].u_int;
    else speed = args[ARG_speed].u_int;
    if (speed < 0) speed = 100000; // set default
    if ((speed < 50000) || (speed > 5000000)) {
        mp_raise_ValueError("I2C speed out of range (50000 ~ 5000000)");
    }

    // Check the peripheral id
    if (i2c_num < 0 || i2c_num >= I2C_MAX_DEVICES) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C device not available"));
    }
    if (i2c_used[i2c_num] > 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C device already used"));
    }
    // Check mode
    int mode = args[ARG_mode].u_int;
    if (mode < 1) mode = I2C_MODE_MASTER; // set default
    if ((mode != I2C_MODE_MASTER) && (mode != I2C_MODE_SLAVE)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "MASTER or SLAVE mode must be selected"));
    }

    // Check SDA & SCL
    if ((args[ARG_sda].u_int < 0) || (args[ARG_sda].u_int >= FPIOA_NUM_IO) ||
        (args[ARG_scl].u_int < 0) || (args[ARG_scl].u_int >= FPIOA_NUM_IO)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "sda & scl must be given"));
    }
    sda = args[ARG_sda].u_int;
    scl = args[ARG_scl].u_int;

    // Create I2C object
    mp_machine_i2c_obj_t *self = m_new_obj(mp_machine_i2c_obj_t );
    self->base.type = &machine_hw_i2c_type;
    self->mode = mode;
    self->i2c_num = i2c_num;
    self->speed = speed;
    self->scl = scl;
    self->sda = sda;
    self->slave_buflen = 0;
    self->slave_rolen = 0;
    self->slave_addr = 0;
    self->slave_busy = false;
    self->slave_cb = NULL;
    self->slave_cbtype = I2C_SLAVE_CBTYPE_NONE;
    self->slave_buffer = NULL;
    self->slave_mutex = NULL;
    self->slave_task_mutex = NULL;

    if (mode == I2C_MODE_SLAVE) {
        for (int i=0; i<I2C_MAX_DEVICES; i++) {
            if (i2c_used[i] == I2C_MODE_SLAVE) {
                nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Only one I2C slave device can be used"));
            }
        }

        if (args[ARG_busy].u_int == 1) self->slave_busy = true;
        // Set I2C slave address
        if ((args[ARG_slaveaddr].u_int > 0) && (args[ARG_slaveaddr].u_int < 256)) {
            _checkAddr(args[ARG_slaveaddr].u_int);
            self->slave_addr = args[ARG_slaveaddr].u_int;
        }
        else self->slave_addr = I2C_SLAVE_ADDR_DEFAULT;
        // Set I2C slave buffers
        if ((args[ARG_slavebuflen].u_int >= I2C_SLAVE_MIN_BUFFER_LENGTH) && (args[ARG_slavebuflen].u_int <= I2C_SLAVE_MAX_BUFFER_LENGTH))
            self->slave_buflen = args[ARG_slavebuflen].u_int;
        else self->slave_buflen = I2C_SLAVE_DEFAULT_BUFF_LEN;

        if ((args[ARG_rolen].u_int > 0) && (args[ARG_rolen].u_int < (self->slave_buflen / 2)))
            self->slave_rolen = args[ARG_rolen].u_int;
        else self->slave_rolen = 0;
    }

    _init_i2c_object(self);

    i2c_used[i2c_num] = mode;

    if (mode == I2C_MODE_SLAVE) {
        i2c_slave_set_clock_rate(self->i2c_handle, self->speed);
        i2c_slave_device = self;
    }
    else i2c_slave_device = NULL;

    return MP_OBJ_FROM_PTR(self);
}

//-----------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_deinit(mp_obj_t self_in)
{
    mp_machine_i2c_obj_t *self = self_in;

    _mp_machine_i2c_deinit(self);

    i2c_used[self->i2c_num] = 0;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_i2c_deinit_obj, mp_machine_i2c_deinit);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    mp_machine_i2c_obj_t *self = pos_args[0];

    mp_arg_val_t args[MP_ARRAY_SIZE(mp_machine_i2c_init_allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(mp_machine_i2c_init_allowed_args), mp_machine_i2c_init_allowed_args, args);

    int8_t sda, scl;
    int32_t speed;
    uint8_t changed = 0;

    uint32_t *slave_cb = self->slave_cb;
    uint8_t slave_cbtype = self->slave_cbtype;
    int8_t slave_addr;
    int buff_len, ro_len;
    bool slave_busy;

    // Check for new config values
    // i2c_num, speed, sda, scl, mode

    // i2c device
    int8_t old_i2c_num = self->i2c_num;
    int8_t i2c_num = args[ARG_id].u_int;
    if (i2c_num < 0) i2c_num = self->i2c_num;

    // Check the peripheral id
    if (i2c_num < 0 || i2c_num >= I2C_MAX_DEVICES) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C device not available"));
    }
    // Check mode
    int mode = args[ARG_mode].u_int;
    if ((mode >= 0) && (mode != self->mode)) {
        if ((mode != I2C_MODE_MASTER) && (mode != I2C_MODE_SLAVE)) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "MASTER or SLAVE mode must be selected"));
        }
        changed++;
    }
    else mode = self->mode;

    // speed
    if (args[ARG_freq].u_int > 0) speed = args[ARG_freq].u_int;
    else speed = args[ARG_speed].u_int;
    if (speed > 0) {
        if ((speed < 50000) || (speed > 5000000)) {
            mp_raise_ValueError("I2C speed out of range (50000 ~ 5000000)");
        }
        if (self->speed != speed) {
            self->speed = speed;
            if (mode == I2C_MODE_SLAVE) {
                i2c_slave_set_clock_rate(self->i2c_handle, self->speed);
            }
        }
    }

    scl = self->scl;
    sda = self->sda;
    buff_len = self->slave_buflen;
    ro_len = self->slave_rolen;
    slave_addr = self->slave_addr;
    slave_busy = self->slave_busy;

    if (args[ARG_sda].u_int >= 0) sda = args[ARG_sda].u_int;
    if (args[ARG_scl].u_int >= 0) scl = args[ARG_scl].u_int;

    // Check if the configuration changed
    if (old_i2c_num != i2c_num) changed++;
    if (self->scl != scl) changed++;
    if (self->sda != sda) changed++;

    if (mode == I2C_MODE_SLAVE) {
        if (args[ARG_busy].u_int >= 0) {
            if (self->slave_busy != ((args[ARG_busy].u_int == 1) ? true : false)) {
                slave_busy = (args[ARG_busy].u_int == 1) ? true : false;
                changed++;
            }
        }
        if ((args[ARG_slaveaddr].u_int > 0) && (args[ARG_slaveaddr].u_int < 256)) {
            _checkAddr(args[ARG_slaveaddr].u_int);
            if (args[ARG_slaveaddr].u_int != slave_addr) {
                slave_addr = args[ARG_slaveaddr].u_int;
                changed++;
            }
        }
        if ((args[ARG_slavebuflen].u_int >= I2C_SLAVE_MIN_BUFFER_LENGTH) && (args[ARG_slavebuflen].u_int <= I2C_SLAVE_MAX_BUFFER_LENGTH)) {
            if (args[ARG_slavebuflen].u_int != buff_len) {
                buff_len = args[ARG_slavebuflen].u_int;
                changed++;
            }
        }
        if ((args[ARG_rolen].u_int > 0) && (args[ARG_rolen].u_int < (buff_len/2))) {
            if (args[ARG_rolen].u_int != ro_len) {
                ro_len = args[ARG_rolen].u_int;
                if ((self->slave_busy) && (self->slave_rolen == 0)) self->slave_rolen = 1;
                changed++;
            }
        }
    }

    if (changed) {
        if (i2c_used[old_i2c_num] > 0) {
            // Deinit old driver, if it was a slave, the task will be stopped
            _mp_machine_i2c_deinit(self);
            i2c_used[old_i2c_num] = 0;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if (i2c_used[i2c_num] > 0) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C device already used"));
        }

        self->i2c_num = i2c_num;
        self->scl = scl;
        self->sda = sda;
        self->mode = mode;

        self->slave_addr = slave_addr;
        self->slave_buflen = buff_len;
        self->slave_rolen = ro_len;
        self->slave_busy = slave_busy;

        self->slave_cb = slave_cb;
        self->slave_cbtype = slave_cbtype;

        _init_i2c_object(self);

        if (mode == I2C_MODE_SLAVE) i2c_slave_device = self;
        else i2c_slave_device = NULL;

        i2c_used[i2c_num] = mode;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_i2c_init_obj, 1, mp_machine_i2c_init);


// ============================================================================================
// ==== I2C master functions ==================================================================
// ============================================================================================

//-------------------------------------------------------------------------------------------------------------------------------------------------
STATIC int mp_i2c_master_write(mp_machine_i2c_obj_t *i2c_obj, uint16_t slave_addr, uint8_t memwrite, uint32_t memaddr, uint8_t *data, uint16_t len)
{
    int ret = -1;
    uint8_t *pdata = data;
    uint8_t *buff = NULL;
    uint8_t mem_buf[4] = { 0 };
    size_t size = len;

    if (memwrite) {
        // send memory address, MSByte first
        for (int i=0; i<memwrite; i++) {
            mem_buf[i] = (memaddr >> (i*8)) & 0xFF;
        }
    }
    if ((memwrite > 0) && (data) && (len > 0)) {
        buff = pvPortMalloc(len+memwrite);
        memcpy(buff, mem_buf, memwrite);
        memcpy(buff+memwrite, data, len);
        pdata = buff;
        size = len + memwrite;
    }
    else if ((memwrite == 0) && (data) && (len > 0)) {
        pdata = data;
        size = len;
    }
    else if ((memwrite > 0) && ((data == NULL) || (len == 0))) {
        pdata = mem_buf;
        size = memwrite;
    }
    else return 0;

    handle_t i2c_dev = i2c_get_device(i2c_obj->i2c_handle, slave_addr, (slave_addr < 0x80) ? 7 : 10);
    i2c_dev_set_clock_rate(i2c_dev, i2c_obj->speed);

    ret = io_write(i2c_dev, pdata, size);

    if (buff) vPortFree(buff);

    io_close(i2c_dev);
    if ((ret > 0) && (memwrite > 0)) ret -= memwrite;

    return ret;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
STATIC int mp_i2c_master_read(mp_machine_i2c_obj_t *i2c_obj, uint16_t slave_addr, uint8_t memread, uint32_t memaddr, uint8_t *data, uint16_t len)
{
    int ret = -1;

    memset(data, 0xFF, len);
    handle_t i2c_dev = i2c_get_device(i2c_obj->i2c_handle, slave_addr, (slave_addr < 0x80) ? 7 : 10);
    i2c_dev_set_clock_rate(i2c_dev, i2c_obj->speed);

    if (memread) {
        // send memory address, MSByte first
        uint8_t mem_buf[4] = { 0 };
        for (int i=0; i<memread; i++) {
            mem_buf[i] = (memaddr >> (i*8)) & 0xFF;
        }
        ret = i2c_dev_transfer_sequential(i2c_dev, mem_buf, memread, data, len);
    }
    else {
        ret = io_read(i2c_dev, data, len);
    }

    io_close(i2c_dev);
    return ret;
}

// ==== Master read & write functions =========================================================

//----------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_readfrom(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    STATIC const mp_arg_t machine_i2c_readfrom_args[] = {
        { MP_QSTR_addr,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_nbytes,  MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = 0} },
    };

    mp_machine_i2c_obj_t *self = pos_args[0];
    _checkMaster(self);

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(machine_i2c_readfrom_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), machine_i2c_readfrom_args, args);

    _checkAddr(args[0].u_int);

    if (args[1].u_int > 0) {
        uint8_t *buf = pvPortMalloc(args[1].u_int);
        if (buf == NULL) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating I2C data buffer"));
        }
        int ret = mp_i2c_master_read(self, args[0].u_int, false, 0, buf, args[1].u_int);
        if (ret <= 0) {
            vPortFree(buf);
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C read error"));
        }
        vstr_t vstr;
        vstr_init_len(&vstr, args[1].u_int);
        memcpy(vstr.buf, buf, args[1].u_int);
        vPortFree(buf);
        // Return read data as string
        return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
    }
    return mp_const_empty_bytes;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_i2c_readfrom_obj, 1, mp_machine_i2c_readfrom);

//---------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_readfrom_into(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    mp_machine_i2c_obj_t *self = pos_args[0];
    _checkMaster(self);

    STATIC const mp_arg_t machine_i2c_readfrom_into_args[] = {
        { MP_QSTR_addr,    MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = 0} },
        { MP_QSTR_buf,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(machine_i2c_readfrom_into_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), machine_i2c_readfrom_into_args, args);

    _checkAddr(args[0].u_int);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1].u_obj, &bufinfo, MP_BUFFER_WRITE);

    int ret = 0;
    if (bufinfo.len > 0) {
        ret = mp_i2c_master_read(self, args[0].u_int, false, 0, bufinfo.buf, bufinfo.len);
        if (ret <= 0) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C read error"));
        }
    }
    // Return read length
    return mp_obj_new_int(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_i2c_readfrom_into_obj, 1, mp_machine_i2c_readfrom_into);

//---------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_writeto(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    mp_machine_i2c_obj_t *self = pos_args[0];
    _checkMaster(self);

    STATIC const mp_arg_t machine_i2c_writeto_args[] = {
        { MP_QSTR_addr,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_buf,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(machine_i2c_writeto_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(args), machine_i2c_writeto_args, args);

    _checkAddr(args[0].u_int);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1].u_obj, &bufinfo, MP_BUFFER_READ);

    int ret = 0;
    if (bufinfo.len > 0) {
        ret = mp_i2c_master_write(self, args[0].u_int, 0, 0, bufinfo.buf, bufinfo.len);
        if (ret <= 0) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C write error"));
        }
    }

    return mp_obj_new_int(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_i2c_writeto_obj, 1, mp_machine_i2c_writeto);


// ==== Master read & write memory functions ==================================================

// Arguments for memory read/write methods
//---------------------------------------------------------
STATIC const mp_arg_t mp_machine_i2c_mem_allowed_args[] = {
    { MP_QSTR_addr,         MP_ARG_REQUIRED | MP_ARG_INT,  {.u_int = 0} },
    { MP_QSTR_memaddr,      MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
    { MP_QSTR_arg,                            MP_ARG_OBJ,  {.u_obj = MP_OBJ_NULL} },
    { MP_QSTR_adrlen,       MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 1} },
    { MP_QSTR_addr_size,    MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 0} },
};

//-----------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_readfrom_mem(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_addr, ARG_memaddr, ARG_n, ARG_memlen, ARG_membits };
    mp_machine_i2c_obj_t *self = pos_args[0];
    _checkMaster(self);

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(mp_machine_i2c_mem_allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(mp_machine_i2c_mem_allowed_args), mp_machine_i2c_mem_allowed_args, args);

    _checkAddr(args[ARG_addr].u_int);

    // Get read length
    int n = mp_obj_get_int(args[ARG_n].u_obj);
    if (n > 0) {
        uint32_t addr = (uint32_t)mp_obj_get_int(args[ARG_memaddr].u_obj);
        uint8_t memlen = getMemAdrLen(args[ARG_memlen].u_int, args[ARG_membits].u_int, addr);

        uint8_t *buf = pvPortMalloc(n);
        if (buf == NULL) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating I2C data buffer"));
        }
        int ret = mp_i2c_master_read(self, args[ARG_addr].u_int, memlen, addr, buf, n);
        if (ret <= 0) {
            vPortFree(buf);
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C read error"));
        }
        vstr_t vstr;
        vstr_init_len(&vstr, n);
        memcpy(vstr.buf, buf, n);
        vPortFree(buf);
        // Return read data as string
        return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
    }
    // Return empty string
    return mp_const_empty_bytes;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_i2c_readfrom_mem_obj, 1, mp_machine_i2c_readfrom_mem);

//----------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_readfrom_mem_into(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_addr, ARG_memaddr, ARG_buf, ARG_memlen, ARG_membits };
    mp_machine_i2c_obj_t *self = pos_args[0];
    _checkMaster(self);

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(mp_machine_i2c_mem_allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(mp_machine_i2c_mem_allowed_args), mp_machine_i2c_mem_allowed_args, args);

    _checkAddr(args[ARG_addr].u_int);

    uint32_t addr = (uint32_t)mp_obj_get_int(args[ARG_memaddr].u_obj);
    uint8_t memlen = getMemAdrLen(args[ARG_memlen].u_int, args[ARG_membits].u_int, addr);

    // Get the output data buffer
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_WRITE);

    int ret = 0;
    if (bufinfo.len > 0) {
        // Transfer data into buffer
        ret = mp_i2c_master_read(self, args[ARG_addr].u_int, memlen, addr, bufinfo.buf, bufinfo.len);
        if (ret <= 0) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C read error"));
        }
    }
    return mp_obj_new_int(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_i2c_readfrom_mem_into_obj, 1, mp_machine_i2c_readfrom_mem_into);

//----------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_writeto_mem(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_addr, ARG_memaddr, ARG_buf, ARG_memlen, ARG_membits };
    mp_machine_i2c_obj_t *self = pos_args[0];
    _checkMaster(self);

    // parse arguments
    mp_arg_val_t args[MP_ARRAY_SIZE(mp_machine_i2c_mem_allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(mp_machine_i2c_mem_allowed_args), mp_machine_i2c_mem_allowed_args, args);

    _checkAddr(args[ARG_addr].u_int);

    uint32_t addr = (uint32_t)mp_obj_get_int(args[ARG_memaddr].u_obj);
    uint8_t memlen = getMemAdrLen(args[ARG_memlen].u_int, args[ARG_membits].u_int, addr);
    uint16_t len = 0;
    uint8_t *data = NULL;

    if (args[ARG_buf].u_obj != MP_OBJ_NULL) {
        // Get the input data buffer
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_READ);
        len = bufinfo.len;
        data = (uint8_t *)bufinfo.buf;
    }

    // Transfer address and, if given, the data
    int ret = mp_i2c_master_write(self, args[ARG_addr].u_int, memlen, addr, data, len);
    if (ret <= 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "I2C write error"));
    }
    if (len == 0) ret = 0;

    return mp_obj_new_int(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_i2c_writeto_mem_obj, 1, mp_machine_i2c_writeto_mem);

//---------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_scan(mp_obj_t self_in)
{
    mp_machine_i2c_obj_t *self = self_in;
    _checkMaster(self);

    mp_obj_t list = mp_obj_new_list(0, NULL);

    // don't include in scan the reserved 7-bit addresses: 0x00-0x07 & 0x78-0x7F
    uint8_t dummy = 0;
    int ret;
    for (int addr = 0x08; addr < 0x78; ++addr) {
        ret = mp_i2c_master_read(self, addr, 0, 0, &dummy, 1);
        if ((ret >= 0) || (ret < -4)) {
            mp_obj_list_append(list, MP_OBJ_NEW_SMALL_INT(addr));
        }
    }
    return list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_i2c_scan_obj, mp_machine_i2c_scan);

//-------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_is_ready(mp_obj_t self_in, mp_obj_t addr_in)
{
    mp_machine_i2c_obj_t *self = self_in;
    _checkMaster(self);

    int addr = mp_obj_get_int(addr_in);
    _checkAddr(addr);

    uint8_t dummy = 0;
    int ret = mp_i2c_master_read(self, addr, 0, 0, &dummy, 1);
    return ((ret >= 0) || (ret < -4)) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_i2c_is_ready_obj, mp_machine_i2c_is_ready);


// ============================================================================================
// ==== I2C slave functions ===================================================================
// ============================================================================================

//------------------------------------------------------------------------
static void _check_addr_len(mp_machine_i2c_obj_t *self, int addr, int len)
{
    if ((len < 1) || (len > self->slave_buflen)) {
        mp_raise_ValueError("Length out of range");
    }
    if (addr >= self->slave_buflen) {
        mp_raise_ValueError("Address not in slave data buffer");
    }
    if ((addr + len) > self->slave_buflen) {
        mp_raise_ValueError("Data outside buffer");
    }
}

//---------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_slave_reset_busy(mp_obj_t self_in)
{
    mp_machine_i2c_obj_t *self = self_in;
    _checkSlave(self);

    if (self->slave_task_mutex) xSemaphoreTake(self->slave_task_mutex, I2C_SLAVE_MUTEX_TIMEOUT);

    int res = i2c_slave_reset_busy(self, I2C_SLAVE_MUTEX_TIMEOUT);

    if (self->slave_task_mutex) xSemaphoreGive(self->slave_task_mutex);

    if (res <= 0) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_i2c_slave_reset_busy_obj, mp_machine_i2c_slave_reset_busy);

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_slave_setdata(mp_obj_t self_in, mp_obj_t buf_in, mp_obj_t addr_in)
{
    mp_machine_i2c_obj_t *self = self_in;
    _checkSlave(self);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    int addr = mp_obj_get_int(addr_in);
    _check_addr_len(self, addr, bufinfo.len);

    if (self->slave_task_mutex) xSemaphoreTake(self->slave_task_mutex, I2C_SLAVE_MUTEX_TIMEOUT);

    int res = i2c_slave_write_buffer(self, bufinfo.buf, addr, bufinfo.len, I2C_SLAVE_MUTEX_TIMEOUT);

    if (self->slave_task_mutex) xSemaphoreGive(self->slave_task_mutex);

    if (res <= 0) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_i2c_slave_setdata_obj, mp_machine_i2c_slave_setdata);

//--------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_slave_setbuffer(mp_obj_t self_in, mp_obj_t byte_in)
{
    mp_machine_i2c_obj_t *self = self_in;
    _checkSlave(self);

    uint8_t fill_byte = (uint8_t)mp_obj_get_int(byte_in);

    if (self->slave_task_mutex) xSemaphoreTake(self->slave_task_mutex, I2C_SLAVE_MUTEX_TIMEOUT);

    int res = i2c_slave_set_buffer(self, fill_byte, I2C_SLAVE_MUTEX_TIMEOUT);

    if (self->slave_task_mutex) xSemaphoreGive(self->slave_task_mutex);

    if (res <= 0) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_i2c_slave_setbuffer_obj, mp_machine_i2c_slave_setbuffer);

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_slave_getdata(mp_obj_t self_in, mp_obj_t addr_in, mp_obj_t len_in)
{
    mp_machine_i2c_obj_t *self = self_in;
    _checkSlave(self);

    int addr = mp_obj_get_int(addr_in);
    int len = mp_obj_get_int(len_in);
    _check_addr_len(self, addr, len);

    uint8_t *databuf = pvPortMalloc(len);
    if (databuf == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating data buffer"));
    }

    mp_obj_t data;
    if (self->slave_task_mutex) xSemaphoreTake(self->slave_task_mutex, I2C_SLAVE_MUTEX_TIMEOUT);

    int res = i2c_slave_read_buffer(self, databuf, addr, len, I2C_SLAVE_MUTEX_TIMEOUT);
    if (res > 0) data = mp_obj_new_bytes(databuf, res);
    else data = mp_const_empty_bytes;

    if (self->slave_task_mutex) xSemaphoreGive(self->slave_task_mutex);

    vPortFree(databuf);
    // Return read data as byte array
    return data;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_i2c_slave_getdata_obj, mp_machine_i2c_slave_getdata);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_machine_i2c_slave_callback(mp_obj_t self_in, mp_obj_t func, mp_obj_t type_in)
{
    mp_machine_i2c_obj_t *self = self_in;
    _checkSlave(self);

    int type = mp_obj_get_int(type_in);

    if ((!mp_obj_is_fun(func)) && (!mp_obj_is_meth(func)) && (func != mp_const_none)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Function argument required"));
    }
    if ((type < 0) || (type > 7)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Invalid callback type"));
    }

    if (self->slave_task_mutex) xSemaphoreTake(self->slave_task_mutex, I2C_SLAVE_MUTEX_TIMEOUT);

    if (func == mp_const_none) self->slave_cb = NULL;
    else self->slave_cb = func;
    self->slave_cbtype = type;

    if (self->slave_task_mutex) xSemaphoreGive(self->slave_task_mutex);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_i2c_slave_callback_obj, mp_machine_i2c_slave_callback);



//===================================================================
STATIC const mp_rom_map_elem_t mp_machine_i2c_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init),                (mp_obj_t)&mp_machine_i2c_init_obj },
    { MP_ROM_QSTR(MP_QSTR_deinit),              (mp_obj_t)&mp_machine_i2c_deinit_obj },

    // Standard methods
    { MP_ROM_QSTR(MP_QSTR_readfrom),            (mp_obj_t)&mp_machine_i2c_readfrom_obj },
    { MP_ROM_QSTR(MP_QSTR_readfrom_into),       (mp_obj_t)&mp_machine_i2c_readfrom_into_obj },
    { MP_ROM_QSTR(MP_QSTR_writeto),             (mp_obj_t)&mp_machine_i2c_writeto_obj },

    // Memory methods
    { MP_ROM_QSTR(MP_QSTR_readfrom_mem),        (mp_obj_t)&mp_machine_i2c_readfrom_mem_obj },
    { MP_ROM_QSTR(MP_QSTR_readfrom_mem_into),   (mp_obj_t)&mp_machine_i2c_readfrom_mem_into_obj },
    { MP_ROM_QSTR(MP_QSTR_writeto_mem),         (mp_obj_t)&mp_machine_i2c_writeto_mem_obj },
    { MP_ROM_QSTR(MP_QSTR_scan),                (mp_obj_t)&mp_machine_i2c_scan_obj },
    { MP_ROM_QSTR(MP_QSTR_is_ready),            (mp_obj_t)&mp_machine_i2c_is_ready_obj },

    // Standard slave methods
    { MP_ROM_QSTR(MP_QSTR_setdata),             (mp_obj_t)&mp_machine_i2c_slave_setdata_obj },
    { MP_ROM_QSTR(MP_QSTR_fillbuffer),          (mp_obj_t)&mp_machine_i2c_slave_setbuffer_obj },
    { MP_ROM_QSTR(MP_QSTR_getdata),             (mp_obj_t)&mp_machine_i2c_slave_getdata_obj },
    { MP_ROM_QSTR(MP_QSTR_resetbusy),           (mp_obj_t)&mp_machine_i2c_slave_reset_busy_obj },
    { MP_ROM_QSTR(MP_QSTR_callback),            (mp_obj_t)&mp_machine_i2c_slave_callback_obj },

    // Constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_MASTER),          MP_OBJ_NEW_SMALL_INT(I2C_MODE_MASTER) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SLAVE),           MP_OBJ_NEW_SMALL_INT(I2C_MODE_SLAVE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CBTYPE_NONE),     MP_OBJ_NEW_SMALL_INT(I2C_SLAVE_CBTYPE_NONE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CBTYPE_ADDR),     MP_OBJ_NEW_SMALL_INT(I2C_SLAVE_CBTYPE_ADDR) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CBTYPE_RXDATA),   MP_OBJ_NEW_SMALL_INT(I2C_SLAVE_CBTYPE_DATA_RX) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CBTYPE_TXDATA),   MP_OBJ_NEW_SMALL_INT(I2C_SLAVE_CBTYPE_DATA_TX) },
};

STATIC MP_DEFINE_CONST_DICT(mp_machine_i2c_locals_dict, mp_machine_i2c_locals_dict_table);

//=========================================
const mp_obj_type_t machine_hw_i2c_type = {
    { &mp_type_type },
    .name = MP_QSTR_I2C,
    .print = mp_machine_i2c_print,
    .make_new = mp_machine_i2c_make_new,
    .locals_dict = (mp_obj_dict_t*)&mp_machine_i2c_locals_dict,
};
