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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "syslog.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "devices.h"
#include "uart.h"
#include "hal.h"
#include "syslog.h"

#include "machine_uart.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "mphalport.h"
#include "mpconfigport.h"

#define UART_BRATE_CONST        16
#define UART_MUTEX_TIMEOUT      (100 / portTICK_PERIOD_MS)

extern pic_irq_handler_t extern_uart_irq_handler;
extern void *extern_uart_irq_userdata;

typedef struct _task_params_t {
    void *uart_obj;
    void *thread_handle;
} task_params_t;

static task_params_t task_params;

static const char *_parity_name[] = {"None", "Odd", "Even"};
static const char *_stopbits_name[] = {"1", "1.5", "2"};

volatile uart_t* const  uart[3] =
{
    (volatile uart_t*)UART1_BASE_ADDR,
    (volatile uart_t*)UART2_BASE_ADDR,
    (volatile uart_t*)UART3_BASE_ADDR
};

static const uint32_t uart_instances[3] = { 0, 1, 2 };

static const char *TAG = "[UART]";

uart_uarts_t mpy_uarts[UART_NUM_MAX] = { 0 };


// ==== UART Ring Buffer functions =========================================

//===========================================
// Interrupt handler for UART
// pushes received byte(s) to the uart buffer
//===========================================
static void uart_on_irq_recv(void *userdata)
{
    uint32_t *nuart = (uint32_t *)userdata;
    mpy_uarts[*nuart].irq_flag = true;
    uart_ringbuf_t *r = mpy_uarts[*nuart].uart_buf;
    uint8_t c;

    while (uart[*nuart]->LSR & 1) {
        c = (uint8_t)(uart[*nuart]->RBR & 0xff);
        if (r->buf) {
            if (r->length < r->size) {
                r->buf[r->tail] = c;
                r->tail = (r->tail + 1) % r->size;
                r->length++;
            }
            else r->overflow++;
        }
        else r->overflow++;
    }
    mpy_uarts[*nuart].irq_flag = false;
    if ((mpy_uarts[*nuart].task_semaphore) && (r->notify)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(mpy_uarts[*nuart].task_semaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }
}

// Put data into buffer
// Operation on uart's ringbuffer is not allowed
//-----------------------------------------------------------
int uart_buf_put(uart_ringbuf_t *r, uint8_t *src, size_t len)
{
    if (r->uart_num < UART_NUM_MAX) return 0;
    if (r->buf == NULL) {
        r->overflow += len;
        return 0;
    }
    size_t cnt = 0;

    for (int i=0; i<len; i++) {
        if (r->length < r->size) {
            r->buf[r->tail] = src[i];
            r->tail = (r->tail + 1) % r->size;
            r->length++;
            cnt++;
        }
        else r->overflow++;
    }
    return cnt;
}

// Remove data from buffer end
// Operation is not allowed on uart's ringbuffer
//---------------------------------------------------------
int uart_buf_remove_from_end(uart_ringbuf_t *r, size_t len)
{
    if (r->uart_num < UART_NUM_MAX) return 0;
    if (r->buf == NULL) return 0;
    size_t cnt = 0;
    size_t length = r->length;

    if (length == 0) return 0;

    for (int i=len; i>=0; i--) {
        if (r->length == 0) break;
        r->tail--;
        r->length--;
        cnt++;
    }
    return cnt;
}

// Get current buffer length
//-----------------------------------------------------
size_t uart_buf_length(uart_ringbuf_t *r, size_t *size)
{
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    size_t length = r->length;
    if (size) *size = r->size;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    return length;
}

// Get and remove data from buffer
//------------------------------------------------------------
int uart_buf_get(uart_ringbuf_t *r, uint8_t *dest, size_t len)
{
    if (r->buf == NULL) return 0;

    size_t cnt = 0;
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    size_t length = r->length;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length == 0) return 0;

    while (length) {
        if (dest) dest[cnt] = r->buf[r->head];
        r->head = (r->head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    r->length -= cnt;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    return cnt;
}

// Remove data from buffer
//------------------------------------------------
int uart_buf_remove(uart_ringbuf_t *r, size_t len)
{
    size_t cnt = 0;
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    size_t length = r->length;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length == 0) return 0;

    while (length) {
        r->head = (r->head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    r->length -= cnt;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    return cnt;
}

// Set the data in uart buffer to blank
//-----------------------------------------------------------
int uart_buf_blank(uart_ringbuf_t *r, size_t pos, size_t len)
{
    size_t cnt = 0;
    if (r->buf == NULL) return 0;

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    int length = r->length - pos;
    size_t head = r->head;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length <= 0) return 0;

    head = (head + pos) % r->size;
    while (length) {
        r->buf[head] = '^';
        head = (head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    return cnt;
}

// Get data from buffer, but leave it in buffer
// Copy from the requested position in the buffer
//------------------------------------------------------------------------------
int uart_buf_copy_from(uart_ringbuf_t *r, size_t pos, uint8_t *dest, size_t len)
{
    if (r->buf == NULL) return 0;
    if (dest == NULL) return 0;    // no destination buffer

    size_t cnt = 0;
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    int length = r->length - pos;
    size_t head = r->head;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length <= 0) return 0;
    if (len > length) len = length;

    head = (head + pos) % r->size;
    while (length) {
        dest[cnt] = r->buf[head];
        head = (head + 1) % r->size;
        length--;
        cnt++;
        if (cnt >= len) break;
    }

    return cnt;
}

// Get data from buffer, but leave it in buffer
// Copy from the buffer start
//-------------------------------------------------------------
int uart_buf_copy(uart_ringbuf_t *r, uint8_t *dest, size_t len)
{
    return uart_buf_copy_from(r, 0, dest, len);
}

// Find pattern in uart buffer
//-------------------------------------------------------------------------------------------------------------------------------
int uart_buf_find_from(uart_ringbuf_t *r, size_t start_pos, size_t size, const char *pattern, int pattern_length, size_t *buflen)
{
    if (r->buf == NULL) return -1;

    int c, d, e, pos, position = -1;
    //if ((pattern == NULL) || (pattern_length == 0)) return -1;

    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    int length = r->length - start_pos;
    size_t head = r->head;
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();

    if (length <= 0) return -1;
    if (size > length) size = length;

    if (buflen) *buflen = (length > size) ? size : length;
    if (pattern_length <= length) {
        for (c = 0; c <= (length - pattern_length); c++) {
            if (c > size) break;
            position = e = c;
            // check pattern
            for (d = 0; d < pattern_length; d++) {
                pos = (head + start_pos + e) % r->size;
                if (pattern[d] == r->buf[pos]) e++;
                else {
                    position = -1;
                    break;
                }
            }
            if (d == pattern_length) break;
        }
    }

    if (position >= 0) return (start_pos + position);
    return position;
}

// Find pattern in uart buffer
//--------------------------------------------------------------------------------------------------------
int uart_buf_find(uart_ringbuf_t *r, size_t size, const char *pattern, int pattern_length, size_t *buflen)
{
    return uart_buf_find_from(r, 0, size, pattern, pattern_length, buflen);
}

// Empty uart buffer
//-------------------------------------
void uart_buf_flush(uart_ringbuf_t *r)
{
    if (r->uart_num < UART_NUM_MAX) taskENTER_CRITICAL();
    if (r->length > 0) {
        r->tail = 0;
        r->head = 0;
        r->length = 0;
        r->overflow = 0;
    }
    if (r->uart_num < UART_NUM_MAX) taskEXIT_CRITICAL();
}

//--------------------------------------
int uart_putc(uint32_t uart_num, char c)
{
    //while ((uart[uart_num]->LSR & (1u << 5)))
    //    ;
    while (!(uart[uart_num]->LSR & (1u << 6)))
        ;
    uart[uart_num]->THR = c;
    return 0;
}

//----------------------------------------------------------------
int uart_write(uint32_t uart_num, const uint8_t *buff, size_t len)
{
    int write = 0;
    while (write < len) {
        //while ((uart[uart_num]->LSR & (1u << 5)))
        //    ;
        while (!(uart[uart_num]->LSR & (1u << 6)))
            ;
        uart[uart_num]->THR = *buff++;
        write++;
    }

    return write;
}

//--------------------------------------------------
void uart_ringbuf_alloc(uint8_t uart_num, size_t sz)
{
    if (uart_num >= UART_NUM_MAX) return;
    mpy_uarts[uart_num].uart_buffer.buf = pvPortMalloc(sz);
    mpy_uarts[uart_num].uart_buffer.size = sz;
    mpy_uarts[uart_num].uart_buffer.head = 0;
    mpy_uarts[uart_num].uart_buffer.tail = 0;
    mpy_uarts[uart_num].uart_buffer.length = 0;
    mpy_uarts[uart_num].uart_buffer.uart_num = uart_num;
    mpy_uarts[uart_num].uart_buffer.notify = false;
    mpy_uarts[uart_num].uart_buf = &mpy_uarts[uart_num].uart_buffer;
}

// =========================================================================

//--------------------------------------------------------------------------------------------------------------------------
int mp_uart_config(uint32_t uart_num, uint32_t baud_rate, uint32_t databits, uart_stopbits_t stopbits, uart_parity_t parity)
{

    mpy_uarts[uart_num].uart_buf->head = 0;
    mpy_uarts[uart_num].uart_buf->tail = 0;
    mpy_uarts[uart_num].uart_buf->length = 0;

    configASSERT(databits >= 5 && databits <= 8);
    if (databits == 5) {
        configASSERT(stopbits != UART_STOP_2);
    }
    else {
        configASSERT(stopbits != UART_STOP_1_5);
    }

    uint32_t stopbit_val = stopbits == UART_STOP_1 ? 0 : 1;
    uint32_t parity_val = 0;
    switch (parity)
    {
    case UART_PARITY_NONE:
        parity_val = 0;
        break;
    case UART_PARITY_ODD:
        parity_val = 1;
        break;
    case UART_PARITY_EVEN:
        parity_val = 3;
        break;
    default:
        configASSERT(!"Invalid parity");
        break;
    }

    uint32_t freq = sysctl_clock_get_freq(SYSCTL_CLOCK_APB0); //SYSCTL_CLOCK_UART1+uart_num);
    uint32_t divisor = freq / baud_rate;
    uint8_t dlh = divisor >> 12;
    uint8_t dll = (divisor - (dlh << 12)) / UART_BRATE_CONST;
    uint8_t dlf = divisor - (dlh << 12) - dll * UART_BRATE_CONST;

    // Set UART registers
    uart[uart_num]->TCR &= ~(1u);
    uart[uart_num]->TCR &= ~(1u << 3);
    uart[uart_num]->TCR &= ~(1u << 4);
    uart[uart_num]->TCR |= (1u << 2);
    uart[uart_num]->TCR &= ~(1u << 1);
    uart[uart_num]->DE_EN &= ~(1u);

    uart[uart_num]->LCR |= 1u << 7;
    uart[uart_num]->DLH = dlh;
    uart[uart_num]->DLL = dll;
    uart[uart_num]->DLF = dlf;

    uart[uart_num]->LCR = 0;
    uart[uart_num]->LCR = (databits - 5) | (stopbit_val << 2) | (parity_val << 3);
    uart[uart_num]->LCR &= ~(1u << 7);
    uart[uart_num]->MCR &= ~3;
    uart[uart_num]->IER = 1; // interrupt on receive
    //uart[uart_num]->IER |= 0x80; // enable by THRESHOLD
    //uart[uart_num]->FCR = UART_RECEIVE_FIFO_1 << 6 | UART_SEND_FIFO_0 << 4 | 0x1 << 3 | 0x1;

    return (freq / divisor);
}

//-----------------------------------
void mp_uart_close(uint32_t uart_num)
{
    // Free the uart buffer
    if (mpy_uarts[uart_num].uart_buf != NULL) {
        if (mpy_uarts[uart_num].uart_buf->buf) vPortFree(mpy_uarts[uart_num].uart_buf->buf);
    }
    // Delete semaphore & mutex
    if (mpy_uarts[uart_num].task_semaphore != NULL) vSemaphoreDelete(mpy_uarts[uart_num].task_semaphore);
    if (mpy_uarts[uart_num].uart_mutex != NULL) vSemaphoreDelete(mpy_uarts[uart_num].uart_mutex);

    memset(&mpy_uarts[uart_num], 0, sizeof(uart_uarts_t));
}

//--------------------------------------------------------------------------------------------------------------------------
int uart_hard_init(uint32_t uart_num, uint8_t tx, uint8_t rx, gpio_pin_func_t func, bool mutex, bool semaphore, int rb_size)
{
    if (mp_used_pins[tx].func != GPIO_FUNC_NONE) {
        LOGD(TAG, "Tx %s", gpiohs_funcs_in_use[mp_used_pins[tx].func]);
        return -1;
    }
    if (mp_used_pins[rx].func != GPIO_FUNC_NONE) {
        LOGD(TAG, "Rx %s", gpiohs_funcs_in_use[mp_used_pins[rx].func]);
        return -2;
    }

    if (mpy_uarts[uart_num].uart_buf == NULL) {
        // First time, create ring buffer
        uart_ringbuf_alloc(uart_num, rb_size);
        if (mpy_uarts[uart_num].uart_buf == NULL) {
            return -3;
        }
    }

    if (mutex) {
        // Create uart mutex
        if (mpy_uarts[uart_num].uart_mutex == NULL) {
            mpy_uarts[uart_num].uart_mutex = xSemaphoreCreateMutex();
            if (!mpy_uarts[uart_num].uart_mutex) {
                return -4;
            }
        }
    }

    if (semaphore) {
        // Create uart semaphore
        if (mpy_uarts[uart_num].task_semaphore == NULL) {
            mpy_uarts[uart_num].task_semaphore = xSemaphoreCreateBinary();
            if (!mpy_uarts[uart_num].task_semaphore) {
                return -5;
            }
        }
    }

    // Configure tx & rx pins
    mp_fpioa_cfg_item_t uart_pin_func[2];
    uart_pin_func[0] = (mp_fpioa_cfg_item_t){-1, rx, GPIO_USEDAS_RX, FUNC_UART1_RX + (uart_num *2)};
    uart_pin_func[1] = (mp_fpioa_cfg_item_t){-1, tx, GPIO_USEDAS_TX, FUNC_UART1_TX + (uart_num *2)};
    // Setup and mark used pins
    fpioa_setup_pins(2, uart_pin_func);
    fpioa_setused_pins(2, uart_pin_func, func);

    // Request external uart interrupt handling
    extern_uart_irq_handler = (pic_irq_handler_t)uart_on_irq_recv;
    extern_uart_irq_userdata = (void *)&uart_instances[uart_num];

    // Open the uart device driver
    switch (uart_num) {
        case 0:
            mpy_uarts[uart_num].handle = io_open("/dev/uart1");
            break;
        case 1:
            mpy_uarts[uart_num].handle = io_open("/dev/uart2");
            break;
        case 2:
            mpy_uarts[uart_num].handle = io_open("/dev/uart3");
            break;
    }

    return 0;
}

//----------------------------------------------------------------------------
bool uart_deinit(uint32_t uart_num, uint8_t *end_task, uint8_t tx, uint8_t rx)
{
    if ((end_task) && (mpy_uarts[uart_num].task_id != NULL)) {
        // stop the uart task
        if (mpy_uarts[uart_num].uart_mutex) xSemaphoreTake(mpy_uarts[uart_num].uart_mutex, UART_MUTEX_TIMEOUT);
        *end_task = 1;
        if (mpy_uarts[uart_num].uart_mutex) xSemaphoreGive(mpy_uarts[uart_num].uart_mutex);
        // wait until ended
        int tmo = 100;
        while ((tmo > 0) && (mpy_uarts[uart_num].task_id != NULL)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            tmo--;
        }
        if (mpy_uarts[uart_num].task_id != NULL) {
            return false;
        }
    }

    // Close UART device
    if (mpy_uarts[uart_num].handle) {
        if (mpy_uarts[uart_num].handle) io_close(mpy_uarts[uart_num].handle);
        mp_uart_close(uart_num);
    }

    // Deconfigure tx & rx pins
    mp_fpioa_cfg_item_t uart_pin_func[2];
    uart_pin_func[0] = (mp_fpioa_cfg_item_t){-1, rx, GPIO_USEDAS_NONE, FUNC_RESV0};
    uart_pin_func[1] = (mp_fpioa_cfg_item_t){-1, tx, GPIO_USEDAS_NONE, FUNC_RESV0};
    fpioa_setup_pins(2, uart_pin_func);
    fpioa_freeused_pins(2, uart_pin_func);

    return true;
}

//-------------------------------------------------------------------------------------
int match_pattern(uint8_t *text, int text_length, uint8_t *pattern, int pattern_length)
{
    int c, d, e, position = -1;

    if (pattern_length > text_length) return -1;

    for (c = 0; c <= (text_length - pattern_length); c++) {
        position = e = c;
        // check pattern
        for (d = 0; d < pattern_length; d++) {
            if (pattern[d] == text[e]) e++;
            else break;
        }
        if (d == pattern_length) return position;
    }

    return -1;
}

//--------------------------------------------------------------------------------------------
static void _sched_callback(mp_obj_t function, int uart, int type, int iarglen, uint8_t *sarg)
{
    mp_obj_t tuple[4];
    tuple[0] = mp_obj_new_int(uart);
    tuple[1] = mp_obj_new_int(type);
    tuple[2] = mp_obj_new_int(iarglen);
    tuple[3] = mp_obj_new_str((const char*)sarg, iarglen);

    mp_sched_schedule(function, mp_obj_new_tuple(4, tuple));
}

//---------------------------------------------
static void uart_event_task(void *pvParameters)
{
    task_params_t *task_params = (task_params_t *)pvParameters;
    // if the task uses some MicroPython functions, we have to save
    // MicroPython state in local storage pointers
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)task_params->thread_handle, THREAD_LSP_STATE));
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)task_params->thread_handle, THREAD_LSP_ARGS));

    machine_uart_obj_t *self = (machine_uart_obj_t *)task_params->uart_obj;
    int res;

    while (1) {
    	if (self->end_task) break;
        // Waiting for UART event.
        if ((mpy_uarts[self->uart_num].uart_buf->length > 0) &&
            (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) == pdTRUE)) {
        	// Received data already placed in MPy buffer
            if ((self->error_cb) && (mpy_uarts[self->uart_num].uart_buf->overflow > 0)) {
                // MPy buffer full (overflow)
                _sched_callback(self->error_cb, self->uart_num, UART_CB_TYPE_ERROR, UART_ERROR_BUFFER_FULL, NULL);
            }
            else {
                if ((self->data_cb) && (self->data_cb_size > 0) && (mpy_uarts[self->uart_num].uart_buf->length >= self->data_cb_size)) {
                    // ** callback on data length received
                    uint8_t *dtmp = pvPortMalloc(self->data_cb_size);
                    if (dtmp) {
                        uart_buf_get(mpy_uarts[self->uart_num].uart_buf, dtmp, self->data_cb_size);
                        _sched_callback(self->data_cb, self->uart_num, UART_CB_TYPE_DATA, self->data_cb_size, dtmp);
                        vPortFree(dtmp);
                    }
                    else _sched_callback(self->data_cb, self->uart_num, UART_CB_TYPE_ERROR, UART_ERROR_NOMEM, NULL);
                }
                else if (self->pattern_cb) {
                    // ** callback on pattern received
                    size_t len = mpy_uarts[self->uart_num].uart_buf->length;
                    uint8_t *dtmp = pvPortMalloc(len+self->pattern_len);
                    if (dtmp) {
                        uart_buf_copy(mpy_uarts[self->uart_num].uart_buf, dtmp, len);
                        res = match_pattern(dtmp, len, self->pattern, self->pattern_len);
                        if (res >= 0) {
                            // found, pull data, including pattern from buffer
                            uart_buf_get(mpy_uarts[self->uart_num].uart_buf, dtmp, res+self->pattern_len);
                            _sched_callback(self->pattern_cb, self->uart_num, UART_CB_TYPE_PATTERN, res, dtmp);
                            vPortFree(dtmp);
                        }
                        else vPortFree(dtmp);
                    }
                    else _sched_callback(self->pattern_cb, self->uart_num, UART_CB_TYPE_ERROR, UART_ERROR_NOMEM, NULL);
                }
            }
            xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
        }
        else (vTaskDelay(2));
    }

    // Terminate task
    xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT);
    mpy_uarts[self->uart_num].task_id = NULL;
    xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
    vTaskDelete(NULL);
}

//-------------------------------------------------------------------------------------
static char *_uart_read_data(handle_t uart_num, size_t len, char *lnend, char *lnstart)
{
    char *rdstr = NULL;
    int rdlen = -1;
    int out_len = -1;

    uint8_t *dtmp = pvPortMalloc(len+1);
    if (dtmp) {
        memset(dtmp, 0, len+1);
        uint8_t *start_ptr = dtmp;
        out_len = len;

        // Copy buffer content to the temporary buffer
        uart_buf_copy(mpy_uarts[uart_num].uart_buf, dtmp, len);
        // And try to match the end string
        rdlen = match_pattern(dtmp, len, (uint8_t *)lnend, strlen(lnend));

        if (rdlen > 0) {
            // End string found
            if (lnstart) {
                // Match beginning string requested
                int start_idx = match_pattern(dtmp, rdlen, (uint8_t *)lnstart, strlen(lnstart));
                if (start_idx >= 0) {
                    start_ptr += start_idx;
                    rdlen += strlen(lnend);
                    out_len = rdlen - start_idx;
                }
                else {
                    vPortFree(dtmp);
                    return NULL;
                }
            }
            else {
                rdlen += strlen(lnend);
                out_len = rdlen;
            }

            // copy found string to the result string
            rdstr = pvPortMalloc(out_len+1);
            if (rdstr) {
                memcpy(rdstr, start_ptr, out_len);
                rdstr[out_len] = '\0';
            }
            else {
                // Leave the string in uart buffer
                vPortFree(dtmp);
                return NULL;
            }

            // All OK, remove data from uart buffer
            uart_buf_get(mpy_uarts[uart_num].uart_buf, (uint8_t *)dtmp, rdlen);
            vPortFree(dtmp);
        }
        else {
            // End string not found
            vPortFree(dtmp);
        }
    }
    else {
        LOGD(TAG, "uart_read: error allocating temporary buffer");
    }

    return rdstr;
}

//-----------------------------------------------------------------------------
char *_uart_read(handle_t uart_num, int timeout, char *lnend, char *lnstart)
{
    char *rdstr = NULL;
    int minlen = strlen(lnend);
    if (lnstart) minlen += strlen(lnstart);

	if (timeout == 0) {
        if (xSemaphoreTake(mpy_uarts[uart_num].uart_mutex, UART_MUTEX_TIMEOUT) != pdTRUE) {
            LOGD(TAG, "uart_read: cannot get mutex");
            return NULL;
        }
    	// check for minimal length
        size_t len = mpy_uarts[uart_num].uart_buf->length;
		if (len < minlen) {
	    	xSemaphoreGive(mpy_uarts[uart_num].uart_mutex);
	    	return NULL;
		}
		rdstr = _uart_read_data(uart_num, len, lnend, lnstart);

		xSemaphoreGive(mpy_uarts[uart_num].uart_mutex);
    }
    else {
    	// wait until 'lnend' received or timeout
    	int wait_end = mp_hal_ticks_ms() + timeout;
        int buflen = 0;
    	mp_hal_wdt_reset();
        size_t len = 0;

        while (mp_hal_ticks_ms() < wait_end) {
            if (xSemaphoreTake(mpy_uarts[uart_num].uart_mutex, UART_MUTEX_TIMEOUT) != pdTRUE) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
                mp_hal_wdt_reset();
                continue;
            }
            len = mpy_uarts[uart_num].uart_buf->length;
			if (buflen < len) {
				// ** new data received, reset timeout
				buflen = len;
		        wait_end = mp_hal_ticks_ms() + timeout;
			}
			if (len < minlen) {
				// ** too few characters received
		    	xSemaphoreGive(mpy_uarts[uart_num].uart_mutex);
	    		vTaskDelay(5 / portTICK_PERIOD_MS);
				mp_hal_wdt_reset();
				continue;
			}
			else {
                // Requested minimal length of bytes received
                rdstr = _uart_read_data(uart_num, len, lnend, lnstart);
                xSemaphoreGive(mpy_uarts[uart_num].uart_mutex);
                break;
			}
		}
    }
	return rdstr;
}


/******************************************************************************/
// MicroPython bindings for UART

//--------------------------------------
static const mp_arg_t allowed_args[] = {
    { MP_QSTR_baudrate,						 MP_ARG_INT, {.u_int = -1} },
    { MP_QSTR_bits,							 MP_ARG_INT, {.u_int = -1} },
    { MP_QSTR_parity,						 MP_ARG_INT, {.u_int = -1} },
    { MP_QSTR_stop,							 MP_ARG_INT, {.u_int = -1} },
    { MP_QSTR_tx,			MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = UART_PIN_NO_CHANGE} },
    { MP_QSTR_rx,			MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = UART_PIN_NO_CHANGE} },
    { MP_QSTR_rts,			MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = UART_PIN_NO_CHANGE} },
    { MP_QSTR_cts,			MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = UART_PIN_NO_CHANGE} },
    { MP_QSTR_timeout,		MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    { MP_QSTR_buffer_size,	MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 512} },
    { MP_QSTR_lineend,		MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    { MP_QSTR_inverted,		MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_int = -1} },
};

enum { ARG_baudrate, ARG_bits, ARG_parity, ARG_stop, ARG_tx, ARG_rx, ARG_rts, ARG_cts, ARG_timeout, ARG_buffer_size, ARG_lineend, ARG_inverted };

//-----------------------------------------------------------------------------------------------
STATIC void machine_uart_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (mpy_uarts[self->uart_num].task_id == NULL) {
        mp_printf(print, "UART(%u: Deinitialized )", self->uart_num);
        return;
    }

    char lnend[16] = {'\0'};
    int lnend_idx = 0;
    for (int i=0; i < strlen((char *)self->lineend); i++) {
    	if (self->lineend[i] == 0) break;
    	if ((self->lineend[i] < 32) || (self->lineend[i] > 126)) {
    		if (self->lineend[i] == '\r') {
        		sprintf(lnend+lnend_idx, "\\r");
        		lnend_idx += 2;
    		}
    		else if (self->lineend[i] == '\n') {
        		sprintf(lnend+lnend_idx, "\\n");
        		lnend_idx += 2;
    		}
    		else {
        		sprintf(lnend+lnend_idx, "\\x%2x", self->lineend[i]);
        		lnend_idx += 4;
    		}
    	}
    	else {
    		sprintf(lnend+lnend_idx, "%c", self->lineend[i]);
    		lnend_idx++;
    	}
    }

    mp_printf(print, "UART(%u, baudrate=%u, bits=%u, parity=%s, stop=%s, tx=%d, rx=%d, rts=%d, cts=%d\n",
        self->uart_num, self->baudrate, self->bits, _parity_name[self->parity], _stopbits_name[self->stop],
		self->tx, self->rx, self->rts, self->cts);
    mp_printf(print, "        timeout=%u, buf_size=%u, lineend=b'%s')",	self->timeout, self->buffer_size, lnend);
    if (self->data_cb) {
    	mp_printf(print, "\n     data CB: True, on len: %d", self->data_cb_size);
    }
    if (self->pattern_cb) {
    	char pattern[80] = {'\0'};
    	for (int i=0; i<self->pattern_len; i++) {
    		if ((self->pattern[i] >= 0x20) && (self->pattern[i] < 0x7f)) pattern[strlen(pattern)] = self->pattern[i];
    		else sprintf(pattern+strlen(pattern), "\\x%02x", self->pattern[i]);
    	}
    	mp_printf(print, "\n     pattern CB: True, pattern: b'%s'", pattern);
    }
    if (self->error_cb) {
    	mp_printf(print, "\n     error CB: True");
    }
    if (mpy_uarts[self->uart_num].task_id) {
    	mp_printf(print, "\n     Event task minimum stack: %u of %u",
    	        uxTaskGetStackHighWaterMark(mpy_uarts[self->uart_num].task_id) * sizeof(StackType_t), configMINIMAL_STACK_SIZE * sizeof(StackType_t));
    }
}

//--------------------------------------------------------------------------------------------------------------------------
STATIC void machine_uart_init_helper(machine_uart_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // set baudrate if needed
    if (args[ARG_baudrate].u_int > 0) {
        self->baudrate = args[ARG_baudrate].u_int;
    }

    // set data bits if needed
    if ((args[ARG_bits].u_int >= 5) && (args[ARG_bits].u_int <= 8) && (args[ARG_bits].u_int != self->bits)) {
		self->bits = args[ARG_bits].u_int;
    }

    // set parity if needed
    if ((args[ARG_parity].u_int >= UART_PARITY_NONE) && (args[ARG_parity].u_int <= UART_PARITY_EVEN) && (args[ARG_parity].u_int != self->parity)) {
        self->parity = args[ARG_parity].u_int;
    }

    // set stop bits if needed
    if ((args[ARG_stop].u_int >= UART_STOP_1) && (args[ARG_stop].u_int <= UART_STOP_2) && (args[ARG_stop].u_int != self->stop)) {
        self->stop = args[ARG_stop].u_int;
    }

    // set inverted pins
    if ((args[ARG_inverted].u_int > -1) && (args[ARG_inverted].u_int != self->inverted)) {
    	self->inverted = args[ARG_inverted].u_int;
    }

    // set pins
    if (((self->tx == -2) && (args[ARG_tx].u_int == UART_PIN_NO_CHANGE)) ||	((self->rx == -2) && (args[ARG_rx].u_int == UART_PIN_NO_CHANGE))) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "Tx&Rx pins must be set: u=machine.UART(uart_num, tx=pin, rx=pin)"));
    }

    if ((self->tx != args[ARG_tx].u_int) || (self->rx != args[ARG_rx].u_int)) {
        self->tx = args[ARG_tx].u_int;
        self->rx = args[ARG_rx].u_int;

        if (mp_used_pins[self->tx].func != GPIO_FUNC_NONE) {
            mp_printf(&mp_plat_print, "Tx pin\r\n");
            mp_raise_ValueError(gpiohs_funcs_in_use[mp_used_pins[self->tx].func]);
        }
        if (mp_used_pins[self->rx].func != GPIO_FUNC_NONE) {
            mp_printf(&mp_plat_print, "Rx pin\r\n");
            mp_raise_ValueError(gpiohs_funcs_in_use[mp_used_pins[self->rx].func]);
        }
        // Configure tx & rx pins
        mp_fpioa_cfg_item_t uart_pin_func[2];
        uart_pin_func[0] = (mp_fpioa_cfg_item_t){-1, self->rx, GPIO_USEDAS_RX, FUNC_UART1_RX + (self->uart_num *2)};
        uart_pin_func[1] = (mp_fpioa_cfg_item_t){-1, self->tx, GPIO_USEDAS_TX, FUNC_UART1_TX + (self->uart_num *2)};
        // Setup and mark used pins
        fpioa_setup_pins(2, uart_pin_func);
        fpioa_setused_pins(2, uart_pin_func, GPIO_FUNC_UART);
    }

    // set timeout
    if (args[ARG_timeout].u_int >= 0) self->timeout = args[ARG_timeout].u_int;

    // set line end
    mp_buffer_info_t lnend_buff;
	mp_obj_type_t *type = mp_obj_get_type(args[ARG_lineend].u_obj);
	if (type->buffer_p.get_buffer != NULL) {
		int ret = type->buffer_p.get_buffer(args[ARG_lineend].u_obj, &lnend_buff, MP_BUFFER_READ);
		if (ret == 0) {
			if ((lnend_buff.len > 0) && (lnend_buff.len < sizeof(self->lineend))) {
				memset(self->lineend, 0, sizeof(self->lineend));
				memcpy(self->lineend, lnend_buff.buf, lnend_buff.len);
			}
		}
	}

	// Initialize uart hardware
    int res = uart_hard_init(self->uart_num, self->tx, self->rx, GPIO_FUNC_UART, true, false, self->buffer_size);
    if (res < 0) {
        mp_raise_ValueError("Error initializing UART hardware");
    }

    self->baudrate = mp_uart_config(self->uart_num, self->baudrate, self->bits, self->stop, self->parity);
}

//------------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_uart_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // get uart id
    mp_int_t uart_num = mp_obj_get_int(args[0]);
    if (uart_num < 0 || uart_num >= UART_NUM_MAX) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "UART(%d) does not exist", uart_num));
    }


	// Create UART instance, set defaults
    machine_uart_obj_t *self = m_new_obj(machine_uart_obj_t);
    self->base.type = &machine_uart_type;
    self->uart_num = uart_num;
    self->baudrate = 115200;
    self->bits = 8;
    self->parity = 0;
    self->stop = UART_STOP_1;
    self->rts = UART_PIN_NO_CHANGE;
    self->cts = UART_PIN_NO_CHANGE;
    self->tx = -2;
    self->rx = -2;
    self->timeout = 0;
    self->pattern[0] = 0;
    self->pattern_len = 0;
    self->data_cb = 0;
    self->pattern_cb = 0;
    self->error_cb = 0;
    self->data_cb_size = 0;
    self->end_task = 0;
    sprintf((char *)self->lineend, "\r\n");

    if (mpy_uarts[uart_num].active) {
        mp_raise_ValueError("uart already used");
    }
    mpy_uarts[uart_num].active = true;

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    mp_arg_val_t kargs[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, args+1, &kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, kargs);

    // Set buffer size
    int bufsize = kargs[ARG_buffer_size].u_int;
    if (bufsize < 512) bufsize = 512;
    if (bufsize > 8192) bufsize = 8192;
    self->buffer_size = bufsize;

    machine_uart_init_helper(self, n_args - 1, args + 1, &kw_args);

    //Create a task to handle UART receiving from uart ISR
    task_params.uart_obj = (void *)self;
    task_params.thread_handle = xTaskGetCurrentTaskHandle();
    if (mpy_uarts[self->uart_num].task_id == 0) {
        BaseType_t res = xTaskCreate(
            uart_event_task,                        // function entry
            "uart_event_task",                      // task name
            configMINIMAL_STACK_SIZE,               // stack_deepth
            (void *)self,                           // function argument
            MICROPY_TASK_PRIORITY,                  // task priority
            &mpy_uarts[self->uart_num].task_id);    // task handle
        if (res != pdPASS) {
            LOGE("UART", "Event task not started");
        }
    }
    return MP_OBJ_FROM_PTR(self);
}

//-----------------------------------------------
static void _check_uart(machine_uart_obj_t *self)
{
    if (mpy_uarts[self->uart_num].task_id == NULL) {
		mp_raise_ValueError("UART not initialized");
    }
}

//-----------------------------------------------------------------------------------------
STATIC mp_obj_t machine_uart_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    machine_uart_init_helper((machine_uart_obj_t *)args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(machine_uart_init_obj, 1, machine_uart_init);

//-----------------------------------------------------
STATIC mp_obj_t machine_uart_deinit(mp_obj_t self_in) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (mpy_uarts[self->uart_num].task_id != NULL) {
        if (!uart_deinit(self->uart_num, &self->end_task, self->tx, self->rx)) {
            mp_raise_ValueError("Cannot stop UART task!");
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_uart_deinit_obj, machine_uart_deinit);

//--------------------------------------------------
STATIC mp_obj_t machine_uart_any(mp_obj_t self_in) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    _check_uart(self);
    int res = 0;
	if (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) == pdTRUE) {
	    res = mpy_uarts[self->uart_num].uart_buf->length;
	    xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
	}

    return MP_OBJ_NEW_SMALL_INT(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_uart_any_obj, machine_uart_any);

//-----------------------------------------------------
STATIC mp_obj_t machine_uart_flush(mp_obj_t self_in) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    _check_uart(self);

    if (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) == pdTRUE) {
        uart_buf_flush(mpy_uarts[self->uart_num].uart_buf);
        xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
    }

	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_uart_flush_obj, machine_uart_flush);

//-----------------------------------------------------------------
mp_obj_t machine_uart_readln(size_t n_args, const mp_obj_t *args) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    _check_uart(self);

    int timeout = self->timeout;
	if (n_args > 1) timeout = mp_obj_get_int(args[1]);

	const char *startstr = NULL;
	if (n_args > 2) startstr = mp_obj_str_get_str(args[2]);

	MP_THREAD_GIL_EXIT();
	char *rdstr = _uart_read(self->uart_num, timeout, (char *)self->lineend, (char *)startstr);
	MP_THREAD_GIL_ENTER();

	if (rdstr == NULL) return mp_obj_new_str("", 0);

	mp_obj_t res_str = mp_obj_new_str((const char *)rdstr, strlen(rdstr));
	if (rdstr != NULL) vPortFree(rdstr);

	return res_str;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_uart_readln_obj, 1, 3, machine_uart_readln);


//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_uart_callback(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
	enum { ARG_type, ARG_func, ARG_pattern, ARG_datalen };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_type,			MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_func,			MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_pattern,		MP_ARG_KW_ONLY  | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_data_len,		MP_ARG_KW_ONLY  | MP_ARG_INT, { .u_int = -1 } },
    };

    machine_uart_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    _check_uart(self);

    int datalen = -1;
    mp_buffer_info_t pattern_buff;
    int cbtype = args[ARG_type].u_int;

    if ((!mp_obj_is_fun(args[ARG_func].u_obj)) && (!mp_obj_is_meth(args[ARG_func].u_obj))) {
    	// CB function not given, disable callback
    	if (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) == pdTRUE) {
            switch(cbtype) {
                case UART_CB_TYPE_DATA:
                    self->data_cb = 0;
                    self->data_cb_size = 0;
                    break;
                case UART_CB_TYPE_PATTERN:
                    self->pattern_cb = 0;
                    self->pattern[0] = 0;
                    self->pattern_len = 0;
                    break;
                case UART_CB_TYPE_ERROR:
                    self->error_cb = 0;
                    break;
                default:
                    break;
            }
            xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
    	}
    	else return mp_const_false;
        return mp_const_true;
    }

    // Get callback parameters
    switch(cbtype) {
        case UART_CB_TYPE_DATA:
            if ((args[ARG_datalen].u_int <= 0) || (args[ARG_datalen].u_int >= self->buffer_size)) {
    			mp_raise_ValueError("invalid data length");
            }
            datalen = args[ARG_datalen].u_int;
            break;
        case UART_CB_TYPE_PATTERN:
        	{
        		bool has_pattern = false;
				mp_obj_type_t *type = mp_obj_get_type(args[ARG_pattern].u_obj);
				if (type->buffer_p.get_buffer != NULL) {
					int ret = type->buffer_p.get_buffer(args[ARG_pattern].u_obj, &pattern_buff, MP_BUFFER_READ);
					if (ret == 0) {
						if ((pattern_buff.len > 0) && (pattern_buff.len <= sizeof(self->pattern))) has_pattern = true;
					}
				}
				if (!has_pattern) {
					mp_raise_ValueError("invalid pattern");
				}
        	}
            break;
        default:
        	break;
    }

    // Set the callback
    if (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) == pdTRUE) {
        switch(cbtype) {
            case UART_CB_TYPE_DATA:
                self->data_cb_size = datalen;
                self->data_cb = args[ARG_func].u_obj;
                break;
            case UART_CB_TYPE_PATTERN:
                memcpy(self->pattern, pattern_buff.buf, pattern_buff.len);
                self->pattern_len = pattern_buff.len;
                self->pattern_cb = args[ARG_func].u_obj;
                break;
            case UART_CB_TYPE_ERROR:
                self->error_cb = args[ARG_func].u_obj;
                break;
            default:
                break;
        }
        xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
    }
    else return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_uart_callback_obj, 2, machine_uart_callback);

//=================================================================
STATIC const mp_rom_map_elem_t machine_uart_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init),			MP_ROM_PTR(&machine_uart_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),			MP_ROM_PTR(&machine_uart_deinit_obj) },

    { MP_ROM_QSTR(MP_QSTR_any),				MP_ROM_PTR(&machine_uart_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),			MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline),		MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto),		MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),			MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_readln),			MP_ROM_PTR(&machine_uart_readln_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush),			MP_ROM_PTR(&machine_uart_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_callback),		MP_ROM_PTR(&machine_uart_callback_obj) },

	// class constants
    { MP_ROM_QSTR(MP_QSTR_CBTYPE_DATA),		MP_ROM_INT(UART_CB_TYPE_DATA) },
    { MP_ROM_QSTR(MP_QSTR_CBTYPE_PATTERN),	MP_ROM_INT(UART_CB_TYPE_PATTERN) },
    { MP_ROM_QSTR(MP_QSTR_CBTYPE_ERROR),	MP_ROM_INT(UART_CB_TYPE_ERROR) },
};
STATIC MP_DEFINE_CONST_DICT(machine_uart_locals_dict, machine_uart_locals_dict_table);


// === Stream UART functions ===

//------------------------------------------------------------------------------------------------
STATIC mp_uint_t machine_uart_read(mp_obj_t self_in, void *buf_in, mp_uint_t size, int *errcode) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if ((mpy_uarts[self->uart_num].task_id == NULL) || (mpy_uarts[self->uart_num].uart_buf == NULL)) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }

    // make sure we want at least 1 char
    if (size == 0) return 0;

    int bytes_read = 0;
    if (self->timeout == 0) {
    	// just return the buffer content
        if (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) != pdTRUE) {
            LOGD(TAG, "Read: cannot get mutex");
            return 0;
        }
    	bytes_read = uart_buf_get(mpy_uarts[self->uart_num].uart_buf, (uint8_t *)buf_in, size);
    	if (mpy_uarts[self->uart_num].uart_mutex) xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
    	if (bytes_read < 0) bytes_read = 0;
    }
    else {
    	// wait until data received or timeout
    	mp_hal_wdt_reset();
        int wait_end = mp_hal_ticks_ms() + self->timeout;
		MP_THREAD_GIL_EXIT();
		while (mp_hal_ticks_ms() < wait_end) {
            if (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) != pdTRUE) {
                vTaskDelay(2 / portTICK_PERIOD_MS);
                mp_hal_wdt_reset();
                continue;
            }

            if (mpy_uarts[self->uart_num].uart_buf->length < size) {
		    	xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
	    		vTaskDelay(2 / portTICK_PERIOD_MS);
				mp_hal_wdt_reset();
				continue;
			}
            // Timeout, read as many bytes as available
	    	bytes_read = uart_buf_get(mpy_uarts[self->uart_num].uart_buf, (uint8_t *)buf_in, size);
	    	xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);
			break;
		}
		MP_THREAD_GIL_ENTER();
    }

    if (bytes_read < 0) {
        *errcode = MP_EAGAIN;
        return MP_STREAM_ERROR;
    }

    return bytes_read;
}

//-------------------------------------------------------------------------------------------------------
STATIC mp_uint_t machine_uart_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (mpy_uarts[self->uart_num].task_id == NULL) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }

    int bytes_written = uart_write(self->uart_num, buf_in, size);

    if (bytes_written < 0) {
        *errcode = MP_EAGAIN;
        return MP_STREAM_ERROR;
    }

    // return number of bytes written
    return bytes_written;
}

//-----------------------------------------------------------------------------------------------------
STATIC mp_uint_t machine_uart_ioctl(mp_obj_t self_in, mp_uint_t request, mp_uint_t arg, int *errcode) {
    machine_uart_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (mpy_uarts[self->uart_num].task_id == NULL) {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }

    mp_uint_t ret;
    if (request == MP_STREAM_POLL) {
        mp_uint_t flags = arg;
        ret = 0;
        size_t rxbufsize;

        if (xSemaphoreTake(mpy_uarts[self->uart_num].uart_mutex, UART_MUTEX_TIMEOUT) != pdTRUE) {
            *errcode = MP_EINVAL;
            return MP_STREAM_ERROR;
        }
        rxbufsize = mpy_uarts[self->uart_num].uart_buf->length;
    	xSemaphoreGive(mpy_uarts[self->uart_num].uart_mutex);

        if ((flags & MP_STREAM_POLL_RD) && rxbufsize > 0) {
            ret |= MP_STREAM_POLL_RD;
        }
        if ((flags & MP_STREAM_POLL_WR) && 1) { // FIXME: uart_tx_any_room(self->uart_num)
            ret |= MP_STREAM_POLL_WR;
        }
    }
    else {
        *errcode = MP_EINVAL;
        ret = MP_STREAM_ERROR;
    }
    return ret;
}

//==========================================
STATIC const mp_stream_p_t uart_stream_p = {
    .read = machine_uart_read,
    .write = machine_uart_write,
    .ioctl = machine_uart_ioctl,
    .is_text = false,
};

//=======================================
const mp_obj_type_t machine_uart_type = {
    { &mp_type_type },
    .name = MP_QSTR_UART,
    .print = machine_uart_print,
    .make_new = machine_uart_make_new,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &uart_stream_p,
    .locals_dict = (mp_obj_dict_t*)&machine_uart_locals_dict,
};

