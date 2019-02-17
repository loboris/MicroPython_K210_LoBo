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

#ifndef INCLUDED_MPHALPORT_H
#define INCLUDED_MPHALPORT_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "py/ringbuf.h"
#include "lib/utils/interrupt_char.h"

#define BLOCK_CTRL_READY    0x06
#define BLOCK_CTRL_REPEAT   0x07
#define BLOCK_CTRL_ABORT    0x08
#define BLOCK_CTRL_ABORT_F  0x09
#define BLOCK_CTRL_ABORT_T  0x0A

extern TaskHandle_t mp_hal_tick_handle;

#if MICROPY_USE_TWO_MAIN_TASKS
extern uint32_t ipc_request;
extern char *ipc_cmd_buff;
extern char *ipc_response_buff;
extern uint32_t ipc_cmd_buff_size;
extern uint32_t ipc_response_buff_size;
extern uint32_t ipc_response_buff_idx;
extern SemaphoreHandle_t inter_proc_mutex;
extern QueueSetMemberHandle_t inter_proc_semaphore;
#endif

typedef struct {
    uint8_t     *buf;
    uint32_t    len;
    uint32_t    pos;
    bool        done;
} recv_block_t;

void mp_hal_uarths_setirqhandle(void *irq_handler, void *userdata);
void mp_hal_uarths_setirq_default();
void mp_hal_uarths_setirq_ymodem();
int mp_hal_stdin_rx_chr(void);
void mp_hal_stdout_tx_strn(const char *str, size_t len);
void mp_hal_debug_tx_strn_cooked(void *env, const char *str, size_t len);
void mp_hal_purge_uart_buffer();

uint16_t mp_hal_crc16(const uint8_t *buf, uint32_t count);
int32_t mp_hal_receive_byte (unsigned char *c, uint32_t timeout);
void mp_hal_send_bytes(char *buf, int len);
void mp_hal_send_byte(char c);
int mp_hal_get_file_block(uint8_t *buff, int size);
int mp_hal_send_file_block(uint8_t *buff, int size, bool docrc);

mp_uint_t mp_hal_ticks_cpu(void);
mp_uint_t mp_hal_ticks_us(void);
mp_uint_t mp_hal_ticks_ms(void);

void mp_hal_usdelay(uint16_t us);
mp_uint_t _mp_hal_delay_us(mp_uint_t us);
mp_uint_t _mp_hal_delay_ms(mp_uint_t ms);
void mp_hal_delay_us(mp_uint_t us);
void mp_hal_delay_ms(mp_uint_t ms);
void mp_hal_init(void);

#endif

