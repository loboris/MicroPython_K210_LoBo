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

#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

#include "mpconfigport.h"
#include "encoding.h"
#include "py/obj.h"
#include "py/mpstate.h"
#include "py/mphal.h"
#include "extmod/misc.h"
#include "lib/utils/pyexec.h"
#include "mphalport.h"
#include "uarths.h"
#include <devices.h>
#include <pin_cfg.h>
#include "encoding.h"
#include "sysctl.h"
#include "sleep.h"
#include "syslog.h"
#include "hal.h"
#include "wdt.h"
#include "mpthreadport.h"
#include "modmachine.h"

extern int MainTaskProc;

handle_t mpy_wdt;
TaskHandle_t mp_hal_tick_handle = 0;
static uint8_t stdin_ringbuf_array[MICRO_PY_UARTHS_BUFFER_SIZE];
ringbuf_t stdin_ringbuf = {stdin_ringbuf_array, sizeof(stdin_ringbuf_array), 0, 0};

#if MICROPY_USE_TWO_MAIN_TASKS
uint32_t ipc_request = 0;
char *ipc_cmd_buff = NULL;
char *ipc_response_buff = NULL;
uint32_t ipc_cmd_buff_size = 0;
uint32_t ipc_response_buff_size = 0;
uint32_t ipc_response_buff_idx = 0;
SemaphoreHandle_t inter_proc_mutex = NULL;
QueueSetMemberHandle_t inter_proc_semaphore = NULL;
#endif

static QueueSetMemberHandle_t mp_hal_uart_semaphore = NULL;
static volatile uarths_t *const uarths = (volatile uarths_t *)UARTHS_BASE_ADDR;

//static volatile wdt_t *wdt_;
static volatile int wdt_count = 0;
static uint64_t rtos_rt_counter = 0;


#if USE_MICROPY_VM_HOOK_LOOP

static volatile bool mp_hall_kbd_irq = false;

// -------------------------------------------------
// The function is executed from MicroPython VM loop
//=================
void vm_loop_hook()
{
    if (mp_hall_kbd_irq) {
        mp_hall_kbd_irq = false;
        // inline version of mp_keyboard_interrupt();
        MP_STATE_VM(mp_pending_exception) = MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception));
        #if MICROPY_ENABLE_SCHEDULER
        if (MP_STATE_VM(sched_state) == MP_SCHED_IDLE) {
            MP_STATE_VM(sched_state) = MP_SCHED_PENDING;
        }
        #endif
    }
}
#endif

// === FreeRTOS functions ====================================
#if ( configGENERATE_RUN_TIME_STATS == 1 )
//=========================================
void vConfigureTimerForRunTimeStats( void )
{
    rtos_rt_counter = 0;
}

//================================
uint64_t vGetRunTimeCounterValue()
{
    return (read_csr64(mcycle) - rtos_rt_counter) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);
}
#endif
// ===========================================================

// FreeRTOS task running at lowest priority
// restarts WDT counter
//------------------------------------------
static void sys_tick_task(void *pvParameter)
{
    //uint64_t notify_val = 0;
    //int notify_res = 0;
    while (1) {
        wdt_restart_counter(mpy_wdt);
        wdt_count = 0;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /*
        //ToDo: Workaround for issue with creating the task on different processor
        notify_res = xTaskNotifyWait(0, ULONG_MAX, &notify_val, 1000 / portTICK_RATE_MS);
        if (notify_res != pdPASS) continue;
        if (notify_val == 0) break; // Terminate task requested
        create_task_params_t *params = (create_task_params_t *)notify_val;
        int res = xTaskCreateAtProcessor(
                params->uxProcessor,    // processor
                params->pxTaskCode,     // function entry
                params->pcName,         // task name
                params->usStackDepth,   // stack_deepth
                params->pvParameters,   // function argument
                params->uxPriority,     // task priority
                params->pxCreatedTask); // task handle
        if ((params->pxCreatedTask != NULL) && (res != pdPASS)) *params->pxCreatedTask = NULL;
        */
    }
    vTaskDelete(NULL);
}

// uarths RX interrupt handler
//=============================================
static void on_irq_haluart_recv(void *userdata)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint8_t c;
    uarths_rxdata_t recv;
    recv = uarths->rxdata;
    if (!recv.empty) {
        c = (int)recv.data;
        if ((mp_interrupt_char >= 0) && (c == mp_interrupt_char)) {
            #if USE_MICROPY_VM_HOOK_LOOP
            // Notify the main MicroPython thread about keyboard interrupt character
            mp_hall_kbd_irq = true;
            #else
            // inline version of mp_keyboard_interrupt();
            MP_STATE_VM(mp_pending_exception) = MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception));
            #if MICROPY_ENABLE_SCHEDULER
            if (MP_STATE_VM(sched_state) == MP_SCHED_IDLE) {
                MP_STATE_VM(sched_state) = MP_SCHED_PENDING;
            }
            #endif
            #endif
        }
        else {
            // Push the character to the uart buffer
            ringbuf_put(&stdin_ringbuf, (uint8_t)c);
            // Inform MicroPython RX function about new character in buffer
            xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(mp_hal_uart_semaphore, &xHigherPriorityTaskWoken);
            if( xHigherPriorityTaskWoken != pdFALSE ) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

//==============================================
static void on_irq_haluart_recv2(void *userdata)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint8_t c;
    uarths_rxdata_t recv;
    recv = uarths->rxdata;
    if (!recv.empty) {
        c = (int)recv.data;
        // Push the character to the uart buffer
        ringbuf_put(&stdin_ringbuf, (uint8_t)c);
        // Inform MicroPython RX function about new character in buffer
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(mp_hal_uart_semaphore, &xHigherPriorityTaskWoken);
        if( xHigherPriorityTaskWoken != pdFALSE ) {
            portYIELD_FROM_ISR();
        }
    }
}

// WDT interrupt handler
//=======================================
static int on_wdt_timeout(void *userdata)
{
    wdt_restart_counter(mpy_wdt);
    wdt_count++;
    LOGW("[WDT]", "Watchdog timeout occurred (%d)", wdt_count);
    if (wdt_count > 10) {
        LOGE("[WDT]", "System reset\n");
        mp_hal_usdelay(500);
        sysctl->soft_reset.soft_reset = 1;
    }
    return 0;
}

//----------------------------------------------------------------
void mp_hal_uarths_setirqhandle(void *irq_handler, void *userdata)
{
    pic_set_irq_enable(IRQN_UARTHS_INTERRUPT, 0);
    uarths->rxctrl.rxcnt = 0;
    uarths->ie.txwm = 0;
    uarths->ie.rxwm = 0;
    pic_set_irq_handler(IRQN_UARTHS_INTERRUPT, NULL, NULL);
    if (irq_handler != NULL) {
        uarths->rxctrl.rxcnt = 0;
        uarths->ie.txwm = 0;
        uarths->ie.rxwm = 1;
        pic_set_irq_handler(IRQN_UARTHS_INTERRUPT, irq_handler, userdata);
        pic_set_irq_priority(IRQN_UARTHS_INTERRUPT, 1);
        pic_set_irq_enable(IRQN_UARTHS_INTERRUPT, 1);
    }
}

//---------------------------------
void mp_hal_uarths_setirq_default()
{
    mp_hal_uarths_setirqhandle(on_irq_haluart_recv, NULL);
}

//--------------------------------
void mp_hal_uarths_setirq_ymodem()
{
    mp_hal_uarths_setirqhandle(on_irq_haluart_recv2, NULL);
}

// This function is called only once, on system start
//====================
void mp_hal_init(void)
{
    // Set the PU clock
    system_set_cpu_frequency(MICRO_PY_DEFAULT_CPU_CLOCK);
    log_divisor = (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);

    // Set the REPL uart baudrate and configure uarths
    uarths_baudrate = uarths_init(MICRO_PY_DEFAULT_BAUDRATE);

    mp_fpioa_cfg_item_t functions[2];
    functions[0] = (mp_fpioa_cfg_item_t){-1, 4, FUNC_UARTHS_RX};
    functions[1] = (mp_fpioa_cfg_item_t){-1, 5, FUNC_UARTHS_TX};
    fpioa_setused_pins(2, functions, GPIO_FUNC_ISP_UART);

    //LOGM("[HAL]", "Config");
    mp_hal_set_interrupt_char(-1);

    // Setup semaphores and mutexes
    if (mp_hal_uart_semaphore == NULL) {
        mp_hal_uart_semaphore = xSemaphoreCreateBinary();
        configASSERT(mp_hal_uart_semaphore);
    }
    #if MICROPY_USE_TWO_MAIN_TASKS
    if (inter_proc_mutex == NULL) {
        inter_proc_mutex = xSemaphoreCreateMutex();
        configASSERT(inter_proc_mutex);
        inter_proc_semaphore = xSemaphoreCreateBinary();
        configASSERT(inter_proc_semaphore);
    }
    #endif

    // Configure Watchdog
    mpy_wdt = io_open("/dev/wdt0");
    configASSERT(mpy_wdt);
    wdt_set_enable(mpy_wdt, 0);

    wdt_set_response_mode(mpy_wdt, /*WDT_RESP_RESET*/ WDT_RESP_INTERRUPT);
    wdt_set_timeout(mpy_wdt, 6*1e9); //6sec
    wdt_set_on_timeout(mpy_wdt, on_wdt_timeout, NULL);
    wdt_set_enable(mpy_wdt, 1);

    // Configure uarths (stdio uart) for interrupts
    mp_hal_uarths_setirq_default();

    // Create systick task with lowest priority
    xTaskCreate(
            sys_tick_task,              // function entry
            "hal_tick_task",            // task name
            configMINIMAL_STACK_SIZE,   // stack_deepth
            NULL,                       // function argument
            0,                          // task priority
            &mp_hal_tick_handle);       // task handle
    configASSERT(mp_hal_tick_handle);

    //LOGM("[HAL]", "Initialized");
}

// ===================================
// === MicroPython stdio functions ===
// ===================================


// MicroPython character receive function
//---------------------------
int mp_hal_stdin_rx_chr(void)
{
    #if MICROPY_USE_TWO_MAIN_TASKS
    if (uxPortGetProcessorId() != MainTaskProc) return -1;
    #endif
    int c = -1;

    for (;;) {
        uarths->ie.rxwm = 0;
        c = ringbuf_get(&stdin_ringbuf);
        uarths->ie.rxwm = 1;
        if (c < 0) {
            // no character in ring buffer
            // wait max 10 ms for character
            MP_THREAD_GIL_EXIT();
            if ( xSemaphoreTake( mp_hal_uart_semaphore, 10 / portTICK_PERIOD_MS ) == pdTRUE ) {
                // received
                MP_THREAD_GIL_ENTER();
                uarths->ie.rxwm = 0;
                c = ringbuf_get(&stdin_ringbuf);
                uarths->ie.rxwm = 1;
            }
            else {
                // not received
                MP_THREAD_GIL_ENTER();
                c = -1;
            }
        }
        if (c >= 0) {
            return c;
        }
        mp_handle_pending();
    }
    return -1;
}

//-----------------------------------------------------------------------
void mp_hal_debug_tx_strn_cooked(void *env, const char *str, size_t len);

//--------------------------------------------------------------------
const mp_print_t mp_debug_print = {NULL, mp_hal_debug_tx_strn_cooked};

// Send string of given length
//-----------------------------------------------------
void mp_hal_stdout_tx_strn(const char *str, size_t len)
{
    #if MICROPY_USE_TWO_MAIN_TASKS
    if (uxPortGetProcessorId() != MainTaskProc) {
        if (ipc_response_buff == NULL) {
            ipc_response_buff = calloc((len > 1024) ? len : 1024, 1);
            if (ipc_response_buff) {
                ipc_response_buff_size = 1024;
                ipc_response_buff_idx = 0;
            }
        }
        if (ipc_response_buff) {
            if ((ipc_response_buff_size-ipc_response_buff_idx) > len) {
                memcpy(ipc_response_buff+ipc_response_buff_idx, str, len);
                ipc_response_buff_idx += len;
            }
        }
        return;
    }
    #endif

    // Only release the GIL if many characters are being sent
    bool release_gil = len > 20;
    if (release_gil) {
        MP_THREAD_GIL_EXIT();
    }
    for (uint32_t i = 0; i < len; ++i) {
        uarths_write_byte(str[i]);
    }
    if (release_gil) {
        MP_THREAD_GIL_ENTER();
    }
    mp_uos_dupterm_tx_strn(str, len);
}

//----------------------------------------------------------------------
void mp_hal_debug_tx_strn_cooked(void *env, const char *str, size_t len)
{
    (void)env;
    #if MICROPY_USE_TWO_MAIN_TASKS
    if (uxPortGetProcessorId() != MainTaskProc) return;
    #endif

    char prev = '\0';
    while (len--) {
        if ((*str == '\n') && (prev != '\r')) {
            uarths_write_byte('\r');
        }
        uarths_write_byte(*str++);
        prev = *str;
    }
}

// ------------------------------
// Block send & receive functions
// ------------------------------


// uarths RX interrupt handler for block transfer
//============================================
void on_irq_haluart_recv_block(void *userdata)
{
    recv_block_t *cfg = (recv_block_t *)userdata;
    uint8_t c;
    uarths_rxdata_t recv;

    recv = uarths->rxdata;
    if (!recv.empty) {
        c = (uint8_t)recv.data;
        // Push the character to the buffer
        if (cfg->pos < cfg->len) {
            cfg->buf[cfg->pos] = (uint8_t)c;
            cfg->pos++;
            if (cfg->pos == cfg->len) {
                uarths->ie.rxwm = 0;
                cfg->done = true;
            }
        }
    }
}

//----------------------------------------
void mp_hal_send_bytes(char *buf, int len)
{
    uint8_t *pbuf = (uint8_t *)buf;
    while (len--) {
        uarths_write_byte(*pbuf++);
    }
}

//---------------------------
void mp_hal_send_byte(char c)
{
    uarths_write_byte(c);
}

//-------------------------------------------------------
uint16_t mp_hal_crc16(const uint8_t *buf, uint32_t count)
{
    uint16_t crc = 0;
    int i;
    const uint8_t *pbuf = buf;

    while(count--) {
        crc = crc ^ *pbuf++ << 8;

        for (i=0; i<8; i++) {
            if (crc & 0x8000) crc = crc << 1 ^ 0x1021;
            else crc = crc << 1;
        }
    }
    return crc;
}

//-------------------------------------------------------
int32_t mp_hal_receive_byte(uint8_t *c, uint32_t timeout)
{
    int cc = -1;

    uarths->ie.rxwm = 0;
    cc = ringbuf_get(&stdin_ringbuf);
    uarths->ie.rxwm = 1;
    if (cc < 0) {
        // no character in ring buffer
        // wait max 'timeout' ms for character
        if ( xSemaphoreTake( mp_hal_uart_semaphore, timeout / portTICK_PERIOD_MS ) == pdTRUE ) {
            // received
            uarths->ie.rxwm = 0;
            cc = ringbuf_get(&stdin_ringbuf);
            uarths->ie.rxwm = 1;
        }
    }
    if (cc >= 0) {
        *c = (uint8_t)cc;
        return 0;
    }
    return -1;
}

//-----------------------------
void mp_hal_purge_uart_buffer()
{
    int cc = -1;

    uarths->ie.rxwm = 0;
    cc = ringbuf_get(&stdin_ringbuf);
    while (cc >= 0) {
        cc = ringbuf_get(&stdin_ringbuf);
    }
    uarths->ie.rxwm = 1;
}

//------------------------------------------------
int mp_hal_get_file_block(uint8_t *buff, int size)
{
    int res, ntry=0;
    uint16_t crc, crc_rec;
    uint64_t tend;
    recv_block_t cfg;
    cfg.buf = buff;
    cfg.len = size+2;
    cfg.pos = 0;
    cfg.done = false;
    mp_hal_uarths_setirqhandle(NULL, NULL);
    xSemaphoreTake(mp_hal_uart_semaphore, 0);

    while (1) {
        // send ready character
        cfg.pos = 0;
        cfg.done = false;
        mp_hal_uarths_setirqhandle(on_irq_haluart_recv_block, (void *)&cfg);
        if (ntry == 0) uarths_write_byte(BLOCK_CTRL_READY);
        mp_hal_delay_ms(50);

        // receive data block
        tend = mp_hal_ticks_ms() + 2000;
        while (mp_hal_ticks_ms() < tend) {
            vTaskDelay(1);
            if (cfg.done) break;
        }
        if (cfg.done) {
            // data block + crc received
            mp_hal_uarths_setirqhandle(NULL, NULL);

            crc_rec = (uint16_t)(buff[size] << 8);
            crc_rec |= (uint16_t)(buff[size+1] & 0xFF);
            crc = mp_hal_crc16(buff, size);
            // check crc
            if (crc == crc_rec) {
                res = 0;
                break;
            }
            // bad crc, repeat block receive
            mp_hal_delay_ms(50);
            ntry++;
            if (ntry < 4) uarths_write_byte(BLOCK_CTRL_REPEAT);
            else {
                // maximal number of retries reached, abort
                uarths_write_byte(BLOCK_CTRL_ABORT);
                res = -2;
                break;
            }
        }
        else {
            // not received, timeout
            mp_hal_uarths_setirqhandle(NULL, NULL);
            uarths_write_byte(BLOCK_CTRL_ABORT_T);
            res = -1;
            break;
        }
    }

    mp_hal_uarths_setirq_default();
    if (res != 0) mp_hal_delay_ms(100);
    return res;
}

//-------------------------------------------------------------
int mp_hal_send_file_block(uint8_t *buff, int size, bool docrc)
{
    int res;
    uint16_t crc;
    uint8_t ack;
    uint64_t tend;
    recv_block_t cfg;
    cfg.buf = &ack;
    cfg.len = 1;
    cfg.pos = 0;
    cfg.done = false;
    mp_hal_uarths_setirqhandle(NULL, NULL);
    xSemaphoreTake(mp_hal_uart_semaphore, 0);

    if (docrc) {
        // prepare crc
        crc = mp_hal_crc16(buff, size);
        buff[size] = (uint8_t)(crc >> 8);
        buff[size+1] = (uint8_t)(crc & 0xFF);
    }

    while (1) {
        cfg.pos = 0;
        cfg.done = false;
        mp_hal_uarths_setirqhandle(on_irq_haluart_recv_block, (void *)&cfg);
        // send block & crc
        mp_hal_send_bytes((char *)buff, size+2);

        // wait for acknowledge, max 2 seconds
        tend = mp_hal_ticks_ms() + 4000;
        while (mp_hal_ticks_ms() < tend) {
            vTaskDelay(1);
            if (cfg.done) break;
        }
        if (cfg.done) {
            mp_hal_uarths_setirqhandle(NULL, NULL);
            // ack received
            if (ack == BLOCK_CTRL_REPEAT) {
                // repeat block requested, send it again
                mp_hal_delay_ms(50);
                continue;
            }
            if (ack == BLOCK_CTRL_ABORT) {
                // abort requested
                res = -2;
                break;
            }
            // block received by host, exit
            res = 0;
            break;
        }
        else {
            // ack not received, timeout
            mp_hal_uarths_setirqhandle(NULL, NULL);
            res = -1;
            break;
        }
    }

    mp_hal_uarths_setirq_default();
    return res;
}

// ===========================================
// === MicroPython ticks & sleep functions ===
// ===========================================

//------------------------------
mp_uint_t mp_hal_ticks_cpu(void)
{
    return read_csr64(mcycle);
}

//-----------------------------
mp_uint_t mp_hal_ticks_us(void)
{
	return (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
}

//-----------------------------
mp_uint_t mp_hal_ticks_ms(void)
{
    return (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000));
}

//------------------------------
void mp_hal_usdelay(uint16_t us)
{
    uint64_t start_us = mp_hal_ticks_us();
    uint64_t end_us = start_us + us;
    while (mp_hal_ticks_us() < end_us) {}
}

/*
 * Delay specified number of micro seconds
 * For the delay time up to 2000 us the function is blocking
 * For delay times greater than 2000 us, the function
 * does not block the execution of the other threads.
 * Waiting can be interrupted if the thread receives the notification
 *
 * Returns the actual delay time.
 *
 */
//--------------------------------------
mp_uint_t _mp_hal_delay_us(mp_uint_t us)
{
    if (us == 0) return 0;
    mp_uint_t start_us = mp_hal_ticks_us();
    mp_uint_t end_us = start_us + us;
    if (us <= (portTICK_PERIOD_MS*2000)) {
        // For delays up to 2000 us we use blocking delay
        while (mp_hal_ticks_us() < end_us) {}
        return us;
    }
    // Dont accept interrupt character while sleeping
    int intr_c = mp_interrupt_char;
    mp_interrupt_char = -1;

    // For longer sleeps, use FreeRTOS ticks to wait
    // While sleeping, allow other threads to run
    MP_THREAD_GIL_EXIT();

    mp_uint_t tend = end_us - 1000;
    mp_uint_t tcurr = start_us;
    mp_uint_t tellapsed = 0;
    bool notified = false;
    int ncheck = 0;
    while (!notified) {
        tcurr = mp_hal_ticks_us();
        tellapsed = tcurr - start_us;
        if (tcurr >= tend) break;
        // wait 1 ms
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // Break if notification received (check every 100 ms)
        ncheck++;
        if (ncheck >= 100) {
            ncheck = 0;
            if (mp_thread_getnotify(1)) {
                notified = true;
                break;
            }
        }
    }
    if (!notified) {
        // wait remaining us time
        tcurr = mp_hal_ticks_us();
        while (tcurr < end_us) {
            tcurr = mp_hal_ticks_us();
        }
        tellapsed = tcurr - start_us;
    }

    // Restore interrupt character
    mp_interrupt_char = intr_c;
    MP_THREAD_GIL_ENTER();
    return tellapsed;
}

//--------------------------------
void mp_hal_delay_us(mp_uint_t us)
{
    _mp_hal_delay_us(us);
}

//--------------------------------
void mp_hal_delay_ms(mp_uint_t ms)
{
    _mp_hal_delay_us(ms * 1000);
}

//--------------------------------------
mp_uint_t _mp_hal_delay_ms(mp_uint_t ms)
{
    return _mp_hal_delay_us(ms * 1000);
}


