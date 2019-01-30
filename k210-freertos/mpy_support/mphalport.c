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

#include "FreeRTOS.h"
#include "task.h"
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
#include "mpthreadport.h"
#include "modmachine.h"

extern int MainTaskProc;

handle_t mpy_wdt;
static uint8_t stdin_ringbuf_array[1024];
ringbuf_t stdin_ringbuf = {stdin_ringbuf_array, sizeof(stdin_ringbuf_array), 0, 0};

static QueueSetMemberHandle_t mp_hal_uart_semaphore = NULL;
static SemaphoreHandle_t sys_tick_mutex = NULL;
static TaskHandle_t mp_hal_tick_handle = 0;
static volatile uarths_t *const uarths = (volatile uarths_t *)UARTHS_BASE_ADDR;

static int wdt_count = 0;
static uint64_t rtos_rt_counter = 0;


// -------------------------------------------------
// The function is executed from MicroPython VM loop
//=================
void vm_loop_hook()
{
    uint32_t nval = mp_thread_getnotify(false);
    if (nval == THREAD_KBD_EXCEPTION) {
        // inline version of mp_keyboard_interrupt();
        MP_STATE_VM(mp_pending_exception) = MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception));
        #if MICROPY_ENABLE_SCHEDULER
        if (MP_STATE_VM(sched_state) == MP_SCHED_IDLE) {
            MP_STATE_VM(sched_state) = MP_SCHED_PENDING;
        }
        #endif
    }
}

// === FreeRTOS functions =======

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

// FreeRTOS task running at lowest priority
// restarts WDT counter
//------------------------------------------
static void sys_tick_task(void *pvParameter)
{
    while (1) {
        wdt_restart_counter(mpy_wdt);
        wdt_count = 0;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
            // Notify the main MicroPython thread about keyboard interrupt character
            xTaskNotifyFromISR(MainTaskHandle, THREAD_KBD_EXCEPTION, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        }
        else {
            // Push the character to the uart buffer
            ringbuf_put(&stdin_ringbuf, (uint8_t)c);
            // Inform MicroPython RX function about new character in buffer
            xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(mp_hal_uart_semaphore, &xHigherPriorityTaskWoken);
            if( xHigherPriorityTaskWoken != pdFALSE ) {
                //portYIELD();
            }
        }
    }
}

// WDT interrupt handler
//=======================================
static int on_wdt_timeout(void *userdata)
{
    wdt_restart_counter(mpy_wdt);
    wdt_count++;
    LOGW("WDT", "Watchdog timeout occured");
    if (wdt_count > 10) {
        //wdt_set_response_mode(mpy_wdt, WDT_RESP_RESET);
        //LOGE("WDT", "Next Watchdog timeout will reset");
        LOGE("WDT", "System reset");
        sysctl->soft_reset.soft_reset = 1;
    }
    return 0;
}

//------------------------------------------------
void mp_hal_uarths_setirqhandle(void *irq_handler)
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
        pic_set_irq_handler(IRQN_UARTHS_INTERRUPT, irq_handler, NULL);
        pic_set_irq_priority(IRQN_UARTHS_INTERRUPT, 1);
        pic_set_irq_enable(IRQN_UARTHS_INTERRUPT, 1);
    }
}

//---------------------------------
void mp_hal_uarths_setirq_default()
{
    mp_hal_uarths_setirqhandle(on_irq_haluart_recv);
}

// This function is called only once, on system start
//====================
void mp_hal_init(void)
{
    system_set_cpu_frequency(MICRO_PY_DEFAULT_CPU_CLOCK/2);

    mp_fpioa_cfg_item_t functions[2];
    functions[0] = (mp_fpioa_cfg_item_t){-1, 4, FUNC_UARTHS_RX};
    functions[1] = (mp_fpioa_cfg_item_t){-1, 5, FUNC_UARTHS_TX};
    fpioa_setused_pins(2, functions, GPIO_FUNC_ISP_UART);

    mp_hal_set_interrupt_char(-1);

    // Setup semaphores and mutexes
    if (mp_hal_uart_semaphore == NULL) {
        mp_hal_uart_semaphore = xSemaphoreCreateBinary();
        configASSERT(mp_hal_uart_semaphore);
    }
    if (sys_tick_mutex == NULL) {
        sys_tick_mutex = xSemaphoreCreateMutex();
        configASSERT(sys_tick_mutex);
    }

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
    xTaskCreateAtProcessor(
            MainTaskProc ^ 1,           // processor
            sys_tick_task,              // function entry
            "hal_tick_task",            // task name
            configMINIMAL_STACK_SIZE,   // stack_deepth
            NULL,                       // function argument
            tskIDLE_PRIORITY,           // task priority
            &mp_hal_tick_handle);       // task handle
    configASSERT(mp_hal_tick_handle);

    LOGD("HAL", "Initialized");
}

// === MicroPython stdio functions ===

// MicroPython character receive function
//---------------------------
int mp_hal_stdin_rx_chr(void)
{
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

//------------------------------------------------------------------------
void mp_hal_debug_tx_strn_cooked(void *env, const char *str, size_t len) {
    (void)env;

    char prev = '\0';
    while (len--) {
        if ((*str == '\n') && (prev != '\r')) {
            uarths_write_byte('\r');
        }
        uarths_write_byte(*str++);
        prev = *str;
    }
}

// === MicroPython ticks & sleep functions ===

//------------------------------
mp_uint_t mp_hal_ticks_cpu(void)
{
    //return update_systicks();
    return read_csr64(mcycle);
}

//-----------------------------
mp_uint_t mp_hal_ticks_us(void)
{
    //uint64_t ticks = update_systicks();
	return (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
}

//-----------------------------
mp_uint_t mp_hal_ticks_ms(void)
{
    //uint64_t ticks = update_systicks();
    return (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000));
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


