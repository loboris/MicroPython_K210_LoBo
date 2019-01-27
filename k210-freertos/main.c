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

/*****std lib****/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <devices.h>
#include <stdarg.h>

//*****mpy****
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/stackctrl.h"
#include "py/mpstate.h"
#include "py/nlr.h"
#include "py/compile.h"
#include "py/mphal.h"
#include "gccollect.h"
#include "lib/utils/pyexec.h"
#include "lib/mp-readline/readline.h"
#include "lib/utils/interrupt_char.h"
#include "py/stream.h"
#include "extmod/vfs.h"

#if MICROPY_PY_THREAD
#include "mpthreadport.h"
#include "py/mpthread.h"
#endif

//*****bsp****
#include "sleep.h"
#include "encoding.h"
#include "sysctl.h"
#include "plic.h"
//*****peripheral****
#include "pin_cfg.h"
#include "fpioa.h"
#include "gpio.h"
#include "timer.h"
#include "w25qxx.h"
#include "uarths.h"
#include "rtc.h"
#include "modmachine.h"
//*****freeRTOS****
#include "FreeRTOS.h"
#include "task.h"
//*******spiffs********
#include "vfs_spiffs.h"

#if MICROPY_VFS_SDCARD
#include "vfs_sdcard.h"
#include <filesystem.h>
#endif

extern handle_t mp_rtc_rtc0;

int MainTaskProc = 0;
static char *mpy_heap = NULL;
static mp_obj_t mp_task_pystack = NULL;

#if MICROPY_PY_THREAD
#if MICROPY_PY_THREAD_STATIC_TASK
static StaticTask_t mp_task_tcb;
#endif
#else
static StackType_t mp_task_stack[MP_TASK_STACK_LEN] __attribute__((aligned (8)));
#endif

const struct tm set_time =
{
    .tm_sec = 59,
    .tm_min = 22,
    .tm_hour = 17,
    .tm_mday = 8,
    .tm_mon = 11 - 1,
    .tm_year = 2018 - 1900,
    .tm_wday = 4,
    .tm_yday = -1,
    .tm_isdst = -1,
};

void do_str(const char *src, mp_parse_input_kind_t input_kind);

const char Banner[] = {"\n __  __              _____  __   __  _____   __     __ \n\
|  \\/  |     /\\     |_   _| \\ \\ / / |  __ \\  \\ \\   / /\n\
| \\  / |    /  \\      | |    \\ V /  | |__) |  \\ \\_/ / \n\
| |\\/| |   / /\\ \\     | |     > <   |  ___/    \\   /  \n\
| |  | |  / ____ \\   _| |_   / . \\  | |         | |   \n\
|_|  |_| /_/    \\_\\ |_____| /_/ \\_\\ |_|         |_|\n\
------------------------------------------------------\n\n"};

const char ver_info[] = {"\nMaixPy-FreeRTOS by LoBo v1.0.2\n\
------------------------------\n"};

static const char* TAG = "[MAIN]";


// ===================================
// MicroPython main task
// ===================================
static void mp_task(void *pvParameter)
{
    volatile void *stack_p = 0;
    volatile void *sp = (void *)((uint64_t)(&stack_p) & 0xFFFFFFFFFFFFFFF8);
    void *stack = (void *)(((uint64_t)pxTaskGetStackStart(NULL) & 0xFFFFFFFFFFFFFFF8)+16);

    mpy_heap = malloc(MICROPY_HEAP_SIZE+16);
    configASSERT(mpy_heap);
    #if MICROPY_ENABLE_PYSTACK
    mp_task_pystack = malloc(MICROPY_PYSTACK_SIZE);
    configASSERT(mp_task_pystack);
    #endif

    LOGD(TAG, "Main task start (proc=%d): heap at %p, pystack at %p, stack at %p (%p), free=%lu",
            (int)uxPortGetProcessorId(), (void *)mpy_heap, (void *)mp_task_pystack, pxTaskGetStackStart(NULL), sp, uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));

    mp_thread_preinit(stack+1024, sp-(stack+1024), mp_task_pystack, MICROPY_PYSTACK_SIZE);

    while (1) {
        // Initialize the stack pointer for the main thread
        mp_stack_set_top((void *)sp);
        mp_stack_set_limit(sp-(stack+1024));

        #if MICROPY_ENABLE_GC
        // Initialize MicroPython heap
        gc_init(mpy_heap, mpy_heap + MICROPY_HEAP_SIZE);
        #endif
        mp_init();
        mp_obj_list_init(mp_sys_path, 0);
        mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_));
        mp_obj_list_init(mp_sys_argv, 0) ;
        // Set gc threshold to 4/5 of the heap size
        MP_STATE_MEM(gc_alloc_threshold) = (MICROPY_HEAP_SIZE * 4 / 5) & 0xFFFFFFFFFFFFFFF8;

        // Initialize spiffs on internal Flash
        bool spiffs_ok = init_flash_spiffs();
        if (spiffs_ok) LOGD(TAG, "SPIFFS initialized");
        else LOGE(TAG, "SPIFFS initialization failed!");

        readline_init0();
        if (spiffs_ok) {
            pyexec_file("/flash/boot.py");
            /*
            // Check if 'main.py' exists and run it
            mp_obj_t args[2];
            args[0] = mp_obj_new_str("/flash/main.py", 14);
            args[1] = mp_obj_new_str("r", 1);
            // Open the file
            mp_obj_t ffd = mp_vfs_open(2, args, (mp_map_t*)&mp_const_empty_map);
            if (ffd) {
                mp_stream_close(ffd);
                pyexec_file("/flash/main.py");
            }
            */
        }

        // ---- Main REPL loop ----------------------------------------------
        for (;;) {
            if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
                if (pyexec_raw_repl() != 0) {
                    break;
                }
            }
            else {
                mp_printf(&mp_plat_print, ver_info);
                if (pyexec_friendly_repl() != 0) {
                    break;
                }
            }
        }
        // ------------------------------------------------------------------

        //ToDo: terminate all running threads ?!
        mp_deinit();
        mp_printf(&mp_plat_print, "PYB: soft reboot\n");
        mp_hal_delay_ms(10);

        //sysctl->soft_reset.soft_reset = 1;
    }
    vTaskDelete(NULL);
}

const fpioa_cfg_t flash_pins_cfg =
{
    .version = PIN_CFG_VERSION,
    .functions_count = 6,
    // SPI Flash pins
    .functions[0] = {30, FUNC_SPI1_SS0},
    .functions[1] = {32, FUNC_SPI1_SCLK},
    .functions[2] = {34, FUNC_SPI1_D0},
    .functions[3] = {43, FUNC_SPI1_D1},
    .functions[4] = {42, FUNC_SPI1_D2},
    .functions[5] = {41, FUNC_SPI1_D3},
};

//========
int main()
{
    int res;
    // Set dvp and spi pins to 1.8V
	sysctl_set_power_mode(SYSCTL_POWER_BANK6,SYSCTL_POWER_V18);
	sysctl_set_power_mode(SYSCTL_POWER_BANK7,SYSCTL_POWER_V18);

    #if !MICROPY_PY_THREAD
	LOGE("MAIXPY", "Threads must be enabled!");
    vTaskDelete(NULL);
    return 1;
    #endif

    // Initialize MPy HAL
	mp_hal_init();

	// Initialize RTC
	mp_rtc_rtc0 = io_open("/dev/rtc0");
    configASSERT(mp_rtc_rtc0);
    rtc_set_datetime(mp_rtc_rtc0, &set_time);
    #if MICROPY_VFS_SDCARD
    filesystem_rtc = mp_rtc_rtc0;
    #endif

    // Initialize SPIFlash
    fpioa_setup_pins(&flash_pins_cfg);
    handle_t flash_spi;
    uint8_t manuf_id, device_id;
    flash_spi = io_open("/dev/spi3");
    configASSERT(flash_spi);
    w25qxx_init(flash_spi, SPI_FF_STANDARD, 20000000);
    w25qxx_read_id(&manuf_id, &device_id);

    // Print some info
    mp_printf(&mp_plat_print, Banner);
    LOGD("[MAIXPY]", "Stack:  min: %lu", uxTaskGetStackHighWaterMark(NULL));
    LOGD("[MAIXPY]", " Pll0: freq: %d", sysctl_clock_get_freq(SYSCTL_CLOCK_PLL0));
    LOGD("[MAIXPY]", " Pll1: freq: %d", sysctl_clock_get_freq(SYSCTL_CLOCK_PLL1));
    LOGD("[MAIXPY]", " Pll2: freq: %d", sysctl_clock_get_freq(SYSCTL_CLOCK_PLL2));
    LOGD("[MAIXPY]", "  Cpu: freq: %lu", uxPortGetCPUClock());
    LOGD("[MAIXPY]", "Flash:   ID: [0x%02x:0x%02x]", manuf_id, device_id);

    // Run Micropython as FreeRTOS task
    #if MICROPY_PY_THREAD_STATIC_TASK
    MainTaskHandle = xTaskCreateStaticAtProcessor(
            MainTaskProc,           // processor
            mp_task,                // function entry
            "mp_task",              // task name
            MICROPY_TASK_STACK_LEN, // stack_deepth
            NULL,                   // function argument
            MICROPY_TASK_PRIORITY,  // task priority
            mp_task_stack,          // static task stack
            &mp_task_tcb );         // static task TCB
    #else
    res = xTaskCreateAtProcessor(
            MainTaskProc,           // processor
            mp_task,                // function entry
            "mp_task",              // task name
            MICROPY_TASK_STACK_LEN, // stack_deepth
            NULL,                   // function argument
            MICROPY_TASK_PRIORITY,  // task priority
            &MainTaskHandle);       // task handle
    #endif
    configASSERT((res == pdPASS));

    vTaskDelete(NULL);
}

//--------------------------------------------------------------
void do_str(const char *src, mp_parse_input_kind_t input_kind) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // uncaught exception
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}

//-----------------------------
void nlr_jump_fail(void *val) {
    mp_printf(&mp_plat_print, "SYSTEM HALTED\r\n");
    while (1);
}

//ToDo: Is this necessary?
#if !MICROPY_DEBUG_PRINTERS
// With MICROPY_DEBUG_PRINTERS disabled DEBUG_printf is not defined but it
// is still needed by esp-open-lwip for debugging output, so define it here.
#include <stdarg.h>
int mp_vprintf(const mp_print_t *print, const char *fmt, va_list args);
int DEBUG_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int ret = mp_vprintf(MICROPY_DEBUG_PRINTER, fmt, ap);
    va_end(ap);
    return ret;
}
#endif
