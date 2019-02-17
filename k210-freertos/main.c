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
handle_t flash_spi;

int MainTaskProc = 0;
static char *mpy_heap[2] = { NULL, NULL };
static mp_obj_t mp_task_pystack[2] = { NULL, NULL };

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

const char ver_info[] = {LOG_BOLD(LOG_COLOR_BROWN)"\nMaixPy-FreeRTOS by LoBo v1.0.5\n------------------------------\n"LOG_RESET_COLOR};

static const char* TAG = "[MAIN]";

// ===================================
// MicroPython main task (REPL)
// ===================================
static void mp_task(void *pvParameter)
{
    volatile void *stack_p = 0;
    volatile void *sp = (void *)((uint64_t)(&stack_p) & 0xFFFFFFFFFFFFFFF8);
    void *stack = (void *)(((uint64_t)pxTaskGetStackStart(NULL) & 0xFFFFFFFFFFFFFFF8)+16);

    int task_proc = (int)uxPortGetProcessorId();
    mpy_heap[task_proc] = malloc(MICROPY_HEAP_SIZE+16);
    configASSERT(mpy_heap[task_proc]);
    #if MICROPY_ENABLE_PYSTACK
    mp_task_pystack[task_proc] = malloc(MICROPY_PYSTACK_SIZE);
    configASSERT(mp_task_pystack[task_proc]);
    #endif

    LOGM(TAG, "Main task start (proc=%d): heap at %p, pystack at %p, stack at %p (%p), free=%lu",
            (int)uxPortGetProcessorId(), (void *)mpy_heap[task_proc], (void *)mp_task_pystack[task_proc], pxTaskGetStackStart(NULL),
            sp, (uint32_t)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));

    #if MICROPY_USE_TWO_MAIN_TASKS
    if (task_proc == MainTaskProc) mp_thread_preinit(stack+1024, sp-(stack+1024), mp_task_pystack[task_proc], MICROPY_PYSTACK_SIZE);
    #else
    mp_thread_preinit(stack+1024, sp-(stack+1024), mp_task_pystack[task_proc], MICROPY_PYSTACK_SIZE);
    #endif

    while (1) {
        // Initialize the stack pointer for the main thread
        mp_stack_set_top((void *)sp);
        mp_stack_set_limit(sp-(stack+1024));

        #if MICROPY_ENABLE_GC
        // Initialize MicroPython heap
        gc_init(mpy_heap[task_proc], mpy_heap[task_proc] + MICROPY_HEAP_SIZE);
        #endif
        mp_init();
        mp_obj_list_init(mp_sys_path, 0);
        mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_));
        mp_obj_list_init(mp_sys_argv, 0) ;
        // Set gc threshold to 4/5 of the heap size
        MP_STATE_MEM(gc_alloc_threshold) = (MICROPY_HEAP_SIZE * 4 / 5) & 0xFFFFFFFFFFFFFFF8;

        #if MICROPY_USE_TWO_MAIN_TASKS
        if (task_proc == MainTaskProc) {
        #endif
            // Initialize spiffs on internal Flash
            bool spiffs_ok = init_flash_spiffs();
            if (spiffs_ok) LOGD(TAG, "SPIFFS initialized");
            else LOGE(TAG, "SPIFFS initialization failed!");

            readline_init0();

            if (spiffs_ok) {
                // 'boot.py' should always exist, so execute it
                pyexec_file("/flash/boot.py");

                // Execute 'main.py' if exists
                spiffs_stat fno;
                if (SPIFFS_stat(&spiffs_user_mount_handle.fs, "main.py", &fno) == SPIFFS_OK) {
                    pyexec_file("/flash/main.py");
                }
            }

            // ---- Main REPL loop ----------------------------------------------
            for (;;) {
                if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
                    int raw_repl_res = pyexec_raw_repl();
                    if (raw_repl_res != 0) {
                        if (raw_repl_res == PYEXEC_FORCED_EXIT) {
                            mp_printf(&mp_plat_print, "PYB: soft reboot\n");
                            mp_hal_delay_ms(10);
                            continue;
                        }
                        break;
                    }
                }
                else {
                    mp_printf(&mp_plat_print, "%s%s", Banner, ver_info);
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
        #if MICROPY_USE_TWO_MAIN_TASKS
        }
        else {
            uint64_t notify_val = 0;
            int notify_res = 0;
            while (1) {
                //notify_res = xTaskNotifyWait(0, ULONG_MAX, &notify_val, 1000 / portTICK_RATE_MS);
                notify_res = xSemaphoreTake(inter_proc_semaphore, 10000 / portTICK_PERIOD_MS);
                if (notify_res != pdPASS) continue;

                xSemaphoreTake(inter_proc_mutex, portMAX_DELAY);
                LOGM("[MPy@1]", "Got notification: %lu", ipc_request);
                if (ipc_request == 0) break; // Terminate task requested
                if (ipc_request == 1) {
                    if (ipc_cmd_buff) {
                        //int res = pyexec_string((const char *)ipc_cmd_buff);
                        LOGM("[MPy@1]", "Executing command [%s]", ipc_cmd_buff);
                        vTaskDelay(5000 / portTICK_PERIOD_MS);
                        mp_hal_stdout_tx_strn("Here is the result", 18);
                        free(ipc_cmd_buff);
                        ipc_cmd_buff_size = 0;
                    }
                    else {
                        LOGW("[MPy@1]", "Cmd buffer is NULL");
                    }
                }
                xSemaphoreGive(inter_proc_mutex);
            }
        }
        #endif
    }
    vTaskDelete(NULL);
}

//========
int main()
{
    user_log_level = CONFIG_LOG_LEVEL;
    log_divisor = (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);

    int res;
    // Set dvp and spi pins to 1.8V
	sysctl_set_power_mode(SYSCTL_POWER_BANK6,SYSCTL_POWER_V18);
	sysctl_set_power_mode(SYSCTL_POWER_BANK7,SYSCTL_POWER_V18);

    #if !MICROPY_PY_THREAD
	LOGE("MAIXPY", "Threads must be enabled!");
    vTaskDelete(NULL);
    return 1;
    #endif

    // Initialize MicroPython HAL
	mp_hal_init();

	// Initialize RTC
	mp_rtc_rtc0 = io_open("/dev/rtc0");
    configASSERT(mp_rtc_rtc0);
    rtc_set_datetime(mp_rtc_rtc0, &set_time);
    #if MICROPY_VFS_SDCARD
    filesystem_rtc = mp_rtc_rtc0;
    #endif

    sysctl_t sysctl;
    sysctl.peri.jtag_clk_bypass = 1;

    // Initialize SPIFlash
    sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI3, 2);
    sysctl_clock_set_clock_select(SYSCTL_CLOCK_SELECT_SPI3, 1);

    flash_spi = io_open("/dev/spi3");
    configASSERT(flash_spi);
    w25qxx_spi_check = true;
    if (w25qxx_init(flash_spi, SPI_FF_QUAD, SPI_DEFAULT_CLOCK) == 0) {
        LOGE("MAIXPY", "Flash initialization error");
        vTaskDelete(NULL);
        return 1;
    }

    // Run MicroPython as FreeRTOS task
    #if MICROPY_USE_TWO_MAIN_TASKS
    res = xTaskCreateAtProcessor(
            MainTaskProc ^ 1,       // processor
            mp_task,                // function entry
            "mp_task2",             // task name
            MICROPY_TASK_STACK_LEN, // stack_deepth
            NULL,                   // function argument
            MICROPY_TASK_PRIORITY,  // task priority
            &MainTaskHandle2);      // task handle
    configASSERT((res == pdPASS));
    vTaskDelay(500);
    #endif

    res = xTaskCreateAtProcessor(
            MainTaskProc,           // processor
            mp_task,                // function entry
            "mp_task",              // task name
            MICROPY_TASK_STACK_LEN, // stack_deepth
            NULL,                   // function argument
            MICROPY_TASK_PRIORITY,  // task priority
            &MainTaskHandle);       // task handle
    configASSERT((res == pdPASS));

    vTaskDelay(100);
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
