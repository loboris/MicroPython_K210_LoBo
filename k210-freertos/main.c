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

#include "mpconfigport.h"
/*****std lib****/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <devices.h>
#include <stdarg.h>
#include "syslog.h"

//*****mpy****
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/stackctrl.h"
#include "py/mpstate.h"
#include "py/nlr.h"
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
//*******flash fs********
#include "vfs_spiffs.h"
#include "littleflash.h"

#if MICROPY_VFS_SDCARD
#include "vfs_sdcard.h"
#include <filesystem.h>
#endif

// === FreeRTOS heap ===
size_t configTOTAL_HEAP_SIZE = FREE_RTOS_TOTAL_HEAP_SIZE;
// This is used by 'heap_4.c'
uint8_t __attribute__((aligned(8))) ucHeap[FREE_RTOS_TOTAL_HEAP_SIZE];

static void *sys_rambuf;

extern handle_t mp_rtc_rtc0;

static uint8_t *mp_heap = NULL;
static uint8_t *mp_task_pystack = NULL;

static uint8_t *mp_heap2 = NULL;
static uint8_t *mp_task_pystack2 = NULL;


const char Banner[] = {"\n __  __              _____  __   __  _____   __     __ \n\
|  \\/  |     /\\     |_   _| \\ \\ / / |  __ \\  \\ \\   / /\n\
| \\  / |    /  \\      | |    \\ V /  | |__) |  \\ \\_/ / \n\
| |\\/| |   / /\\ \\     | |     > <   |  ___/    \\   /  \n\
| |  | |  / ____ \\   _| |_   / . \\  | |         | |   \n\
|_|  |_| /_/    \\_\\ |_____| /_/ \\_\\ |_|         |_|\n\
------------------------------------------------------\n"};

static const char *ver_info1 = "\nMaixPy-FreeRTOS by LoBo v"MICROPY_PY_LOBO_VERSION" (two MPY tasks)\n------------------------------------------------\n";
static const char *ver_info2 = "\nMaixPy-FreeRTOS by LoBo v"MICROPY_PY_LOBO_VERSION"\n--------------------------------\n";

static const char* TAG = "[MAIN]";
static const char* TAG_MAIN = "[MAIXPY]";
static const char* DEFAULT_BOOT_PY = "# This file is executed on every boot\nimport sys\nsys.path().append('/flash/lib')\n";

static bool task0_started = false;
static bool flash_fs_ok = false;
static bool exec_boot_py = true;
static bool exec_main_py = true;
static bool use_default_config = false;

//----------------------------
size_t _check_remaining_heap()
{
    uint32_t hp_size = 1*1024*1024;
    uint8_t * hp_testbuf = NULL;
    while (hp_size > 1024) {
        hp_testbuf = malloc(hp_size);
        if (hp_testbuf) {
            free(hp_testbuf);
            break;
        }
        hp_size -= 1024;
    }
    return hp_size;
}

//-------------------------
static void check_boot_py()
{
    if (mp_vfs_import_stat("/flash/boot.py") != MP_IMPORT_STAT_FILE) {
        // No 'boot.py' found, create the default one
        mp_obj_t args[2];
        args[0] = mp_obj_new_str("/flash/boot.py", 14);
        args[1] = mp_obj_new_str("wb", 2);
        // Open the file for writing
        mp_obj_t ffd = mp_vfs_open(2, args, (mp_map_t*)&mp_const_empty_map);
        if (!ffd) {
            LOGW(TAG, "Error creating 'boot.py'");
            return;
        }
        mp_stream_posix_write((void *)ffd, DEFAULT_BOOT_PY, strlen(DEFAULT_BOOT_PY));
        mp_stream_close(ffd);
        LOGD(TAG, "'boot.py' created");
    }
}

// ============================================
// MicroPython instance #1 FreeRTOS task (REPL)
// =========================================
static void mp_task_proc0(void *pvParameter)
{
    // Create main MicroPython thread for MicroPython instance #1
    mp_thread_preinit((void *)(pxTaskGetStackStart(NULL)+MICROPY_TASK_STACK_RESERVED),
            (uint32_t)(pxTaskGetStackEnd(NULL) - pxTaskGetStackStart(NULL) - MICROPY_TASK_STACK_RESERVED),
            (void *)mp_task_pystack, mpy_config.config.pystack_size, MAIN_TASK_PROC);

    while (1) {
        // ** Initialize the stack pointer for the main thread
        mp_stack_set_top((void *)pxTaskGetStackEnd(NULL));
        mp_stack_set_limit((size_t)(pxTaskGetStackEnd(NULL) - pxTaskGetStackStart(NULL) - MICROPY_TASK_STACK_RESERVED));

        #if MICROPY_ENABLE_GC
        // ** Initialize MicroPython heap
        gc_init(mp_heap, mp_heap + mpy_config.config.heap_size1);
        #endif
        mp_init();
        mp_obj_list_init((mp_obj_list_t *)mp_sys_path, 0);
        mp_obj_list_append((mp_obj_t)mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_));
        mp_obj_list_init((mp_obj_list_t *)mp_sys_argv, 0) ;
        // Set gc threshold to 4/5 of the heap size
        MP_STATE_MEM(gc_alloc_threshold) = (mpy_config.config.heap_size1 * 4 / 5) & 0xFFFFFFFFFFFFFFF8;

        // Initialize file system on internal Flash
        flash_fs_ok = init_flash_filesystem();
        if (!flash_fs_ok) LOGE(TAG, "FLASH File system initialization failed!");

        readline_init0();

        if (flash_fs_ok) {
            // 'boot.py' should always exist
            check_boot_py();
            if ((exec_boot_py) && (mp_vfs_import_stat("/flash/boot.py") == MP_IMPORT_STAT_FILE)) pyexec_file("/flash/boot.py");

            // Execute 'main.py' if it exists
            if (mp_vfs_import_stat("/flash/main.py") == MP_IMPORT_STAT_FILE) {
                pyexec_file("/flash/main.py");
            }
        }

        // Print MicroPython banners
        mp_printf(&mp_plat_print, "%s%s%s%s%s", term_color(BLUE), Banner, term_color(BROWN), (mpy_config.config.use_two_main_tasks) ? ver_info1 : ver_info2, term_color(DEFAULT));
        task0_started = true;

        // ==== Main REPL loop =========================================
        for (;;) {
            if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
                int raw_repl_res = pyexec_raw_repl();
                if (raw_repl_res != 0) {
                    if (raw_repl_res == PYEXEC_FORCED_EXIT) {
                        mp_printf(&mp_plat_print, "MPY: soft reboot\n");
                        mp_hal_delay_ms(10);
                        continue;
                    }
                    break;
                }
            }
            else {
                if (pyexec_friendly_repl() != 0) {
                    break;
                }
            }
        }
        // =============================================================

        //ToDo: terminate all running threads ?!
        mp_deinit();
        mp_printf(&mp_plat_print, "MPY: soft reboot\n");
        mp_hal_delay_ms(10);
    }
    vTaskDelete(NULL);
}

// =========================================
// MicroPython instance #2 FreeRTOS task
// =========================================
static void mp_task_proc1(void *pvParameter)
{
    // Create main MicroPython thread for MicroPython instance #2
    mp_thread_preinit((void *)(pxTaskGetStackStart(NULL)+MICROPY_TASK_STACK_RESERVED),
            (uint32_t)(pxTaskGetStackEnd(NULL) - pxTaskGetStackStart(NULL) - MICROPY_TASK_STACK_RESERVED),
            (void *)mp_task_pystack2, mpy_config.config.pystack_size, MAIN_TASK_PROC ^ 1);

    while (1) {
        // ** Initialize the stack pointer for the main thread
        mp_stack_set_top((void *)pxTaskGetStackEnd(NULL));
        mp_stack_set_limit((size_t)(pxTaskGetStackEnd(NULL) - pxTaskGetStackStart(NULL) - MICROPY_TASK_STACK_RESERVED));

        #if MICROPY_ENABLE_GC
        // ** Initialize MicroPython heap
        gc_init(mp_heap2, mp_heap2 + mpy_config.config.heap_size2);
        #endif
        mp_init();
        mp_obj_list_init(mp_sys_path, 0);
        mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_));
        mp_obj_list_init(mp_sys_argv, 0) ;
        // Set gc threshold to 4/5 of the heap size
        MP_STATE_MEM(gc_alloc_threshold) = (mpy_config.config.heap_size2 * 4 / 5) & 0xFFFFFFFFFFFFFFF8;

        thread_msg_t msg;

        vTaskDelay(5);
        if (flash_fs_ok) {
            // set mount point for flash file system
            mp_vfs_mount_t *vfs = m_new_obj_maybe(mp_vfs_mount_t);
            if (vfs) {
                vfs->str = "/flash";
                vfs->len = 6;
                vfs->obj = MP_OBJ_FROM_PTR(vfs_flashfs);
                vfs->next = NULL;
                mp_vfs_mount_t *vfs_sys = MP_STATE_VM(vfs_mount_table);
                if (vfs_sys == NULL) {
                    MP_STATE_VM(vfs_mount_table) = vfs;
                }
                else {
                    vfs_sys->next = vfs;
                }
            }

            // Execute 'boot2.py' if it exists
            if ((exec_boot_py) && (mp_vfs_import_stat("/flash/boot2.py") == MP_IMPORT_STAT_FILE)) {
                pyexec_file("/flash/boot2.py");
            }
        }

        vTaskDelay(5);
        // ==== Main 2nd MicroPPython instance loop ========
        while (1) {
            // === Wait for command from main (or other) MicroPython task ===
            MP_THREAD_GIL_EXIT();
            if (xQueueReceive(thread_entry2.threadQueue, &msg, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
                MP_THREAD_GIL_ENTER();
                vTaskDelay(2 / portTICK_PERIOD_MS);
                // Got a message
                if ((msg.type == THREAD_MSG_TYPE_INTEGER) || (msg.type == THREAD_MSG_TYPE_STRING)) {
                    if (ipc_callback_1) {
                        mp_obj_t tuple[4];
                        tuple[0] = mp_obj_new_int(msg.type);
                        tuple[1] = mp_obj_new_int((uintptr_t)msg.sender_id);
                        tuple[2] = mp_obj_new_int(msg.intdata);
                        if (msg.type == THREAD_MSG_TYPE_STRING) tuple[3] = mp_obj_new_str((const char*)msg.strdata, msg.strlen);
                        else tuple[3] = mp_const_none;
                        mp_sched_schedule(ipc_callback_1, mp_obj_new_tuple(4, tuple));
                    }
                    if (msg.type == THREAD_MSG_TYPE_INTEGER) {
                        // ** Integer type command **
                        if (msg.intdata == THREAD_IPC_TYPE_TERMINATE) break;
                    }
                    else if (msg.type == THREAD_MSG_TYPE_STRING) {
                        // ** String type command **
                        if (msg.strdata != NULL) {
                            if (msg.intdata == THREAD_IPC_TYPE_EXECUTE) {
                                // Execute string
                                xSemaphoreTake(inter_proc_mutex, portMAX_DELAY);
                                task_ipc.busy = true;
                                xSemaphoreGive(inter_proc_mutex);

                                // ==== execute python code from string ===============================================
                                mp_obj_t res = exec_code_from_str((const char *)msg.strdata);
                                if (res != mp_const_true) {
                                    // uncaught exception
                                    mp_printf(&mp_plat_print, "Error in [%s]\r\n", (const char *)msg.strdata);
                                    mp_obj_print_exception(&mp_plat_print, res);
                                }
                                // ====================================================================================

                                xSemaphoreTake(inter_proc_mutex, portMAX_DELAY);
                                task_ipc.busy = false;
                                xSemaphoreGive(inter_proc_mutex);
                            }
                            vPortFree(msg.strdata);
                        }
                    }
                }
            }
        }
        // ================================================

        //ToDo: terminate all running threads ?!
        mp_deinit();
        mp_printf(&mp_plat_print, "MPY2: soft reboot\n");
        mp_hal_delay_ms(10);
    }
    vTaskDelete(NULL);
}

//========
int main()
{
    // ==== Allocate ram buffer ====
    sys_rambuf = malloc(MYCROPY_SYS_RAMBUF_SIZE);
    if (sys_rambuf) sys_rambuf_ptr = (uintptr_t)sys_rambuf;

    // ==== Basic initialization ====
    sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_MAX_OUTPUT_FREQ);
    sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_MAX_OUTPUT_FREQ);
    sysctl_pll_set_freq(SYSCTL_PLL2, PLL2_MAX_OUTPUT_FREQ);
    sysctl_clock_set_threshold(SYSCTL_THRESHOLD_AI, 0);
    // Set SPI3 clock to PLL0 / 2
    sysctl_clock_set_clock_select(SYSCTL_CLOCK_SELECT_SPI3, 0);
    sysctl_clock_set_threshold(SYSCTL_THRESHOLD_SPI3, 0);
    sysctl_clock_set_clock_select(SYSCTL_CLOCK_SELECT_SPI3, 1);

    sys_us_counter_cpu = read_csr64(mcycle);
    uarths_baudrate = uarths_init(MICRO_PY_DEFAULT_BAUDRATE);

    // ==== Get reset status ====
    system_status = sysctl->reset_status.soft_reset_sts |
            (sysctl->reset_status.wdt0_reset_sts << 1) |
            (sysctl->reset_status.wdt1_reset_sts << 2) |
            (sysctl->reset_status.pin_reset_sts << 3);
    sysctl->reset_status.reset_sts_clr = 1;

    // ==== Set dvp and lcd spi pins to 1.8V ====
	sysctl_set_power_mode(SYSCTL_POWER_BANK6,SYSCTL_POWER_V18);
	sysctl_set_power_mode(SYSCTL_POWER_BANK7,SYSCTL_POWER_V18);

    user_log_level = LOG_WARN;
    user_log_color = 0;
    printf("\r\n");

    #if !MICROPY_PY_THREAD
	LOGE(TAG_MAIN, "Threads must be enabled!");
    vTaskDelete(NULL);
    return 1;
    #endif

    // ==== Initialize the SPI Flash hardware ====
    flash_spi = io_open("/dev/spi3");
    configASSERT(flash_spi);

    w25qxx_spi_check = false;
    uint32_t res = w25qxx_init(flash_spi, SPI_FF_QUAD, WQ25QXX_MAX_SPEED);
    configASSERT(res);
    vTaskDelay(2);

    // === Read MicroPython configuration from flash ===
    bool cfg_loaded = false;
    if (!mpy_read_config()) {
        vTaskDelay(2);
        mpy_config_set_default();
    }
    else cfg_loaded = true;
    configASSERT(mpy_config_crc(false));

    user_log_level = mpy_config.config.log_level;
    user_log_color = mpy_config.config.log_color;
    vTaskDelay(2);
    if (cfg_loaded) LOGM(TAG_MAIN, "Configuration loaded from flash");

    LOGQ(TAG_MAIN, "PLL0=%u, PLL1=%u, PLL2=%u, SPI3clk=%u",
            sysctl_clock_get_freq(SYSCTL_CLOCK_PLL0), sysctl_clock_get_freq(SYSCTL_CLOCK_PLL1),
            sysctl_clock_get_freq(SYSCTL_CLOCK_PLL2), sysctl_clock_get_freq(SYSCTL_CLOCK_SPI3));
    if (sys_rambuf) LOGQ(TAG_MAIN, "RAM buffer of %d bytes allocated at %p", MYCROPY_SYS_RAMBUF_SIZE, sys_rambuf);

    // ==== Initialize MicroPython HAL ====
	mp_hal_init();

	// === Initialize RTC ===
	mp_rtc_rtc0 = io_open("/dev/rtc0");
    configASSERT(mp_rtc_rtc0);

    struct tm default_time =
    {
        .tm_sec = 0,
        .tm_min = 0,
        .tm_hour = 12,
        .tm_mday = 3,
        .tm_mon = 7 - 1,
        .tm_year = 2019 - 1900,
        .tm_wday = 3,
        .tm_yday = -1,
        .tm_isdst = -1,
    };
    _set_sys_time(&default_time, 0);

    #if MICROPY_VFS_SDCARD
    filesystem_rtc = mp_rtc_rtc0;
    #endif

    sysctl->peri.jtag_clk_bypass = 1;

    // ==== Initialize GPIOHS for boot menu pin check====
    if (mpy_config.config.boot_menu_pin) {
        machine_init_gpiohs();
        configASSERT(gpiohs_handle);

        // ==== Check boot menu pin ====
        handle_t gio = io_open("/dev/gpio0");
        int  bpin_gpiohs = 20;
        configASSERT(gio);
        fpioa_set_function(mpy_config.config.boot_menu_pin, FUNC_GPIOHS0 + bpin_gpiohs);
        gpio_set_drive_mode(gio, bpin_gpiohs, GPIO_DM_INPUT_PULL_UP);
        vTaskDelay(1);
        uint64_t timeout;
        int bpin_val = gpio_get_pin_value(gio, bpin_gpiohs);
        if (bpin_val  == 0) {
            char key = 0;
            char sel_f = ' ';
            char sel_b = ' ';
            char sel_m = ' ';
            char sel_d = ' ';
            // boot menu pin is low, display the menu
        start_menu:
            timeout = mp_hal_ticks_ms() + 10000;
            mp_printf(&mp_plat_print, "%s\nBoot menu, select an option\n%s", term_color(BROWN), term_color(DEFAULT));
            mp_printf(&mp_plat_print, "%c%sF%s - force format File system\n", sel_f, term_color(BROWN), term_color(DEFAULT));
            mp_printf(&mp_plat_print, "%c%sB%s - do not execute 'boot.py'\n", sel_b, term_color(BROWN), term_color(DEFAULT));
            mp_printf(&mp_plat_print, "%c%sM%s - do not execute 'main.py'\n", sel_m, term_color(BROWN), term_color(DEFAULT));
            mp_printf(&mp_plat_print, "%c%sD%s - use default configuration\n", sel_d, term_color(BROWN), term_color(DEFAULT));
            mp_printf(&mp_plat_print, "%s Q%s - exit the menu\n", term_color(BROWN), term_color(DEFAULT));
            while (1) {
                if (mp_hal_ticks_ms() > timeout) break;
                key = wait_key("", 500);
                bpin_val = gpio_get_pin_value(gio, bpin_gpiohs);
                if (bpin_val == 1) break;
                if ((key == 'q') || (key == 'Q')) break;
                else if ((key == 'm') || (key == 'M')) {
                    exec_main_py = false;
                    sel_m = '*';
                    goto start_menu;
                }
                else if ((key == 'b') || (key == 'B')) {
                    exec_boot_py = false;
                    sel_b = '*';
                    goto start_menu;
                }
                else if ((key == 'f') || (key == 'F')) {
                    force_erase_fs_flash = true;
                    sel_f = '*';
                    goto start_menu;
                }
                else if ((key == 'd') || (key == 'D')) {
                    use_default_config = true;
                    sel_d = '*';
                    goto start_menu;
                }
            }
        }
    }

    if (use_default_config) {
        vTaskDelay(2);
        mpy_config_set_default();
        vTaskDelay(2);
        sysctl->soft_reset.soft_reset = 1;
        while (1) {
            ;
        }
    }

    // ======================================
    // ==== Allocate MicroPython heap(s) ====
    // ======================================
    mp_heap = pvPortMalloc(mpy_config.config.heap_size1 + 16);
    configASSERT(mp_heap);
    if (mpy_config.config.pystack_enabled) {
        mp_task_pystack = pvPortMalloc(mpy_config.config.pystack_size);
        configASSERT(mp_task_pystack);
    }

    if (mpy_config.config.use_two_main_tasks) {
        mp_heap2 = pvPortMalloc(mpy_config.config.heap_size2 + 16);
        configASSERT(mp_heap2);
        if (mpy_config.config.pystack_enabled) {
            mp_task_pystack2 = pvPortMalloc(mpy_config.config.pystack_size);
            configASSERT(mp_task_pystack2);
        }
        LOGM(TAG_MAIN, "Heaps: FreeRTOS=%lu KB (%lu KB free), MPy_1=%u KB, MPy_2=%u KB, other=%lu KB",
                FREE_RTOS_TOTAL_HEAP_SIZE/1024, xPortGetFreeHeapSize()/1024, mpy_config.config.heap_size1/1024, mpy_config.config.heap_size2/1024, _check_remaining_heap()/1024);
    }
    else {
        LOGM(TAG_MAIN, "Heaps: FreeRTOS=%lu KB (%lu KB free), MPy=%u KB, other=%lu KB",
                FREE_RTOS_TOTAL_HEAP_SIZE/1024, xPortGetFreeHeapSize()/1024, mpy_config.config.heap_size1/1024, _check_remaining_heap()/1024);
    }

    // ======================================================
    // ==== Run MicroPython instance(s) as FreeRTOS task ====
    // ======================================================
    res = xTaskCreateAtProcessor(
            MAIN_TASK_PROC,                         // MPy instance #1 processor
            mp_task_proc0,                          // function entry
            "main_mp_task",                         // task name
            mpy_config.config.main_task_stack_size, // stack_deepth
            NULL,                                   // function argument
            MICROPY_TASK_PRIORITY,                  // task priority
            &MainTaskHandle);                       // task handle
    configASSERT((res == pdPASS));

    if (mpy_config.config.use_two_main_tasks) {
        // === wait for the 1st task to start and start the 2nd MicroPython task ===
        while (!task0_started) {
            vTaskDelay(10);
        }
        vTaskDelay(50);
        res = xTaskCreateAtProcessor(
                MAIN_TASK_PROC ^ 1,                     // MPy instance #2 processor
                mp_task_proc1,                          // function entry
                "main_mp_task2",                        // task name
                mpy_config.config.main_task_stack_size, // stack_deepth
                NULL,                                   // function argument
                MICROPY_TASK_PRIORITY,                  // task priority
                &MainTaskHandle2);                      // task handle
        configASSERT((res == pdPASS));
    }

    // The 'main' function runs as FreeRTOS task,
    // which is no longer needed, we can delete it now
    vTaskDelay(100);
    vTaskDelete(NULL);
}


// This is called by nlr_jump if no nlr buf has been pushed.
// It must not return, but rather should bail out with a fatal error.
//---------------------------
void nlr_jump_fail(void *val)
{
    //taskENTER_CRITICAL();
    printk("\r\n");
    LOGe("[RESET]", "nlr_jump FAILED (proc=%lu)\r\n", xTaskGetProcessor(NULL));
    mp_hal_usdelay(5000);
    mp_hal_wtd1_enable(true, 50);

    // This function does not return.
    //sysctl->soft_reset.soft_reset = 1;
    while (1) {
        ;
    }
}


/*

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
*/
