/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// LoBo: various changes added
#ifndef _SYSLOG_H
#define _SYSLOG_H

#include <stdint.h>
#include <stdio.h>
#include <printf.h>
#include <encoding.h>
#include "sysctl.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Logging library
 *
 * Log library has two ways of managing log verbosity: compile time, set via
 * menuconfig
 *
 * At compile time, filtering is done using CONFIG_LOG_DEFAULT_LEVEL macro, set via
 * menuconfig. All logging statments for levels higher than CONFIG_LOG_DEFAULT_LEVEL
 * will be removed by the preprocessor.
 *
 *
 * How to use this library:
 *
 * In each C file which uses logging functionality, define TAG variable like this:
 *
 *      static const char *TAG = "MODULE_NAME";
 *
 * then use one of logging macros to produce output, e.g:
 *
 *      LOGW(TAG, "Interrupt error %d", error);
 *
 * Several macros are available for different verbosity levels:
 *
 *      LOGE - error
 *      LOGW - warning
 *      LOGI - info
 *      LOGD - debug
 *      LOGV - verbose
 *
 * To override default verbosity level at file or component scope, define LOG_LEVEL macro.
 * At file scope, define it before including esp_log.h, e.g.:
 *
 *      #define LOG_LEVEL LOG_VERBOSE
 *      #include "dxx_log.h"
 *
 * At component scope, define it in component makefile:
 *
 *      CFLAGS += -D LOG_LEVEL=LOG_DEBUG
 *
 *
 */

/* clang-format off */
enum kendryte_log_level_e
{
    LOG_NONE,       /*!< No log output */
    LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
    LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
    LOG_INFO,       /*!< Information messages which describe normal flow of events */
    LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
};
/* clang-format on */

/* clang-format off */
#if CONFIG_LOG_COLORS
#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_BROWN   "33"
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_PURPLE  "35"
#define LOG_COLOR_CYAN    "36"
//#define LOG_COLOR(COLOR)  "\033[0;" COLOR "m"
//#define LOG_BOLD(COLOR)   "\033[1;" COLOR "m"
//#define LOG_RESET_COLOR   "\033[0m"
#define LOG_COLOR(COLOR)  "\033[" COLOR "m"
#define LOG_BOLD(COLOR)   "\033[" COLOR ";1m"
#define LOG_RESET_COLOR   "\033[0m"

#define LOG_COLOR_E       LOG_COLOR(LOG_COLOR_RED)
#define LOG_COLOR_e       LOG_BOLD(LOG_COLOR_RED)
#define LOG_COLOR_W       LOG_COLOR(LOG_COLOR_BROWN)
#define LOG_COLOR_I       LOG_COLOR(LOG_COLOR_GREEN)
#define LOG_COLOR_D
#define LOG_COLOR_V
#define LOG_COLOR_M       LOG_COLOR(LOG_COLOR_CYAN)
#define LOG_COLOR_Q       LOG_BOLD(LOG_COLOR_PURPLE)
#define LOG_COLOR_Y       LOG_BOLD(LOG_COLOR_BROWN)
#else /* CONFIG_LOG_COLORS */
#define LOG_COLOR_E
#define LOG_COLOR_e
#define LOG_COLOR_W
#define LOG_COLOR_I
#define LOG_COLOR_D
#define LOG_COLOR_V
#define LOG_COLOR_M
#define LOG_COLOR_Q
#define LOG_COLOR_Y
#define LOG_RESET_COLOR
#endif /* CONFIG_LOG_COLORS */
/* clang-format on */

/*
 * LoBo:
 *   print log time as micro seconds
 *   make syslog level configurable during run time
 *   make syslog colors configurable during run time
 *   add syslog mutex
 *   enable filtering of non-printable characters
 */
//------------------------------------
extern uint32_t user_log_level;
extern uint32_t user_log_color;
extern uint64_t mp_hal_ticks_us(void);
extern QueueHandle_t syslog_mutex;
extern int kprint_filter_nonprint;
extern char kprint_nonprint_char;
extern uint8_t kprint_cr_lf;
//------------------------------------

#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter #letter " (%lu) %s: " format LOG_RESET_COLOR "\r\n"
#define LOG_FORMAT_NC(letter, format)  #letter " (%lu) %s: " format "\r\n"

#ifdef LOG_LEVEL
#undef CONFIG_LOG_LEVEL
#define CONFIG_LOG_LEVEL LOG_LEVEL
#endif

#ifdef LOG_KERNEL
#define LOG_PRINTF printk
#else
#define LOG_PRINTF printf
#endif

#ifdef CONFIG_LOG_ENABLE
#define LOGE(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   {if (user_log_color) LOG_PRINTF(LOG_FORMAT(E, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(E, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGe(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   {if (user_log_color) LOG_PRINTF(LOG_FORMAT(e, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(e, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGW(tag, format, ...)  do {if (user_log_level >= LOG_WARN)    {if (user_log_color) LOG_PRINTF(LOG_FORMAT(W, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(W, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGI(tag, format, ...)  do {if (user_log_level >= LOG_INFO)    {if (user_log_color) LOG_PRINTF(LOG_FORMAT(I, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(I, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGD(tag, format, ...)  do {if (user_log_level >= LOG_DEBUG)   {if (user_log_color) LOG_PRINTF(LOG_FORMAT(D, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(D, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGV(tag, format, ...)  do {if (user_log_level >= LOG_VERBOSE) {if (user_log_color) LOG_PRINTF(LOG_FORMAT(V, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(V, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGM(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   {if (user_log_color) LOG_PRINTF(LOG_FORMAT(M, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(M, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGQ(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   {if (user_log_color) LOG_PRINTF(LOG_FORMAT(Q, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(Q, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
#define LOGY(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   {if (user_log_color) LOG_PRINTF(LOG_FORMAT(Y, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); else LOG_PRINTF(LOG_FORMAT_NC(Y, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__);} } while (0)
/*
#define LOGE(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   LOG_PRINTF(LOG_FORMAT(E, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGe(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   LOG_PRINTF(LOG_FORMAT(e, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGW(tag, format, ...)  do {if (user_log_level >= LOG_WARN)    LOG_PRINTF(LOG_FORMAT(W, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGI(tag, format, ...)  do {if (user_log_level >= LOG_INFO)    LOG_PRINTF(LOG_FORMAT(I, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGD(tag, format, ...)  do {if (user_log_level >= LOG_DEBUG)   LOG_PRINTF(LOG_FORMAT(D, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGV(tag, format, ...)  do {if (user_log_level >= LOG_VERBOSE) LOG_PRINTF(LOG_FORMAT(V, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGM(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   LOG_PRINTF(LOG_FORMAT(M, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGQ(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   LOG_PRINTF(LOG_FORMAT(Q, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
#define LOGY(tag, format, ...)  do {if (user_log_level >= LOG_ERROR)   LOG_PRINTF(LOG_FORMAT(Y, format), mp_hal_ticks_us(), tag, ##__VA_ARGS__); } while (0)
*/
#else
#define LOGE(tag, format, ...)
#define LOGe(tag, format, ...)
#define LOGW(tag, format, ...)
#define LOGI(tag, format, ...)
#define LOGD(tag, format, ...)
#define LOGV(tag, format, ...)
#define LOGM(tag, format, ...)
#define LOGQ(tag, format, ...)
#define LOGY(tag, format, ...)
#endif  /* LOG_ENABLE */

#ifdef __cplusplus
}
#endif


#endif /* _SYSLOG_H */

