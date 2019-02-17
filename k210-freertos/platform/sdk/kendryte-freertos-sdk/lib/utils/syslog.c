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
#ifndef _SYSLOG_H
#define _SYSLOG_H

#include "syslog.h"
#include "printf.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t user_log_level = 2;
uint64_t log_divisor = (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000);

int log_print(const char* format, ...)
{
    va_list ap;

    va_start(ap, format);
    /* Begin protected code */
    corelock_lock(&lock);
    tfp_format(stdout_putp, uart_putf, format, ap);
    /* End protected code */
    corelock_unlock(&lock);
    va_end(ap);

    return 0;
}

#ifdef __cplusplus
}
#endif


