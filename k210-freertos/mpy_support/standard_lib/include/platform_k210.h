/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Mqtt Module using MQTT task.
 * Based on ESP32 MQTT Library by Tuan PM, https://github.com/tuanpmt/espmqtt
 * Adapted for MicroPython by Boris Lovosevic, https://github.com/loboris
 *
 */

#ifndef _PLATFORM_K210_H__
#define _PLATFORM_K210_H__

#include "mpconfigport.h"

#include "syslog.h"
#include "FreeRTOS.h"

#include "modmachine.h"
#include "mphalport.h"
#if MICROPY_PY_USE_NETTWORK
#include "at_util.h"
#endif

#define K210_ERR_NO_MEM             0x101   /*!< Out of memory */
#define K210_ERR_INVALID_ARG        0x102   /*!< Invalid argument */
#define K210_ERR_INVALID_STATE      0x103   /*!< Invalid state */
#define K210_ERR_INVALID_SIZE       0x104   /*!< Invalid size */
#define K210_ERR_NOT_FOUND          0x105   /*!< Requested resource not found */
#define K210_ERR_NOT_SUPPORTED      0x106   /*!< Operation or feature not supported */
#define K210_ERR_TIMEOUT            0x107   /*!< Operation timed out */
#define K210_ERR_INVALID_RESPONSE   0x108   /*!< Received response was invalid */
#define K210_ERR_INVALID_CRC        0x109   /*!< CRC or checksum was invalid */
#define K210_ERR_INVALID_VERSION    0x10A   /*!< Version was invalid */
#define K210_ERR_INVALID_MAC        0x10B   /*!< MAC address was invalid */

char *platform_create_id_string();
int platform_random(int max);
long long platform_tick_get_ms();
void ms_to_timeval(int timeout_ms, struct timeval *tv);
bool base64_encode(unsigned char* in, int inlen, unsigned char* out, int *outlen);
char *mqttstrdup(const char *src);

#define K210_MEM_CHECK(TAG, a, action) if (!(a)) {                                        \
        LOGE(TAG,"%s:%d (%s): %s", __FILE__, __LINE__, __FUNCTION__, "Memory exhausted"); \
        action;                                                                           \
        }

#endif
