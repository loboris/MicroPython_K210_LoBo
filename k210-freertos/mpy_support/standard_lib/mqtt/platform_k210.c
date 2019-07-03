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

#include <sys/time.h>
#include "platform_k210.h"

static const char *TAG = "[PLATFORM]";
static const char b64str[64] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

#define MAX_ID_STRING (32)

//-------------------------------
char *platform_create_id_string()
{
    uint32_t rnd_id = (100000 + random_at_most(9990000));

    char *id_string = pvPortMalloc(MAX_ID_STRING);
    K210_MEM_CHECK(TAG, id_string, return NULL);
    sprintf(id_string, "K210_%02x%02X%02X%02X", (rnd_id >> 24) & 0xFF, (rnd_id >> 16) & 0xFF, (rnd_id >> 8) & 0xFF, rnd_id & 0xFF);
    return id_string;
}

//--------------------------
int platform_random(int max)
{
    return random_at_most(max);
}

//------------------------------
long long platform_tick_get_ms()
{
    return mp_hal_ticks_ms();
}

//----------------------------------------------------
void ms_to_timeval(int timeout_ms, struct timeval *tv)
{
    tv->tv_sec = timeout_ms / 1000;
    tv->tv_usec = (timeout_ms - (tv->tv_sec * 1000)) * 1000;
}

//-------------------------------
char *mqttstrdup(const char *src)
{
    char *dst = pvPortMalloc(strlen (src) + 1); // Space for length plus null
    if (dst == NULL) return NULL;               // No memory
    strcpy(dst, src);                           // Copy the characters
    return dst;                                 // Return the new string
}

// Base64 encode input array 'in' of length 'inlen'
// into output string 'out' of maximal size 'outlen'
// place the created base64 string length into 'outlen'
// If 'out' is NULL, only calculate base64 string length
//-------------------------------------------------------------------------------
bool base64_encode(unsigned char* in, int inlen, unsigned char* out, int *outlen)
{
    int min_outlen = (inlen + ((inlen % 3) ? (3 - (inlen % 3)) : 0)) * 4 / 3;
    if (min_outlen >= *outlen) {
        LOGD(TAG, "B64: out string length too small");
        return false;
    }

    int in_remain = inlen;
    unsigned char* in_ptr = in;
    size_t len = 0;
    while (in_remain > 0) {
        // process 3 input bytes
        if (out) out[len] = b64str[(in_ptr[0] >> 2) & 0x3f];
        len++;
        if (len >= *outlen) break;

        if (out) out[len] = b64str[((in_ptr[0] << 4) + (--in_remain ? in_ptr[1] >> 4 : 0)) & 0x3f];
        len++;
        if (len >= *outlen) break;

        if (out) out[len] = (in_remain ? b64str[((in_ptr[1] << 2) + (--in_remain ? in_ptr[2] >> 6 : 0)) & 0x3f] : '=');
        len++;
        if (len >= *outlen) break;

        out[len] = in_remain ? b64str[in_ptr[2] & 0x3f] : '=';
        len++;
        if (len >= *outlen) break;

        if (in_remain) in_remain--;
        if (in_remain) in_ptr += 3;
    }

    if (len >= *outlen) return false;
    *outlen = len;
    if (out) {
        out[len] = '\0';
        LOGD(TAG, "B64: len=%d [%s]", *outlen, out);
    }
    return true;
}

