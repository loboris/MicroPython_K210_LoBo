// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include "syslog.h"
#include "http_header.h"
#include "http_utils.h"
#include "platform_k210.h"

static const char *TAG = "HTTP_HEADER";
#define HEADER_BUFFER (1024)

/**
 * dictionary item struct, with key-value pair
 */
typedef struct http_header_item {
    char *key;                          /*!< key */
    char *value;                        /*!< value */
    STAILQ_ENTRY(http_header_item) next;   /*!< Point to next entry */
} http_header_item_t;

STAILQ_HEAD(http_header, http_header_item);


http_header_handle_t http_header_init()
{
    http_header_handle_t header = pvPortMalloc(sizeof(struct http_header));
    HTTP_MEM_CHECK(TAG, header, return NULL);
    memset(header, 0, sizeof(struct http_header));
    STAILQ_INIT(header);
    return header;
}

int http_header_destroy(http_header_handle_t header)
{
    int err = http_header_clean(header);
    if (header) vPortFree(header);
    return err;
}

http_header_item_handle_t http_header_get_item(http_header_handle_t header, const char *key)
{
    http_header_item_handle_t item;
    if (header == NULL || key == NULL) {
        return NULL;
    }
    STAILQ_FOREACH(item, header, next) {
        if (strcasecmp(item->key, key) == 0) {
            return item;
        }
    }
    return NULL;
}

int http_header_get(http_header_handle_t header, const char *key, char **value)
{
    http_header_item_handle_t item;

    item = http_header_get_item(header, key);
    if (item) {
        *value = item->value;
    } else {
        *value = NULL;
    }

    return 0;
}

static int http_header_new_item(http_header_handle_t header, const char *key, const char *value)
{
    http_header_item_handle_t item;

    item = pvPortMalloc(sizeof(http_header_item_t));
    HTTP_MEM_CHECK(TAG, item, return K210_ERR_NO_MEM);
    memset(item, 0, sizeof(http_header_item_t));
    http_utils_assign_string(&item->key, key, 0);
    HTTP_MEM_CHECK(TAG, item->key, goto _header_new_item_exit);
    http_utils_trim_whitespace(&item->key);
    http_utils_assign_string(&item->value, value, 0);
    HTTP_MEM_CHECK(TAG, item->value, goto _header_new_item_exit);
    http_utils_trim_whitespace(&item->value);
    STAILQ_INSERT_TAIL(header, item, next);
    return 0;
_header_new_item_exit:
    if (item->key) vPortFree(item->key);
    if (item->value) vPortFree(item->value);
    return K210_ERR_NO_MEM;
}

int http_header_set(http_header_handle_t header, const char *key, const char *value)
{
    http_header_item_handle_t item;

    if (value == NULL) {
        return http_header_delete(header, key);
    }

    item = http_header_get_item(header, key);

    if (item) {
        vPortFree(item->value);
        item->value = string_dup(value);
        http_utils_trim_whitespace(&item->value);
        return 0;
    }
    return http_header_new_item(header, key, value);
}

int http_header_set_from_string(http_header_handle_t header, const char *key_value_data)
{
    char *eq_ch;
    char *p_str;

    p_str = string_dup(key_value_data);
    HTTP_MEM_CHECK(TAG, p_str, return K210_ERR_NO_MEM);
    eq_ch = strchr(p_str, ':');
    if (eq_ch == NULL) {
        vPortFree(p_str);
        return K210_ERR_INVALID_ARG;
    }
    *eq_ch = 0;

    http_header_set(header, p_str, eq_ch + 1);
    vPortFree(p_str);
    return 0;
}


int http_header_delete(http_header_handle_t header, const char *key)
{
    http_header_item_handle_t item = http_header_get_item(header, key);
    if (item) {
        STAILQ_REMOVE(header, item, http_header_item, next);
        if (item->key) vPortFree(item->key);
        if (item->value) vPortFree(item->value);
        vPortFree(item);
    } else {
        return K210_ERR_NOT_FOUND;
    }
    return 0;
}


int http_header_set_format(http_header_handle_t header, const char *key, const char *format, ...)
{
    va_list argptr;
    int len = 0;
    char *buf = NULL;
    va_start(argptr, format);
    len = vasprintf(&buf, format, argptr);
    HTTP_MEM_CHECK(TAG, buf, return 0);
    va_end(argptr);
    if (buf == NULL) {
        return 0;
    }
    http_header_set(header, key, buf);
    vPortFree(buf);
    return len;
}

int http_header_generate_string(http_header_handle_t header, int index, char *buffer, int *buffer_len)
{
    http_header_item_handle_t item;
    int siz = 0;
    int idx = 0;
    int ret_idx = -1;
    bool is_end = false;
    STAILQ_FOREACH(item, header, next) {
        if (item->value && idx >= index) {
            siz += strlen(item->key);
            siz += strlen(item->value);
            siz += 4; //': ' and '\r\n'
        }
        idx ++;

        if (siz + 1 > *buffer_len - 2) {
            ret_idx = idx - 1;
        }
    }

    if (siz == 0) {
        return 0;
    }
    if (ret_idx < 0) {
        ret_idx = idx;
        is_end = true;
    }

    int str_len = 0;
    idx = 0;
    STAILQ_FOREACH(item, header, next) {
        if (item->value && idx >= index && idx < ret_idx) {
            str_len += snprintf(buffer + str_len, *buffer_len - str_len, "%s: %s\r\n", item->key, item->value);
        }
        idx ++;
    }
    if (is_end) {
        str_len += snprintf(buffer + str_len, *buffer_len - str_len, "\r\n");
    }
    *buffer_len = str_len;
    return ret_idx;
}

int http_header_clean(http_header_handle_t header)
{
    http_header_item_handle_t item = STAILQ_FIRST(header), tmp;
    while (item != NULL) {
        tmp = STAILQ_NEXT(item, next);
        if (item->key) vPortFree(item->key);
        if (item->value) vPortFree(item->value);
        if (item) vPortFree(item);
        item = tmp;
    }
    STAILQ_INIT(header);
    return 0;
}

int http_header_count(http_header_handle_t header)
{
    http_header_item_handle_t item;
    int count = 0;
    STAILQ_FOREACH(item, header, next) {
        count ++;
    }
    return count;
}
