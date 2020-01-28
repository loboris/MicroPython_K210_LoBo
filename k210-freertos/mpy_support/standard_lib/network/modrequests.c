/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, vPortFree of charge, to any person obtaining a copy
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


#include "platform_k210.h"

#if MICROPY_PY_USE_REQUESTS

#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "syslog.h"

#include "http_client.h"
#include "transport.h"

#include "py/obj.h"
#include "py/runtime.h"
#include "modmachine.h"
#include "extmod/vfs.h"
#include "py/stream.h"

#define MAX_HTTP_RECV_BUFFER    512
#define FLOAT_FIELD_DEC_PLACES  8
#define DEFAULT_RQBODY_LEN      64*1024
#define DEFAULT_RQHEADER_LEN    2048

static const char *TAG = "[REQUESTS]";
static const char *TAG_EVENT = "[REQUESTS EVENT]";

static char *rqheader = NULL;
static char *rqbody = NULL;
static mp_obj_t rqbody_file = mp_const_none;
static int rqheader_ptr = 0;
static int rqbody_len = DEFAULT_RQBODY_LEN;
static int rqbody_ptr = 0;
static bool rqbody_ok = true;
static bool rqheader_ok = true;
static bool rq_base64 = false;
static char *cert_pem = NULL;

#if MICROPY_PY_USE_WIFI == 0
static bool wifi_task_semaphore_active;
#endif


/*
 * You can get the Root cert for the server using openssl

   The PEM file can be  extracted from the output of this command:
   openssl s_client -showcerts -connect <the_server>:443

   The CA root cert is the last cert given in the chain of certs.
*/

//----------------------------------------------------------------
static int _http_event_handler(esp_http_client_event_t *evt)
{
    mp_hal_wdt_reset();
    /*
    bool mutex_taken = false;
    if ((net_active_interfaces & ACTIVE_INTERFACE_WIFI)) {
        mutex_taken = wifi_take_mutex();
        if (!mutex_taken) {
            if (transport_debug) LOGE(TAG_EVENT, "ERROR taking mutex");
        }
    }
    */
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            if (transport_debug) LOGW(TAG_EVENT, "ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            if (transport_debug) LOGI(TAG_EVENT, "OnConnected");
            break;
        case HTTP_EVENT_HEADER_SENT:
            if (transport_debug) {
                LOGD(TAG_EVENT, "Header sent");
                LOGD(TAG_EVENT, "  Key: [%s]", evt->header_key);
                LOGD(TAG_EVENT, "Value: [%s]", evt->header_value);
            }
            break;
        case HTTP_EVENT_ON_HEADER:
            if (transport_debug) LOGI(TAG_EVENT, "OnHeader: key=%s, value=%s", evt->header_key, evt->header_value);
            if ((rqheader != NULL) && (rqheader_ok)) {
                int len = rqheader_ptr + strlen(evt->header_key) + strlen(evt->header_value) + 5;
                if (len > DEFAULT_RQHEADER_LEN) rqheader_ok = false;
                if (rqheader_ok) {
                    strcat(rqheader, evt->header_key);
                    strcat(rqheader, ": ");
                    strcat(rqheader, evt->header_value);
                    strcat(rqheader, "\r\n");
                    rqheader_ptr = strlen(rqheader);
                }
                else if (transport_debug) LOGW(TAG_EVENT, "Header buffer size to small (%d)", DEFAULT_RQHEADER_LEN);
            }
            break;
        case HTTP_EVENT_ON_DATA:
            if (transport_debug) LOGI(TAG_EVENT, "OnData: len=%d", evt->data_len);
            if (rqbody_ok) {
                if (rqbody_file != mp_const_none) {
                    int nwrite = mp_stream_posix_write((void *)rqbody_file, evt->data, evt->data_len);
                    if (nwrite <= 0) {
                        rqbody_ok = false;
                        if (transport_debug) LOGE(TAG_EVENT, "Download: Error writing to file %d", nwrite);
                    }
                    else {
                        if (transport_debug) LOGI(TAG_EVENT, "Write data to body file: rqptr=%d + %d", rqbody_ptr, nwrite);
                        rqbody_ptr += evt->data_len;
                    }
                }
                else {
                    if (transport_debug) LOGI(TAG_EVENT, "Write data to body buffer: rqptr=%d/%d", rqbody_ptr, rqbody_len);
                    if (rqbody != NULL) {
                        int len = evt->data_len + rqbody_ptr;
                        if (len > rqbody_len) rqbody_ok = false;
                        if (rqbody_ok) {
                            memcpy(rqbody + rqbody_ptr, evt->data, evt->data_len);
                            rqbody_ptr += evt->data_len;
                            rqbody[rqbody_ptr] = '\0';
                            if (transport_debug) LOGD(TAG_EVENT, "Write %d byte(s) to body buffer", evt->data_len);
                        }
                        else if (transport_debug) LOGW(TAG_EVENT, "Body buffer size to small");
                    }
                    else {
                        rqbody_ok = false;
                        if (transport_debug) LOGW(TAG_EVENT, "No allocated body buffer");
                    }
                }
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            if (transport_debug) LOGI(TAG_EVENT, "OnFinish");
            break;
        case HTTP_EVENT_DISCONNECTED:
            if (transport_debug) LOGI(TAG_EVENT, "Disconnected");
            break;
        default:
            if (transport_debug) LOGW(TAG_EVENT, "Unhandled: %d", evt->event_id);
    }
    //if (mutex_taken) wifi_give_mutex();
    return 0;
}

// Prepare a string of parameters from dictionary items
//-----------------------------------------------
static char *url_post_fields(mp_obj_dict_t *dict)
{
    char *params = pvPortMalloc(256);
    if (params == NULL) return 0;
    memset(params, 0 , 256);

    int nparam = 0;
    const char *key;
    const char *value;
    char sval[64];
    size_t max = dict->map.alloc;
    mp_map_t *map = &dict->map;
    mp_map_elem_t *next;
    size_t cur = 0;
    while (1) {
        next = NULL;
        for (size_t i = cur; i < max; i++) {
            if (mp_map_slot_is_filled(map, i)) {
                cur = i + 1;
                next = &(map->table[i]);
                break;
            }
        }
        if (next == NULL) break;

        value = NULL;
        key = mp_obj_str_get_str(next->key);

        if (mp_obj_is_str(next->value)) {
            value = mp_obj_str_get_str(next->value);
            for (int i=0; i<strlen(value); i++) {
                if ((value[i] < 0x20) || (value[i] > 0x7F)) {
                    value = NULL;
                    break;
                }
            }
        }
        else if (mp_obj_is_int(next->value)) {
            int ival = mp_obj_get_int(next->value);
            sprintf(sval,"%d", ival);
            value = sval;
        }
        else if (mp_obj_is_float(next->value)) {
            double fval = mp_obj_get_float(next->value);
            sprintf(sval,"%.*f", FLOAT_FIELD_DEC_PLACES, fval);
            value = sval;
        }
        else {
            if (transport_debug) LOGW(TAG, "Unsupported type for key '%s'", key);
        }
        if ((value) && ((strlen(params) + strlen(key) + strlen(value) + 2) < 256)) {
            if (strlen(params) > 0) strcat(params, "&");
            strcat(params, key);
            strcat(params, "=");
            strcat(params, value);
            nparam++;
        }
    }
    if (nparam == 0) {
        vPortFree(params);
        params = NULL;
    }
    return params;
}

//-------------------------------------------------------------------------------------------------------------------------------
static int post_field(esp_http_client_handle_t client, char *bndry, char *key, char *cont_type, char *value, int vlen, bool send)
{
    int len = vlen;
    int res;
    char buff[256];
    sprintf(buff, "--%s\r\nContent-Disposition: form-data; name=%s\r\n%s\r\n\r\n", bndry, key, cont_type);
    len += strlen(buff);
    if (send) {
        res = esp_http_client_write(client, buff, strlen(buff));
        if (transport_debug) mp_printf(&mp_plat_print, "%s", buff);
        if (res <= 0) return res;
        res = esp_http_client_write(client, value, vlen);
        if (transport_debug) mp_printf(&mp_plat_print, "%s", value);
        if (res <= 0) return res;
    }

    sprintf(buff, "\r\n--%s\r\n\r\n", bndry);
    len += strlen(buff);
    if (send) {
        res = esp_http_client_write(client, buff, strlen(buff));
        if (transport_debug) mp_printf(&mp_plat_print, "%s", buff);
        if (res <= 0) return res;
    }
    return len;
}

//-----------------------------------------------------------------------------------------------------
static int handle_file(esp_http_client_handle_t client, char *bndry, char *key, char *fname, bool send)
{
    int flen = 0, res = 0;
    if (strlen(fname) < 128) {
        if (mp_vfs_import_stat(fname) != MP_IMPORT_STAT_FILE) return 0;
        // Multipart field value is an existing file
        mp_obj_t fargs[2];
        fargs[0] = mp_obj_new_str(fname, strlen(fname));
        fargs[1] = mp_obj_new_str("rb", 2);
        mp_obj_t ffd = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);
        if (ffd) {
            char *buff = pvPortMalloc(1024);
            if (buff == NULL) return flen;

            if (key) {
                sprintf(buff, "--%s\r\nContent-Disposition: form-data; name=\"file_%s\"; filename=\"%s\"\r\nContent-Type: application/octet-stream\r\n\r\n", bndry, key, fname);
                flen += strlen(buff);
                if (send) {
                    res = esp_http_client_write(client, buff, strlen(buff));
                    if (transport_debug) mp_printf(&mp_plat_print, "%s", buff);
                    if (res <= 0) {
                        vPortFree(buff);
                        mp_stream_close(ffd);
                        return -1;
                    }
                }
            }
            res = 1;
            while (res > 0) {
                res = mp_stream_posix_read((void *)ffd, buff, 1024);
                if (res <= 0) break;
                if (send) {
                    if (res > 0) buff[res] = 0;
                    //if ((transport_debug) && (res > 0)) mp_printf(&mp_plat_print, "%s", buff);
                    res = esp_http_client_write(client, buff, res);
                }
                flen += res;
            }
            mp_stream_close(ffd);

            if (key) {
                sprintf(buff, "\r\n--%s\r\n\r\n", bndry);
                flen += strlen(buff);
                if (send) {
                    res = esp_http_client_write(client, buff, strlen(buff));
                    if (transport_debug) mp_printf(&mp_plat_print, "%s", buff);
                    if (res <= 0) {
                        vPortFree(buff);
                        return -1;
                    }
                }
            }
            vPortFree(buff);
        }
    }
    return flen;
}

// Parse MicroPython dictionary, calculate the content length
// or send the body to the server
//------------------------------------------------------------------------------------------------------------
static int multipart_post_fields(mp_obj_dict_t *dict, char *bndry, esp_http_client_handle_t client, bool send)
{
    int data_len = 0;
    int res;
    int nparam = 0;
    char *key;
    char *value;
    int vlen;
    bool value_free;
    char sval[64];
    char cont_type[64];

    sprintf(cont_type, "%s", "Content-Type: text/plain");

    // Parse dictionary
    size_t max = dict->map.alloc;
    mp_map_t *map = &dict->map;
    mp_map_elem_t *next;
    size_t cur = 0;
    if ((transport_debug) && (send)) mp_printf(&mp_plat_print, ">>> SEND FIELDS [\n");
    while (1) {
        next = NULL;
        for (size_t i = cur; i < max; i++) {
            if (mp_map_slot_is_filled(map, i)) {
                cur = i + 1;
                next = &(map->table[i]);
                break;
            }
        }
        if (next == NULL) break;

        value = NULL;
        value_free = false;
        key = (char *)mp_obj_str_get_str(next->key);

        if (mp_obj_is_str(next->value)) {
            // String, it can be string to send or file name
            value = (char *)mp_obj_str_get_str(next->value);
            vlen = strlen(value);
            res = handle_file(client, bndry, key, value, send);
            if (res == 0) {
                for (int i=0; i<strlen(value); i++) {
                    if ((value[i] < 0x20) || (value[i] > 0x7F)) {
                        value = NULL;
                        break;
                    }
                }
            }
            else {
                data_len += res;
                nparam++;
                value = NULL;
            }
        }
        else if (mp_obj_is_int(next->value)) {
            int ival = mp_obj_get_int(next->value);
            sprintf(sval,"%d", ival);
            value = sval;
            vlen = strlen(sval);
        }
        else if (mp_obj_is_float(next->value)) {
            double fval = mp_obj_get_float(next->value);
            sprintf(sval,"%.*f", FLOAT_FIELD_DEC_PLACES, fval);
            value = sval;
            vlen = strlen(sval);
        }
        else {
            mp_obj_type_t *type = mp_obj_get_type(next->value);
            mp_buffer_info_t value_buff;
            if (type->buffer_p.get_buffer != NULL) {
                // bytes or string data
                int ret = type->buffer_p.get_buffer(next->value, &value_buff, MP_BUFFER_READ);
                if ((ret == 0) && (value_buff.buf) && (value_buff.len > 0)) {
                    if (rq_base64) {
                        // Send as base64 encoded string
                        vlen = ((value_buff.len + ((value_buff.len % 3) ? (3 - (value_buff.len % 3)) : 0)) * 4 / 3) + 2;
                        value = pvPortMalloc(vlen);
                        if (value != NULL) {
                            if (transport_debug) LOGD(TAG, "Encoding bytes, p=%p (%p), len=%lu (%d)", (unsigned char *)value_buff.buf, (unsigned char *)value, value_buff.len, vlen);
                            if (base64_encode((unsigned char *)value_buff.buf, value_buff.len, (unsigned char *)value, &vlen)) {
                                value[vlen] = '\0';
                                sprintf(cont_type, "%s", "Content-Type: binary\r\nContent-Encoding: base64");
                                value_free = true;
                            }
                            else {
                                vPortFree((void *)value);
                                value = NULL;
                            }
                        }
                    }
                    else {
                        value = value_buff.buf;
                        vlen = value_buff.len;
                    }
                }
            }
        }
        if (value) {
            res = post_field(client, bndry, key, cont_type, value, vlen, send);
            if (value_free) vPortFree((void *)value);
            if (res <= 0) return res;
            data_len += res;
            nparam++;
        }
    }
    if (transport_debug) {
        if (send) {
            mp_printf(&mp_plat_print, "] LENGTH=%d\n", data_len);
        }
        else mp_printf(&mp_plat_print, ">>> CONTENT LENGTH = %d\n", data_len);
    }
    return data_len;
}

//----------------------------------------------------------------------------------------------------------------
static mp_obj_t request(int method, bool multipart, mp_obj_t post_data_in, char * url, char *tofile, int buf_size)
{
    if (transport_debug) LOGI(TAG, "Preparing HTTP Request");
    int status;
    char err_msg[128] = {'\0'};
    int err;
    bool perform_handled = false;
    bool free_post_data = false;

    // Check if the response is redirected to file
    rqbody_file = mp_const_none;
    if (tofile != NULL) {
        // GET to file
        mp_obj_t fargs[2];
        fargs[0] = mp_obj_new_str(tofile, strlen(tofile));
        fargs[1] = mp_obj_new_str("wb", 2);
        rqbody_file = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);
        if (!rqbody_file) {
            mp_raise_msg(&mp_type_OSError, "Error opening file");
        }
    }

    char* post_data = NULL;
    char bndry[32];

    esp_http_client_config_t config = {0};
    config.url = url;
    config.event_handler = _http_event_handler;
    config.buffer_size = buf_size;
    config.cert_pem = cert_pem;

    // Initialize the http_client and set the method
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
        mp_raise_msg(&mp_type_OSError, "Error initializing http client");
    }
    esp_http_client_set_method(client, method);

    // Free buffers if allocated previously
    if (rqheader) vPortFree(rqheader);
    if (rqbody) vPortFree(rqbody);
    rqheader = NULL;
    rqbody = NULL;

    // Allocate buffers
    rqheader = pvPortMalloc(DEFAULT_RQHEADER_LEN);
    if (rqheader != NULL) memset(rqheader, 0, DEFAULT_RQHEADER_LEN);
    else {
        if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
        mp_raise_msg(&mp_type_OSError, "Error allocating header buffer");
    }
    int rqbody_size = rqbody_len;
    if (tofile != NULL) rqbody_size = 384;
    rqbody = pvPortMalloc(rqbody_size);
    if (rqbody != NULL) memset(rqbody, 0, rqbody_size);
    else {
        if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
        if (rqheader) vPortFree(rqheader);
        mp_raise_msg(&mp_type_OSError, "Error allocating body buffer");
    }
    rqbody_ptr = 0;
    rqbody_ok = true;
    rqheader_ok = true;
    wifi_task_semaphore_active = true;

    // Execute requested method
    if (method == HTTP_METHOD_POST) {
        mp_obj_dict_t *dict;
        if (!multipart) {
            // === url encoded POST request ===
            if (transport_debug) LOGD(TAG, "POST, url encoded");
            if (mp_obj_is_type(post_data_in, &mp_type_dict)) {
                dict = MP_OBJ_TO_PTR(post_data_in);
                post_data = url_post_fields(dict);
                err = esp_http_client_set_post_field(client, post_data, strlen(post_data));
                if (err != 0) {
                    if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
                    vPortFree(post_data);
                    wifi_task_semaphore_active = false;
                    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error setting post fields"));
                }
                free_post_data = true;
            }
            else if (mp_obj_is_str(post_data_in)) {
                post_data = (char *)mp_obj_str_get_str(post_data_in);
                err = esp_http_client_set_post_field(client, post_data, strlen(post_data));
                if (err != 0) {
                    if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
                    wifi_task_semaphore_active = false;
                    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error setting post fields"));
                }
            }
            else {
                if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
                wifi_task_semaphore_active = false;
                nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Expected Dict or String type argument"));
            }
        }
        else {
            // === multipart POST ===
            if (mp_obj_is_type(post_data_in, &mp_type_dict)) {
                dict = MP_OBJ_TO_PTR(post_data_in);
            }
            else {
                if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
                wifi_task_semaphore_active = false;
                nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Expected Dict type argument"));
            }

            // Prepare multipart boundary
            memset(bndry,0x00,20);
            int randn = rand();
            sprintf(bndry, "_____%d_____", randn);
            // Get body length
            int cont_len = multipart_post_fields(dict, bndry, client, false);
            if (cont_len <= 0) {
                nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Nothing to send"));
            }
            char temp_buf[128];
            sprintf(temp_buf, "multipart/form-data; boundary=%s", bndry);
            esp_http_client_set_header(client, "Content-Type", temp_buf);

            // Perform actions
            MP_THREAD_GIL_EXIT();
            err = 0;
            do {
                if ((err = esp_http_client_open(client, cont_len)) != 0) {
                    sprintf(err_msg, "Http client error: open");
                    break;
                }

                // Send content
                cont_len = multipart_post_fields(dict, bndry, client, true);

                // Check response
                if ((esp_http_client_perform_response(client)) != 0) {
                    sprintf(err_msg, "Http client error: response");
                    break;
                }
            } while (esp_http_client_process_again(client));
            esp_http_client_cleanup(client);
            MP_THREAD_GIL_ENTER();
            perform_handled = true;
        }
    }
    else if ((method == HTTP_METHOD_PUT) || (method == HTTP_METHOD_PATCH) || (method == HTTP_METHOD_DELETE)) {
        if (mp_obj_is_str(post_data_in)) {
            post_data = (char *)mp_obj_str_get_str(post_data_in);
            err = esp_http_client_set_post_field(client, post_data, strlen(post_data));
            if (err != 0) {
                if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
                wifi_task_semaphore_active = false;
                nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error setting post fields"));
            }
        }
    }

    if (!perform_handled) {
        // POST method is already handled, handle others methods here
        MP_THREAD_GIL_EXIT();
        err = esp_http_client_perform(client);
        esp_http_client_cleanup(client);
        if ((free_post_data) && (post_data)) vPortFree(post_data);
        MP_THREAD_GIL_ENTER();
    }

    if (err != 0) {
        if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
        if (rqheader) vPortFree(rqheader);
        if (rqbody) vPortFree(rqbody);
        rqbody_file = NULL;
        rqheader = NULL;
        rqbody = NULL;
        wifi_task_semaphore_active = false;
        if (transport_debug) LOGE(TAG, "HTTP Request failed: %d [%s]", err, err_msg);
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "HTTP Request failed"));
    }

    status = esp_http_client_get_status_code(client);

    // Prepare the return value, 3-item tuple (status, header, body);
    mp_obj_t tuple[3];

    tuple[0] = mp_obj_new_int(status);

    if ((rqheader) && (rqheader_ptr)) tuple[1] = mp_obj_new_str(rqheader, rqheader_ptr);
    else tuple[1] = mp_const_none;

    if (rqbody_file != mp_const_none) {
        sprintf(rqbody, "Saved to file '%s', size=%d", tofile, rqbody_ptr);
        tuple[2] = mp_obj_new_str(rqbody, strlen(rqbody));
    }
    else if ((rqbody) && (rqbody_ptr)) tuple[2] = mp_obj_new_bytes((const byte*)rqbody, rqbody_ptr);
    else tuple[2] = mp_const_none;

    // Free file stream and buffers
    if (rqbody_file != mp_const_none) mp_stream_close(rqbody_file);
    if (rqheader) vPortFree(rqheader);
    if (rqbody) vPortFree(rqbody);
    rqbody_file = mp_const_none;
    rqheader = NULL;
    rqbody = NULL;
    wifi_task_semaphore_active = false;

    return mp_obj_new_tuple(3, tuple);
}

//-----------------------------------------------------
void get_certificate(mp_obj_t cert, char *cert_pem_buf)
{
    if (mp_obj_is_str(cert)) {
        mp_obj_t fargs[2];
        fargs[0] = cert;
        fargs[1] = mp_obj_new_str("rb", 2);
        mp_obj_t fhndl = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);

        if (!fhndl) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error opening certificate file"));
        }

        // Get file size
        int fsize = mp_stream_posix_lseek((void *)fhndl, 0, SEEK_END);
        int at_start = mp_stream_posix_lseek((void *)fhndl, 0, SEEK_SET);
        if ((fsize <= 0) || (at_start != 0)) {
            mp_stream_close(fhndl);
            mp_raise_ValueError("Error getting file size");
        }
        if (cert_pem_buf) vPortFree(cert_pem_buf);
        cert_pem_buf = NULL;
        cert_pem_buf = pvPortMalloc(fsize+16);
        if (cert_pem_buf == NULL) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating certificate buffer"));
        }
        memset(cert_pem_buf, 0, fsize);
        if ( mp_stream_posix_read((void *)fhndl, cert_pem_buf, fsize) == fsize) {
            mp_stream_close(fhndl);
        }
        else {
            mp_stream_close(fhndl);
            vPortFree(cert_pem_buf);
            cert_pem_buf = NULL;
            mp_raise_ValueError("Error reading certificate file");
        }
    }
    else if (cert == mp_const_false) {
        if (cert_pem_buf) vPortFree(cert_pem_buf);
        cert_pem_buf = NULL;
    }
}

//--------------------------------------------------------------------------------------
STATIC mp_obj_t requests_GET(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    //network_checkConnection();
    enum { ARG_url, ARG_file, ARG_bufsize };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_url,   MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_file,                    MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_bufsize,                 MP_ARG_INT, { .u_int = 1536 } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *url = NULL;
    char *fname = NULL;

    url = (char *)mp_obj_str_get_str(args[ARG_url].u_obj);

    if (mp_obj_is_str(args[ARG_file].u_obj)) {
        // GET to file
        fname = (char *)mp_obj_str_get_str(args[ARG_file].u_obj);
    }

    mp_obj_t res = request(HTTP_METHOD_GET, false, NULL, url, fname, args[ARG_bufsize].u_int);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(requests_GET_obj, 1, requests_GET);

//--------------------------------------------------------------------------------------
STATIC mp_obj_t requests_HEAD(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    //network_checkConnection();
    enum { ARG_url };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_url,   MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *url = NULL;

    url = (char *)mp_obj_str_get_str(args[ARG_url].u_obj);

    mp_obj_t res = request(HTTP_METHOD_HEAD, false, NULL, url, NULL, 1536);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(requests_HEAD_obj, 1, requests_HEAD);

//---------------------------------------------------------------------------------------
STATIC mp_obj_t requests_POST(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    //network_checkConnection();
    enum { ARG_url, ARG_params, ARG_file, ARG_multipart, ARG_base64, ARG_bufsize };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_url,        MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_params,     MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_file,                         MP_ARG_OBJ,  { .u_obj = mp_const_none } },
        { MP_QSTR_multipart,                    MP_ARG_BOOL, { .u_bool = false } },
        { MP_QSTR_base64,                       MP_ARG_BOOL, { .u_bool = false } },
        { MP_QSTR_bufsize,                      MP_ARG_INT, { .u_int = 1536 } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *url = NULL;
    char *fname = NULL;

    rq_base64 = args[ARG_base64].u_bool;
    url = (char *)mp_obj_str_get_str(args[ARG_url].u_obj);

    if (mp_obj_is_str(args[ARG_file].u_obj)) {
        // POST response to file
        fname = (char *)mp_obj_str_get_str(args[ARG_file].u_obj);
    }

    mp_obj_t res = request(HTTP_METHOD_POST, args[ARG_multipart].u_bool, args[ARG_params].u_obj, url, fname, args[ARG_bufsize].u_int);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(requests_POST_obj, 1, requests_POST);

//--------------------------------------------------------------------------------------
STATIC mp_obj_t requests_PUT(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    //network_checkConnection();
    enum { ARG_url, ARG_data };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_url,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_data, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *url = NULL;

    url = (char *)mp_obj_str_get_str(args[ARG_url].u_obj);

    mp_obj_t res = request(HTTP_METHOD_PUT, false, args[ARG_data].u_obj, url, NULL, 1536);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(requests_PUT_obj, 1, requests_PUT);

//----------------------------------------------------------------------------------------
STATIC mp_obj_t requests_PATCH(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    //network_checkConnection();
    enum { ARG_url, ARG_data };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_url,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_data, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *url = NULL;

    url = (char *)mp_obj_str_get_str(args[ARG_url].u_obj);

    mp_obj_t res = request(HTTP_METHOD_PATCH, false, args[ARG_data].u_obj, url, NULL, 1536);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(requests_PATCH_obj, 1, requests_PATCH);

//-----------------------------------------------------------------------------------------
STATIC mp_obj_t requests_DELETE(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    //network_checkConnection();
    enum { ARG_url, ARG_data };
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_url,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_data,                   MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *url = NULL;

    url = (char *)mp_obj_str_get_str(args[ARG_url].u_obj);

    mp_obj_t res = request(HTTP_METHOD_DELETE, false, args[ARG_data].u_obj, url, NULL, 1536);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(requests_DELETE_obj, 1, requests_DELETE);

//---------------------------------------------
STATIC mp_obj_t requests_debug(mp_obj_t dbg_in)
{
    transport_debug = mp_obj_is_true(dbg_in);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(requests_debug_obj, requests_debug);

//-----------------------------------------------------------------------
STATIC mp_obj_t requests_certificate(size_t n_args, const mp_obj_t *args)
{
    if (n_args > 0) {
        get_certificate(args[0], cert_pem);
    }

    return (cert_pem == NULL) ? mp_const_false : mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(requests_certificate_obj, 0, 1, requests_certificate);

//----------------------------------------------------------------------
STATIC mp_obj_t requests_bodybuffer(size_t n_args, const mp_obj_t *args)
{
    int body_len = rqbody_len;
    if (n_args > 0) {
        body_len = mp_obj_get_int(args[0]);
        if (body_len < 4096) rqbody_len = 4096;
        else if (body_len > 524288) rqbody_len = 524288;
        else rqbody_len = body_len;
    }
    return mp_obj_new_int(rqbody_len);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(requests_bodybuffer_obj, 0, 1, requests_bodybuffer);


//=============================================================
STATIC const mp_rom_map_elem_t requests_locals_dict_table[] = {
        { MP_ROM_QSTR(MP_QSTR_get),         MP_ROM_PTR(&requests_GET_obj) },
        { MP_ROM_QSTR(MP_QSTR_head),        MP_ROM_PTR(&requests_HEAD_obj) },
        { MP_ROM_QSTR(MP_QSTR_post),        MP_ROM_PTR(&requests_POST_obj) },
        { MP_ROM_QSTR(MP_QSTR_put),         MP_ROM_PTR(&requests_PUT_obj) },
        { MP_ROM_QSTR(MP_QSTR_patch),       MP_ROM_PTR(&requests_PATCH_obj) },
        { MP_ROM_QSTR(MP_QSTR_delete),      MP_ROM_PTR(&requests_DELETE_obj) },
        { MP_ROM_QSTR(MP_QSTR_debug),       MP_ROM_PTR(&requests_debug_obj) },
        { MP_ROM_QSTR(MP_QSTR_certificate), MP_ROM_PTR(&requests_certificate_obj) },
        { MP_ROM_QSTR(MP_QSTR_bodybuffer),  MP_ROM_PTR(&requests_bodybuffer_obj) },
};
STATIC MP_DEFINE_CONST_DICT(requests_locals_dict, requests_locals_dict_table);

//===================================
const mp_obj_type_t requests_type = {
    { &mp_type_type },
    .name = MP_QSTR_requests,
    //.print = mqtt_print,
    //.make_new = mqtt_make_new,
    .locals_dict = (mp_obj_dict_t*)&requests_locals_dict,
};

#endif
