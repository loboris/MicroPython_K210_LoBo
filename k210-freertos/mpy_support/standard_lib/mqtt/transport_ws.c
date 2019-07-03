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

#include "platform_k210.h"

#ifdef MICROPY_PY_USE_MQTT

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <netif/ppp/polarssl/sha1.h>

#include "syslog.h"

#include "transport.h"
#include "transport_tcp.h"
#include "transport_ws.h"

static const char *TAG = "TRANSPORT_WS";

#define DEFAULT_WS_BUFFER (1024)

typedef struct {
    char *path;
    char *buffer;
    transport_handle_t parent;
} transport_ws_t;

/*
// Table 6-bit-index-to-ASCII used for base64-encoding
static const char base64_table[] = {
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h',
  'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
  'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
  '+', '/'
};

//----------------------------------------------------------------------------------------------------------------------------
static void base64_encode(unsigned char* target, size_t target_len, size_t *outlen, unsigned  char* source, size_t source_len)
{
  size_t i;
  s8_t j;
  size_t target_idx = 0;
  size_t longer = (source_len % 3) ? (3 - (source_len % 3)) : 0;
  size_t source_len_b64 = source_len + longer;
  size_t len = (((source_len_b64) * 4) / 3);
  u8_t x = 5;
  u8_t current = 0;

  if (target_len >= len) {
      *outlen = 0;
      return;
  }

  for (i = 0; i < source_len_b64; i++) {
    u8_t b = (i < source_len ? (u8_t)source[i] : 0);
    for (j = 7; j >= 0; j--, x--) {
      if ((b & (1 << j)) != 0) {
        current = (u8_t)(current | (1U << x));
      }
      if (x == 0) {
        target[target_idx++] = base64_table[current];
        x = 6;
        current = 0;
      }
    }
  }
  for (i = len - longer; i < len; i++) {
    target[i] = '=';
  }
  *outlen = len;
}
*/

//------------------------------------------
static char *trimwhitespace(const char *str)
{
    char *end;

    // Trim leading space
    while (isspace((unsigned char)*str)) str++;

    if (*str == 0) return (char *)str;

    // Trim trailing space
    end = (char *)(str + strlen(str) - 1);
    while (end > str && isspace((unsigned char)*end)) end--;

    // Write new null terminator
    *(end + 1) = 0;

    return (char *)str;
}

/*
//------------------------------------------------------
static char* stristr(const char* str1, const char* str2)
{
    const char* p1 = str1 ;
    const char* p2 = str2 ;
    const char* r = *p2 == 0 ? str1 : 0 ;

    while ( *p1 != 0 && *p2 != 0 ) {
        if (tolower((unsigned char)*p1 ) == tolower((unsigned char)*p2)) {
            if ( r == 0 ) r = p1 ;
            p2++ ;
        }
        else {
            p2 = str2 ;
            if ( r != 0 ) p1 = r + 1 ;
            if (tolower((unsigned char)*p1 ) == tolower((unsigned char)*p2)) {
                r = p1 ;
                p2++ ;
            }
            else r = 0 ;
        }
        p1++ ;
    }

    return *p2 == 0 ? (char*)r : 0 ;
}
*/

//-------------------------------------------------------
char* stristr(const char* haystack, const char* needle) {
  do {
    const char* h = haystack;
    const char* n = needle;
    while (tolower((unsigned char) *h) == tolower((unsigned char ) *n) && *n) {
      h++;
      n++;
    }
    if (*n == 0) {
      return (char *) haystack;
    }
  } while (*haystack++);
  return NULL;
}

//---------------------------------------------------------------
static char *get_http_header(const char *buffer, const char *key)
{
    char *found = stristr(buffer, key);
    if (found) {
        found += strlen(key);
        char *found_end = strstr(found, "\r\n");
        if (found_end) {
            found_end[0] = 0; //terminate string

            return trimwhitespace(found);
        }
    }
    return NULL;
}

//-------------------------------------------------------------------------------------
static int ws_connect(transport_handle_t t, const char *host, int port, int timeout_ms)
{
    transport_ws_t *ws = transport_get_context_data(t);
    if (transport_connect(ws->parent, host, port, timeout_ms) < 0) {
        if (transport_debug) LOGE(TAG, "Error connecting to the server");
        return -1;
    }

    int klen;
    // Generate & Base64 encode random key
    unsigned char random_key[16] = { 0 }, client_key[32] = {0};
    for (int i = 0; i < sizeof(random_key); i++) {
        random_key[i] = random_at_most(255) & 0xFF;
    }
    klen = 32;
    if (!base64_encode(random_key, 16, client_key, &klen)) return -1;

    int len =   snprintf(ws->buffer, DEFAULT_WS_BUFFER,
                         "GET %s HTTP/1.1\r\n"
                         "Host: %s\r\n"
                         "User-Agent: ESP32 MQTT Client\r\n"
                         "Connection: Upgrade\r\n"
                         "Upgrade: websocket\r\n"
                         "Sec-WebSocket-Version: 13\r\n"
                         "Sec-WebSocket-Protocol: mqtt\r\n"
                         "Sec-WebSocket-Key: %s\r\n\r\n",
                         ws->path,
                         host,
                         client_key);

    if (transport_debug) LOGD(TAG, "Write upgrade header\r\n%s", ws->buffer);
    if (transport_write(ws->parent, ws->buffer, len, timeout_ms) <= 0) {
        if (transport_debug) LOGE(TAG, "Error write Upgrade header %s", ws->buffer);
        return -1;
    }
    if ((len = transport_read(ws->parent, ws->buffer, DEFAULT_WS_BUFFER, timeout_ms)) <= 0) {
        if (transport_debug) LOGE(TAG, "Error reading response for Upgrade header");
        return -1;
    }

    char *server_key = get_http_header(ws->buffer, "Sec-WebSocket-Accept:");
    if (server_key == NULL) {
        if (transport_debug) LOGE(TAG, "Sec-WebSocket-Accept not found in\r\n[%s]", ws->buffer);
        return -1;
    }

    // Check Accept Key
    unsigned char client_key_b64[96], valid_client_key[20], accept_key[32] = {0};
    int key_len = sprintf((char*)client_key_b64, "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", (char*)client_key);
    sha1(client_key_b64, (size_t)key_len, valid_client_key);
    klen = 32;
    if (!base64_encode(valid_client_key, 20, accept_key, &klen)) return -1;

    if (transport_debug) LOGD(TAG, "server key=%s, send_key=%s, accept_key=%s", (char *)server_key, (char*)client_key, accept_key);
    if (strcmp((char*)accept_key, (char*)server_key) != 0) {
        if (transport_debug) LOGE(TAG, "Invalid websocket key");
        return -1;
    }
    return 0;
}

//----------------------------------------------------------------------------------
static int ws_write(transport_handle_t t, const char *buff, int len, int timeout_ms)
{
    transport_ws_t *ws = transport_get_context_data(t);
    char ws_header[MAX_WEBSOCKET_HEADER_SIZE];
    char *mask;
    int header_len = 0, i;
    char *buffer = (char *)buff;
    int poll_write;
    if ((poll_write = transport_poll_write(ws->parent, timeout_ms)) <= 0) {
        return poll_write;
    }

    ws_header[header_len++] = WS_OPCODE_BINARY | WS_FIN;

    // NOTE: no support for > 16-bit sized messages
    if (len > 125) {
        ws_header[header_len++] = WS_SIZE16 | WS_MASK;
        ws_header[header_len++] = (uint8_t)(len >> 8);
        ws_header[header_len++] = (uint8_t)(len & 0xFF);
    } else {
        ws_header[header_len++] = (uint8_t)(len | WS_MASK);
    }
    mask = &ws_header[header_len];
    ws_header[header_len++] = rand() & 0xFF;
    ws_header[header_len++] = rand() & 0xFF;
    ws_header[header_len++] = rand() & 0xFF;
    ws_header[header_len++] = rand() & 0xFF;

    for (i = 0; i < len; ++i) {
        buffer[i] = (buffer[i] ^ mask[i % 4]);
    }
    if (transport_write(ws->parent, ws_header, header_len, timeout_ms) != header_len) {
        if (transport_debug) LOGE(TAG, "Write header error");
        return -1;
    }
    return transport_write(ws->parent, buffer, len, timeout_ms);
}

//-----------------------------------------------------------------------------
static int ws_read(transport_handle_t t, char *buffer, int len, int timeout_ms)
{
    transport_ws_t *ws = transport_get_context_data(t);
    int payload_len;
    char *data_ptr = buffer, opcode, mask, *mask_key = NULL;
    int rlen;
    int poll_read;
    if ((poll_read = transport_poll_read(ws->parent, timeout_ms)) <= 0) {
        return poll_read;
    }
    if ((rlen = transport_read(ws->parent, buffer, len, timeout_ms)) <= 0) {
        if (transport_debug) LOGE(TAG, "Read data error");
        return rlen;
    }

    opcode = (*data_ptr & 0x0F);
    data_ptr ++;
    mask = ((*data_ptr >> 7) & 0x01);
    payload_len = (*data_ptr & 0x7F);
    data_ptr++;
    if (transport_debug) LOGD(TAG, "Opcode: %d, mask: %d, len: %d\r\n", opcode, mask, payload_len);
    if (payload_len == 126) {
        // headerLen += 2;
        payload_len = data_ptr[0] << 8 | data_ptr[1];
        data_ptr += 2;
    } else if (payload_len == 127) {
        // headerLen += 8;

        if (data_ptr[0] != 0 || data_ptr[1] != 0 || data_ptr[2] != 0 || data_ptr[3] != 0) {
            // really too big!
            payload_len = 0xFFFFFFFF;
        } else {
            payload_len = data_ptr[4] << 24 | data_ptr[5] << 16 | data_ptr[6] << 8 | data_ptr[7];
        }
        data_ptr += 8;
    }

    if (mask) {
        mask_key = data_ptr;
        data_ptr += 4;
        for (int i = 0; i < payload_len; i++) {
            buffer[i] = (data_ptr[i] ^ mask_key[i % 4]);
        }
    } else {
        memmove(buffer, data_ptr, payload_len);
    }
    return payload_len;
}

//-----------------------------------------------------------
static int ws_poll_read(transport_handle_t t, int timeout_ms)
{
    transport_ws_t *ws = transport_get_context_data(t);
    return transport_poll_read(ws->parent, timeout_ms);
}

//------------------------------------------------------------
static int ws_poll_write(transport_handle_t t, int timeout_ms)
{
    transport_ws_t *ws = transport_get_context_data(t);
    return transport_poll_write(ws->parent, timeout_ms);;
}

//---------------------------------------
static int ws_close(transport_handle_t t)
{
    transport_ws_t *ws = transport_get_context_data(t);
    return transport_close(ws->parent);
}

//-----------------------------------------
static int ws_destroy(transport_handle_t t)
{
    transport_ws_t *ws = transport_get_context_data(t);
    vPortFree(ws->buffer);
    vPortFree(ws->path);
    vPortFree(ws);
    return 0;
}

//----------------------------------------------------------------
void transport_ws_set_path(transport_handle_t t, const char *path)
{
    transport_ws_t *ws = transport_get_context_data(t);
    ws->path = realloc(ws->path, strlen(path) + 1);
    strcpy(ws->path, path);
}

//--------------------------------------------------------------------
transport_handle_t transport_ws_init(transport_handle_t parent_handle)
{
    transport_handle_t t = transport_init();
    transport_ws_t *ws = pvPortMalloc(sizeof(transport_ws_t));
    K210_MEM_CHECK(TAG, ws, return NULL);
    memset(ws, 0, sizeof(transport_ws_t));
    ws->parent = parent_handle;

    ws->path = mqttstrdup("/mqtt");
    K210_MEM_CHECK(TAG, ws->path, return NULL);
    ws->buffer = pvPortMalloc(DEFAULT_WS_BUFFER);
    K210_MEM_CHECK(TAG, ws->buffer, {
        vPortFree(ws->path);
        vPortFree(ws);
        return NULL;
    });

    transport_set_func(t, ws_connect, ws_read, ws_write, ws_close, ws_poll_read, ws_poll_write, ws_destroy);
    transport_set_context_data(t, ws);
    return t;
}

#endif
