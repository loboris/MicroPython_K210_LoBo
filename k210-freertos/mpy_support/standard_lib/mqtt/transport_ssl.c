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

#include <string.h>
#include <stdlib.h>

#include "transport.h"
#include "transport_ssl.h"


static const char *TAG = "TRANS_SSL";

/*
 *  WiFi SSL specific transport data
 */
typedef struct {
    socket_obj_t    *sock;
    void            *cert_pem_data;
    int             cert_pem_len;
    /*
    void            *client_cert_pem_data;
    int             client_cert_pem_len;
    void            *client_key_pem_data;
    int             client_key_pem_len;
    bool            mutual_authentication;
    bool            ssl_initialized;
    bool            verify_server;
     */
} transport_ssl_t;

static int ssl_close(transport_handle_t t);

//--------------------------------------------------------------------------------------
static int ssl_connect(transport_handle_t t, const char *host, int port, int timeout_ms)
{
    int ret = -1;
    transport_ssl_t *ssl = transport_get_context_data(t);

    if (!ssl) return -1;

    if (ssl->cert_pem_data) {
        ret = wifi_sendcertificate((const char *)ssl->cert_pem_data, ssl->cert_pem_len);
        if (ret < 0) {
            if (transport_debug) LOGE(TAG, "error loading CA certificate %d", ret);
            goto exit;
        }
    }
    else {
    }

    ret = -1;
    /*
    if (ssl->client_cert_pem_data && ssl->client_key_pem_data) {
        if (transport_debug) LOGW(TAG, "Client cert/key authentication not available");
    }
    else if (ssl->client_cert_pem_data || ssl->client_key_pem_data) {
        if (transport_debug) LOGE(TAG, "You have to provide both client_cert_pem and client_key_pem for mutual authentication");
        goto exit;
    }
    */

    ssl->sock->fd = at_get_socket(ssl->sock);
    if (ssl->sock->fd < 0) {
        vPortFree(ssl);
        if (transport_debug) LOGE(TAG, "Error creating socket");
        goto exit;
    }
    ssl->sock->link_id = ssl->sock->fd;
    if (transport_debug) LOGI(TAG, "Socket created: %d", ssl->sock->fd);

    ret = wifi_connect(ssl->sock, host, port, 0);
    if (ret < 0) {
        if (transport_debug) LOGE(TAG, "error connecting %d", ret);
        goto exit;
    }
    if (transport_debug) LOGD(TAG, "Connected to %s:%d", host, port);
    ssl->sock->peer_closed = false;
    return ssl->sock->fd;
exit:
    ssl_close(t);
    return ret;
}

//------------------------------------------------------------
static int ssl_poll_read(transport_handle_t t, int timeout_ms)
{
    transport_ssl_t *ssl = transport_get_context_data(t);

    int buflen = 0;
    int wait_end = mp_hal_ticks_ms() + timeout_ms;
    // wait for socket data
    while (mp_hal_ticks_ms() <= wait_end) {
        buflen = ssl->sock->buffer.length;
        if (buflen > 0) break;
        vTaskDelay(10);
    }

    return buflen;
}

//-------------------------------------------------------------
static int ssl_poll_write(transport_handle_t t, int timeout_ms)
{
    //transport_ssl_t *ssl = transport_get_context_data(t);
    return 1;
}

//-------------------------------------------------------------------------------------
static int ssl_write(transport_handle_t t, const char *buffer, int len, int timeout_ms)
{
    int ret;
    transport_ssl_t *ssl = transport_get_context_data(t);

    ret = wifi_send(ssl->sock, buffer, len);
    if (ret <= 0) {
        if (transport_debug) LOGE(TAG, "Write error, errno=%s", strerror(errno));
    }
    return ret;
}

//------------------------------------------------------------------------------
static int ssl_read(transport_handle_t t, char *buffer, int len, int timeout_ms)
{
    int poll = -1, ret;
    transport_ssl_t *ssl = transport_get_context_data(t);

    if (ssl->sock->buffer.length <= 0) {
        if ((poll = transport_poll_read(t, timeout_ms)) <= 0) {
            return poll;
        }
    }
    ret = wifi_read(ssl->sock, buffer, len);
    if (ret <= 0) {
        return -1;
    }
    return ret;
}

//----------------------------------------
static int ssl_close(transport_handle_t t)
{
    int ret = -1;
    transport_ssl_t *ssl = transport_get_context_data(t);
    ret = wifi_close(ssl->sock);
    ssl->sock->fd = -1;
    return ret;
}

//------------------------------------------
static int ssl_destroy(transport_handle_t t)
{
    transport_ssl_t *ssl = transport_get_context_data(t);
    transport_close(t);
    vPortFree(ssl);
    return 0;
}

//-------------------------------------------------------------------------------
void transport_ssl_set_cert_data(transport_handle_t t, const char *data, int len)
{
    transport_ssl_t *ssl = transport_get_context_data(t);
    if (t && ssl) {
        ssl->cert_pem_data = (void *)data;
        ssl->cert_pem_len = len;
    }
}

//--------------------------------------------------------------------------------------
void transport_ssl_set_client_cert_data(transport_handle_t t, const char *data, int len)
{
    /*
    transport_ssl_t *ssl = transport_get_context_data(t);
    if (t && ssl) {
        ssl->client_cert_pem_data = (void *)data;
        ssl->client_cert_pem_len = len;
    }
    */
}

//-------------------------------------------------------------------------------------
void transport_ssl_set_client_key_data(transport_handle_t t, const char *data, int len)
{
    /*
    transport_ssl_t *ssl = transport_get_context_data(t);
    if (t && ssl) {
        ssl->client_key_pem_data = (void *)data;
        ssl->client_key_pem_len = len;
    }
    */
}

//-------------------------------------
transport_handle_t transport_ssl_init()
{
    if (!(net_active_interfaces & ACTIVE_INTERFACE_WIFI)) {
        if (transport_debug) LOGW(TAG, "Only available for WiFi network interface");
        return NULL;
    }
    int ssl_bufsize = 0;
    bool f = wifi_set_ssl_buffer_size(&ssl_bufsize);
    if (f) {
        if (ssl_bufsize < 16384) {
            ssl_bufsize = 16384;
            wifi_set_ssl_buffer_size(&ssl_bufsize);
        }
    }
    transport_handle_t t = transport_init();
    transport_ssl_t *ssl = pvPortMalloc(sizeof(transport_ssl_t));
    K210_MEM_CHECK(TAG, ssl, return NULL);
    memset(ssl, 0, sizeof(transport_ssl_t));

    ssl->sock = _new_socket();
    ssl->sock->proto = WIFI_IPPROTO_SSL;
    ssl->sock->fd = -1;

    transport_set_context_data(t, ssl);
    transport_set_func(t, ssl_connect, ssl_read, ssl_write, ssl_close, ssl_poll_read, ssl_poll_write, ssl_destroy);
    return t;
}

#endif
