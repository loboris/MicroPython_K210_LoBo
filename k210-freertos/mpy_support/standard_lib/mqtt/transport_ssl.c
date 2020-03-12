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

#if MICROPY_PY_USE_NETTWORK

#include <string.h>
#include <stdlib.h>

#include "transport.h"
#include "transport_ssl.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mbedtls/platform.h"
#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"
#include "mbedtls/net_sockets.h"

static const char *TAG = "TRANS_SSL";

#ifdef MBEDTLS_DEBUG_C
static void mbedtls_debug(void *ctx, int level, const char *file, int line, const char *str) {
    (void)ctx;
    if (!transport_debug) return;
    char *pfile = strrchr(file, '/');
    if (!pfile) {
        LOGM("[mbedtls]", "[%d] %s:%04d: %s", level, file, line, str);
    }
    pfile += 1;
    if (level == 1) LOGE("[mbedtls]", "[%d] %s:%04d: %s", level, pfile, line, str);
    else if (level == 2) LOGW("[mbedtls]", "[%d] %s:%04d: %s", level, pfile, line, str);
    else if (level == 3) LOGI("[mbedtls]", "[%d] %s:%04d: %s", level, pfile, line, str);
    else if (level == 4) LOGD("[mbedtls]", "[%d] %s:%04d: %s", level, pfile, line, str);
}
#endif


/*
 *  WiFi SSL specific transport data
 */
typedef struct {
    socket_obj_t            *sock;
    void                    *cert_pem_data;
    int                     cert_pem_len;
    // used only with lwip
    mbedtls_entropy_context  entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ssl_context      ctx;
    mbedtls_x509_crt         cacert;
    mbedtls_x509_crt         client_cert;
    mbedtls_pk_context       client_key;
    mbedtls_ssl_config       conf;
    mbedtls_net_context      client_fd;
    void                     *client_cert_pem_data;
    int                      client_cert_pem_len;
    void                     *client_key_pem_data;
    int                      client_key_pem_len;
    bool                     mutual_authentication;
    bool                     ssl_initialized;
    bool                     verify_server;
} transport_ssl_t;

static int ssl_close(transport_handle_t t);

//--------------------------------------------------------------------------------------
static int ssl_connect(transport_handle_t t, const char *host, int port, int timeout_ms)
{
    int ret = -1;
    transport_ssl_t *ssl = transport_get_context_data(t);
    if (!ssl) return -1;

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        #if MICROPY_PY_USE_WIFI

        if (ssl->cert_pem_data) {
            ret = wifi_sendcertificate((const char *)ssl->cert_pem_data, ssl->cert_pem_len);
            if (ret < 0) {
                if (transport_debug) LOGE(TAG, "error loading CA certificate %d", ret);
                goto wexit;
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
            goto wexit;
        }
        ssl->sock->link_id = ssl->sock->fd;
        if (transport_debug) LOGI(TAG, "Socket created: %d", ssl->sock->fd);

        #if MICROPY_PY_USE_WIFI
        ret = wifi_connect(ssl->sock, host, port, 0);
        #endif
        if (ret < 0) {
            if (transport_debug) LOGE(TAG, "error connecting %d", ret);
            goto wexit;
        }
        if (transport_debug) LOGD(TAG, "Connected to %s:%d", host, port);
        ssl->sock->peer_closed = false;
        return ssl->sock->fd;
    wexit:
        ssl_close(t);
        return ret;
        #else
        return -1;
        #endif
    }

    // Connect using lwip sockets
    int flags;
    struct timeval tv;
    ssl->ssl_initialized = true;
    mbedtls_ssl_init(&ssl->ctx);
    mbedtls_ctr_drbg_init(&ssl->ctr_drbg);
    mbedtls_ssl_config_init(&ssl->conf);
    mbedtls_entropy_init(&ssl->entropy);

    if ((ret = mbedtls_ssl_config_defaults(&ssl->conf,
                                           MBEDTLS_SSL_IS_CLIENT,
                                           MBEDTLS_SSL_TRANSPORT_STREAM,
                                           MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
        if (transport_debug) LOGE(TAG, "mbedtls_ssl_config_defaults returned %d", ret);
        goto exit;
    }

    if ((ret = mbedtls_ctr_drbg_seed(&ssl->ctr_drbg, mbedtls_entropy_func, &ssl->entropy, NULL, 0)) != 0) {
        if (transport_debug) LOGE(TAG, "mbedtls_ctr_drbg_seed returned %d", ret);
        goto exit;
    }

    mbedtls_x509_crt_init(&ssl->cacert);
    if (ssl->cert_pem_data) {
        ssl->verify_server = true;
        if ((ret = mbedtls_x509_crt_parse(&ssl->cacert, ssl->cert_pem_data, ssl->cert_pem_len + 1)) < 0) {
            if (transport_debug) LOGE(TAG, "mbedtls_x509_crt_parse returned -0x%x\r\nDATA=%s,len=%d", -ret, (char*)ssl->cert_pem_data, ssl->cert_pem_len);
            goto exit;
        }
        mbedtls_ssl_conf_ca_chain(&ssl->conf, &ssl->cacert, NULL);
        mbedtls_ssl_conf_authmode(&ssl->conf, MBEDTLS_SSL_VERIFY_REQUIRED);

        if ((ret = mbedtls_ssl_set_hostname(&ssl->ctx, host)) != 0) {
            if (transport_debug) LOGE(TAG, "mbedtls_ssl_set_hostname returned -0x%x", -ret);
            goto exit;
        }
    } else {
        mbedtls_ssl_conf_authmode(&ssl->conf, MBEDTLS_SSL_VERIFY_NONE);
    }

    mbedtls_x509_crt_init(&ssl->client_cert);
    mbedtls_pk_init(&ssl->client_key);
    if (ssl->client_cert_pem_data && ssl->client_key_pem_data) {
        ssl->mutual_authentication = true;
        if ((ret = mbedtls_x509_crt_parse(&ssl->client_cert, ssl->client_cert_pem_data, ssl->client_cert_pem_len + 1)) < 0) {
            if (transport_debug) LOGE(TAG, "mbedtls_x509_crt_parse returned -0x%x\r\nDATA=%s,len=%d", -ret, (char*)ssl->client_cert_pem_data, ssl->client_cert_pem_len);
            goto exit;
        }
        if ((ret = mbedtls_pk_parse_key(&ssl->client_key, ssl->client_key_pem_data, ssl->client_key_pem_len + 1, NULL, 0)) < 0) {
            if (transport_debug) LOGE(TAG, "mbedtls_pk_parse_keyfile returned -0x%x\r\nDATA=%s,len=%d", -ret, (char*)ssl->client_key_pem_data, ssl->client_key_pem_len);
            goto exit;
        }

        if ((ret = mbedtls_ssl_conf_own_cert(&ssl->conf, &ssl->client_cert, &ssl->client_key)) < 0) {
            if (transport_debug) LOGE(TAG, "mbedtls_ssl_conf_own_cert returned -0x%x\n", -ret);
            goto exit;
        }
    } else if (ssl->client_cert_pem_data || ssl->client_key_pem_data) {
        if (transport_debug) LOGE(TAG, "You have to provide both client_cert_pem and client_key_pem for mutual authentication");
        goto exit;
    }

    mbedtls_ssl_conf_rng(&ssl->conf, mbedtls_ctr_drbg_random, &ssl->ctr_drbg);

    #ifdef MBEDTLS_DEBUG_C
    mbedtls_ssl_conf_dbg(&ssl->conf, mbedtls_debug, NULL);
    // Debug level (0-4)
    mbedtls_debug_set_threshold(1);
    #endif

    if ((ret = mbedtls_ssl_setup(&ssl->ctx, &ssl->conf)) != 0) {
        if (transport_debug) LOGE(TAG, "mbedtls_ssl_setup returned -0x%x", -ret);
        goto exit;
    }
    if (transport_debug) LOGD(TAG, "mbedtls setup OK");

    mbedtls_net_init(&ssl->client_fd);

    ms_to_timeval(timeout_ms, &tv);

    lwip_setsockopt(ssl->client_fd.fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (transport_debug) LOGD(TAG, "Connect to %s:%d", host, port);
    char port_str[8] = {0};
    sprintf(port_str, "%d", port);
    if ((ret = mbedtls_net_connect(&ssl->client_fd, host, port_str, MBEDTLS_NET_PROTO_TCP)) != 0) {
        if (transport_debug) LOGE(TAG, "mbedtls_net_connect returned -%x", -ret);
        goto exit;
    }

    mbedtls_ssl_set_bio(&ssl->ctx, &ssl->client_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

    if((ret = mbedtls_ssl_set_hostname(&ssl->ctx, host)) != 0) {
        if (transport_debug) LOGE(TAG, " failed\n  ! mbedtls_ssl_set_hostname returned %d\r\n", ret);
        goto exit;
    }

    if (transport_debug) LOGM(TAG, "Performing the SSL/TLS handshake...");

    while ((ret = mbedtls_ssl_handshake(&ssl->ctx)) != 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            if (transport_debug) LOGE(TAG, "mbedtls_ssl_handshake returned -0x%x", -ret);
            goto exit;
        }
    }

    if (transport_debug) LOGD(TAG, "Verifying peer X.509 certificate...");

    if ((flags = mbedtls_ssl_get_verify_result(&ssl->ctx)) != 0) {
        /* In real life, we probably want to close connection if ret != 0 */
        LOGW(TAG, "Failed to verify peer certificate!");
        if (ssl->cert_pem_data) {
            goto exit;
        }
    } else {
        if (transport_debug) LOGD(TAG, "Certificate verified.");
    }

    if (transport_debug) LOGD(TAG, "Cipher suite is %s", mbedtls_ssl_get_ciphersuite(&ssl->ctx));
    return 0;
exit:
    ssl_close(t);
    return ret;
}

//------------------------------------------------------------
static int ssl_poll_read(transport_handle_t t, int timeout_ms)
{
    transport_ssl_t *ssl = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        #if MICROPY_PY_USE_WIFI
        int buflen = 0;
        int wait_end = mp_hal_ticks_ms() + timeout_ms;
        // wait for socket data
        while (mp_hal_ticks_ms() <= wait_end) {
            buflen = ssl->sock->buffer.length;
            if (buflen > 0) break;
            vTaskDelay(10);
        }

        return buflen;
        #else
        return 0;
        #endif
    }

    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(ssl->client_fd.fd, &readset);
    struct timeval timeout;
    ms_to_timeval(timeout_ms, &timeout);

    return lwip_select(ssl->client_fd.fd + 1, &readset, NULL, NULL, &timeout);
}

//-------------------------------------------------------------
static int ssl_poll_write(transport_handle_t t, int timeout_ms)
{
    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) return 1;

    transport_ssl_t *ssl = transport_get_context_data(t);
    fd_set writeset;
    FD_ZERO(&writeset);
    FD_SET(ssl->client_fd.fd, &writeset);
    struct timeval timeout;
    ms_to_timeval(timeout_ms, &timeout);
    return lwip_select(ssl->client_fd.fd + 1, NULL, &writeset, NULL, &timeout);
}

//-------------------------------------------------------------------------------------
static int ssl_write(transport_handle_t t, const char *buffer, int len, int timeout_ms)
{
    int ret = -1;
    transport_ssl_t *ssl = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        #if MICROPY_PY_USE_WIFI
        ret = wifi_send(ssl->sock, buffer, len);
        if (ret <= 0) {
            if (transport_debug) LOGE(TAG, "Write error, errno=%s", strerror(errno));
        }
        return ret;
        #else
        return -1;
        #endif
    }

    int poll;

    if ((poll = transport_poll_write(t, timeout_ms)) <= 0) {
        if (transport_debug) LOGW(TAG, "Poll timeout or error, errno=%s, fd=%d, timeout_ms=%d", strerror(errno), ssl->client_fd.fd, timeout_ms);
        return poll;
    }
    ret = mbedtls_ssl_write(&ssl->ctx, (const unsigned char *) buffer, len);
    if (ret <= 0) {
        if (transport_debug) LOGE(TAG, "mbedtls_ssl_write error, errno=%s", strerror(errno));
    }
    return ret;
}

//------------------------------------------------------------------------------
static int ssl_read(transport_handle_t t, char *buffer, int len, int timeout_ms)
{
    int poll = -1, ret = -1;
    transport_ssl_t *ssl = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        if (ssl->sock->buffer.length <= 0) {
            if ((poll = transport_poll_read(t, timeout_ms)) <= 0) {
                return poll;
            }
        }
        #if MICROPY_PY_USE_WIFI
        ret = wifi_read(ssl->sock, buffer, len);
        #endif
        if (ret <= 0) return -1;
        return ret;
    }

    if (mbedtls_ssl_get_bytes_avail(&ssl->ctx) <= 0) {
        if ((poll = transport_poll_read(t, timeout_ms)) <= 0) {
            return poll;
        }
    }
    ret = mbedtls_ssl_read(&ssl->ctx, (unsigned char *)buffer, len);
    if (ret == 0) {
        return -1;
    }
    return ret;
}

//----------------------------------------
static int ssl_close(transport_handle_t t)
{
    int ret = -1;
    transport_ssl_t *ssl = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        #if MICROPY_PY_USE_WIFI
        ret = wifi_close(ssl->sock);
        #endif
        ssl->sock->fd = -1;
        return ret;
    }

    if (ssl->ssl_initialized) {
        if (transport_debug) LOGD(TAG, "Cleanup mbedtls");
        mbedtls_ssl_close_notify(&ssl->ctx);
        mbedtls_ssl_session_reset(&ssl->ctx);
        mbedtls_net_free(&ssl->client_fd);
        mbedtls_ssl_config_free(&ssl->conf);
        if (ssl->verify_server) {
            mbedtls_x509_crt_free(&ssl->cacert);
        }
        if (ssl->mutual_authentication) {
            mbedtls_x509_crt_free(&ssl->client_cert);
            mbedtls_pk_free(&ssl->client_key);
        }
        mbedtls_ctr_drbg_free(&ssl->ctr_drbg);
        mbedtls_entropy_free(&ssl->entropy);
        mbedtls_ssl_free(&ssl->ctx);
        ssl->mutual_authentication = false;
        ssl->ssl_initialized = false;
        ssl->verify_server = false;
    }
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
    transport_ssl_t *ssl = transport_get_context_data(t);
    if (t && ssl) {
        ssl->client_cert_pem_data = (void *)data;
        ssl->client_cert_pem_len = len;
    }
}

//-------------------------------------------------------------------------------------
void transport_ssl_set_client_key_data(transport_handle_t t, const char *data, int len)
{
    transport_ssl_t *ssl = transport_get_context_data(t);
    if (t && ssl) {
        ssl->client_key_pem_data = (void *)data;
        ssl->client_key_pem_len = len;
    }
}

//-------------------------------------
transport_handle_t transport_ssl_init()
{
    transport_handle_t t = transport_init();
    transport_ssl_t *ssl = pvPortMalloc(sizeof(transport_ssl_t));
    K210_MEM_CHECK(TAG, ssl, return NULL);
    memset(ssl, 0, sizeof(transport_ssl_t));

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        #if MICROPY_PY_USE_WIFI
        int ssl_bufsize = 0;
        bool f = wifi_set_ssl_buffer_size(&ssl_bufsize);
        if (f) {
            if (ssl_bufsize < 16384) {
                ssl_bufsize = 16384;
                wifi_set_ssl_buffer_size(&ssl_bufsize);
            }
        }

        ssl->sock = _new_socket();
        ssl->sock->proto = WIFI_IPPROTO_SSL;
        ssl->sock->fd = -1;
        #else
        return NULL;
        #endif
    }
    else {
        mbedtls_net_init(&ssl->client_fd);
    }

    transport_set_context_data(t, ssl);
    transport_set_func(t, ssl_connect, ssl_read, ssl_write, ssl_close, ssl_poll_read, ssl_poll_write, ssl_destroy);
    return t;
}

#endif
