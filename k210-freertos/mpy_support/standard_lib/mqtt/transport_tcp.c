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

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "syslog.h"

#include "transport.h"

static const char *TAG = "TRANS_TCP";

typedef struct {
    socket_obj_t    *sock;
} transport_tcp_t;

//--------------------------------------------------------------
static int resolve_dns(const char *host, struct sockaddr_in *ip)
{
    struct hostent *he;
    struct in_addr **addr_list;
    he = lwip_gethostbyname(host);
    if (he == NULL) {
        return -1;
    }
    addr_list = (struct in_addr **)he->h_addr_list;
    if (addr_list[0] == NULL) {
        return -1;
    }
    ip->sin_family = AF_INET;
    memcpy(&ip->sin_addr, addr_list[0], sizeof(ip->sin_addr));
    return 0;
}

//----------------------------------------------------------------------------------------
static int mqtcp_connect(transport_handle_t t, const char *host, int port, int timeout_ms)
{
    transport_tcp_t *tcp = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        tcp->sock->fd = at_get_socket(tcp->sock);
        if (tcp->sock->fd < 0) {
            if (transport_debug) LOGE(TAG, "Error creating socket");
            return -1;
        }
        tcp->sock->link_id = tcp->sock->fd;

        if (transport_debug) LOGD(TAG, "[sock=%d] Connecting to server: %s, Port:%d...", tcp->sock->fd, host, port);
        int ret = wifi_connect(tcp->sock, host, port, 0);
        if (ret < 0) {
            if (transport_debug) LOGE(TAG, "Error connecting (%d)", ret);
            return -1;
        }
        if (transport_debug) LOGD(TAG, "Connected to %s:%d", host, port);
        return tcp->sock->fd;
    }

    // Connect using lwip sockets
    struct sockaddr_in remote_ip;
    struct timeval tv;

    bzero(&remote_ip, sizeof(struct sockaddr_in));

    //if stream_host is not ip address, resolve it AF_INET,servername,&serveraddr.sin_addr
    if (lwip_inet_pton(AF_INET, host, &remote_ip.sin_addr) != 1) {
        if (resolve_dns(host, &remote_ip) < 0) return -1;
    }

    tcp->sock->fd = lwip_socket(PF_INET, SOCK_STREAM, 0);

    if (tcp->sock->fd < 0) {
        if (transport_debug) LOGE(TAG, "Error creating socket");
        return -1;
    }

    remote_ip.sin_family = AF_INET;
    remote_ip.sin_port = htons(port);

    ms_to_timeval(timeout_ms, &tv);

    lwip_setsockopt(tcp->sock->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (transport_debug) LOGD(TAG, "[sock=%d] Connecting to server IP: %s, Port: %d...",
             tcp->sock->fd, ipaddr_ntoa((const ip_addr_t*)&remote_ip.sin_addr.s_addr), port);
    if (lwip_connect(tcp->sock->fd, (struct sockaddr *)(&remote_ip), sizeof(struct sockaddr)) != 0) {
        if (transport_debug) LOGE(TAG, "Error connecting");
        lwip_close(tcp->sock->fd);
        tcp->sock->fd = -1;
        return -1;
    }
    if (transport_debug) LOGD(TAG, "Connected to %s:%d", host, port);
    return tcp->sock->fd;
}

//---------------------------------------------------------------------------------------
static int mqtcp_write(transport_handle_t t, const char *buffer, int len, int timeout_ms)
{
    int poll, ret;
    transport_tcp_t *tcp = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        ret = wifi_send(tcp->sock, buffer, len);
        if (ret <= 0) {
            if (transport_debug) LOGE(TAG, "Write error, errno=%s", strerror(errno));
        }
        return ret;
    }

    // Write using lwip socket
    if ((poll = transport_poll_write(t, timeout_ms)) <= 0) {
        if (transport_debug) LOGE(TAG, "Write error, poll=%d", poll);
        return poll;
    }
    ret = lwip_write(tcp->sock->fd, buffer, len);
    return ret;
}

//--------------------------------------------------------------------------------
static int mqtcp_read(transport_handle_t t, char *buffer, int len, int timeout_ms)
{
    transport_tcp_t *tcp = transport_get_context_data(t);
    int poll = -1;

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        if (tcp->sock->buffer.length <= 0) {
            poll = transport_poll_read(t, timeout_ms);
            if (poll <= 0) return poll;
        }
        int ret = wifi_read(tcp->sock, buffer, len);
        if (ret <= 0) {
            return -1;
        }
        return ret;
    }

    // Read using lwip socket
    if ((poll = transport_poll_read(t, timeout_ms)) <= 0) return poll;

    int read_len = lwip_read(tcp->sock->fd, buffer, len);
    if (read_len == 0) return -1;
    return read_len;
}

//--------------------------------------------------------------
static int mqtcp_poll_read(transport_handle_t t, int timeout_ms)
{
    transport_tcp_t *tcp = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        int buflen = 0;
        int wait_end = mp_hal_ticks_ms() + timeout_ms;
        // wait for socket data
        while (mp_hal_ticks_ms() <= wait_end) {
            buflen = tcp->sock->buffer.length;
            if (buflen > 0) break;
            vTaskDelay(10);
        }

        return buflen;
    }

    // Check lwip socket for data available
    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(tcp->sock->fd, &readset);
    struct timeval timeout;
    ms_to_timeval(timeout_ms, &timeout);

    return lwip_select(tcp->sock->fd + 1, &readset, NULL, NULL, &timeout);
}

//---------------------------------------------------------------
static int mqtcp_poll_write(transport_handle_t t, int timeout_ms)
{
    transport_tcp_t *tcp = transport_get_context_data(t);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) return 1;

    // Check lwip socket for write ready
    fd_set writeset;
    FD_ZERO(&writeset);
    FD_SET(tcp->sock->fd, &writeset);
    struct timeval timeout;
    ms_to_timeval(timeout_ms, &timeout);
    return lwip_select(tcp->sock->fd + 1, NULL, &writeset, NULL, &timeout);
}

//------------------------------------------
static int mqtcp_close(transport_handle_t t)
{
    transport_tcp_t *tcp = transport_get_context_data(t);
    int ret = -1;

    if (transport_debug) LOGD(TAG, "Close socket (%d)", tcp->sock->fd);
    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        ret = wifi_close(tcp->sock);
    }
    else if (tcp->sock->fd >= 0) {
        ret = lwip_close(tcp->sock->fd);
        tcp->sock->fd = -1;
    }
    return ret;
}

//--------------------------------------------
static int mqtcp_destroy(transport_handle_t t)
{
    transport_tcp_t *tcp = transport_get_context_data(t);
    transport_close(t);
    vPortFree(tcp);
    return 0;
}

//-------------------------------------
transport_handle_t transport_tcp_init()
{
    transport_handle_t t = transport_init();
    transport_tcp_t *tcp = pvPortMalloc(sizeof(transport_tcp_t));
    K210_MEM_CHECK(TAG, tcp, return NULL);
    memset(tcp, 0, sizeof(transport_tcp_t));

    tcp->sock = _new_socket();
    tcp->sock->fd = -1;

    transport_set_func(t, mqtcp_connect, mqtcp_read, mqtcp_write, mqtcp_close, mqtcp_poll_read, mqtcp_poll_write, mqtcp_destroy);
    transport_set_context_data(t, tcp);

    return t;
}

#endif
