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
 *
 */

#include "at_util.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "py/runtime0.h"
#include "py/nlr.h"
#include "py/objlist.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "lib/netutils/netutils.h"

#include "network.h"
#include "lwip/opt.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/ip4.h"
#include "lwip/igmp.h"
#include "syslog.h"

#define SOCKET_POLL_US      (100000)
#define SOCKET_TIMEOUT_MAX  43200000
//#define IP_ADD_MEMBERSHIP 0x400

bool tcpip_adapter_initialized = false;
static const char *TAG = "[MODSOCKET]";

void _socket_settimeout(socket_obj_t *sock, uint64_t timeout_ms);
//socket_obj_t *_new_socket();


#if MICROPY_PY_USOCKET_EVENTS
// Support for callbacks on asynchronous socket events (when socket becomes readable)

// This divisor is used to reduce the load on the system, so it doesn't poll sockets too often
#define USOCKET_EVENTS_DIVISOR (8)

STATIC uint8_t usocket_events_divisor;
STATIC socket_obj_t *usocket_events_head;

void usocket_events_deinit(void) {
    usocket_events_head = NULL;
}

// Assumes the socket is not already in the linked list, and adds it
STATIC void usocket_events_add(socket_obj_t *sock) {
    sock->events_next = usocket_events_head;
    usocket_events_head = sock;
}

// Assumes the socket is already in the linked list, and removes it
STATIC void usocket_events_remove(socket_obj_t *sock) {
    for (socket_obj_t **s = &usocket_events_head;; s = &(*s)->events_next) {
        if (*s == sock) {
            *s = (*s)->events_next;
            return;
        }
    }
}

// Polls all registered sockets for readability and calls their callback if they are readable
void usocket_events_handler(void) {
    if (usocket_events_head == NULL) {
        return;
    }
    if (--usocket_events_divisor) {
        return;
    }
    usocket_events_divisor = USOCKET_EVENTS_DIVISOR;

    fd_set rfds;
    FD_ZERO(&rfds);
    int max_fd = 0;

    for (socket_obj_t *s = usocket_events_head; s != NULL; s = s->events_next) {
        FD_SET(s->fd, &rfds);
        max_fd = MAX(max_fd, s->fd);
    }

    // Poll the sockets
    struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };
    int r = lwip_select(max_fd + 1, &rfds, NULL, NULL, &timeout);
    if (r <= 0) {
        return;
    }

    // Call the callbacks
    for (socket_obj_t *s = usocket_events_head; s != NULL; s = s->events_next) {
        if (FD_ISSET(s->fd, &rfds)) {
            mp_call_function_1_protected(s->events_callback, s);
        }
    }
}

#endif // MICROPY_PY_USOCKET_EVENTS


//--------------------------------
static void check_net_interfaces()
{
    if (net_active_interfaces == ACTIVE_INTERFACE_NONE) {
        mp_raise_msg(&mp_type_NotImplementedError, "No active network interfaces");
    }
}

//---------------------------------------------------
NORETURN static void exception_from_errno(int _errno)
{
    // Here we need to convert from lwip errno values to MicroPython's standard ones
    if (_errno == EINPROGRESS) {
        _errno = MP_EINPROGRESS;
    }
    mp_raise_OSError(_errno);
}

//-------------------------------------------
static inline void check_for_exceptions(void)
{
    mp_handle_pending();
}

//----------------------------------------------------
static void _socket_freeaddrinfo(struct addrinfo *res)
{
    if (res) {
        if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) vPortFree(res);
        else lwip_freeaddrinfo(res);
    }
}
//------------------------------------------------------------------------------------------------
static int _socket_getaddrinfo2(const mp_obj_t host, const mp_obj_t portx, struct addrinfo **resp)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };

    mp_obj_t port = portx;
    if (mp_obj_is_small_int(port)) {
        // This is perverse, because lwip_getaddrinfo promptly converts it back to an integer, but
        // that's the API we have to work with ...
        port = mp_obj_str_binary_op(MP_BINARY_OP_MODULO, mp_obj_new_str_via_qstr("%s", 2), port);
    }

    const char *host_str = mp_obj_str_get_str(host);
    const char *port_str = mp_obj_str_get_str(port);

    if (host_str[0] == '\0') {
        // a host of "" is equivalent to the default/all-local IP address
        host_str = "0.0.0.0";
    }

    int res;
    MP_THREAD_GIL_EXIT();
    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        *resp = NULL;
        res = wifi_get_addrinfo(host_str, port_str, &hints, resp);
    }

    else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
        *resp = NULL;
        res = gsm_get_addrinfo(host_str, port_str, &hints, resp);
    }

    else {
        res = lwip_getaddrinfo(host_str, port_str, &hints, resp);
    }
    MP_THREAD_GIL_ENTER();

    return res;
}

//-----------------------------------------------------------------------
int _socket_getaddrinfo(const mp_obj_t addrtuple, struct addrinfo **resp)
{
    mp_uint_t len = 0;
    mp_obj_t *elem;
    mp_obj_get_array(addrtuple, &len, &elem);
    if (len != 2) return -1;
    return _socket_getaddrinfo2(elem[0], elem[1], resp);
}

// Only used for WiFi interface
//----------------------------------------------
static void _set_listen_mode(socket_obj_t *self)
{
    if (!self->listening) {
        int max_srv_n = AT_MAX_SERV_SOCKETS;
        int srv_n = max_srv_n;

        // find the free server socket
        for (srv_n=0; srv_n<max_srv_n; srv_n++) {
            if (at_server_socket[srv_n] == NULL) break;
        }
        if (srv_n == max_srv_n) {
            mp_raise_ValueError("Max number of listening sockets reached");
        }

        if ((self->max_conn < 1) || (self->max_conn >= AT_MAX_SOCKETS)) self->max_conn = 1;
        self->conn_id = 1;

        // create mutex and semaphore
        if (!self->semaphore) {
            self->semaphore = xSemaphoreCreateBinary();
            if (!self->semaphore) {
                mp_raise_ValueError("Error creating socket semaphore");
            }
        }
        if (!self->mutex) {
            self->mutex = xSemaphoreCreateMutex();
            if (!self->mutex) {
                mp_raise_ValueError("Error creating socket mutex");
            }
        }
        // mark as listening socket
        self->listening = true;

        // release the socket from list of client sockets
        if (self->fd >= 0) at_sockets[self->fd] = NULL;
        // assign the listening socket fd
        self->fd = AT_SERV_SOCK_FD_BASE + srv_n;

        for (int i=0; i<MAX_SERVER_CONNECTIONS; i++) {
            self->conn_fd[i] = -1;
        }
        // mark the socket as used in list of server sockets
        at_server_socket[srv_n] = self;
    }
}

//--------------------------------------------------------------
STATIC mp_obj_t socket_bind(size_t n_args, const mp_obj_t *args)
{
    check_net_interfaces();
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        int port = 0, tmo = 300;
        bool ssl = false;
        if (n_args > 2) {
            // the 3rd argument can be the connected socket's timeout
            // default value is 300 seconds
            tmo = mp_obj_get_int(args[2]);
            if (tmo < 5) tmo = 5;
            if (tmo > 7200) tmo = 7200;
        }
        // for WiFi interface, the 4th argument can be maximum number of allowed connections
        if (n_args > 3) {
            self->max_conn = mp_obj_get_int(args[3]);
            if ((self->max_conn < 1) || (self->max_conn >= AT_MAX_SOCKETS)) self->max_conn = 1;
        }

        // the 5th argument can be SSL mode
        if (n_args > 4) {
            ssl = mp_obj_is_true(args[4]);
        }

        // get the bind port, it can be given as address/port tuple or integer value
        if (mp_obj_is_type(args[1], &mp_type_tuple)) {
            mp_uint_t len = 0;
            mp_obj_t *elem;
            mp_obj_get_array(args[1], &len, &elem);
            if (len != 2) {
                mp_raise_msg(&mp_type_ValueError, "Wrong arguments");
            }
            port = mp_obj_get_int(elem[1]);
        }
        else port = mp_obj_get_int(args[1]);

        // ** Set the socket to listening mode if not already listening
        _set_listen_mode(self);

        int res = wifi_server(self, port, ssl, self->max_conn, tmo);
        if (res < 0) {
            exception_from_errno(errno);
        }
    }

    else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
        mp_raise_msg(&mp_type_NotImplementedError, "Not available in GSM IDLE mode");
    }

    else {
        // ---- lwip interface ----
        // Set the socket to listening mode if not listening
        if (!self->listening) {
            if ((self->max_conn < 1) || (self->max_conn > 4)) self->max_conn = 4;
            int r = lwip_listen(self->fd, self->max_conn);
            if (r < 0) exception_from_errno(errno);
            self->listening = true;
        }

        struct addrinfo *res;
        _socket_getaddrinfo(args[1], &res);

        int r = lwip_bind(self->fd, res->ai_addr, res->ai_addrlen);
        _socket_freeaddrinfo(res);
        if (r < 0) exception_from_errno(errno);

        struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
        self->bind_port = ntohs(addr->sin_port);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_bind_obj, 2, 5, socket_bind);

//----------------------------------------------------------------
STATIC mp_obj_t socket_listen(size_t n_args, const mp_obj_t *args)
{
    check_net_interfaces();
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int backlog = 1;

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        // for WiFi interface, 'backlog' argument is used as maximum number of allowed connections
        backlog = 2;
        if (n_args > 1) backlog = mp_obj_get_int(args[1]);
        if ((backlog > 0) && (backlog < AT_MAX_SOCKETS)) self->max_conn = backlog;
        else self->max_conn = 1;

        // Set the socket to listening mode
        _set_listen_mode(self);
    }

    else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
        mp_raise_msg(&mp_type_NotImplementedError, "Not available in GSM IDLE mode");
    }

    else {
        if (n_args > 1) backlog = mp_obj_get_int(args[1]);
        int r = lwip_listen(self->fd, backlog);
        if (r < 0) exception_from_errno(errno);
        self->listening = true;
        self->max_conn = backlog;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_listen_obj, 1, 2, socket_listen);

//------------------------------------------------------------
static mp_obj_t _socket_accept(const mp_obj_t arg0, bool raise)
{
    check_net_interfaces();
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);

    if (!self->listening) {
        mp_raise_ValueError("Not listening");
    }

    struct sockaddr addr;
    socklen_t addr_len = sizeof(addr);
    int new_fd = -1;
    size_t wait_start = mp_hal_ticks_ms();
    size_t wait_end = wait_start + self->timeout;

    if (self->bind_port <= 0) {
        mp_raise_msg(&mp_type_OSError, "Not binded");
    }

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        int con_n = -1, con_fd;
        //if (wifi_debug) LOGQ(TAG, "Accept: server %d (tmo=%lu, max_con=%d)", self->fd, self->timeout, self->max_conn);
        self->accepting = true;
        if (self->max_conn > 0) {
            // the connection may already be accepted by the wifi task
            // if this is the case we don't wait for the connection
            int fd = 9999;
            if (xSemaphoreTake(self->mutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                for (int i=0; i<MAX_SERVER_CONNECTIONS; i++) {
                    if ((self->conn_fd[i] >= 0) && (self->conn_fd[i] < AT_MAX_SOCKETS)) {
                        con_fd = self->conn_fd[i];
                        if ((at_sockets[con_fd]->parent_sock == self) && (at_sockets[con_fd]->conn_id >= 0) && (at_sockets[con_fd]->conn_id < fd)) {
                            fd = at_sockets[con_fd]->conn_id;
                            con_n = i;
                            new_fd = con_fd;
                        }
                    }
                }
                if (new_fd >= 0) {
                    self->conn_fd[con_n] = -1;
                    if (wifi_debug) LOGY(TAG, "Accept: connection ready, fd=%d", new_fd);
                }
                xSemaphoreGive(self->mutex);
            }

            if (new_fd < 0) {
                //if (wifi_debug) LOGQ(TAG, "Accept: wait");
                // === wait for connection ===
                while (mp_hal_ticks_ms() <= wait_end) {
                    // take the mutex (prevent WiFi task to update)
                    if (xSemaphoreTake(self->mutex, 100 / portTICK_PERIOD_MS) != pdTRUE) continue;
                    // check if the connection was accepted by WiFi task
                    fd = 9999;
                    for (int i=0; i<MAX_SERVER_CONNECTIONS; i++) {
                        if ((self->conn_fd[i] >= 0) && (self->conn_fd[i] < AT_MAX_SOCKETS)) {
                            con_fd = self->conn_fd[i];
                            if ((at_sockets[con_fd]->parent_sock == self) && (at_sockets[con_fd]->conn_id >= 0) && (at_sockets[con_fd]->conn_id < fd)) {
                                fd = at_sockets[con_fd]->conn_id;
                                con_n = i;
                                new_fd = con_fd;
                            }
                        }
                    }
                    if (new_fd >= 0) {
                        self->conn_fd[con_n] = -1;
                        if (wifi_debug) LOGY(TAG, "Accept: connection detected, fd=%d, time=%lu", new_fd, mp_hal_ticks_ms()-wait_start);
                    }
                    xSemaphoreGive(self->mutex);
                    if (new_fd >= 0) break;

                    MP_THREAD_GIL_EXIT();
                    // WiFi task will set the semaphore if the connection was accepted
                    if (self->semaphore) xSemaphoreTake(self->semaphore, 100 / portTICK_PERIOD_MS);
                    else vTaskDelay(10 / portTICK_PERIOD_MS);
                    MP_THREAD_GIL_ENTER();

                    check_for_exceptions();
                    mp_hal_wdt_reset();
                }
            }
        }
    }
    else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
        mp_raise_msg(&mp_type_NotImplementedError, "Not available in GSM IDLE mode");
    }
    else {
        while (mp_hal_ticks_ms() <= wait_end) {
            MP_THREAD_GIL_EXIT();
            new_fd = lwip_accept(self->fd, &addr, &addr_len);
            MP_THREAD_GIL_ENTER();
            if (new_fd >= 0) break;
            if (errno != EAGAIN) {
                exception_from_errno(errno);
            }
            check_for_exceptions();
            mp_hal_wdt_reset();
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
    }

    mp_obj_tuple_t *client = (mp_obj_tuple_t *)mp_obj_new_tuple(2, NULL);
    if (new_fd < 0) {
        //if (wifi_debug) LOGQ(TAG, "Accept: server %d, No connection", self->fd);
        // ==== Timeout, raise exception or prepare None result ===
        self->accepting = false;
        if (raise) {
            mp_raise_OSError(MP_ETIMEDOUT);
        }
        else {
            // ** Timeout, return None for sock object
            client->items[0] = mp_const_none;
            client->items[1] = mp_const_none;
        }
    }
    else {
        // ==== Connection accepted, prepare the result ====
        socket_obj_t *sock;
        if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
            // Socket is already created by WiFi task
            sock = at_sockets[new_fd];

            // Get socket's remote IP address and port
            char IP[32] = { '\0' };
            int port = 0, res = 0;

            if ((strlen(sock->remote_ip) > 0) && (sock->remote_port > 0)) {
                // IP & port prepared by WiFi task
                strcpy(IP, sock->remote_ip);
                port = sock->remote_port;
                if (wifi_debug) LOGY(TAG, "Accepted: fd=%d, link_id=%d, from %s:%d", new_fd, sock->link_id, IP, port);
                mp_obj_t tuple[2] = {
                    tuple[0] = mp_obj_new_str(IP, strlen(IP)),
                    tuple[1] = mp_obj_new_int(port),
                };
                client->items[0] = (mp_obj_t)sock;
                client->items[1] = mp_obj_new_tuple(2, tuple);
                self->accepting = false;
                res = 1;
            }
            else {
                // IP & port not yet available, we must get one for the connected socket
                self->accepting = false;
            }

            if ((res != 1) || (strlen(IP) == 0) || (port == 0)) {
                // No IP and/or port, raise exception or prepare the None result
                if (wifi_debug) LOGW(TAG, "Accepted: fd=%d, error getting IP & port", new_fd);
                if (raise) {
                    mp_raise_OSError(MP_ENOTCONN);
                }
                else {
                    // ** Timeout, return None for sock object
                    client->items[0] = mp_const_none;
                    client->items[1] = mp_const_none;
                }
            }
            sock->is_accepted = true;
        }
        else {
            // for lwip interface create the new socket object
            socket_obj_t *sock = _new_socket();
            // make the return value
            uint8_t *ip = (uint8_t*)&((struct sockaddr_in*)&addr)->sin_addr;
            mp_uint_t port = lwip_ntohs(((struct sockaddr_in*)&addr)->sin_port);
            snprintf(sock->remote_ip, 16, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
            sock->remote_port = port;
            client->items[0] = (mp_obj_t)sock;
            client->items[1] = netutils_format_inet_addr(ip, port, NETUTILS_BIG);
            self->accepting = false;
        }
    }

    return (mp_obj_t)client;
}

//------------------------------------------------
STATIC mp_obj_t socket_accept(const mp_obj_t arg0)
{
    return _socket_accept(arg0, true);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(socket_accept_obj, socket_accept);

// Same as socket_accept(), but does not raise exception if not accepted
// instead, it returns the (None, None) tuple if not accepted
//--------------------------------------------------
STATIC mp_obj_t socket_accepted(const mp_obj_t arg0)
{
    return _socket_accept(arg0, false);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(socket_accepted_obj, socket_accepted);

// socket.connect((host, port) [, local_port])
//-----------------------------------------------------------------
STATIC mp_obj_t socket_connect(size_t n_args, const mp_obj_t *args)
{
    check_net_interfaces();
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (self->listening) {
        mp_raise_ValueError("Socket in listening mode");
    }

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        mp_uint_t len = 0;
        mp_obj_t *elem;

        mp_obj_get_array(args[1], &len, &elem);
        if (len != 2) {
            mp_raise_msg(&mp_type_ValueError, "Wrong arguments");
        }

        // in WiFi mode we don't need to resolve the IP address
        // WiFi module can accept the domain name or IP address
        const char *host_str = mp_obj_str_get_str(elem[0]);
        int port = mp_obj_get_int(elem[1]);
        int local_port = 0;
        if (n_args >= 3) {
            // the 2nd argument can be the local port number
            local_port = mp_obj_get_int(args[2]);
            if ((local_port < 0) || (local_port > 65535)) {
                mp_raise_msg(&mp_type_ValueError, "Local port out of range");
            }
        }

        MP_THREAD_GIL_EXIT();
        int res = wifi_connect(self, host_str, port, local_port);
        MP_THREAD_GIL_ENTER();

        if (res < 0) {
            exception_from_errno(errno);
        }
        self->local_port = local_port;
        self->remote_port = port;
        return mp_const_none;
    }
    else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
        mp_raise_msg(&mp_type_NotImplementedError, "Not available in GSM IDLE mode");
    }

    struct addrinfo *res;
    _socket_getaddrinfo(args[1], &res);

    // Nonblocking
    MP_THREAD_GIL_EXIT();

    int r, opt;
    fd_set readset;
    fd_set writeset;
    fd_set errset;
    struct timeval tv;

    r = lwip_fcntl(self->fd, F_GETFL, 0);
    if (r < 0) {
        LOGE(TAG, "Error getting options");
        goto exit;
    }
    r |= O_NONBLOCK;
    r = lwip_fcntl(self->fd, F_SETFL, r);
    if (r < 0) {
        LOGE(TAG, "Error setting options");
        goto exit;
    }
    // connect
    r = lwip_connect(self->fd, res->ai_addr, res->ai_addrlen);
    // should have an error: "inprogress"

    FD_ZERO(&readset);
    FD_SET(self->fd, &readset);
    FD_ZERO(&writeset);
    FD_SET(self->fd, &writeset);
    FD_ZERO(&errset);
    FD_SET(self->fd, &errset);
    tv.tv_sec = 10;
    tv.tv_usec = 0;

    r = lwip_select(self->fd + 1, &readset, &writeset, &errset, &tv);
    //LOGM(TAG, "Select: (%d), %lu, %lu, %lu", r, FD_ISSET(self->fd, &writeset), FD_ISSET(self->fd, &readset), FD_ISSET(self->fd, &errset));
    if (r < 1) r = -1;

exit:
    if (self->timeout == SOCKET_TIMEOUT_MAX) {
        opt = lwip_fcntl(self->fd, F_GETFL, 0);
        if (opt < 0) {
            LOGE(TAG, "Exit: Error getting options");
        }
        else {
            opt &= ~O_NONBLOCK;
            opt = lwip_fcntl(self->fd, F_SETFL, r);
            if (opt < 0) {
                LOGE(TAG, "Exit: Error setting options");
            }
        }
    }

    MP_THREAD_GIL_ENTER();
    _socket_freeaddrinfo(res);
    if (r < 0) {
        exception_from_errno(errno);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_connect_obj, 2, 3, socket_connect);

//--------------------------------------------------------------------
STATIC mp_obj_t socket_setsockopt(size_t n_args, const mp_obj_t *args)
{
    check_net_interfaces();
    // for WiFi interface this method is not used
    if (!(net_active_interfaces & ACTIVE_INTERFACE_LWIP)) return mp_const_none;

    (void)n_args; // always 4
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    int opt = mp_obj_get_int(args[2]);

    switch (opt) {
        // level: SOL_SOCKET
        case SO_REUSEADDR: {
            int val = mp_obj_get_int(args[3]);
            int ret = lwip_setsockopt(self->fd, SOL_SOCKET, opt, &val, sizeof(int));
            if (ret != 0) {
                exception_from_errno(errno);
            }
            break;
        }

        #if MICROPY_PY_USOCKET_EVENTS
        // level: SOL_SOCKET
        // special "register callback" option
        case 20: {
            if (args[3] == mp_const_none) {
                if (self->events_callback != MP_OBJ_NULL) {
                    usocket_events_remove(self);
                    self->events_callback = MP_OBJ_NULL;
                }
            } else {
                if (self->events_callback == MP_OBJ_NULL) {
                    usocket_events_add(self);
                }
                self->events_callback = args[3];
            }
            break;
        }
        #endif

        // level: IPPROTO_IP
        case IP_ADD_MEMBERSHIP: {
            mp_buffer_info_t bufinfo;
            mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_READ);
            if (bufinfo.len != sizeof(ip4_addr_t) * 2) {
                mp_raise_ValueError(NULL);
            }

            // POSIX setsockopt has order: group addr, if addr, lwIP has it vice-versa
            err_t err = igmp_joingroup((const ip4_addr_t*)bufinfo.buf + 1, bufinfo.buf);
            if (err != ERR_OK) {
                mp_raise_OSError(-err);
            }
            break;
        }

        default:
            mp_printf(&mp_plat_print, "Warning: lwip.setsockopt() option not implemented\n");
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_setsockopt_obj, 4, 4, socket_setsockopt);

//--------------------------------------------------------------
void _socket_settimeout(socket_obj_t *sock, uint64_t timeout_ms)
{
    struct timeval timeout;
    uint32_t is_nonblocking = 0;
    if ((timeout_ms >= 10) && (timeout_ms <= SOCKET_TIMEOUT_MAX)) {
        is_nonblocking = O_NONBLOCK;
        sock->timeout = timeout_ms;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
    }
    else {
        sock->timeout = 10;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;
    }

    if (net_active_interfaces & ACTIVE_INTERFACE_LWIP) {
        lwip_setsockopt(sock->fd, SOL_SOCKET, SO_SNDTIMEO, (const void *)&timeout, sizeof(timeout));
        lwip_setsockopt(sock->fd, SOL_SOCKET, SO_RCVTIMEO, (const void *)&timeout, sizeof(timeout));
        int res = lwip_fcntl(sock->fd, F_GETFL, 0);
        res |= is_nonblocking;
        lwip_fcntl(sock->fd, F_SETFL, res);
    }
}

// Set or get socket's timeout
// The optional argument value is the timeout value in milli seconds
// Accepted range is 10 ~ 43200000 ms (0 ~ 12 hours)
// Returns the socket timeout in milli seconds
//-----------------------------------------------------------------
STATIC mp_obj_t socket_timeout(size_t n_args, const mp_obj_t *args)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args > 1) {
        mp_uint_t tmo = mp_obj_get_int(args[1]);
        if ((tmo >= 10) && (tmo <= SOCKET_TIMEOUT_MAX)) _socket_settimeout(self, tmo);
    }
    return mp_obj_new_int(self->timeout);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_timeout_obj, 1, 2, socket_timeout);

// Set the socket's timeout
// The float argument value is the timeout value in seconds
// Accepted range is 0.01 ~ 43200.0 s (0 ~ 12 hours)
//------------------------------------------------------------------------------
STATIC mp_obj_t socket_settimeout(const mp_obj_t self_in, const mp_obj_t tmo_in)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_float_t tmo = mp_obj_get_float(tmo_in);
    if ((tmo >= 0.0) && (tmo <= 43200.0)) _socket_settimeout(self, (mp_uint_t)(tmo * 1000.0));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_settimeout_obj, socket_settimeout);

//--------------------------------------------------------------------------
STATIC mp_obj_t socket_setblocking(const mp_obj_t arg0, const mp_obj_t arg1)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);
    if (mp_obj_is_true(arg1)) _socket_settimeout(self, SOCKET_TIMEOUT_MAX);
    else _socket_settimeout(self, 10);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_setblocking_obj, socket_setblocking);

//--------------------------------------------------------------------------
STATIC mp_uint_t _socket_read_data(mp_obj_t self_in, void *buf, size_t size,
    struct sockaddr *from, socklen_t *from_len, int *errcode)
{
    check_net_interfaces();
    socket_obj_t *sock = MP_OBJ_TO_PTR(self_in);
    int wait_end = mp_hal_ticks_ms() + sock->timeout;

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        if (sock->listening) {
            mp_raise_ValueError("Socket in listening mode");
        }

        if (wifi_task_semaphore) wifi_task_semaphore_active = true;
        int r = 0;
        if ((wifi_debug) && (size >= 256)) LOGD(TAG, "Read (%lu)", size);
        while (mp_hal_ticks_ms() <= wait_end) {
            MP_THREAD_GIL_EXIT();
            r = wifi_read(sock, buf, size);
            MP_THREAD_GIL_ENTER();
            if (wifi_task_semaphore) xSemaphoreGive(wifi_task_semaphore);

            if (r >= 0) {
                if (wifi_task_semaphore) wifi_task_semaphore_active = false;
                if ((wifi_debug) && (size >= 256)) LOGD(TAG, "Read: Got %d byte(s)", r);
                return r;
            }
            if (errno != EWOULDBLOCK) {
                if (wifi_task_semaphore) wifi_task_semaphore_active = false;
                if (wifi_debug) LOGE(TAG, "wifi_read returned error");
                *errcode = errno;
                return MP_STREAM_ERROR;
            }
            check_for_exceptions();
            vTaskDelay(50);
            mp_hal_wdt_reset();
        }

        if (wifi_task_semaphore) wifi_task_semaphore_active = false;
        if ((wifi_debug) && (size >= 256)) LOGY(TAG, "Read: timeout (%lu)", sock->timeout);
        *errcode = (mp_hal_ticks_ms() <= wait_end) ? MP_EWOULDBLOCK : MP_ETIMEDOUT;
        return MP_STREAM_ERROR;
    }
    else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
        mp_raise_msg(&mp_type_NotImplementedError, "Not available in GSM IDLE mode");
    }

    // If the peer closed the connection then the lwIP socket API will only return "0" once
    // from lwip_recvfrom and then block on subsequent calls.  To emulate POSIX behaviour,
    // which continues to return "0" for each call on a closed socket, we set a flag when
    // the peer closed the socket.
    if (sock->peer_closed) {
        return 0;
    }

    int r = 0;

    while (mp_hal_ticks_ms() <= wait_end) {
        MP_THREAD_GIL_EXIT();
        r = lwip_recvfrom(sock->fd, buf, size, 0, from, from_len);
        MP_THREAD_GIL_ENTER();
        if (r == 0) sock->peer_closed = true;
        if (r >= 0) return r;
        if (errno != EWOULDBLOCK) {
            *errcode = errno;
            return MP_STREAM_ERROR;
        }
        check_for_exceptions();
        vTaskDelay(5);
        mp_hal_wdt_reset();
    }

    *errcode = (mp_hal_ticks_ms() <= wait_end) ? MP_EWOULDBLOCK : MP_ETIMEDOUT;
    return MP_STREAM_ERROR;
}

// This is much more efficient way to read line than using the stream
//-----------------------------------------------------------------------
STATIC mp_obj_t socket_wifi_readline(size_t n_args, const mp_obj_t *args)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        int wait_end = mp_hal_ticks_ms() + self->timeout;
        char lend[2] = {'\0'};
        const char *p_lend;
        char *p_outstr = NULL;
        int size = -1;
        mp_obj_t res_obj = mp_const_empty_bytes;;

        if (n_args > 1) {
            p_lend = mp_obj_str_get_str(args[1]);
        }
        else {
            lend[0] = '\n';
            p_lend = lend;
        }

        //if (wifi_debug) LOGY(TAG, "Readline: [%s]", p_lend);
        if (wifi_task_semaphore) wifi_task_semaphore_active = true;

        while (mp_hal_ticks_ms() <= wait_end) {
            MP_THREAD_GIL_EXIT();
            p_outstr = wifi_read_lineend(self, p_lend, &size);
            MP_THREAD_GIL_ENTER();
            if (wifi_task_semaphore) xSemaphoreGive(wifi_task_semaphore);

            if (p_outstr != NULL) break;
            vTaskDelay(50);
            mp_hal_wdt_reset();
        }

        if (wifi_task_semaphore) wifi_task_semaphore_active = false;
        if (p_outstr != NULL) {
            res_obj = mp_obj_new_str_of_type(&mp_type_bytes, (const byte*)p_outstr, size);
            vPortFree(p_outstr);
        }
        //if (wifi_debug) LOGY(TAG, "Readline: size=%d", size);

        return res_obj;
    }
    else {
        mp_raise_msg(&mp_type_NotImplementedError, "Only available in WiFi mode");
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_wifi_readline_obj, 1, 2, socket_wifi_readline);

//------------------------------------------------------------------------------------------------------
mp_obj_t _socket_recvfrom(mp_obj_t self_in, mp_obj_t len_in, struct sockaddr *from, socklen_t *from_len)
{
    size_t len = mp_obj_get_int(len_in);
    vstr_t vstr;
    vstr_init_len(&vstr, len);

    int errcode;
    mp_uint_t ret = _socket_read_data(self_in, vstr.buf, len, from, from_len, &errcode);
    if (ret == MP_STREAM_ERROR) {
        exception_from_errno(errcode);
    }

    vstr.len = ret;
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

//------------------------------------------------------------
STATIC mp_obj_t socket_recv(mp_obj_t self_in, mp_obj_t len_in)
{
    return _socket_recvfrom(self_in, len_in, NULL, NULL);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_recv_obj, socket_recv);

//----------------------------------------------------------------
STATIC mp_obj_t socket_recvfrom(mp_obj_t self_in, mp_obj_t len_in)
{
    check_net_interfaces();
    if (!(net_active_interfaces & ACTIVE_INTERFACE_LWIP)) {
        mp_raise_msg(&mp_type_NotImplementedError, "Only available in LWIP mode");
    }
    struct sockaddr from;
    socklen_t fromlen = sizeof(from);

    mp_obj_t tuple[2];
    tuple[0] = _socket_recvfrom(self_in, len_in, &from, &fromlen);

    uint8_t *ip = (uint8_t*)&((struct sockaddr_in*)&from)->sin_addr;
    mp_uint_t port = lwip_ntohs(((struct sockaddr_in*)&from)->sin_port);
    tuple[1] = netutils_format_inet_addr(ip, port, NETUTILS_BIG);

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_recvfrom_obj, socket_recvfrom);

//--------------------------------------------------------------------
int _socket_send(socket_obj_t *sock, const char *data, size_t datalen)
{
    int sentlen = 0;
    int wait_end = mp_hal_ticks_ms() + sock->timeout;
    int r = 0;

    check_net_interfaces();
    if (sock->listening) {
        mp_raise_ValueError("Socket in listening mode");
    }

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        if (wifi_debug) LOGQ(TAG, "Send (%lu)", datalen);
        MP_THREAD_GIL_EXIT();
        sentlen = wifi_send(sock, data, datalen);
        MP_THREAD_GIL_ENTER();
    }
    else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
        mp_raise_msg(&mp_type_NotImplementedError, "Not available in GSM IDLE mode");
    }
    else {
        while ((mp_hal_ticks_ms() <= wait_end) && (sentlen < datalen)) {
            MP_THREAD_GIL_EXIT();
            r = lwip_write(sock->fd, data+sentlen, datalen-sentlen);
            MP_THREAD_GIL_ENTER();

            if ((r < 0) && (errno != EWOULDBLOCK)) exception_from_errno(errno);
            if (r > 0) sentlen += r;
            check_for_exceptions();
            vTaskDelay(2);
            mp_hal_wdt_reset();
        }
    }
    if (sentlen == 0) mp_raise_OSError(MP_ETIMEDOUT); 

    return sentlen;
}

//-------------------------------------------------------------------
STATIC mp_obj_t socket_send(const mp_obj_t arg0, const mp_obj_t arg1)
{
    socket_obj_t *sock = MP_OBJ_TO_PTR(arg0);
    mp_uint_t datalen;
    const char *data = mp_obj_str_get_data(arg1, &datalen);
    int r = _socket_send(sock, data, datalen);

    return mp_obj_new_int(r);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_send_obj, socket_send);

//----------------------------------------------------------------------
STATIC mp_obj_t socket_sendall(const mp_obj_t arg0, const mp_obj_t arg1)
{
    // XXX behaviour when nonblocking (see extmod/modlwip.c)
    // XXX also timeout behaviour.
    socket_obj_t *sock = MP_OBJ_TO_PTR(arg0);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg1, &bufinfo, MP_BUFFER_READ);

    int r = _socket_send(sock, bufinfo.buf, bufinfo.len);
    if (r < bufinfo.len) mp_raise_OSError(MP_ETIMEDOUT);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_sendall_obj, socket_sendall);

//---------------------------------------------------------------------------------
STATIC mp_obj_t socket_sendto(mp_obj_t self_in, mp_obj_t data_in, mp_obj_t addr_in)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(self_in);

    check_net_interfaces();

    int ret = 0;
    // get the buffer to send
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);

    if (!(net_active_interfaces & ACTIVE_INTERFACE_LWIP)) {
        //mp_raise_msg(&mp_type_NotImplementedError, "Only available in LWIP mode");
        mp_uint_t len = 0;
        mp_obj_t *elem;
        mp_obj_get_array(addr_in, &len, &elem);
        if (len != 2) {
            mp_raise_msg(&mp_type_ValueError, "Wrong arguments");
        }
        const char *host_str = mp_obj_str_get_str(elem[0]);
        int port = mp_obj_get_int(elem[1]);

        MP_THREAD_GIL_EXIT();
        ret = wifi_send_to(self, bufinfo.buf, bufinfo.len, host_str, port);
        MP_THREAD_GIL_ENTER();
        if (ret > 0) return mp_obj_new_int_from_uint(ret);
        return mp_obj_new_int_from_uint(-1);
    }

    // create the destination address
    struct sockaddr_in to;
    to.sin_len = sizeof(to);
    to.sin_family = AF_INET;
    to.sin_port = lwip_htons(netutils_parse_inet_addr(addr_in, (uint8_t*)&to.sin_addr, NETUTILS_BIG));

    // send the data
    int wait_end = mp_hal_ticks_ms() + self->timeout;

    while (mp_hal_ticks_ms() <= wait_end) {
        MP_THREAD_GIL_EXIT();
        ret = lwip_sendto(self->fd, bufinfo.buf, bufinfo.len, 0, (struct sockaddr*)&to, sizeof(to));
        MP_THREAD_GIL_ENTER();
        if (ret > 0) return mp_obj_new_int_from_uint(ret);
        if ((ret == -1) && (errno != EWOULDBLOCK)) {
            exception_from_errno(errno);
        }
        check_for_exceptions();
        vTaskDelay(2);
        mp_hal_wdt_reset();
    }
    mp_raise_OSError(MP_ETIMEDOUT);
    return mp_obj_new_int_from_uint(-1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(socket_sendto_obj, socket_sendto);

//------------------------------------------------
STATIC mp_obj_t socket_fileno(const mp_obj_t arg0)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);

    return mp_obj_new_int(self->fd);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(socket_fileno_obj, socket_fileno);

//------------------------------------------------------------------
STATIC mp_obj_t socket_makefile(size_t n_args, const mp_obj_t *args)
{
    (void)n_args;
    return args[0];
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(socket_makefile_obj, 1, 3, socket_makefile);

//--------------------------------------------------------------------------------------------
STATIC mp_uint_t socket_stream_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode)
{
    return _socket_read_data(self_in, buf, size, NULL, NULL, errcode);
}

//---------------------------------------------------------------------------------------------------
STATIC mp_uint_t socket_stream_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode)
{
    socket_obj_t *sock = (socket_obj_t *)self_in;
    int r = 0;

    if ((net_active_interfaces == ACTIVE_INTERFACE_NONE) ||
        (net_active_interfaces & ACTIVE_INTERFACE_GSM) ||
        (sock->listening)) {
        *errcode = ENOTSOCK;
        return MP_STREAM_ERROR;
    }

    if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
        MP_THREAD_GIL_EXIT();
        r = wifi_send(sock, buf, size);
        MP_THREAD_GIL_ENTER();

        if (r > 0) return r;
        *errcode = errno;
    }
    else {
        int wait_end = mp_hal_ticks_ms() + sock->timeout;
        while (mp_hal_ticks_ms() <= wait_end) {
            MP_THREAD_GIL_EXIT();
            r = lwip_write(sock->fd, buf, size);
            MP_THREAD_GIL_ENTER();

            if (r > 0) return r;

            if ((r < 0) && (errno != EWOULDBLOCK)) {
                *errcode = errno;
                return MP_STREAM_ERROR;
            }
            check_for_exceptions();
            vTaskDelay(2);
            mp_hal_wdt_reset();
        }
        *errcode = (mp_hal_ticks_ms() <= wait_end) ? MP_EWOULDBLOCK : MP_ETIMEDOUT;
    }
    return MP_STREAM_ERROR;
}

//----------------------------------------------------------------------------------------------------
STATIC mp_uint_t socket_stream_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode)
{
    socket_obj_t * socket = (socket_obj_t *)self_in;
    if (request == MP_STREAM_POLL) {
        mp_uint_t ret = 0;
        struct timeval timeout = { .tv_sec = 0, .tv_usec = 0 };

        if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
            if (arg & MP_STREAM_POLL_RD) {
                if (socket->buffer.length > 0) ret |= MP_STREAM_POLL_RD;
            }
            if (arg & MP_STREAM_POLL_WR) ret |= MP_STREAM_POLL_WR;
        }
        else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
            if (arg & MP_STREAM_POLL_HUP) ret |= MP_STREAM_POLL_HUP;
        }
        else {
            fd_set rfds; FD_ZERO(&rfds);
            fd_set wfds; FD_ZERO(&wfds);
            fd_set efds; FD_ZERO(&efds);
            if (arg & MP_STREAM_POLL_RD) FD_SET(socket->fd, &rfds);
            if (arg & MP_STREAM_POLL_WR) FD_SET(socket->fd, &wfds);
            if (arg & MP_STREAM_POLL_HUP) FD_SET(socket->fd, &efds);

            int r = lwip_select((socket->fd)+1, &rfds, &wfds, &efds, &timeout);
            if (r < 0) {
                *errcode = MP_EIO;
                return MP_STREAM_ERROR;
            }

            if (FD_ISSET(socket->fd, &rfds)) ret |= MP_STREAM_POLL_RD;
            if (FD_ISSET(socket->fd, &wfds)) ret |= MP_STREAM_POLL_WR;
            if (FD_ISSET(socket->fd, &efds)) ret |= MP_STREAM_POLL_HUP;
        }
        return ret;
    }
    else if (request == MP_STREAM_CLOSE) {
        if ((net_active_interfaces != ACTIVE_INTERFACE_NONE) && (socket->fd >= 0)) {
            int ret = 0;
            if (net_active_interfaces & ACTIVE_INTERFACE_WIFI) {
                if (wifi_debug) LOGY(TAG, "Close socket %d", socket->fd);
                /*
                // Do not close the socket connected to the listening socket
                // if it was not accepted !
                if ((socket->parent_sock != NULL) && (!socket->is_accepted)) {
                    if (wifi_debug) LOGQ(TAG, "not closed, socket waiting to be accepted");
                    return 0;
                }
                */
                ret = wifi_close(socket);
            }
            else if (net_active_interfaces & ACTIVE_INTERFACE_GSM) {
                ret = 0;
            }
            else {
                #if MICROPY_PY_USOCKET_EVENTS
                if (socket->events_callback != MP_OBJ_NULL) {
                    usocket_events_remove(socket);
                    socket->events_callback = MP_OBJ_NULL;
                }
                #endif
                ret = lwip_close(socket->fd);
            }
            if (ret != 0) {
                *errcode = errno;
                return MP_STREAM_ERROR;
            }
            socket->fd = -1;
        }
        return 0;
    }
    else if (request == MP_STREAM_GET_FILENO) {
        return socket->fd;
    }

    *errcode = MP_EINVAL;
    return MP_STREAM_ERROR;
}

//-----------------------------------------------
STATIC mp_obj_t socket_getfd(const mp_obj_t arg0)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);

    return mp_obj_new_int(self->fd);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(socket_getfd_obj, socket_getfd);

//--------------------------------------------------------------------
STATIC mp_obj_t socket_callback(const mp_obj_t arg0, mp_obj_t cb_func)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);

    if ((mp_obj_is_fun(cb_func)) || (mp_obj_is_meth(cb_func)) || (cb_func == mp_const_none)) {
        self->cb = cb_func;
    }
    else {
        mp_raise_ValueError("Function or None argument expected");
    }

    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(socket_callback_obj, socket_callback);

//-----------------------------------------------------
STATIC mp_obj_t socket_peer_closed(const mp_obj_t arg0)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);

    if (self->peer_closed) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(socket_peer_closed_obj, socket_peer_closed);

//------------------------------------------------
STATIC mp_obj_t socket_in_buf(const mp_obj_t arg0)
{
    socket_obj_t *self = MP_OBJ_TO_PTR(arg0);

    return mp_obj_new_int(self->buffer.length);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(socket_in_buf_obj, socket_in_buf);

//-----------------------------------------------------------------------------------------
STATIC void socket_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    socket_obj_t * sock = (socket_obj_t *)self_in;

    if (sock->fd < 0) {
        mp_printf(print, "Socket ( Closed )\r\n");
        return;
    }

    char domain[16];
    char type[16];
    char proto[16];
    if (sock->domain == AF_INET) sprintf(domain, "AF_INET");
    else if (sock->domain == AF_INET6) sprintf(domain, "AF_INET6");
    else sprintf(domain, "Unknown");

    if (sock->type == SOCK_STREAM) sprintf(type, "SOCK_STREAM");
    else if (sock->type == SOCK_DGRAM) sprintf(type, "SOCK_DGRAM");
    else if (sock->type == SOCK_RAW) sprintf(type, "SOCK_RAW");
    else sprintf(type, "Unknown");

    if (sock->proto == IPPROTO_TCP) sprintf(proto, "IPPROTO_TCP");
    else if (sock->proto == IPPROTO_UDP) sprintf(proto, "IPPROTO_UDP");
    else if (sock->proto == WIFI_IPPROTO_SSL) sprintf(proto, "IPPROTO_SSL");
    else sprintf(proto, "Unknown");

    mp_printf(print, "Socket (fd=%d, link_id=%d, domain=%s, type=%s, proto=%s%s\r\n",
            sock->fd, sock->link_id, domain, type, proto, (sock->parent_sock) ? ", connected from listening socket" : "");
    mp_printf(print, "        timeout=%d, peer_closed=%s, buffer=%s\r\n",
            sock->timeout, (sock->peer_closed) ? "True" : "False", (sock->buffer.buf) ? "Yes" : "No");
    if (sock->connect_time > 0) {
        mp_printf(print, "        connected=%s, connect_time=%lu ms\r\n",
                (sock->peer_closed) ? "False" : "True",
                (sock->connected_time > 0) ? sock->connected_time : (mp_hal_ticks_ms() - sock->connect_time));
    }
    if (sock->buffer.buf) {
        mp_printf(print, "        buf_size=%d, buf_length=%d, buf_owerflow=%d\r\n", sock->buffer.size, sock->buffer.length, sock->buffer.overflow);
    }
    if (sock->listening) {
        mp_printf(print, "        Listening");
        if (sock->bind_port > 0) mp_printf(print, " on port %d", sock->bind_port);
        mp_printf(print, ", active connections: %d of %d\r\n", sock->active_conn, sock->max_conn);
    }
    mp_printf(print, "       )\r\n");
}


//===========================================================
STATIC const mp_rom_map_elem_t socket_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__),         MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),           MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_bind),            MP_ROM_PTR(&socket_bind_obj) },
    { MP_ROM_QSTR(MP_QSTR_listen),          MP_ROM_PTR(&socket_listen_obj) },
    { MP_ROM_QSTR(MP_QSTR_accept),          MP_ROM_PTR(&socket_accept_obj) },
    { MP_ROM_QSTR(MP_QSTR_accepted),        MP_ROM_PTR(&socket_accepted_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect),         MP_ROM_PTR(&socket_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),            MP_ROM_PTR(&socket_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_sendall),         MP_ROM_PTR(&socket_sendall_obj) },
    { MP_ROM_QSTR(MP_QSTR_sendto),          MP_ROM_PTR(&socket_sendto_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv),            MP_ROM_PTR(&socket_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_recvfrom),        MP_ROM_PTR(&socket_recvfrom_obj) },
    { MP_ROM_QSTR(MP_QSTR_setsockopt),      MP_ROM_PTR(&socket_setsockopt_obj) },
    { MP_ROM_QSTR(MP_QSTR_settimeout),      MP_ROM_PTR(&socket_settimeout_obj) },
    { MP_ROM_QSTR(MP_QSTR_timeout),         MP_ROM_PTR(&socket_timeout_obj) },
    { MP_ROM_QSTR(MP_QSTR_setblocking),     MP_ROM_PTR(&socket_setblocking_obj) },
    { MP_ROM_QSTR(MP_QSTR_makefile),        MP_ROM_PTR(&socket_makefile_obj) },
    { MP_ROM_QSTR(MP_QSTR_fileno),          MP_ROM_PTR(&socket_fileno_obj) },
    { MP_ROM_QSTR(MP_QSTR_peer_closed),     MP_ROM_PTR(&socket_peer_closed_obj) },
    { MP_ROM_QSTR(MP_QSTR_callback),        MP_ROM_PTR(&socket_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_inbuf),           MP_ROM_PTR(&socket_in_buf_obj) },
    { MP_ROM_QSTR(MP_QSTR_getFD),           MP_ROM_PTR(&socket_getfd_obj) },
    { MP_ROM_QSTR(MP_QSTR_wifi_readline),   MP_ROM_PTR(&socket_wifi_readline_obj) },

    { MP_ROM_QSTR(MP_QSTR_read),            MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto),        MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline),        MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&mp_stream_write_obj) },
};
STATIC MP_DEFINE_CONST_DICT(socket_locals_dict, socket_locals_dict_table);

//============================================
STATIC const mp_stream_p_t socket_stream_p = {
    .read = socket_stream_read,
    .write = socket_stream_write,
    .ioctl = socket_stream_ioctl
};

//========================================
STATIC const mp_obj_type_t socket_type = {
    { &mp_type_type },
    .name = MP_QSTR_socket,
    .print = socket_print,
    .protocol = &socket_stream_p,
    .locals_dict = (mp_obj_dict_t *)&socket_locals_dict,
};


//-------------------------
socket_obj_t *_new_socket()
{
    socket_obj_t *sock = m_new_obj_with_finaliser(socket_obj_t);
    sock->base.type = &socket_type;
    sock->fd = -1;
    sock->link_id = -1;
    sock->conn_id = -1;
    sock->domain = AF_INET;
    sock->type = SOCK_STREAM;
    sock->proto = IPPROTO_TCP;
    sock->buffer.uart_num = 255;
    sock->static_buffer = mp_const_none;
    sock->cb = mp_const_none;
    sock->max_conn = 1;
    sock->active_conn = 0;
    sock->bind_port = -1;
    for (int i=0; i<MAX_SERVER_CONNECTIONS; i++) {
        sock->conn_fd[i] = -1;
    }
    sock->peer_closed = false;
    sock->buffer.buf = NULL;
    sock->buffer.size = 0;
    sock->buffer.head = 0;
    sock->buffer.tail = 0;
    sock->buffer.length = 0;
    sock->semaphore = NULL;
    sock->mutex = NULL;
    sock->connect_time = 0;
    sock->connected_time = 0;
    sock->total_received = 0;
    sock->listening = false;
    sock->accepting = false;
    sock->is_accepted = false;
    sock->parent_sock = NULL;
    memset(sock->remote_ip, 0, sizeof(sock->remote_ip));
    sock->remote_port = 0;

    _socket_settimeout(sock, SOCKET_TIMEOUT_MAX);

    return sock;
}

//---------------------------------------------------------------------------------------
STATIC mp_obj_t get_socket(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
            { MP_QSTR_domain,                     MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_type,                       MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_proto,                      MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_bufsize,  MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_cb,       MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    socket_obj_t *sock = _new_socket();

    if (args[0].u_obj != mp_const_none) sock->domain = mp_obj_get_int(args[0].u_obj);
    if (args[1].u_obj != mp_const_none) sock->type = mp_obj_get_int(args[1].u_obj);
    if (args[2].u_obj != mp_const_none) sock->proto = mp_obj_get_int(args[2].u_obj);

    if ((net_active_interfaces & ACTIVE_INTERFACE_WIFI) || (net_active_interfaces & ACTIVE_INTERFACE_GSM)) {
        if (args[3].u_obj != mp_const_none) {
            // 4th argument can be buffer size
            int buf_size = mp_obj_get_int(args[3].u_obj);
            vstr_t vstr;
            vstr_init(&vstr, buf_size);
            vstr.len = buf_size-1;
            sock->buffer.buf = (uint8_t *)vstr.buf;
            sock->buffer.size = buf_size-1;
            sock->buffer.head = 0;
            sock->buffer.tail = 0;
            sock->buffer.length = 0;
            sock->buffer.uart_num = 255;
            sock->static_buffer = mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
        sock->fd = at_get_socket(sock);
        sock->link_id = sock->fd;
        if ((mp_obj_is_fun(args[4].u_obj)) || (mp_obj_is_meth(args[4].u_obj))) {
            // register socket's callback method
            sock->cb = args[4].u_obj;
        }
    }
    else {
        sock->fd = lwip_socket(sock->domain, sock->type, sock->proto);
    }
    if (sock->fd < 0) {
        exception_from_errno(errno);
    }

    return MP_OBJ_FROM_PTR(sock);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(get_socket_obj, 0, get_socket);

//-------------------------------------------------------------------------
STATIC mp_obj_t mod_socket_getaddrinfo(size_t n_args, const mp_obj_t *args)
{
    // TODO support additional args beyond the first two

    struct addrinfo *res = NULL;
    _socket_getaddrinfo2(args[0], args[1], &res);
    mp_obj_t ret_list = mp_obj_new_list(0, NULL);

    for (struct addrinfo *resi = res; resi; resi = resi->ai_next) {
        mp_obj_t addrinfo_objs[5] = {
            mp_obj_new_int(resi->ai_family),
            mp_obj_new_int(resi->ai_socktype),
            mp_obj_new_int(resi->ai_protocol),
            mp_obj_new_str(resi->ai_canonname, strlen(resi->ai_canonname)),
            mp_const_none
        };
        
        if (resi->ai_family == AF_INET) {
            struct sockaddr_in *addr = (struct sockaddr_in *)resi->ai_addr;
            // This looks odd, but it's really just a u32_t
            ip4_addr_t ip4_addr = { .addr = addr->sin_addr.s_addr };
            char buf[16];
            ip4addr_ntoa_r(&ip4_addr, buf, sizeof(buf));
            mp_obj_t inaddr_objs[2] = {
                mp_obj_new_str(buf, strlen(buf)),
                mp_obj_new_int(ntohs(addr->sin_port))
            };
            addrinfo_objs[4] = mp_obj_new_tuple(2, inaddr_objs);
        }
        mp_obj_list_append(ret_list, mp_obj_new_tuple(5, addrinfo_objs));
    }

    if (res) {
        _socket_freeaddrinfo(res);
    }
    return ret_list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_socket_getaddrinfo_obj, 2, 6, mod_socket_getaddrinfo);

//-------------------------------------
STATIC mp_obj_t mod_socket_initialize()
{
    if (!tcpip_adapter_initialized) {
        LOGD(TAG, "Network init (from socket module)");
        network_init();
        tcpip_adapter_initialized = true;
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_socket_initialize_obj, mod_socket_initialize);

//-------------------------------
STATIC mp_obj_t mod_socket_info()
{
    if (!(net_active_interfaces & ACTIVE_INTERFACE_WIFI)) {
        mp_raise_ValueError("Available only for WiFi interface");
    }

    int n = 0;
    int max_srv_n = AT_MAX_SERV_SOCKETS;

    for (int i=0; i<AT_MAX_SOCKETS; i++) {
        if (at_sockets[i] != NULL) n++;
    }
    mp_printf(&mp_plat_print, "Client sockets: %d\r\n", n);
    for (int i=0; i<AT_MAX_SOCKETS; i++) {
        if (at_sockets[i] != NULL) {
            if (at_sockets[i]->parent_sock) {
                bool f = false;
                for (int srv_n=0; srv_n<max_srv_n; srv_n++) {
                    if ((at_server_socket[srv_n] != NULL) && (at_server_socket[srv_n] == at_sockets[i])) f=true;
                }
                if (!f) at_sockets[i]->parent_sock = NULL;
            }
            socket_print(&mp_plat_print, at_sockets[i], PRINT_STR);
        }
    }

    n = 0;
    for (int srv_n=0; srv_n<max_srv_n; srv_n++) {
        if (at_server_socket[srv_n] != NULL) n++;
    }
    mp_printf(&mp_plat_print, "\r\nServer sockets: %d\r\n", n);
    for (int srv_n=0; srv_n<max_srv_n; srv_n++) {
        if (at_server_socket[srv_n] != NULL) {
            socket_print(&mp_plat_print, at_server_socket[srv_n], PRINT_STR);
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_socket_info_obj, mod_socket_info);

//----------------------------------------------------
STATIC mp_obj_t mod_socket_closesocket(mp_obj_t fd_in)
{
    if (!(net_active_interfaces & ACTIVE_INTERFACE_WIFI)) {
        mp_raise_ValueError("Available only for WiFi interface");
    }
    int fd = mp_obj_get_int(fd_in);
    int res = 0;
    if (fd < AT_SERV_SOCK_FD_BASE) {
        for (int i=0; i<AT_MAX_SOCKETS; i++) {
            if (at_sockets[i] != NULL) {
                if (at_sockets[i]->fd == fd) {
                    res = wifi_close(at_sockets[i]);
                    if (res == 0) {
                        at_sockets[i]->fd = -1;
                        if (wifi_debug) LOGM(TAG, "Closed socket %d", fd);
                    }
                    else if (wifi_debug) LOGM(TAG, "Error closing socket %d", fd);
                    break;
                }
            }
        }
    }
    else {
        int res;
        int max_srv_n = AT_MAX_SERV_SOCKETS;
        int srv_n = max_srv_n;
        for (srv_n=0; srv_n<max_srv_n; srv_n++) {
            if (at_server_socket[srv_n] != NULL) {
                if (at_server_socket[srv_n]->fd == fd) {
                    res = wifi_close(at_server_socket[srv_n]);
                    if (res == 0) {
                        at_sockets[srv_n]->fd = -1;
                        if (wifi_debug) LOGM(TAG, "Closed server socket %d", fd);
                    }
                    else if (wifi_debug) LOGM(TAG, "Error closing server socket %d", fd);
                    break;
                }
            }
        }
    }
    return (res == 0) ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_socket_closesocket_obj, mod_socket_closesocket);


//=================================================================
STATIC const mp_rom_map_elem_t mp_module_socket_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_usocket) },
    { MP_ROM_QSTR(MP_QSTR___init__),            MP_ROM_PTR(&mod_socket_initialize_obj) },
    { MP_ROM_QSTR(MP_QSTR_socket),              MP_ROM_PTR(&get_socket_obj) },
    { MP_ROM_QSTR(MP_QSTR_getaddrinfo),         MP_ROM_PTR(&mod_socket_getaddrinfo_obj) },
    { MP_ROM_QSTR(MP_QSTR_check),               MP_ROM_PTR(&mod_socket_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),               MP_ROM_PTR(&mod_socket_closesocket_obj) },

    { MP_ROM_QSTR(MP_QSTR_AF_INET),             MP_ROM_INT(AF_INET) },
    { MP_ROM_QSTR(MP_QSTR_AF_INET6),            MP_ROM_INT(AF_INET6) },
    { MP_ROM_QSTR(MP_QSTR_SOCK_STREAM),         MP_ROM_INT(SOCK_STREAM) },
    { MP_ROM_QSTR(MP_QSTR_SOCK_DGRAM),          MP_ROM_INT(SOCK_DGRAM) },
    { MP_ROM_QSTR(MP_QSTR_SOCK_RAW),            MP_ROM_INT(SOCK_RAW) },
    { MP_ROM_QSTR(MP_QSTR_IPPROTO_TCP),         MP_ROM_INT(IPPROTO_TCP) },
    { MP_ROM_QSTR(MP_QSTR_IPPROTO_UDP),         MP_ROM_INT(IPPROTO_UDP) },
    { MP_ROM_QSTR(MP_QSTR_IPPROTO_IP),          MP_ROM_INT(IPPROTO_IP) },
    { MP_ROM_QSTR(MP_QSTR_IPPROTO_SSL),         MP_ROM_INT(WIFI_IPPROTO_SSL) },
    { MP_ROM_QSTR(MP_QSTR_SOL_SOCKET),          MP_ROM_INT(SOL_SOCKET) },
    { MP_ROM_QSTR(MP_QSTR_SO_REUSEADDR),        MP_ROM_INT(SO_REUSEADDR) },
    { MP_ROM_QSTR(MP_QSTR_IP_ADD_MEMBERSHIP),   MP_ROM_INT(IP_ADD_MEMBERSHIP) },
};
STATIC MP_DEFINE_CONST_DICT(mp_module_socket_globals, mp_module_socket_globals_table);

//=========================================
const mp_obj_module_t mp_module_usocket = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_socket_globals,
};
