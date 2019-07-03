/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
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

#ifndef _LIBAT_UTIL_H_
#define _LIBAT_UTIL_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "netif/ppp/pppos.h"
#include "netif/ppp/ppp.h"
#include "netif/ppp/pppapi.h"
#include "lwip/opt.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/ip4_addr.h"

#include "machine_uart.h"


#define ATDEV_STATEDISCONNECTED 0
#define ATDEV_STATECONNECTED    1
#define ATDEV_STATECONNECTING   2
#define ATDEV_STATEIDLE         89
#define ATDEV_STATEFIRSTINIT    98

#define PPPOSMUTEX_TIMEOUT      (100 / portTICK_RATE_MS)
#define DEFAULT_SNTP_SERVER     "pool.ntp.org"
#define PPP_MAX_NAME_LEN        32
#define AT_MAX_SOCKETS          5
#define AT_MAX_SERV_SOCKETS     3
#define AT_SERV_SOCK_FD_BASE    90
#define WIFI_IPPROTO_SSL        253
#define AT_MAX_RESPONSES        6
#define AT_MAX_COMMANDS         6
#define MAX_INIT_TRIES          3
#define BDRATES_MAX             6
#define MAX_SERVER_CONNECTIONS  4

#define AT_OK_Str               "\r\nOK\r\n"
#define AT_Error_Str            "\r\nERROR\r\n"
#define AT_Fail_Str             "FAIL"
#define AT_Busy_Str             "busy"

#define ACTIVE_INTERFACE_NONE       0
#define ACTIVE_INTERFACE_GSM        1
#define ACTIVE_INTERFACE_WIFI       2
#define ACTIVE_INTERFACE_LWIP       4

typedef struct _socket_obj_t {
    mp_obj_base_t           base;
    int                     fd;
    int                     link_id;
    uint8_t                 domain;
    uint8_t                 type;
    uint8_t                 proto;
    bool                    peer_closed;
    uint64_t                timeout;
    #if MICROPY_PY_USOCKET_EVENTS
    mp_obj_t                events_callback;
    struct _socket_obj_t    *events_next;
    #endif
    uart_ringbuf_t          buffer;
    bool                    listening;
    bool                    accepting;
    bool                    is_accepted;
    int                     bind_port;
    int                     max_conn;
    int                     active_conn;
    int                     conn_id;
    int                     conn_fd[MAX_SERVER_CONNECTIONS];
    uint32_t                connect_time;
    uint32_t                connected_time;
    mp_obj_t                cb;
    uint32_t                total_received;
    mp_obj_t                static_buffer;
    QueueSetMemberHandle_t  semaphore;
    QueueHandle_t           mutex;
    char                    local_ip[16];
    uint16_t                local_port;
    char                    remote_ip[16];
    uint16_t                remote_port;
    void                    *parent_sock;
} __attribute__((aligned(8))) socket_obj_t;

typedef struct _at_responses_t {
    int  nresp;
    char *resp[AT_MAX_RESPONSES];
} __attribute__((aligned(8))) at_responses_t;

typedef struct _at_command_t {
    int             at_uart_num;
    bool            dbg;
    bool            type_data;
    bool            flush;
    const char      *cmd;
    int             cmdSize;
    at_responses_t  *responses;
    char            *respbuff;
    size_t          respbSize;
    size_t          respbLen;
    uint32_t        timeout;
    uint32_t        repeat;
    int             result;
    bool            not_include_cond;
} __attribute__((aligned(8))) at_command_t;

typedef struct _at_commandss_t {
    int             ncmd;
    int             at_uart_num;
    bool            dbg;
    int             delay[AT_MAX_COMMANDS];
    uint8_t         expect_resp[AT_MAX_COMMANDS];
    at_command_t    commands[AT_MAX_COMMANDS];
} __attribute__((aligned(8))) at_commands_t;

extern uint8_t net_active_interfaces;
extern int bd_rates[BDRATES_MAX];

extern socket_obj_t *at_sockets[AT_MAX_SOCKETS];
extern socket_obj_t *at_server_socket[AT_MAX_SERV_SOCKETS];
extern char at_canonname[DNS_MAX_NAME_LENGTH+1];

socket_obj_t *_new_socket();
int setNTP_cb(void *cb_func);

int at_uart_read_bytes(int uart_n, uint8_t *data, uint32_t size, uint32_t timeout);
int at_uart_write(int uart_n, const uint8_t *data, size_t size);
void at_uart_flush(int uart_n);
void at_infoCommand(char *cmd, int cmdSize, char *info, at_responses_t *responses);
int _at_Cmd_Response(at_command_t *command);
int at_Cmd_Response(at_command_t *command);
int at_Commands(at_commands_t *commands, int *n_proc);
bool at_initCmd(const char *cmd, at_command_t *command, int tmo, int nfail);
bool at_check_baudrate(bool debug, int *baudrate, int uart_num, at_command_t *command, bool disconnect);
bool _disconnect(at_command_t *command, int uart_num);

/*
 * Get transmitted and received bytes count
 * If 'rst' = 1, resets the counters
 */
//===============================================================
void pppos_getRxTxCount(uint32_t *rx, uint32_t *tx, uint8_t rst);
void wifi_getRxTxCount(uint32_t *rx, uint32_t *tx, uint8_t rst);

/*
 * Resets transmitted and received bytes counters
 */
//==========================
void pppos_resetRxTxCount();
void wifi_resetRxTxCount();

/*
 * Get GSM/Task status
 *
 * Result:
 * ATDEV_STATEDISCONNECTED    (0)     Disconnected from Internet
 * ATDEV_STATECONNECTED       (1)     Connected to Internet
 * ATDEV_STATECONNECING       (2)     Connecting to Internet
 * ATDEV_STATEIDLE            (89)    Disconnected from Internet, Task idle, waiting for reconnect request
 * ATDEV_STATEFIRSTINIT       (98)    Task started, initializing PPPoS
 */
//============================================================
int ppposStatus(uint32_t *ip, uint32_t *netmask, uint32_t *gw);


// ==== Global GSM/WiFi functions ================================

extern QueueSetMemberHandle_t wifi_task_semaphore;
extern bool wifi_task_semaphore_active;
extern bool wifi_debug;

int wifi_get_addrinfo(const char *domain, const char *portname, const struct addrinfo *hints, struct addrinfo **resp);
int gsm_get_addrinfo(const char *domain, const char *portname, const struct addrinfo *hints, struct addrinfo **resp);
int at_get_socket(socket_obj_t *sock);
int wifi_connect(socket_obj_t *sock, const char *host, int port, int localport);
int wifi_send(socket_obj_t *sock, const char *data, size_t data_len);
int wifi_send_to(socket_obj_t *sock, const char *data, size_t data_len, const char *host, int port);
int wifi_sock_buff_len(socket_obj_t *sock);
char *wifi_read_lineend(socket_obj_t *sock, const char *lend, int *size);
int wifi_read(socket_obj_t *sock, const char *data, size_t data_len);
int wifi_close(socket_obj_t *sock);
bool wifi_set_ssl_buffer_size(int *size);
bool wifi_get_time(time_t *seconds, bool set_rtc);
bool wifi_reset();
int wifi_ping(const char *host);
bool wifi_set_baudrate(int bdr);
int wifi_sslcconf(int mode);
int wifi_set_cpufreq(int freq);
int wifi_get_link_ip_port(int link_id, char *IP, int *port);
int wifi_server(socket_obj_t *sock, int port, bool ssl, int maxcon, int timeout);
int wifi_sendcertificate(const char *data, size_t data_len);
bool wifi_take_mutex();
void wifi_give_mutex();
int wifiStatus();

int wifi_at_Cmd(at_command_t *command);
int wifi_at_Commands(at_commands_t *commands, int *processed);
mp_obj_t mpy_atCmd(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args, int uart, bool dbg, const char *tag);

#endif
