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

#include "mpconfigport.h"

#if MICROPY_PY_USE_WIFI

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "syslog.h"
#include "network.h"
#include "devices.h"

#include "py/runtime.h"
#include "py/obj.h"
#include "lib/netutils/netutils.h"
#include "mphalport.h"
#include "modmachine.h"
#include "at_util.h"

#define WIFI_TASK_PRIORITY      13
#define WIFI_TASK_BUF_SIZE      3072
#define RECEIVE_TIMEOUT         3000
#define WIFI_UART_BUFFER_SIZE   3072

QueueSetMemberHandle_t wifi_task_semaphore = NULL;
bool wifi_task_semaphore_active = false;
bool wifi_debug = false;

extern handle_t mp_rtc_rtc0;
//extern socket_obj_t *_new_socket();

static TaskHandle_t wifiTaskHandle = NULL;
static int wifi_task_started = 0;
static uint8_t wifi_status = ATDEV_STATEFIRSTINIT;
static int wifi_uart_num = 0;
static int wifi_uart_baudrate = 115200;
static int wifi_pin_tx = UART_PIN_NO_CHANGE;
static int wifi_pin_rx = UART_PIN_NO_CHANGE;
static char wifiSSID_PASS[128] = {0};
static bool wifi_exit_task = false;
static bool wifi_restart_task = false;
static uint64_t receive_start_time = 0;
static uint64_t last_received_time = 0;
static int timezone = 0;
static uint32_t wifi_rx_count = 0;
static uint32_t wifi_tx_count = 0;
static bool wifi_tcpsend_wait_sent = true;

static at_responses_t at_responses = { 0 };
static at_command_t at_command = { 0 };
static const char *WIFI_TAG = "[WIFI]";
static const char *WIFI_TASK_TAG = "[WIFI_TASK]";
/*
static const char *months_names[12] =
{
    "Jan",
    "Feb",
    "Mar",
    "Apr",
    "May",
    "Jun",
    "Jul",
    "Aug",
    "Sep",
    "Oct",
    "Nov",
    "Dec"
};
*/

//-------------------------------------------------
static void _close_socket(int link_id, bool is_CIP)
{
    char cmd[32] = {'\0'};
    sprintf(cmd, "AT+%sCLOSE=%d\r\n", (is_CIP) ? "CIP" : "TCP", link_id);

    at_responses_t responses;
    memset(&responses, 0, sizeof(at_responses_t));
    responses.nresp = 2;
    responses.resp[0] = AT_OK_Str;
    responses.resp[1] = AT_Error_Str;

    at_command_t command;
    memset(&command, 0, sizeof(at_command_t));
    command.cmd = cmd;
    command.cmdSize = -1;
    command.dbg = wifi_debug;
    command.responses = &responses;
    command.at_uart_num = wifi_uart_num;
    command.flush = true;
    command.timeout = 200;
    at_Cmd_Response(&command);
}

//--------------------------------------------
static void _recv_hold(int link_id, int state)
{
    char cmd[32] = {'\0'};
    sprintf(cmd, "AT+TCPHOLD=%d,%d\r\n", link_id, state);

    at_responses_t responses;
    memset(&responses, 0, sizeof(at_responses_t));
    responses.nresp = 2;
    responses.resp[0] = AT_OK_Str;
    responses.resp[1] = AT_Error_Str;

    at_command_t command;
    memset(&command, 0, sizeof(at_command_t));
    command.cmd = cmd;
    command.cmdSize = -1;
    command.dbg = wifi_debug;
    command.responses = &responses;
    command.at_uart_num = wifi_uart_num;
    command.flush = true;
    command.timeout = 200;
    at_Cmd_Response(&command);
}

//----------------------------------------------------------------------------
static void _create_new_socket(int link_id, uint8_t srv_n, char *ip, int port)
{
    // Check if open socket in listening mode exists
    if (at_server_socket[srv_n] != NULL) {
        if ((link_id < 0) || (link_id >= AT_MAX_SOCKETS)) {
            if (wifi_debug) {
                LOGW(WIFI_TASK_TAG, "New_socket: error, wrong link_id (%d)", link_id);
            }
            return;
        }
        int sock_fd = AT_MAX_SOCKETS;
        if (at_sockets[link_id] != NULL) {
            // Find free socket
            for (sock_fd=0; sock_fd<AT_MAX_SOCKETS; sock_fd++) {
                if (at_sockets[sock_fd] == NULL) break;
            }
        }
        else sock_fd = link_id;

        if (sock_fd >= AT_MAX_SOCKETS) {
            if (wifi_debug) {
                LOGM(WIFI_TASK_TAG, "Socket for link_id %d already exists", link_id);
            }
            return;
        }

        if (at_server_socket[srv_n]->active_conn < at_server_socket[srv_n]->max_conn) {
            // Create new socket for the connection
            socket_obj_t *new_sock = _new_socket();
            new_sock->fd = sock_fd;
            new_sock->link_id = link_id;
            new_sock->parent_sock = (void *)at_server_socket[srv_n];
            new_sock->connect_time = (uint32_t)mp_hal_ticks_ms();
            new_sock->conn_id = at_server_socket[srv_n]->conn_id++;
            if ((ip != NULL) && (port > 0)) {
                strcpy(new_sock->remote_ip, ip);
                new_sock->remote_port = port;
                if (wifi_debug) {
                    LOGQ(WIFI_TASK_TAG, "New socket (%d) created for server %d: link_id %d [from %s:%d], id[%d]=%d",
                            sock_fd, at_server_socket[srv_n]->fd, link_id, ip, port, at_server_socket[srv_n]->active_conn, new_sock->conn_id);
                }
            }
            else if (wifi_debug) {
                if (wifi_debug) {
                    LOGQ(WIFI_TASK_TAG, "New socket (%d) created for server %d: link_id %d, id[%d]=%d",
                            sock_fd, at_server_socket[srv_n]->fd, link_id, at_server_socket[srv_n]->active_conn, new_sock->conn_id);
                }
            }
            at_sockets[sock_fd] = new_sock;

            int mutex_taken = 0;
            bool accepting = false;
            // --- listening socket parameters are protected by mutex
            if (at_server_socket[srv_n]->mutex) mutex_taken = xSemaphoreTake(at_server_socket[srv_n]->mutex, 100 / portTICK_PERIOD_MS);
            if (mutex_taken) {
                accepting = at_server_socket[srv_n]->accepting;
                // set the listening socket's connection fd
                at_server_socket[srv_n]->conn_fd[at_server_socket[srv_n]->active_conn] = sock_fd;
                // increase active number of connections
                at_server_socket[srv_n]->active_conn++;
                // set the listening socket's semaphore to inform it about the event
                if ((at_server_socket[srv_n]->semaphore) && (accepting))
                    xSemaphoreGive(at_server_socket[srv_n]->semaphore);
                // --- release the mutex
                xSemaphoreGive(at_server_socket[srv_n]->mutex);
            }
            /*
            if (accepting) {
                int wait_end = mp_hal_ticks_ms() + 100;
                //xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
                // Give listening socket some time to accept the connection
                while ((at_server_socket[srv_n]->accepting) && (mp_hal_ticks_ms() < wait_end)) {
                    vTaskDelay(2 / portTICK_PERIOD_MS);
                }
                //xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
                if ((mp_hal_ticks_ms() >= wait_end) && (wifi_debug)) {
                    LOGW(WIFI_TASK_TAG, "Timeout waiting for listening socket %d to accept connection", srv_n);
                }
                else if (wifi_debug) {
                    LOGM(WIFI_TASK_TAG, "Listening socket %d accepted connection", srv_n);
                }
            }
            */
            // === schedule listening socket callback function for new connection if defined ===
            if (at_server_socket[srv_n]->cb != mp_const_none) {
                // Inform listening socket about new connection
                mp_obj_t tuple[3];
                tuple[0] = MP_OBJ_FROM_PTR(at_server_socket[srv_n]);    // socket object
                tuple[1] = mp_obj_new_int(2);                           // type=2: new client connected
                tuple[2] = MP_OBJ_FROM_PTR(new_sock);                   // client socket object

                mp_sched_schedule(at_server_socket[srv_n]->cb, mp_obj_new_tuple(3, tuple));
                if (wifi_debug) {
                    LOGM(WIFI_TASK_TAG, "Listening socket %d callback scheduled", srv_n);
                }
            }
        }
        else {
            _close_socket(link_id, false);
            if (wifi_debug) {
                LOGW(WIFI_TASK_TAG, "New socket error, max sockets reached");
            }
        }
    }
    else if (wifi_debug) {
        _close_socket(link_id, false);
        LOGW(WIFI_TASK_TAG, "New socket error, no listening socket");
    }
}

// Data from WiFi module, read it and move to the socket buffer
//------------------------------------------------------------------------------------------
static void _get_data_to_socket(char* data, size_t size, int link_id, int len, uint8_t type)
{
    int inbuf, remain;
    int rd_len, to_read;
    char req;

    // Check if open socket with the link_id exists
    socket_obj_t *sock = NULL;
    bool write_sock = false;
    for (int i=0; i<AT_MAX_SOCKETS; i++) {
        if ((at_sockets[i] != NULL) && (at_sockets[i]->link_id == link_id)) {
            sock = at_sockets[i];
            break;
        }
    }
    if (sock) {
        write_sock = true;
        if (sock->buffer.buf == NULL) {
            // Allocate socket receive buffer if not allocated
            sock->buffer.buf = pvPortMalloc(len+1460);
            if (sock->buffer.buf) {
                sock->buffer.size = len+1460;
                sock->buffer.head = 0;
                sock->buffer.tail = 0;
                sock->buffer.length = 0;
                sock->buffer.uart_num = 255;
            }
            else {
                write_sock = false;
                sock = NULL;
            }
        }
        else if (sock->total_received == 0) {
            sock->buffer.head = 0;
            sock->buffer.tail = 0;
            sock->buffer.length = 0;
            sock->buffer.uart_num = 255;
        }
    }
    else if (wifi_debug) LOGW(WIFI_TASK_TAG, "no open socket for link_id %d", link_id);

    // === Get all data to socket buffer ===
    if (sock) {
        // Check if socket buffer needs to be expanded
        if ((sock->buffer.length + len) >= sock->buffer.size) {
            // need to expand the socket buffer
            if (sock->static_buffer == mp_const_none) {
                size_t old_size = sock->buffer.size;
                size_t new_size = sock->buffer.size + len + 1460;
                uint8_t *ptemp = pvPortMalloc(new_size);
                if (ptemp != NULL) {
                    memcpy(ptemp, sock->buffer.buf, old_size);
                    vPortFree(sock->buffer.buf);
                    sock->buffer.buf = ptemp;
                    sock->buffer.size = new_size;
                }
                else if (wifi_debug) {
                    write_sock = false;
                    LOGW(WIFI_TAG, "Error expanding socket buffer");
                }
            }
            else write_sock = false;
        }
    }

    if (type == 1) {
        MP_THREAD_GIL_ENTER();
        // +TCP data, request is expected, send it
        if (wifi_debug) LOGM(WIFI_TASK_TAG, "+TCP Requesting data");
        req = 'r';
        uart_write(wifi_uart_num, (uint8_t *)&req, 1);
        mp_hal_usdelay(500);
    }

    // Read data
    remain = len;
    rd_len = 0;
    // ESP8266/8285 sends maximum of 1490 bytes in one transaction
    // it can take up to ~150 ms (at 115200 bd) to transfer the block
    // so wait max 0.5 second to receive all data
    uint64_t wait_end = mp_hal_ticks_ms() + 500;
    while (remain > 0) {
        mp_hal_wdt_reset();
        to_read = (remain > size) ? size : remain;
        inbuf = uart_buf_get(mpy_uarts[wifi_uart_num].uart_buf, (uint8_t *)data, to_read);
        if (inbuf == 0) {
            // timeout handling
            if (mp_hal_ticks_ms() > wait_end) {
                if (wifi_debug) LOGE(WIFI_TAG, "Timeout waiting for data");
                break;
            }
            mp_hal_usdelay(500);
            continue;
        }
        //wait_end = mp_hal_ticks_ms() + 500;
        rd_len += inbuf;
        // some data received in uart buffer, copy to the socket buffer
        if (sock) {
            if (write_sock) {
                int wr_len = uart_buf_put(&sock->buffer, (uint8_t *)data, inbuf);
                if ((wr_len != inbuf) && (wifi_debug)) {
                    LOGW(WIFI_TAG, "Socket buffer write error (%d <> %d)", wr_len, inbuf);
                }
            }
            else sock->buffer.overflow += inbuf;
            sock->total_received += inbuf;
        }
        wifi_rx_count += inbuf;
        remain -= inbuf;
    }

    if (wifi_debug) {
        if (rd_len != len) LOGE(WIFI_TAG, "Not all data read (%d <> %d)", rd_len, len);
        if (sock) {
            LOGM(WIFI_TASK_TAG, "received (%lu ms); socket: len=%lu, ovf=%lu, tail=%lu, head=%lu",
                    mp_hal_ticks_ms()-receive_start_time, sock->buffer.length, sock->buffer.overflow, sock->buffer.tail, sock->buffer.head);
        }
        else {
            LOGM(WIFI_TASK_TAG, "received (%lu ms); no socket, len=%d", mp_hal_ticks_ms()-receive_start_time, len);
        }
    }
    last_received_time = mp_hal_ticks_ms();

    // === schedule socket callback function for data received if defined ===
    if (sock) {
        if (sock->cb != mp_const_none) {
            mp_obj_t tuple[3];
            tuple[0] = MP_OBJ_FROM_PTR(sock);   // socket object
            tuple[1] = mp_obj_new_int(1);       // type=1: new data in buffer
            tuple[2] = mp_obj_new_int(len);     // received data length

            mp_sched_schedule(sock->cb, mp_obj_new_tuple(3, tuple));
        }
    }

    if (type == 1) {
        if (rd_len != len) {
            // abort the connection
            if (wifi_debug) LOGM(WIFI_TASK_TAG, "+TCP abort receiving");
            req = 'a';
            uart_write(wifi_uart_num, (uint8_t *)&req, 1);
            return;
        }

        if ((sock) && (wifi_task_semaphore) && (wifi_task_semaphore_active)) {
            // wait for data to be processed by other tasks (socket read functions)
            if (wifi_debug) LOGY(WIFI_TASK_TAG, "Wait for data processing");
            uint64_t proc_start_time = mp_hal_ticks_ms();
            // release the mutex so that other thread can read socket data
            xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);

            if (xSemaphoreTake(wifi_task_semaphore, 500 / portTICK_PERIOD_MS) == pdTRUE) {
                // socket data was processed
                if (wifi_debug) LOGQ(WIFI_TASK_TAG, "Data processed in %lu ms", mp_hal_ticks_ms()-proc_start_time);
            }
            else if (wifi_debug) LOGQ(WIFI_TASK_TAG, "Data was not processed");

            // take the mutex back
            while (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) {
                vTaskDelay(3 / portTICK_PERIOD_MS);
            }
        }

        // confirm receive
        if (wifi_debug) LOGM(WIFI_TASK_TAG, "+TCP Confirm receive");
        req = 'y';
        uart_write(wifi_uart_num, (uint8_t *)&req, 1);
        mp_hal_usdelay(500);
        MP_THREAD_GIL_EXIT();
    }
}

// Parse +IPD (type=0) or +TCP (type=1) data request from WiFi module
// '+IPD,' or '+TCP,' is at the 'start_pos' position in the uart buffer
//-------------------------------------------------------------------------
static void parse_IPD(char* data, size_t size, int start_pos, uint8_t type)
{
    // === Data received pattern found, copy data from UART buffer and analyze ===
    int position = -1, link_id = 0, srv_id = 9, len = 0;
    char *pipd = NULL;
    char *pipd_cmdend = NULL;
    char *sep = NULL;
    uint8_t cmd_len = 9;
    bool has_sock = false;

    /*
     * If the +IPD is received in passive mode it will be in the form '+IPD,<link_id>,<length>\r\n' (not supported)
     * In active mode it will be in the form '+IPD,<link_id>,<length>:data....'
     * If the +TCP is received it will be in the form '+TCP,<link_id>,<server_id>,<length>:' and request for data is expected
     */
    int ntry = 10;
    // Wait for complete command to arrive
    while (ntry > 0) {
        position = uart_buf_find_from(mpy_uarts[wifi_uart_num].uart_buf, start_pos, 9999, ":", 1, NULL);
        if (position < 0) {
            if (type == 0) position = uart_buf_find_from(mpy_uarts[wifi_uart_num].uart_buf, start_pos, 9999, "\r\n", 2, NULL);
            if (position < 0) {
                mp_hal_usdelay(500);
                ntry--;
                continue;
            }
            cmd_len = position - start_pos + 2;
        }
        else cmd_len = position - start_pos + 1;

        // copy the full command into data buffer
        uart_buf_copy_from(mpy_uarts[wifi_uart_num].uart_buf, start_pos, (uint8_t *)data, cmd_len);
        data[cmd_len] = '\0';

        if (wifi_debug)
            LOGM(WIFI_TASK_TAG, "+%s data request found [%s]", (type == 0) ? "IPD" : "TCP", data);
        // Check once again for data received pattern
        if (type == 0) pipd = strstr(data, "+IPD,");
        else pipd = strstr(data, "+TCP,");
        if (pipd == NULL) break;

        pipd_cmdend = strchr(pipd, ':');
        if (pipd_cmdend == NULL) break;

        // receiving in active mode
        if (((pipd_cmdend-pipd) < ((type == 1) ? 10: 8)) || ((pipd_cmdend-pipd) > ((type == 1) ? 13: 11))) {
            pipd_cmdend = NULL;
            break;
        }
        // parse parameters
        pipd += 5;
        // link_id
        sep = strchr(pipd, ',');
        if (sep == NULL) goto err_param;
        *sep = '\0';
        link_id = strtol(pipd, (char **)NULL, 10);
        if (type == 1) {
            // server_id
            pipd = sep+1;
            sep = strchr(pipd, ',');
            if (sep == NULL) goto err_param;
            *sep = '\0';
            srv_id = strtol(pipd, (char **)NULL, 10);
        }
        else srv_id = 9;
        // length
        pipd = sep+1;
        sep = strchr(pipd, ':');
        if (sep == NULL) goto err_param;
        *sep = '\0';
        len = strtol(pipd, (char **)NULL, 10);

        if ((len <= 0) || (link_id < 0) || (link_id >= AT_MAX_SOCKETS)) goto err_param;
        // params, ok
        break;
err_param:
        if (wifi_debug) LOGE(WIFI_TASK_TAG, "+%s, parameter error [%s]", (type == 0) ? "IPD" : "TCP", data);
        pipd_cmdend = NULL;
        break;
    }

    if ((pipd == NULL) || (pipd_cmdend == NULL)) {
        // Full +IPD or +TCP command not received, blank all command related data in uart buffer
        uart_buf_blank(mpy_uarts[wifi_uart_num].uart_buf, start_pos, cmd_len);
        if (wifi_debug) {
            LOGE(WIFI_TASK_TAG, "Error receiving data (+%s%s%s)",
                    (type == 0) ? "IPD" : "TCP", (pipd == NULL) ? ": par start" : "", (pipd_cmdend == NULL) ? ", par_end" : "");
        }

        if (type == 1) {
            // +TCP data, request abort
            char req = 'a';
            uart_write(wifi_uart_num, (uint8_t *)&req, 1);
        }
        return;
    }

    // ==== Ready to receive data =================================
    if (receive_start_time == 0) {
        receive_start_time = mp_hal_ticks_ms();
        last_received_time = receive_start_time;
    }
    if (wifi_debug) LOGM(WIFI_TASK_TAG, "Data for link_id %d (srv=%d), len=%d [%s]", link_id, srv_id, len, data);

    // Remove all data in from buffer
    uart_buf_remove(mpy_uarts[wifi_uart_num].uart_buf, start_pos + cmd_len);

    // Check if open socket with the link_id exists
    for (int i=0; i<AT_MAX_SOCKETS; i++) {
        if ((at_sockets[i] != NULL) && (at_sockets[i]->link_id == link_id)) {
            has_sock = true;
            break;
        }
    }
    if (!has_sock) {
        if ((srv_id >= 0) && (srv_id < AT_MAX_SERV_SOCKETS)) {
            // data received, but no socket exists, probably the new link for listening socket
            if (wifi_debug) LOGY(WIFI_TASK_TAG, "Data for listening socket received, try to create new socket");
            _create_new_socket(link_id, srv_id, NULL, 0);
        }
        else if (wifi_debug) {
            LOGW(WIFI_TASK_TAG, "No listening socket!");
        }
    }

    // === Receive the data ===
    _get_data_to_socket(data, size, link_id, len, type);
}

/* Check if any known pattern was received from WiFi module
 * Checks for:
 * +IPD,        data received for socket
 * +TCP,        data received for socket
 * ,CLOSED      link (socket) closed
 * ,CONNECT     connection to the server socket
 * ,TCPconnect  connection to the server socket
 * ready\r\n    WiFi module reset
 */

//-------------------------------------------------------
static void _check_wifi_response(char* data, size_t size)
{
    size_t buflen, bufsize;
    int inbuf, position;

check_again:

    buflen = uart_buf_length(mpy_uarts[wifi_uart_num].uart_buf, &bufsize);
    if (buflen == 0) {
        // === No data in buffer ===
        return;
    }

    // =========================================================
    // === Check if 'n,CLOSED' pattern exists in uart buffer ===
    position = uart_buf_find(mpy_uarts[wifi_uart_num].uart_buf, buflen, ",CLOSED", 7, NULL);
    if (position > 0) {
        inbuf = uart_buf_copy_from(mpy_uarts[wifi_uart_num].uart_buf, position-1, (uint8_t *)data, 8);
        int link_id = (int)data[0] - '0';
        // Check if open socket with the link_id exists
        socket_obj_t *sock = NULL;
        for (int i=0; i<AT_MAX_SOCKETS; i++) {
            if ((at_sockets[i] != NULL) && (at_sockets[i]->link_id == link_id)) {
                sock = at_sockets[i];
                break;
            }
        }
        if (sock) {
            sock->peer_closed = true;
            sock->connected_time = (uint32_t)(mp_hal_ticks_ms() - sock->connect_time);
        }
        if (wifi_debug) {
            LOGY(WIFI_TASK_TAG, "connection for socket with link_id %d closed, active=%lu ms, time=%lu ms",
                    link_id, (receive_start_time > 0) ? mp_hal_ticks_ms()-receive_start_time : 0,
                    (receive_start_time > 0) ? last_received_time-receive_start_time : 0);
        }
        receive_start_time = 0;
        last_received_time = 0;
        if (wifi_debug) {
            data[inbuf] = '\0';
            LOGW(WIFI_TASK_TAG, "closed, removed [%s]", data);
        }
        uart_buf_blank(mpy_uarts[wifi_uart_num].uart_buf, position-1, 8);

        // === Allow other tasks to process socket closing ===
        if (wifi_debug) LOGQ(WIFI_TASK_TAG, "Processing delay");
        // release the mutex
        xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // take the mutex back
        while (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) {
            vTaskDelay(3 / portTICK_PERIOD_MS);
        }
        if (wifi_debug) LOGQ(WIFI_TASK_TAG, "Continue");

        goto check_again;
    }

    // ==============================================================================
    // === Check if 'n,CONNECT' or 's,n,TCPconnect:' pattern exists in uart buffer ===
    //     link_id,CONNECT (NOT SUPPORTED)
    //     link_id,srv_id,TCPconnect:"IPaddr",port\r\n
    uint8_t type = 0;
    int cmd_size = 0;
    position = uart_buf_find(mpy_uarts[wifi_uart_num].uart_buf, buflen, ",CONNECT", 8, NULL);
    if (position < 2) {
        position = uart_buf_find(mpy_uarts[wifi_uart_num].uart_buf, buflen, ",TCPconnect:", 12, NULL);
        if (position >= 3) {
            // TCP Server connection detected, get the full command
            type = 1;
            int cmd_end = -1;
            while (cmd_end < 0) {
                cmd_end = uart_buf_find_from(mpy_uarts[wifi_uart_num].uart_buf, position, 9999, "\r\n", 2, NULL);
            }
            cmd_size = (cmd_end - position) + 3;
        }
    }
    else cmd_size = 9;

    if (cmd_size) {
        inbuf = uart_buf_copy_from(mpy_uarts[wifi_uart_num].uart_buf, position-((type == 0) ? 1 : 3), (uint8_t *)data, cmd_size);
        data[inbuf] = '\0';
        int srv_n = AT_MAX_SERV_SOCKETS;
        int link_id = 9;
        int port = 0;
        char *ip_addr = NULL;
        if (type == 1) {
            srv_n = (int)(data[0] - '0');
            link_id = (int)(data[2] - '0');
            // get IP and port
            ip_addr = strchr((char *)data, ':');
            if (ip_addr) {
                ip_addr++;
                char *pport = strchr(ip_addr, ',');
                if (pport) {
                    *pport = '\0';
                    pport++;
                    port = strtol(pport, (char **)NULL, 10);
                    if ((port < 1) || (port > 65535)) {
                        port = 0;
                        ip_addr = NULL;
                    }
                }
                else ip_addr = NULL;
            }
        }
        else link_id = (int)(data[0] - '0');

        uart_buf_blank(mpy_uarts[wifi_uart_num].uart_buf, position-((type == 0) ? 1 : 3), cmd_size);

        if (type == 0) {
            // CIP Connect is not supported
            if (wifi_debug) LOGY(WIFI_TASK_TAG, "Unsupported connection type! (%s)", data);
            _close_socket(link_id, false);
            goto check_again;
        }

        if ((link_id >= 0) && (link_id < AT_MAX_SOCKETS)) {
            if (wifi_debug) LOGY(WIFI_TASK_TAG, "Listening socket connect. link_id=%d", link_id);
            if ((srv_n >= 0) && (srv_n < AT_MAX_SERV_SOCKETS)) {
                if ((ip_addr != NULL) && (port > 0)) _create_new_socket(link_id, srv_n, ip_addr, port);
                else if (wifi_debug) {
                    LOGW(WIFI_TASK_TAG, "CONNECT detected, but no IP and port decoded [%s]", data);
                }
            }
            else if (wifi_debug) {
                LOGW(WIFI_TASK_TAG, "CONNECT detected, but no listening socket [%s]", data);
            }
        }
        else if (wifi_debug) {
            LOGW(WIFI_TASK_TAG, "CONNECT detected, but wrong link_id (%d) [%s]", link_id, data);
        }

        // === Allow other tasks to process connect ===
        if (wifi_debug) LOGQ(WIFI_TASK_TAG, "Processing delay");
        // release the mutex
        xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        // take the mutex back
        while (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) {
            vTaskDelay(3 / portTICK_PERIOD_MS);
        }
        if (wifi_debug) LOGQ(WIFI_TASK_TAG, "Continue");

        goto check_again;
    }

    // ======================================================
    // === Check if 'ready' pattern exists in uart buffer ===
    position = uart_buf_find(mpy_uarts[wifi_uart_num].uart_buf, buflen, "ready\r\n", 7, NULL);
    if (position > 0) {
        // *** probably WiFi device reset ***
        if (wifi_debug) {
            inbuf = uart_buf_copy(mpy_uarts[wifi_uart_num].uart_buf, (uint8_t *)data, buflen-8);
            data[inbuf] = '\0';
            LOGE(WIFI_TASK_TAG, "WiFi RESET [%s]", data);
        }
        uart_buf_remove(mpy_uarts[wifi_uart_num].uart_buf, position+7);
        wifi_restart_task = true;
        return;
    }

    // ======================================================
    // === Check if '+TCP,' pattern exists in uart buffer ===
    position = uart_buf_find(mpy_uarts[wifi_uart_num].uart_buf, buflen, "+TCP,", 5, NULL);
    if (position >= 0) {
        parse_IPD(data, size, position, 1);
        goto check_again;
    }

    // ======================================================
    // === Check if '+IPD,' pattern exists in uart buffer ===
    position = uart_buf_find(mpy_uarts[wifi_uart_num].uart_buf, buflen, "+IPD,", 5, NULL);
    if (position >= 0) {
        parse_IPD(data, size, position, 0);
        goto check_again;
    }

    // === Remove other strings ===
    // 'buflen' bytes has been checked and no match found, we can safely remove those bytes from buffer
    // but we take into account that the partial check strings can be at the end
    //if ((buflen > 16) && ((mp_hal_ticks_ms()-last_received_time) > RECEIVE_TIMEOUT)) {
    if (buflen > 16) {
        // leave 16 bytes in buffer
        if (wifi_debug) {
            inbuf = uart_buf_copy(mpy_uarts[wifi_uart_num].uart_buf, (uint8_t *)data, buflen-16);
            data[inbuf] = '\0';
            LOGW(WIFI_TASK_TAG, "no match, removed [%s]", data);
        }
        uart_buf_remove(mpy_uarts[wifi_uart_num].uart_buf, buflen-16);
    }

    if (mpy_uarts[wifi_uart_num].task_semaphore) {
        if (xSemaphoreTake(mpy_uarts[wifi_uart_num].task_semaphore, 0) == pdTRUE) {
            // some new data received, do not exit, check again
            if (wifi_debug) LOGQ(WIFI_TASK_TAG, "more data, check again");
            goto check_again;
        }
    }

    // === Check sockets ===
    for (int i=0; i<AT_MAX_SOCKETS; i++) {
        if (at_sockets[i] != NULL) {
            if ((at_sockets[i]->link_id < 0) || (at_sockets[i]->link_id >= AT_MAX_SOCKETS)) {
                at_sockets[i] = NULL;
                if (wifi_debug) LOGe(WIFI_TASK_TAG, "Orphaned socket at %d", i);
            }
        }
    }
}


/*
 * WiFi TASK
 * Handles WiFi initialization and WiFi device responses
 *
 * WiFi device must be already powered on and ready for communication
 */
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static void wifi_task(void *pv)
{
    // if the task uses some MicroPython functions, we have to save
    // MicroPython state in local storage pointers
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)pv, THREAD_LSP_STATE));
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)pv, THREAD_LSP_ARGS));

    wifi_task_started = -1;
    wifi_exit_task = false;
    wifi_restart_task = false;
    wifi_tx_count = 0;
    wifi_rx_count = 0;
    wifi_status = ATDEV_STATEFIRSTINIT;
    char* data = NULL;
    int mutex_taken = 0;

    // === Initialize WiFi UART first ===================================================
    if (wifi_debug) {
        LOGM(WIFI_TAG, "Initialize WiFi UART");
    }
    // === Initialize the WiFi UART ===
    // Find free uart
    wifi_uart_num = -1;
    for (int i=0; i < UART_NUM_MAX; i++) {
        if (mpy_uarts[i].handle == 0) {
            wifi_uart_num = i;
            break;
        }
    }
    if (wifi_uart_num < 0) {
        LOGE(WIFI_TAG, "No free uart available");
        // Terminate task
        wifi_task_started = 0;
        vTaskDelete(NULL);
    }
    memset(&mpy_uarts[wifi_uart_num], 0, sizeof(uart_uarts_t));

    // === Initialize uart hardware and drivers ===
    if (wifi_debug) {
        LOGM(WIFI_TAG,"UART #%d: initialize uart hardware", wifi_uart_num);
    }
    int res = uart_hard_init(wifi_uart_num, wifi_pin_tx, wifi_pin_rx, GPIO_FUNC_WIFI_UART, true, true, WIFI_UART_BUFFER_SIZE);
    if (res < 0) {
        LOGE(WIFI_TAG, "Error initializing UART hardware (error %d)", res);
        // Terminate task
        wifi_task_started = 0;
        vTaskDelete(NULL);
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);

    uint32_t bdr = mp_uart_config(wifi_uart_num, wifi_uart_baudrate, 8, UART_STOP_1, UART_PARITY_NONE);

    if (wifi_debug) {
        LOGM(WIFI_TAG,"UART #%d: initialized, tx=%d, rx=%d, bdr=%d", wifi_uart_num, wifi_pin_tx, wifi_pin_rx, bdr);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
    // ==================================================================================

    data = pvPortMalloc(WIFI_TASK_BUF_SIZE);
    if (data == NULL) {
        LOGE(WIFI_TAG,"Failed to allocate data buffer.");
        goto exit;
    }

    mutex_taken = xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
    if (mutex_taken != pdTRUE) {
        goto exit;
    }

    if (wifi_debug) {
        LOGM(WIFI_TAG, "WiFi TASK STARTED");
    }
    wifi_task_started = 1;

    at_responses_t responses;
    memset(&responses, 0, sizeof(at_responses_t));

    at_command_t command;
    memset(&command, 0, sizeof(at_command_t));
    command.cmd = NULL;
    command.cmdSize = -1;
    command.dbg = wifi_debug;
    command.responses = &responses;
    command.at_uart_num = wifi_uart_num;
    command.flush = true;

    // === Check baud rate and change if necessary ===
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (!at_check_baudrate(wifi_debug, &wifi_uart_baudrate, wifi_uart_num, &command, false)) goto exit;

    memset(data, 0, WIFI_TASK_BUF_SIZE);

    wifi_task_semaphore = xSemaphoreCreateBinary();
    wifi_task_semaphore_active = false;
    bool exit_task = false;
    // === Main task's loop ===
    while(1)
    {
        if (mutex_taken != pdTRUE) mutex_taken = xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
        if (wifi_debug) LOGM(WIFI_TAG,"WiFi device initialization start");
        vTaskDelay(500 / portTICK_PERIOD_MS);

        int cmd_res = 0;
        // ===== WiFi Initialization ==============================================================================
        responses.nresp = 1;
        responses.resp[0] = AT_OK_Str;
        if (!at_initCmd("AT\r\n", &command, 100, 4)) goto exit;
        if (!at_initCmd("ATE0\r\n", &command, 100, 2)) goto exit;

        if (!at_initCmd("AT+TCPCLOSE?\r\n", &command, 100, 2)) {
            if (wifi_debug) LOGE(WIFI_TAG,"Firmware supporting TCP commands must be used");
            goto exit;
        }

        responses.resp[0] = "+CWMODE:1";
        if (!at_initCmd("AT+CWMODE?\r\n", &command, 100, 4)) {
            if (!at_initCmd("AT+CWMODE=1\r\n", &command, 100, 2)) goto exit;
        }

        responses.resp[0] = "+CWJAP:";
        if (!at_initCmd("AT+CWJAP?\r\n", &command, 100, 2)) {
            responses.resp[0] = AT_OK_Str;
            responses.resp[1] = "WIFI GOT IP";
            responses.resp[2] = AT_Fail_Str;
            responses.nresp = 3;
            command.cmd = wifiSSID_PASS;
            command.timeout = 8000;
            cmd_res = at_Cmd_Response(&command);
            if ((cmd_res != 1) && (cmd_res != 2)) goto exit;
        }

        responses.resp[0] = AT_OK_Str;
        responses.nresp = 1;
        if (!at_initCmd("AT+CIPMUX=1\r\n", &command, 100, 2)) goto exit;
        if (!at_initCmd("AT+CIPDINFO=0\r\n", &command, 100, 2)) goto exit;
        if (at_initCmd("AT+CIPRECVMODE?\r\n", &command, 100, 2)) {
            // command implemented
            if (!at_initCmd("AT+CIPRECVMODE=0\r\n", &command, 100, 2)) goto exit;
        }

        at_initCmd("AT+SYSCPUFREQ=160\r\n", &command, 100, 2);

        responses.resp[0] = "+CIPSNTPCFG:1";
        responses.resp[1] = "+CIPSNTPCFG:0";
        responses.nresp = 2;
        command.cmd = "AT+CIPSNTPCFG?\r\n";
        command.timeout = 500;
        cmd_res = at_Cmd_Response(&command);
        if (cmd_res != 1) {
            char sntp_cfg[128];
            sprintf(sntp_cfg, "AT+CIPSNTPCFG=1,%d,\"%s\"\r\n", timezone, DEFAULT_SNTP_SERVER);
            responses.resp[0] = AT_OK_Str;
            responses.resp[1] = AT_Error_Str;
            command.cmd = sntp_cfg;
            cmd_res = at_Cmd_Response(&command);
            if ((cmd_res != 1) && (wifi_debug)) {
                LOGW(WIFI_TAG,"SNTP enable error");
            }
            else {
                responses.resp[0] = "+CIPSNTPCFG:1";
                responses.resp[1] = "+CIPSNTPCFG:0";
                command.cmd = "AT+CIPSNTPCFG?\r\n";
                cmd_res = at_Cmd_Response(&command);
                if ((cmd_res != 1) && (wifi_debug)) {
                    LOGW(WIFI_TAG,"SNTP config error");
                }
            }
        }
        // =======================================================================================================

        vTaskDelay(100 / portTICK_PERIOD_MS);
        uart_buf_flush(mpy_uarts[wifi_uart_num].uart_buf);

        wifi_status = ATDEV_STATEIDLE;
        net_active_interfaces |= ACTIVE_INTERFACE_WIFI;
        xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
        mutex_taken = 0;
        if (wifi_debug) LOGM(WIFI_TAG,"WiFi initialized and ready.");

        // === WiFi module is now initialized         ========
        // === Process incoming data from WiFi module ========
        bool do_check = false;
        while (1) {
            if (mpy_uarts[wifi_uart_num].task_semaphore) {
                do_check = (xSemaphoreTake(mpy_uarts[wifi_uart_num].task_semaphore, 5 / portTICK_PERIOD_MS ) == pdTRUE);
            }
            else {
                vTaskDelay(5 / portTICK_PERIOD_MS);
                do_check = true;
            }
            mutex_taken = xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
            if (mutex_taken != pdTRUE) continue;

            // ==== Check response from WiFi module ====
            if ((wifi_task_semaphore) && (wifi_task_semaphore_active)) xSemaphoreTake(wifi_task_semaphore, 0);

            //-----------------------------------------------------------
            if (do_check) _check_wifi_response(data, WIFI_TASK_BUF_SIZE);
            //-----------------------------------------------------------

            exit_task = wifi_exit_task;
            xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
            mutex_taken = 0;
            if (exit_task) break;  // terminate task requested
            if (wifi_restart_task) {
                wifi_restart_task = false;
                uart_buf_flush(mpy_uarts[wifi_uart_num].uart_buf);
                wifi_status = ATDEV_STATEFIRSTINIT;
                break;
            }
        }
        // ===================================================

        if (exit_task) {
            exit_task = false;
            break;  // terminate task
        }
    }  // main task loop

exit:
    // Terminate WiFi task
    if (mutex_taken == pdTRUE) xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);

    if (data) vPortFree(data);

    uart_deinit(wifi_uart_num, NULL, wifi_pin_tx, wifi_pin_rx);
    if (wifi_debug) {
        LOGM(WIFI_TAG,"UART #%d deinitialized", wifi_uart_num);
    }

    wifi_status = ATDEV_STATEFIRSTINIT;

    wifi_task_started = 0;

    if (wifi_task_semaphore != NULL) vSemaphoreDelete(wifi_task_semaphore);
    wifi_task_semaphore = NULL;
    wifi_task_semaphore_active = false;

    if (wifi_debug) {
        LOGM(WIFI_TAG, "WiFi TASK TERMINATED");
    }
    vTaskDelete(NULL);
}


//---------------------------------------------------------------------------------
static int wifi_Init(int tx, int rx, int bdr, char *ssid, char *pass, uint8_t wait)
{
    int task_s = wifi_task_started;
    int tmo = 0;

    if (task_s == 1) return 0;
    if (task_s == -1) {
        // Task starting
        tmo = 2000;
        while (task_s == -1) {
            vTaskDelay(10 / portTICK_RATE_MS);
            tmo -= 10;
            if (tmo <= 0) return -1;
            task_s = wifi_task_started;
        }
    }
    if (task_s == 0) {
        // WiFi task not running, create it
        wifi_pin_tx = tx;
        wifi_pin_rx = rx;
        wifi_uart_baudrate = bdr;
        sprintf(wifiSSID_PASS, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);

        TaskHandle_t curr_task_handle = xTaskGetCurrentTaskHandle();
        // === Create and start the WiFi task ===
        BaseType_t res = xTaskCreate(
            wifi_task,                 // function entry
            "WiFi_Task",               // task name
            configMINIMAL_STACK_SIZE,  // stack_deepth
            curr_task_handle,          // function argument
            WIFI_TASK_PRIORITY,        // task priority
            &wifiTaskHandle            // task handle
        );
        if ((res != pdPASS) || (wifiTaskHandle == NULL)) {
            return -2;
        }

        vTaskDelay(10 / portTICK_RATE_MS);
        // The task should be running by now
        task_s = wifi_task_started;
        if (task_s == -1) {
            // Task starting
            tmo = 2000;
            while (task_s == -1) {
                vTaskDelay(10 / portTICK_RATE_MS);
                tmo -= 10;
                if (tmo <= 0) return -1;
                task_s = wifi_task_started;
            }
        }
        if (task_s == 0) return -2;
    }

    if (wait) {
        // Wait until task is ready
        tmo = 20000;
        while (tmo >= 0) {
            vTaskDelay(10 / portTICK_RATE_MS);
            tmo -= 10;
            task_s = wifi_task_started;
            if (task_s == 0) return -3;
            if (wifi_status == ATDEV_STATEIDLE) return 0;
        }
    }

    return 0;
}

//--------------------------------
static int wifi_endTask(bool wait)
{
    if (mpy_uarts[wifi_uart_num].uart_mutex == NULL) return -1;
    int task_s = wifi_task_started;
    if (task_s == 0) return -2;  // task not stared, cannot end

    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) return -2;
    wifi_exit_task = true;
    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);

    if (wait) {
        int tmo = 0;
        while (task_s != 0) {
            vTaskDelay(10 / portTICK_RATE_MS);
            task_s = wifi_task_started;
            tmo++;
            if (tmo > 1500) return -1;
        }
        return 0;
    }
    return 0;
}

//--------------
int wifiStatus()
{
    if (wifi_task_started == 0) return ATDEV_STATEFIRSTINIT;
    if (mpy_uarts[wifi_uart_num].uart_mutex == NULL) return ATDEV_STATEFIRSTINIT;
    return wifi_status;
}

//---------------------------------
static void wifi_setDebug(bool dbg)
{
    int taken = pdFALSE;
    if (mpy_uarts[wifi_uart_num].uart_mutex != NULL) taken = xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
    wifi_debug = dbg;
    if (taken) xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
}

//-------------------------
static bool wifi_getDebug()
{
    int taken = pdFALSE;
    if (mpy_uarts[wifi_uart_num].uart_mutex != NULL) taken = xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
    bool dbg = wifi_debug;
    if (taken) xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
    return dbg;
}

// === AT processing =========================================================================

//------------------------------------
int wifi_at_Cmd(at_command_t *command)
{
    if (wifiStatus() != ATDEV_STATEIDLE) return 0;

    // prevent the WiFi task to process uart buffer
    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*4) != pdTRUE) {
        if (wifi_debug) {
            LOGE(WIFI_TAG, "ATCmd: couldn't get mutex");
        }
        return 0;
    }
    int res = at_Cmd_Response(command);
    // allow the WiFi task to process uart buffer
    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);

    return res;
}

//-----------------------------------------------------------
int wifi_at_Commands(at_commands_t *commands, int *processed)
{
    if (wifiStatus() != ATDEV_STATEIDLE) return 0;

    // prevent the WiFi task to process uart buffer
    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*4) != pdTRUE) {
        if (wifi_debug) {
            LOGE(WIFI_TAG, "ATCommands: couldn't get mutex");
        }
        return 0;
    }
    int res = at_Commands(commands, processed);
    // allow the WiFi task to process uart buffer
    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);

    return res;
}


// === Global functions (used by socket and other modules) ==============

//-------------------------------------------------------------------------------------------------------------------
int wifi_get_addrinfo(const char *domain, const char *portname, const struct addrinfo *hints, struct addrinfo **resp)
{
    int port_nr = 0;
    ip_addr_t addr;
    *resp = NULL;

    if ((domain == NULL) || (strlen(domain) >= 64)) {
        if (wifi_debug) {
            LOGE(WIFI_TAG, "No domain");
        }
        return EAI_NONAME;
    }
    if (portname != NULL) {
      // port string specified: convert to port number
      port_nr = atoi(portname);
      if ((port_nr <= 0) || (port_nr > 0xffff)) {
          if (wifi_debug) {
              LOGE(WIFI_TAG, "Wrong port");
          }
          return EAI_SERVICE;
      }
    }
    memset(at_canonname, 0, sizeof(at_canonname));

    char cmd[128];
    sprintf(cmd, "AT+CIPDOMAIN=\"%s\"\r\n", domain);

    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 18000;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.respbuff = pvPortMalloc(256);
    if (at_command.respbuff == NULL) {
        if (wifi_debug) {
            LOGE(WIFI_TAG, "Buffer allocation error");
        }
        return -1;
    }
    at_command.respbSize = 256;
    memset(at_command.respbuff, 0, at_command.respbSize);

    // If the IP cannot be resolved it can take up to 15 seconds !!
    int res = wifi_at_Cmd(&at_command);

    if (res <= 0) {
        if (wifi_debug) {
            LOGE(WIFI_TAG, "No response");
        }
        return -1;
    }

    char *pbuf = strstr(at_command.respbuff, "+CIPDOMAIN:");
    if (pbuf) {
        pbuf += 11;
        struct addrinfo *wifi_addrinfo = (struct addrinfo *)pvPortMalloc(sizeof(struct addrinfo)+sizeof(struct sockaddr_storage));
        if (wifi_addrinfo == NULL) {
            if (wifi_debug) {
                LOGE(WIFI_TAG, "Error allocating addrinfo");
            }
            vPortFree(at_command.respbuff);
            return -3;
        }

        struct sockaddr_storage *sa = NULL;
        sa = (struct sockaddr_storage *)(void *)((u8_t *)wifi_addrinfo + sizeof(struct addrinfo));
        struct sockaddr_in *sa4 = (struct sockaddr_in *)sa;
        wifi_addrinfo->ai_flags = 0;
        wifi_addrinfo->ai_next = NULL;
        if (hints != NULL) {
            wifi_addrinfo->ai_family = hints->ai_family;
            wifi_addrinfo->ai_socktype = hints->ai_socktype;
            wifi_addrinfo->ai_protocol = hints->ai_protocol;
        }
        else {
            wifi_addrinfo->ai_family = AF_INET;
            wifi_addrinfo->ai_socktype = SOCK_STREAM;
            wifi_addrinfo->ai_protocol = 0;
        }
        char outbuf[18] = {'\0'};
        for (int i=0; i<17; i++) {
            if (((*pbuf >= '0') && (*pbuf <= '9')) || (*pbuf == '.')) {
                outbuf[i] = *pbuf;
                outbuf[i+1] = '\0';
                pbuf++;
            }
            else break;
        }
        addr.addr = ipaddr_addr(outbuf);
        inet_addr_from_ip4addr(&sa4->sin_addr, ip_2_ip4(&addr));
        sa4->sin_family = AF_INET;
        sa4->sin_len = sizeof(struct sockaddr_in);
        sa4->sin_port = lwip_htons((u16_t)port_nr);
        wifi_addrinfo->ai_family = AF_INET;

        // copy domain to canonname
        snprintf(at_canonname, DNS_MAX_NAME_LENGTH, "%s", domain);

        wifi_addrinfo->ai_canonname = (char *)&at_canonname[0];
        wifi_addrinfo->ai_addrlen = sizeof(struct sockaddr_storage);
        wifi_addrinfo->ai_addr = (struct sockaddr *)sa;
        *resp = wifi_addrinfo;

        vPortFree(at_command.respbuff);
        return 0;
    }
    else if (wifi_debug) {
        LOGW(WIFI_TAG, "Wrong response [%s]", at_command.respbuff);
    }
    vPortFree(at_command.respbuff);
    return -2;
}

//-----------------------------------------------------------------------------
int wifi_connect(socket_obj_t *sock, const char *host, int port, int localport)
{
    if (at_sockets[sock->fd] != sock) {
        errno = ENOTSOCK;
        return -1;
    }
    int keep_alive = 0;
    char cmd[256];
    char type[8];
    char response[16] = {'\0'};
    if (sock->proto == IPPROTO_UDP) {
        sprintf(type, "UDP");
        sprintf(response, "\r\nOK\r\n");
    }
    else if (sock->proto == WIFI_IPPROTO_SSL) {
        sprintf(type, "SSL");
        sprintf(response, "%d,CONNECT\r\n", sock->link_id);
    }
    else {
        sprintf(type, "TCP");
        sprintf(response, "%d,CONNECT\r\n", sock->link_id);
    }
    if (sock->proto != IPPROTO_UDP) keep_alive = 3;

    if (sock->proto != IPPROTO_UDP)
        sprintf(cmd, "AT+TCPSTART=%d,\"%s\",\"%s\",%d,%d,%d\r\n", sock->link_id, type, host, port, keep_alive, localport);
    else {
        errno = ENOTCONN;
        return -1;
    }
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 3;
    at_responses.resp[0] = response;
    at_responses.resp[1] = AT_Error_Str;
    at_responses.resp[2] = "\r\n+TCPSTART";
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 6000;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    int res = wifi_at_Cmd(&at_command);
    if (res == 1) {
        sock->connect_time = (uint32_t)mp_hal_ticks_ms();
        errno = 0;
        return 0;
    }
    errno = ENOTCONN;
    return -1;
}

//------------------------------------------------------------------------------
int wifi_server(socket_obj_t *sock, int port, bool ssl, int maxcon, int timeout)
{
    int max_srv_n = AT_MAX_SERV_SOCKETS;
    int srv_n = max_srv_n;

    for (srv_n=0; srv_n<max_srv_n; srv_n++) {
        if (at_server_socket[srv_n] == sock) break;
    }
    if (srv_n == max_srv_n) {
        errno = ENOTSOCK;
        return -1;
    }
    char cmd[256];

    sprintf(cmd, "AT+TCPSERVER=%d,1,%d,%d,%d,%d\r\n", srv_n, port, ssl, maxcon, timeout);
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 500;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    int res = wifi_at_Cmd(&at_command);
    if (res == 1) {
        sock->bind_port = port;
        errno = 0;
        return 0;
    }
    errno = ENOTCONN;
    return -1;
}

//--------------------------------
int wifi_close(socket_obj_t *sock)
{
    char cmd[32];
    int max_srv_n = AT_MAX_SERV_SOCKETS;
    int res, srv_n = max_srv_n;

    if (sock->listening) {
        // Closing the server socket
        for (srv_n=0; srv_n<max_srv_n; srv_n++) {
            if (at_server_socket[srv_n] == sock) break;
        }
        if (srv_n == max_srv_n) {
            errno = ENOTSOCK;
            return -1;
        }
        // close all connected clients first
        if (wifi_debug) LOGQ(WIFI_TAG, "Close server socket %d", srv_n);
        for (int i=0; i<AT_MAX_SOCKETS; i++) {
            if (at_sockets[i] != NULL) {
                if (at_sockets[i]->parent_sock == sock) {
                    if (wifi_debug) LOGQ(WIFI_TAG, "Close connected socket %d (%d)", at_sockets[i]->fd, at_sockets[i]->link_id);
                    res = wifi_close(at_sockets[i]);
                }
            }
        }
        // close the server socket
        sprintf(cmd, "AT+TCPSERVER=%d,0\r\n", srv_n);
    }
    else {
        if (at_sockets[sock->fd] != sock) {
            errno = ENOTSOCK;
            return -1;
        }
        if (wifi_debug) LOGQ(WIFI_TAG, "Close socket %d (%d)", sock->fd, sock->link_id);
        sprintf(cmd, "AT+TCPCLOSE=%d\r\n", sock->link_id);
    }

    bool closed = sock->peer_closed;
    int wait_end = mp_hal_ticks_ms() + 2000;

    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 500;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;

    while (mp_hal_ticks_ms() < wait_end) {
        if (!closed) {
            res = wifi_at_Cmd(&at_command);
            // ERROR may be received if the link was closed,
            // after all data received, so it is considered OK
            if ((res != 1) && (!sock->peer_closed)) {
                at_command.dbg = false;
                vTaskDelay(50 / portTICK_RATE_MS);
                continue;
            }
            else closed = true;
        }
        // Closed
        if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) {
            vTaskDelay(50 / portTICK_RATE_MS);
            continue;
        }

        if (sock->buffer.buf) {
            if (sock->static_buffer == mp_const_none) {
                // Free socket buffer if allocated on FreeRTOS heap
                vPortFree(sock->buffer.buf);
                sock->buffer.buf = NULL;
            }
        }

        if ((sock->listening) && (srv_n < AT_MAX_SERV_SOCKETS)) at_server_socket[srv_n] = NULL;
        else at_sockets[sock->fd] = NULL;

        if (sock->semaphore) vSemaphoreDelete(sock->semaphore);
        if (sock->mutex) vSemaphoreDelete(sock->mutex);
        sock->semaphore = NULL;
        sock->mutex = NULL;

        if (sock->parent_sock != NULL) {
            // update listening socket connection number
            socket_obj_t *lsock = (socket_obj_t *)sock->parent_sock;
            if ((lsock != NULL) && (lsock->active_conn > 0)) lsock->active_conn--;
            if (wifi_debug) LOGQ(WIFI_TAG, "Close: parent socket (%d) active connections: %d", lsock->fd, lsock->active_conn);
        }

        xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
        vTaskDelay(10 / portTICK_RATE_MS);
        errno = 0;
        return 0;
    }

    sock->peer_closed = true;
    errno = ENOTCONN;
    vTaskDelay(10 / portTICK_RATE_MS);
    return -1;
}

//------------------------------------------------------------------------------------------------------
static int _wifi_send(socket_obj_t *sock, const char *data, size_t data_len, const char *host, int port)
{
    if ((at_sockets[sock->fd] != sock) || (at_sockets[sock->fd]->peer_closed)) {
        if (wifi_debug) LOGE(WIFI_TAG, "Send: Not a socket %d (%d, %d)", sock->fd, (at_sockets[sock->fd] != sock), (at_sockets[sock->fd]->peer_closed));
        errno = ENOTSOCK;
        return -1;
    }
    char cmd[256] = {'\0'};
    char send_resp[32] = {'\0'};

    at_responses_t at_responses1;
    memset(&at_responses1, 0, sizeof(at_responses_t));
    at_responses1.nresp = 4;
    // '+TCPSEND:n' is returned before 'OK'
    if (wifi_tcpsend_wait_sent) at_responses1.resp[0] = "\r\n+TCPSEND:Complete\r\n";
    else at_responses1.resp[0] = send_resp;
    at_responses1.resp[1] = AT_OK_Str;
    at_responses1.resp[2] = AT_Error_Str;
    at_responses1.resp[3] = AT_Fail_Str;

    at_commands_t at_commands;
    memset(&at_commands, 0, sizeof(at_commands_t));
    at_commands.ncmd = 2;
    at_commands.at_uart_num = wifi_uart_num;
    at_commands.dbg = wifi_debug;
    at_commands.expect_resp[0] = 1;
    //at_commands.delay[0] = 0;
    //at_commands.delay[1] = 0;

    memset(&at_responses, 0, sizeof(at_responses_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = "\r\n>";
    at_responses.resp[1] = "\r\n+TCPSEND:Busy\r\n";

    at_commands.commands[0].cmd = cmd;
    at_commands.commands[0].cmdSize = -1;
    at_commands.commands[0].responses = &at_responses;
    at_commands.commands[0].timeout = 50;
    at_commands.commands[0].at_uart_num = wifi_uart_num;
    at_commands.commands[0].dbg = wifi_debug;

    at_commands.commands[1].cmd = NULL;
    at_commands.commands[1].cmdSize = 0;
    at_commands.commands[1].type_data = true;
    at_commands.commands[1].responses = &at_responses1;
    at_commands.commands[1].timeout = 1000;
    at_commands.commands[1].at_uart_num = wifi_uart_num;
    at_commands.commands[1].dbg = wifi_debug;
    if (wifi_debug) {
        at_commands.commands[1].respbuff = pvPortMalloc(256);
        if (at_commands.commands[1].respbuff) {
            memset(at_commands.commands[1].respbuff, 0, 256);
            at_commands.commands[1].respbSize = 256;
        }
    }

    // Only up to 2048 bytes can be sent at once
    // if we need to send more data, it has to be divided into 2K blocks
    size_t remain = data_len;
    size_t to_send;
    size_t idx = 0;
    int res, n_proc = 0;
    int n_try = 10;

    mp_hal_wdt_reset();
    while (remain > 0) {
        to_send = (remain > 2048) ? 2048 : remain;
        // prepare the command
        sprintf(cmd, "AT+TCPSEND=%d,%lu\r\n", sock->link_id, to_send);
        sprintf(send_resp, "\r\n+TCPSEND:%lu\r\n", to_send);
        at_commands.commands[0].cmd = cmd;
        at_commands.commands[1].cmd = data + idx;
        at_commands.commands[1].cmdSize = to_send;

        res = wifi_at_Commands(&at_commands, &n_proc);
        if (wifi_debug) LOGI(WIFI_TAG, "Send: response=%d,%d", res, n_proc);

        if ((n_proc == 2) && ((res == 1) || (res == 2))) {
            // Send OK, prepare next block
            remain -= to_send;
            idx += to_send;
            wifi_tx_count += to_send;
        }
        else {
            // error sending
            if ((n_proc == 1) && (res != 1)) {
                // === Prompt not received
                //     Busy (previous send not yet finished) or command not received,
                //     try to send again ===
                n_try--;
                if (n_try == 0) {
                    // quit trying, but
                    if (wifi_debug) LOGE(WIFI_TAG, "Send: no prompt");
                    errno = ETIMEDOUT;
                    return -2;
                }
                if (wifi_debug) LOGQ(WIFI_TAG, "Send: No prompt (%d,%d)", res, n_proc);
                vTaskDelay(50 / portTICK_RATE_MS);
                continue;
            }
            // === Data not sent successfully ===
            if (wifi_debug) {
                LOGE(WIFI_TAG, "Send: error sending data");
                if (at_commands.commands[1].respbuff) {
                    LOGQ(WIFI_TAG, "Status: [%s]", at_commands.commands[1].respbuff);
                    vPortFree(at_commands.commands[1].respbuff);
                }
            }
            errno = ENETUNREACH;
            return -1;
        }
        if (!wifi_tcpsend_wait_sent) vTaskDelay(10 / portTICK_RATE_MS);
        n_try = 10;
        mp_hal_wdt_reset();
    }

    // All blocks sent
    if (at_commands.commands[1].respbuff) {
        LOGI(WIFI_TAG, "Status: [%s]", at_commands.commands[1].respbuff);
        vPortFree(at_commands.commands[1].respbuff);
    }

    errno = 0;
    return idx;
}

//------------------------------------------------------------------
int wifi_send(socket_obj_t *sock, const char *data, size_t data_len)
{
    return _wifi_send(sock, data, data_len, NULL, 0);
}

//-------------------------------------------------------------------------------------------------
int wifi_send_to(socket_obj_t *sock, const char *data, size_t data_len, const char *host, int port)
{
    return _wifi_send(sock, data, data_len, host, port);
}

//----------------------------------------
int wifi_sock_buff_len(socket_obj_t *sock)
{
    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*2) != pdTRUE) {
        // Cannot acquire mutex, WiFi task probably receiving data
        return 0;
    }
    int len = sock->buffer.length;
    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
    return len;
}

// Check if the socket buffer contains line end pattern
//----------------------------------------------------------------------
char *wifi_read_lineend(socket_obj_t *sock, const char *lend, int *size)
{
    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*2) != pdTRUE) {
        // Cannot acquire mutex, WiFi task probably receiving data
        return NULL;
    }

    char *data = NULL;
    int pos = -1;
    int buflen = uart_buf_length(&sock->buffer, NULL);

    if (buflen > 0) pos = uart_buf_find(&sock->buffer, buflen, (const char *)lend, strlen(lend), NULL);
    if (pos >= 0) {
        //if (wifi_debug) LOGQ(WIFI_TAG, "Lineend: found [%s] at pos %d", lend, pos);
        data = pvPortMalloc(pos+strlen(lend)+1);
        if (data != NULL) {
            memset(data, 0, pos+strlen(lend)+1);
            pos = uart_buf_get(&sock->buffer, (uint8_t *)data, pos+strlen(lend));
            *size = pos;
            //if (wifi_debug) LOGQ(WIFI_TAG, "Lineend: got data [%s] len=%lu (%d)", data, strlen(data), pos);
        }
        else {
            if (wifi_debug) LOGE(WIFI_TAG, "Lineend: Error allocating buffer");
        }
    }

    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
    return data;
}

//------------------------------------------------------------------
int wifi_read(socket_obj_t *sock, const char *data, size_t data_len)
{
    if (at_sockets[sock->fd] != sock) {
        errno = ENOTSOCK;
        return -1;
    }

    // === Return received data ===
    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*4) != pdTRUE) {
        // Cannot acquire mutex, WiFi task probably receiving data
        errno = EWOULDBLOCK;
        return -1;
    }

    if (((sock->buffer.size == 0) || (sock->buffer.length == 0)) && (sock->peer_closed)) {
        // no data in buffer and peer closed
        xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
        //errno = ENOTCONN;
        //return -1;
        errno = 0;
        return 0;
    }
    else if (sock->buffer.length == 0) {
        // no data in buffer (peer still connected)
        xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
        errno = EWOULDBLOCK;
        return -1;
    }

    int rdlen = uart_buf_get(&sock->buffer, (uint8_t *)data, data_len);

    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
    errno = 0;
    return rdlen;
}

//---------------------------------------------------------
int wifi_get_link_ip_port(int link_id, char *IP, int *port)
{
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 3;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_responses.resp[2] = AT_Busy_Str;
    char cmd[32];
    sprintf(cmd, "AT+TCPSTATUS=%d\r\n", link_id);
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 100;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.respbuff = pvPortMalloc(128);
    at_command.respbSize = 128;
    if (at_command.respbuff == NULL) {
        if (wifi_debug) {
            LOGE(WIFI_TAG, "Get IP_port: Buffer allocation error");
        }
        return -1;
    }
    memset(at_command.respbuff, 0, at_command.respbSize);

    int res = -1;
    while (1) {
        res = wifi_at_Cmd(&at_command);
        if (res != 3) break;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    if (res != 1) {
        if (wifi_debug) {
            LOGE(WIFI_TAG, "Get IP_port: wrong response (%d) [%s]", res, at_command.respbuff);
        }
        vPortFree(at_command.respbuff);
        return -2;
    }

    // === Parse the response ===
    char s_IP[32] = {'\0'};
    char s_port[32] = {'\0'};
    char *presp = NULL;
    char *presp_end = NULL;

    // +TCPSTATUS:"IPaddr",port,is_server
    // +TCPSTATUS:"192.168.0.63",50888,1
    // OK
    res = -4;
    presp = strstr(at_command.respbuff, "+TCPSTATUS:\"");
    if (presp) {
        presp += 12;

        presp_end = strstr(presp, "\",");
        if (presp_end) {
            memcpy(s_IP, presp, (presp_end-presp));
            presp = presp_end + 2;
            presp_end = strstr(presp, ",");
            if (presp_end) {
                memcpy(s_port, presp, (presp_end-presp));
                *port = strtol(s_port, (char **)NULL, 10);
                strcpy(IP, s_IP);
                res = 1;
            }
        }
    }

    vPortFree(at_command.respbuff);
    return res;
}

//---------------
bool wifi_reset()
{
    int res = 0;
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 1;
    at_responses.resp[0] = "ready\r\n";
    at_command.responses = &at_responses;
    at_command.timeout = 2000;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.flush = true;

    at_command.cmd = "AT+RST\r\n";
    at_command.cmdSize = -1;

    res = wifi_at_Cmd(&at_command);
    if (res == 1) {
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        wifi_restart_task = true;
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // Wait until task is ready
        int tmo = 20000;
        while (tmo >= 0) {
            vTaskDelay(10 / portTICK_RATE_MS);
            tmo -= 10;
            if (wifi_status == ATDEV_STATEIDLE) return true;
        }
        if (tmo <= 0) return false;
    }

    return (res == 1);
}

//---------------------------------------------------------
int wifi_sendcertificate(const char *data, size_t data_len)
{
    if (data_len > 4062) return -1;

    char cmd[256];
    bool send_len = (data[data_len-1] != '^');

    at_responses_t at_responses1;
    memset(&at_responses1, 0, sizeof(at_responses_t));
    at_responses1.nresp = 2;
    at_responses1.resp[0] = AT_OK_Str;
    at_responses1.resp[1] = AT_Error_Str;

    at_commands_t at_commands;
    memset(&at_commands, 0, sizeof(at_commands_t));
    at_commands.ncmd = 2;
    at_commands.at_uart_num = wifi_uart_num;
    at_commands.dbg = wifi_debug;
    at_commands.expect_resp[0] = 1;
    //at_commands.delay[0] = 0;
    //at_commands.delay[1] = 0;

    memset(&at_responses, 0, sizeof(at_responses_t));
    at_responses.nresp = 1;
    at_responses.resp[0] = "\r\n>";
    //at_responses.resp[1] = AT_Error_Str;

    if (data_len == 0) sprintf(cmd, "AT+SSLLOADCERT=0,0\r\n");
    else if (send_len) sprintf(cmd, "AT+SSLLOADCERT=0,%lu\r\n", data_len);
    else sprintf(cmd, "AT+SSLLOADCERT=0\r\n");
    at_commands.commands[0].cmd = cmd;
    at_commands.commands[0].cmdSize = -1;
    at_commands.commands[0].responses = &at_responses;
    at_commands.commands[0].timeout = 50;
    at_commands.commands[0].at_uart_num = wifi_uart_num;
    at_commands.commands[0].dbg = wifi_debug;

    at_commands.commands[1].cmd = NULL;
    at_commands.commands[1].cmdSize = 0;
    at_commands.commands[1].type_data = true;
    at_commands.commands[1].responses = &at_responses1;
    at_commands.commands[1].timeout = 1000;
    at_commands.commands[1].at_uart_num = wifi_uart_num;
    at_commands.commands[1].dbg = wifi_debug;
    /*if (wifi_debug) {
        at_commands.commands[1].respbuff = pvPortMalloc(256);
        if (at_commands.commands[1].respbuff) {
            memset(at_commands.commands[1].respbuff, 0, 256);
            at_commands.commands[1].respbSize = 256;
        }
    }*/

    at_commands.commands[1].cmd = data;
    at_commands.commands[1].cmdSize = data_len;

    int n_proc = 0;
    int res = wifi_at_Commands(&at_commands, &n_proc);

    if (at_commands.commands[1].respbuff) {
        LOGQ(WIFI_TAG, "Status: [%s]", at_commands.commands[1].respbuff);
        vPortFree(at_commands.commands[1].respbuff);
    }
    if ((n_proc == 2) && (res == 1)) return 0;

    if (wifi_debug) {
        if (n_proc == 1) LOGE(WIFI_TAG, "Send: no prompt");
        else LOGE(WIFI_TAG, "Send: error sending data");
    }
    return -1;
}

//-----------------------------
bool wifi_set_baudrate(int bdr)
{
    char cmd[128];
    sprintf(cmd, "AT+UART=%d,8,1,0,0\r\n", bdr);
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 500;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    int res = wifi_at_Cmd(&at_command);

    if (res == 1) {
        mp_uart_config(wifi_uart_num, bdr, 8, UART_STOP_1, UART_PARITY_NONE);
        wifi_uart_baudrate = bdr;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return (res == 1);
}

//-------------------------
int wifi_sslcconf(int mode)
{
    char cmd[128];
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));

    if (mode < 0) {
        sprintf(cmd, "AT+SSLCCONF?\r\n");
        at_responses.nresp = 5;
        at_responses.resp[0] = "+SSLCCONF:0";
        at_responses.resp[1] = "+SSLCCONF:1";
        at_responses.resp[2] = "+SSLCCONF:2";
        at_responses.resp[3] = "+SSLCCONF:3";
        at_responses.resp[4] = AT_OK_Str;
    }
    else {
        mode &= 3;
        sprintf(cmd, "AT+SSLCCONF=%d\r\n", mode);
        at_responses.nresp = 2;
        at_responses.resp[0] = AT_OK_Str;
        at_responses.resp[1] = AT_Error_Str;
    }
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 100;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    int res = wifi_at_Cmd(&at_command);

    if (mode < 0) {
        if ((res <= 1) && (res > 4)) return -1;
        return (res - 1);
    }
    if (res != 1) return -1;
    return mode;
}

//----------------------------
int wifi_set_cpufreq(int freq)
{
    char cmd[128];
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    if (freq <= 0) {
        sprintf(cmd, "AT+SYSCPUFREQ?\r\n");
        at_responses.nresp = 3;
        at_responses.resp[0] = "+SYSCPUFREQ:80";
        at_responses.resp[1] = "+SYSCPUFREQ:160";
        at_responses.resp[2] = AT_Error_Str;
    }
    else {
        sprintf(cmd, "AT+SYSCPUFREQ=%d\r\n", freq);
        at_responses.nresp = 2;
        at_responses.resp[0] = AT_OK_Str;
        at_responses.resp[2] = AT_Error_Str;
    }
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 100;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    int res = wifi_at_Cmd(&at_command);

    if (freq <= 0) {
        if ((res != 1) && (res != 2)) return -1;
        else if (res == 1) return 80;
        return 160;
    }
    return res;
}

//--------------------------------------
bool wifi_set_ssl_buffer_size(int *size)
{
    int res = 0;
    char cmd[128];
    bool get_size = (*size == 0);

    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.timeout = 500;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;

    if (!get_size) {
        if ((*size < 2048) || (*size > 16384)) return false;
        sprintf(cmd, "AT+CIPSSLSIZE=%d\r\n", *size);
    }
    else {
        sprintf(cmd, "AT+CIPSSLSIZE?\r\n");
        at_command.respbuff = pvPortMalloc(128);
        if (at_command.respbuff == NULL) {
            return false;
        }
        at_command.respbSize = 128;
        memset(at_command.respbuff, 0, at_command.respbSize);
    }
    at_command.responses = &at_responses;
    at_command.cmd = cmd;
    at_command.cmdSize = -1;

    res = wifi_at_Cmd(&at_command);
    if (res != 1) return false;
    if (!get_size) return true;

    char *presp = strstr(at_command.respbuff, "+CIPSSLSIZE:");
    if (presp) {
        sscanf(presp+12, "%d", size);
        vPortFree(at_command.respbuff);
        return true;
    }
    vPortFree(at_command.respbuff);
    return false;
}

//-----------------------------
int wifi_ping(const char *host)
{
    char cmd[128];

    sprintf(cmd, "AT+PING=\"%s\"\r\n", host);
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = "+timeout";
    at_command.cmd = cmd;
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 4000;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.respbSize = 128;
    at_command.respbuff = pvPortMalloc(128);
    if (at_command.respbuff == NULL) {
        return -4;
    }
    memset(at_command.respbuff, 0, at_command.respbSize);

    int res = wifi_at_Cmd(&at_command);
    int resp_time = -1;
    if (res == 1) {
        resp_time = -3;
        char *presp = strstr(at_command.respbuff, "+PING:");
        if (presp) sscanf(presp+6, "%d", &resp_time);
        else sscanf(at_command.respbuff, "%d", &resp_time);
    }
    else if (res == 2) resp_time = -2;
    vPortFree(at_command.respbuff);

    return resp_time;
}
//-----------------------------------------------
int wifi_ifconfig(char *ip, char *gw, char *mask)
{
    //+CIPSTA:ip:"192.168.0.16"\r\n+CIPSTA:gateway:"192.168.0.1"\r\n+CIPSTA:netmask:"255.255.255.0"\r\n\r\nOK\r\n

    ip[0] = '\0';
    gw[0] = '\0';
    mask[0] = '\0';

    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = "AT+CIPSTA?\r\n";
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 500;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.respbSize = 256;
    at_command.respbuff = pvPortMalloc(256);
    if (at_command.respbuff == NULL) return -1;
    memset(at_command.respbuff, 0, at_command.respbSize);

    int res = wifi_at_Cmd(&at_command);
    if (res == 1) {
        char *pbuf = at_command.respbuff;
        char *pstart;
        char *pend;
        // Get IP address
        pstart = strstr(pbuf, "+CIPSTA:ip:\"");
        if (pstart) {
            pbuf = pstart + 12;
            pend = strchr(pbuf, '"');
            if (pend) {
                *pend = '\0';
                strcpy(ip, pbuf);
                // Get gateway
                pbuf = pend + 1;
                pstart = strstr(pbuf, "+CIPSTA:gateway:\"");
                if (pstart) {
                    pbuf = pstart + 17;
                    pend = strchr(pbuf, '"');
                    if (pend) {
                        *pend = '\0';
                        strcpy(gw, pbuf);
                        // Get net mask
                        pbuf = pend + 1;
                        pstart = strstr(pbuf, "+CIPSTA:netmask:\"");
                        if (pstart) {
                            pbuf = pstart + 17;
                            pend = strchr(pbuf, '"');
                            if (pend) {
                                *pend = '\0';
                                strcpy(mask, pbuf);
                                // Get net mask
                                pbuf = pend + 1;;
                            }
                        }
                    }
                }
            }
            else res = -3;
        }
        else res = -3;
    }
    else res = -2;

    vPortFree(at_command.respbuff);
    return res;
}

//-------------------------------------
int wifi_getdns(char *dns1, char *dns2)
{
    //+CIPDNS_CUR:83.139.103.3\r\n+CIPDNS_CUR:83.139.121.8\r\n\r\nOK

    dns1[0] = '\0';
    dns2[0] = '\0';

    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = "AT+CIPDNS_CUR?\r\n";
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 500;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.respbSize = 256;
    at_command.respbuff = pvPortMalloc(256);
    if (at_command.respbuff == NULL) return -1;
    memset(at_command.respbuff, 0, at_command.respbSize);

    int res = wifi_at_Cmd(&at_command);
    if (res == 1) {
        char *pbuf = at_command.respbuff;
        char *pstart;
        char *pend;
        // Get 1st DNS
        pstart = strstr(pbuf, "+CIPDNS_CUR:");
        if (pstart) {
            pbuf = pstart + 12;
            pend = strchr(pbuf, '\r');
            if (pend) {
                *pend = '\0';
                strcpy(dns1, pbuf);
                // Get 2nd DNS
                pbuf = pend + 1;;
                pstart = strstr(pbuf, "+CIPDNS_CUR:");
                if (pstart) {
                    pbuf = pstart + 12;
                    pend = strchr(pbuf, '\r');
                    if (pend) {
                        *pend = '\0';
                        strcpy(dns2, pbuf);
                    }
                }
            }
            else res = -3;
        }
        else res = -3;
    }
    else res = -2;

    vPortFree(at_command.respbuff);
    return res;
}

/*
//-------------------------------------------------------
static bool _wifi_get_time(time_t *seconds, bool set_rtc)
{
    bool ret = false;
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = "AT+CIPSNTPTIME?\r\n";
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 1000;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.respbSize = 256;
    at_command.respbuff = pvPortMalloc(256);
    if (at_command.respbuff == NULL) {
        return false;
    }
    memset(at_command.respbuff, 0, at_command.respbSize);

    int res = wifi_at_Cmd(&at_command);

    if (res == 1) {
        // '+CIPSNTPTIME:Thu Mar 21 15:38:52 2019\r\nOK\r\n'
        char *tstartp = strstr(at_command.respbuff, "+CIPSNTPTIME:");
        if (tstartp) {
            tstartp += 13;
            char *tendp = strchr(at_command.respbuff, '\r');
            if ((tendp) && ((tendp-tstartp) > 20)) {
                char time_str[32] = {'\0'};
                memcpy(time_str, tstartp, (tendp-tstartp));
                // 'Thu Mar 21 15:38:52 2019'
                struct tm tm;
                char sday[8], smonth[8];
                int day, hour, mnt, sec, month = -1, year;
                sscanf(time_str, "%s %s %d %d:%d:%d %d", sday, smonth, &day, &hour, &mnt, &sec, &year);
                for (int i=0; i<12; i++) {
                    if (strstr(months_names[i], smonth)) {
                        month = i;
                        break;
                    }
                }
                if (month >= 0) {
                    tm.tm_year = year - 1900;
                    tm.tm_mon = month;
                    tm.tm_mday = day;
                    tm.tm_hour = hour;
                    tm.tm_min = mnt;
                    tm.tm_sec = sec;
                    tm.tm_wday = 0;
                    tm.tm_yday = 0;
                    time_t time_sec = mktime(&tm);
                    if (seconds) *seconds = time_sec;
                    if (set_rtc) {
                        // Set system time
                        if (time_sec > 0) rtc_set_datetime(mp_rtc_rtc0, &tm); // set RTC time
                    }
                    ret = true;
                }
            }
        }
    }

    if (at_command.respbuff) vPortFree(at_command.respbuff);
    return ret;
}
*/

//-----------------------------------------------
bool wifi_get_time(time_t *seconds, bool set_rtc)
{
    bool ret = false;
    memset(&at_responses, 0, sizeof(at_responses_t));
    memset(&at_command, 0, sizeof(at_command_t));
    at_responses.nresp = 2;
    at_responses.resp[0] = AT_OK_Str;
    at_responses.resp[1] = AT_Error_Str;
    at_command.cmd = "AT+SNTPTIME?\r\n";
    at_command.cmdSize = -1;
    at_command.responses = &at_responses;
    at_command.timeout = 1000;
    at_command.at_uart_num = wifi_uart_num;
    at_command.dbg = wifi_debug;
    at_command.respbSize = 256;
    at_command.respbuff = pvPortMalloc(256);
    if (at_command.respbuff == NULL) {
        return false;
    }
    memset(at_command.respbuff, 0, at_command.respbSize);

    int res = wifi_at_Cmd(&at_command);

    if (res == 1) {
        // '+SNTPTIME:1556721502,2019-05-01 14:38:22\r\nOK\r\n'
        char *tstartp = strstr(at_command.respbuff, "+SNTPTIME:");
        if (tstartp) {
            tstartp += 10;
            char *tendp = strchr(at_command.respbuff, ',');
            if (tendp) {
                *tendp = '\0';
                struct tm *tm;
                time_t time_sec;
                sscanf(tstartp, "%ld", &time_sec);
                tm = localtime(&time_sec);
                if (seconds) *seconds = time_sec;
                if (set_rtc) {
                    // Set system time
                    //if (time_sec > 0) rtc_set_datetime(mp_rtc_rtc0, tm); // set RTC time
                    if (time_sec > 0) _set_sys_time(tm, 0);
                }
                ret = true;
            }
        }
    }

    if (at_command.respbuff) vPortFree(at_command.respbuff);
    return ret;
}

//-------------------------------------------------------------
void wifi_getRxTxCount(uint32_t *rx, uint32_t *tx, uint8_t rst)
{
    *rx = 0;
    *tx = 0;
    if (mpy_uarts[wifi_uart_num].uart_mutex == NULL) return;

    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) return;
    *rx = wifi_rx_count;
    *tx = wifi_tx_count;
    if (rst) {
        wifi_rx_count = 0;
        wifi_tx_count = 0;
    }
    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
}

//------------------------
void wifi_resetRxTxCount()
{
    if (mpy_uarts[wifi_uart_num].uart_mutex == NULL) return;
    if (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) return;
    wifi_rx_count = 0;
    wifi_tx_count = 0;
    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
}

//--------------------
bool wifi_take_mutex()
{
    return (xSemaphoreTake(mpy_uarts[wifi_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*10) == pdTRUE);
}

//--------------------
void wifi_give_mutex()
{
    xSemaphoreGive(mpy_uarts[wifi_uart_num].uart_mutex);
}



// ===== MicroPython bindings ===============================================================

//-------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_start(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
            { MP_QSTR_tx,           MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = -1} },
            { MP_QSTR_rx,           MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = -1} },
            { MP_QSTR_baudrate,                       MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 115200} },
            { MP_QSTR_ssid,                           MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_password,                       MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_wait,                           MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char ssid[PPP_MAX_NAME_LEN+1] = {0};
    char pass[PPP_MAX_NAME_LEN+1] = {0};
    int rx, tx, bdr;

    if (mp_obj_is_str(args[3].u_obj)) {
        snprintf(ssid, PPP_MAX_NAME_LEN, mp_obj_str_get_str(args[3].u_obj));
    }

    if (mp_obj_is_str(args[4].u_obj)) {
        snprintf(pass, PPP_MAX_NAME_LEN, mp_obj_str_get_str(args[4].u_obj));
    }

    tx = args[0].u_int;
    rx = args[1].u_int;
    if ((tx < 0) || (rx < 0)) goto exit;

    bdr = args[2].u_int;

    int res = wifi_Init(tx, rx, bdr, ssid, pass, args[5].u_bool);

    if (res == 0) return mp_const_true;

exit:
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_wifi_start_obj, 0, mod_wifi_start);

//----------------------------------------------------------------
STATIC mp_obj_t mod_wifi_stop(size_t n_args, const mp_obj_t *args)
{
    bool wait = false;
    if (n_args > 0) wait = mp_obj_is_true(args[0]);
    if (wifi_endTask(wait) == 0) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_stop_obj, 0, 1, mod_wifi_stop);

//-------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_atCmd(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    return mpy_atCmd(n_args, pos_args, kw_args, wifi_uart_num, wifi_debug, WIFI_TAG);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_wifi_atCmd_obj, 1, mod_wifi_atCmd);


//------------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_get_counters(size_t n_args, const mp_obj_t *args)
{
    uint8_t rst = 0;
    uint32_t rx=0, tx=0;
    if (n_args > 0) rst = mp_obj_is_true(args[0]);

    wifi_getRxTxCount(&rx, &tx, rst);

    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(rx);
    tuple[1] = mp_obj_new_int(tx);

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_get_counters_obj, 0, 1, mod_wifi_get_counters);

//------------------------------
STATIC mp_obj_t mod_wifi_state()
{
    mp_obj_t tuple[2];
    char state[20] = {'\0'};
    int WiFi_state = wifiStatus();

    tuple[0] = mp_obj_new_int(WiFi_state);

    if (WiFi_state == ATDEV_STATEDISCONNECTED) sprintf(state, "Disconnected");
    else if (WiFi_state == ATDEV_STATECONNECTED) sprintf(state, "Connected");
    else if (WiFi_state == ATDEV_STATEIDLE) sprintf(state, "Idle");
    else if (WiFi_state == ATDEV_STATEFIRSTINIT) sprintf(state, "Not started");
    else sprintf(state, "Unknown");
    tuple[1] = mp_obj_new_str(state, strlen(state));

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_wifi_state_obj, mod_wifi_state);

//-----------------------------------------------------------------
STATIC mp_obj_t mod_wifi_debug(size_t n_args, const mp_obj_t *args)
{
    if (n_args > 0) wifi_setDebug(mp_obj_is_true(args[0]));
    bool dbg = wifi_getDebug();
    if (dbg) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_debug_obj, 0, 1, mod_wifi_debug);

//------------------------------
STATIC mp_obj_t mod_wifi_reset()
{
    wifi_reset();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_wifi_reset_obj, mod_wifi_reset);

//-------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_ntptime(size_t n_args, const mp_obj_t *args)
{
    bool set_rtc = false;
    if (n_args > 0) set_rtc = mp_obj_is_true(args[0]);

    time_t seconds = 0;
    bool res = wifi_get_time(&seconds, set_rtc);
    if (!res) return mp_const_false;
    return mp_obj_new_int(seconds);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_ntptime_obj, 0, 1, mod_wifi_ntptime);

//--------------------------------------------------
STATIC mp_obj_t mod_wifi_sslbuffer(size_t n_args, const mp_obj_t *args)
{
    bool res = false;
    int size = 0;
    if (n_args > 0) size = mp_obj_get_int(args[0]);

    res = wifi_set_ssl_buffer_size(&size);
    if (!res) return mp_const_false;
    return mp_obj_new_int(size);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_sslbuffer_obj, 0, 1, mod_wifi_sslbuffer);

//--------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_baudrate(size_t n_args, const mp_obj_t *args)
{
    if (n_args == 0) {
        return mp_obj_new_int(wifi_uart_baudrate);
    }

    int baudrate = mp_obj_get_int(args[0]);
    int i = BDRATES_MAX;
    for (i=0; i<BDRATES_MAX; i++) {
        if (baudrate == bd_rates[i]) break;
    }
    if (i >= BDRATES_MAX) {
        mp_raise_ValueError("Wrong baudrate");
    }

    bool res = wifi_set_baudrate(baudrate);

    if (!res) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_baudrate_obj, 0, 1, mod_wifi_baudrate);

//-------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_cpufreq(size_t n_args, const mp_obj_t *args)
{
    int res;
    int freq;
    if (n_args == 0) {
        res = wifi_set_cpufreq(-1);
        if (res > 0) return mp_obj_new_int(res);
        return mp_const_false;
    }

    freq = mp_obj_get_int(args[0]);
    if ((freq != 80) && (freq != 160)) {
        mp_raise_ValueError("Only 80 or 160 MHz can be set");
    }
    res = wifi_set_cpufreq(freq);

    if (res == 1) return mp_obj_new_int(freq);
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_cpufreq_obj, 0, 1, mod_wifi_cpufreq);

//-------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_sslmode(size_t n_args, const mp_obj_t *args)
{
    int res;
    int mode;
    if (n_args == 0) {
        res = wifi_sslcconf(-1);
    }
    else {
        mode = mp_obj_get_int(args[0]);
        if ((mode < 0) || (mode > 3)) {
            mp_raise_ValueError("Allowed range: 0 ~ 3");
        }
        res = wifi_sslcconf(mode);
    }

    return (res < 0) ? mp_const_false : mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_sslmode_obj, 0, 1, mod_wifi_sslmode);

//-------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_ifconfig(size_t n_args, const mp_obj_t *args)
{
    int res;
    char ip[18] = {'\0'};
    char gw[18] = {'\0'};
    char mask[18] = {'\0'};
    char dns1[18] = {'\0'};
    char dns2[18] = {'\0'};

    mp_obj_t tuple[5];
    for (int i=0; i<5; i++) {
        tuple[i] = mp_const_none;
    }
    res = wifi_ifconfig(ip, gw, mask);
    if (res == 1) {
        res = wifi_getdns(dns1, dns2);
        tuple[0] = mp_obj_new_str(ip, strlen(ip));
        tuple[1] = mp_obj_new_str(gw, strlen(gw));
        tuple[2] = mp_obj_new_str(mask, strlen(mask));
        if (res == 1) {
            tuple[3] = mp_obj_new_str(dns1, strlen(dns1));
            tuple[4] = mp_obj_new_str(dns2, strlen(dns2));
        }
    }
    return mp_obj_new_tuple(5, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_ifconfig_obj, 0, 1, mod_wifi_ifconfig);

//-----------------------------------------------------------------------
STATIC mp_obj_t mod_wifi_certificate(size_t n_args, const mp_obj_t *args)
{
    int res;
    if (n_args == 0) {
        char cmd[128];

        sprintf(cmd, "AT+SSLLOADCERT?\r\n");
        memset(&at_responses, 0, sizeof(at_responses_t));
        memset(&at_command, 0, sizeof(at_command_t));
        at_responses.nresp = 1;
        at_responses.resp[0] = AT_OK_Str;
        at_command.cmd = cmd;
        at_command.cmdSize = -1;
        at_command.responses = &at_responses;
        at_command.timeout = 500;
        at_command.at_uart_num = wifi_uart_num;
        at_command.dbg = wifi_debug;
        at_command.respbSize = 4096;
        at_command.respbuff = pvPortMalloc(4096);
        if (at_command.respbuff == NULL) return mp_const_false;
        memset(at_command.respbuff, 0, at_command.respbSize);

        mp_obj_t response = mp_const_false;
        res = wifi_at_Cmd(&at_command);
        if (res == 1) {
            char *pbuf_sect = strstr(at_command.respbuff, "+SSLLOADCERT:0");
            if (pbuf_sect != NULL) {
                if (strlen(at_command.respbuff) > 30) {
                    pbuf_sect += 14;
                    int cert_len = 0;
                    int cert_sect = 0;
                    if (*pbuf_sect == ',') {
                        pbuf_sect++;
                        char *pbuf_len = strstr(pbuf_sect, ",");
                        if (pbuf_len != NULL) {
                            *pbuf_len = '\0';
                            pbuf_len++;
                            cert_sect = strtol(pbuf_sect, (char **)NULL, 16);
                            char *pbuf_cert = strstr(pbuf_len, "\r\n");
                            if (pbuf_cert != NULL) {
                                *pbuf_cert = '\0';
                                pbuf_cert += 2;
                                cert_len = strtol(pbuf_len, (char **)NULL, 10);

                                mp_obj_t tuple[3];
                                tuple[0] = mp_obj_new_int(cert_sect);
                                tuple[1] = mp_obj_new_int(cert_len);
                                tuple[2] = mp_obj_new_str(pbuf_cert, cert_len);
                                response = mp_obj_new_tuple(3, tuple);
                            }
                        }
                    }
                }
            }
        }
        vPortFree(at_command.respbuff);
        return response;
    }

    size_t data_len;
    const char *cert_data = mp_obj_str_get_data(args[0], &data_len);

    res = wifi_sendcertificate(cert_data, data_len);

    if (res == 0) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_certificate_obj, 0, 1, mod_wifi_certificate);

//----------------------------------------------------------------
STATIC mp_obj_t mod_wifi_ping(size_t n_args, const mp_obj_t *args)
{
    int n = 1;
    int ping_time, max_ping_time = -1;
    const char *host = mp_obj_str_get_str(args[0]);
    if (n_args > 1) {
        n = mp_obj_get_int(args[1]);
        if (n < 1) n = 1;
        if (n > 10) n = 10;
    }

    mp_printf(&mp_plat_print, "Ping %s\r\n", host);
    for (int i=0; i<n; i++) {
        ping_time = wifi_ping(host);
        if (ping_time >= 0) {
            if (ping_time > max_ping_time) max_ping_time = ping_time;
            mp_printf(&mp_plat_print, "time=%d ms\r\n", ping_time);
        }
        else if (ping_time == -2) {
            mp_printf(&mp_plat_print, "time=timeout\r\n");
        }
        else {
            mp_printf(&mp_plat_print, "time=error (%d)\r\n", ping_time);
        }
        if ((i+1) < n) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return mp_obj_new_int(max_ping_time);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_wifi_ping_obj, 1, 2, mod_wifi_ping);

//=========================================================
STATIC const mp_rom_map_elem_t wifi_locals_dict_table[] = {
        { MP_ROM_QSTR(MP_QSTR_start),       MP_ROM_PTR(&mod_wifi_start_obj) },
        { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&mod_wifi_stop_obj) },
        { MP_ROM_QSTR(MP_QSTR_debug),       MP_ROM_PTR(&mod_wifi_debug_obj) },
        { MP_ROM_QSTR(MP_QSTR_status),      MP_ROM_PTR(&mod_wifi_state_obj) },
        { MP_ROM_QSTR(MP_QSTR_atcmd),       MP_ROM_PTR(&mod_wifi_atCmd_obj) },
        { MP_ROM_QSTR(MP_QSTR_getCounters), MP_ROM_PTR(&mod_wifi_get_counters_obj) },
        { MP_ROM_QSTR(MP_QSTR_ntptime),     MP_ROM_PTR(&mod_wifi_ntptime_obj) },
        { MP_ROM_QSTR(MP_QSTR_sslbuffer),   MP_ROM_PTR(&mod_wifi_sslbuffer_obj) },
        { MP_ROM_QSTR(MP_QSTR_baudrate),    MP_ROM_PTR(&mod_wifi_baudrate_obj) },
        { MP_ROM_QSTR(MP_QSTR_cpufreq),     MP_ROM_PTR(&mod_wifi_cpufreq_obj) },
        { MP_ROM_QSTR(MP_QSTR_ping),        MP_ROM_PTR(&mod_wifi_ping_obj) },
        { MP_ROM_QSTR(MP_QSTR_certificate), MP_ROM_PTR(&mod_wifi_certificate_obj) },
        { MP_ROM_QSTR(MP_QSTR_sslmode),     MP_ROM_PTR(&mod_wifi_sslmode_obj) },
        { MP_ROM_QSTR(MP_QSTR_reset),       MP_ROM_PTR(&mod_wifi_reset_obj) },
        { MP_ROM_QSTR(MP_QSTR_ifconfig),    MP_ROM_PTR(&mod_wifi_ifconfig_obj) },
        // Constants
};
STATIC MP_DEFINE_CONST_DICT(wifi_locals_dict, wifi_locals_dict_table);

//==============================
const mp_obj_type_t wifi_type = {
    { &mp_type_type },
    .name = MP_QSTR_wifi,
    //.print = wifi_print,
    .locals_dict = (mp_obj_dict_t*)&wifi_locals_dict,
};

#endif
