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


#include "at_util.h"
#include <time.h>
#include <string.h>
#include "devices.h"
#include "syslog.h"

#include "libGSM.h"
#include "py/runtime.h"
#include "machine_uart.h"
#include "mphalport.h"


uint8_t net_active_interfaces = ACTIVE_INTERFACE_NONE;
int bd_rates[BDRATES_MAX] = {115200, 230400, 460800, 921600, 1000000, 2000000};

static const char *TAG = "[ATCMD]";

socket_obj_t *at_sockets[AT_MAX_SOCKETS] = {NULL};
socket_obj_t *at_server_socket[AT_MAX_SERV_SOCKETS] = {NULL};

char at_canonname[DNS_MAX_NAME_LENGTH+1] = {'\0'};


// Read maximum of 'size' bytes from uart buffer, waiting maximum of 'timeout' ms
// Number of actual bytes read is returned
// !!this should be executed protected by mutex !!
//--------------------------------------------------------------------------------
int at_uart_read_bytes(int uart_n, uint8_t *data, uint32_t size, uint32_t timeout)
{
    if (mpy_uarts[uart_n].uart_buf == NULL) return -1;

    // make sure we want at least 1 char
    if (size == 0) return 0;

    int bytes_read = 0;

    if (timeout == 0) {
        // === just return the buffer content ===
        bytes_read = uart_buf_get(mpy_uarts[uart_n].uart_buf, data, size);
        return bytes_read;
    }

    // === wait until requested number of bytes received or timeout ===
    mp_hal_wdt_reset();
    int bread = 0;
    int remain = size;
    uint8_t *pdata = data;
    int wait_end = mp_hal_ticks_ms() + timeout;

    while (mp_hal_ticks_ms() < wait_end) {
        bread = uart_buf_get(mpy_uarts[uart_n].uart_buf, pdata, remain);
        bytes_read += bread;
        pdata += bread;
        remain -= bread;
        if (remain <= 0) return bytes_read;

        vTaskDelay(2);
        mp_hal_wdt_reset();
    }
    // Timeout, return as many bytes as available
    return bytes_read;
}

// !!this should be executed protected by mutex !!
//----------------------------
void at_uart_flush(int uart_n)
{
    uart_buf_flush(mpy_uarts[uart_n].uart_buf);
}

//-------------------------------------------------------------
int at_uart_write(int uart_n, const uint8_t *data, size_t size)
{
    if (xSemaphoreTake(mpy_uarts[uart_n].uart_mutex, PPPOSMUTEX_TIMEOUT*4) != pdTRUE) {
        return -1;
    }
    mpy_uarts[uart_n].uart_buffer.notify = false;
    if (mpy_uarts[uart_n].task_semaphore) xSemaphoreTake(mpy_uarts[uart_n].task_semaphore, 0);
    int res = uart_write(uart_n, data, size);
    mpy_uarts[uart_n].uart_buffer.notify = true;
    xSemaphoreGive(mpy_uarts[uart_n].uart_mutex);
    return res;
}

//--------------------------------------------------------------------------------
void at_infoCommand(char *cmd, int cmdSize, char *info, at_responses_t *responses)
{
    if ((cmd) && (cmdSize > 0) ) {
        char sresp[256] = { '\0' };
        char old_npc = kprint_nonprint_char;
        int old_kfilter = kprint_filter_nonprint;
        kprint_filter_nonprint = 1;
        kprint_nonprint_char = '~';


        if ((responses) && (responses->nresp > 0)) {
            sprintf(sresp, "Expecting: ");
            char *psresp = sresp + strlen(sresp);
            for (int i=0; i<responses->nresp; i++) {
                int len = strlen(responses->resp[i]) + ((i==0) ? 2 : 4);
                if ((strlen(sresp)+len) >= 256) break;
                sprintf(psresp, "%s[%s]", (i==0) ? "" : ", ", responses->resp[i]);
                psresp += len;
            }
        }

        LOGM(TAG,"%s [%s] %s", info, cmd, sresp);

        kprint_filter_nonprint = old_kfilter;;
        kprint_nonprint_char = old_npc;
    }
}

//----------------------------------------------------------------------------
static void _copy_to_buf(at_command_t *command, size_t from_pos, int copy_len)
{
    if (copy_len <= 0) return;
    size_t len = copy_len;
    if (command->respbuff != NULL) {
        // === Copy uart buffer to the provided output buffer (allocated on FreeRTOS heap) ===
        if ((copy_len + command->respbLen) >= command->respbSize) {
            // === Buffer too small and allocated on heap, expand it ===
            size_t old_size = command->respbSize;
            size_t new_size = command->respbSize + copy_len + 256;
            char *ptemp = pvPortMalloc(new_size);
            if (ptemp == NULL) {
                /*if (command->dbg) {
                    LOGW(TAG,"AT RESPONSE: Error expanding response buffer (%lu -> %lu)", old_size, new_size);
                }*/
                len = 0;
            }
            else {
                memcpy(ptemp, command->respbuff, old_size);
                vPortFree(command->respbuff);
                command->respbuff = ptemp;
                command->respbSize = new_size;
                /*if (command->dbg) {
                    LOGM(TAG,"AT RESPONSE: Response buffer reallocated (%lu -> %lu)", old_size, new_size);
                }*/
            }
            len = copy_len;
        }
        if (len > 0) {
            // copy data from uart buffer to the response buffer
            /*if (command->dbg) {
                LOGM(TAG,"AT RESPONSE: Copy to response buffer: from: %lu, to: %lu; %lu [%d])", from_pos, command->respbLen, len, copy_len);
            }*/
            len = uart_buf_copy_from(mpy_uarts[command->at_uart_num].uart_buf, from_pos, (uint8_t *)command->respbuff + command->respbLen, len);
            command->respbLen += len;
            command->respbuff[command->respbLen] = '\0';
        }
    }
}

/*
 * Main function for dealing with sending AT commands and parsing the response
 * - this function should be executed protected by mutex
 * - disables semaphore handling
 *
 * params:
 *   cmd        command or data to send, can be NULL
 *   cmdSize    length of the command or data
 *   responses  terminating string(s) to check for
 *   respbuff   buffer pointer, copy the response to it if not NULL
 *   respbSize  initial size of the response buffer
 *   timeout    time in ms to wait for the response
 * returns:
 *   0          timeout, no terminating string found
 *   n          found terminating string n
 *
 */
//-----------------------------------------
int _at_Cmd_Response(at_command_t *command)
{
    size_t buf_pos = 0;

    // === Send command or data to GSM/WiFi if requested ===
    if (command->cmd != NULL) {
        int wrlen = 0;
        int cmdsize = 0;
        if ((command->type_data) && (command->cmdSize > 0)) {
            // Write data
            cmdsize = command->cmdSize;
            if (command->dbg) {
                at_infoCommand("Send data", 9, "AT COMMAND:", command->responses);
                int data_len = (cmdsize > 80) ? 80 : cmdsize;
                char xdata[data_len+32];
                char *pxdata = xdata;
                for (int i=0; i<data_len; i++) {
                    if ((command->cmd[i] >= ' ') && (command->cmd[i] <= '~')) {
                        *pxdata = command->cmd[i];
                        pxdata++;
                    }
                    else {
                        sprintf(pxdata, "\\x%02X", command->cmd[i]);
                        pxdata += 4;
                    }
                    *pxdata = '\0';
                    if ((pxdata-xdata) > data_len+24) break;
                }
                LOGW(TAG, "data: [%s]", xdata);
            }
            buf_pos = uart_buf_length(mpy_uarts[command->at_uart_num].uart_buf, NULL);
            wrlen = uart_write(command->at_uart_num, (const uint8_t *)command->cmd, cmdsize);
        }
        else {
            // Write command
            cmdsize = (command->cmdSize < 0) ? strlen(command->cmd) : command->cmdSize;
            if (cmdsize > 0) {
                if (command->flush) {
                    // Delete all uart buffer content before sending the command
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    uart_buf_flush(mpy_uarts[command->at_uart_num].uart_buf);
                    buf_pos = 0;
                }

                if (command->dbg) at_infoCommand((char *)command->cmd, cmdsize, "AT COMMAND:", command->responses);
                buf_pos = uart_buf_length(mpy_uarts[command->at_uart_num].uart_buf, NULL);
                wrlen = uart_write(command->at_uart_num, (const uint8_t *)command->cmd, cmdsize);
            }
            else {
                // Only check responses
                if (command->dbg) at_infoCommand("None", 4, "AT COMMAND:", command->responses);
                buf_pos = uart_buf_length(mpy_uarts[command->at_uart_num].uart_buf, NULL);
            }
        }
        if (wrlen != cmdsize) {
            if (command->dbg) {
                LOGE(TAG, "Error sending command/data (%d <> %d)", wrlen, cmdsize);
            }
            return 0;
        }
    }
    else {
        if (command->dbg) {
            at_infoCommand("None", 4, "AT COMMAND:", command->responses);
        }
        buf_pos = uart_buf_length(mpy_uarts[command->at_uart_num].uart_buf, NULL);
    }

    // === Wait for and check the response for terminating string(s) ===
    command->result = 0;
    size_t len = 0;
    int res, pos;
    int max_resp_len = 0;
    int min_resp_len = 10000;
    size_t buf_len = 0, bufsize = 0;
    int resp_len;
    uint8_t n_matched = 0;
    int wait_end = mp_hal_ticks_ms() + command->timeout;

    for (int i=0; i<command->responses->nresp; i++) {
        if (command->responses->resp[i]) {
            if (strlen(command->responses->resp[i]) > max_resp_len) max_resp_len = strlen(command->responses->resp[i]);
            if (strlen(command->responses->resp[i]) < min_resp_len) min_resp_len = strlen(command->responses->resp[i]);
        }
    }
    res = 0;
    pos = -1;

    while(1) {
        mp_hal_wdt_reset();
        if (len > buf_len) buf_len = len;
        res = 0;
        vTaskDelay(2 / portTICK_PERIOD_MS);
        if (mp_hal_ticks_ms() > wait_end) {
            if (command->dbg) {
                LOGW(TAG, "AT COMMAND: TIMEOUT (%d ms, %lu)", command->timeout, buf_len);
            }
            break;
        }
        // get the current buffer length (from initial position)
        len = uart_buf_length(mpy_uarts[command->at_uart_num].uart_buf, &bufsize) - buf_pos;
        if (len < min_resp_len) continue; // not enough characters to check

        // Try to find the terminating string
        for (int i=0; i<command->responses->nresp; i++) {
            if (command->responses->resp[i]) {
                resp_len = strlen(command->responses->resp[i]);
                if (len >= resp_len) {
                    pos = uart_buf_find_from(mpy_uarts[command->at_uart_num].uart_buf, buf_pos,
                            bufsize, command->responses->resp[i], resp_len, NULL);
                    if (pos >= 0) {
                        res = i+1;
                        n_matched++;
                        command->result = res;
                        // ** terminating string found **
                        // if the response buffer is provided, move the received data,
                        // including the terminating string from uart buffer (if requested) to the response buffer
                        // else, just remove the data from uart buffer or mark it blank
                        if (command->respbuff != NULL) {
                            _copy_to_buf(command, buf_pos, pos - buf_pos + ((command->not_include_cond) ? 0 : resp_len));
                        }
                        if (command->flush) {
                            // Delete all uart buffer content up to and including the found response
                            uart_buf_remove(mpy_uarts[command->at_uart_num].uart_buf, pos + resp_len);
                        }
                        else {
                            // Blank the found response
                            uart_buf_blank(mpy_uarts[command->at_uart_num].uart_buf, pos, resp_len);
                        }
                        if (command->dbg) {
                            int wait_time = command->timeout - (wait_end - (int)mp_hal_ticks_ms());
                            size_t recv_bytes = (command->respbuff != NULL) ? strlen(command->respbuff) : 0;
                            LOGM(TAG, "AT RESPONSE: match condition %d at position %d (%d ms, %lu byte(s), buf_start=%lu)", res, pos, wait_time, recv_bytes, buf_pos);
                        }
                        break;
                    }
                }
            }
        }
        if (res > 0) {
            // === terminating string found ===
            if ((res == command->repeat) && (n_matched < command->responses->nresp)) continue; // match all responses requested
            break;
        }
    }

    if (res <= 0) {
        // === terminating string not found ===
        // if the response buffer is provided, move the received data from uart buffer to the response buffer
        // else, just remove the data from uart buffer (if requested)
        pos = uart_buf_length(mpy_uarts[command->at_uart_num].uart_buf, NULL);
        if (command->respbuff != NULL) _copy_to_buf(command, buf_pos, pos - buf_pos);
        if (command->flush) uart_buf_flush(mpy_uarts[command->at_uart_num].uart_buf);
        else uart_buf_blank(mpy_uarts[command->at_uart_num].uart_buf, buf_pos, pos-buf_pos);

        if ((command->dbg) && (command->respbuff != NULL)) {
            LOGM(TAG, "AT RESPONSE: no response (%lu) [%s]", strlen(command->respbuff), command->respbuff);
        }
    }

    return res;
}

//----------------------------------------
int at_Cmd_Response(at_command_t *command)
{
    // Disable semaphore handling
    mpy_uarts[command->at_uart_num].uart_buffer.notify = false;
    if (mpy_uarts[command->at_uart_num].task_semaphore) xSemaphoreTake(mpy_uarts[command->at_uart_num].task_semaphore, 0);

    int res = _at_Cmd_Response(command);

    size_t len = uart_buf_length(mpy_uarts[command->at_uart_num].uart_buf, NULL);
    mpy_uarts[command->at_uart_num].uart_buffer.notify = true;
    if ((mpy_uarts[command->at_uart_num].task_semaphore) && (len > 0))
        // force the main task to process the remaining bytes in uart buffer
        xSemaphoreGive(mpy_uarts[command->at_uart_num].task_semaphore);

    return res;
}

//---------------------------------------------------
int at_Commands(at_commands_t *commands, int *n_proc)
{
    // Disable semaphore handling
    mpy_uarts[commands->at_uart_num].uart_buffer.notify = false;
    if (mpy_uarts[commands->at_uart_num].task_semaphore) xSemaphoreTake(mpy_uarts[commands->at_uart_num].task_semaphore, 0);

    int res = 0, last_cmd = 1;
    for (int cmd_n=0; cmd_n<commands->ncmd; cmd_n++) {
        last_cmd = cmd_n + 1;
        res = _at_Cmd_Response(&commands->commands[cmd_n]);
        if (res == 0) break; // no expected response found
        if ((commands->expect_resp[cmd_n] > 0) && (res != commands->expect_resp[cmd_n])) break;
        if (commands->delay[cmd_n] > 0) vTaskDelay(commands->delay[cmd_n] / portTICK_RATE_MS);
    }

    size_t len = uart_buf_length(mpy_uarts[commands->at_uart_num].uart_buf, NULL);
    mpy_uarts[commands->at_uart_num].uart_buffer.notify = true;
    if ((mpy_uarts[commands->at_uart_num].task_semaphore) && (len > 0))
        // force the main task to process the remaining bytes in uart buffer
        xSemaphoreGive(mpy_uarts[commands->at_uart_num].task_semaphore);

    if (n_proc != NULL) *n_proc = last_cmd;
    return res;
}

//-------------------------------------------------------------------------
bool at_initCmd(const char *cmd, at_command_t *command, int tmo, int nfail)
{
    // command->response must be set
    command->cmd = cmd;
    command->timeout = tmo;
    bool res = false;
    while (nfail) {
        int cmd_res = at_Cmd_Response(command);
        if (cmd_res == 1) {
            res = true;
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        nfail--;
    }
    command->responses->nresp = 1;
    for (int i=1; i<AT_MAX_RESPONSES; i++) {
        command->responses->resp[i] = NULL;
    }
    return res;
}

//---------------------------------------------------
bool _disconnect(at_command_t *command, int uart_num)
{
    command->responses->nresp = 3;
    command->responses->resp[0] = AT_OK_Str;
    command->responses->resp[1] = "NO CARRIER";
    command->responses->resp[2] = "DISCONNECTED";
    command->timeout = 3000;
    command->cmd = "ATH\r\n";
    int res;
    int n = 2;
    while (n > 0) {
        vTaskDelay(1200 / portTICK_PERIOD_MS);
        at_uart_flush(uart_num);
        uart_write(uart_num, (const uint8_t *)"+++", 3);
        vTaskDelay(1200 / portTICK_PERIOD_MS);

        res = at_Cmd_Response(command);
        if (res != 0) return true;

        n--;
    }
    return false;
}

//---------------------------------------------------------------------------------------------------------
bool at_check_baudrate(bool debug, int *dev_baudrate, int uart_num, at_command_t *command, bool disconnect)
{
    if (debug) {
        LOGM(TAG, "Check and set device baudrate");
    }
    bool dbg = debug;
    debug = false;
    bool res;
    int baudrate = *dev_baudrate;

    memset(command->responses, 0, sizeof(at_responses_t));
    command->responses->nresp = 1;
    command->responses->resp[0] = AT_OK_Str;
    command->dbg = false;

    if (at_initCmd("AT\r\n", command, 100, MAX_INIT_TRIES)) {
        command->dbg = dbg;
        debug = dbg;
        if (debug) {
            LOGM(TAG, "Device communicating at %d bd", baudrate);
        }
        return true;
    }
    if (disconnect) {
        command->dbg = dbg;
        res = _disconnect(command, uart_num);
        if (res) {
            debug = dbg;
            if (debug) {
                LOGM(TAG, "Device communicating at %d bd", baudrate);
            }
            return true;
        }
        command->dbg = false;
        command->responses->nresp = 1;
        command->responses->resp[0] = AT_OK_Str;
    }

    // Try other baud rates
    for (int i=0; i<BDRATES_MAX; i++) {
        if (bd_rates[i] != baudrate) {
            if (dbg) {
                LOGW(TAG, "Trying at %d bd", bd_rates[i]);
            }
            mp_uart_config(uart_num, bd_rates[i], 8, UART_STOP_1, UART_PARITY_NONE);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            if (at_initCmd("AT\r\n", command, 100, MAX_INIT_TRIES)) {
                baudrate = bd_rates[i];
                command->dbg = dbg;
                debug = dbg;
                if (debug) {
                    LOGW(TAG, "Device baudrate is %d bd", baudrate);
                }
                *dev_baudrate = baudrate;
                return true;
            }
            if (disconnect) {
                res = _disconnect(command, uart_num);
                if (res) {
                    baudrate = bd_rates[i];
                    debug = dbg;
                    if (debug) {
                        LOGW(TAG, "Device baudrate is %d bd", baudrate);
                    }
                }
                command->responses->nresp = 1;
                command->responses->resp[0] = AT_OK_Str;
            }
        }
    }

    mp_uart_config(uart_num, baudrate, 8, UART_STOP_1, UART_PARITY_NONE);
    command->dbg = dbg;
    debug = dbg;
    if (debug) {
        LOGE(TAG, "Error setting baudrate");
    }
    return false;
}


//-------------------------------------
int at_get_socket(socket_obj_t *sock)
{
    for (int i=0; i<AT_MAX_SOCKETS; i++) {
        if (at_sockets[i] == NULL) {
            at_sockets[i] = sock;
            return i;
        }
    }
    errno = ENOTSOCK;
    return -1;
}

// ===============================================================
// ==== MicroPython AT command method common for WiFi and GSM ====
// ===============================================================

//---------------------------------------------------------------------
static void _get_responses(mp_obj_t resp_in, at_responses_t *responses)
{
    memset(responses, 0, sizeof(at_responses_t));
    // Get responses, string, tuple of strings or None is accepted
    if (mp_obj_is_str(resp_in)) {
        responses->nresp = 1;
        responses->resp[0] = (char *)mp_obj_str_get_str(resp_in);
    }
    else if (mp_obj_is_type(resp_in, &mp_type_tuple)) {
        // Multiple responses from tuple
        mp_obj_t *resp_items;
        size_t n_items = 0;

        mp_obj_get_array(resp_in, &n_items, &resp_items);
        if (n_items > 0) {
            for (int i=0; i<n_items; i++) {
                if ((mp_obj_is_str(resp_items[i])) && (responses->nresp < AT_MAX_RESPONSES)) {
                    responses->resp[i] = (char *)mp_obj_str_get_str(resp_items[i]);
                    responses->nresp++;
                }
            }
        }
    }
    else {
        // set default response, "OK" or "ERROR"
        responses->nresp = 2;
        responses->resp[0] = AT_OK_Str;
        responses->resp[1] = AT_Error_Str;
    }
}

//--------------------------------------------------------------------------------------------------------------------
mp_obj_t mpy_atCmd(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args, int uart, bool dbg, const char *tag)
{
    enum { ARG_cmd, ARG_timeout, ARG_response, ARG_delay, ARG_incresp };
    const mp_arg_t allowed_args[] = {
            { MP_QSTR_cmd,      MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_timeout,                    MP_ARG_INT,  {.u_int = 500} },
            { MP_QSTR_response,                   MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_delay,                      MP_ARG_INT,  {.u_int = 5} },
            { MP_QSTR_incresp,                    MP_ARG_BOOL, {.u_bool = true} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int res;
    at_responses_t at_responses = { 0 };
    at_command_t at_command = { 0 };
    mp_obj_t response = mp_const_false;

    if (mp_obj_is_str(args[ARG_cmd].u_obj)) {
        // === Command is string or bytes ===
        // Get responses, string, tuple of strings or None is accepted
        _get_responses(args[ARG_response].u_obj, &at_responses);

        memset(&at_command, 0, sizeof(at_command_t));
        at_command.respbuff = pvPortMalloc(256);
        if (at_command.respbuff == NULL) {
            mp_raise_msg(&mp_type_OSError, "Response buffer allocation error");
        }
        memset(at_command.respbuff, 0, 256);
        at_command.responses = &at_responses;
        at_command.timeout = args[ARG_timeout].u_int;
        at_command.at_uart_num = uart;
        at_command.dbg = dbg;
        at_command.respbSize = 256;
        at_command.respbLen = 0;
        at_command.not_include_cond = !args[ARG_incresp].u_bool;
        //at_command.flush = true;

        if (dbg) {
            LOGM(tag, "atcmd: string command");
        }

        char *cmd = (char *)mp_obj_str_get_str(args[ARG_cmd].u_obj);
        char atcmd[strlen(cmd)+4];
        if (strlen(cmd) >= 2) {
            if ((strstr(cmd, "AT") != cmd) && (strstr(cmd, "at") != cmd)) {
                mp_raise_ValueError("Command must start with 'AT' or 'at'");
            }
            strcpy(atcmd, cmd);
            if ((cmd[strlen(cmd)-2] != '\r') || (cmd[strlen(cmd)-1] != '\n')) strcat(atcmd, "\r\n");

            at_command.cmdSize = strlen(atcmd);
            at_command.cmd = atcmd;
        }
        else {
            at_command.cmdSize = 0;
            at_command.cmd = "";
        }

        // Execute command
        //------------------------------------------------------
        if (strstr(tag, "WIFI")) res = wifi_at_Cmd(&at_command);
        else res = gsm_at_Cmd(&at_command);
        //------------------------------------------------------

        mp_obj_t tuple[2];
        tuple[0] = mp_obj_new_int(res);
        tuple[1] = mp_obj_new_str(at_command.respbuff, strlen(at_command.respbuff));

        if (at_command.respbuff) vPortFree(at_command.respbuff);

        response = mp_obj_new_tuple(2, tuple);
        return response;
    }
    else if (mp_obj_is_type(args[ARG_cmd].u_obj, &mp_type_tuple)) {
        if (dbg) {
            LOGM(tag, "atcmd: multiple commands");
        }
        // === Multiple commands, each should be a tuple with the following items:
        // (command_data, is_data, timeout, response, repeat, incresp)
        // the 'response' item can be a string or tuple of strings or 'None'
        mp_obj_t *commands_items;
        size_t commands_n_items = 0;

        mp_obj_get_array(args[ARG_cmd].u_obj, &commands_n_items, &commands_items);
        if (commands_n_items > 0) {
            if (commands_n_items > AT_MAX_COMMANDS) commands_n_items = AT_MAX_COMMANDS;

            at_responses_t at_resp[AT_MAX_COMMANDS];
            memset(&at_resp, 0, sizeof(at_resp));

            at_commands_t at_commands;
            memset(&at_commands, 0, sizeof(at_commands_t));

            // initialize structures
            at_commands.at_uart_num = uart;
            at_commands.dbg = dbg;
            for (int i=0; i<commands_n_items; i++) {
                memset(&at_commands.commands[i], 0, sizeof(at_command_t));
                at_commands.commands[i].respbuff = pvPortMalloc(256);
                if (at_commands.commands[i].respbuff != NULL) {
                    memset(at_commands.commands[i].respbuff, 0, 256);
                    at_commands.commands[i].respbSize = 256;
                }
                at_commands.commands[i].respbLen = 0;
                at_commands.commands[i].cmdSize = -1;
                at_commands.commands[i].at_uart_num = uart;
                at_commands.commands[i].dbg = dbg;
                //at_commands.commands[i].flush = true;
            }

            // Iterate all commands
            for (int i=0; i<commands_n_items; i++) {
                if ((mp_obj_is_type(commands_items[i], &mp_type_tuple)) && (at_commands.ncmd < AT_MAX_COMMANDS)) {
                    mp_obj_t *cmd_items;
                    size_t cmd_n_items = 0;
                    mp_obj_get_array(commands_items[i], &cmd_n_items, &cmd_items);
                    if (cmd_n_items == 6) {
                        bool added = false;
                        for (int j=0; j<cmd_n_items; j++) {
                            added = false;
                            if (mp_obj_is_str(cmd_items[0])) {
                                at_commands.delay[i] = args[ARG_delay].u_int;
                                at_commands.commands[i].cmd = (char *)mp_obj_str_get_str(cmd_items[0]);
                                if (mp_obj_is_int(cmd_items[2])) at_commands.commands[i].timeout = mp_obj_get_int(cmd_items[2]);
                                else at_commands.commands[i].timeout = 500;
                                at_commands.commands[i].type_data = mp_obj_is_true(cmd_items[1]);
                                at_commands.commands[i].cmdSize = strlen(at_commands.commands[i].cmd);
                                _get_responses(cmd_items[3], &at_resp[i]);
                                at_commands.commands[i].responses = &at_resp[i];
                                at_commands.commands[i].repeat = mp_obj_get_int(cmd_items[4]);
                                at_commands.commands[i].not_include_cond = !mp_obj_is_true(cmd_items[5]);
                                added = true;
                            }
                        }
                        if (added) at_commands.ncmd++;
                    }
                }
            }
            if (at_commands.ncmd > 0) {
                if (dbg) {
                    LOGM(tag, "atcmd: Executing %d command(s)", at_commands.ncmd);
                }
                // Execute multiple at commands
                //------------------------------------------------------------------
                if (strstr(tag, "WIFI")) res = wifi_at_Commands(&at_commands, NULL);
                else res = gsm_at_Commands(&at_commands, NULL);
                //------------------------------------------------------------------

                if (res > 0) {
                    mp_obj_t res_tuple[at_commands.ncmd];
                    mp_obj_t tuple[2];

                    for (int i=0; i<at_commands.ncmd; i++) {
                        tuple[0] = mp_obj_new_int(at_commands.commands[i].result);
                        if (at_commands.commands[i].respbuff) {
                            tuple[1] = mp_obj_new_str(at_commands.commands[i].respbuff, strlen(at_commands.commands[i].respbuff));
                            vPortFree(at_commands.commands[i].respbuff);
                        }
                        else tuple[1] = mp_const_none;
                        res_tuple[i] = mp_obj_new_tuple(2, tuple);
                    }
                    response = mp_obj_new_tuple(at_commands.ncmd, res_tuple);
                }
            }
            // Free any remaining buffers
            for (int i=0; i<commands_n_items; i++) {
                if (at_commands.commands[i].respbuff != NULL) {
                    vPortFree(at_commands.commands[i].respbuff);
                    at_commands.commands[i].respbuff = NULL;
                }
            }
            return response;
        }
        return response;
    }

    mp_raise_ValueError("Wrong 'cmd' argument type");
    return mp_const_none;
}

