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

#if MICROPY_PY_USE_GSM

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "syslog.h"
#include "devices.h"
#include "lwip/apps/sntp.h"

#include "network.h"

#include "modmachine.h"
#include "machine_uart.h"
#include "libGSM.h"
#include "py/runtime.h"
#include "mphalport.h"

#define GSM_TASK_PRIORITY   13

#define BUF_SIZE (2048)

#define GSM_MAX_INIT_TRIES      4
#define UART_WAIT_AFTER_SEND    (1 / portTICK_RATE_MS)

extern bool tcpip_adapter_initialized;

// shared variables, use mutex to access them
static bool full_connect_init = false;
static int gsm_pppos_task_started = 0;

static void *new_SMS_cb = NULL;
static uint32_t SMS_check_interval = 0;
static uint8_t doCheckSMS = 1;

// local variables
static TaskHandle_t gsmPPPoSTaskHandle = NULL;
static char GSM_APN[PPP_MAX_NAME_LEN+1] = {0};
static int gsm_pin_tx = UART_PIN_NO_CHANGE;
static int gsm_pin_rx = UART_PIN_NO_CHANGE;
static int gsm_pin_cts = UART_PIN_NO_CHANGE;
static int gsm_pin_rts = UART_PIN_NO_CHANGE;
static uint64_t sms_timer = 0;
static bool allow_roaming = false;
static uint32_t pppos_rx_count = 0;
static uint32_t pppos_tx_count = 0;

int gsm_uart_baudrate = 115200;
bool gsm_debug = false;
int gsm_uart_num = -1;
at_responses_t gsm_at_responses = { 0 };
at_command_t gsm_at_command = { 0 };

static int do_pppos_connect = 1;
// The PPP control block
static ppp_pcb *ppp_ppp_pcb = NULL;
// The PPP IP interface
static struct netif ppp_netif;
static char PPP_User[PPP_MAX_NAME_LEN+1] = {0};
static char PPP_Pass[PPP_MAX_NAME_LEN+1] = {0};
static uint8_t ppp_status = ATDEV_STATEFIRSTINIT;
static uint32_t gsm_ppp_ip = 0;
static uint32_t gsm_ppp_netmask = 0;
static uint32_t gsm_ppp_gw = 0;
static time_t ntp_got_time = 0;

static char gsm_connect_string[32] = "ATDT*99***1#\r\n";  // "AT+CGDATA=\"PPP\",1\r\n"
const char *GSM_PPP_TAG = "[GSM_PPPOS]";

typedef struct
{
	char		*cmd;
	int16_t		cmdSize;
	char		*cmdResponseOnOk;
	uint16_t	timeoutMs;
	uint16_t	delayMs;
	uint8_t		skip;
}GSM_Cmd;

static GSM_Cmd cmd_AT =
{
	.cmd = "AT\r\n",
	.cmdSize = -1,
	.cmdResponseOnOk = AT_OK_Str,
	.timeoutMs = 500,
	.delayMs = 0,
	.skip = 0,
};

static GSM_Cmd cmd_NoSMSInd =
{
	.cmd = "AT+CNMI=0,0,0,0,0\r\n",
	.cmdSize = -1,
	.cmdResponseOnOk = AT_OK_Str,
	.timeoutMs = 1200,
	.delayMs = 0,
	.skip = 0,
};

static GSM_Cmd cmd_Reset =
{
	.cmd = "ATZ\r\nATE0\r\n",
	.cmdSize = -1,
	.cmdResponseOnOk = AT_OK_Str,
	.timeoutMs = 1200,
	.delayMs = 0,
	.skip = 0,
};

// Some GSM modules (like SIM5360E) need up to 10 seconds
// to initialize after "AT+CFUN=1"
static GSM_Cmd cmd_RFOn =
{
	.cmd = "AT+CFUN=1\r\n",
	.cmdSize = -1,
	.cmdResponseOnOk = AT_OK_Str,
	.timeoutMs = 20000,
	.delayMs = 1200,
	.skip = 0,
};

static GSM_Cmd cmd_EchoOff =
{
	.cmd = "ATE0\r\n",
	.cmdSize = -1,
	.cmdResponseOnOk = AT_OK_Str,
	.timeoutMs = 500,
	.delayMs = 0,
	.skip = 0,
};

static GSM_Cmd cmd_SMSmode =
{
    .cmd = "AT+CMGF=1\r\n",
    .cmdSize = -1,
    .cmdResponseOnOk = AT_OK_Str,
    .timeoutMs = 500,
    .delayMs = 0,
    .skip = 0,
};

static GSM_Cmd cmd_Pin =
{
	.cmd = "AT+CPIN?\r\n",
	.cmdSize = -1,
	.cmdResponseOnOk = "CPIN: READY",
	.timeoutMs = 5000,
	.delayMs = 0,
	.skip = 0,
};

static GSM_Cmd cmd_Reg =
{
	.cmd = "AT+CREG?\r\n",
	.cmdSize = -1,
	.cmdResponseOnOk = "CREG: 0,1",
	.timeoutMs = 3000,
	.delayMs = 2000,
	.skip = 0,
};

static GSM_Cmd cmd_APN =
{
	.cmd = NULL,
	.cmdSize = 0,
	.cmdResponseOnOk = AT_OK_Str,
	.timeoutMs = 1000,
	.delayMs = 0,
	.skip = 0,
};

static GSM_Cmd cmd_Connect =
{
    .cmd = gsm_connect_string,
	.cmdSize = -1,
	.cmdResponseOnOk = "CONNECT",
	.timeoutMs = 30000,
	.delayMs = 1000,
	.skip = 0,
};

static GSM_Cmd *GSM_Init[] =
{
		&cmd_AT,
		&cmd_Reset,
		&cmd_EchoOff,
		&cmd_RFOn,
		&cmd_SMSmode,
		&cmd_Pin,
		&cmd_Reg,
		&cmd_NoSMSInd,
		&cmd_APN,
		&cmd_Connect,
};

#define GSM_InitCmdsSize  (sizeof(GSM_Init)/sizeof(GSM_Cmd *))

//extern handle_t mp_rtc_rtc0;

static void *ntp_time_cb = NULL;

//==============================================
// Function used by SNTP to synchronize the time
//==============================================
void set_rtc_time_from_seconds(time_t seconds)
{
    struct tm *tm_info;
    tm_info = gmtime(&seconds);
    //rtc_set_datetime(mp_rtc_rtc0, tm_info); // set RTC time
    _set_sys_time(tm_info, 0);
    ntp_got_time = seconds;
}
//==============================================

// === Handle receiving data from GSM modem (requested by PPPoS) ===
//--------------------------------------------------------------------------
uint32_t ppp_output_callback(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
    LWIP_UNUSED_ARG(ctx);
    LWIP_UNUSED_ARG(pcb);
    uint32_t ret = uart_write(gsm_uart_num, (const uint8_t *)data, len);
    if (ret > 0) {
        //if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) return ret;
        pppos_rx_count += ret;
        //xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
    }
    return ret;
}
//--------------------------------------------------------------------------

// === PPPoS status callback (requested by PPPoS) ===
//--------------------------------------------------------------
static void ppp_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
    struct netif *pppif = ppp_netif(pcb);
    LWIP_UNUSED_ARG(ctx);
    int taken;

    if (gsm_debug) {
        LOGM(GSM_PPP_TAG,"status_cb: Code %d", err_code);
    }
    switch(err_code) {
        case PPPERR_NONE: {
            // No error, PPP is connected
            if (gsm_debug) {
                LOGM(GSM_PPP_TAG,"status_cb: Error=None, Connected");
            }
            taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
            int gstat = ppp_status;
            if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
            if (gstat != ATDEV_STATECONNECTED) {
                if (gsm_debug) {
                    LOGM(GSM_PPP_TAG,"status_cb: Connected");
                    #if PPP_IPV4_SUPPORT
                    LOGM(GSM_PPP_TAG,"   ipaddr    = %s", ipaddr_ntoa(&pppif->ip_addr));
                    LOGM(GSM_PPP_TAG,"   gateway   = %s", ipaddr_ntoa(&pppif->gw));
                    LOGM(GSM_PPP_TAG,"   netmask   = %s", ipaddr_ntoa(&pppif->netmask));
                    #endif

                    #if PPP_IPV6_SUPPORT
                    LOGM(GSM_PPP_TAG,"   ip6addr   = %s", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
                    #endif
                }
                taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
                gsm_ppp_ip = pppif->ip_addr.addr;
                gsm_ppp_netmask = pppif->netmask.addr;
                gsm_ppp_gw = pppif->gw.addr;
                ppp_status = ATDEV_STATECONNECTED;
                net_active_interfaces &= ~ACTIVE_INTERFACE_GSM;
                net_active_interfaces |= ACTIVE_INTERFACE_LWIP;
                if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

                //synchronize time from NTP server
                if (sntp_enabled() == 0) {
                    sntp_setoperatingmode(SNTP_OPMODE_POLL);
                    sntp_setservername(0, DEFAULT_SNTP_SERVER);
                    sntp_init();
                    if (gsm_debug) {
                        LOGM(GSM_PPP_TAG, "NTP service started");
                    }
                }
                else {
                    sntp_stop();
                    sntp_setoperatingmode(SNTP_OPMODE_POLL);
                    sntp_setservername(0, DEFAULT_SNTP_SERVER);
                    sntp_init();
                    if (gsm_debug) {
                        LOGM(GSM_PPP_TAG, "NTP service restarted");
                    }
                }
            }
            else {
                // The 2nd connect message, if it occurs while already in connected state
                // it usually means something wrong happened, like modem reset,
                // so we request the disconnect
                if (gsm_debug) {
                    LOGM(GSM_PPP_TAG,"status_cb: Connected (2nd)");
                }
                if (sntp_enabled() != 0) {
                    sntp_stop();
                }
                taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
                do_pppos_connect = 0;
                if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
            }
            break;
        }
        case PPPERR_PARAM: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Invalid parameter");
            }
            break;
        }
        case PPPERR_OPEN: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Unable to open PPP session");
            }
            break;
        }
        case PPPERR_DEVICE: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Invalid I/O device for PPP");
            }
            break;
        }
        case PPPERR_ALLOC: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Unable to allocate resources");
            }
            break;
        }
        case PPPERR_USER: {
            /* ppp_free(); -- can be called here */
            if (gsm_debug) {
                LOGW(GSM_PPP_TAG,"status_cb: User interrupt (disconnected)");
            }
            if (sntp_enabled() != 0) {
                sntp_stop();
            }
            //pppapi_free(ppp_ppp_pcb);
            //ppp_ppp_pcb = NULL;
            taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
            gsm_ppp_ip = 0;
            gsm_ppp_netmask = 0;
            gsm_ppp_gw = 0;
            ppp_status = ATDEV_STATEDISCONNECTED;
            net_active_interfaces &= ~ACTIVE_INTERFACE_LWIP;
            if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
            break;
        }
        case PPPERR_CONNECT: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Connection lost");
            }
            if (sntp_enabled() != 0) {
                sntp_stop();
            }
            taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
            gsm_ppp_ip = 0;
            gsm_ppp_netmask = 0;
            gsm_ppp_gw = 0;
            ppp_status = ATDEV_STATEDISCONNECTED;
            net_active_interfaces &= ~ACTIVE_INTERFACE_LWIP;
            if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
            break;
        }
        case PPPERR_AUTHFAIL: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Failed authentication challenge");
            }
            break;
        }
        case PPPERR_PROTOCOL: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Failed to meet protocol");
            }
            break;
        }
        case PPPERR_PEERDEAD: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Connection timeout");
            }
            break;
        }
        case PPPERR_IDLETIMEOUT: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Idle Timeout");
            }
            break;
        }
        case PPPERR_CONNECTTIME: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Max connect time reached");
            }
            break;
        }
        case PPPERR_LOOPBACK: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Loopback detected");
            }
            break;
        }
        default: {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG,"status_cb: Unknown error code %d", err_code);
            }
            break;
        }
    }
}
//--------------------------------------------------------------

//-------------------------------------------
static int _cfun_state(at_command_t *command)
{
    command->responses->nresp = 2;
    command->responses->resp[0] = "+CFUN: 1";
    command->responses->resp[1] = "+CFUN: 4";
    command->timeout = 100;
    command->cmd = "AT+CFUN?\r\n";
    return at_Cmd_Response(command);
}

//-------------------------------------------------------
static bool _do_rfOnOff(at_command_t *command, bool rfOn)
{
    bool ret = true;
    int res = _cfun_state(command);
    if (res == 0) return false;

    if (((res == 1) && (!rfOn)) || ((res == 2) && (rfOn))) {
        // RF state must be changed
        if (gsm_debug) {
            LOGW(GSM_PPP_TAG,"Set RF %s", (rfOn) ? "on" : "off");
        }
        command->responses->nresp = 2;
        command->responses->resp[0] = AT_OK_Str;
        command->responses->resp[1] = AT_Error_Str;
        command->timeout = 20000;
        sprintf((char *)command->cmd, "AT+CFUN=%d\r\n", (rfOn) ? 1 : 4);
        cmd_Reg.timeoutMs = 10000;
        res = at_Cmd_Response(command);
        ret = (res == 1);
    }
    return ret;
}

//----------------------------
static void enableAllInitCmd()
{
	for (int idx = 0; idx < GSM_InitCmdsSize; idx++) {
		GSM_Init[idx]->skip = 0;
	}
}

int checkMessages(uint8_t rd_status, int sms_idx, SMS_Msg *msg, SMS_indexes *indexes, uint8_t sort);

// used only inside GSM task
//--------------------
static void checkSMS()
{
	if ((new_SMS_cb) && (SMS_check_interval > 0) && (doCheckSMS)) {
		// Check for new SMS and schedule MicroPython callback function
		if (mp_hal_ticks_ms() > sms_timer) {
	        bool dbg = gsm_debug;
	        gsm_debug = false;
			sms_timer = mp_hal_ticks_ms() + SMS_check_interval;
			SMS_indexes indexes;
			int nmsg = checkMessages(SMS_LIST_NEW, 0, NULL, &indexes, SMS_SORT_ASC);
			if (nmsg > 0) {
				if (nmsg > 100) nmsg = 100;
				// Create a tuple containing SMS indexes
			    mp_obj_t tuple[nmsg];
                for (int i=0; i<nmsg; i++) {
                    tuple[i] = mp_obj_new_int(indexes.idx[i]);
                }
                mp_sched_schedule((mp_obj_t)new_SMS_cb, mp_obj_new_tuple(nmsg, tuple));
			}
	        gsm_debug = dbg;
		}
	}
}

//---------------------
static int _task_idle()
{
    if (gsm_debug) {
        LOGM(GSM_PPP_TAG, "PPPoS IDLE mode");
    }
    int gstat = 0;
    net_active_interfaces |= ACTIVE_INTERFACE_GSM;
    while (gstat == 0) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) {
            //if (gsm_debug) {
            //    LOGW(GSM_PPP_TAG, "Error taking mutex (2)");
            //}
            continue;
        }

        gstat = do_pppos_connect;
        xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
        checkSMS();
    }
    net_active_interfaces &= ~ACTIVE_INTERFACE_GSM;
    return gstat;
}

//----------------------------------------
static void _handle_pppos_data(char *data)
{
    // --- Handle data received from GSM (to be passed to the PPPoS) ---
    bool do_check = false;
    if (mpy_uarts[gsm_uart_num].task_semaphore) {
        do_check = (xSemaphoreTake(mpy_uarts[gsm_uart_num].task_semaphore, 5 / portTICK_PERIOD_MS ) == pdTRUE);
    }
    else {
        vTaskDelay(5 / portTICK_PERIOD_MS);
        do_check = true;
    }
    if (ntp_got_time > 0) {
        if (gsm_debug) LOGM(GSM_PPP_TAG, "NTP time synchronized (%lu)", ntp_got_time);
        if (ntp_time_cb) mp_sched_schedule((mp_obj_t)ntp_time_cb, mp_obj_new_int(ntp_got_time));
        ntp_got_time = 0;
        if (sntp_enabled()) {
            sntp_stop();
        }
    }

    if (do_check) {
        //if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) == pdTRUE) {
        int len;
        while (1) {
            len = uart_buf_length(mpy_uarts[gsm_uart_num].uart_buf, NULL);
            if (len == 0) break;
            uart_buf_get(mpy_uarts[gsm_uart_num].uart_buf, (uint8_t *)data, len);
            // pass received data to PPPoS
            pppos_input_tcpip(ppp_ppp_pcb, (u8_t*)data, len);
            pppos_tx_count += len;
        }
        //    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
        //}
    }
}

/*
 * PPPoS TASK
 * Handles GSM initialization, disconnects and GSM modem responses
 *
 * GSM module must be already powered on and ready for communication
 */
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static void gsm_pppos_client_task(void *pv)
{
    // if the task uses some MicroPython functions, we have to save
    // MicroPython state in local storage pointers
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)pv, THREAD_LSP_STATE));
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, pvTaskGetThreadLocalStoragePointer((TaskHandle_t)pv, THREAD_LSP_ARGS));

    gsm_pppos_task_started = -1;
    gsm_ppp_ip = 0;
    gsm_ppp_netmask = 0;
    gsm_ppp_gw = 0;
    char* data = NULL;
	char PPP_ApnATReq[strlen(GSM_APN)+24];
	int mutex_taken = -1;
    net_active_interfaces &= ~ACTIVE_INTERFACE_GSM;
    net_active_interfaces &= ~ACTIVE_INTERFACE_LWIP;

    // === Initialize GSM UART first ===================================================
    if (gsm_debug) {
        LOGM(GSM_PPP_TAG, "Initialize UART");
    }
    // Find free uart
    gsm_uart_num = -1;
    for (int i=0; i < UART_NUM_MAX; i++) {
        if (mpy_uarts[i].handle == 0) {
            gsm_uart_num = i;
            break;
        }
    }
    if (gsm_uart_num < 0) {
        LOGE(GSM_PPP_TAG, "No free uart available");
        // Terminate task
        gsm_pppos_task_started = 0;
        vTaskDelete(NULL);
    }
    memset(&mpy_uarts[gsm_uart_num], 0, sizeof(uart_uarts_t));

    if (gsm_debug) {
        LOGM(GSM_PPP_TAG,"UART #%d, initialize uart hardware", gsm_uart_num);
    }
    int res = uart_hard_init(gsm_uart_num, gsm_pin_tx, gsm_pin_rx, GPIO_FUNC_GSM_UART, true, true, BUF_SIZE*2);
    if (res < 0) {
        LOGE(GSM_PPP_TAG, "Error initializing UART hardware (%d)", res);
        // Terminate task
        gsm_pppos_task_started = 0;
        vTaskDelete(NULL);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint32_t bdr = mp_uart_config(gsm_uart_num, gsm_uart_baudrate, 8, UART_STOP_1, UART_PARITY_NONE);

    if (gsm_debug) {
        LOGM(GSM_PPP_TAG,"UART #%d initialized: tx=%d, rx=%d, bdr=%d", gsm_uart_num, gsm_pin_tx, gsm_pin_rx, bdr);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
    // =================================================================================

    data = pvPortMalloc(BUF_SIZE+1);
    if (data == NULL) {
        LOGE(GSM_PPP_TAG,"Failed to allocate data buffer.");
        goto exit;
    }

    // Take the mutex during initialization
    mutex_taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
    if (mutex_taken != pdTRUE) {
        goto exit;
    }

    if (gsm_debug) {
        LOGM(GSM_PPP_TAG, "GSM TASK STARTED");
    }
    gsm_pppos_task_started = 1;

    at_responses_t responses;
    memset(&responses, 0, sizeof(at_responses_t));

    at_command_t command;
    memset(&command, 0, sizeof(at_command_t));
    command.cmd = NULL;
    command.cmdSize = -1;
    command.dbg = gsm_debug;
    command.responses = &responses;
    command.at_uart_num = gsm_uart_num;
    command.flush = true;

    // === Check baud rate and change if necessary ===
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (!at_check_baudrate(gsm_debug, &gsm_uart_baudrate, gsm_uart_num, &command, false)) {
        if (!at_check_baudrate(gsm_debug, &gsm_uart_baudrate, gsm_uart_num, &command, true)) goto exit;
    }

    if (gsm_debug) {
        LOGM(GSM_PPP_TAG, "Set APN");
    }
	// === Set APN from config ===
	sprintf(PPP_ApnATReq, "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", GSM_APN);
	cmd_APN.cmd = PPP_ApnATReq;
	cmd_APN.cmdSize = strlen(PPP_ApnATReq);

    pppos_tx_count = 0;
    pppos_rx_count = 0;
	ppp_status = ATDEV_STATEFIRSTINIT;

	enableAllInitCmd();

	// wait for 2 seconds silence on uart Rx
    if (gsm_debug) {
        LOGM(GSM_PPP_TAG, "Wait for no data from GSM module");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    at_uart_flush(gsm_uart_num);
    while (1) {
        if (uart_buf_length(mpy_uarts[gsm_uart_num].uart_buf, NULL) == 0) break;
        if (gsm_debug) {
            LOGM(GSM_PPP_TAG, "[%s]", data);
        }
        at_uart_flush(gsm_uart_num);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    memset(data, 0, BUF_SIZE);

    // === Main task's loop ===
	while(1)
	{
	    if (mutex_taken != pdTRUE) mutex_taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
        if (mutex_taken != pdTRUE) {
            if (gsm_debug) {
                LOGE(GSM_PPP_TAG, "Error taking mutex (1)");
            }
            break; // error, end task!
        }
		if (gsm_debug) {
			LOGM(GSM_PPP_TAG,"GSM initialization start");
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);

		if (do_pppos_connect <= 0) {
			cmd_Connect.skip = 1;
			//cmd_APN.skip = 1;
		}
		int gsmCmdIter = 0;
		int nfail = 0;
		int cmd_res = 0;

		// ===== GSM Initialization loop =========================================================================
		while(gsmCmdIter < GSM_InitCmdsSize)
		{
            if ((GSM_Init[gsmCmdIter] == &cmd_RFOn) && (allow_roaming)) {
                // Skip RFOn command if already ON
                GSM_Init[gsmCmdIter]->skip = (_cfun_state(&command) == 1);
            }
			if (GSM_Init[gsmCmdIter]->skip) {
			    // Skip this command
				if (gsm_debug) {
					at_infoCommand(GSM_Init[gsmCmdIter]->cmd, GSM_Init[gsmCmdIter]->cmdSize, "SKIP COMMAND:", NULL);
				}
				gsmCmdIter++;
				continue;
			}
			// Prepare command
		    command.responses->nresp = 1;
		    command.responses->resp[0] = GSM_Init[gsmCmdIter]->cmdResponseOnOk;
		    command.responses->resp[1] = NULL;
		    command.timeout = GSM_Init[gsmCmdIter]->timeoutMs;
		    command.cmdSize = GSM_Init[gsmCmdIter]->cmdSize;
		    command.cmd = GSM_Init[gsmCmdIter]->cmd;

		    if ((GSM_Init[gsmCmdIter] == &cmd_Reg) && (allow_roaming)) {
	            command.responses->nresp = 2;
	            command.responses->resp[1] = "CREG: 0,5";
			}
		    // Execute command
		    cmd_res = at_Cmd_Response(&command);

            if (cmd_res == 0) {
				// * No response or not as expected, start from first initialization command
	            if (GSM_Init[gsmCmdIter] == &cmd_Connect) {
	                // Try to connect only once
	                do_pppos_connect = 0;
	                cmd_Connect.skip = 1;
	                //cmd_APN.skip = 1;
	                gsmCmdIter++;
	                continue;
	            }

	            if (gsm_debug) {
					LOGW(GSM_PPP_TAG,"Wrong response, restarting... (%d/%d)", nfail+1, GSM_MAX_INIT_TRIES);
				}

				if (++nfail >= GSM_MAX_INIT_TRIES) goto exit;

				vTaskDelay(3000 / portTICK_PERIOD_MS);
				gsmCmdIter = 0;
				continue;
			}
            // Response as expected
            if ((GSM_Init[gsmCmdIter] == &cmd_Reg) && (allow_roaming)) {
                if (cmd_res == 2) {
                    if (gsm_debug) {
                        LOGW(GSM_PPP_TAG,"Connected in roaming");
                    }
                }
            }

            if (GSM_Init[gsmCmdIter]->delayMs > 0) {
                // Delay after command requested
                vTaskDelay(GSM_Init[gsmCmdIter]->delayMs / portTICK_PERIOD_MS);
            }
            // Command OK, skip it if requested again
			GSM_Init[gsmCmdIter]->skip = 1;
			if (GSM_Init[gsmCmdIter] == &cmd_Reg) GSM_Init[gsmCmdIter]->delayMs = 0;

			//if (GSM_Init[gsmCmdIter] == &cmd_Connect) at_uart_flush(gsm_uart_num);

			// === Next command ===
			gsmCmdIter++;
		}
		// =======================================================================================================

		// === GSM is now initialized ===
        if (ppp_status == ATDEV_STATEFIRSTINIT) {
			// === After first successful initialization create PPP control block ===
			ppp_ppp_pcb = pppapi_pppos_create(&ppp_netif, ppp_output_callback, ppp_status_cb, NULL);

			if (ppp_ppp_pcb == NULL) {
				if (gsm_debug) {
					LOGE(GSM_PPP_TAG, "Error initializing PPPoS");
				}
				break; // error, end task!
			}
			if (gsm_debug) {
				LOGM(GSM_PPP_TAG, "PPPoS control block created");
			}
		}

        // Set the current status
        if (do_pppos_connect <= 0) ppp_status = ATDEV_STATEIDLE;
        else ppp_status = ATDEV_STATECONNECTING;

        xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
        mutex_taken = 0;

        if (gsm_debug) {
            if (do_pppos_connect <= 0) {
                LOGM(GSM_PPP_TAG,"GSM initialized and ready.");
            }
            else {
                LOGM(GSM_PPP_TAG,"GSM initialized, waiting for connection...");
            }
        }

		int gstat = 0;
		if (do_pppos_connect <= 0) {
			/*
			 * === Connection to the Internet was not requested, stay in idle mode ===
			 * In IDLE mode requests from main task are processed and SMS checked
			 */
			if (gsm_debug) {
				LOGM(GSM_PPP_TAG, "PPPoS IDLE mode");
			}

			//-------------------
			gstat = _task_idle();
            //-------------------

			// Main task request received
			if (gstat < 0) break;  // terminate task requested

			// Connect requested, prepare commands
			gsmCmdIter = 0;
			if (full_connect_init) enableAllInitCmd();
			cmd_Connect.skip = 0;
			cmd_APN.skip = 0;
			do_pppos_connect = 1;
			if (gsm_debug) {
				LOGM(GSM_PPP_TAG, "Connect requested.");
			}
			continue; // Main task's loop
		}

        /*
         * === Connection to the Internet was requested =================
         */
        net_active_interfaces &= ~ACTIVE_INTERFACE_GSM;
        net_active_interfaces &= ~ACTIVE_INTERFACE_LWIP;
        if (gsm_debug) {
            if (mpy_uarts[gsm_uart_num].task_semaphore) {
                LOGM(GSM_PPP_TAG, "Using uart semaphore (%d)", mpy_uarts[gsm_uart_num].uart_buffer.notify);
                mpy_uarts[gsm_uart_num].uart_buffer.notify = true;
            }
            LOGM(GSM_PPP_TAG, "Configure PPPoS");
        }
        // Prepare and start PPPoS
		res = pppapi_set_default(ppp_ppp_pcb);
		if (res == 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
		    //if ((strlen(PPP_User) == 0) && (strlen(PPP_Pass) == 0)) ppp_set_auth(ppp_ppp_pcb, PPPAUTHTYPE_NONE, PPP_User, PPP_Pass);
		    //else ppp_set_auth(ppp_ppp_pcb, PPPAUTHTYPE_PAP, PPP_User, PPP_Pass);
		    ppp_set_auth(ppp_ppp_pcb, PPPAUTHTYPE_PAP, PPP_User, PPP_Pass);
            vTaskDelay(100 / portTICK_PERIOD_MS);

            res = pppapi_connect(ppp_ppp_pcb, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		if (res != 0) {
		    do_pppos_connect = 0;
	        if (gsm_debug) {
	            LOGW(GSM_PPP_TAG, "Error starting PPPoS (%d)", res);
	        }
            continue; // Main task's loop
		}

		// ===== LOOP: Handle GSM modem responses & disconnect requests ===================
        if (gsm_debug) {
            LOGM(GSM_PPP_TAG, "PPPoS loop");
        }
		while(1) {
	        gstat = 1;
            // === Check if disconnect or end task was requested ===
			if (do_pppos_connect <= 0) {
			    // Disconnect or task end requested
                int end_task = do_pppos_connect;
                if (gsm_debug) {
                    if (end_task < 0) {
                        LOGM(GSM_PPP_TAG, "Task end requested.");
                    }
                    else {
                        LOGM(GSM_PPP_TAG, "Disconnect requested.");
                    }
                }

                do_pppos_connect = 1;
				// Close PPPoS, ATDEV_STATEDISCONNECTED status will be set by PPPoS
				pppapi_close(ppp_ppp_pcb, 1);

				// Wait for disconnect
				uint64_t wait_end = mp_hal_ticks_ms() + 30000;
				while (ppp_status != ATDEV_STATEDISCONNECTED) {
                    // --- Handle data received from GSM (to be passed to the PPPoS) ---
                    //-----------------------
                    _handle_pppos_data(data);
                    //-----------------------

                    if (mp_hal_ticks_ms() > wait_end) {
		                if (gsm_debug) {
		                    LOGW(GSM_PPP_TAG, "Disconnect failed (timeout.");
		                }
		                break;
				    }
				}

                // disconnect modem
                vTaskDelay(500 / portTICK_PERIOD_MS);
                at_uart_flush(gsm_uart_num);
                mutex_taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
                bool dissc = _disconnect(&command, gsm_uart_num);
                if (mutex_taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
                mutex_taken = 0;

                if (gsm_debug) {
                    if (dissc) {
                        LOGM(GSM_PPP_TAG, "PPPoS Disconnected, GSM disconnected");
                    }
                    else {
                        LOGW(GSM_PPP_TAG, "PPPoS Disconnected, GSM still online");
                    }
				}

                if (end_task < 0) goto exit; // exit task requested

                // === Prepare commands ===
				gsmCmdIter = 0;
				if (full_connect_init) enableAllInitCmd();
	            cmd_Connect.skip = 0;
	            cmd_APN.skip = 0;
                do_pppos_connect = 0;
                ppp_status = ATDEV_STATEIDLE;
				break; // exit to the Main task's loop (will stay in IDLE mode)
			}

            // =================================================================================
			// === In PPPoS connected mode =====================================================
            // =================================================================================

			// Check if disconnected (shouldn't be)
			if (ppp_status == ATDEV_STATEDISCONNECTED) {
				ppp_status = ATDEV_STATEIDLE;
				if (gsm_debug) {
					LOGE(GSM_PPP_TAG, "Disconnected, trying to connect again...");
				}

				pppapi_close(ppp_ppp_pcb, 1);

				enableAllInitCmd();
				gsmCmdIter = 0;
				vTaskDelay(3000 / portTICK_PERIOD_MS);
				// goto main loop, initialize the GSM modem again
				break;
			}

			// --- Handle data received from GSM (to be passed to the PPPoS) ---
			//-----------------------
			_handle_pppos_data(data);
            //-----------------------

			// =================================================================================

		}  // Handle GSM modem responses & disconnects loop

		if (gstat < 0) break;  // terminate task requested
	}  // main task loop

exit:
	// Terminate GSM task
    if (mutex_taken == pdTRUE) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

    // free PPP control block
    if (ppp_ppp_pcb) {
        ppp_free(ppp_ppp_pcb);
        ppp_ppp_pcb = NULL;
    }

    if (data) vPortFree(data);

    uart_deinit(gsm_uart_num, NULL, gsm_pin_tx, gsm_pin_rx);
    if (gsm_debug) {
        LOGM(GSM_PPP_TAG,"UART #%d deinitialized", gsm_uart_num);
    }

    gsm_pppos_task_started = 0;
	ppp_status = ATDEV_STATEFIRSTINIT;

	if (gsm_debug) {
		LOGM(GSM_PPP_TAG, "PPPoS TASK TERMINATED");
	}
	vTaskDelete(NULL);
}

//=====================================================================================================================================
int gsm_ppposInit(int tx, int rx, int rts, int cts, int bdr, char *user, char *pass, char *apn, uint8_t wait, int doconn, bool roaming)
{
    int task_s = gsm_pppos_task_started;
    int tmo = 0;

    if (task_s == 1) return 0;
    if (task_s == -1) {
        // Task starting
        tmo = 2000;
        while (task_s == -1) {
            vTaskDelay(10 / portTICK_RATE_MS);
            tmo -= 10;
            if (tmo <= 0) return -1;
            task_s = gsm_pppos_task_started;
        }
    }

    if (task_s == 0) {
        // PPPoS task not running, create it
        gsm_pin_tx = tx;
        gsm_pin_rx = rx;
        gsm_pin_cts = cts;
        gsm_pin_rts = rts;
        gsm_uart_baudrate = bdr;
        do_pppos_connect = doconn;
        allow_roaming = roaming;
        memset(PPP_User, 0, sizeof(PPP_User));
        memset(PPP_Pass, 0, sizeof(PPP_Pass));
        memset(GSM_APN, 0, sizeof(GSM_APN));
        strncpy(PPP_User, user, PPP_MAX_NAME_LEN);
        strncpy(PPP_Pass, pass, PPP_MAX_NAME_LEN);
        strncpy(GSM_APN, apn, PPP_MAX_NAME_LEN);

        if (!tcpip_adapter_initialized) {
            if (gsm_debug) LOGM(GSM_PPP_TAG,"Network init (from GSM module)");
            network_init();
            tcpip_adapter_initialized = true;
        }

        TaskHandle_t curr_task_handle = xTaskGetCurrentTaskHandle();
        // Create and start GSM task
        BaseType_t res = xTaskCreate(
            gsm_pppos_client_task,     // function entry
            "GSM_PPPoS",               // task name
            configMINIMAL_STACK_SIZE,  // stack_deepth
            curr_task_handle,          // function argument
            GSM_TASK_PRIORITY,         // task priority
            &gsmPPPoSTaskHandle        // task handle
        );
        if ((res != pdPASS) || (gsmPPPoSTaskHandle == NULL)) {
            return -2;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
        // The task should be running by now
        task_s = gsm_pppos_task_started;
        if (task_s == -1) {
            // Task starting
            tmo = 2000;
            while (task_s == -1) {
                vTaskDelay(10 / portTICK_RATE_MS);
                tmo -= 10;
                if (tmo <= 0) return -1;
                task_s = gsm_pppos_task_started;
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
            task_s = gsm_pppos_task_started;
            if (task_s == 0) return -3;
            if ((ppp_status == ATDEV_STATEIDLE) || (ppp_status == ATDEV_STATECONNECTED) || (ppp_status == ATDEV_STATECONNECTING)) return 0;
        }
    }

    return 0;
}

//=============================================================
int gsm_ppposConnect(char *conn_str, bool full_init, bool wait)
{
	if (mpy_uarts[gsm_uart_num].uart_mutex == NULL) return -1;
    if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) return -2;

    do_pppos_connect = 1;
	full_connect_init = full_init;
	int gstat = ppp_status;
	int task_s = gsm_pppos_task_started;
	int do_conn = 1;
	if (conn_str) {
	    sprintf(gsm_connect_string, "%s\r\n", conn_str);
	}
	xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

	if (task_s == 0) return -2;                  // error, task not stared
	if (gstat == ATDEV_STATECONNECTED) return 0; // ok, already connected

	if (wait) {
        // wait up to 25 seconds for connect
        int tmo = 0;
        while (gstat != ATDEV_STATECONNECTED) {
            vTaskDelay(100 / portTICK_RATE_MS);
            if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) == pdTRUE) {
                gstat = ppp_status;
                task_s = gsm_pppos_task_started;
                do_conn = do_pppos_connect;
                xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
            }
            if (task_s == 0) return -2;
            if ((do_conn <= 0) && (tmo > 10)) return -1;
            tmo++;
            if (tmo > 2500) return -1;
        }
	}

	return 0;
}

//===============================================
int gsm_ppposDisconnect(bool end_task, bool wait)
{
	if (mpy_uarts[gsm_uart_num].uart_mutex == NULL) return -1;
    if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) return -2;

    int gstat = ppp_status;
	int task_s = gsm_pppos_task_started;
	xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

	if (task_s == 0) return -2;                                  // error, task not stared
	if ((gstat == ATDEV_STATEIDLE) && (!end_task)) return 0; // already not connected, end task not requested

	vTaskDelay(2000 / portTICK_RATE_MS);
    if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) != pdTRUE) return -2;

    if (end_task) do_pppos_connect = -1;
	else do_pppos_connect = 0;
	xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

	if (wait) {
        gstat = 0;
        int tmo = 0;
        if (end_task) {
            while ((gstat != ATDEV_STATEIDLE) && (task_s != 0)) {
                if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) == pdTRUE) {
                    gstat = ppp_status;
                    task_s = gsm_pppos_task_started;
                    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
                }
                vTaskDelay(10 / portTICK_RATE_MS);
                tmo++;
                if (tmo > 1500) return -1;
            }
            if (task_s == 0) return 0;
        }
        else {
            while ((gstat != ATDEV_STATEIDLE) && (task_s != 0)) {
                vTaskDelay(100 / portTICK_RATE_MS);
                if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT) == pdTRUE) {
                    gstat = ppp_status;
                    task_s = gsm_pppos_task_started;
                    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
                }
                tmo++;
                if (tmo > 1500) return -1;
            }
        }
	}
	return 0;
}

//=============
int gsm_RFOff()
{
	if (mpy_uarts[gsm_uart_num].uart_mutex == NULL) return 1;
	if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) return 0;
	int gstat = ppp_status;

	if (gstat != ATDEV_STATEIDLE) {
	    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
	    return 0;
	}

    memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    memset(&gsm_at_command, 0, sizeof(at_command_t));
    gsm_at_command.cmdSize = -1;
    gsm_at_command.responses = &gsm_at_responses;
    gsm_at_command.at_uart_num = gsm_uart_num;
    gsm_at_command.dbg = gsm_debug;
    gsm_at_command.flush = true;

    bool ret = _do_rfOnOff(&gsm_at_command, false);
    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

    return (ret) ? 1 : 0;
}

//============
int gsm_RFOn()
{
	if (mpy_uarts[gsm_uart_num].uart_mutex == NULL) return 1;

    if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) return 0;
	int gstat = ppp_status;

    if (gstat != ATDEV_STATEIDLE) {
        xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
        return 0;
    }

    memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    memset(&gsm_at_command, 0, sizeof(at_command_t));
    gsm_at_command.cmdSize = -1;
    gsm_at_command.responses = &gsm_at_responses;
    gsm_at_command.at_uart_num = gsm_uart_num;
    gsm_at_command.dbg = gsm_debug;
    gsm_at_command.flush = true;

    bool ret = _do_rfOnOff(&gsm_at_command, true);
    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

    return (ret) ? 1 : 0;
}


// ==== SMS Functions ==========================================================================

// Check if GSM is online (connected),
// if yes SMS and some other commands cannot be executed
//---------------------
static bool is_OnLine()
{
	if (ppposStatus(NULL, NULL, NULL) != ATDEV_STATEIDLE) {
	    if (gsm_debug) {
	        LOGW(GSM_PPP_TAG, "Online, cannot execute");
	    }
	    return true;
	}
	bool ret = true;

    if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) {
        if (gsm_debug) {
            LOGW(GSM_PPP_TAG, "Cannot get mutex");
        }
        return true;
    }
	doCheckSMS = 0;

	// RF must be on
    memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    memset(&gsm_at_command, 0, sizeof(at_command_t));
    gsm_at_command.cmdSize = -1;
    gsm_at_command.responses = &gsm_at_responses;
    gsm_at_command.at_uart_num = gsm_uart_num;
    gsm_at_command.dbg = gsm_debug;
    gsm_at_command.flush = true;

    ret = _do_rfOnOff(&gsm_at_command, true);
    if ((!ret) && (gsm_debug)) {
        LOGW(GSM_PPP_TAG, "Cannot set GSM RFOn");
        ret = true;
    }
    else ret = false;

    doCheckSMS = 1;

    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
	return ret;
}

//-------------------------------------------------
static time_t sms_time(char * msg_time, int *tzone)
{
	if (strlen(msg_time) >= 20) {
		// Convert message time to time structure
		int hh,mm,ss,yy,mn,dd, tz;
		struct tm tm;
		sscanf(msg_time, "%u/%u/%u,%u:%u:%u%d", &yy, &mn, &dd, &hh, &mm, &ss, &tz);
		tm.tm_hour = hh;
		tm.tm_min = mm;
		tm.tm_sec = ss;
		tm.tm_year = yy+100;
		tm.tm_mon = mn-1;
		tm.tm_mday = dd;
		if (tzone) *tzone = tz/4;	// time zone info
		return mktime(&tm);	// Linux time
	}
	return 0;
}

// Parse message in buffer to message structure
//---------------------------------------------------------------
static int getSMS(char *msgstart, SMS_Msg *msg, uint8_t msgalloc)
{
    // Message starts
    //   here ˇ
    // +CMGL: 0,"REC READ","4951575555","","19/03/08,00:58:06+04"\r\nsome message tsex ....\r\n
	char *msgidx = msgstart;
	// Clear message structure
	memset(msg, 0, sizeof(SMS_Msg));

	// Get message info
	char *pend = strstr(msgidx, "\r\n"); // message header end pointer
	if (pend == NULL) return 0;

	int len = pend-msgidx;
	char hdr[len+4];
	char buf[32];

	memset(hdr, 0, len+4);
	memcpy(hdr, msgidx, len);
	hdr[len] = '\0';

	if (msgalloc) {
		msgidx = pend + 2;
		// Allocate message body buffer and copy the data
		len = strlen(msgidx);
		msg->msg = (char *)pvPortMalloc(len+1);
		if (msg->msg) {
		    memset(msg->msg, 0, len+1);
			memcpy(msg->msg, msgidx, len);
			msg->msg[len] = '\0';
		}
	}

	// Parse message info
	msgidx = hdr;
	pend = strstr(hdr, ",\"");
	int i = 1;
	while (pend != NULL) {
		len = pend-msgidx;
		if ((len < 32) && (len > 0)) {
			memset(buf, 0, 32);
			strncpy(buf, msgidx, len);
			buf[len] = '\0';
			if (buf[len-1] == '"') buf[len-1] = '\0';

			if (i == 1) msg->idx = (int)strtol(buf, NULL, 0);   // ** message index
			else if (i == 2) strcpy(msg->stat, buf);			// ** message status
			else if (i == 3) strcpy(msg->from, buf);			// ** phone number of message sender
			else if (i == 5) strcpy(msg->time, buf);			// ** the time when the message was sent
		}
		i++;
		msgidx = pend + 2;
		pend = strstr(msgidx, ",\"");
		if (pend == NULL) pend = strstr(msgidx, "\"");
	}

	msg->time_value = sms_time(msg->time, &msg->tz);

	return 1;
}

// Get message index and time
//-----------------------------------------------------
static int getSMSindex(char *msgstart, time_t *msgtime)
{
    // Message starts
    //   here ˇ
    // +CMGL: 0,"REC READ","4951575555","","19/03/08,00:58:06+04"\r\n........
	char *msgidx = msgstart;
	// Get message info
	char *pend = strstr(msgidx, "\r\n"); // message header end pointer
	if (pend == NULL) return 0;

	int len = pend-msgidx;
	char hdr[len+4];
	char buf[32];
	char msg_time[32] = {'\0'};
	int msg_idx = 0;

	memcpy(hdr, msgidx, len);
	hdr[len] = '\0';

	// Parse message info from header
	msgidx = hdr;
	pend = strstr(hdr, ",\"");
	int i = 1;
	while (pend != NULL) {
		len = pend-msgidx;
		if ((len < 32) && (len > 0)) {
			memset(buf, 0, 32);
			memcpy(buf, msgidx, len);
			buf[len] = '\0';
			if (buf[len-1] == '"') buf[len-1] = '\0';

			if (i == 1) msg_idx = (int)strtol(buf, NULL, 0);	// ** message index
			else if (i == 5) strcpy(msg_time, buf);				// ** the time when the message was sent
		}
		i++;
		msgidx = pend + 2;
		// find next entry
		pend = strstr(msgidx, ",\"");
		if (pend == NULL) pend = strstr(msgidx, "\"");
	}

	*msgtime = sms_time(msg_time, NULL);

	return msg_idx;
}

//-------------------------------------------------------------------------------------------------
int checkMessages(uint8_t rd_status, int sms_idx, SMS_Msg *msg, SMS_indexes *indexes, uint8_t sort)
{
    char cmd[32] = {0};
    char *list_change = "";
    if (msg == NULL) list_change = ",1";
    char *rbuffer = pvPortMalloc(1024);
    if (rbuffer == NULL) {
        if (gsm_debug) {
            LOGE(GSM_PPP_TAG,"Check SMS, Error allocating receive buffer");
        }
        return 0;
    }
    memset(rbuffer, 0, 1024);

    // ** Send command to GSM and get the response
    if (rd_status == SMS_LIST_NEW) sprintf(cmd, "%s%s\r\n", SMS_LIST_NEW_STR, list_change);
    else if (rd_status == SMS_LIST_OLD) sprintf(cmd, "%s%s\r\n", SMS_LIST_OLD_STR, list_change);
    else sprintf(cmd, "%s%s\r\n", SMS_LIST_ALL_STR, list_change);

    // Read all messages to buffer
    at_responses_t at_responses1;
    memset(&at_responses1, 0, sizeof(at_responses_t));
    at_responses1.nresp = 2;
    at_responses1.resp[0] = AT_OK_Str;
    at_responses1.resp[1] = AT_Error_Str;

    memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    gsm_at_responses.nresp = 2;
    gsm_at_responses.resp[0] = "\r\n\r\nOK\r\n"; // wait for end of messages list
    gsm_at_responses.resp[1] = AT_Error_Str;

    at_commands_t at_commands;
    memset(&at_commands, 0, sizeof(at_commands_t));
    at_commands.ncmd = 2;
    at_commands.at_uart_num = gsm_uart_num;
    at_commands.dbg = gsm_debug;
    at_commands.expect_resp[0] = 1;
    at_commands.expect_resp[1] = 1;

    at_commands.commands[0].cmd = "AT+CMGF=1\r\n";
    at_commands.commands[0].cmdSize = -1;
    at_commands.commands[0].responses = &at_responses1;
    at_commands.commands[0].timeout = 100;
    at_commands.commands[0].at_uart_num = gsm_uart_num;
    at_commands.commands[0].dbg = gsm_debug;
    at_commands.commands[0].flush = true;

    at_commands.commands[1].cmd = cmd;
    at_commands.commands[1].cmdSize = strlen(cmd);
    at_commands.commands[1].responses = &gsm_at_responses;
    at_commands.commands[1].timeout = 2000;
    at_commands.commands[1].at_uart_num = gsm_uart_num;
    at_commands.commands[1].dbg = gsm_debug;
    at_commands.commands[1].respbSize = 1024;
    at_commands.commands[1].respbuff = rbuffer;
    at_commands.commands[1].flush = true;

    int buflen = 0;
    int res, n_proc = 0;
    int msgidx = 0;
    time_t msgtime = 0;

    // Execute command
    res = gsm_at_Commands(&at_commands, &n_proc);

    if ((n_proc == 2) && (res == 1)) buflen = strlen(at_commands.commands[1].respbuff);

	if (indexes !=NULL) memset(indexes, 0, sizeof(SMS_indexes));
	// The buffer may have been expanded
	rbuffer = at_commands.commands[1].respbuff;

	// Parse the response
	int nmsg = 0;
	uint8_t idx_found = 0;
	char *msgstart = rbuffer;
	char *msgend_h = NULL;
    char *msgend_t = NULL;

    while (buflen > 0) {
        // Check message start string
        msgstart = strstr(rbuffer, "+CMGL: ");
        if (msgstart) {
            msgend_h = strstr(msgstart, "\r\n"); // find message header terminator
            if (msgend_h) {
                msgend_t = strstr(msgend_h+2, "\r\n"); // find message text terminator
                if (msgend_h) *msgend_t = '\0'; // terminate the message string
            }
        }
        if ((msgstart) && (msgend_h) && (msgend_t)) {
            // We have at least one whole message in the buffer
            nmsg++;
            msgidx = getSMSindex(msgstart+7, &msgtime);
            if ((indexes !=NULL) && (nmsg < SMS_MAX_MESSAGES)) {
                // Only save message index and time
                indexes->idx[nmsg-1] = msgidx;
                indexes->time[nmsg-1] = msgtime;
            }
            if ((sms_idx == msgidx) && (msg != NULL)) {
                // Get the message text if requested, save message info and text
                getSMS(msgstart+7, msg, 1);
                idx_found = 1;
                break;
            }

            // Delete the processed message from buffer
            memmove(rbuffer, msgend_t+2, buflen - (msgend_t-rbuffer+2));
            buflen -= (msgend_t-rbuffer+2);
            rbuffer[buflen] = '\0';
        }
        else break;
    }

	vPortFree(rbuffer);

	if ((msg != NULL) && (idx_found == 0)) {
	    // Message text at index requested, but no message found
	    return 0;
	}

	if ((nmsg > 0) && (indexes != NULL) && (sort != SMS_SORT_NONE)) {
		// Only indexes requested and sort is given, sort the messages
    	bool f;
    	int temp;
    	time_t tempt;
		for (int i=0; i<nmsg; ++i) {
		    for (int j=i+1; j<nmsg; ++j) {
		    	if (sort == SMS_SORT_ASC) f = (indexes->time[i] > indexes->time[j]);
		    	else f = (indexes->time[i] < indexes->time[j]);
		        if (f) {
		            temp = indexes->idx[i];
		            tempt = indexes->time[i];
		            indexes->idx[i] = indexes->idx[j];
		            indexes->time[i] = indexes->time[j];
		            indexes->idx[j] = temp;
		            indexes->time[j] = tempt;
		        }
		    }
		}
	}
	return nmsg;
}

//==================================
int smsSend(char *smsnum, char *msg)
{
	if (is_OnLine()) return 0;

	doCheckSMS = 0;

	char *msgbuf = NULL;
	int res = 0;
	char cmd[64];
	int len = strlen(msg);
    sprintf(cmd, "AT+CMGS=\"%s\"\r\n", smsnum);

    msgbuf = pvPortMalloc(len+2);
    if (msgbuf == NULL) {
        res = 0;
        if (gsm_debug) {
            LOGE(GSM_PPP_TAG, "Error allocating message buffer");
        }
        goto exit;
    }

    sprintf(msgbuf, "%s\x1A", msg);

    at_responses_t at_responses1;
    memset(&at_responses1, 0, sizeof(at_responses_t));
    at_responses1.nresp = 2;
    at_responses1.resp[0] = "+CMGS: ";
    at_responses1.resp[1] = AT_Error_Str;

    at_commands_t at_commands;
    memset(&at_commands, 0, sizeof(at_commands_t));
    at_commands.ncmd = 2;
    at_commands.at_uart_num = gsm_uart_num;
    at_commands.dbg = gsm_debug;
    at_commands.delay[0] = 5;

    memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    gsm_at_responses.nresp = 2;
    gsm_at_responses.resp[0] = ">";
    gsm_at_responses.resp[1] = AT_Error_Str;

    at_commands.commands[0].cmd = cmd;
    at_commands.commands[0].cmdSize = -1;
    at_commands.commands[0].responses = &gsm_at_responses;
    at_commands.commands[0].timeout = 5000;
    at_commands.commands[0].at_uart_num = gsm_uart_num;
    at_commands.commands[0].dbg = gsm_debug;
    at_commands.commands[0].flush = true;

    at_commands.commands[1].cmd = msgbuf;
    at_commands.commands[1].cmdSize = strlen(msgbuf);
    at_commands.commands[1].type_data = true;
    at_commands.commands[1].responses = &at_responses1;
    at_commands.commands[1].timeout = 40000;
    at_commands.commands[1].at_uart_num = gsm_uart_num;
    at_commands.commands[1].dbg = gsm_debug;
    at_commands.commands[1].flush = true;

    res = gsm_at_Commands(&at_commands, NULL);
    if (res != 1) {
        // Try to recover
        at_uart_write(gsm_uart_num, (uint8_t *)"\x1B", 1);
    }

exit:
    if (msgbuf) vPortFree(msgbuf);

	doCheckSMS = 1;

	return res;
}

//============================================================
int smsCount(uint8_t type, SMS_indexes *indexes, uint8_t sort)
{
	if (is_OnLine()) return -1;

	doCheckSMS = 0;

	int res = checkMessages(type, 0, NULL, indexes, sort);

	doCheckSMS = 1;

	return res;
}

//===================================================================================================
int getMessagesList(uint8_t rd_status, int sms_idx, SMS_Msg *msg, SMS_indexes *indexes, uint8_t sort)
{
	if (is_OnLine()) return -1;

	doCheckSMS = 0;

	int res = checkMessages(rd_status, sms_idx, msg, indexes, sort);

	doCheckSMS = 1;

	return res;
}

//====================
int smsDelete(int idx)
{
	if (is_OnLine()) return 0;

	char cmd[64];

	doCheckSMS = 0;

	sprintf(cmd,"AT+CMGD=%d\r\n", idx);

	memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    memset(&gsm_at_command, 0, sizeof(at_command_t));
    gsm_at_responses.nresp = 2;
    gsm_at_responses.resp[0] = AT_OK_Str;
    gsm_at_responses.resp[1] = AT_Error_Str;
    gsm_at_command.cmd = cmd;
    gsm_at_command.cmdSize = -1;
    gsm_at_command.responses = &gsm_at_responses;
    gsm_at_command.timeout = 5000;
    gsm_at_command.at_uart_num = gsm_uart_num;
    gsm_at_command.dbg = gsm_debug;
    gsm_at_command.flush = true;

    int ret = gsm_at_Cmd(&gsm_at_command);

	doCheckSMS = 1;

	return ret;
}

//---------------------------------------
bool gsm_set_baudrate(int bdr, bool perm)
{
    if (is_OnLine()) return 0;

    char cmd[64];
    if (perm) sprintf(cmd, "AT+IPREX=%d\r\n", bdr);
    else sprintf(cmd, "AT+IPR=%d\r\n", bdr);
    memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    memset(&gsm_at_command, 0, sizeof(at_command_t));
    gsm_at_responses.nresp = 2;
    gsm_at_responses.resp[0] = AT_OK_Str;
    gsm_at_responses.resp[1] = AT_Error_Str;
    gsm_at_command.cmd = cmd;
    gsm_at_command.cmdSize = -1;
    gsm_at_command.responses = &gsm_at_responses;
    gsm_at_command.timeout = 500;
    gsm_at_command.at_uart_num = gsm_uart_num;
    gsm_at_command.dbg = gsm_debug;
    gsm_at_command.flush = true;
    int res = gsm_at_Cmd(&gsm_at_command);

    if (res == 1) {
        int baud = mp_uart_config(gsm_uart_num, bdr, 8, UART_STOP_1, UART_PARITY_NONE);
        gsm_uart_baudrate = bdr;
        if (gsm_debug) {
            LOGM(GSM_PPP_TAG, "GSM uart baudrate set to %d (%d)", bdr, baud);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    return (res == 1);
}


//==========================
int setNTP_cb(void *cb_func)
{
    int stat = ppposStatus(NULL, NULL, NULL);
    int taken = pdFALSE;
    if ((stat != ATDEV_STATEFIRSTINIT) && (mpy_uarts[gsm_uart_num].uart_mutex != NULL))
        taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5);

    ntp_time_cb = cb_func;

    if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
    return 1;
}

//=============================================
int setSMS_cb(void *cb_func, uint32_t interval)
{
    int stat = ppposStatus(NULL, NULL, NULL);
    int taken = pdFALSE;
    if ((stat != ATDEV_STATEFIRSTINIT) && (mpy_uarts[gsm_uart_num].uart_mutex != NULL))
        taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5);

    new_SMS_cb = cb_func;
	SMS_check_interval = interval;
    sms_timer = mp_hal_ticks_ms() + SMS_check_interval;
    doCheckSMS = 1;

    if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
	return 1;
}

//=========================
void gsm_setDebug(bool dbg)
{
    int stat = ppposStatus(NULL, NULL, NULL);
    int taken = pdFALSE;
	if ((stat != ATDEV_STATEFIRSTINIT) && (mpy_uarts[gsm_uart_num].uart_mutex != NULL)) taken = xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT);
	gsm_debug = dbg;
	if (taken) xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);
}

//===================================
int gsm_at_Cmd(at_command_t *command)
{
	if (ppposStatus(NULL, NULL, NULL) != ATDEV_STATEIDLE) return 0;

    if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) {
        if (gsm_debug) {
            LOGE(GSM_PPP_TAG, "ATCmd: Coldn't get mutex");
        }
        return 0;
    }
    int res = at_Cmd_Response(command);
    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

    return res;
}

//==========================================================
int gsm_at_Commands(at_commands_t *commands, int *processed)
{
    if (ppposStatus(NULL, NULL, NULL) != ATDEV_STATEIDLE) return 0;

    if (xSemaphoreTake(mpy_uarts[gsm_uart_num].uart_mutex, PPPOSMUTEX_TIMEOUT*5) != pdTRUE) {
        if (gsm_debug) {
            LOGE(GSM_PPP_TAG, "ATCommands: Coldn't get mutex");
        }
        return 0;
    }
    int res = at_Commands(commands, processed);
    xSemaphoreGive(mpy_uarts[gsm_uart_num].uart_mutex);

    return res;
}

//============================================================
int ppposStatus(uint32_t *ip, uint32_t *netmask, uint32_t *gw)
{
    if (mpy_uarts[gsm_uart_num].uart_mutex == NULL) return ATDEV_STATEFIRSTINIT;

    int gstat = ppp_status;
    if (ip) *ip = gsm_ppp_ip;
    if (netmask) *netmask = gsm_ppp_netmask;
    if (gw) *gw = gsm_ppp_gw;

    return gstat;
}

//==============================================================
void pppos_getRxTxCount(uint32_t *rx, uint32_t *tx, uint8_t rst)
{
    *rx = 0;
    *tx = 0;
    if (mpy_uarts[gsm_uart_num].uart_mutex == NULL) return;

    *rx = pppos_rx_count;
    *tx = pppos_tx_count;
    if (rst) {
        pppos_rx_count = 0;
        pppos_tx_count = 0;
    }
}

//=========================
void pppos_resetRxTxCount()
{
    if (mpy_uarts[gsm_uart_num].uart_mutex == NULL) return;
    pppos_rx_count = 0;
    pppos_tx_count = 0;
}


// === Global functions (used by socket module) ==============

//-------------------------------------------------------------------------------------------------------------------
int gsm_get_addrinfo(const char *domain, const char *portname, const struct addrinfo *hints, struct addrinfo **resp)
{
    int port_nr = 0;
    ip_addr_t addr;
    *resp = NULL;

    if ((domain == NULL) || (strlen(domain) >= 64)) {
        if (gsm_debug) {
            LOGE(GSM_PPP_TAG, "No domain");
        }
        return EAI_NONAME;
    }
    if (portname != NULL) {
      // port string specified: convert to port number
      port_nr = atoi(portname);
      if ((port_nr <= 0) || (port_nr > 0xffff)) {
          if (gsm_debug) {
              LOGE(GSM_PPP_TAG, "Wrong port");
          }
          return EAI_SERVICE;
      }
    }
    memset(at_canonname, 0, sizeof(at_canonname));

    char cmd[128];
    sprintf(cmd, "AT+CDNSGIP=\"%s\"\r\n", domain);

    memset(&gsm_at_responses, 0, sizeof(at_responses_t));
    memset(&gsm_at_command, 0, sizeof(at_command_t));
    gsm_at_responses.nresp = 2;
    gsm_at_responses.resp[0] = AT_OK_Str;
    gsm_at_responses.resp[1] = AT_Error_Str;
    gsm_at_command.cmd = cmd;
    gsm_at_command.cmdSize = -1;
    gsm_at_command.responses = &gsm_at_responses;
    gsm_at_command.timeout = 18000;
    gsm_at_command.at_uart_num = gsm_uart_num;
    gsm_at_command.dbg = gsm_debug;
    gsm_at_command.flush = true;
    gsm_at_command.respbuff = pvPortMalloc(256);
    if (gsm_at_command.respbuff == NULL) {
        if (gsm_debug) {
            LOGE(GSM_PPP_TAG, "Buffer allocation error");
        }
        return -1;
    }
    gsm_at_command.respbSize = 256;
    // If the IP cannot be resolved it can take up to 15 seconds !!
    int res = gsm_at_Cmd(&gsm_at_command);

    if (res <= 0) {
        if (gsm_debug) {
            LOGE(GSM_PPP_TAG, "No response");
        }
        return -1;
    }

    char *pbuf1 = strstr(gsm_at_command.respbuff, "+CDNSGIP: 1,");
    if (pbuf1) {
        pbuf1 += 12;
        char *pbuf = strstr(pbuf1, "\",\"");
        if (pbuf) {
            pbuf += 3;
            struct addrinfo *gsm_addrinfo = (struct addrinfo *)pvPortMalloc(sizeof(struct addrinfo)+sizeof(struct sockaddr_storage));
            if (gsm_addrinfo == NULL) {
                if (gsm_debug) {
                    LOGE(GSM_PPP_TAG, "Error allocating addrinfo");
                }
                vPortFree(gsm_at_command.respbuff);
                return -3;
            }

            struct sockaddr_storage *sa = NULL;
            sa = (struct sockaddr_storage *)(void *)((u8_t *)gsm_addrinfo + sizeof(struct addrinfo));
            struct sockaddr_in *sa4 = (struct sockaddr_in *)sa;
            gsm_addrinfo->ai_flags = 0;
            gsm_addrinfo->ai_next = NULL;
            if (hints != NULL) {
                gsm_addrinfo->ai_family = hints->ai_family;
                gsm_addrinfo->ai_socktype = hints->ai_socktype;
                gsm_addrinfo->ai_protocol = hints->ai_protocol;
            }
            else {
                gsm_addrinfo->ai_family = AF_INET;
                gsm_addrinfo->ai_socktype = SOCK_STREAM;
                gsm_addrinfo->ai_protocol = 0;
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
            gsm_addrinfo->ai_family = AF_INET;

            // copy domain to canonname
            snprintf(at_canonname, DNS_MAX_NAME_LENGTH, "%s", domain);

            gsm_addrinfo->ai_canonname = (char *)&at_canonname[0];
            gsm_addrinfo->ai_addrlen = sizeof(struct sockaddr_storage);
            gsm_addrinfo->ai_addr = (struct sockaddr *)sa;
            *resp = gsm_addrinfo;

            vPortFree(gsm_at_command.respbuff);
            return 0;
        }
    }
    else if (gsm_debug) {
        LOGW(GSM_PPP_TAG, "Wrong response [%s]", gsm_at_command.respbuff);
    }
    vPortFree(gsm_at_command.respbuff);
    return -2;
}


#endif
