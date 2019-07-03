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

#ifndef _LIBGSM_H_
#define _LIBGSM_H_

#include "at_util.h"
#include "mpconfigport.h"

#if MICROPY_PY_USE_GSM

#include <time.h>
#include <stdint.h>

#define SMS_SORT_NONE	0
#define SMS_SORT_ASC	1
#define SMS_SORT_DESC	2

#define SMS_LIST_ALL	0
#define SMS_LIST_NEW	1
#define SMS_LIST_OLD	2
#define SMS_LIST_ALL_STR	"AT+CMGL=\"ALL\"\r\n"
#define SMS_LIST_NEW_STR	"AT+CMGL=\"REC UNREAD\"\r\n"
#define SMS_LIST_OLD_STR	"AT+CMGL=\"REC READ\"\r\n"

#define SMS_MAX_MESSAGES    64


typedef struct
{
	int		idx;
	char	*msg;
	char	stat[32];
	char	from[32];
	char	time[32];
	time_t	time_value;
	int		tz;
}SMS_Msg;

typedef struct
{
	uint8_t	idx[SMS_MAX_MESSAGES];
	time_t	time[SMS_MAX_MESSAGES];
}SMS_indexes;

extern bool gsm_debug;
extern int gsm_uart_num;
extern at_responses_t gsm_at_responses;
extern at_command_t gsm_at_command;
extern const char *GSM_PPP_TAG;
extern int gsm_uart_baudrate;


/*
 * Create GSM/PPPoS task if not already created
 * Initialize GSM and connect to Internet
 * Handle all PPPoS requests
 * Disconnect/Reconnect from/to Internet on user request
 * If 'wait' = 1, wait until connected
 * If 'doconn' = 0, only initialize the task, do not connect to Internet
 */
//======================================================================================================================================
int gsm_ppposInit(int tx, int rx, int rts, int cts, int bdr, char *user, char *pass, char *apn, uint8_t wait, int doconn, bool roaming);

/*
 * Disconnect from Internet
 * If 'end_task' = 1 also terminate GSM/PPPoS task
 * If already disconnected, this function does nothing
 */
//================================================
int gsm_ppposDisconnect(bool end_task, bool wait);

/*
 * Connect from Internet
 * If already connected, this function does nothing
 */
//==============================================================
int gsm_ppposConnect(char *conn_str, bool full_init, bool wait);

/*
 * Turn GSM RF Off
 */
//==============
int gsm_RFOff();

/*
 * Turn GSM RF On
 */
//=============
int gsm_RFOn();

/*
 * Send SMS
 *
 * Params:
 *   smsnum:	Pointer to phone number in international format (+<counry_code><gsm number>)
 *      msg:	Pointer to message text
 */
//==================================
int smsSend(char *smsnum, char *msg);

/*
 * Get messages list
 *
 * Params:
 *  rd_status:	check all, unread or read messages
 *    sms_idx:	return sms at index in msg
 *        msg:	SMS message structure pointer
 *    indexes:	pointer to indexes of the detected message
 *       sort:	sort the indexes
 */
//====================================================================================================
int getMessagesList(uint8_t rd_status, int sms_idx, SMS_Msg *msg, SMS_indexes *indexes, uint8_t sort);

/*
 * return number of messages of given type
 * and, optionally the indexes of all new messages
 */
//=============================================================
int smsCount(uint8_t type, SMS_indexes *indexes, uint8_t sort);

/*
 * Delete the message at GSM message index 'idx'
 */
//=====================
int smsDelete(int idx);

//==============================================
int setSMS_cb(void *cb_func, uint32_t interval);

//==========================
void gsm_setDebug(bool dbg);

//====================================
int gsm_at_Cmd(at_command_t *command);

//===========================================
int gsm_at_Commands(at_commands_t *commands);

//========================================
bool gsm_set_baudrate(int bdr, bool perm);

#endif
#endif

