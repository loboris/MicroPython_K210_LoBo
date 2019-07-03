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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "syslog.h"

#include "py/runtime.h"
#include "py/obj.h"
#include "lib/netutils/netutils.h"
#include "mphalport.h"
#include "modmachine.h"
#include "libGSM.h"

//-------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_startGSM(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
			{ MP_QSTR_tx,		    MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = -1} },
			{ MP_QSTR_rx,		    MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = -1} },
			{ MP_QSTR_baudrate,	                      MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = 115200} },
			{ MP_QSTR_user,                           MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
			{ MP_QSTR_password,                       MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
			{ MP_QSTR_apn,          MP_ARG_REQUIRED | MP_ARG_KW_ONLY  | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
			{ MP_QSTR_connect,                        MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false} },
			{ MP_QSTR_wait,	                          MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false} },
			{ MP_QSTR_rts,		                      MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = -1} },
			{ MP_QSTR_cts,		                      MP_ARG_KW_ONLY  | MP_ARG_INT,  {.u_int = -1} },
            { MP_QSTR_roaming,                        MP_ARG_KW_ONLY  | MP_ARG_BOOL, {.u_bool = false} },
	};
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char user[PPP_MAX_NAME_LEN+1] = {0};
    char pass[PPP_MAX_NAME_LEN+1] = {0};
    char apn[PPP_MAX_NAME_LEN+1] = {0};
    int rx, tx, bdr;

    if (mp_obj_is_str(args[3].u_obj)) {
        snprintf(user, PPP_MAX_NAME_LEN, mp_obj_str_get_str(args[3].u_obj));
    }

	if (mp_obj_is_str(args[4].u_obj)) {
    	snprintf(pass, PPP_MAX_NAME_LEN, mp_obj_str_get_str(args[4].u_obj));
    }

	if (mp_obj_is_str(args[5].u_obj)) {
    	snprintf(apn, PPP_MAX_NAME_LEN, mp_obj_str_get_str(args[5].u_obj));
    }
	else goto exit;

    tx = args[0].u_int;
    rx = args[1].u_int;
    if ((tx < 0) || (rx < 0)) goto exit;

    bdr = args[2].u_int;

    int res = gsm_ppposInit(tx, rx, args[8].u_int, args[9].u_int, bdr, user, pass, apn, args[7].u_bool, args[6].u_bool, args[10].u_bool);

    if (res == 0) return mp_const_true;

exit:
	return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_gsm_startGSM_obj, 0, mod_gsm_startGSM);

//------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_stopGSM(size_t n_args, const mp_obj_t *args)
{
    bool wait = false;
    if (n_args > 0) wait = mp_obj_is_true(args[0]);
    if (gsm_ppposDisconnect(1, wait) == 0) return mp_const_true;
	return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_gsm_stopGSM_obj, 0, 1, mod_gsm_stopGSM);

//------------------------------------
STATIC mp_obj_t mod_gsm_stateGSM()
{
	mp_obj_t tuple[2];
	char state[20] = {'\0'};
	int gsm_state = ppposStatus(NULL, NULL, NULL);

	tuple[0] = mp_obj_new_int(gsm_state);

	if (gsm_state == ATDEV_STATEDISCONNECTED) sprintf(state, "Disconnected");
	else if (gsm_state == ATDEV_STATECONNECTED) sprintf(state, "Connected");
    else if (gsm_state == ATDEV_STATECONNECTING) sprintf(state, "Connecting");
	else if (gsm_state == ATDEV_STATEIDLE) sprintf(state, "Idle");
	else if (gsm_state == ATDEV_STATEFIRSTINIT) sprintf(state, "Not started");
	else sprintf(state, "Unknown");
	tuple[1] = mp_obj_new_str(state, strlen(state));

	return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_gsm_stateGSM_obj, mod_gsm_stateGSM);

//--------------------------------
STATIC mp_obj_t mod_gsm_ifconfig()
{
    uint32_t ip, netmask, gw;
    ppposStatus(&ip, &netmask, &gw);

    mp_obj_t tuple[3] = {
        (mp_obj_t)netutils_format_ipv4_addr((uint8_t*)&ip, NETUTILS_BIG),
        (mp_obj_t)netutils_format_ipv4_addr((uint8_t*)&netmask, NETUTILS_BIG),
        (mp_obj_t)netutils_format_ipv4_addr((uint8_t*)&gw, NETUTILS_BIG)
    };
    // Return tuple: (ip, netmask, gateway)
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_gsm_ifconfig_obj, mod_gsm_ifconfig);

//-----------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_connectGSM(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
            { MP_QSTR_connstr,  MP_ARG_OBJ,  {.u_obj = mp_const_none} },
            { MP_QSTR_fullinit, MP_ARG_BOOL, {.u_bool = false} },
            { MP_QSTR_wait,     MP_ARG_BOOL, {.u_bool = false} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_hal_wdt_reset();

    char *conn_str = NULL;
    if (args[0].u_obj != mp_const_none) {
        conn_str = (char *)mp_obj_str_get_str(args[0].u_obj);
    }
    int res = gsm_ppposConnect(conn_str, args[1].u_bool, args[2].u_bool);

    if (res == 0) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_gsm_connectGSM_obj, 0, mod_gsm_connectGSM);

//------------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_disconnectGSM(size_t n_args, const mp_obj_t *args)
{
    uint8_t end_task = false;
    bool wait = false;
    if (n_args > 0) {
        end_task = mp_obj_is_true(args[0]);
        if (n_args > 1) {
            wait = mp_obj_is_true(args[1]);
        }
    }

    if (gsm_ppposDisconnect(end_task, wait) == 0) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_gsm_disconnectGSM_obj, 0, 2, mod_gsm_disconnectGSM);

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_sendSMS(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
			{ MP_QSTR_gsmnum,  MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
			{ MP_QSTR_msg,     MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
	};
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    char *num = NULL;
    char *msg = NULL;
    int res = 0;

    if (mp_obj_is_str(args[0].u_obj)) {
        num = (char *)mp_obj_str_get_str(args[0].u_obj);
    }
    if (mp_obj_is_str(args[1].u_obj)) {
        msg = (char *)mp_obj_str_get_str(args[1].u_obj);
    }
    if ((num) && (msg)) {
    	res = smsSend(num, msg);
    }
    if (res) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_gsm_sendSMS_obj, 2, mod_gsm_sendSMS);

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_checkSMS(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
			{ MP_QSTR_sort,		MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SMS_SORT_NONE} },
			{ MP_QSTR_status,	MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SMS_LIST_ALL} },
	};
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int sort = SMS_SORT_NONE;
    int rd_status = SMS_LIST_ALL;
    if ((args[0].u_int == SMS_SORT_ASC) || (args[0].u_int == SMS_SORT_DESC)) sort = args[0].u_int;
    if ((args[1].u_int == SMS_LIST_NEW) || (args[1].u_int == SMS_LIST_OLD)) rd_status = args[1].u_int;

    mp_obj_t msg = mp_const_none;
	SMS_indexes indexes;
	int nmsg = getMessagesList(rd_status, 0, NULL, &indexes, sort);
	if (nmsg > 0) {
		mp_obj_t msgidx_tuple[nmsg];
		for (int i=0; i<nmsg; i++) {
			msgidx_tuple[i] = mp_obj_new_int(indexes.idx[i]);
		}
		msg = mp_obj_new_tuple(nmsg, msgidx_tuple);
	}
	else {
        msg = mp_obj_new_tuple(0, NULL);
	}

	return msg;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_gsm_checkSMS_obj, 0, mod_gsm_checkSMS);

//------------------------------------------------
STATIC mp_obj_t mod_gsm_deleteSMS(mp_obj_t idx_in)
{
	if (smsDelete(mp_obj_get_int(idx_in))) return mp_const_true;
	else return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_gsm_deleteSMS_obj, mod_gsm_deleteSMS);

//------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_readSMS(size_t n_args, const mp_obj_t *args)
{
    mp_obj_t msg = mp_const_none;
	SMS_Msg smsmsg;
	int idx = mp_obj_get_int(args[0]);
	int delsms = 0;
	if (n_args > 1) delsms = mp_obj_get_int(args[1]);

	int nmsg = getMessagesList(SMS_LIST_ALL, idx, &smsmsg, NULL, SMS_SORT_NONE);

	if (nmsg > 0) {
		mp_obj_t sms_tuple[7];
		sms_tuple[0] = mp_obj_new_int(smsmsg.idx);
		sms_tuple[1] = mp_obj_new_str(smsmsg.stat, strlen(smsmsg.stat));
		sms_tuple[2] = mp_obj_new_str(smsmsg.from, strlen(smsmsg.from));
		sms_tuple[3] = mp_obj_new_str(smsmsg.time, strlen(smsmsg.time));
		sms_tuple[4] = mp_obj_new_int(smsmsg.time_value);
		sms_tuple[5] = mp_obj_new_int(smsmsg.tz);

		if (smsmsg.msg) {
			sms_tuple[6] = mp_obj_new_str(smsmsg.msg, strlen(smsmsg.msg));
			vPortFree(smsmsg.msg);
		}
		else sms_tuple[6] = mp_const_none;

		msg = mp_obj_new_tuple(7, sms_tuple);

		if (delsms) smsDelete(idx);
	}

	return msg;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_gsm_readSMS_obj, 1, 2, mod_gsm_readSMS);


//-----------------------------------------------------------------
STATIC mp_obj_t mod_gsm_SMS_cb(size_t n_args, const mp_obj_t *args)
{
    if ((mp_obj_is_fun(args[0])) || (mp_obj_is_meth(args[0]))) {
		int interval = 60;
    	if (n_args > 1) interval = mp_obj_get_int(args[1]);
		if ((interval < 5) || (interval > 86400)) return mp_const_false;

		if (setSMS_cb((void *)args[0], interval*1000)) return mp_const_true;
		else return mp_const_false;
	}
	else if (args[0] == mp_const_none) {
		if (setSMS_cb(NULL, 0)) return mp_const_true;
		else return mp_const_false;
	}
	else {
		return mp_const_false;
	}
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_gsm_SMS_cb_obj, 1, 2, mod_gsm_SMS_cb);

//------------------------------------------------
STATIC mp_obj_t mod_gsm_NTP_cb(mp_obj_t cbfunc_in)
{
    if ((mp_obj_is_fun(cbfunc_in)) || (mp_obj_is_meth(cbfunc_in))) {
        if (setNTP_cb((void *)cbfunc_in)) return mp_const_true;
        else return mp_const_false;
    }
    else if (cbfunc_in == mp_const_none) {
        if (setNTP_cb(NULL)) return mp_const_true;
        else return mp_const_false;
    }
    else {
        return mp_const_false;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_gsm_NTP_cb_obj, mod_gsm_NTP_cb);

//------------------------------------------------
STATIC mp_obj_t mod_gsm_GSM_debug(mp_obj_t dbg_in)
{
    gsm_setDebug(mp_obj_is_true(dbg_in));
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_gsm_GSM_debug_obj, mod_gsm_GSM_debug);

//------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_atCmd(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    return mpy_atCmd(n_args, pos_args, kw_args, gsm_uart_num, gsm_debug, GSM_PPP_TAG);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_gsm_atCmd_obj, 1, mod_gsm_atCmd);

//-----------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_get_counters(size_t n_args, const mp_obj_t *args)
{
    uint8_t rst = 0;
    uint32_t rx=0, tx=0;
    if (n_args > 0) rst = mp_obj_is_true(args[0]);

    pppos_getRxTxCount(&rx, &tx, rst);

    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int(rx);
    tuple[1] = mp_obj_new_int(tx);

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_gsm_get_counters_obj, 0, 1, mod_gsm_get_counters);

//-------------------------------------------------------------------
STATIC mp_obj_t mod_gsm_baudrate(size_t n_args, const mp_obj_t *args)
{
    if (n_args == 0) {
        return mp_obj_new_int(gsm_uart_baudrate);
    }

    bool perm = false;
    if (n_args > 1) perm = mp_obj_is_true(args[1]);
    int baudrate = mp_obj_get_int(args[0]);
    int i = BDRATES_MAX;
    for (i=0; i<BDRATES_MAX; i++) {
        if (baudrate == bd_rates[i]) break;
    }
    if (i >= BDRATES_MAX) {
        mp_raise_ValueError("Wrong baudrate");
    }

    bool res = gsm_set_baudrate(baudrate, perm);

    if (!res) return mp_const_false;
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_gsm_baudrate_obj, 0, 2, mod_gsm_baudrate);


//=============================================================
STATIC const mp_rom_map_elem_t gsm_locals_dict_table[] = {
        { MP_ROM_QSTR(MP_QSTR_start),       MP_ROM_PTR(&mod_gsm_startGSM_obj) },
        { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&mod_gsm_stopGSM_obj) },
        { MP_ROM_QSTR(MP_QSTR_status),      MP_ROM_PTR(&mod_gsm_stateGSM_obj) },
        { MP_ROM_QSTR(MP_QSTR_ifconfig),    MP_ROM_PTR(&mod_gsm_ifconfig_obj) },
        { MP_ROM_QSTR(MP_QSTR_disconnect),  MP_ROM_PTR(&mod_gsm_disconnectGSM_obj) },
        { MP_ROM_QSTR(MP_QSTR_connect),     MP_ROM_PTR(&mod_gsm_connectGSM_obj) },
        { MP_ROM_QSTR(MP_QSTR_sendSMS),     MP_ROM_PTR(&mod_gsm_sendSMS_obj) },
        { MP_ROM_QSTR(MP_QSTR_checkSMS),    MP_ROM_PTR(&mod_gsm_checkSMS_obj) },
        { MP_ROM_QSTR(MP_QSTR_readSMS),     MP_ROM_PTR(&mod_gsm_readSMS_obj) },
        { MP_ROM_QSTR(MP_QSTR_deleteSMS),   MP_ROM_PTR(&mod_gsm_deleteSMS_obj) },
        { MP_ROM_QSTR(MP_QSTR_sms_cb),      MP_ROM_PTR(&mod_gsm_SMS_cb_obj) },
        { MP_ROM_QSTR(MP_QSTR_ntp_cb),      MP_ROM_PTR(&mod_gsm_NTP_cb_obj) },
        { MP_ROM_QSTR(MP_QSTR_debug),       MP_ROM_PTR(&mod_gsm_GSM_debug_obj) },
        { MP_ROM_QSTR(MP_QSTR_atcmd),       MP_ROM_PTR(&mod_gsm_atCmd_obj) },
        { MP_ROM_QSTR(MP_QSTR_getCounters), MP_ROM_PTR(&mod_gsm_get_counters_obj) },
        { MP_ROM_QSTR(MP_QSTR_baudrate),    MP_ROM_PTR(&mod_gsm_baudrate_obj) },

        // Constants
        { MP_ROM_QSTR(MP_QSTR_SORT_NONE),   MP_ROM_INT(SMS_SORT_NONE) },
        { MP_ROM_QSTR(MP_QSTR_SORT_ASC),    MP_ROM_INT(SMS_SORT_ASC) },
        { MP_ROM_QSTR(MP_QSTR_SORT_DESC),   MP_ROM_INT(SMS_SORT_DESC) },
        { MP_ROM_QSTR(MP_QSTR_SMS_READ),    MP_ROM_INT(SMS_LIST_OLD) },
        { MP_ROM_QSTR(MP_QSTR_SMS_UNREAD),  MP_ROM_INT(SMS_LIST_NEW) },
        { MP_ROM_QSTR(MP_QSTR_SMS_ALL),     MP_ROM_INT(SMS_LIST_ALL) },
};
STATIC MP_DEFINE_CONST_DICT(gsm_locals_dict, gsm_locals_dict_table);

//==============================
const mp_obj_type_t gsm_type = {
    { &mp_type_type },
    .name = MP_QSTR_gsm,
    //.print = gsm_print,
    //.make_new = gsm_make_new,
    .locals_dict = (mp_obj_dict_t*)&gsm_locals_dict,
};

#endif

