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

#if MICROPY_PY_USE_NETTWORK

#include "at_util.h"
#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "py/mpprint.h"


//--------------------------------------------
STATIC mp_obj_t mp_module_network_initialize()
{
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_module_network_initialize_obj, mp_module_network_initialize);


#if MICROPY_PY_USE_WIFI
extern const mp_obj_type_t wifi_type;
#endif

#if MICROPY_PY_USE_GSM
extern const mp_obj_type_t gsm_type;
#endif

#if MICROPY_PY_USE_MQTT
extern const mp_obj_type_t mqtt_type;
#endif

#if MICROPY_PY_USE_REQUESTS
extern const mp_obj_type_t requests_type;
#endif


//---------------------------------------
STATIC mp_obj_t mod_network_wifi_active()
{
    #if MICROPY_PY_USE_WIFI
    return (wifiStatus() == ATDEV_STATEIDLE) ? mp_const_true : mp_const_false;
    #else
    return mp_const_false;
    #endif
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_network_wifi_active_obj, mod_network_wifi_active);

//--------------------------------------
STATIC mp_obj_t mod_network_gsm_active()
{
    #if MICROPY_PY_USE_GSM
    int gsm_state = ppposStatus(NULL, NULL, NULL);
    return ((gsm_state == ATDEV_STATEIDLE) || (gsm_state == ATDEV_STATECONNECTED)) ? mp_const_true : mp_const_false;
    #else
    return mp_const_false;
    #endif
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_network_gsm_active_obj, mod_network_gsm_active);

//-----------------------------------------
STATIC mp_obj_t mod_network_gsm_connected()
{
    #if MICROPY_PY_USE_GSM
    int gsm_state = ppposStatus(NULL, NULL, NULL);
    return (gsm_state == ATDEV_STATECONNECTED) ? mp_const_true : mp_const_false;
    #else
    return mp_const_false;
    #endif
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_network_gsm_connected_obj, mod_network_gsm_connected);

//==============================================================
STATIC const mp_map_elem_t mp_module_network_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_network) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___init__),    (mp_obj_t)&mp_module_network_initialize_obj },

    { MP_ROM_QSTR(MP_QSTR_wifi_active),     MP_ROM_PTR(&mod_network_wifi_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_gsm_active),      MP_ROM_PTR(&mod_network_gsm_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_gsm_connected),   MP_ROM_PTR(&mod_network_gsm_connected_obj) },

    #if MICROPY_PY_USE_WIFI
    { MP_ROM_QSTR(MP_QSTR_wifi),            MP_ROM_PTR(&wifi_type) },
    #endif

    #if MICROPY_PY_USE_GSM
    { MP_ROM_QSTR(MP_QSTR_gsm),             MP_ROM_PTR(&gsm_type) },
    #endif

    #if MICROPY_PY_USE_MQTT
    { MP_ROM_QSTR(MP_QSTR_mqtt),            MP_ROM_PTR(&mqtt_type) },
    #endif

    #if MICROPY_PY_USE_REQUESTS
    { MP_ROM_QSTR(MP_QSTR_requests),        MP_ROM_PTR(&requests_type) },
    #endif
};

//===========================
STATIC MP_DEFINE_CONST_DICT (
        mp_module_network_globals,
        mp_module_network_globals_table
);

const mp_obj_module_t mp_module_network = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_network_globals,
};

#endif
