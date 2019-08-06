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

#include <stdio.h>
#include <string.h>

#include "py/nlr.h"
#include "py/runtime.h"
#include "modmachine.h"

// Forward dec'l
extern const mp_obj_type_t machine_pwm_type;

//--------------------------------------------
static void check_pwm(machine_pwm_obj_t *self)
{
    if (!self->handle) {
        mp_raise_ValueError("PWM not initialized.");
    }
}

//--------------------------------------------------------------------------------------------
STATIC void machine_pwm_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "PWM(%u [%d:%d]) ", self->channel, self->channel / 4, self->channel % 4);
    if (self->handle) {
        char sres[16];
        double res = 1.0 / (double)self->periods;
        if (res >= 0.001) sprintf(sres,"%0.3f", res);
        else sprintf(sres, "<0.001");

        mp_printf(print, "pin=%d%s, freq=%0.3f Hz, duty=%0.3f (step=%s), active=%s)",
                self->pin, (self->invert) ? " (inverted)" : "", self->freq,
                self->dperc, sres, (self->active) ? "True" : "False");
    }
    else mp_printf(print, ", not initialized");
}

//---------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_pwm_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    int channel = mp_obj_get_int(args[0]);
    if ((channel < 0) || (channel >= TIMER_MAX_TIMERS)) {
        mp_raise_ValueError("Only PWM 0~11 can be used.");
    }
    if (mpy_timers_used[channel] != NULL) {
        // check if the timer group is used by Timer
        uint8_t tg = channel / 4;
        mp_obj_type_t *obj;
        for (int i=tg; i<(tg+4); i++) {
            obj = (mp_obj_type_t *)mpy_timers_used[i];
            if (mp_obj_is_type(obj, &machine_timer_type)) {
                mp_raise_ValueError("Timer group used used by Timer.");
            }
        }
        mp_raise_ValueError("Channel already used used by another PWM.");
    }

    // create PWM object for requested channel
    machine_pwm_obj_t *self = m_new_obj(machine_pwm_obj_t);
    self->base.type = &machine_pwm_type;
    self->pin = -1;
    self->active = 0;
    self->handle = 0;
    self->dperc = 0.5;
    self->freq = 1000;
    self->channel = channel;
    mpy_timers_used[channel] = (void *)self;

    return MP_OBJ_FROM_PTR(self);
}

//----------------------------------------------------------
STATIC mp_obj_t machine_pwm_disable(machine_pwm_obj_t *self)
{
    if (self->handle) {
        pwm_set_enable(self->handle, (self->channel % 4), false);
        io_close(self->handle);
    }
    if (self->pin >= 0) {
        mp_used_pins[self->pin].func = GPIO_FUNC_NONE;
        mp_used_pins[self->pin].usedas = GPIO_USEDAS_NONE;
        mp_used_pins[self->pin].gpio = -1;
        mp_used_pins[self->pin].fpioa_func = FUNC_DEBUG31;
    }

    mpy_timers_used[self->channel] = NULL;
    self->pin = -1;
    self->active = false;
    self->handle = 0;
    self->dperc = 0.5;
    self->freq = 1000;
    self->invert = false;

    return mp_const_none;
}

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t  machine_pwm_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_pin, ARG_invert, ARG_freq, ARG_duty, ARG_start };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin,      MP_ARG_REQUIRED | MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_invert,                     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_freq,                       MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_duty,                       MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_start,                      MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };

    machine_pwm_obj_t *self = pos_args[0];

    machine_pwm_disable(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // prepare PWM pin
    int wanted_pin = args[0].u_int;
    if ((wanted_pin < 0) || (wanted_pin >= FPIOA_NUM_IO)) {
        mp_raise_ValueError("invalid pin");
    }
    if (mp_used_pins[wanted_pin].func != GPIO_FUNC_NONE) {
        mp_raise_ValueError(gpiohs_funcs_in_use[mp_used_pins[wanted_pin].func]);
    }
    // get frequency
    double freq = 1000.0;
    if (args[ARG_freq].u_obj != mp_const_none) {
        freq = mp_obj_get_float(args[ARG_freq].u_obj);
        if ((freq < 0.25) || (freq >= 100000000.0)) {
            mp_raise_ValueError("Frequency out of range (0.25 Hz ~ 100.0 MHz)");
        }
    }
    // get duty cycle percentage
    double dperc = 0.5;
    if (args[ARG_duty].u_obj != mp_const_none) {
        dperc = mp_obj_get_float(args[ARG_duty].u_obj);
        if ((dperc < 0) || (dperc > 1.0)) {
            mp_raise_ValueError("Duty out of range (0.0 ~ 1.0)");
        }
    }

    // open pwm device
    char pwm_dev[16] = {'\0'};
    sprintf(pwm_dev, "/dev/pwm%d", self->channel / 4);
    self->handle = io_open(pwm_dev);
    if (self->handle == 0) {
        mp_raise_ValueError("Error opening pwm device");
    }

    self->pin = wanted_pin;
    // configure the pin
    if (fpioa_set_function(self->pin, FUNC_TIMER0_TOGGLE1 + self->channel) < 0) {
        io_close(self->handle);
        self->handle = 0;
        mp_raise_ValueError("error initializing pwm pin");
    }

    self->invert = args[ARG_invert].u_bool;
    if (self->invert) {
        fpioa_io_config_t cfg = fpioa->io[self->pin];
        // Set inverted pin
        cfg.do_inv = 1;
        // Atomic write register
        fpioa->io[self->pin] = cfg;
    }

    // mark the pin as used by PWM
    mp_used_pins[self->pin].func = GPIO_FUNC_PWM;
    mp_used_pins[self->pin].usedas = GPIO_USEDAS_OUTPUT;
    mp_used_pins[self->pin].gpio = -1;
    mp_used_pins[self->pin].fpioa_func = FUNC_TIMER0_TOGGLE1 + self->channel;

    // set the frequency and duty cycle
    freq = pwm_set_frequency(self->handle, freq);
    if (freq == 0.0) {
        machine_pwm_disable(self);
        mp_raise_ValueError("Error setting frequency");
    }
    self->freq = freq;
    self->dperc = pwm_set_active_duty_cycle_percentage(self->handle, (self->channel % 4), dperc, &self->perc, &self->periods);

    if (args[ARG_start].u_bool) {
        pwm_set_enable(self->handle, (self->channel % 4), true);
        self->active = true;
    }

    mpy_timers_used[self->channel] = (void *)self;;

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_init_obj, 1, machine_pwm_init);

//--------------------------------------------------
STATIC mp_obj_t machine_pwm_deinit(mp_obj_t self_in)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    machine_pwm_disable(self);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pwm_deinit_obj, machine_pwm_deinit);

//-------------------------------------------------------------------
STATIC mp_obj_t machine_pwm_freq(size_t n_args, const mp_obj_t *args)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    check_pwm(self);

    if (n_args > 1) {
        // set the frequency
        double new_freq = mp_obj_get_float(args[1]);
        if (new_freq != self->freq) {
            if ((new_freq < 0.25) || (new_freq >= 100000000.0)) {
                mp_raise_ValueError("Frequency out of range (0.25 Hz ~ 100.0 MHz)");
            }
            new_freq = pwm_set_frequency(self->handle, new_freq);
            if (new_freq == 0.0) {
                mp_raise_ValueError("Error setting frequency");
            }
            self->freq = new_freq;
            // Activate the new frequency
            self->dperc = pwm_set_active_duty_cycle_percentage(self->handle, (self->channel % 4), self->dperc, &self->perc, &self->periods);
        }
    }
    return mp_obj_new_float(self->freq);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_freq_obj, 1, 2, machine_pwm_freq);

//-------------------------------------------------------------------
STATIC mp_obj_t machine_pwm_duty(size_t n_args, const mp_obj_t *args)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    check_pwm(self);

    if (n_args > 1) {
        // set the new duty cycle
        float dperc = mp_obj_get_float(args[1]);
        if ((dperc < 0.001) || (dperc > 1.0)) {
            mp_raise_ValueError("Duty cycle out of range (0.001 ~ 1.0)");
        }
        self->dperc = pwm_set_active_duty_cycle_percentage(self->handle, (self->channel % 4), dperc, &self->perc, &self->periods);
    }
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_float(self->dperc);
    tuple[1] = mp_obj_new_float(1.0 / (double)self->periods);

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_duty_obj, 1, 2, machine_pwm_duty);

//-------------------------------------------------
STATIC mp_obj_t machine_pwm_pause(mp_obj_t self_in)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_pwm(self);

    pwm_set_enable(self->handle, (self->channel % 4), false);
    self->active = false;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pwm_pause_obj, machine_pwm_pause);

//--------------------------------------------------
STATIC mp_obj_t machine_pwm_resume(mp_obj_t self_in)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_pwm(self);

    pwm_set_enable(self->handle, (self->channel % 4), true);
    self->active = true;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pwm_resume_obj, machine_pwm_resume);

//-------------------------------------------------------------------------------------------
static int group_enable(machine_pwm_obj_t *self, bool enable, double delay_perc, int mask_in)
{
    if (((delay_perc > 0.0) && (delay_perc < 0.01)) || (delay_perc > 1.0)) {
        mp_raise_ValueError("Delay percentage out of range (0.01 ~ 1.0)");
    }
    uint32_t mask = 0;
    mp_obj_type_t *obj;
    machine_pwm_obj_t *pwm_obj;

    for (int i=0; i<4; i++) {
        int idx = i + (self->channel / 4);
        if (mpy_timers_used[idx]) {
            obj = (mp_obj_type_t *)mpy_timers_used[idx];
            if (mp_obj_is_type(obj, &machine_pwm_type)) {
                pwm_obj = (machine_pwm_obj_t *)obj;
                if (mask_in & (1 << i)) {
                    mask |= (1 << i);
                    pwm_obj->active = enable;
                }
            }
        }
    }
    if (mask) mask = pwm_set_enable_multi(self->handle, mask, enable, delay_perc);
    return mask;
}

//--------------------------------------------------------------------------
STATIC mp_obj_t machine_pwm_enablegroup(size_t n_args, const mp_obj_t *args)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    check_pwm(self);

    int mask_in = 15;
    if (n_args > 1) mask_in = mp_obj_get_int(args[1]) & 15;
    int mask = group_enable(self, true, 0.0, mask_in);

    return mp_obj_new_int(mask);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_enablegroup_obj, 1, 2, machine_pwm_enablegroup);

//--------------------------------------------------------------------------
STATIC mp_obj_t machine_pwm_disablegroup(size_t n_args, const mp_obj_t *args)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    check_pwm(self);

    int mask_in = 15;
    if (n_args > 1) mask_in = mp_obj_get_int(args[1]) & 15;
    int mask = group_enable(self, false, 0.0, mask_in);

    return mp_obj_new_int(mask);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_disablegroup_obj, 1, 2, machine_pwm_disablegroup);

//--------------------------------------------------------------------------------
STATIC mp_obj_t machine_pwm_enablegroup_delay(size_t n_args, const mp_obj_t *args)
{
    machine_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    check_pwm(self);

    int mask_in = 15;
    if (n_args > 2) mask_in = mp_obj_get_int(args[2]) & 15;
    int mask = group_enable(self, true, mp_obj_get_float(args[1]), mask_in);

    return mp_obj_new_int(mask);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_enablegroup_delay_obj, 2, 3, machine_pwm_enablegroup_delay);

//--------------------------------
STATIC mp_obj_t machine_pwm_list()
{
    machine_pwm_obj_t *obj;
    for (int i=0; i<TIMER_MAX_TIMERS; i++) {
        if (mpy_timers_used[i]) {
            obj = (machine_pwm_obj_t *)mpy_timers_used[i];
            if (mp_obj_is_type(obj, &machine_pwm_type)) {
                machine_pwm_print(&mp_plat_print, obj, 0);
                mp_printf(&mp_plat_print, "\n");
            }
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_pwm_list_obj, machine_pwm_list);


//==============================================================
STATIC const mp_rom_map_elem_t machine_pwm_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&machine_pwm_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),      MP_ROM_PTR(&machine_pwm_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_freq),        MP_ROM_PTR(&machine_pwm_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_duty),        MP_ROM_PTR(&machine_pwm_duty_obj) },
    { MP_ROM_QSTR(MP_QSTR_pause),       MP_ROM_PTR(&machine_pwm_pause_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&machine_pwm_pause_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume),      MP_ROM_PTR(&machine_pwm_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),       MP_ROM_PTR(&machine_pwm_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_group), MP_ROM_PTR(&machine_pwm_enablegroup_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_group),  MP_ROM_PTR(&machine_pwm_disablegroup_obj) },
    { MP_ROM_QSTR(MP_QSTR_sync_group),  MP_ROM_PTR(&machine_pwm_enablegroup_delay_obj) },
    { MP_ROM_QSTR(MP_QSTR_list),        MP_ROM_PTR(&machine_pwm_list_obj) },
};
STATIC MP_DEFINE_CONST_DICT(machine_pwm_locals_dict, machine_pwm_locals_dict_table);

//======================================
const mp_obj_type_t machine_pwm_type = {
    { &mp_type_type },
    .name = MP_QSTR_PWM,
    .print = machine_pwm_print,
    .make_new = machine_pwm_make_new,
    .locals_dict = (mp_obj_dict_t*)&machine_pwm_locals_dict,
};
