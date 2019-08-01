/*
 * This file is part of the MicroPython ESP32 project, https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo
 *
 * The MIT License (MIT)
 *
 * Based on original 'machine-timer.c' from https://github.com/micropython/micropython-esp32
 *   completely rewritten and many new functions and features added
 *
 * Copyright (c) 2017 Boris Lovosevic (https://github.com/loboris)
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

#include <stdint.h>
#include <stdio.h>

#include "py/runtime.h"
#include "modmachine.h"

#define TIMER_RUNNING	1
#define TIMER_PAUSED	0

#define TIMER_TYPE_ONESHOT	0
#define TIMER_TYPE_PERIODIC	1
#define TIMER_TYPE_CHRONO	2
#define TIMER_TYPE_MAX		3

#define TIMER_MAX_TIMERS    12
#define TIMER_TASK_EXIT     0xA5

typedef struct _machine_timer_obj_t {
    mp_obj_base_t   base;
    uint8_t         id;         // timer number (0~11)
    uint8_t         state;      // current timer state
    uint8_t         type;       // timer type
    int8_t          pin;        // timer pin, if not used: -1
    int8_t          pin_gpio;
    int8_t          pin_mode;
    int8_t          pin_pull;
    uint8_t         reserved;
    handle_t        handle;     // hw timer handle
    bool            repeat;     // true for periodic type
    uint64_t        period;     // timer period in us
    int64_t         remain;     // remaining us until timer event
    uint32_t        interval;   // hw timer interval in nanoseconds
    uint32_t        loadval;    // hw timer load register value
    double          resolution; // hw timer resolution in nanoseconds
    uint64_t        event_num;  // number of timer events
    uint64_t        cb_num;     // number of scheduled timer callbacks
    mp_obj_t        callback;   // timer callback function
} machine_timer_obj_t;

const mp_obj_type_t machine_timer_type;

machine_timer_obj_t *mpy_timers_used[TIMER_MAX_TIMERS] = {NULL};

TaskHandle_t timer_task_handle = NULL;

//----------------------------------------
static void timer_task(void *pvParameters)
{
    TaskHandle_t *task_handle = (TaskHandle_t)pvParameters;
    // if the task uses some MicroPython functions, we have to save
    // MicroPython state in local storage pointers
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, pvTaskGetThreadLocalStoragePointer(task_handle, THREAD_LSP_STATE));
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, pvTaskGetThreadLocalStoragePointer(task_handle, THREAD_LSP_ARGS));

    // on entry, the pin interrupt is still disabled
    machine_timer_obj_t *self = NULL;
    uint64_t notify_val = 0;
    int notify_res = 0;

    while (1) {
        // Check notification
        notify_val = 0;
        notify_res = xTaskNotifyWait(0, ULONG_MAX, &notify_val, 1000 / portTICK_RATE_MS);
        if (notify_res != pdPASS) continue;
        if (notify_val == TIMER_TASK_EXIT) break; // Terminate task requested

        self = (machine_timer_obj_t *)(notify_val & 0xffffffff);
        if ((self->type != TIMER_TYPE_CHRONO) && (self->pin >= 0)) {
            gpio_set_pin_value(gpiohs_handle, self->pin_gpio, (self->event_num & 1));
        }
        // schedule timer event
        if ((self->callback) && (mp_sched_schedule(self->callback, self))) self->cb_num++;

    } // task's main loop

    // Terminate debounce task
    timer_task_handle = NULL;
    vTaskDelete(NULL);
}


//----------------------------------------------------------------------------------------------
STATIC void machine_timer_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    machine_timer_obj_t *self = self_in;

    mp_printf(print, "Timer(%d) ", self->id);

    if ((self->type >= TIMER_TYPE_MAX) || (self->handle == 0)) {
        mp_printf(print, "Not initialized");
        return;
    }

    char stype[16];
    if (self->type == TIMER_TYPE_PERIODIC) sprintf(stype, "Periodic");
    else if (self->type == TIMER_TYPE_ONESHOT) sprintf(stype, "One shot");
    else if (self->type == TIMER_TYPE_CHRONO) sprintf(stype, "Chrono");
    else sprintf(stype, "Unknown");

    if (self->type == TIMER_TYPE_CHRONO) {
    	mp_printf(print, "Period: 1 us; ");
    }
    else {
    	mp_printf(print, "Period: %d us; Resolution: %0.3f; Type: %s; Running: %s\n",
    	        self->period, self->resolution, stype, (self->state == TIMER_RUNNING) ? "yes" : "no");
    }
    if (self->type != TIMER_TYPE_CHRONO) {
        mp_printf(print, "         Events: %lu; Callbacks: %lu; Missed: %lu\n",
                self->event_num, self->cb_num, self->event_num - self->cb_num);
    }
    if (self->pin >= 0) {
        mp_printf(print, "         Pin output on gpio %d", self->pin);
    }
}

//-------------------------------------------
STATIC void machine_timer_isr(void *userdata)
{
    machine_timer_obj_t *self = (machine_timer_obj_t *)userdata;

    bool notify = false;
    self->event_num++;
    if (self->type == TIMER_TYPE_CHRONO) return;

    if (self->period > 1000000) {
        // Period > 1 second
        self->event_num--;
        self->remain -= (self->interval / 1000);
        if (self->remain <= 0) {
            self->event_num++;
            if (!self->repeat) {
                timer_set_enable(self->handle, false);
                self->state = TIMER_PAUSED;
            }
            else self->remain = self->period;
            notify = true;
        }
    }
    else {
        if (!self->repeat) {
            timer_set_enable(self->handle, false);
            self->state = TIMER_PAUSED;
        }
        notify = true;
    }
    if ((notify) && (timer_task_handle)) {
        BaseType_t HPTaskAwoken = pdFALSE;
        if (xTaskNotifyFromISR(timer_task_handle, (uintptr_t)self, eSetValueWithoutOverwrite, &HPTaskAwoken) == pdPASS) {
            if (HPTaskAwoken == pdTRUE) vPortYieldFromISR();
        }
    }
}

//-----------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_timer_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 1, false);
    machine_timer_obj_t *self = m_new_obj(machine_timer_obj_t);

    self->base.type = &machine_timer_type;

    self->callback = NULL;
    self->handle = 0;
    self->event_num = 0;
    self->cb_num = 0;
    self->pin = -1;
	self->state = TIMER_PAUSED;
	self->type = TIMER_TYPE_MAX;

    int tmr = mp_obj_get_int(args[0]);
    if ((tmr < 0) || (tmr > 11)) {
    	mp_raise_ValueError("Only timers 0~11 can be used.");
    }
    if (mpy_timers_used[tmr] != NULL) {
        mp_raise_ValueError("Timer already used used.");
    }
    self->id = tmr;
    mpy_timers_used[tmr] = self;

    if (timer_task_handle == NULL) {
        BaseType_t res = xTaskCreate(
                timer_task,                             // function entry
                "Timer_task",                           // task name
                configMINIMAL_STACK_SIZE,               // stack_deepth
                (void *)xTaskGetCurrentTaskHandle(),    // function argument
                MICROPY_TASK_PRIORITY+1,                // task priority
                &timer_task_handle);                    // task handle
        if (res != pdPASS) timer_task_handle = NULL;
    }

    return self;
}

//-----------------------------------------------------------------
static void timer_set_period(machine_timer_obj_t *self, bool start)
{
    uint8_t old_state = self->state;
    timer_set_enable(self->handle, false);
    self->state = TIMER_PAUSED;

    if (self->type == TIMER_TYPE_CHRONO) {
        // Set interval to 1 us
        timer_set_interval(self->handle, 1000);
    }
    else {
        if (self->period > 1000000) self->interval = 1000000000;
        else self->interval = self->period * 1000;
        self->remain = self->period;
        timer_set_interval(self->handle, self->interval);
    }
    timer_get_value(self->handle, &self->resolution, &self->loadval);

    if ((start) || (old_state == TIMER_RUNNING)) {
        timer_set_enable(self->handle, true);
        self->state = TIMER_RUNNING;
    }
}

//---------------------------------------------------------
static void machine_timer_enable(machine_timer_obj_t *self)
{
    char timer_dev[16];

    sprintf(timer_dev, "/dev/timer%d", self->id);
    self->handle = io_open(timer_dev);
    if (self->handle == 0) {
        mp_raise_ValueError("Error opening timed device");
    }

    timer_set_on_tick(self->handle, machine_timer_isr, (void *)self);

    timer_set_period(self, true);
}

//----------------------------------------------------------
static void machine_timer_disable(machine_timer_obj_t *self)
{
    if (self->handle) {
        timer_set_enable(self->handle, false);
        io_close(self->handle);
    }
    if (self->pin >= 0) {
        gpiohs_set_free(self->pin_gpio);
        mp_used_pins[self->pin].func = GPIO_FUNC_NONE;
        mp_used_pins[self->pin].gpio = 0;
        mp_used_pins[self->pin].fpioa_func = FUNC_DEBUG31;
        self->pin_gpio = -1;
        self->pin = -1;
    }
    mpy_timers_used[self->id] = NULL;
    self->id = -1;
    self->callback = NULL;
    self->handle = 0;
    self->event_num = 0;
    self->cb_num = 0;
    self->state = TIMER_PAUSED;
    self->type = TIMER_TYPE_MAX;

    int i = TIMER_MAX_TIMERS;
    for (i=0; i< TIMER_MAX_TIMERS; i++) {
        if (mpy_timers_used[i]) break;
    }
    // Terminate Timer task if no timers running
    if (i >= TIMER_MAX_TIMERS) xTaskNotify(timer_task_handle, TIMER_TASK_EXIT, eSetValueWithOverwrite);
}

//---------------------------------------------------------------------
static void set_timer_pin(machine_timer_obj_t *self, int8_t wanted_pin)
{
    if ((wanted_pin < 0) || (wanted_pin >= FPIOA_NUM_IO)) {
        mp_raise_ValueError("invalid pin");
    }
    if (mp_used_pins[wanted_pin].func != GPIO_FUNC_NONE) {
        mp_raise_ValueError(gpiohs_funcs_in_use[mp_used_pins[wanted_pin].func]);
    }
    int pio_num = gpiohs_get_free();
    if (pio_num < 0) {
        mp_raise_ValueError("no free gpiohs handles");
    }

    self->pin = wanted_pin;
    self->pin_gpio = pio_num;
    self->pin_mode = GPIO_DM_OUTPUT;
    self->pin_pull = GPIO_DM_INPUT;
    // configure the pin for gpio
    if (fpioa_set_function(self->pin, FUNC_GPIOHS0 + self->pin_gpio) < 0) {
        gpiohs_set_free(self->pin_gpio);
        mp_raise_ValueError("error initializing gpio");
    }

    if (self->pin_mode == GPIO_DM_OUTPUT) gpio_set_drive_mode(gpiohs_handle, self->pin_gpio, self->pin_mode);
    else gpio_set_drive_mode(gpiohs_handle, self->pin_gpio, self->pin_pull);

    mp_used_pins[self->pin].func = GPIO_FUNC_TIMER;
    mp_used_pins[self->pin].usedas = (self->pin_mode == GPIO_DM_OUTPUT) ? GPIO_USEDAS_OUTPUT: GPIO_USEDAS_INPUT;
    mp_used_pins[self->pin].gpio = self->pin_gpio;
    mp_used_pins[self->pin].fpioa_func = FUNC_GPIOHS0 + self->pin_gpio;

    // set initial value
    if (self->pin_mode == GPIO_DM_OUTPUT) {
        gpio_set_pin_value(gpiohs_handle, self->pin_gpio, 0);
    }

    // Disable gpio interrupt
    gpio_set_pin_edge(gpiohs_handle, self->pin_gpio, GPIO_PE_NONE);
}

//---------------------------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_timer_init_helper(machine_timer_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_period,       MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1000000} },
        { MP_QSTR_mode,         MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = TIMER_TYPE_PERIODIC} },
        { MP_QSTR_callback,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_pin,          MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    machine_timer_disable(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if ((args[1].u_int < 0) || (args[1].u_int > TIMER_TYPE_MAX)) {
        mp_raise_ValueError("Wrong timer type");
    }
    self->handle = 0;
    self->event_num = 0;
    self->cb_num = 0;
    self->pin = -1;

    self->type = args[1].u_int & 3;

    if (self->type == TIMER_TYPE_CHRONO) {
        // Chrono Timer uses an 1 MHz clock, no callback
        // set period in us
        self->period = 1;
        self->repeat = 0;
        self->callback = NULL;
    }
    else {
        if ((args[0].u_int < 1) || (args[0].u_int > 86400000000)) {
            mp_raise_ValueError("Period out of range (1 ~ 86400000000 us)");
        }
        self->period = args[0].u_int;
        self->repeat = args[1].u_int & 1;

        // Set the timer callback if given
        if (args[2].u_obj != mp_const_none) {
            if ((mp_obj_is_fun(args[2].u_obj)) || (mp_obj_is_meth(args[2].u_obj)))
                self->callback = args[2].u_obj;
            else {
                mp_raise_ValueError("Callback function expected");
            }
        }

        // set the timer pin if used
        if ((args[3].u_int >= 0) && (args[3].u_int < 34)) {
            set_timer_pin(self, args[3].u_int);
        }
    }
    // enable and start the timer
    machine_timer_enable(self);

    if (self->type != TIMER_TYPE_CHRONO) self->state = TIMER_RUNNING;
    else self->state = TIMER_PAUSED;

    return mp_const_none;
}

//-------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_timer_init(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    return machine_timer_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_timer_init_obj, 1, machine_timer_init);

//----------------------------------------------------
STATIC mp_obj_t machine_timer_deinit(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;
    machine_timer_disable(self);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_deinit_obj, machine_timer_deinit);

//---------------------------------------------------
STATIC mp_obj_t machine_timer_value(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;
    uint64_t result;

    if (self->type == TIMER_TYPE_CHRONO) result = self->event_num;
    else {
        uint32_t val = timer_get_value(self->handle, NULL, NULL);
        double tmns = (double)(self->loadval - val) * self->resolution;
        result = (uint64_t)tmns / 1000;
        if (self->period > 1000000) {
            result += self->remain;
        }
    }
    return mp_obj_new_int(result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_value_obj, machine_timer_value);

//---------------------------------------------------
STATIC mp_obj_t machine_timer_pause(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;

    timer_set_enable(self->handle, false);
    self->state = TIMER_PAUSED;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_pause_obj, machine_timer_pause);

//----------------------------------------------------
STATIC mp_obj_t machine_timer_resume(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;

    timer_set_enable(self->handle, true);
    self->state = TIMER_RUNNING;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_resume_obj, machine_timer_resume);

//---------------------------------------------------
STATIC mp_obj_t machine_timer_start(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;

    timer_set_enable(self->handle, false);
    self->event_num = 0;
    self->cb_num = 0;
    timer_set_period(self, true);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_start_obj, machine_timer_start);

//---------------------------------------------------------------------
STATIC mp_obj_t machine_timer_shot(size_t n_args, const mp_obj_t *args)
{
    machine_timer_obj_t *self = args[0];

    if (self->type != TIMER_TYPE_ONESHOT) {
    	mp_raise_ValueError("Timer is not one_shot timer.");
    }

    if (n_args > 1) {
        // Pause timer
        timer_set_enable(self->handle, false);
        self->state = TIMER_PAUSED;
        // Set new period
        int64_t period = mp_obj_get_int(args[1]);
        if ((period < 1) || (period > 86400000000)) {
            mp_raise_ValueError("Period out of range (1 ~ 86400000000 us)");
        }
        self->period = period;
    }

    timer_set_period(self, true);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_shot_obj, 1, 2, machine_timer_shot);

//------------------------------------------------
STATIC mp_obj_t machine_timer_id(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;

    return MP_OBJ_NEW_SMALL_INT(self->id);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_id_obj, machine_timer_id);

//----------------------------------------------------
STATIC mp_obj_t machine_timer_events(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;
    mp_obj_t tuple[2];
    tuple[0] = mp_obj_new_int_from_ull(self->event_num);
    tuple[1] = mp_obj_new_int_from_ull(self->cb_num);

    return mp_obj_new_tuple(2, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_events_obj, machine_timer_events);

//-------------------------------------------------------
STATIC mp_obj_t machine_timer_isrunning(mp_obj_t self_in)
{
    machine_timer_obj_t *self = self_in;

    if (self->state == TIMER_RUNNING) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_isrunning_obj, machine_timer_isrunning);

//-----------------------------------------------------------------------
STATIC mp_obj_t machine_timer_period(size_t n_args, const mp_obj_t *args)
{
    machine_timer_obj_t *self = args[0];

    if ((n_args > 1) && (self->type != TIMER_TYPE_CHRONO)) {
        int64_t period;
		uint8_t old_state = self->state;
		// Pause timer
        timer_set_enable(self->handle, false);
		self->state = TIMER_PAUSED;

		// Set new period
		period = mp_obj_get_int(args[1]);
        if ((period < 1) || (period > 86400000000)) {
            timer_set_period(self, (old_state == TIMER_RUNNING));
            mp_raise_ValueError("Period out of range (1 ~ 86400000000 us)");
        }
        self->period = period;
        timer_set_period(self, (old_state == TIMER_RUNNING));
    }

    return mp_obj_new_int(self->period);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_period_obj, 1, 2, machine_timer_period);

//-------------------------------------------------------------------------
STATIC mp_obj_t machine_timer_callback(size_t n_args, const mp_obj_t *args)
{
    machine_timer_obj_t *self = args[0];

    if (n_args == 1) {
    	if (self->callback == NULL) return mp_const_false;
    	return mp_const_true;
    }

    if ((mp_obj_is_fun(args[1])) || (mp_obj_is_meth(args[1]))) {
		// Set the new callback
		self->callback = args[1];
    }
    else if (args[1] == mp_const_none) self->callback = NULL;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_callback_obj, 1, 2, machine_timer_callback);

//==============================================================
STATIC const mp_map_elem_t machine_timer_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__),		(mp_obj_t)&machine_timer_deinit_obj },
    { MP_ROM_QSTR(MP_QSTR_deinit),		(mp_obj_t)&machine_timer_deinit_obj },
    { MP_ROM_QSTR(MP_QSTR_init),		(mp_obj_t)&machine_timer_init_obj },
    { MP_ROM_QSTR(MP_QSTR_value),		(mp_obj_t)&machine_timer_value_obj },
    { MP_ROM_QSTR(MP_QSTR_events),		(mp_obj_t)&machine_timer_events_obj },
    { MP_ROM_QSTR(MP_QSTR_reshoot),		(mp_obj_t)&machine_timer_shot_obj },
    { MP_ROM_QSTR(MP_QSTR_start),		(mp_obj_t)&machine_timer_start_obj },
    { MP_ROM_QSTR(MP_QSTR_stop),		(mp_obj_t)&machine_timer_pause_obj },
    { MP_ROM_QSTR(MP_QSTR_pause),		(mp_obj_t)&machine_timer_pause_obj },
    { MP_ROM_QSTR(MP_QSTR_resume),		(mp_obj_t)&machine_timer_resume_obj },
    { MP_ROM_QSTR(MP_QSTR_timernum),	(mp_obj_t)&machine_timer_id_obj },
    { MP_ROM_QSTR(MP_QSTR_period),		(mp_obj_t)&machine_timer_period_obj },
    { MP_ROM_QSTR(MP_QSTR_callback),	(mp_obj_t)&machine_timer_callback_obj },
    { MP_ROM_QSTR(MP_QSTR_isrunning),	(mp_obj_t)&machine_timer_isrunning_obj },

	{ MP_ROM_QSTR(MP_QSTR_ONE_SHOT),	MP_ROM_INT(TIMER_TYPE_ONESHOT) },
    { MP_ROM_QSTR(MP_QSTR_PERIODIC),	MP_ROM_INT(TIMER_TYPE_PERIODIC) },
    { MP_ROM_QSTR(MP_QSTR_CHRONO),		MP_ROM_INT(TIMER_TYPE_CHRONO) },
};
STATIC MP_DEFINE_CONST_DICT(machine_timer_locals_dict, machine_timer_locals_dict_table);

//========================================
const mp_obj_type_t machine_timer_type = {
    { &mp_type_type },
    .name = MP_QSTR_Timer,
    .print = machine_timer_print,
    .make_new = machine_timer_make_new,
    .locals_dict = (mp_obj_t)&machine_timer_locals_dict,
};

