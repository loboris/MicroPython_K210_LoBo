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
#include <sys/time.h>
#include "portmacro.h"

#include "py/runtime.h"
#include "extmod/virtpin.h"
#include "modmachine.h"
#include "mphalport.h"

#define MP_OBJ_IS_METH(o) (MP_OBJ_IS_OBJ(o) && (((mp_obj_base_t*)MP_OBJ_TO_PTR(o))->type->name == MP_QSTR_bound_method))

// ---------------------------------------------------------------
// Each Pin have its own debounce task which runs in high priority
// ---------------------------------------------------------------
//--------------------------------------------
static void debounce_task(void *pvParameters)
{
    // on entry, the pin interrupt is still disabled
    machine_pin_obj_t *self = (machine_pin_obj_t *)pvParameters;
    uint64_t notify_val = 0;
    int notify_res = 0;
    bool irq_passed;
    uint64_t start_time, end_time, curr_time;
    uint8_t level;

    while (1) {
        // Check notification
        notify_val = 0;
        notify_res = xTaskNotifyWait(0, ULONG_MAX, &notify_val, 1000 / portTICK_RATE_MS);
        if (notify_res != pdPASS) continue;
        if (notify_val == 0xA500) break; // Terminate task requested
        if (notify_val != self->pin) continue;

        // ------------------------------------------------------------------
        // ** Notification received from interrupt handler
        //    gpio interrupt is still disabled **
        //    'self->irq_level' contains the gpio level taken after interrupt
        // ------------------------------------------------------------------

        // Re-enable interrupt for debounce processing
        irq_passed = false;
        start_time = self->irq_time;
        end_time = start_time + (self->irq_debounce * 2);
        self->irq_dbcpulses = 0;
        self->irq_dbcproc = 1;
        gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);

        // -----------------------------------------------------------
        // ** We are first waiting for debounce time to elapse
        //    interrupt is still disabled and no gpio level is checked
        while (1) {
            // check the time from last interrupt
            curr_time = mp_hal_ticks_us();
            if ((curr_time - self->irq_time) > self->irq_debounce) {
                level = gpio_get_pin_value(gpiohs_handle, self->gpio);
                if ((self->irq_type == GPIO_PE_RISING) && (level != 0)) irq_passed = true;
                else if ((self->irq_type == GPIO_PE_FALLING) && (level == 0)) irq_passed = true;
                else if ((self->irq_type == GPIO_PE_BOTH) && (level == self->irq_lastlevel)) irq_passed = true;
                self->irq_time = curr_time;
                break;
            }
            // check the total ellapsed time
            if (curr_time > end_time) break;
            vTaskDelay(0); // allow other tasks to run
        }
        // Disable pin interrupt
        gpio_set_pin_edge(gpiohs_handle, self->gpio, GPIO_PE_NONE);
        // -----------------------------------------------------------

        if ((irq_passed) && (self->irq_handler)) {
            if (mp_sched_schedule(self->irq_handler, MP_OBJ_FROM_PTR(self))) self->irq_scheduled++;
            else self->irq_missed++;
        }
        self->irq_dbcproc = 0;
        // Re-enable interrupt if no callback is set or not passed
        if ((!irq_passed) || (!self->irq_handler)) gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
    } // task's main loop

    // Terminate debounce task
    self->debounce_task = NULL;
    vTaskDelete(NULL);
}

// -------------------------
// ** Gpio interrupt handler
//----------------------------------------------------------
static void machine_pin_isr_handler(uint32_t pin, void *arg)
{
    machine_pin_obj_t *self = (machine_pin_obj_t *)arg;

    // Ignore false (too short) interrupts
    uint8_t level = gpio_get_pin_value(gpiohs_handle, self->gpio);
    if ((self->irq_type == GPIO_PE_RISING) && (level == 0)) {
        gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
        return;
    }
    else if ((self->irq_type == GPIO_PE_FALLING) && (level != 0)) {
        gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
        return;
    }
    else if (self->irq_type == GPIO_PE_BOTH) {
        if ((self->irq_lastlevel >= 0) && (level == self->irq_lastlevel)) {
            gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
            return;
        }
        else self->irq_lastlevel = level;
    }

    self->irq_level = level;
    self->irq_time = mp_hal_ticks_us();
    if (self->irq_dbcproc) {
        // === Debounce processing in progress ===
        self->irq_dbcpulses++;
        gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
        return;
    }
    self->irq_num++;

    // The interrupt can be processed by the debounce task
    // or just handled here
    if ((self->debounce_task) && (self->irq_debounce > 0)) {
        // === Debounce processing is used ===
        // Notify the debounce task to process the interrupt
        BaseType_t HPTaskAwoken = pdFALSE;
        if (xTaskNotifyFromISR(self->debounce_task, self->pin, eSetValueWithoutOverwrite, &HPTaskAwoken) == pdPASS) {
            //if (HPTaskAwoken == pdTRUE) vPortYieldFromISR();
        }
        else gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
    }
    else {
        // === Debounce processing is not used ===
        if (self->irq_handler) {
            // Callback will handle re-enabling interrupt
            if (mp_sched_schedule(self->irq_handler, MP_OBJ_FROM_PTR(self))) self->irq_scheduled++;
            else self->irq_missed++;
        }
        else {
            // re-enable interrupt
            gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
            self->irq_missed++;
        }
    }
}

//----------------------------------------------
static void _pin_deinit(machine_pin_obj_t *self)
{
    if ((self->pin >= 0) && (self->pin >= 0)) {
        // Terminate debounce task
        while (self->debounce_task) {
            // Terminate the debounce task
            BaseType_t res = xTaskNotify(self->debounce_task, 0xA500, eSetValueWithoutOverwrite);
            vTaskDelay(10);
            if (res == pdPASS) break;
        }
        if (self->irq_type != GPIO_PE_NONE) {
            // Disable pin interrupt while configuring
            self->irq_type = GPIO_PE_NONE;
            gpio_set_on_changed(gpiohs_handle, self->gpio, NULL, NULL);
            gpio_set_pin_edge(gpiohs_handle, self->gpio, GPIO_PE_NONE);
        }

        gpiohs_set_free(self->gpio);
        mp_used_pins[self->pin].func = GPIO_FUNC_NONE;
        mp_used_pins[self->pin].gpio = 0;
        mp_used_pins[self->pin].fpioa_func = FUNC_DEBUG31;
        self->gpio = -1;
        self->pin = -1;
    }
}

//----------------------------------------------------------------------------------------------
STATIC void machine_pin_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if ((self->pin < 0) || (self->gpio < 0)) {
        mp_printf(print, "Deinitialized\n");
        return;
    }

    char tmpstr[32];

    if (self->mode == GPIO_DM_INPUT) sprintf(tmpstr, "IN");
    else if (self->mode == GPIO_DM_OUTPUT) sprintf(tmpstr, "OUT");
    else sprintf(tmpstr, "Unknown");

    if (self->mode == GPIO_DM_INPUT) {
        if (self->pull == GPIO_DM_INPUT_PULL_DOWN) strcat(tmpstr, ", PULL_DOWN");
        else if (self->pull == GPIO_DM_INPUT_PULL_UP) strcat(tmpstr, ", PULL_UP");
        else if (self->pull == GPIO_DM_INPUT) strcat(tmpstr, ", PULL_FLOAT");
        else strcat(tmpstr, ", Unknown");
    }

    mp_printf(print, "Pin(%02u) mode=%s, gpiohs=%d\n", self->pin, tmpstr, self->gpio);
    if (self->mode == GPIO_DM_INPUT) {
        if (self->irq_type == GPIO_PE_NONE) sprintf(tmpstr, "IRQ_DISABLED");
        else if (self->irq_type == GPIO_PE_RISING) sprintf(tmpstr, "IRQ_RISING");
        else if (self->irq_type == GPIO_PE_FALLING) sprintf(tmpstr, "IRQ_FALLING");
        else if (self->irq_type == GPIO_PE_BOTH) sprintf(tmpstr, "IRQ_ANYEDGE");
        else sprintf(tmpstr, "Unknown");
        mp_printf(print, "        handler=%s, trigger=%s, debounce=%d us (task %s)\n        interrupts=%u (rejected=%u; scheduled=%u; missed=%u)",
                (self->irq_handler) ? "True" : "False", tmpstr, self->irq_debounce, self->debounce_task ? "running" : "not running",
                self->irq_num, self->irq_num - (self->irq_scheduled+self->irq_missed), self->irq_scheduled, self->irq_missed);
    }
}

//-------------------------------------------------------
static void _createDebounceTask( machine_pin_obj_t *self)
{
    // Create the debounce task if needed
    if ((self->mode == GPIO_DM_INPUT) && (self->irq_debounce > 0) && (self->debounce_task == NULL)) {
        char dbc_task_name[16];
        sprintf(dbc_task_name, "Pin_%02d_task", self->pin);
        BaseType_t res = xTaskCreateAtProcessor(
                MainTaskProc,               // processor
                debounce_task,              // function entry
                dbc_task_name,              // task name
                configMINIMAL_STACK_SIZE,   // stack_deepth
                (void *)self,               // function argument
                12,                         // task priority
                &self->debounce_task);      // task handle
        if (res != pdPASS) self->debounce_task = NULL;
        if (self->debounce_task == NULL) {
            mp_raise_ValueError("error creating debounce task");
        }
    }
}

// constructor(id, ...)
//-------------------------------------------------------------------------------------------------------
mp_obj_t mp_pin_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
	enum { ARG_pin, ARG_mode, ARG_pull, ARG_value, ARG_handler, ARG_trigger, ARG_debounce };
	static const mp_arg_t mp_pin_allowed_args[] = {
	    { MP_QSTR_pin,						 MP_ARG_INT, {.u_int = -1}},
	    { MP_QSTR_mode,						 MP_ARG_OBJ, {.u_obj = mp_const_none}},
	    { MP_QSTR_pull,						 MP_ARG_OBJ, {.u_obj = mp_const_none}},
	    { MP_QSTR_value,	MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_bool = false}},
	    { MP_QSTR_handler,	MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
	    { MP_QSTR_trigger,	MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = GPIO_PE_NONE} },
        { MP_QSTR_debounce, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
	};

	mp_arg_val_t args[MP_ARRAY_SIZE(mp_pin_allowed_args)];
	mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(mp_pin_allowed_args), mp_pin_allowed_args, args);

	if (!machine_init_gpiohs()) {
        mp_raise_ValueError("Cannot initialize gpiohs");
	}
    // get the wanted pin object
    int wanted_pin = args[ARG_pin].u_int;

    if ((wanted_pin < 0) || (wanted_pin >= FPIOA_NUM_IO)) {
        mp_raise_ValueError("invalid pin");
    }
    if (mp_used_pins[wanted_pin].func != GPIO_FUNC_NONE) {
        char info[64];
        sprintf(info, "Pin in use by %s", gpiohs_funcs[mp_used_pins[wanted_pin].func]);
        mp_raise_ValueError(info);
    }
    int pio_num = gpiohs_get_free();
    if (pio_num < 0) {
        mp_raise_ValueError("no free gpiohs handles");
    }

	// Create Pin object
    //machine_pin_obj_t *self = m_new_obj(machine_pin_obj_t);
    machine_pin_obj_t *self = m_new_obj_with_finaliser(machine_pin_obj_t);
    memset(self, 0, sizeof(machine_pin_obj_t));
    self->base.type = &machine_pin_type;
    self->pin = wanted_pin;
    self->gpio = pio_num;
    self->mode = GPIO_DM_INPUT;
    self->pull = GPIO_DM_INPUT;
    self->irq_lastlevel = -1;

    // configure mode
    if (args[ARG_mode].u_obj != mp_const_none) {
    	int pin_io_mode = mp_obj_get_int(args[ARG_mode].u_obj);
        if ((pin_io_mode != GPIO_DM_INPUT) && (pin_io_mode != GPIO_DM_OUTPUT)) {
            gpiohs_set_free(self->gpio);
            mp_raise_ValueError("unsupported pin mode");
        }
        self->mode = pin_io_mode;
    }

    // configure pull
    if ((args[ARG_pull].u_obj != mp_const_none) && (self->mode == GPIO_DM_INPUT)) {
    	int pin_pull = mp_obj_get_int(args[ARG_pull].u_obj);
        if ((pin_pull != GPIO_DM_INPUT) && (pin_pull != GPIO_DM_INPUT_PULL_DOWN) && (pin_pull != GPIO_DM_INPUT_PULL_UP)) {
            gpiohs_set_free(self->gpio);
            mp_raise_ValueError("unsupported pull mode");
        }
        self->pull = pin_pull;
    }

    // configure the pin for gpio
    if (fpioa_set_function(self->pin, FUNC_GPIOHS0 + self->gpio) < 0) {
        gpiohs_set_free(self->gpio);
        mp_raise_ValueError("error initializing gpio");
    }

    if (self->mode == GPIO_DM_OUTPUT) gpio_set_drive_mode(gpiohs_handle, self->gpio, self->mode);
    else gpio_set_drive_mode(gpiohs_handle, self->gpio, self->pull);

    mp_used_pins[self->pin].func = GPIO_FUNC_PIN;
    mp_used_pins[self->pin].gpio = self->gpio;
    mp_used_pins[self->pin].fpioa_func = FUNC_GPIOHS0 + self->gpio;

    // set initial value
    if (self->mode == GPIO_DM_OUTPUT) {
        gpio_set_pin_value(gpiohs_handle, self->gpio, args[ARG_value].u_bool);
    }

    // Disable gpio interrupt
    gpio_set_pin_edge(gpiohs_handle, self->gpio, GPIO_PE_NONE);

    // === Configure interrupt ===
    if (self->mode == GPIO_DM_INPUT) {
        // Only input modes can have interrupts, configure it

        self->irq_lastlevel = -1;
        // Check arguments
        if ((args[ARG_trigger].u_int < GPIO_PE_NONE) || (args[ARG_trigger].u_int > GPIO_PE_BOTH)) {
            mp_raise_ValueError("invalid trigger type");
        }
        if ((args[ARG_debounce].u_int != 0) && ((args[ARG_debounce].u_int < 100) || (args[ARG_debounce].u_int > 500000))) {
            mp_raise_ValueError("wrong debounce range (0 or 100 - 500000 us)");
        }
        self->irq_type = (int8_t)args[ARG_trigger].u_int;
        self->irq_debounce = args[ARG_debounce].u_int;

        if ((MP_OBJ_IS_FUN(args[ARG_handler].u_obj)) || (MP_OBJ_IS_METH(args[ARG_handler].u_obj))) {
            self->irq_handler = args[ARG_handler].u_obj;
        }

        // Create the debounce task if needed
        _createDebounceTask(self);
        // Set irq type and register interrupt service
        gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
        gpio_set_on_changed(gpiohs_handle, self->gpio, machine_pin_isr_handler, (void*)self);
    }

    return MP_OBJ_FROM_PTR(self);
}

// pin.init(mode, pull [, kwargs])
//---------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_pin_obj_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode, ARG_pull, ARG_value, ARG_handler, ARG_trigger, ARG_debounce };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,                      MP_ARG_INT, {.u_int = -1}},
        { MP_QSTR_pull,                      MP_ARG_INT, {.u_int = -1}},
        { MP_QSTR_value,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        { MP_QSTR_handler,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        { MP_QSTR_trigger,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1}},
        { MP_QSTR_debounce, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    // parse arguments
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // ** Save previous irq mode and disable pin interrupt
    // Terminate debounce task
    while (self->debounce_task) {
        // Terminate the debounce task
        BaseType_t res = xTaskNotify(self->debounce_task, 0xA500, eSetValueWithoutOverwrite);
        vTaskDelay(10);
        if (res == pdPASS) break;
    }
    // Disable pin interrupt while configuring
    gpio_set_on_changed(gpiohs_handle, self->gpio, NULL, NULL);
    gpio_set_pin_edge(gpiohs_handle, self->gpio, GPIO_PE_NONE);

    bool changed = false;
    // configure gpio mode
    if ((args[ARG_mode].u_int >= 0) && (self->mode != args[ARG_mode].u_int)) {
        // mode provided and different from previous
        int pin_io_mode = args[ARG_mode].u_int;
        if ((pin_io_mode == GPIO_DM_INPUT) || (pin_io_mode == GPIO_DM_OUTPUT)) {
            self->mode = pin_io_mode;
            if (self->mode == GPIO_DM_OUTPUT) {
                self->pull = GPIO_DM_INPUT;
                self->irq_type = GPIO_PE_NONE;
                self->irq_debounce = 0;
                self->irq_handler = NULL;
            }
            changed = true;
        }
    }

    // configure pull
    if ((args[ARG_pull].u_int >= 0) && (self->mode == GPIO_DM_OUTPUT) && (self->pull != args[ARG_pull].u_int)) {
        // pull provided and different from previous
        int pin_pull = args[ARG_pull].u_int;
        if ((pin_pull == GPIO_DM_INPUT) || (pin_pull == GPIO_DM_INPUT_PULL_DOWN) || (pin_pull == GPIO_DM_INPUT_PULL_UP)) {
            self->pull = pin_pull;
            changed = true;
        }
    }

    if (changed) {
        if (self->mode == GPIO_DM_OUTPUT) {
            gpio_set_drive_mode(gpiohs_handle, self->gpio, self->mode);
            if (args[ARG_value].u_obj != MP_OBJ_NULL) {
                gpio_set_pin_value(gpiohs_handle, self->gpio, args[ARG_value].u_bool);
            }
        }
        else gpio_set_drive_mode(gpiohs_handle, self->gpio, self->pull);
    }

    if (self->mode == GPIO_DM_INPUT) {
        if ((args[ARG_trigger].u_int >= GPIO_PE_NONE) && (args[ARG_trigger].u_int <= GPIO_PE_BOTH) && (self->irq_type != args[ARG_trigger].u_int)) {
                self->irq_type = (int8_t)args[ARG_trigger].u_int;
        }

        if (((MP_OBJ_IS_FUN(args[ARG_handler].u_obj)) || (MP_OBJ_IS_METH(args[ARG_handler].u_obj))) && (self->irq_handler != args[ARG_handler].u_obj)) {
            self->irq_handler = args[ARG_handler].u_obj;
        }

        if ((args[ARG_debounce].u_int >= 100) && (args[ARG_debounce].u_int <= 500000) && (self->irq_debounce != args[ARG_debounce].u_int)) {
            self->irq_debounce = args[ARG_debounce].u_int;
        }

        // Create the debounce task if needed
        _createDebounceTask(self);
        // Set irq type and register interrupt service
        gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
        gpio_set_on_changed(gpiohs_handle, self->gpio, machine_pin_isr_handler, (void*)self);
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(machine_pin_init_obj, 1, machine_pin_obj_init);


// fast method for getting/setting pin value
//----------------------------------------------------------------------------------------------------
STATIC mp_obj_t machine_pin_call(mp_obj_t self_in, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (n_args == 0) {
        // get pin
        return MP_OBJ_NEW_SMALL_INT(gpio_get_pin_value(gpiohs_handle, self->gpio));
    }
    else {
        // set pin
        if (self->mode == GPIO_DM_OUTPUT) gpio_set_pin_value(gpiohs_handle, self->gpio, mp_obj_is_true(args[0]));
        return mp_const_none;
    }
}

// pin.value([value])
//----------------------------------------------------------------------
STATIC mp_obj_t machine_pin_value(size_t n_args, const mp_obj_t *args) {
    return machine_pin_call(args[0], n_args - 1, 0, args + 1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pin_value_obj, 1, 2, machine_pin_value);

//-------------------------------------------------------
STATIC mp_obj_t machine_pin_irq_value(mp_obj_t self_in) {
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    return MP_OBJ_NEW_SMALL_INT(self->irq_level);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pin_irq_value_obj, machine_pin_irq_value);

//----------------------------------------------------
STATIC mp_obj_t machine_pin_getpin(mp_obj_t self_in) {
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    return MP_OBJ_NEW_SMALL_INT(self->pin);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pin_getpin_obj, machine_pin_getpin);

//-------------------------------------------------------
STATIC mp_obj_t machine_pin_irq_time(mp_obj_t self_in) {
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    return MP_OBJ_NEW_SMALL_INT(self->irq_time);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pin_irq_time_obj, machine_pin_irq_time);

//--------------------------------------------------------
STATIC mp_obj_t machine_pin_irq_dbcnum(mp_obj_t self_in) {
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    return MP_OBJ_NEW_SMALL_INT(self->irq_dbcpulses);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pin_irq_dbcnum_obj, machine_pin_irq_dbcnum);

//---------------------------------------------------------------------
STATIC mp_obj_t machine_pin_stat(size_t n_args, const mp_obj_t *args) {
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    gpio_set_pin_edge(gpiohs_handle, self->gpio, GPIO_PE_NONE);

    mp_obj_t tuple[5];
    tuple[0] = mp_obj_new_int(self->pin);
    tuple[1] = mp_obj_new_int(self->gpio);
    tuple[2] = mp_obj_new_int(self->irq_num);
    tuple[3] = mp_obj_new_int(self->irq_scheduled);
    tuple[4] = mp_obj_new_int(self->irq_missed);
    if (n_args > 1) {
        if (mp_obj_is_true(args[1])) {
            self->irq_num = 0;
            self->irq_scheduled = 0;
            self->irq_missed = 0;
        }
    }

    gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);

    return mp_obj_new_tuple(5, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pin_stat_obj, 1, 2, machine_pin_stat);

//--------------------------------------------------
STATIC mp_obj_t machine_pin_deinit(mp_obj_t self_in)
{
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    _pin_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_pin_deinit_obj, machine_pin_deinit);

//-----------------------------------------------------------------------
STATIC mp_obj_t machine_pin_enable_irg(mp_obj_t self_in, mp_obj_t enable)
{
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->mode == GPIO_DM_INPUT) {
        if (mp_obj_is_true(enable)) gpio_set_pin_edge(gpiohs_handle, self->gpio, self->irq_type);
        else gpio_set_pin_edge(gpiohs_handle, self->gpio, GPIO_PE_NONE);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_pin_enable_irg_obj, machine_pin_enable_irg);

//================================================================
STATIC const mp_rom_map_elem_t machine_pin_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR___del__),		MP_ROM_PTR(&machine_pin_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_init),		MP_ROM_PTR(&machine_pin_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),      MP_ROM_PTR(&machine_pin_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_value),		MP_ROM_PTR(&machine_pin_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_stat),        MP_ROM_PTR(&machine_pin_stat_obj) },
    { MP_ROM_QSTR(MP_QSTR_pin),         MP_ROM_PTR(&machine_pin_getpin_obj) },
    { MP_ROM_QSTR(MP_QSTR_irqvalue),	MP_ROM_PTR(&machine_pin_irq_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_irqtime),     MP_ROM_PTR(&machine_pin_irq_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_irqdbcp),     MP_ROM_PTR(&machine_pin_irq_dbcnum_obj) },
    { MP_ROM_QSTR(MP_QSTR_irqenable),   MP_ROM_PTR(&machine_pin_enable_irg_obj) },

    // class constants
    { MP_ROM_QSTR(MP_QSTR_IN),			MP_ROM_INT(GPIO_DM_INPUT) },
    { MP_ROM_QSTR(MP_QSTR_OUT),			MP_ROM_INT(GPIO_DM_OUTPUT) },

	{ MP_ROM_QSTR(MP_QSTR_PULL_UP),		MP_ROM_INT(GPIO_DM_INPUT_PULL_UP) },
    { MP_ROM_QSTR(MP_QSTR_PULL_DOWN),	MP_ROM_INT(GPIO_DM_INPUT_PULL_DOWN) },
    { MP_ROM_QSTR(MP_QSTR_PULL_FLOAT),	MP_ROM_INT(GPIO_DM_INPUT) },

	{ MP_ROM_QSTR(MP_QSTR_IRQ_DISABLE),	MP_ROM_INT(GPIO_PE_NONE) },
	{ MP_ROM_QSTR(MP_QSTR_IRQ_RISING),	MP_ROM_INT(GPIO_PE_RISING) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_FALLING),	MP_ROM_INT(GPIO_PE_FALLING) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_ANYEDGE),	MP_ROM_INT(GPIO_PE_BOTH) },

    { MP_ROM_QSTR(MP_QSTR_LEDR),        MP_ROM_INT(14) },
    { MP_ROM_QSTR(MP_QSTR_LEDG),        MP_ROM_INT(13) },
    { MP_ROM_QSTR(MP_QSTR_LEDB),        MP_ROM_INT(12) },
};

//--------------------------------------------------------------------------------------------
STATIC mp_uint_t pin_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    (void)errcode;
    machine_pin_obj_t *self = MP_OBJ_TO_PTR(self_in);

    switch (request) {
        case MP_PIN_READ: {
            return gpio_get_pin_value(gpiohs_handle, self->gpio);
        }
        case MP_PIN_WRITE: {
            if (self->mode == GPIO_DM_OUTPUT) {
                gpio_set_pin_value(gpiohs_handle, self->gpio, arg);
                return 0;
            }
        }
    }
    return -1;
}

STATIC MP_DEFINE_CONST_DICT(machine_pin_locals_dict, machine_pin_locals_dict_table);

//-----------------------------------
STATIC const mp_pin_p_t pin_pin_p = {
  .ioctl = pin_ioctl,
};

//======================================
const mp_obj_type_t machine_pin_type = {
    { &mp_type_type },
    .name = MP_QSTR_Pin,
    .print = machine_pin_print,
    .make_new = mp_pin_make_new,
    .call = machine_pin_call,
    .protocol = &pin_pin_p,
    .locals_dict = (mp_obj_t)&machine_pin_locals_dict,
};
