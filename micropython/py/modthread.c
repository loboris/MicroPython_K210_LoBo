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
#include <stdlib.h>
#include "syslog.h"

#include "py/runtime.h"
#include "py/stackctrl.h"

#if MICROPY_PY_THREAD

#include "py/mpthread.h"
#include "modmachine.h"
#include "mphalport.h"
#include "gccollect.h"


/****************************************************************/
// _thread module

size_t thread_stack_size = MP_THREAD_DEFAULT_STACK_SIZE;

//--------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_stack_size(size_t n_args, const mp_obj_t *args) {
    mp_obj_t ret;

    if (n_args > 0) {
    	size_t stack_size = mp_obj_get_int(args[0]);
        if (stack_size == 0) {
        	stack_size = MP_THREAD_DEFAULT_STACK_SIZE; //use default stack size
        }
        else {
            if (stack_size < MP_THREAD_MIN_STACK_SIZE) stack_size = MP_THREAD_MIN_STACK_SIZE;
            else if (stack_size > MP_THREAD_MAX_STACK_SIZE) stack_size = MP_THREAD_MAX_STACK_SIZE;
        }
        thread_stack_size = (stack_size / 8) * 8;
        ret = mp_obj_new_int_from_uint(thread_stack_size);
    }
    else {
        ret = mp_obj_new_int_from_uint(thread_stack_size);
    }
    return ret;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_thread_stack_size_obj, 0, 1, mod_thread_stack_size);

// -------------------------------------------------
// This is the entry for every created Python thread
//---------------------------------
void mp_thread_entry(void *args_in)
{
    // === Execution begins here for a new thread.  We do not have the GIL. ===
    thread_entry_args_t *args = (thread_entry_args_t*)args_in;
    thread_t *th = args->thread;

    // ** Create thread state variable **
    mp_state_ctx_t th_state;

    if (mpy_config.config.use_two_main_tasks) {
        if (args->thread->processor == MAIN_TASK_PROC) {
            memcpy(&th_state.vm, &mp_state_ctx.vm, sizeof(mp_state_vm_t));
            memcpy(&th_state.mem, &mp_state_ctx.mem, sizeof(mp_state_mem_t));
        }
        else {
            memcpy(&th_state.vm, &mp_state_ctx2.vm, sizeof(mp_state_vm_t));
            memcpy(&th_state.mem, &mp_state_ctx2.mem, sizeof(mp_state_mem_t));
        }
    }
    else {
        memcpy(&th_state.vm, &mp_state_ctx.vm, sizeof(mp_state_vm_t));
        memcpy(&th_state.mem, &mp_state_ctx.mem, sizeof(mp_state_mem_t));
    }

    // Save thread state in local storage pointer #0
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, &th_state);

    // Initialize PyStack
    mp_pystack_init(args->pystack_start, args->pystack_end, mpy_config.config.pystack_enabled);

    // Set stack top & limit
    mp_stack_set_top(&th_state.thread + sizeof(void*)); // need to include thread state in root-pointer scan
    mp_stack_set_limit((mp_uint_t)((void *)&th_state.thread - (void *)pxTaskGetStackStart(NULL)) + sizeof(void*) - MICROPY_TASK_STACK_RESERVED);

    // Set locals and globals from the calling context
    mp_locals_set(args->dict_locals);
    mp_globals_set(args->dict_globals);

    // Set the thread handle
    th->id = xTaskGetCurrentTaskHandle();

    // Save the thread arguments in local storage pointer #1
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, args->args);

    // **Signal that we are set up and running
    vTaskDelay(1 / portTICK_PERIOD_MS);
    th->ready = 1;

    MP_THREAD_GIL_ENTER();

    // TODO set more thread-specific state here:
    //  mp_pending_exception? (root pointer)
    //  cur_exception (root pointer)

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
    	// === Start executing the byte code ===
        mp_call_function_n_kw(args->fun, args->n_args, args->n_kw, args->args);
        nlr_pop();
    }
    else { // uncaught exception
        // check for SystemExit
        mp_obj_base_t *exc = (mp_obj_base_t*)nlr.ret_val;
        if (mp_obj_is_subclass_fast(MP_OBJ_FROM_PTR(exc->type), MP_OBJ_FROM_PTR(&mp_type_SystemExit))) {
            // swallow exception silently
        }
        else if (mp_obj_is_subclass_fast(MP_OBJ_FROM_PTR(exc->type), MP_OBJ_FROM_PTR(&mp_type_KeyboardInterrupt))) {
            // Simulated Keyboard interrupt exits the thread, not a real exception
            /*
            mp_printf(&mp_plat_print, "Thread exit requested in thread started by ");
            mp_obj_print_helper(&mp_plat_print, args->fun, PRINT_REPR);
            mp_printf(&mp_plat_print, "\n");
            */
        }
        else {
            // print exception out
            mp_printf(&mp_plat_print, "Unhandled exception in thread started by ");
            mp_obj_print_helper(&mp_plat_print, args->fun, PRINT_REPR);
            mp_printf(&mp_plat_print, "\n");
            mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(exc));
        }
    }

    MP_THREAD_GIL_EXIT();

    // signal that we are finished
    mp_thread_finish();
    // and remove the thread from the linked list of all threads
    mp_thread_remove_thread();

    gc_collect();

    vTaskDelete(NULL);
}

//-----------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_start_new_thread(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
	const mp_arg_t allowed_args[] = {
	   { MP_QSTR_name,		MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
	   { MP_QSTR_func,		MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
	   { MP_QSTR_arg,		MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
	   { MP_QSTR_kwarg,		                  MP_ARG_OBJ,  { .u_obj = mp_const_none } },
	   { MP_QSTR_stacksize,                   MP_ARG_INT,  { .u_int = -1 } },
       { MP_QSTR_priority,                    MP_ARG_INT,  { .u_int = MICROPY_TASK_PRIORITY } },
       { MP_QSTR_pystacksize,                 MP_ARG_INT,  { .u_int = -1 } },
	};

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // This structure holds the Python function and arguments for thread entry.
    // We copy all arguments into this structure to keep ownership of them.
    // We must be very careful about root pointers because this pointer may
    // disappear from our address space before the thread is created.
    thread_entry_args_t *th_args;

    char *name = NULL;
	if (mp_obj_is_str(args[0].u_obj)) {
		name = (char *)mp_obj_str_get_str(args[0].u_obj);
	}
	else {
        mp_raise_TypeError("expecting a string for thread name argument");
	}

	if ((!mp_obj_is_fun(args[1].u_obj)) && (!mp_obj_is_meth(args[1].u_obj))) {
       	mp_raise_TypeError("expecting a function for thread function argument");
    }

    // get positional arguments
    size_t pos_args_len;
    mp_obj_t *pos_args_items;
    mp_obj_get_array(args[2].u_obj, &pos_args_len, &pos_args_items);

    // check for keyword arguments
    if (args[3].u_obj == mp_const_none) {
        // only positional arguments
        th_args = m_new_obj_var(thread_entry_args_t, mp_obj_t, pos_args_len);
        th_args->n_kw = 0;
    }
    else {
        // positional and keyword arguments
        if (mp_obj_get_type(args[3].u_obj) != &mp_type_dict) {
            mp_raise_TypeError("expecting a dict for keyword args");
        }
        mp_map_t *map = &((mp_obj_dict_t*)MP_OBJ_TO_PTR(args[3].u_obj))->map;
        th_args = m_new_obj_var(thread_entry_args_t, mp_obj_t, pos_args_len + 2 * map->used);
        th_args->n_kw = map->used;
        // copy across the keyword arguments
        for (size_t i = 0, n = pos_args_len; i < map->alloc; ++i) {
            if (MP_MAP_SLOT_IS_FILLED(map, i)) {
                th_args->args[n++] = map->table[i].key;
                th_args->args[n++] = map->table[i].value;
            }
        }
    }

    // copy across the positional arguments
    th_args->n_args = pos_args_len;
    memcpy(th_args->args, pos_args_items, pos_args_len * sizeof(mp_obj_t));

    // pass our locals and globals into the new thread
    th_args->dict_locals = mp_locals_get();
    th_args->dict_globals = mp_globals_get();
    th_args->thread = NULL;

    // Set stack size(s)
    size_t task_stack_size = thread_stack_size;
    size_t py_stack_size = MICROPY_PYSTACK_SIZE;

    if (mpy_config.config.pystack_enabled) {
        if (args[6].u_int >= MP_THREAD_MIN_PYSTACK_SIZE) {
            py_stack_size = args[6].u_int;
            if (py_stack_size > MP_THREAD_MAX_PYSTACK_SIZE) py_stack_size = MP_THREAD_MAX_PYSTACK_SIZE;
            py_stack_size = (py_stack_size / 8) * 8;
        }
    }
    if (args[4].u_int >= MP_THREAD_MIN_STACK_SIZE) {
        task_stack_size = args[4].u_int;
        if (task_stack_size > MP_THREAD_MAX_STACK_SIZE) task_stack_size = MP_THREAD_MAX_STACK_SIZE;
        task_stack_size = (task_stack_size / 8) * 8;
    }

    // Set priority
    int priority = args[5].u_int;
    if ((priority < 0) || (priority > MP_THREAD_MAX_PRIORITY)) priority = MICROPY_TASK_PRIORITY;

    // Set the function for thread entry
    th_args->fun = args[1].u_obj;

    // === Create the thread task ===
    uintptr_t thr_id = (uintptr_t)mp_thread_create(mp_thread_entry, th_args, task_stack_size, py_stack_size, priority, name);

    return mp_obj_new_int_from_uint((uintptr_t)thr_id);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_thread_start_new_thread_obj, 3, mod_thread_start_new_thread);

/*
//------------------------------------------
STATIC mp_obj_t mod_thread_get_ident(void) {
    return mp_obj_new_int_from_uint((uintptr_t)mp_thread_get_state());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_get_ident_obj, mod_thread_get_ident);

//-------------------------------------
STATIC mp_obj_t mod_thread_exit(void) {
    nlr_raise(mp_obj_new_exception(&mp_type_SystemExit));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_exit_obj, mod_thread_exit);

//----------------------------------------------
STATIC mp_obj_t mod_thread_allocate_lock(void) {
    return MP_OBJ_FROM_PTR(mp_obj_new_thread_lock());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_allocate_lock_obj, mod_thread_allocate_lock);
*/

/****************************************************************/
// Inter-thread notifications and messaging


//-------------------------------------------------
STATIC mp_obj_t mod_thread_status(mp_obj_t in_id) {
	uintptr_t thr_id = mp_obj_get_int(in_id);

	int res = mp_thread_status((void *)thr_id);
    return MP_OBJ_NEW_SMALL_INT(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_status_obj, mod_thread_status);

//-------------------------------------------------------
STATIC mp_obj_t mod_thread_allowsuspend(mp_obj_t in_id) {
	int allow = mp_obj_is_true(in_id);

	mp_thread_allowsuspend(allow);
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_allowsuspend_obj, mod_thread_allowsuspend);

//--------------------------------------------------
STATIC mp_obj_t mod_thread_suspend(mp_obj_t in_id) {
	uintptr_t thr_id = mp_obj_get_int(in_id);

	if (mp_thread_suspend((void *)thr_id)) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_suspend_obj, mod_thread_suspend);

//-------------------------------------------------
STATIC mp_obj_t mod_thread_resume(mp_obj_t in_id) {
	uintptr_t thr_id = mp_obj_get_int(in_id);

	if (mp_thread_resume((void *)thr_id)) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_resume_obj, mod_thread_resume);

//-----------------------------------------------
STATIC mp_obj_t mod_thread_stop(mp_obj_t in_id) {
	uintptr_t thr_id = mp_obj_get_int(in_id);

	//if (mp_thread_notify((void *)thr_id, THREAD_NOTIFY_EXIT)) return mp_const_true;
    //return mp_const_false;
    mp_thread_kbd_interrupt((void *)thr_id);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_stop_obj, mod_thread_stop);

//--------------------------------------------------------------------
STATIC mp_obj_t mod_thread_notify(mp_obj_t in_id, mp_obj_t in_value) {
	uintptr_t thr_id = mp_obj_get_int(in_id);
	uint32_t notify_value = mp_obj_get_int(in_value);
	if (notify_value == 0) return mp_const_false;

	if (mp_thread_notify((void *)thr_id, notify_value)) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_thread_notify_obj, mod_thread_notify);

//-----------------------------------------------------
STATIC mp_obj_t mod_thread_isnotified(mp_obj_t in_id) {
	uintptr_t thr_id = mp_obj_get_int(in_id);
	int notify = mp_thread_notifyPending((void *)thr_id);

	if (notify == 0) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_isnotified_obj, mod_thread_isnotified);

//--------------------------------------
STATIC mp_obj_t mod_thread_getMAINId() {
    return mp_obj_new_int_from_uint((uintptr_t)MainTaskHandle);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_getMAINId_obj, mod_thread_getMAINId);

//---------------------------------------
STATIC mp_obj_t mod_thread_getMAIN2Id() {
    if (mpy_config.config.use_two_main_tasks) return mp_obj_new_int_from_uint((uintptr_t)MainTaskHandle2);
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_getMAIN2Id_obj, mod_thread_getMAIN2Id);

//--------------------------------------
STATIC mp_obj_t mod_thread_getnotify() {

	uint32_t not_val = mp_thread_getnotify(0);
    return MP_OBJ_NEW_SMALL_INT(not_val);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_getnotify_obj, mod_thread_getnotify);

//----------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_sendmsg(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_ThreadID, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_Message,  MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
       { MP_QSTR_Code,                       MP_ARG_INT, { .u_int = 0 } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const char *msg = NULL;
    size_t msglen = 0;
    mp_int_t msg_int = 0;
    uintptr_t thr_id = 1;

    int type = THREAD_MSG_TYPE_INTEGER;

	if (mp_obj_is_int(args[0].u_obj)) {
		thr_id = mp_obj_get_int((mp_const_obj_t)args[0].u_obj);
	}
	else return mp_const_false;

	if (mp_obj_is_str(args[1].u_obj)) {
        msg = mp_obj_str_get_data(args[1].u_obj, &msglen);
        if (msglen == 0) return mp_const_false;

        msg_int = args[2].u_int;
        type = THREAD_MSG_TYPE_STRING;
    }
	else if (mp_obj_is_int(args[1].u_obj)) {
    	msg_int = mp_obj_get_int(args[1].u_obj);
    }
	else return mp_const_false;

	int res = mp_thread_semdmsg((void *)thr_id, type, msg_int, (uint8_t *)msg, msglen);

	return MP_OBJ_NEW_SMALL_INT(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_thread_sendmsg_obj, 2, mod_thread_sendmsg);

//---------------------------------
STATIC mp_obj_t mod_thread_getmsg()
{
	uint8_t *buf = NULL;
	uint32_t msg_int = 0;
	int res = 0;
	uint32_t buflen = 0;
	uintptr_t sender = 0;
    mp_obj_t tuple[4];

	res = mp_thread_getmsg(&msg_int, &buf, &buflen, (uint64_t *)&sender);

	tuple[0] = mp_obj_new_int(res);
    tuple[1] = mp_obj_new_int(sender);
    tuple[2] = mp_const_none;
    tuple[3] = mp_const_none;
	if (res != THREAD_MSG_TYPE_NONE) {
	    tuple[2] = mp_obj_new_int(msg_int);
		if (buf != NULL) {
			tuple[3] = mp_obj_new_str((char *)buf, buflen);
			vPortFree(buf);
		}
	}

    return mp_obj_new_tuple(4, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_getmsg_obj, mod_thread_getmsg);

//--------------------------------------------------
STATIC mp_obj_t mod_thread_getname(mp_obj_t in_id) {
	uintptr_t thr_id = mp_obj_get_int(in_id);
	char name[THREAD_NAME_MAX_SIZE] = {'\0'};

	int res = mp_thread_getname((void *)thr_id, name);
	if (!res) {
		sprintf(name,"unknown");
	}
	return mp_obj_new_str((char *)name, strlen(name));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_getname_obj, mod_thread_getname);

//------------------------------------------------------
STATIC mp_obj_t mod_thread_getpriority(mp_obj_t in_id) {
    uintptr_t thr_id = mp_obj_get_int(in_id);
    if (thr_id == 0) thr_id = mp_thread_getSelfID();

    int res = mp_thread_get_priority((void *)thr_id);
    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_getpriority_obj, mod_thread_getpriority);

//-----------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_setpriority(mp_obj_t in_id, mp_obj_t new_priority) {
    uintptr_t thr_id = mp_obj_get_int(in_id);
    if (thr_id == 0) thr_id = mp_thread_getSelfID();
    int priority = mp_obj_get_int(new_priority);
    if ((priority < 0) || (priority > MP_THREAD_MAX_PRIORITY)) {
        mp_raise_ValueError("Priority outside allowed range");
    }

    mp_thread_set_priority((void *)thr_id, priority);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_thread_setpriority_obj, mod_thread_setpriority);

//------------------------------------------------------
STATIC mp_obj_t mod_thread_getSelfname() {
	char name[THREAD_NAME_MAX_SIZE] = {'\0'};

	int res = mp_thread_getSelfname(name);
	if (!res) {
		sprintf(name,"unknown");
	}
	return mp_obj_new_str((char *)name, strlen(name));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_getSelfname_obj, mod_thread_getSelfname);

//-----------------------------------------------------------------------
STATIC mp_obj_t mod_thread_list(mp_uint_t n_args, const mp_obj_t *args) {
    TaskHandle_t selfID = (TaskHandle_t)mp_thread_getSelfID();
    // Raise thread priority while procesing list
    int t_priority = mp_thread_set_priority(selfID, MP_THREAD_MAX_PRIORITY);

    int prn = 1, n = 0;
    if (n_args > 0) prn = mp_obj_is_true(args[0]);

	thread_list_t list = {0, NULL};

	// Acquire MicroPython threads list
	uint32_t num = mp_thread_list(&list);

	if ((num == 0) || (list.threads == NULL)) return mp_const_none;

	threadlistitem_t *thr = NULL;
	if (prn) {
	    // Print the threads list
	    bool has_stats = false;
        char th_state[16] = {'\0'};
        TaskStatus_t *pxTaskStatusArray = NULL;
        volatile UBaseType_t uxArraySize, uxArraySize1, x;
        uint64_t ulTotalRunTime;
        double ulStatsAsPercentage;
        uxArraySize = uxTaskGetNumberOfTasksAllProc();

        #if configUSE_TRACE_FACILITY
        // Get Run time statistics
        // Take a snapshot of the number of tasks in case it changes while this function is executing.
        // Allocate a TaskStatus_t structure for each task.  An array could be allocated statically at compile time.
        pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t) + 8);
        if ( pxTaskStatusArray != NULL ) {
            memset(pxTaskStatusArray, 0, uxArraySize * sizeof(TaskStatus_t) + 8);
            // Generate raw status information about each task.
            uxArraySize1 = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, (uint64_t *)&ulTotalRunTime );
            if (uxArraySize != uxArraySize1) {
                mp_printf(&mp_plat_print, "System state reported number of running tasks: %d", uxArraySize1);
            }
            // Avoid divide by zero errors.
            if ((ulTotalRunTime / 100) > 0) has_stats = true;
        }
        #endif

        mp_printf(&mp_plat_print, "\nTotal system run time: ");
        if (has_stats) {
            mp_printf(&mp_plat_print, "%.3f s, number of tasks running: %d", (double)ulTotalRunTime / 1000000.0, uxArraySize);
            // For percentage calculations.
            ulTotalRunTime /= 100UL;
        }
        else {
            mp_printf(&mp_plat_print, "%.3f s", (double)mp_hal_ticks_us() / 1000000.0);
        }
        mp_printf(&mp_plat_print, "\nMicroPython threads:\n");
        if (mpy_config.config.pystack_enabled) {
            mp_printf(&mp_plat_print, "------------------------------------------------------------------------------------------------");
            if (has_stats) mp_printf(&mp_plat_print, "-------------------\n");
            else mp_printf(&mp_plat_print, "\n");
            mp_printf(&mp_plat_print, "ID(handle) Proc             Name      State  Stack  Used MaxUsed PyStack   Used    Type Priority");
            if (has_stats) mp_printf(&mp_plat_print, " Run time (s)   (%%)\n");
            else mp_printf(&mp_plat_print, "\n");
            mp_printf(&mp_plat_print, "------------------------------------------------------------------------------------------------");
            if (has_stats) mp_printf(&mp_plat_print, "-------------------\n");
            else mp_printf(&mp_plat_print, "\n");
        }
        else {
            mp_printf(&mp_plat_print, "---------------------------------------------------------------------------------");
            if (has_stats) mp_printf(&mp_plat_print, "-------------------\n");
            else mp_printf(&mp_plat_print, "\n");
            mp_printf(&mp_plat_print, "ID(handle) Proc             Name      State  Stack  Used MaxUsed    Type Priority");
            if (has_stats) mp_printf(&mp_plat_print, " Run time (s)   (%%)\n");
            else mp_printf(&mp_plat_print, "\n");
            mp_printf(&mp_plat_print, "---------------------------------------------------------------------------------");
            if (has_stats) mp_printf(&mp_plat_print, "-------------------\n");
            else mp_printf(&mp_plat_print, "\n");
        }

        for (n=0; n<num; n++) {
			thr = list.threads + (sizeof(threadlistitem_t) * n);
			char th_type[8] = {'\0'};
			if (thr->type == THREAD_TYPE_MAIN) sprintf(th_type, "MAIN");
			else if (thr->type == THREAD_TYPE_PYTHON) sprintf(th_type, "PYTHON");
			else if (thr->type == THREAD_TYPE_SERVICE) sprintf(th_type, "SERVICE");
			else sprintf(th_type, "Unknown");
			if (thr->suspended) sprintf(th_state, "suspended");
			else if (thr->waiting) sprintf(th_state, "waiting");
			else sprintf(th_state, "running");


	        if (mpy_config.config.pystack_enabled) {
                mp_printf(&mp_plat_print, "%10u%5d%17s%s%10s%7d%6d%8d%8d%7d%8s%9d",
                        thr->id, thr->proc, thr->name, (thr->current) ? "*" : " ", th_state, thr->stack_len, thr->stack_curr, thr->stack_max,
                        thr->pystack_len, thr->pystack_used, th_type, (thr->id == (uint64_t)selfID) ? t_priority : thr->priority);
	        }
            else {
                mp_printf(&mp_plat_print, "%10u%5d%17s%s%10s%7d%6d%8d%8s%9d",
                        thr->id, thr->proc, thr->name, (thr->current) ? "*" : " ", th_state, thr->stack_len, thr->stack_curr, thr->stack_max,
                        th_type, (thr->id == (uint64_t)selfID) ? t_priority : thr->priority);
            }
			if (has_stats) {
			    for ( x = 0; x < uxArraySize; x++ ) {
			        if (pxTaskStatusArray[x].xHandle == (TaskHandle_t)thr->id) {
			            ulStatsAsPercentage = (double)pxTaskStatusArray[x].ulRunTimeCounter / (double)ulTotalRunTime;
			            mp_printf(&mp_plat_print, "%13.3f%6.2f", (double)pxTaskStatusArray[x].ulRunTimeCounter / 1000000.0, ulStatsAsPercentage);
			            break;
			        }
			    }
			}
			mp_printf(&mp_plat_print, "\n");
		}

	    vPortFree(list.threads);

		if (has_stats) {
		    char taskname[configMAX_TASK_NAME_LEN] = {'\0'};
            mp_printf(&mp_plat_print, "\nFreeRTOS tasks running:\n");
            mp_printf(&mp_plat_print, "-------------------------------------------------------------------------------\n");
            mp_printf(&mp_plat_print, "ID(handle) Proc             Name     State MinStack Priority Run time (s)   (%%)\n");
            mp_printf(&mp_plat_print, "-------------------------------------------------------------------------------\n");
            for ( x = 0; x < uxArraySize; x++ ) {
                memcpy(taskname, pxTaskStatusArray[x].pcTaskName, configMAX_TASK_NAME_LEN);
                for (int i=0; i<configMAX_TASK_NAME_LEN; i++) {
                    if (taskname[i] == '\0') break;
                    if ((taskname[i] < ' ') || (taskname [i] > 0x7f)) {
                        sprintf(taskname, "?");
                        break;
                    }
                }
                if (pxTaskStatusArray[x].eCurrentState == eRunning) sprintf(th_state, "Running");
                else if (pxTaskStatusArray[x].eCurrentState == eReady) sprintf(th_state, "Ready");
                else if (pxTaskStatusArray[x].eCurrentState == eBlocked) sprintf(th_state, "Blocked");
                else if (pxTaskStatusArray[x].eCurrentState == eSuspended) sprintf(th_state, "Suspended");
                else if (pxTaskStatusArray[x].eCurrentState == eDeleted) sprintf(th_state, "Deleted");
                else sprintf(th_state, "Invalid");

                ulStatsAsPercentage = (double)pxTaskStatusArray[x].ulRunTimeCounter / (double)ulTotalRunTime;
                mp_printf(&mp_plat_print, "%10u%5d%17s%10s%9u%9d%13.3f%6.2f\n",
                        pxTaskStatusArray[x].xHandle, pxTaskStatusArray[x].xProcessor,
                        taskname, th_state, pxTaskStatusArray[x].usStackHighWaterMark * sizeof(StackType_t),
                        pxTaskStatusArray[x].uxCurrentPriority, (double)pxTaskStatusArray[x].ulRunTimeCounter / 1000000.0, ulStatsAsPercentage);
            }
            mp_printf(&mp_plat_print, "-------------------------------------------------------------------------------\n");
            mp_printf(&mp_plat_print, "FreeRTOS heap: Size: %lu, Free: %lu, Min free: %lu\n",
                    FREE_RTOS_TOTAL_HEAP_SIZE, xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
            mp_printf(&mp_plat_print, "  System heap: Free: %lu\n", _check_remaining_heap());
        }

        mp_printf(&mp_plat_print, "\n");

        #if configUSE_TRACE_FACILITY
        if ( pxTaskStatusArray != NULL ) vPortFree(pxTaskStatusArray);
        #endif

        mp_thread_set_priority(selfID, t_priority);
		return mp_const_none;
	}
	else {
	    // Return threads list tuple
		int services = 0;
		mp_obj_t thr_info[7];
		mp_obj_t tuple[num+services];
		for (n=0; n<num; n++) {
			thr = list.threads + (sizeof(threadlistitem_t) * n);
			thr_info[0] = mp_obj_new_int(thr->id);
			thr_info[1] = mp_obj_new_int(thr->type);
			thr_info[2] = mp_obj_new_str(thr->name, strlen(thr->name));
			if (thr->suspended) thr_info[3] = mp_obj_new_int(1);
			else if (thr->waiting) thr_info[3] = mp_obj_new_int(2);
			else thr_info[3] = mp_obj_new_int(0);
			thr_info[4] = mp_obj_new_int(thr->stack_len);
			thr_info[5] = mp_obj_new_int(thr->stack_max);
            thr_info[6] = mp_obj_new_int((thr->id == (uint64_t)selfID) ? t_priority : thr->priority);
			tuple[n] = mp_obj_new_tuple(7, thr_info);
		}
		vPortFree(list.threads);

	    mp_thread_set_priority(selfID, t_priority);
		return mp_obj_new_tuple(n, tuple);
	}
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_thread_list_obj, 0, 1, mod_thread_list);

//--------------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_mainAcceptMsg(mp_uint_t n_args, const mp_obj_t *args) {
	int res = 0;
    if (n_args == 0) {
    	res = mp_thread_mainAcceptMsg(-1);
    }
    else {
    	res = mp_thread_mainAcceptMsg(mp_obj_is_true(args[0]));
    }
	return mp_obj_new_bool(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_thread_mainAcceptMsg_obj, 0, 1, mod_thread_mainAcceptMsg);

//-----------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_waitnotify(mp_uint_t n_args, const mp_obj_t *args) {
	uint32_t tmo = portMAX_DELAY;
    if (n_args > 0) {
    	tmo = mp_obj_get_int(args[0]);
    	if (tmo < 1) tmo = 1;
    	tmo /= portTICK_PERIOD_MS;
    }
	uint64_t not_val = 0;
	mp_obj_t ret = mp_const_none;

   	if (mp_thread_setblocked()) {
   		MP_THREAD_GIL_EXIT();
		if (xTaskNotifyWait(0, 0, &not_val, tmo) == pdPASS) {
			xTaskNotifyWait(ULLONG_MAX, ULLONG_MAX, NULL, 0);
			mp_thread_resetPending();
			ret = MP_OBJ_NEW_SMALL_INT(not_val);
		}
		mp_thread_setnotblocked();
		MP_THREAD_GIL_ENTER();
   	}

	return ret;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_thread_waitnotify_obj, 0, 1, mod_thread_waitnotify);

//-------------------------------
STATIC mp_obj_t mod_thread_lock()
{
    TaskHandle_t selfID = (TaskHandle_t)mp_thread_getSelfID();
    int t_priority = mp_thread_set_priority(selfID, MP_THREAD_MAX_PRIORITY);

    int res = mp_thread_suspend_others();

    mp_thread_set_priority(selfID, t_priority);
    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_lock_obj, mod_thread_lock);

//---------------------------------
STATIC mp_obj_t mod_thread_unlock()
{
    TaskHandle_t selfID = (TaskHandle_t)mp_thread_getSelfID();
    int t_priority = mp_thread_set_priority(selfID, MP_THREAD_MAX_PRIORITY);

    int res = mp_thread_resume_others();

    mp_thread_set_priority(selfID, t_priority);
    // Switch threads
    MP_THREAD_GIL_EXIT();
    MP_THREAD_GIL_ENTER();
    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_unlock_obj, mod_thread_unlock);


//------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_pystack(mp_uint_t n_args, const mp_obj_t *args)
{
    uintptr_t thr_id = 0;
    int res = -1;
    if (n_args > 0) thr_id = mp_obj_get_int(args[0]);
    if (thr_id == 0) {
        res = mp_thread_pystack_get_size(NULL);
    }
    else {
        res = mp_thread_pystack_get_size((void *)thr_id);
    }
    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_thread_pystack_obj, 0, 1, mod_thread_pystack);

//-----------------------------------------------------
STATIC mp_obj_t mod_thread_set_callback(mp_obj_t cb_in)
{
    if (xTaskGetCurrentTaskHandle() != MainTaskHandle) return mp_const_false;

    if ((mp_obj_is_fun(cb_in)) || (mp_obj_is_meth(cb_in))) {
        main_task_callback = cb_in;
    }
    else if ((cb_in == mp_const_none) || (cb_in == mp_const_false)) {
        main_task_callback = 0;
    }
    else {
        mp_raise_ValueError("Function, None or False argument expected");
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_set_callback_obj, mod_thread_set_callback);


// -------------------------------------
// Inter Processor Communication methods
// -------------------------------------

typedef struct _prnbuf_t {
    char   *data;
    size_t len;
    size_t size;
    bool   status;
} prnbuf_t;

//-------------------------------------------------------------------
STATIC void _mp_print_to_buf(void *data, const char *str, size_t len)
{
    prnbuf_t *buf = (prnbuf_t *)data;
    if (!buf->status) return;

    if ((buf->len + len) >= buf->size) {
        // reallocate the buffer
        char *tmp_buf = pvPortMalloc(buf->len + len + 256);
        if (tmp_buf) {
            memset(tmp_buf, 0, buf->len + len + 256);
            memcpy(tmp_buf, buf->data, buf->len);
            buf->size = buf->len + len + 256;
            vPortFree(buf->data);
            buf->data = tmp_buf;
        }
        else {
            buf->status = false;
            return;
        }
    }
    memcpy(buf->data + buf->len, str, len);
    buf->len += len;
    buf->data[buf->len] = '\0';
}

//-----------------------------------------
STATIC mp_obj_t mod_thread_get_ipc_outbuf()
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_false;
    if (xSemaphoreTake(inter_proc_mutex, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        return mp_const_false;
    }

    mp_obj_t res = mp_const_none;
    if (task_ipc.ipc_response_buff != NULL) {
        res = mp_obj_new_str(task_ipc.ipc_response_buff, task_ipc.ipc_response_buff_idx);
        // clear the buffer
        task_ipc.ipc_response_buff_size = 0;
        task_ipc.ipc_response_buff_idx = 0;
    }
    xSemaphoreGive(inter_proc_mutex);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_get_ipc_outbuf_obj, mod_thread_get_ipc_outbuf);

//--------------------------------------------
STATIC mp_obj_t mod_thread_get_ipc_outbuflen()
{
    if (xSemaphoreTake(inter_proc_mutex, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        return mp_const_false;
    }

    int res = task_ipc.ipc_response_buff_idx;
    xSemaphoreGive(inter_proc_mutex);

    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_get_ipc_outbuflen_obj, mod_thread_get_ipc_outbuflen);

//-------------------------------------------
STATIC mp_obj_t mod_thread_get_ipc_clearout()
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_false;
    if (xSemaphoreTake(inter_proc_mutex, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        return mp_const_false;
    }

    if (task_ipc.ipc_response_buff != NULL) {
        vPortFree(task_ipc.ipc_response_buff);
        task_ipc.ipc_response_buff = NULL;
    }
    task_ipc.ipc_response_buff_size = 0;
    task_ipc.ipc_response_buff_idx = 0;
    xSemaphoreGive(inter_proc_mutex);

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_get_ipc_clearout_obj, mod_thread_get_ipc_clearout);

//-----------------------------------------------------
STATIC mp_obj_t mod_thread_ipc_command(mp_obj_t cmd_in)
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_false;
    int res;

    if (mp_obj_is_str(cmd_in)) {
        size_t cmdlen = 0;
        const char *cmd = mp_obj_str_get_data(cmd_in, &cmdlen);
        if (xTaskGetCurrentTaskHandle() == MainTaskHandle)
            res = mp_thread_sendmsg_to_mpy2(THREAD_MSG_TYPE_STRING, THREAD_IPC_TYPE_EXECUTE, (uint8_t *)cmd, cmdlen);
        else
            res = mp_thread_sendmsg_to_mpy1(THREAD_MSG_TYPE_STRING, THREAD_IPC_TYPE_EXECUTE, (uint8_t *)cmd, cmdlen);
    }
    else {
        int icmd = mp_obj_get_int(cmd_in);
        if (xTaskGetCurrentTaskHandle() == MainTaskHandle)
            res = mp_thread_sendmsg_to_mpy2(THREAD_MSG_TYPE_INTEGER, icmd, NULL, 0);
        else
            res = mp_thread_sendmsg_to_mpy1(THREAD_MSG_TYPE_INTEGER, icmd, NULL, 0);
    }

    return mp_obj_new_bool(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_ipc_command_obj, mod_thread_ipc_command);

//------------------------------------------------------------------------
STATIC mp_obj_t mod_thread_ipc_msg(mp_uint_t n_args, const mp_obj_t *args)
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_false;
    int res;

    if (n_args > 1) {
        int icode = mp_obj_get_int(args[0]);
        size_t msglen = 0;
        const char *msg = mp_obj_str_get_data(args[1], &msglen);
        if (xTaskGetCurrentTaskHandle() == MainTaskHandle)
            res = mp_thread_sendmsg_to_mpy2(THREAD_MSG_TYPE_STRING, icode, (uint8_t *)msg, msglen);
        else
            res = mp_thread_sendmsg_to_mpy1(THREAD_MSG_TYPE_STRING, icode, (uint8_t *)msg, msglen);
    }
    else {
        int icmd = mp_obj_get_int(args[0]);
        if (xTaskGetCurrentTaskHandle() == MainTaskHandle)
            res = mp_thread_sendmsg_to_mpy2(THREAD_MSG_TYPE_INTEGER, icmd, NULL, 0);
        else
            res = mp_thread_sendmsg_to_mpy1(THREAD_MSG_TYPE_INTEGER, icmd, NULL, 0);
    }

    return mp_obj_new_bool(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_thread_ipc_msg_obj, 1, 2, mod_thread_ipc_msg);

//-----------------------------------------------------------------------
STATIC mp_obj_t mod_thread_set_ipc_var(mp_obj_t name_in, mp_obj_t obj_in)
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_false;
    const char *name = mp_obj_str_get_str(name_in);

    prnbuf_t pbuf;
    mp_print_t print;

    pbuf.data = pvPortMalloc(512);
    if (pbuf.data == NULL) {
        mp_raise_ValueError("Error allocating object buffer");
    }
    memset(pbuf.data, 0, 512);
    sprintf(pbuf.data, "%s=", name);
    pbuf.len = strlen(pbuf.data);
    pbuf.size = 512;
    pbuf.status = true;

    print.data = (void *)&pbuf;
    print.print_strn = (mp_print_strn_t)_mp_print_to_buf;
    mp_obj_print_helper(&print, obj_in, PRINT_REPR);

    if (!pbuf.status) {
        vPortFree(pbuf.data);
        return mp_const_false;
    }

    if (xSemaphoreTake(inter_proc_mutex, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        return mp_const_false;
    }

    int res;
    if (xTaskGetCurrentTaskHandle() == MainTaskHandle)
        res = mp_thread_sendmsg_to_mpy2(THREAD_MSG_TYPE_STRING, THREAD_IPC_TYPE_EXECUTE, (uint8_t *)pbuf.data, strlen(pbuf.data));
    else
        res = mp_thread_sendmsg_to_mpy1(THREAD_MSG_TYPE_STRING, THREAD_IPC_TYPE_EXECUTE, (uint8_t *)pbuf.data, strlen(pbuf.data));
    vPortFree(pbuf.data);

    xSemaphoreGive(inter_proc_mutex);

    return mp_obj_new_bool(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_thread_set_ipc_var_obj, mod_thread_set_ipc_var);

//----------------------------------------
STATIC mp_obj_t mod_thread_get_ipc_state()
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_false;
    if (xSemaphoreTake(inter_proc_mutex, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        return mp_const_false;
    }
    mp_obj_t res = mp_obj_new_bool(task_ipc.busy);
    xSemaphoreGive(inter_proc_mutex);

    return res;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_get_ipc_state_obj, mod_thread_get_ipc_state);

//-----------------------------------------------
STATIC mp_obj_t mod_thread_get_ipc_setexception()
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_none;
    if (xSemaphoreTake(inter_proc_mutex, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        return mp_const_false;
    }
    if (task_ipc.busy) task_ipc.irq = true;
    xSemaphoreGive(inter_proc_mutex);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mod_thread_get_ipc_setexception_obj, mod_thread_get_ipc_setexception);

//------------------------------------------------------
STATIC mp_obj_t mod_thread_ipc_notify(mp_obj_t in_value)
{
    if (!mpy_config.config.use_two_main_tasks) return mp_const_false;
    uintptr_t thr_id = (uintptr_t)MainTaskHandle2;
    if (xTaskGetCurrentTaskHandle() == MainTaskHandle2) thr_id = (uintptr_t)MainTaskHandle;

    uint32_t notify_value = mp_obj_get_int(in_value);
    if (notify_value == 0) return mp_const_false;

    if (mp_thread_notify((void *)thr_id, notify_value)) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_thread_ipc_notify_obj, mod_thread_ipc_notify);


//=================================================================
STATIC const mp_rom_map_elem_t mp_module_thread_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),			MP_ROM_QSTR(MP_QSTR__thread) },
    { MP_ROM_QSTR(MP_QSTR_stack_size),			MP_ROM_PTR(&mod_thread_stack_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_new_thread),	MP_ROM_PTR(&mod_thread_start_new_thread_obj) },
    { MP_ROM_QSTR(MP_QSTR_allowsuspend),		MP_ROM_PTR(&mod_thread_allowsuspend_obj) },
    { MP_ROM_QSTR(MP_QSTR_suspend),				MP_ROM_PTR(&mod_thread_suspend_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume),				MP_ROM_PTR(&mod_thread_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),				MP_ROM_PTR(&mod_thread_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_notify),				MP_ROM_PTR(&mod_thread_notify_obj) },
    { MP_ROM_QSTR(MP_QSTR_getnotification),		MP_ROM_PTR(&mod_thread_getnotify_obj) },
    { MP_ROM_QSTR(MP_QSTR_isnotified),			MP_ROM_PTR(&mod_thread_isnotified_obj) },
    { MP_ROM_QSTR(MP_QSTR_getMainID),			MP_ROM_PTR(&mod_thread_getMAINId_obj) },
    { MP_ROM_QSTR(MP_QSTR_getMain2ID),          MP_ROM_PTR(&mod_thread_getMAIN2Id_obj) },
    { MP_ROM_QSTR(MP_QSTR_mainAcceptMsg),		MP_ROM_PTR(&mod_thread_mainAcceptMsg_obj) },
    { MP_ROM_QSTR(MP_QSTR_sendmsg),				MP_ROM_PTR(&mod_thread_sendmsg_obj) },
    { MP_ROM_QSTR(MP_QSTR_getmsg),				MP_ROM_PTR(&mod_thread_getmsg_obj) },
    { MP_ROM_QSTR(MP_QSTR_list),				MP_ROM_PTR(&mod_thread_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_getThreadName),		MP_ROM_PTR(&mod_thread_getname_obj) },
    { MP_ROM_QSTR(MP_QSTR_getSelfName),			MP_ROM_PTR(&mod_thread_getSelfname_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),				MP_ROM_PTR(&mod_thread_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_wait),				MP_ROM_PTR(&mod_thread_waitnotify_obj) },
    { MP_ROM_QSTR(MP_QSTR_lock),				MP_ROM_PTR(&mod_thread_lock_obj) },
    { MP_ROM_QSTR(MP_QSTR_unlock),				MP_ROM_PTR(&mod_thread_unlock_obj) },
    { MP_ROM_QSTR(MP_QSTR_getPriority),         MP_ROM_PTR(&mod_thread_getpriority_obj) },
    { MP_ROM_QSTR(MP_QSTR_setPriority),         MP_ROM_PTR(&mod_thread_setpriority_obj) },
    { MP_ROM_QSTR(MP_QSTR_pystack),             MP_ROM_PTR(&mod_thread_pystack_obj) },
    { MP_ROM_QSTR(MP_QSTR_mainCallback),        MP_ROM_PTR(&mod_thread_set_callback_obj) },

    { MP_ROM_QSTR(MP_QSTR_ipc_command),         MP_ROM_PTR(&mod_thread_ipc_command_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_msg),             MP_ROM_PTR(&mod_thread_ipc_msg_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_getout),          MP_ROM_PTR(&mod_thread_get_ipc_outbuf_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_clearout),        MP_ROM_PTR(&mod_thread_get_ipc_clearout_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_getoutlen),       MP_ROM_PTR(&mod_thread_get_ipc_outbuflen_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_setvar),          MP_ROM_PTR(&mod_thread_set_ipc_var_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_bussy),           MP_ROM_PTR(&mod_thread_get_ipc_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_break),           MP_ROM_PTR(&mod_thread_get_ipc_setexception_obj) },
    { MP_ROM_QSTR(MP_QSTR_ipc_notify),          MP_ROM_PTR(&mod_thread_ipc_notify_obj) },

    { MP_ROM_QSTR(MP_QSTR_IPC_EXEC),            MP_ROM_INT(THREAD_IPC_TYPE_EXECUTE) },

    // Constants
	{ MP_ROM_QSTR(MP_QSTR_PAUSE),				MP_ROM_INT(THREAD_NOTIFY_PAUSE) },
	{ MP_ROM_QSTR(MP_QSTR_SUSPEND),				MP_ROM_INT(THREAD_NOTIFY_PAUSE) },
	{ MP_ROM_QSTR(MP_QSTR_RESUME),				MP_ROM_INT(THREAD_NOTIFY_RESUME) },
	{ MP_ROM_QSTR(MP_QSTR_EXIT),				MP_ROM_INT(THREAD_NOTIFY_EXIT) },
	{ MP_ROM_QSTR(MP_QSTR_STOP),				MP_ROM_INT(THREAD_NOTIFY_EXIT) },
	{ MP_ROM_QSTR(MP_QSTR_STATUS),				MP_ROM_INT(THREAD_NOTIFY_STATUS) },

	{ MP_ROM_QSTR(MP_QSTR_RUNNING),				MP_ROM_INT(THREAD_STATUS_RUNNING) },
	{ MP_ROM_QSTR(MP_QSTR_SUSPENDED),			MP_ROM_INT(THREAD_STATUS_SUSPENDED) },
	{ MP_ROM_QSTR(MP_QSTR_WAITING),				MP_ROM_INT(THREAD_STATUS_WAITING) },
	{ MP_ROM_QSTR(MP_QSTR_TERMINATED),			MP_ROM_INT(THREAD_STATUS_TERMINATED) },
};
STATIC MP_DEFINE_CONST_DICT(mp_module_thread_globals, mp_module_thread_globals_table);

//========================================
const mp_obj_module_t mp_module_thread = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_thread_globals,
};

#endif // MICROPY_PY_THREAD
