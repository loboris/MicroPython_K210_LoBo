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
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "syslog.h"
#include <projdefs.h>
#include <portmacro.h>
#include "py/mpconfig.h"

#if MICROPY_PY_THREAD

#include "py/mpstate.h"
#include "py/gc.h"
#include "py/mpthread.h"
#include "py/mphal.h"
#include "mpthreadport.h"
#include "mphalport.h"
#include "modmachine.h"
#include "gccollect.h"
#if MICROPY_ENABLE_PYSTACK
#include "py/pystack.h"
#endif

extern int MainTaskProc;

TaskHandle_t MainTaskHandle = NULL;
TaskHandle_t MainTaskHandle2 = NULL;

uint8_t main_accept_msg = 1;

// the mutex controls access to the linked list
STATIC mp_thread_mutex_t thread_mutex;
STATIC thread_t thread_entry0;

// === Linked list of all created threads ===
// thread always points to the last created thread or NULL if no threads are created
// thread.next points to the previously created thread in the linked list
// root pointer, handled by mp_thread_gc_others
STATIC thread_t *thread = NULL;


// === Initialize the main MicroPython thread ===
// this is only called once, from 'main.c'
//--------------------------------------------------------------------------------------
void mp_thread_preinit(void *stack, uint32_t stack_len, void *pystack, int pystack_size)
{
    // Initialize threads mutex
    mp_thread_mutex_init(&thread_mutex);

    mp_thread_set_state(&mp_state_ctx.thread);
    #if MICROPY_ENABLE_PYSTACK
    mp_pystack_init(pystack, pystack + pystack_size);
    #endif

    // ** create the first entry in linked list of all threads
    thread = &thread_entry0;
    memset(thread, 0, sizeof(thread_t));

    thread->id = xTaskGetCurrentTaskHandle();
    thread->state_thread = &mp_state_ctx.thread;
    thread->ready = 1;
    thread->arg = NULL;
    thread->stack = stack;
    thread->curr_sp = stack+stack_len;
    thread->stack_len = stack_len;
    thread->pystack = pystack;
    thread->pystack_size = pystack_size;
    sprintf(thread->name, "MainThread");
    thread->threadQueue = xQueueCreate( THREAD_QUEUE_MAX_ITEMS, sizeof(thread_msg_t) );
    thread->processor = MainTaskProc;
    thread->allow_suspend = 0;
    thread->suspended = 0;
    thread->waiting = 0;
    thread->locked = false;
    thread->priority = MICROPY_TASK_PRIORITY;
    thread->deleted = 0;
    thread->notifyed = 0;
    thread->type = THREAD_TYPE_MAIN;
    thread->next = NULL;
    MainTaskHandle = thread->id;
}

//-------------------------
int mp_thread_gc_others() {
    int n_th = 0;
    void **ptrs;
    mp_state_thread_t *state;

    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (!th->ready) continue;                               // thread not ready
        if (th->type == THREAD_TYPE_SERVICE) continue;          // Only scan PYTHON threads
        if (th->id == xTaskGetCurrentTaskHandle()) continue;    // Do not process the running thread

        state = (mp_state_thread_t *)th->state_thread;
        n_th++;

        // Mark the root pointers on thread
        gc_collect_root((void **)state->dict_locals, 1);

        if (th->arg) {
            // Mark the pointers on thread arguments
            ptrs = (void**)(void*)&th->arg;
            gc_collect_root(ptrs, 1);
        }

        #if MICROPY_ENABLE_PYSTACK
        // Mark the pointers on thread pystack
        ptrs = (void**)(void*)state->pystack_start;
        gc_collect_root(ptrs, (state->pystack_cur - state->pystack_start) / sizeof(void*));
        #endif

        // If PyStack is used, no pointers to MPy heap are placed on tasks stack
        #if !MICROPY_ENABLE_PYSTACK
        // Mark the pointers on thread stack
        gc_collect_root(th->curr_sp, ((void *)state->stack_top - th->curr_sp) / sizeof(void*)); // probably not needed
        #endif
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return n_th;
}

//--------------------------------------------
mp_state_thread_t *mp_thread_get_state(void) {
    return pvTaskGetThreadLocalStoragePointer(NULL, 0);
}

//-------------------------------------
void mp_thread_set_state(void *state) {
    vTaskSetThreadLocalStoragePointer(NULL, 0, state);
}

//--------------------------
void mp_thread_start(void) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            th->ready = 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

//------------------------------------
int mp_thread_started(TaskHandle_t id)
{
    int res = -1;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            res = 0;
            if (th->ready != 0) {
                res = 1;
                break;
            }
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

STATIC void *(*ext_thread_entry)(void*) = NULL;


extern void mp_thread_entry(void *args_in);
/*
//-----------------------------------
STATIC void freertos_entry(void *arg)
{
    mp_thread_entry(arg);
    vTaskDelete(NULL);
}
*/
//----------------------------
void mp_thread_remove_thread()
{
    thread_t *prev = NULL;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; prev = th, th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            // unlink the node from the list
            if (prev != NULL) {
                prev->next = th->next;
            } else {
                // move the start pointer
                thread = th->next;
            }
            free(th);
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

//---------------------------
void mp_thread_finish(void) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            if (th->threadQueue) {
                // Free thread queue
                int n = 1;
                while (n > 0) {
                    n = uxQueueMessagesWaiting(th->threadQueue);
                    if (n > 0) {
                        thread_msg_t msg;
                        xQueueReceive(th->threadQueue, &msg, 0);
                        if (msg.strdata != NULL) free(msg.strdata);
                    }
                }
                if (th->threadQueue) vQueueDelete(th->threadQueue);
                th->threadQueue = NULL;
            }
            #if MICROPY_ENABLE_PYSTACK
            mp_state_thread_t *state = (mp_state_thread_t *)th->state_thread;
            if (state->pystack_start != NULL) {
                // Free PyStack
                free(state->pystack_start);
                th->pystack = NULL;
            }
            #endif
            th->ready = 0;
            th->deleted = 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------
static TaskHandle_t mp_thread_create_ex(void *entry, void *arg, size_t task_stack_size, size_t pystack_size, int priority, char *name, bool same_core)
{
	// store thread entry function into a global variable so we can access it
    ext_thread_entry = entry;

    // Check thread stack sizes
    if (task_stack_size == 0) {
    	task_stack_size = MP_THREAD_DEFAULT_STACK_SIZE; //use default stack size
    }
    else {
        if (task_stack_size < MP_THREAD_MIN_TASK_STACK_SIZE) task_stack_size = MP_THREAD_MIN_TASK_STACK_SIZE;
        else if (task_stack_size > MP_THREAD_MAX_TASK_STACK_SIZE) task_stack_size = MP_THREAD_MAX_TASK_STACK_SIZE;
    }
    task_stack_size &= 0x7FFFFFF8;

    #if MICROPY_ENABLE_PYSTACK
    if (pystack_size == 0) {
        pystack_size = MP_THREAD_DEFAULT_STACK_SIZE; //use default stack size
    }
    else {
        if (pystack_size < MP_THREAD_MIN_STACK_SIZE) pystack_size = MP_THREAD_MIN_STACK_SIZE;
        else if (pystack_size > MP_THREAD_MAX_STACK_SIZE) pystack_size = MP_THREAD_MAX_STACK_SIZE;
    }
    pystack_size &= 0x7FFFFFF8;
    #else
    pystack_size = 0;
    #endif

    // Allocate linked-list node
    thread_t *th = NULL;
    void *th_stack = NULL;

    #if MICROPY_ENABLE_PYSTACK
    th_stack = malloc(pystack_size);
    if (th_stack == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating PyStack"));
    }
    #endif

	th = malloc(sizeof(thread_t));
    if (th == NULL) {
        if (th_stack) free(th_stack);
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "can't create thread (th)"));
    }
    memset(th, 0, sizeof(thread_t));

    mp_thread_mutex_lock(&thread_mutex, 1);

    // adjust the task_stack_size to provide room to recover from hitting the limit
    //task_stack_size -= 1024;

    int run_on_core = MainTaskProc & 1;
    if (!same_core) run_on_core ^= 1;
    // -----------------------------------------------
    // ** add thread to the linked list of all threads
    // -----------------------------------------------
    th->ready = 0;
    th->arg = arg;
    th->pystack = th_stack;
    th->pystack_size = pystack_size;
    th->stack_len = task_stack_size*sizeof(StackType_t) - 1024;
    th->next = thread;
    snprintf(th->name, THREAD_NAME_MAX_SIZE, name);
    th->threadQueue = xQueueCreate( THREAD_QUEUE_MAX_ITEMS, sizeof(thread_msg_t) );
    th->processor = run_on_core;
    th->allow_suspend = 1;
    th->suspended = 0;
    th->waiting = 0;
    th->locked = false;
    th->priority = priority;
    th->deleted = 0;
    th->notifyed = 0;
    th->type = THREAD_TYPE_PYTHON;
    thread = th;

    // === Create and start the thread task ===
    TaskHandle_t id = NULL;

    //if (run_on_core != MainTaskProc) {
        /*
        //ToDo: Workaround for issue with creating the task on different processor
        create_task_params_t params;
        params.uxProcessor = run_on_core;
        params.pxTaskCode = mp_thread_entry;
        snprintf(params.pcName, THREAD_NAME_MAX_SIZE, name);
        params.usStackDepth = task_stack_size;
        params.pvParameters = arg;
        params.uxPriority = priority;
        params.pxCreatedTask = (TaskHandle_t *)&id;
        xTaskNotify(mp_hal_tick_handle, (uint64_t )&params, eSetValueWithOverwrite);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        */
    //}
    //else {
        BaseType_t res = xTaskCreateAtProcessor(
                    run_on_core,            // processor
                    //freertos_entry,         // function entry
                    mp_thread_entry,        // function entry
                    th->name,               // task name
                    task_stack_size,        // stack_deepth
                    arg,                    // function argument
                    priority,               // task priority
                    (TaskHandle_t *)&id);   // task handle
        if (res != pdPASS) id = NULL;
    //}

    if (id == NULL) {
        // Task not started, restore previous thread and clean-up
        thread = th->next;
        if (th->threadQueue) vQueueDelete(th->threadQueue);
        free(th);
        if (th_stack) free(th_stack);
        mp_thread_mutex_unlock(&thread_mutex);
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error creating thread"));
    }
    th->id = id;

    mp_thread_mutex_unlock(&thread_mutex);

    // wait for thread to start
    int th_started = 0;
    int tmo = 20;
    while ( th_started != 1) {
        tmo--;
        if (tmo == 0) break;
        vTaskDelay(5 / portTICK_PERIOD_MS);
        th_started = mp_thread_started(id);
    }

    if (th_started != 1) {
        LOGE("[THREAD]", "New thread not started");
    }
    return id;
}

//-----------------------------------------------------------------------------------------------------------------------------------
void *mp_thread_create(void *entry, void *arg, size_t task_stack_size, size_t pystack_size, char *name, bool same_core, int priority)
{
    int task_priority = MP_THREAD_PRIORITY;
    if (priority) task_priority = priority;
    return mp_thread_create_ex(entry, arg, task_stack_size, pystack_size, task_priority, name, same_core);
}

//---------------------------------------------------
void mp_thread_mutex_init(mp_thread_mutex_t *mutex) {
    mutex->handle = xSemaphoreCreateMutexStatic(&mutex->buffer);
}

//------------------------------------------------------------
int mp_thread_mutex_lock(mp_thread_mutex_t *mutex, int wait) {
    return (pdTRUE == xSemaphoreTake(mutex->handle, wait ? portMAX_DELAY : 0));
}

//-----------------------------------------------------
void mp_thread_mutex_unlock(mp_thread_mutex_t *mutex) {
    xSemaphoreGive(mutex->handle);
}

//--------------------------------------
void mp_thread_allowsuspend(int allow) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        // don't allow suspending main task task
        if ((th->id != MainTaskHandle) && (th->id == xTaskGetCurrentTaskHandle())) {
        	th->allow_suspend = allow & 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

//--------------------------------------
int mp_thread_suspend(TaskHandle_t id) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        // don't suspend the current task
        if (th->id == xTaskGetCurrentTaskHandle()) {
            continue;
        }
        if (th->id == id) {
        	if ((th->allow_suspend) && (th->suspended == 0) && (th->waiting == 0)) {
        		th->suspended = 1;
        		vTaskSuspend(th->id);
        		res = 1;
        	}
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//-------------------------------------
int mp_thread_resume(TaskHandle_t id) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        // don't resume the current task
        if (th->id == xTaskGetCurrentTaskHandle()) {
            continue;
        }
        if (th->id == id) {
        	if ((th->allow_suspend) && (th->suspended) && (th->waiting == 0)) {
        		th->suspended = 0;
        		vTaskResume(th->id);
        		res = 1;
        	}
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//--------------------------
int mp_thread_setblocked() {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
			th->waiting = 1;
			res = 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//------------------------------
int mp_thread_suspend_others() {
    int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            // lock the current task
            th->locked = true;
        }
        else if ((th->allow_suspend) && (th->suspended == 0) && (th->waiting == 0)) {
            th->suspended = 1;
            vTaskSuspend(th->id);
            res++;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//-----------------------------
int mp_thread_resume_others() {
    int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            // unlock the current task
            th->locked = false;
        }
        else if ((th->allow_suspend) && (th->suspended) && (th->waiting == 0)) {
            th->suspended = 0;
            vTaskResume(th->id);
            res++;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//-----------------------
bool mp_thread_locked() {
    bool res = false;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            res = th->locked;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//---------------------------------------------------------------
thread_t *mp_thread_get_thread(TaskHandle_t id, thread_t *self) {
    thread_t *res_th = NULL;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            memcpy(self, th, sizeof(thread_t));
            res_th = th;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res_th;
}

//----------------------------------------------------------
void mp_thread_set_priority(TaskHandle_t id, int priority) {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            th->priority = priority;
            vTaskPrioritySet(th->id, priority);
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

//-------------------------------------------
int mp_thread_get_priority(TaskHandle_t id) {
    int res = -1;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            res = th->priority;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//-------------------------------------------
int mp_thread_set_sp(void *stack, int size) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            th->stack = stack;
            th->stack_len = size;
			th->curr_sp = stack+size;
			res = 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//--------------------------
int mp_thread_get_sp(void) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
			res = (uintptr_t)th->curr_sp;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//----------------------------
void mp_thread_set_pystack() {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            #if MICROPY_ENABLE_PYSTACK
            mp_pystack_init(th->pystack, th->pystack+th->pystack_size);
            #endif
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

//---------------------------------------
int mp_thread_set_th_state(void *state) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
			th->state_thread = state;
			res = 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//---------------------------------------------------
thread_t *mp_thread_get_th_from_id(TaskHandle_t id) {
    thread_t *self = NULL;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            self = th;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return self;
}

//-----------------------------
int mp_thread_setnotblocked() {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
        	th->waiting = 0;
        	res = 1;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

// Send notification to thread 'id'
// or to all threads if id=0
//-----------------------------------------------------
int mp_thread_notify(TaskHandle_t id, uint32_t value) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if ( (th->id != xTaskGetCurrentTaskHandle()) && ( (id == 0) || (th->id == id) ) ) {
        	res = xTaskNotify(th->id, value, eSetValueWithOverwrite); //eSetValueWithoutOverwrite
        	th->notifyed = 1;
            if (id != 0) break;
        }
    }
    if (id == 0) res = 1;
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//---------------------------
int mp_thread_num_threads() {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id != xTaskGetCurrentTaskHandle()) res++;
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//---------------------------------------------
uint32_t mp_thread_getnotify(bool check_only) {
	uint64_t value = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
      		if (!check_only) {
      			xTaskNotifyWait(0, ULLONG_MAX, &value, 0);
          		th->notifyed = 0;
      		}
      		else xTaskNotifyWait(0, 0, &value, 0);
			break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return value;
}

//--------------------------------------------
int mp_thread_notifyPending(TaskHandle_t id) {
	int res = -1;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
        	res = th->notifyed;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//-----------------------------
void mp_thread_resetPending() {
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
        	th->notifyed = 0;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
}

//------------------------------
uint64_t mp_thread_getSelfID() {
	uint32_t id = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
        	id = (uint64_t)th->id;
			break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return id;
}

//------------------------------
uint8_t mp_thread_getSelfIdx() {
    uint32_t idx = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            break;
        }
        idx++;
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return idx;
}

//-------------------------------------
int mp_thread_getSelfname(char *name) {
	int res = 0;
	name[0] = '?';
	name[1] = '\0';
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
        	sprintf(name, th->name);
        	res = 1;
			break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//--------------------------------------------------
int mp_thread_getname(TaskHandle_t id, char *name) {
	int res = 0;
	name[0] = '?';
	name[1] = '\0';
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
        	sprintf(name, th->name);
        	res = 1;
			break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//-------------------------------------------------------------------------------------------------
int mp_thread_semdmsg(TaskHandle_t id, int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        // don't send to the current task or service thread
        if ((th->id == xTaskGetCurrentTaskHandle()) || (th->type == THREAD_TYPE_SERVICE)) {
            continue;
        }
        if ((id == 0) || (th->id == id)) {
        	if (th->threadQueue == NULL) break;
    		thread_msg_t msg;
    	    struct timeval tv;
    	    uint64_t tmstamp;
    	    gettimeofday(&tv, NULL);
    	    tmstamp = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
    	    msg.timestamp = tmstamp;
			msg.sender_id = xTaskGetCurrentTaskHandle();
			if (type == THREAD_MSG_TYPE_INTEGER) {
				msg.intdata = msg_int;
				msg.strdata = NULL;
				msg.type = type;
				res = 1;
			}
			else if (type == THREAD_MSG_TYPE_STRING) {
				msg.intdata = buflen;
				msg.strdata = malloc(buflen+1);
				if (msg.strdata != NULL) {
					memcpy(msg.strdata, buf, buflen);
					msg.strdata[buflen] = 0;
					msg.type = type;
					res = 1;
				}
			}
			if (res) {
				if (xQueueSend(th->threadQueue, &msg, 0) != pdTRUE) res = 0;
			}
            if (id != 0) break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//------------------------------------------------------------------------------------------
int mp_thread_getmsg(uint32_t *msg_int, uint8_t **buf, uint32_t *buflen, uint64_t *sender) {
	int res = 0;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        // get message for current task
        if ((th->id == xTaskGetCurrentTaskHandle()) && (th->type != THREAD_TYPE_SERVICE)) {
        	if (th->threadQueue == NULL) break;

        	thread_msg_t msg;
        	if (xQueueReceive(th->threadQueue, &msg, 0) == pdTRUE) {
        		*sender = (uint64_t)msg.sender_id;
        		if (msg.type == THREAD_MSG_TYPE_INTEGER) {
        			*msg_int = msg.intdata;
        			*buflen = 0;
        			res = THREAD_MSG_TYPE_INTEGER;
        		}
        		else if (msg.type == THREAD_MSG_TYPE_STRING) {
        			*msg_int = 0;
        			if ((msg.strdata != NULL) && (msg.intdata > 0)) {
            			*buflen = msg.intdata;
            			*buf = msg.strdata;
            			res = THREAD_MSG_TYPE_STRING;
        			}
        		}
        	}
        	break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);

    return res;
}

//-------------------------------------
int mp_thread_status(TaskHandle_t id) {
	int res = -1;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if ((th->id == xTaskGetCurrentTaskHandle()) || (th->type == THREAD_TYPE_SERVICE)) {
            continue;
        }
        if (th->id == id) {
			if (!th->deleted) {
				if (th->suspended) res = 1;
				else if (th->waiting) res = 2;
				else res = 0;
			}
			break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//------------------------------
int mp_thread_stack_max_used() {
    int res = -1;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            res = th->stack_len - uxTaskGetStackHighWaterMark(NULL);
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//------------------------------
int mp_thread_stack_get_size() {
    int res = -1;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            res = th->stack_len;
            break;
        }
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return res;
}

//---------------------------------------
int mp_thread_list(thread_list_t *list) {
	int num = 0;

    mp_thread_mutex_lock(&thread_mutex, 1);

    for (thread_t *th = thread; th != NULL; th = th->next) {
    	num++;
    }
    if ((num == 0) || (list == NULL)) {
        mp_thread_mutex_unlock(&thread_mutex);
    	return num;
    }

	list->nth = num;
	list->threads = malloc(sizeof(threadlistitem_t) * (num+1));
	if (list->threads == NULL) num = 0;
	else {
		int nth = 0;
		threadlistitem_t *thr = NULL;
		uint32_t min_stack;
		for (thread_t *th = thread; th != NULL; th = th->next) {
			thr = list->threads + (sizeof(threadlistitem_t) * nth);
	        if (th->id == xTaskGetCurrentTaskHandle()) min_stack = uxTaskGetStackHighWaterMark(NULL);
	        else min_stack = uxTaskGetStackHighWaterMark(th->id);
	        min_stack *= sizeof(StackType_t);

			thr->id = (uint64_t)th->id;
			sprintf(thr->name, "%s", th->name);
			thr->suspended = th->suspended;
			thr->waiting = th->waiting;
			thr->type = th->type;
			thr->stack_len = th->stack_len;
			thr->stack_max = th->stack_len - min_stack;
			thr->pystack_len = th->pystack_size;
			thr->priority = th->priority;
			nth++;
			if (nth > num) break;
		}
		if (nth != num) {
			free(list->threads);
			list->threads = NULL;
			num = 0;
		}
    }
    mp_thread_mutex_unlock(&thread_mutex);
    return num;
}

//------------------------------------------
int mp_thread_mainAcceptMsg(int8_t accept) {
	int res = main_accept_msg;
    mp_thread_mutex_lock(&thread_mutex, 1);
    for (thread_t *th = thread; th != NULL; th = th->next) {
    	 if (th->id == xTaskGetCurrentTaskHandle()) {
    		 if ((th->id == MainTaskHandle) && (accept >= 0)) {
    			 main_accept_msg = accept & 1;
    		 }
			 break;
    	 }
    }
    mp_thread_mutex_unlock(&thread_mutex);

    return res;
}

#endif
