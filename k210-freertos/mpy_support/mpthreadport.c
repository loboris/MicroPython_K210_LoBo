/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, vPortFree of charge, to any person obtaining a copy
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
#include "py/pystack.h"

TaskHandle_t MainTaskHandle = NULL;
TaskHandle_t MainTaskHandle2 = NULL;

uint8_t main_accept_msg = 1;

// === Linked list of all created threads ===
// thread always points to the last created thread or NULL if no threads are created
// thread.next points to the previously created thread in the linked list
// root pointer, handled by mp_thread_gc_others
static thread_t *thread = NULL;
static thread_t *thread0 = NULL;

// the mutex controls access to the linked list
static SemaphoreHandle_t thread_mutex = NULL;
thread_t thread_entry0;

static thread_t *thread1 = NULL;
static SemaphoreHandle_t thread_mutex2 = NULL;
thread_t thread_entry2;
mp_state_ctx_t mp_state_ctx2 = { 0 };

extern void mp_thread_entry(void *args_in);


// === Initialize the main MicroPython thread ===
// this is only called once, from 'main.c'
//-----------------------------------------------------------------------------------------------------
void mp_thread_preinit(void *stack, uint32_t stack_len, void *pystack, int pystack_size, int task_proc)
{
    // Initialize threads mutex and create thread local storage pointers
    if (mpy_config.config.use_two_main_tasks) {
        if (task_proc == MAIN_TASK_PROC) {
            thread_mutex = xSemaphoreCreateMutex();
            configASSERT(thread_mutex);

            // ** create the first entry in linked list of all threads
            thread = &thread_entry0;
            thread0 = &thread_entry0;
        }
        else {
            thread_mutex2 = xSemaphoreCreateMutex();
            configASSERT(thread_mutex2);

            // ** create the first entry in linked list of all threads
            thread = &thread_entry2;
            thread1 = &thread_entry2;
        }
    }
    else {
        thread_mutex = xSemaphoreCreateMutex();
        configASSERT(thread_mutex);

        // ** create the first entry in linked list of all threads
        thread = &thread_entry0;
        thread0 = &thread_entry0;
    }
    memset(thread, 0, sizeof(thread_t));

    thread->id = xTaskGetCurrentTaskHandle();

    // Save thread state in local storage pointer #0
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, (task_proc == MAIN_TASK_PROC) ? &mp_state_ctx : &mp_state_ctx2);

    // Initialize PyStack
    mp_pystack_init(pystack, pystack + pystack_size, mpy_config.config.pystack_enabled);

    // Save the thread arguments in local storage pointer #1
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, NULL);
    thread->ready = 1;
    sprintf(thread->name, "MainThread%s", (task_proc == MAIN_TASK_PROC) ? "" : "2");
    thread->threadQueue = xQueueCreate( THREAD_QUEUE_MAX_ITEMS, sizeof(thread_msg_t) );
    thread->processor = task_proc;
    thread->allow_suspend = 0;
    thread->suspended = 0;
    thread->waiting = 0;
    thread->locked = false;
    thread->priority = MICROPY_TASK_PRIORITY;
    thread->deleted = 0;
    thread->notifyed = 0;
    thread->type = THREAD_TYPE_MAIN;
    thread->next = NULL;
    if (mpy_config.config.use_two_main_tasks) {
        if (task_proc == MAIN_TASK_PROC) MainTaskHandle = thread->id;
        else MainTaskHandle2 = thread->id;
    }
    else MainTaskHandle = thread->id;
}

//--------------------------------
static void mp_lock_thread_mutex()
{
    if (mpy_config.config.use_two_main_tasks) {
        if (uxPortGetProcessorId() == MAIN_TASK_PROC) {
            xSemaphoreTake(thread_mutex, portMAX_DELAY);
            thread = thread0;
        }
        else {
            xSemaphoreTake(thread_mutex2, portMAX_DELAY);
            thread = thread1;
        }
    }
    else {
        xSemaphoreTake(thread_mutex, portMAX_DELAY);
        thread = thread0;
    }
}

//----------------------------------
static void mp_unlock_thread_mutex()
{
    if (mpy_config.config.use_two_main_tasks) {
        if (uxPortGetProcessorId() == MAIN_TASK_PROC) {
            thread0 = thread;
            if (thread_mutex) xSemaphoreGive(thread_mutex);
        }
        else {
            thread1 = thread;
            if (thread_mutex2) xSemaphoreGive(thread_mutex2);
        }
    }
    else {
        thread0 = thread;
        xSemaphoreGive(thread_mutex);
    }
}

// Returns the pointer to the MP state for the current thread
//--------------------------------
mp_state_ctx_t *mp_get_state(void)
{
    return (mp_state_ctx_t *)pvTaskGetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE);
}

//------------------------------
void mp_set_state(void *state) {
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE, state);
}

//---------------------
void *mp_get_args(void)
{
    return pvTaskGetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS);
}

//----------------------------
void mp_set_args(void *args) {
    vTaskSetThreadLocalStoragePointer(NULL, THREAD_LSP_ARGS, args);
}

//-------------------------
int mp_thread_gc_others() {
    int n_th = 0;
    void **ptrs;
    size_t root_start, root_end;
    mp_state_ctx_t *state;

    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (!th->ready) continue;                               // thread not ready
        if (th->id == xTaskGetCurrentTaskHandle()) continue;    // Do not process the running thread
        // Only scan PYTHON threads and the main thread
        if ((th->type != THREAD_TYPE_PYTHON) && (th->type != THREAD_TYPE_MAIN)) continue;

        DEBUG_GC_printf("\r\n[GC_COLLECT] Others - %s\r\n", th->name);

        // get the thread's MP state
        state = (mp_state_ctx_t *)pvTaskGetThreadLocalStoragePointer(th->id, THREAD_LSP_STATE);

        // === Mark the root pointers on thread
        ptrs = (void**)(void*)state;
        root_start = (void *)&state->thread.dict_locals - (void *)state;
        root_end = (void *)&state->vm.qstr_last_chunk - (void *)state;
        DEBUG_GC_printf("  root pointers (%p [%p]) [%lu,%lu]\r\n", state, ptrs + root_start / sizeof(void*), root_start, root_end);
        gc_collect_root(ptrs + root_start / sizeof(void*), (root_end - root_start) / sizeof(void*));

        void *args = pvTaskGetThreadLocalStoragePointer(th->id, THREAD_LSP_ARGS);
        if (args) {
            DEBUG_GC_printf("  args\r\n");
            // Mark the pointers on thread arguments
            ptrs = (void**)(void*)args;
            gc_collect_root(ptrs, 1);
        }

        if (mpy_config.config.pystack_enabled) {
            // === Mark the pointers on thread pystack
            ptrs = (void**)(void*)state->thread.pystack_start;
            gc_collect_root(ptrs, (state->thread.pystack_cur - state->thread.pystack_start) / sizeof(void*));
        }

        // === Mark the pointers on thread stack
        DEBUG_GC_printf("  stack\r\n");
        void *sp = (void *)pxTaskGetStackTop(th->id);
        ptrs = (void**)(void*)sp;
        gc_collect_root(ptrs, ((void *)state->thread.stack_top - sp) / sizeof(void*));
    }
    mp_unlock_thread_mutex();
    return n_th;
}

//------------------------------------
int mp_thread_started(TaskHandle_t id)
{
    int res = -1;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            res = th->ready;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//----------------------------
void mp_thread_remove_thread()
{
    // Start scanning the threads with the LATEST created thread
    // 'th->next' is the pointer to the PREVIOUSLY created thread !
    thread_t *prev = NULL;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            // === Remove the current thread if it is not the base (MicroPython) thread ===
            if (th == thread) {
                // this is the last thread in the linked list of all threads
                // make the previous thread the last one
                thread = th->next;
                vPortFree(th);
                break;
            }
            else {
                // unlink the node from the list
                if (prev != NULL) {
                    // set the previous's thread next to this thread's next
                    prev->next = th->next;
                }
                else {
                    // move the start pointer
                    thread = th->next;
                }
                vPortFree(th);
                break;
            }
        }
        prev = th;
    }
    mp_unlock_thread_mutex();
}

//---------------------------
void mp_thread_finish(void) {
    mp_lock_thread_mutex();
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
                        if (msg.strdata != NULL) vPortFree(msg.strdata);
                    }
                }
                if (th->threadQueue) vQueueDelete(th->threadQueue);
                th->threadQueue = NULL;
            }
            if (mpy_config.config.pystack_enabled) {
                mp_state_ctx_t *state = (mp_state_ctx_t *)pvTaskGetThreadLocalStoragePointer(th->id, THREAD_LSP_STATE);
                if (state->thread.pystack_start != NULL) {
                    // Free PyStack
                    vPortFree(state->thread.pystack_start);
                }
            }
            th->ready = 0;
            th->deleted = 1;
            break;
        }
    }
    mp_unlock_thread_mutex();
}

//-----------------------------------------------------------------------------------------------------------------------------------------
TaskHandle_t mp_thread_create(void *entry, thread_entry_args_t *arg, size_t task_stack_size, size_t pystack_size, int priority, char *name)
{
    // Check thread stack sizes
    if (task_stack_size == 0) {
    	task_stack_size = MP_THREAD_DEFAULT_STACK_SIZE; //use default stack size
    }
    else {
        if (task_stack_size < MP_THREAD_MIN_STACK_SIZE) task_stack_size = MP_THREAD_MIN_STACK_SIZE;
        else if (task_stack_size > MP_THREAD_MAX_STACK_SIZE) task_stack_size = MP_THREAD_MAX_STACK_SIZE;
    }
    task_stack_size = (task_stack_size / 8) * 8;

    if (mpy_config.config.pystack_enabled) {
        if (pystack_size == 0) {
            pystack_size = MICROPY_PYSTACK_SIZE; //use default PyStackstack size
        }
        else {
            if (pystack_size < MP_THREAD_MIN_PYSTACK_SIZE) pystack_size = MP_THREAD_MIN_PYSTACK_SIZE;
            else if (pystack_size > MP_THREAD_MAX_PYSTACK_SIZE) pystack_size = MP_THREAD_MAX_PYSTACK_SIZE;
        }
        pystack_size = (pystack_size / 8) * 8;
    }
    else {
        pystack_size = 0;
        arg->pystack_start = NULL;
        arg->pystack_end = NULL;
    }

    // Allocate linked-list thread node
    thread_t *th = NULL;
    void *th_stack = NULL;

    if (mpy_config.config.pystack_enabled) {
        th_stack = pvPortMalloc(pystack_size+8);
        if (th_stack == NULL) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error allocating PyStack"));
        }
        memset(th_stack, 0, pystack_size);
        arg->pystack_start = th_stack;
        arg->pystack_end = th_stack + pystack_size;
    }

	th = pvPortMalloc(sizeof(thread_t));
    if (th == NULL) {
        if (th_stack) vPortFree(th_stack);
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "can't create thread (th)"));
    }
    memset(th, 0, sizeof(thread_t));

    //vTaskDelay(10 / portTICK_PERIOD_MS);
    // Lock threads and set 'thread' pointer
    //mp_lock_thread_mutex();
    if (mpy_config.config.use_two_main_tasks) {
        if (uxPortGetProcessorId() == MAIN_TASK_PROC) {
            xSemaphoreTake(thread_mutex, portMAX_DELAY);
        }
        else {
            xSemaphoreTake(thread_mutex2, portMAX_DELAY);
        }
    }
    else xSemaphoreTake(thread_mutex, portMAX_DELAY);

    // adjust the task_stack_size to provide room to recover from hitting the limit
    //task_stack_size -= 1024;

    thread_t *last_th = thread0;
    if (mpy_config.config.use_two_main_tasks) {
        if (uxPortGetProcessorId() != MAIN_TASK_PROC) last_th = thread1;
    }
    // -----------------------------------------------
    // ** add thread to the linked list of all threads
    // -----------------------------------------------
    th->ready = 0;
    th->next = last_th;
    snprintf(th->name, THREAD_NAME_MAX_SIZE, name);
    th->threadQueue = xQueueCreate( THREAD_QUEUE_MAX_ITEMS, sizeof(thread_msg_t) );
    th->processor = uxPortGetProcessorId();
    th->allow_suspend = 1;
    th->suspended = 0;
    th->waiting = 0;
    th->locked = false;
    th->priority = priority;
    th->deleted = 0;
    th->notifyed = 0;
    th->type = THREAD_TYPE_PYTHON;
    // 'thread' now points to the last created thread
    if (mpy_config.config.use_two_main_tasks) {
        if (uxPortGetProcessorId() == MAIN_TASK_PROC) thread0 = th;
        else thread1 = th;
    }
    else thread0 = th;

    thread_entry_args_t *th_args = arg;
    th_args->thread = th;

    // === Create and start the thread task ===
    TaskHandle_t id = NULL;

    BaseType_t res = xTaskCreate(
                mp_thread_entry,        // function entry
                th->name,               // task name
                task_stack_size,        // stack_deepth
                arg,                    // function argument
                priority,               // task priority
                (TaskHandle_t *)&id);   // task handle
    if (res != pdPASS) id = NULL;

    if (id == NULL) {
        // Task not started, restore previous thread and clean-up
        if (mpy_config.config.use_two_main_tasks) {
            if (uxPortGetProcessorId() == MAIN_TASK_PROC) thread0 = last_th;
            else thread1 = last_th;
        }
        else thread0 = last_th;
        if (th->threadQueue) vQueueDelete(th->threadQueue);
        vPortFree(th);
        if (th_stack) vPortFree(th_stack);
        //mp_unlock_thread_mutex();
        if (mpy_config.config.use_two_main_tasks) {
            if (uxPortGetProcessorId() == MAIN_TASK_PROC) {
                if (thread_mutex) xSemaphoreGive(thread_mutex);
            }
            else {
                if (thread_mutex2) xSemaphoreGive(thread_mutex2);
            }
        }
        else xSemaphoreGive(thread_mutex);
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error creating thread"));
    }

    //mp_unlock_thread_mutex();
    if (mpy_config.config.use_two_main_tasks) {
        if (uxPortGetProcessorId() == MAIN_TASK_PROC) {
            if (thread_mutex) xSemaphoreGive(thread_mutex);
        }
        else {
            if (thread_mutex2) xSemaphoreGive(thread_mutex2);
        }
    }
    else xSemaphoreGive(thread_mutex);

    // wait for thread to start
    vTaskDelay(2 / portTICK_PERIOD_MS);

    return id;
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
    taskYIELD();
}

//--------------------------------------
void mp_thread_allowsuspend(int allow) {
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        // don't allow suspending main task task
        if ((th->id != MainTaskHandle) && (th->id == xTaskGetCurrentTaskHandle())) {
        	th->allow_suspend = allow & 1;
            break;
        }
    }
    mp_unlock_thread_mutex();
}

//--------------------------------------
int mp_thread_suspend(TaskHandle_t id) {
	int res = 0;
    mp_lock_thread_mutex();
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
    mp_unlock_thread_mutex();
    return res;
}

//-------------------------------------
int mp_thread_resume(TaskHandle_t id) {
	int res = 0;
    mp_lock_thread_mutex();
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
    mp_unlock_thread_mutex();
    return res;
}

//--------------------------
int mp_thread_setblocked() {
	int res = 0;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
			th->waiting = 1;
			res = 1;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//-----------------------------
int mp_thread_setnotblocked() {
    int res = 0;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            th->waiting = 0;
            res = 1;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//------------------------------
int mp_thread_suspend_others() {
    int res = 0;
    mp_lock_thread_mutex();
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
    mp_unlock_thread_mutex();
    return res;
}

//-----------------------------
int mp_thread_resume_others() {
    int res = 0;
    mp_lock_thread_mutex();
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
    mp_unlock_thread_mutex();
    return res;
}

//-----------------------
bool mp_thread_locked() {
    bool res = false;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            res = th->locked;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//---------------------------------------------------------------
thread_t *mp_thread_get_thread(TaskHandle_t id, thread_t *self) {
    thread_t *res_th = NULL;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            memcpy(self, th, sizeof(thread_t));
            res_th = th;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res_th;
}

//---------------------------------------------------------
int mp_thread_set_priority(TaskHandle_t id, int priority) {
    int res = -1;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            res = th->priority;
            th->priority = priority;
            vTaskPrioritySet(th->id, priority);
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//-------------------------------------------
int mp_thread_get_priority(TaskHandle_t id) {
    int res = -1;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            res = th->priority;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//---------------------------------------------------
thread_t *mp_thread_get_th_from_id(TaskHandle_t id) {
    thread_t *self = NULL;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            self = th;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return self;
}

// Send notification to thread 'id'
// or to all threads if id=0
//-----------------------------------------------------
int mp_thread_notify(TaskHandle_t id, uint32_t value) {
	int res = 0;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if ( (th->id != xTaskGetCurrentTaskHandle()) && ( (id == 0) || (th->id == id) ) ) {
        	res = xTaskNotify(th->id, value, eSetValueWithOverwrite); //eSetValueWithoutOverwrite
        	th->notifyed = 1;
            if (id != 0) break;
        }
    }
    if (id == 0) res = 1;
    mp_unlock_thread_mutex();
    return res;
}

//---------------------------
int mp_thread_num_threads() {
	int res = 0;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id != xTaskGetCurrentTaskHandle()) res++;
    }
    mp_unlock_thread_mutex();
    return res;
}

//---------------------------------------------
uint32_t mp_thread_getnotify(bool check_only) {
	uint64_t value = 0;
    mp_lock_thread_mutex();
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
    mp_unlock_thread_mutex();
    return value;
}

//--------------------------------------------
int mp_thread_notifyPending(TaskHandle_t id) {
	int res = -1;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
        	res = th->notifyed;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//-----------------------------
void mp_thread_resetPending() {
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
        	th->notifyed = 0;
            break;
        }
    }
    mp_unlock_thread_mutex();
}

//------------------------------
uint64_t mp_thread_getSelfID() {
	uint32_t id = 0;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
        	id = (uint64_t)th->id;
			break;
        }
    }
    mp_unlock_thread_mutex();
    return id;
}

//------------------------------
uint8_t mp_thread_getSelfIdx() {
    uint32_t idx = 0;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
            break;
        }
        idx++;
    }
    mp_unlock_thread_mutex();
    return idx;
}

//-------------------------------------
int mp_thread_getSelfname(char *name) {
	int res = 0;
	name[0] = '?';
	name[1] = '\0';
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == xTaskGetCurrentTaskHandle()) {
        	sprintf(name, th->name);
        	res = 1;
			break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//--------------------------------------------------
int mp_thread_getname(TaskHandle_t id, char *name) {
	int res = 0;
	name[0] = '?';
	name[1] = '\0';
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
        	sprintf(name, th->name);
        	res = 1;
			break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//------------------------------------------------------------------------------------------
static int _sendmsg(thread_t *th, int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen)
{
    int res = 0;
    if (th->threadQueue) {
        thread_msg_t msg;
        msg.timestamp = mp_hal_ticks_ms();
        msg.sender_id = xTaskGetCurrentTaskHandle();
        msg.intdata = msg_int;
        msg.strdata = NULL;
        msg.strlen = 0;
        msg.type = type;
        if (type == THREAD_MSG_TYPE_INTEGER) res = 1;
        else if (type == THREAD_MSG_TYPE_STRING) {
            if (buf) {
                msg.strdata = pvPortMalloc(buflen+1);
                if (msg.strdata != NULL) {
                    msg.strlen = buflen;
                    memcpy(msg.strdata, buf, buflen);
                    msg.strdata[buflen] = 0;
                    res = 1;
                }
            }
        }
        if (res) {
            if (xQueueSend(th->threadQueue, &msg, 0) != pdTRUE) res = 0;
        }
    }
    return res;
}

//--------------------------------------------------------------------------------------
int mp_thread_sendmsg_to_mpy2(int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen)
{
    return _sendmsg(&thread_entry2, type, msg_int, buf, buflen);
}

//--------------------------------------------------------------------------------------
int mp_thread_sendmsg_to_mpy1(int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen)
{
    return _sendmsg(&thread_entry0, type, msg_int, buf, buflen);
}

//-------------------------------------------------------------------------------------------------
int mp_thread_semdmsg(TaskHandle_t id, int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen) {
	int nsent = 0;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        // don't send to the current task or service thread
        if ((th->id == xTaskGetCurrentTaskHandle()) || (th->type == THREAD_TYPE_SERVICE)) {
            continue;
        }
        // If id is zero, sent to all threads
        if ((id == 0) || (th->id == id)) {
      	    nsent += _sendmsg(th, type, msg_int, buf, buflen);
            if (id != 0) break;
        }
    }
    mp_unlock_thread_mutex();
    return nsent;
}

//------------------------------------------------------------------------------------------
int mp_thread_getmsg(uint32_t *msg_int, uint8_t **buf, uint32_t *buflen, uint64_t *sender) {
	int res = 0;
    mp_lock_thread_mutex();
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
        			*msg_int = msg.intdata;
        			if ((msg.strdata != NULL) && (msg.strlen > 0)) {
            			*buflen = msg.strlen;
            			*buf = msg.strdata;
            			res = THREAD_MSG_TYPE_STRING;
        			}
        		}
        	}
        	break;
        }
    }
    mp_unlock_thread_mutex();

    return res;
}

//-------------------------------------
int mp_thread_status(TaskHandle_t id) {
	int res = -1;
    mp_lock_thread_mutex();
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
    mp_unlock_thread_mutex();
    return res;
}

//-----------------------------------------------
int mp_thread_pystack_get_size(TaskHandle_t id) {
    int res = -1;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if ((id == NULL) && (th->id == xTaskGetCurrentTaskHandle())) {
            mp_state_ctx_t *state = (mp_state_ctx_t *)pvTaskGetThreadLocalStoragePointer(NULL, THREAD_LSP_STATE);
            res = state->thread.pystack_end - state->thread.pystack_start;
            break;
        }
        else if (th->id == id) {
            mp_state_ctx_t *state = (mp_state_ctx_t *)pvTaskGetThreadLocalStoragePointer(th->id, THREAD_LSP_STATE);
            res = state->thread.pystack_end - state->thread.pystack_start;
            break;
        }
    }
    mp_unlock_thread_mutex();
    return res;
}

//-------------------------------------------
void mp_thread_kbd_interrupt(TaskHandle_t id)
{
    if ((id == xTaskGetCurrentTaskHandle()) || (id == MainTaskHandle) || (id == MainTaskHandle2)) return;

    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
        if (th->id == id) {
            mp_state_ctx_t *state = (mp_state_ctx_t *)pvTaskGetThreadLocalStoragePointer(th->id, THREAD_LSP_STATE);
            state->vm.mp_pending_exception = MP_OBJ_FROM_PTR(&state->vm.mp_kbd_exception);
            #if MICROPY_ENABLE_SCHEDULER
            if (state->vm.sched_state == MP_SCHED_IDLE) {
                state->vm.sched_state = MP_SCHED_PENDING;
            }
            #endif
            break;
        }
    }
    mp_unlock_thread_mutex();
}

//-------------------------------------
int mp_thread_list(thread_list_t *list)
{
	int num = 0;

    mp_lock_thread_mutex();

    // Get number of MicroPython threads
    for (thread_t *th = thread; th != NULL; th = th->next) {
    	num++;
    }
    if ((num == 0) || (list == NULL)) {
        mp_unlock_thread_mutex();
    	return num;
    }

	list->nth = num;
	list->threads = pvPortMalloc(sizeof(threadlistitem_t) * (num+1));
	if (list->threads == NULL) num = 0;
	else {
		int nth = 0;
		threadlistitem_t *thr = NULL;
        mp_state_ctx_t *state;
		for (thread_t *th = thread; th != NULL; th = th->next) {
            state = (mp_state_ctx_t *)pvTaskGetThreadLocalStoragePointer(th->id, THREAD_LSP_STATE);
			thr = list->threads + (sizeof(threadlistitem_t) * nth);

			thr->id = (uint64_t)th->id;
			sprintf(thr->name, "%s", th->name);
			thr->suspended = th->suspended;
			thr->waiting = th->waiting;
			thr->type = th->type;

			//thr->stack_len = state->thread.stack_limit;
            //thr->stack_curr = (void *)state->thread.stack_top - (void *)pxTaskGetStackTop((TaskHandle_t)th->id);
			//thr->stack_max = state->thread.stack_limit - (uxTaskGetStackHighWaterMark((TaskHandle_t)th->id) * sizeof(StackType_t));
            thr->stack_len = (uint32_t)(pxTaskGetStackEnd((TaskHandle_t)th->id) - pxTaskGetStackStart((TaskHandle_t)th->id));
            thr->stack_len += sizeof(StackType_t);
            thr->stack_curr = (uint32_t)(pxTaskGetStackEnd((TaskHandle_t)th->id) - pxTaskGetStackTop((TaskHandle_t)th->id));
            thr->stack_max = thr->stack_len - (uxTaskGetStackHighWaterMark((TaskHandle_t)th->id) * sizeof(StackType_t));

            thr->pystack_len = state->thread.pystack_end - state->thread.pystack_start;
            if (mpy_config.config.pystack_enabled)
                thr->pystack_used = (state->thread.pystack_cur - state->thread.pystack_start);
            else thr->pystack_used = 0;
			thr->priority = th->priority;
			thr->proc = xTaskGetProcessor((TaskHandle_t)th->id);
			thr->current = (th->id == xTaskGetCurrentTaskHandle());
			nth++;
			if (nth > num) break;
		}
		if (nth != num) {
			vPortFree(list->threads);
			list->threads = NULL;
			num = 0;
		}
    }
    mp_unlock_thread_mutex();
    return num;
}

//------------------------------------------
int mp_thread_mainAcceptMsg(int8_t accept) {
	int res = main_accept_msg;
    mp_lock_thread_mutex();
    for (thread_t *th = thread; th != NULL; th = th->next) {
    	 if (th->id == xTaskGetCurrentTaskHandle()) {
    		 if ((th->id == MainTaskHandle) && (accept >= 0)) {
    			 main_accept_msg = accept & 1;
    		 }
			 break;
    	 }
    }
    mp_unlock_thread_mutex();

    return res;
}

#endif
