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

#ifndef __MICROPY_INCLUDED_MPTHREADPORT_H__
#define __MICROPY_INCLUDED_MPTHREADPORT_H__

#if MICROPY_PY_THREAD

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "mpconfigport.h"
#include "py/obj.h"

// Local storage pointers id's
#define THREAD_LSP_STATE                    0
#define THREAD_LSP_ARGS                     1

// Thread types
#define THREAD_TYPE_MAIN		            1
#define THREAD_TYPE_PYTHON		            2
#define THREAD_TYPE_SERVICE		            3

// Reserved thread notification constants
#define THREAD_NOTIFY_PAUSE		            0x01000000
#define THREAD_NOTIFY_RESUME	            0x02000000
#define THREAD_NOTIFY_EXIT		            0x04000000
#define THREAD_NOTIFY_STATUS	            0x08000000
#define THREAD_NOTIFY_RESET		            0x10000000
#define THREAD_WAIT_TIMEOUT		            0x20000000
#define THREAD_KBD_EXCEPTION                0x30000000

#define THREAD_STATUS_RUNNING		        0
#define THREAD_STATUS_SUSPENDED		        1
#define THREAD_STATUS_WAITING		        2
#define THREAD_STATUS_TERMINATED            -1

#define MP_THREAD_PRIORITY                  MICROPY_TASK_PRIORITY
#define MP_THREAD_MAX_PRIORITY              (configMAX_PRIORITIES-1)

#define MP_THREAD_MIN_SERVICE_STACK_SIZE    (2*1024)  // in stack_type units (64-bits)

#define MP_THREAD_MIN_STACK_SIZE			1024      // in stack_type units (64-bits)
#define MP_THREAD_DEFAULT_STACK_SIZE		1024      // in stack_type units (64-bits)
#define MP_THREAD_MAX_STACK_SIZE			(16*1024) // in stack_type units (64-bits)

#define MP_THREAD_MIN_PYSTACK_SIZE          2048
#define MP_THREAD_MAX_PYSTACK_SIZE          65536

#define THREAD_NAME_MAX_SIZE		        16
#define THREAD_MGG_BROADCAST		        0xFFFFEEEE
#define THREAD_MSG_TYPE_NONE		        0
#define THREAD_MSG_TYPE_INTEGER		        1
#define THREAD_MSG_TYPE_STRING		        2
#define MAX_THREAD_MESSAGES			        8
#define THREAD_QUEUE_MAX_ITEMS		        8

#define THREAD_IPC_TYPE_EXECUTE             1
#define THREAD_IPC_TYPE_TERMINATE           0xA55A

#define SYS_TASK_NOTIFY_SUSPEND             1ULL
#define SYS_TASK_NOTIFY_RESUME              2ULL
#define SYS_TASK_NOTIFY_SUSPEND_OTHERS      3ULL
#define SYS_TASK_NOTIFY_RESUME_OTHERS       4ULL


typedef struct _mp_thread_mutex_t {
    SemaphoreHandle_t handle;
    StaticSemaphore_t buffer;
} mp_thread_mutex_t;

// this structure forms a linked list, one node per active thread
//========================
typedef struct _thread_t {
    TaskHandle_t id;                    // system id of thread
    int ready;                          // whether the thread is ready and running
    char name[THREAD_NAME_MAX_SIZE];    // thread name
    QueueHandle_t threadQueue;          // queue used for inter thread communication
    int8_t processor;
    int8_t allow_suspend;
    int8_t suspended;
    int8_t waiting;
    int8_t deleted;
    int16_t notifyed;
    uint16_t type;
    int priority;
    bool locked;
    struct _thread_t *next;
} __attribute__((aligned(8))) thread_t;

// this structure is used for inter-thread communication/data passing
typedef struct _thread_msg_t {
    int type;						// message type
    TaskHandle_t sender_id;			// id of the message sender
    uint64_t intdata;				// integer data
    uint64_t timestamp;				// message timestamp in ms
    uint8_t *strdata;               // string data
    uint32_t strlen;                // string data length
} __attribute__((aligned(8))) thread_msg_t;

typedef struct _thread_listitem_t {
    uint64_t id;						// thread id
    char name[THREAD_NAME_MAX_SIZE];	// thread name
    uint8_t suspended;
    uint8_t waiting;
    uint8_t type;
    uint8_t proc;
    uint8_t current;
    uint32_t stack_len;
    uint32_t stack_max;
    uint32_t stack_curr;
    uint32_t pystack_len;
    uint32_t pystack_used;
    int priority;
} __attribute__((aligned(8))) threadlistitem_t;

typedef struct _thread_list_t {
    int nth;						// number of active threads
    threadlistitem_t *threads;		// pointer to thread info
} thread_list_t;

//-----------------------------------
typedef struct _thread_entry_args_t {
    mp_obj_dict_t   *dict_locals;
    mp_obj_dict_t   *dict_globals;
    thread_t        *thread;
    mp_obj_t        fun;
    size_t          n_args;
    size_t          n_kw;
    void            *pystack_start;
    void            *pystack_end;
    mp_obj_t        args[];
} __attribute__((aligned(8))) thread_entry_args_t;

extern TaskHandle_t MainTaskHandle2;
extern thread_t thread_entry2;

extern TaskHandle_t MainTaskHandle;
extern thread_t thread_entry0;

extern thread_msg_t thread_messages[MAX_THREAD_MESSAGES];

extern uint8_t main_accept_msg;

TaskHandle_t mp_thread_create(void *entry, thread_entry_args_t *arg, size_t task_stack_size, size_t pystack_size, int priority, char *name);

void mp_thread_preinit(void *stack, uint32_t stack_len, void *pystack, int pystack_size, int task_proc);
int mp_thread_num_threads();
int mp_thread_gc_others();

void mp_thread_mutex_init(mp_thread_mutex_t *mutex);

int mp_thread_mutex_lock(mp_thread_mutex_t *mutex, int wait);
void mp_thread_mutex_unlock(mp_thread_mutex_t *mutex);
bool mp_thread_locked();

thread_t *mp_thread_get_th_from_id(TaskHandle_t id);
int mp_thread_pystack_get_size(TaskHandle_t id);
int mp_thread_started(TaskHandle_t id);

void mp_thread_allowsuspend(int allow);
int mp_thread_suspend(TaskHandle_t id);
int mp_thread_resume(TaskHandle_t id);
int mp_thread_suspend_others();
int mp_thread_resume_others();

int mp_thread_notify(TaskHandle_t id, uint32_t value);
uint32_t mp_thread_getnotify(bool check_only);
int mp_thread_notifyPending(TaskHandle_t id);
void mp_thread_resetPending();
int mp_thread_semdmsg(TaskHandle_t id, int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen);
int mp_thread_sendmsg_to_mpy2(int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen);
int mp_thread_sendmsg_to_mpy1(int type, uint32_t msg_int, uint8_t *buf, uint32_t buflen);
int mp_thread_getmsg(uint32_t *msg_int, uint8_t **buf, uint32_t *buflen, uint64_t *sender);

int mp_thread_status(TaskHandle_t id);
thread_t *mp_thread_get_thread(TaskHandle_t id, thread_t *self);

int mp_thread_set_priority(TaskHandle_t id, int priority);
int mp_thread_get_priority(TaskHandle_t id);

int mp_thread_setblocked();
int mp_thread_setnotblocked();

uint64_t mp_thread_getSelfID();
uint8_t mp_thread_getSelfIdx();
int mp_thread_getSelfname(char *name);
int mp_thread_getname(TaskHandle_t id, char *name);

int mp_thread_list(thread_list_t *list);

int mp_thread_mainAcceptMsg(int8_t accept);
void mp_thread_kbd_interrupt(TaskHandle_t id);

#endif

#endif // __MICROPY_INCLUDED_MPTHREADPORT_H__
