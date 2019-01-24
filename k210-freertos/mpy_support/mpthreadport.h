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

// Thread types
#define THREAD_TYPE_MAIN		1
#define THREAD_TYPE_PYTHON		2
#define THREAD_TYPE_SERVICE		3

// Reserved thread notification constants
#define THREAD_NOTIFY_PAUSE		0x01000000
#define THREAD_NOTIFY_RESUME	0x02000000
#define THREAD_NOTIFY_EXIT		0x04000000
#define THREAD_NOTIFY_STATUS	0x08000000
#define THREAD_NOTIFY_RESET		0x10000000
#define THREAD_WAIT_TIMEOUT		0x20000000
#define THREAD_KBD_EXCEPTION    0x30000000

#define THREAD_STATUS_RUNNING		0
#define THREAD_STATUS_SUSPENDED		1
#define THREAD_STATUS_WAITING		2
#define THREAD_STATUS_TERMINATED	-1

#define MP_THREAD_PRIORITY      MICROPY_TASK_PRIORITY
#define MP_THREAD_MAX_PRIORITY  (configMAX_PRIORITIES-1)

#define MP_THREAD_MIN_SERVICE_STACK_SIZE    (2*1024)

#define MP_THREAD_MIN_STACK_SIZE			1580
#define MP_THREAD_DEFAULT_STACK_SIZE		(MICROPY_THREAD_STACK_SIZE)
#define MP_THREAD_MAX_STACK_SIZE			(48*1024)

#define MP_THREAD_MIN_TASK_STACK_SIZE       1580
#define MP_THREAD_DEFAULT_TASK_STACK_SIZE   2048
#define MP_THREAD_MAX_TASK_STACK_SIZE       (48*1024)

#define THREAD_NAME_MAX_SIZE		16
#define THREAD_MGG_BROADCAST		0xFFFFEEEE
#define THREAD_MSG_TYPE_NONE		0
#define THREAD_MSG_TYPE_INTEGER		1
#define THREAD_MSG_TYPE_STRING		2
#define MAX_THREAD_MESSAGES			8
#define THREAD_QUEUE_MAX_ITEMS		8

#define SYS_TASK_NOTIFY_SUSPEND         1ULL
#define SYS_TASK_NOTIFY_RESUME          2ULL
#define SYS_TASK_NOTIFY_SUSPEND_OTHERS  3ULL
#define SYS_TASK_NOTIFY_RESUME_OTHERS   4ULL


typedef struct _mp_thread_mutex_t {
    SemaphoreHandle_t handle;
    StaticSemaphore_t buffer;
} mp_thread_mutex_t;

// this structure forms a linked list, one node per active thread
//========================
typedef struct _thread_t {
    TaskHandle_t id;                    // system id of thread
    int ready;                          // whether the thread is ready and running
    void *arg;                          // thread Python args, a GC root pointer
    void *stack;                        // pointer to the stack
    size_t stack_len;                   // number of words in the stack
    void *curr_sp;                      // current stack pointer
    void *pystack;                      // pointer to the pystack
    int pystack_size;                   // size of pystack
    void *state_thread;                 // thread state
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
} thread_t;

// this structure is used for inter-thread communication/data passing
typedef struct _thread_msg_t {
    int type;						// message type
    TaskHandle_t sender_id;			// id of the message sender
    uint32_t intdata;					// integer data or string data length
    uint8_t *strdata;				// string data
    uint32_t timestamp;				// message timestamp in ms
} thread_msg_t;

typedef struct _thread_listitem_t {
    uint64_t id;						// thread id
    char name[THREAD_NAME_MAX_SIZE];	// thread name
    uint8_t suspended;
    uint8_t waiting;
    uint8_t type;
    uint32_t stack_len;
    uint32_t stack_max;
    uint32_t pystack_len;
    int priority;
} threadlistitem_t;

typedef struct _thread_list_t {
    int nth;						// number of active threads
    threadlistitem_t *threads;		// pointer to thread info
} thread_list_t;

extern TaskHandle_t MainTaskHandle;
extern TaskHandle_t MPySysTaskHandle;

extern thread_msg_t thread_messages[MAX_THREAD_MESSAGES];

extern uint8_t main_accept_msg;

void mp_thread_preinit(void *stack, uint32_t stack_len, void *pystack, int pystack_size);
int mp_thread_num_threads();
int mp_thread_gc_others();

void mp_thread_mutex_init(mp_thread_mutex_t *mutex);
int mp_thread_mutex_lock(mp_thread_mutex_t *mutex, int wait);
void mp_thread_mutex_unlock(mp_thread_mutex_t *mutex);

thread_t *mp_thread_get_th_from_id(TaskHandle_t id);
void mp_thread_set_pystack();
int mp_thread_stack_max_used();
int mp_thread_stack_get_size();

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
int mp_thread_getmsg(uint32_t *msg_int, uint8_t **buf, uint32_t *buflen, uint64_t *sender);
int mp_thread_status(TaskHandle_t id);
bool mp_thread_locked();
thread_t *mp_thread_get_thread(TaskHandle_t id, thread_t *self);

void mp_thread_set_priority(TaskHandle_t id, int priority);
int mp_thread_get_priority(TaskHandle_t id);

int mp_thread_set_sp(void *stack, int size);
int mp_thread_get_sp(void);
int mp_thread_set_th_state(void *state);
int mp_thread_setblocked();
int mp_thread_setnotblocked();

uint64_t mp_thread_getSelfID();
uint8_t mp_thread_getSelfIdx();
int mp_thread_getSelfname(char *name);
int mp_thread_getname(TaskHandle_t id, char *name);
int mp_thread_list(thread_list_t *list);
int mp_thread_mainAcceptMsg(int8_t accept);

#endif

#endif // __MICROPY_INCLUDED_MPTHREADPORT_H__
