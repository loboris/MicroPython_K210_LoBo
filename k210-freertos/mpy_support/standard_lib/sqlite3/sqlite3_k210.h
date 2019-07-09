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


#ifndef SQLITE3_K210_H
#define SQLITE3_K210_H

#include "mpconfigport.h"

#if MICROPY_PY_USE_SQLITE

#include "sqlite3.h"
#include "py/runtime.h"

#define CACHEBLOCKSZ                64
#define K210_DEFAULT_MAXNAMESIZE    127
// set to 1 to use FreeRTOS memory allocator
// ToDo: using it may cause crash with some SQL statements
// do not use it for now
#define USER_MEM_ALLOC              0

typedef struct st_linkedlist {
    uint16_t blockid;
    struct st_linkedlist *next;
    uint8_t data[CACHEBLOCKSZ];
} linkedlist_t, *pLinkedList_t;

typedef struct st_filecache {
    uint32_t size;
    linkedlist_t *list;
} filecache_t, *pFileCache_t;

typedef struct K210_file {
    sqlite3_file base;
    mp_obj_t fd;
    filecache_t *cache;
    char name[K210_DEFAULT_MAXNAMESIZE];
} K210_file;

int K210_Close(sqlite3_file*);
int K210_Lock(sqlite3_file *, int);
int K210_Unlock(sqlite3_file*, int);
int K210_Sync(sqlite3_file*, int);
int K210_Open(sqlite3_vfs*, const char *, sqlite3_file *, int, int*);
int K210_Read(sqlite3_file*, void*, int, sqlite3_int64);
int K210_Write(sqlite3_file*, const void*, int, sqlite3_int64);
int K210_Truncate(sqlite3_file*, sqlite3_int64);
int K210_Delete(sqlite3_vfs*, const char *, int);
int K210_FileSize(sqlite3_file*, sqlite3_int64*);
int K210_Access(sqlite3_vfs*, const char*, int, int*);
int K210_FullPathname( sqlite3_vfs*, const char *, int, char*);
int K210_CheckReservedLock(sqlite3_file*, int *);
int K210_FileControl(sqlite3_file *, int, void*);
int K210_SectorSize(sqlite3_file*);
int K210_DeviceCharacteristics(sqlite3_file*);
int K210_Randomness(sqlite3_vfs*, int, char*);
int K210_Sleep(sqlite3_vfs*, int);
int K210_CurrentTime(sqlite3_vfs*, double*);

int K210mem_Close(sqlite3_file*);
int K210mem_Read(sqlite3_file*, void*, int, sqlite3_int64);
int K210mem_Write(sqlite3_file*, const void*, int, sqlite3_int64);
int K210mem_FileSize(sqlite3_file*, sqlite3_int64*);
int K210mem_Sync(sqlite3_file*, int);

void errorLogCallback(void *pArg, int iErrCode, const char *zMsg);

#if USER_MEM_ALLOC
void *K210alloc_Malloc(int size);
void K210alloc_Free(void *ptr);
void *K210alloc_Realloc(void *mem, int size);
int K210alloc_Size(void *ptr);
int K210alloc_Roundup(int size);
int K210alloc_Init(void *arg);
void K210alloc_Shutdown(void *arg);

extern const sqlite3_mem_methods K210AllocMethods;
#endif

extern bool sqlite3_debug;
extern bool sqlite3_alloc_debug;

#endif

#endif
