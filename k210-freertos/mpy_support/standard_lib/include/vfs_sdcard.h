/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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
#ifndef MICROPY_INCLUDED_EXTMOD_VFS_SDCARD_H
#define MICROPY_INCLUDED_EXTMOD_VFS_SDCARD_H

#include "py/lexer.h"
#include "py/obj.h"
#include "extmod/vfs.h"
#include "ff.h"

// these are the values for fs_user_mount_t.flags
#define MODULE_SDCARD        (0x0001) // readblocks[2]/writeblocks[2] contain native func
#define SYS_SDCARD           (0x0002) // fs_user_mount_t obj should be freed on umount
#define FSUSER_HAVE_IOCTL    (0x0004) // new protocol with ioctl
#define FSUSER_NO_FILESYSTEM (0x0008) // the block device has no filesystem on it

typedef struct _sdcard_user_mount_t {
    mp_obj_base_t base;
    uint16_t flags;
    FATFS *fs;
	mp_vfs_proto_t *protocol;
} sdcard_user_mount_t;

extern const byte fresult_to_errno_table[20];
extern const mp_obj_type_t mp_sdcard_vfs_type;
void sdcard_fix_path(const char *path,  char *fullpath);

MP_DECLARE_CONST_FUN_OBJ_3(sdcard_vfs_open_obj);

#endif // MICROPY_INCLUDED_EXTMOD_VFS_SDCARD_H
