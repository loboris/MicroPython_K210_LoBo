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

#include "py/mpconfig.h"

#if MICROPY_VFS && MICROPY_VFS_LITTLEFS

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "syslog.h"
#include "devices.h"
#include "w25qxx.h"

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "mphalport.h"

#include "littleflash.h"

//const mp_obj_type_t mp_type_vfs_littlefs_textio;

extern handle_t mp_rtc_rtc0;
static const char* TAG = "[LFS_FILE]";

//-----------------------------------------------------------------------------------------
STATIC void file_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    mp_printf(print, "<io.%s %p>", mp_obj_get_type_str(self_in), MP_OBJ_TO_PTR(self_in));
}

//---------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode)
{
    littlefs_file_obj_t *self = MP_OBJ_TO_PTR(self_in);

    lfs_ssize_t read = lfs_file_read(self->fs, &self->fd, buf, size);
    return (mp_uint_t)read;
}

//----------------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode)
{
    littlefs_file_obj_t *self = MP_OBJ_TO_PTR(self_in);

    //_lock_acquire(&self->lock);
    lfs_ssize_t written = lfs_file_write(self->fs, &self->fd, buf, size);
    //_lock_release(&self->lock);

    if (written < 0) {
        if (w25qxx_debug) LOGD(TAG, "Write error (%d)", written);
        *errcode = MP_EIO;
        return MP_STREAM_ERROR;
    }
    if (written != size) {
        /*if (w25qxx_debug)*/ LOGQ(TAG, "Write error (%d <> %lu)", written, size);
        *errcode = MP_ENOSPC;
        return MP_STREAM_ERROR;
    }
    return (mp_uint_t)written;
}

//--------------------------------------------------------------------
STATIC mp_obj_t file_obj___exit__(size_t n_args, const mp_obj_t *args)
{
    (void)n_args;
    return mp_stream_close(args[0]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(file_obj___exit___obj, 4, 4, file_obj___exit__);

//--------------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_ioctl(mp_obj_t o_in, mp_uint_t request, uintptr_t arg, int *errcode)
{
    littlefs_file_obj_t *self = MP_OBJ_TO_PTR(o_in);
    if (request == MP_STREAM_SEEK) {
        struct mp_stream_seek_t *s = (struct mp_stream_seek_t*)(uintptr_t)arg;
        lfs_soff_t pos = 0;

        switch (s->whence) {
            case 0: // SEEK_SET
                pos = lfs_file_seek(self->fs, &self->fd, s->offset, LFS_SEEK_SET);
                break;

            case 1: // SEEK_CUR
                pos = lfs_file_seek(self->fs, &self->fd, s->offset, LFS_SEEK_CUR);
                break;

            case 2: // SEEK_END
                pos = lfs_file_seek(self->fs, &self->fd, s->offset, LFS_SEEK_END);
                break;
        }

        s->offset = pos;
        return 0;

    }
    else if (request == MP_STREAM_FLUSH) {
        int ret = lfs_file_sync(self->fs, &self->fd);
        if (ret != 0) {
            *errcode = MP_EIO;
            return MP_STREAM_ERROR;
        }
        return 0;

    }
    else if (request == MP_STREAM_CLOSE) {
        int err = lfs_file_close(self->fs, &self->fd);
        if (err != 0) {
            *errcode = MP_EIO;
            return MP_STREAM_ERROR;
        }
        return 0;

    }
    else {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
    return MP_STREAM_ERROR;
}

//----------------------------------------
STATIC const mp_arg_t file_open_args[] = {
    { MP_QSTR_file,     MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_rom_obj = MP_ROM_PTR(&mp_const_none_obj)} },
    { MP_QSTR_mode,     MP_ARG_OBJ,                   {.u_obj     = MP_OBJ_NEW_QSTR(MP_QSTR_r)} },
    { MP_QSTR_encoding, MP_ARG_OBJ | MP_ARG_KW_ONLY,  {.u_rom_obj = MP_ROM_PTR(&mp_const_none_obj)} },
};
#define FILE_OPEN_NUM_ARGS MP_ARRAY_SIZE(file_open_args)

//--------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t file_open(littlefs_user_mount_t *vfs, const mp_obj_type_t *type, mp_arg_val_t *args, bool raise)
{
	const char *file_name = mp_obj_str_get_str(args[0].u_obj);
    const char *mode_s = mp_obj_str_get_str(args[1].u_obj);
    if (strlen(file_name) > LITTLEFS_CFG_MAX_NAME_LEN) {
        if (raise) mp_raise_ValueError("name too long");
        return mp_const_none;
    }

    const char *lpath = littlefs_local_path(file_name);
    if (w25qxx_debug) LOGD(TAG, "OPEN [%s]->[%s]", file_name, lpath);

    uint32_t mode = 0;
    while (*mode_s) {
        switch (*mode_s++) {
            case 'r':
                mode |= LFS_O_RDONLY;
                break;
            case 'w':
                mode |= LFS_O_RDWR | LFS_O_CREAT;
                break;
            case 'x':
                mode |= LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC;
                break;
            case 'a':
                mode |= LFS_O_RDWR | LFS_O_APPEND;
                break;
            case '+':
                mode |= LFS_O_RDWR;
                break;
            #if MICROPY_PY_IO_FILEIO
            case 'b':
                type = &mp_type_vfs_littlefs_fileio;
                break;
            #endif
            case 't':
                type = &mp_type_vfs_littlefs_textio;
                break;
            default:
                if (raise) mp_raise_ValueError("not allowed mode character");
                return mp_const_none;
        }
    }

    littlefs_file_obj_t *o = m_new_obj_with_finaliser(littlefs_file_obj_t);

    memset(o, 0, sizeof(littlefs_file_obj_t));
    o->base.type = type;
    o->fs = &vfs->fs->lfs;
    o->cfg.buffer = &o->file_buffer;
    if (mode != LFS_O_RDONLY) {
        struct tm now;
        rtc_get_datetime(mp_rtc_rtc0, &now);
        o->timestamp = (uint32_t)mktime(&now);
    }
    o->attrs.type = LITTLEFS_ATTR_MTIME;
    o->attrs.buffer = &o->timestamp;
    o->attrs.size = sizeof(uint32_t);
    o->cfg.attr_count = 1;
    o->cfg.attrs = &o->attrs;

    int err = lfs_file_opencfg(&vfs->fs->lfs, &o->fd, lpath, mode, &o->cfg);

    if(err != LFS_ERR_OK) {
        if (w25qxx_debug) LOGD("[LFS_FILE]", "OPEN error %d", err);
        m_del_obj(littlefs_file_obj_t, o);
        if (raise) mp_raise_OSError(map_lfs_error(err));
        return mp_const_none;
    }

    return MP_OBJ_FROM_PTR(o);
}

//------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t file_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    mp_arg_parse_all_kw_array(n_args, n_kw, args, FILE_OPEN_NUM_ARGS, file_open_args, arg_vals);
    return file_open(NULL, type, arg_vals, true);
}

// TODO gc hook to close the file if not already closed

STATIC const mp_rom_map_elem_t rawfile_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_readlines), MP_ROM_PTR(&mp_stream_unbuffered_readlines_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&mp_stream_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_seek), MP_ROM_PTR(&mp_stream_seek_obj) },
    { MP_ROM_QSTR(MP_QSTR_tell), MP_ROM_PTR(&mp_stream_tell_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&mp_identity_obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&file_obj___exit___obj) },
};

STATIC MP_DEFINE_CONST_DICT(rawfile_locals_dict, rawfile_locals_dict_table);
#if MICROPY_PY_IO_FILEIO
STATIC const mp_stream_p_t fileio_stream_p = {
    .read = file_obj_read,
    .write = file_obj_write,
    .ioctl = file_obj_ioctl,
};


const mp_obj_type_t mp_type_vfs_littlefs_fileio = {
    { &mp_type_type },
    .name = MP_QSTR_FileIO,
    .print = file_obj_print,
    .make_new = file_obj_make_new,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &fileio_stream_p,
    .locals_dict = (mp_obj_dict_t*)&rawfile_locals_dict,
};
#endif
STATIC const mp_stream_p_t textio_stream_p = {
    .read = file_obj_read,
    .write = file_obj_write,
    .ioctl = file_obj_ioctl,
    .is_text = true,
};

const mp_obj_type_t mp_type_vfs_littlefs_textio = {
    { &mp_type_type },
    .name = MP_QSTR_TextIOWrapper,
    .print = file_obj_print,
    .make_new = file_obj_make_new,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &textio_stream_p,
    .locals_dict = (mp_obj_dict_t*)&rawfile_locals_dict,
};


// Factory function for I/O stream classes
//----------------------------------------------------------------------------------------
STATIC mp_obj_t littlefs_builtin_open_self(mp_obj_t self_in, mp_obj_t path, mp_obj_t mode)
{
    // TODO: analyze buffering args and instantiate appropriate type
    littlefs_user_mount_t *self = MP_OBJ_TO_PTR(self_in);
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    arg_vals[0].u_obj = path;
    arg_vals[1].u_obj = mode;
    arg_vals[2].u_obj = mp_const_none;
    return file_open(self, &mp_type_vfs_littlefs_textio, arg_vals, true);
}
MP_DEFINE_CONST_FUN_OBJ_3(littlefs_vfs_open_obj, littlefs_builtin_open_self);

//-------------------------------------------------------------------------------------------
STATIC mp_obj_t littlefs_builtin_open_ex_self(mp_obj_t self_in, mp_obj_t path, mp_obj_t mode)
{
    // TODO: analyze buffering args and instantiate appropriate type
    littlefs_user_mount_t *self = MP_OBJ_TO_PTR(self_in);
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    arg_vals[0].u_obj = path;
    arg_vals[1].u_obj = mode;
    arg_vals[2].u_obj = mp_const_none;
    return file_open(self, &mp_type_vfs_littlefs_textio, arg_vals, false);
}
MP_DEFINE_CONST_FUN_OBJ_3(littlefs_vfs_open_ex_obj, littlefs_builtin_open_ex_self);

#endif // MICROPY_VFS && MICROPY_VFS_LITTLEFS
