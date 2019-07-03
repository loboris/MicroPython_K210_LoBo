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

#include "py/mpconfig.h"

#if MICROPY_VFS && MICROPY_VFS_SDCARD

#include <stdio.h>
#include <string.h>

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "vfs_sdcard.h"


const mp_obj_type_t mp_type_vfs_sdcard_textio;
#if MICROPY_PY_IO_FILEIO
const mp_obj_type_t mp_type_vfs_sdcard_fileio;
#endif

typedef struct _sdcard_file_obj_t {
    mp_obj_base_t base;
    FIL   fp;
} sdcard_file_obj_t;

//-----------------------------------------------------------------------------------------
STATIC void file_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    mp_printf(print, "<io.%s %p>", mp_obj_get_type_str(self_in), MP_OBJ_TO_PTR(self_in));
}

//---------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode)
{
    sdcard_file_obj_t *self = MP_OBJ_TO_PTR(self_in);

    uint32_t bread = 0;
    int res = f_read(&self->fp, buf, size, &bread);
    if (res != FR_OK) {
        *errcode = fresult_to_errno_table[res];
        return MP_STREAM_ERROR;
    }
    return (mp_uint_t)bread;
}

//----------------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode)
{
    sdcard_file_obj_t *self = MP_OBJ_TO_PTR(self_in);

    uint32_t written = 0, total = 0, wrsize;
    int remain = size;
    while (remain > 0) {
        wrsize = (remain > 256) ? 256 : remain;
        int res = f_write(&self->fp, buf+total, wrsize, &written);
        if (res != FR_OK) {
            *errcode = fresult_to_errno_table[res];
            return MP_STREAM_ERROR;
        }
        if (written != wrsize) {
            // The FatFS documentation says that this means disk full.
            *errcode = MP_ENOSPC;
            return MP_STREAM_ERROR;
        }
        remain -= wrsize;
        total += wrsize;
    }

    return (mp_uint_t)total;
}

//----------------------------------------------------------------------
STATIC mp_obj_t file_obj___exit__(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    return mp_stream_close(args[0]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(file_obj___exit___obj, 4, 4, file_obj___exit__);

//--------------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_ioctl(mp_obj_t o_in, mp_uint_t request, uintptr_t arg, int *errcode)
{
    sdcard_file_obj_t *self = MP_OBJ_TO_PTR(o_in);

    if (request == MP_STREAM_SEEK) {
        struct mp_stream_seek_t *s = (struct mp_stream_seek_t*)(uintptr_t)arg;

        switch (s->whence) {
            case 0: // SEEK_SET
                f_lseek(&self->fp, s->offset);
                break;

            case 1: // SEEK_CUR
                f_lseek(&self->fp, f_tell(&self->fp) + s->offset);
                break;

            case 2: // SEEK_END
                f_lseek(&self->fp, f_size(&self->fp) + s->offset);
                break;
        }

        s->offset = f_tell(&self->fp);
        return 0;

    } else if (request == MP_STREAM_FLUSH) {
        FRESULT res = f_sync(&self->fp);
        if (res != FR_OK) {
            *errcode = fresult_to_errno_table[res];
            return MP_STREAM_ERROR;
        }
        return 0;

    } else if (request == MP_STREAM_CLOSE) {
        // if fs==NULL then the file is closed and in that case this method is a no-op
        if (self->fp.obj.fs != NULL) {
            FRESULT res = f_close(&self->fp);
            if (res != FR_OK) {
                *errcode = fresult_to_errno_table[res];
                return MP_STREAM_ERROR;
            }
        }
        return 0;

    } else {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
    return MP_STREAM_ERROR;
}

//----------------------------------------
STATIC const mp_arg_t file_open_args[] = {
    { MP_QSTR_file, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_rom_obj = MP_ROM_PTR(&mp_const_none_obj)} },
    { MP_QSTR_mode, MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_QSTR(MP_QSTR_r)} },
    { MP_QSTR_encoding, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_PTR(&mp_const_none_obj)} },
};
#define FILE_OPEN_NUM_ARGS MP_ARRAY_SIZE(file_open_args)

//------------------------------------------------------------------------------------------------
STATIC mp_obj_t file_open(sdcard_user_mount_t *vfs, const mp_obj_type_t *type, mp_arg_val_t *args)
{
	const char* file_name = mp_obj_str_get_str(args[0].u_obj);
    const char *mode_s = mp_obj_str_get_str(args[1].u_obj);
    const char *lpath = sdcard_local_path(file_name, vfs);
    uint32_t mode = 0;
    while (*mode_s) {
        switch (*mode_s++) {
            case 'r':
                mode |= FA_READ;
                break;
            case 'w':
                mode |= FA_WRITE | FA_CREATE_ALWAYS;
                break;
            case 'x':
                mode |= FA_WRITE | FA_CREATE_NEW;
                break;
            case 'a':
                mode |= FA_WRITE | FA_OPEN_ALWAYS;
                break;
            case '+':
                mode |= FA_READ | FA_WRITE;
                break;
            #if MICROPY_PY_IO_FILEIO
            case 'b':
                type = &mp_type_vfs_sdcard_fileio;
                break;
            #endif
            case 't':
                type = &mp_type_vfs_sdcard_textio;
                break;
            default:
                mp_raise_ValueError("Invalid mode");
        }
    }

    sdcard_file_obj_t *o = m_new_obj_with_finaliser(sdcard_file_obj_t);
    o->base.type = type;
    FRESULT res = f_open(&o->fp, lpath, mode);
    if (res != FR_OK) {
        m_del_obj(sdcard_file_obj_t, o);
        mp_raise_OSError(fresult_to_errno_table[res]);
    }

    // for 'a' mode, we must begin at the end of the file
    if ((mode & FA_OPEN_ALWAYS) != 0) {
        f_lseek(&o->fp, f_size(&o->fp));
    }

    return MP_OBJ_FROM_PTR(o);
}

//------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t file_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    mp_arg_parse_all_kw_array(n_args, n_kw, args, FILE_OPEN_NUM_ARGS, file_open_args, arg_vals);
    return file_open(NULL, type, arg_vals);
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

const mp_obj_type_t mp_type_vfs_sdcard_fileio = {
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

const mp_obj_type_t mp_type_vfs_sdcard_textio = {
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
STATIC mp_obj_t sdcard_builtin_open_self(mp_obj_t self_in, mp_obj_t path, mp_obj_t mode) {
    // TODO: analyze buffering args and instantiate appropriate type
    sdcard_user_mount_t *self = MP_OBJ_TO_PTR(self_in);
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    arg_vals[0].u_obj = path;
    arg_vals[1].u_obj = mode;
    arg_vals[2].u_obj = mp_const_none;
    return file_open(self, &mp_type_vfs_sdcard_textio, arg_vals);
}
MP_DEFINE_CONST_FUN_OBJ_3(sdcard_vfs_open_obj, sdcard_builtin_open_self);

#endif // MICROPY_VFS && MICROPY_VFS_SDCARD
