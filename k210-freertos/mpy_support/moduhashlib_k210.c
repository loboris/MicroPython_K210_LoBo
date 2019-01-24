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

#include <assert.h>
#include <string.h>
#include "py/runtime.h"

#if MICROPY_PY_UHASHLIB_K210

#if MICROPY_PY_UHASHLIB_MD5_K210
#include <netif/ppp/polarssl/md5.h>
#endif

#if MICROPY_PY_UHASHLIB_SHA1_K210
#include <netif/ppp/polarssl/sha1.h>
#endif

#if MICROPY_PY_UHASHLIB_SHA256_K210
#include <netif/ppp/polarssl/sha256.h>
#include <devices.h>
#endif

typedef struct _mp_obj_hash_t {
    mp_obj_base_t base;
    char state[0];
} mp_obj_hash_t;


#if MICROPY_PY_UHASHLIB_SHA1_K210
STATIC mp_obj_t uhashlib_sha1_update(mp_obj_t self_in, mp_obj_t arg);

STATIC mp_obj_t uhashlib_sha1_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    mp_obj_hash_t *o = m_new_obj_var(mp_obj_hash_t, char, sizeof(sha1_context));
    o->base.type = type;
    sha1_starts((sha1_context*)o->state);
    if (n_args == 1) {
        uhashlib_sha1_update(MP_OBJ_FROM_PTR(o), args[0]);
    }
    return MP_OBJ_FROM_PTR(o);
}

STATIC mp_obj_t uhashlib_sha1_update(mp_obj_t self_in, mp_obj_t arg) {
    mp_obj_hash_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg, &bufinfo, MP_BUFFER_READ);
    sha1_update((sha1_context*)self->state, bufinfo.buf, bufinfo.len);
    return mp_const_none;
}

STATIC mp_obj_t uhashlib_sha1_digest(mp_obj_t self_in) {
    mp_obj_hash_t *self = MP_OBJ_TO_PTR(self_in);
    vstr_t vstr;
    vstr_init_len(&vstr, 20);
    sha1_finish((sha1_context*)self->state, (byte*)vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC mp_obj_t uhashlib_sha1(mp_obj_t arg) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg, &bufinfo, MP_BUFFER_READ);
    vstr_t vstr;
    vstr_init_len(&vstr, 20);
    sha1(bufinfo.buf, bufinfo.len, (byte*)vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC MP_DEFINE_CONST_FUN_OBJ_2(uhashlib_sha1_update_obj, uhashlib_sha1_update);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(uhashlib_sha1_digest_obj, uhashlib_sha1_digest);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(uhashlib_sha1_obj, uhashlib_sha1);

STATIC const mp_rom_map_elem_t uhashlib_sha1_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_update), MP_ROM_PTR(&uhashlib_sha1_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_digest), MP_ROM_PTR(&uhashlib_sha1_digest_obj) },
};
STATIC MP_DEFINE_CONST_DICT(uhashlib_sha1_locals_dict, uhashlib_sha1_locals_dict_table);

STATIC const mp_obj_type_t uhashlib_sha1_type = {
    { &mp_type_type },
    .name = MP_QSTR_sha1,
    .make_new = uhashlib_sha1_make_new,
    .locals_dict = (void*)&uhashlib_sha1_locals_dict,
};
#endif

#if MICROPY_PY_UHASHLIB_SHA256_K210
STATIC mp_obj_t uhashlib_sha256_update(mp_obj_t self_in, mp_obj_t arg);

STATIC mp_obj_t uhashlib_sha256_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    mp_obj_hash_t *o = m_new_obj_var(mp_obj_hash_t, char, sizeof(sha256_context));
    o->base.type = type;
    sha256_starts((sha256_context*)o->state, 0);
    if (n_args == 1) {
        uhashlib_sha256_update(MP_OBJ_FROM_PTR(o), args[0]);
    }
    return MP_OBJ_FROM_PTR(o);
}

STATIC mp_obj_t uhashlib_sha256_update(mp_obj_t self_in, mp_obj_t arg) {
    mp_obj_hash_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg, &bufinfo, MP_BUFFER_READ);
    sha256_update((sha256_context*)self->state, bufinfo.buf, bufinfo.len);
    return mp_const_none;
}

STATIC mp_obj_t uhashlib_sha256_digest(mp_obj_t self_in) {
    mp_obj_hash_t *self = MP_OBJ_TO_PTR(self_in);
    vstr_t vstr;
    vstr_init_len(&vstr, 32);
    sha256_finish((sha256_context*)self->state, (byte*)vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC mp_obj_t uhashlib_sha256(mp_obj_t arg) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg, &bufinfo, MP_BUFFER_READ);
    vstr_t vstr;
    vstr_init_len(&vstr, 32);
    sha256(bufinfo.buf, bufinfo.len, (byte*)vstr.buf, 0);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC mp_obj_t uhashlib_sha256_hard(mp_obj_t arg) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg, &bufinfo, MP_BUFFER_READ);
    vstr_t vstr;
    vstr_init_len(&vstr, 32);
    sha256_hard_calculate(bufinfo.buf, bufinfo.len, (byte*)vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC MP_DEFINE_CONST_FUN_OBJ_2(uhashlib_sha256_update_obj, uhashlib_sha256_update);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(uhashlib_sha256_digest_obj, uhashlib_sha256_digest);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(uhashlib_sha256_obj, uhashlib_sha256);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(uhashlib_sha256_hard_obj, uhashlib_sha256_hard);

STATIC const mp_rom_map_elem_t uhashlib_sha256_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_update), MP_ROM_PTR(&uhashlib_sha256_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_digest), MP_ROM_PTR(&uhashlib_sha256_digest_obj) },
};
STATIC MP_DEFINE_CONST_DICT(uhashlib_sha256_locals_dict, uhashlib_sha256_locals_dict_table);

STATIC const mp_obj_type_t uhashlib_sha256_type = {
    { &mp_type_type },
    .name = MP_QSTR_sha256,
    .make_new = uhashlib_sha256_make_new,
    .locals_dict = (void*)&uhashlib_sha256_locals_dict,
};
#endif

#if MICROPY_PY_UHASHLIB_MD5_K210
STATIC mp_obj_t uhashlib_md5_update(mp_obj_t self_in, mp_obj_t arg);

STATIC mp_obj_t uhashlib_md5_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    mp_obj_hash_t *o = m_new_obj_var(mp_obj_hash_t, char, sizeof(md5_context));
    o->base.type = type;
    md5_starts((md5_context*)o->state);
    if (n_args == 1) {
        uhashlib_md5_update(MP_OBJ_FROM_PTR(o), args[0]);
    }
    return MP_OBJ_FROM_PTR(o);
}

STATIC mp_obj_t uhashlib_md5_update(mp_obj_t self_in, mp_obj_t arg) {
    mp_obj_hash_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg, &bufinfo, MP_BUFFER_READ);
    md5_update((md5_context*)self->state, bufinfo.buf, bufinfo.len);
    return mp_const_none;
}

STATIC mp_obj_t uhashlib_md5_digest(mp_obj_t self_in) {
    mp_obj_hash_t *self = MP_OBJ_TO_PTR(self_in);
    vstr_t vstr;
    vstr_init_len(&vstr, 16);
    md5_finish((md5_context*)self->state, (byte*)vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC mp_obj_t uhashlib_md5(mp_obj_t arg) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(arg, &bufinfo, MP_BUFFER_READ);
    vstr_t vstr;
    vstr_init_len(&vstr, 16);
    md5(bufinfo.buf, bufinfo.len, (byte*)vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}

STATIC MP_DEFINE_CONST_FUN_OBJ_2(uhashlib_md5_update_obj, uhashlib_md5_update);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(uhashlib_md5_digest_obj, uhashlib_md5_digest);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(uhashlib_md5_obj, uhashlib_md5);

STATIC const mp_rom_map_elem_t uhashlib_md5_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_update), MP_ROM_PTR(&uhashlib_md5_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_digest), MP_ROM_PTR(&uhashlib_md5_digest_obj) },
};
STATIC MP_DEFINE_CONST_DICT(uhashlib_md5_locals_dict, uhashlib_md5_locals_dict_table);

STATIC const mp_obj_type_t uhashlib_md5_type = {
    { &mp_type_type },
    .name = MP_QSTR_md5,
    .make_new = uhashlib_md5_make_new,
    .locals_dict = (void*)&uhashlib_md5_locals_dict,
};
#endif // MICROPY_PY_UHASHLIB_MD5_K210

STATIC const mp_rom_map_elem_t mp_module_uhashlib_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_uhashlib) },
    #if MICROPY_PY_UHASHLIB_SHA1_K210
    { MP_ROM_QSTR(MP_QSTR_sha1), MP_ROM_PTR(&uhashlib_sha1_type) },
    { MP_ROM_QSTR(MP_QSTR_get_sha1), MP_ROM_PTR(&uhashlib_sha1_obj) },
    #endif
    #if MICROPY_PY_UHASHLIB_SHA256_K210
    { MP_ROM_QSTR(MP_QSTR_sha256), MP_ROM_PTR(&uhashlib_sha256_type) },
    { MP_ROM_QSTR(MP_QSTR_get_sha256), MP_ROM_PTR(&uhashlib_sha256_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sha256_hard), MP_ROM_PTR(&uhashlib_sha256_hard_obj) },
    #endif
    #if MICROPY_PY_UHASHLIB_MD5_K210
    { MP_ROM_QSTR(MP_QSTR_md5), MP_ROM_PTR(&uhashlib_md5_type) },
    { MP_ROM_QSTR(MP_QSTR_get_md5), MP_ROM_PTR(&uhashlib_md5_obj) },
    #endif
};

STATIC MP_DEFINE_CONST_DICT(mp_module_uhashlib_globals, mp_module_uhashlib_globals_table);

const mp_obj_module_t mp_module_uhashlib = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_uhashlib_globals,
};

#endif //MICROPY_PY_UHASHLIB_K210
