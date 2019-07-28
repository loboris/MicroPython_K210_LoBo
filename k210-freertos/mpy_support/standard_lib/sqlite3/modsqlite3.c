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


#include "mpconfigport.h"

#if MICROPY_PY_USE_SQLITE

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include "syslog.h"

#include "sqlite3_k210.h"

#include "py/obj.h"
#include "py/runtime.h"


typedef struct _sqlite_Cursor_t
{
    sqlite3_stmt    *stmt;
    int             step_result;
    int             column_number;
    int             rows_fetched;
} sqlite_Cursor_t;

typedef struct _pysqlite_Connection_t
{
    mp_obj_base_t base;

    char            *fname;
    bool            autocommit;
    sqlite3         *db;
} pysqlite_Connection_t;

typedef struct _pysqlite_Cursor_t
{
    mp_obj_base_t           base;

    pysqlite_Connection_t   *connection;
    sqlite_Cursor_t         cursor;
} pysqlite_Cursor_t;

typedef struct _pysqlite_state_t
{
    bool    is_init;
    int     connected_db;
} pysqlite_state_t;


const mp_obj_type_t mod_sqlite3_Connection_type;
const mp_obj_type_t mod_sqlite3_Cursor_type;

static pysqlite_state_t sqlite_state = {
        false,
        0
};

static const char* TAG = "[MODSQLITE3]";


//----------------------------------------------------------------------------------------------------
static mp_obj_t cursor_execute(sqlite3* db, sqlite_Cursor_t *cursor, mp_obj_t sql_in, mp_obj_t params)
{
    if (db == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Database not opened"));
    }
    if ((params != mp_const_none) && (!mp_obj_is_type(params, &mp_type_tuple))) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Tuple argument expected"));
    }

    char *sql = (char *)mp_obj_str_get_str(sql_in);

    if (cursor->stmt) {
        // Finalize previous statement
        sqlite3_finalize(cursor->stmt);
        cursor->stmt = NULL;
    }

    cursor->step_result = SQLITE_OK;
    cursor->column_number = 0;
    cursor->rows_fetched = 0;

    // === Prepare the statement ===
    int rc = sqlite3_prepare_v2(db, sql, strlen(sql)+1, &cursor->stmt, NULL);
    if ((rc != SQLITE_OK) || (cursor->stmt == NULL)) {
        cursor->stmt = NULL;
        if (sqlite3_debug) LOGQ(TAG, "Prepare error %d [%s]", rc, sqlite3_errstr(rc));
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error preparing sql statement"));
    }
    cursor->column_number = sqlite3_column_count(cursor->stmt);

    if (params != mp_const_none) {
        // Get parameters and bind to statement
        mp_obj_t *t_items;
        size_t t_len;

        mp_obj_tuple_get(params, &t_len, &t_items);
        if (t_len > 0) {
            for (int i=0; i < t_len; i++) {
                rc = SQLITE_OK;
                if (mp_obj_is_int(t_items[i])) {
                    rc = sqlite3_bind_int(cursor->stmt, i+1, mp_obj_get_int(t_items[i]));
                }
                else if (mp_obj_is_type(t_items[i], &mp_type_float)) {
                    rc = sqlite3_bind_double(cursor->stmt, i+1, mp_obj_get_float(t_items[i]));
                }
                else if (mp_obj_is_str(t_items[i])) {
                    const char *tx_par = mp_obj_str_get_str(t_items[i]);
                    rc = sqlite3_bind_text(cursor->stmt, i+1, tx_par, -1, SQLITE_TRANSIENT);
                }
                else if (t_items[i] == mp_const_none) {
                    rc = sqlite3_bind_null(cursor->stmt, i+1);
                }
                else {
                    sqlite3_finalize(cursor->stmt);
                    cursor->stmt = NULL;
                    cursor->step_result = SQLITE_DONE;
                    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Only int, float, str and None parameter types are supported"));
                }
                if (rc != SQLITE_OK) {
                    sqlite3_finalize(cursor->stmt);
                    cursor->stmt = NULL;
                    cursor->step_result = SQLITE_DONE;
                    if (sqlite3_debug) LOGQ(TAG, "Binding error %d [%s]", rc, sqlite3_errstr(rc));
                    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error binding parameters"));
                }
            }
        }
    }

    // === Execute the step function once ===
    rc = sqlite3_step(cursor->stmt);
    if (rc == SQLITE_DONE) {
        // Statement executed, no data returned
        rc = sqlite3_finalize(cursor->stmt);
        cursor->stmt = NULL;
        cursor->step_result = SQLITE_DONE;
        if (sqlite3_debug) LOGM(TAG, "Step OK, no result");
        return mp_const_false;
    }
    else if (rc == SQLITE_ROW) cursor->step_result = SQLITE_ROW;
    else {
        cursor->step_result = SQLITE_DONE;
        if (sqlite3_debug) LOGQ(TAG, "Step error %d [%s]", rc, sqlite3_errstr(rc));
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error executing step function"));
    }

    return mp_const_true;
}

//------------------------------------------------------------------------------------------
static mp_obj_t cursor_execute_script(sqlite3* db, sqlite_Cursor_t *cursor, mp_obj_t sql_in)
{
    if (db == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Database not opened"));
    }

    const char *sql = (char *)mp_obj_str_get_str(sql_in);

    if (cursor->stmt) {
        // Finalize previous statement
        sqlite3_finalize(cursor->stmt);
        cursor->stmt = NULL;
    }

    cursor->step_result = SQLITE_DONE;
    cursor->column_number = 0;
    cursor->rows_fetched = 0;

    const char *pzTail = NULL;
    int n_executed = 0;
    mp_obj_t res = mp_const_true;
    int rc = -1;
    while (1) {
        if (cursor->stmt) {
            sqlite3_finalize(cursor->stmt);
            cursor->stmt = NULL;
        }
        // Prepare the statement
        rc = sqlite3_prepare_v2(db, sql, -1, &cursor->stmt, &pzTail);
        if (rc != SQLITE_OK) {
            cursor->stmt = NULL;
            if (sqlite3_debug) LOGQ(TAG, "Prepare error %d [%s] at %d", rc, sqlite3_errstr(rc), n_executed+1);
            res = mp_const_false;
            break;
        }
        if (cursor->stmt == NULL) break; // no more statements

        // Execute the step function once
        rc = sqlite3_step(cursor->stmt);
        if ((rc != SQLITE_DONE) && (rc != SQLITE_ROW)) {
            if (sqlite3_debug) LOGW(TAG, "Error in step function at %d (%d)", n_executed+1, rc);
            res = mp_const_false;
            break;
        }
        n_executed++;

        if (pzTail == NULL) break;
        sql = pzTail;
        pzTail = NULL;
    }
    if ((res == mp_const_true) && (sqlite3_debug)) LOGM(TAG, "Finished, %d statements executed", n_executed);

    if (cursor->stmt) {
        sqlite3_finalize(cursor->stmt);
        cursor->stmt = NULL;
    }

    return res;
}

//----------------------------------------------------------------------------
static void connection_open_db(pysqlite_Connection_t *self, mp_obj_t fname_in)
{
    int rc;

    if (!sqlite_state.is_init) {
        #if USER_MEM_ALLOC
        sqlite3_config(SQLITE_CONFIG_MALLOC, K210AllocMethods);
        #endif
        sqlite3_config(SQLITE_CONFIG_LOG, errorLogCallback, NULL);
        // Initialize SQLite3
        rc = sqlite3_initialize();
        if (rc) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error initializing SQLite3"));
        }
        sqlite_state.is_init = true;
    }

    char fullname[128] = {'\0'};
    sprintf(fullname, ":memory:");

    if (fname_in != mp_const_none) {
        char *fname = (char *)mp_obj_str_get_str(fname_in);

        if (strstr(fname, ":memory") != fname) {
            strcpy(fullname, fname);
        }
    }

    // Open database file
    rc = sqlite3_open(fullname, &self->db);
    if (rc) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error opening database file"));
    }
    sqlite_state.connected_db++;

    self->fname = pvPortMalloc(strlen(fullname)+1);
    sprintf(self->fname, "%s", fullname);

    self->autocommit = sqlite3_get_autocommit(self->db);
    if (sqlite3_debug) LOGM(TAG, "Autocommit: %s\n", (self->autocommit) ? "True" : "False");
}

// === Connection class =============================================

//------------------------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_connection_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    enum { ARG_file };
    //-----------------------------------------------------
    const mp_arg_t mod_sqlite3_connection_allowed_args[] = {
            { MP_QSTR_file,  MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(mod_sqlite3_connection_allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(mod_sqlite3_connection_allowed_args), mod_sqlite3_connection_allowed_args, args);

    pysqlite_Connection_t *self = m_new_obj(pysqlite_Connection_t);
    memset(self, 0, sizeof(pysqlite_Connection_t));
    self->base.type = &mod_sqlite3_Connection_type;

    connection_open_db(self, args[0].u_obj);
    return MP_OBJ_FROM_PTR(self);
}

//--------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_connection_close(mp_obj_t self_in) {
    pysqlite_Connection_t *self = (pysqlite_Connection_t *)self_in;

    if (self->db == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Database not opened"));
    }

    sqlite3_close(self->db);
    self->db = NULL;
    if (self->fname) vPortFree(self->fname);

    sqlite_state.connected_db--;
    if (sqlite_state.connected_db <= 0) {
        sqlite3_shutdown();
        sqlite_state.is_init = false;
    }

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_sqlite3_connection_close_obj, mod_sqlite3_connection_close);

//-------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_connection_open(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    const mp_arg_t allowed_args[] = {
            { MP_QSTR_file,         MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };
    pysqlite_Connection_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (self->db) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Already opened"));
    }

    connection_open_db(self, args[0].u_obj);

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_sqlite3_connection_open_obj, 1, mod_sqlite3_connection_open);

//---------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_connection_cursor(mp_obj_t self_in) {
    pysqlite_Connection_t *self = (pysqlite_Connection_t *)self_in;

    if (self->db == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Database not opened"));
    }

    pysqlite_Cursor_t *cursor = m_new_obj(pysqlite_Cursor_t);
    memset(cursor, 0, sizeof(pysqlite_Cursor_t));
    cursor->base.type = &mod_sqlite3_Cursor_type;
    cursor->connection = self;

    return MP_OBJ_FROM_PTR(cursor);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_sqlite3_connection_cursor_obj, mod_sqlite3_connection_cursor);

//----------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_connection_execute(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    const mp_arg_t allowed_args[] = {
            { MP_QSTR_sql,      MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_params,                     MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };
    pysqlite_Connection_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pysqlite_Cursor_t *cursor = m_new_obj(pysqlite_Cursor_t);
    memset(cursor, 0, sizeof(pysqlite_Cursor_t));
    cursor->base.type = &mod_sqlite3_Cursor_type;
    cursor->connection = self;

    cursor_execute(cursor->connection->db, &cursor->cursor, args[0].u_obj, args[1].u_obj);

    return MP_OBJ_FROM_PTR(cursor);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_sqlite3_connection_execute_obj, 1, mod_sqlite3_connection_execute);


// === Cursor class =============================================

//------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_cursor_execute(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    const mp_arg_t allowed_args[] = {
            { MP_QSTR_sql,      MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
            { MP_QSTR_params,                     MP_ARG_OBJ, { .u_obj = mp_const_none } },
    };
    pysqlite_Cursor_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    return cursor_execute(self->connection->db, &self->cursor, args[0].u_obj, args[1].u_obj);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_sqlite3_cursor_execute_obj, 1, mod_sqlite3_cursor_execute);

//------------------------------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_cursor_execute_script(mp_obj_t self_in, mp_obj_t sql_in) {

    pysqlite_Cursor_t *self = MP_OBJ_TO_PTR(self_in);

    return cursor_execute_script(self->connection->db, &self->cursor, sql_in);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_sqlite3_cursor_execute_script_obj, mod_sqlite3_cursor_execute_script);

//-----------------------------------------------
static void cursor_check(pysqlite_Cursor_t *self)
{
    if (self->connection->db == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Database not opened"));
    }

    if (self->cursor.stmt == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "No sql statement is prepared"));
    }

    if (self->cursor.column_number <= 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "No columns in sql result"));
    }
}

// Fetches the next row of a query result set, returning a single sequence,
// or None when no more data is available.
//-------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_cursor_fetchone(mp_obj_t self_in) {

    pysqlite_Cursor_t *self = (pysqlite_Cursor_t *)self_in;

    cursor_check(self);

    if (self->cursor.step_result != SQLITE_ROW) {
        sqlite3_finalize(self->cursor.stmt);
        self->cursor.stmt = NULL;
        return mp_const_none;
    }

    // Get row into tuple
    char *res = NULL;
    mp_obj_t tuple[self->cursor.column_number];

    for (int i=0; i<self->cursor.column_number; i++) {
        res = (char *)sqlite3_column_text(self->cursor.stmt, i);
        if (res) tuple[i] = mp_obj_new_str((char *)res, strlen(res));
        else tuple[i] = mp_const_none;
    }
    self->cursor.rows_fetched++;

    // Fetch next row
    self->cursor.step_result = sqlite3_step(self->cursor.stmt);
    if ((self->cursor.step_result != SQLITE_DONE) && (self->cursor.step_result != SQLITE_ROW)) {
        self->cursor.step_result = 0;
        sqlite3_finalize(self->cursor.stmt);
        self->cursor.stmt = NULL;
    }

    return mp_obj_new_tuple(self->cursor.column_number, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_sqlite3_cursor_fetchone_obj, mod_sqlite3_cursor_fetchone);

// Fetches all (remaining) rows of a query result, returning a list.
// Note that the cursor’s arraysize attribute can affect the performance of this operation.
// An empty list is returned when no rows are available.
//-------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_cursor_fetchall(mp_obj_t self_in) {

    pysqlite_Cursor_t *self = (pysqlite_Cursor_t *)self_in;

    cursor_check(self);

    mp_obj_dict_t *dct = mp_obj_new_dict(0);

    if (self->cursor.step_result != SQLITE_ROW) {
        sqlite3_finalize(self->cursor.stmt);
        self->cursor.stmt = NULL;
        return dct;
    }

    char *res = NULL;
    mp_obj_t tuple[self->cursor.column_number];

    int rec_no = 0;
    while (self->cursor.step_result == SQLITE_ROW) {
        self->cursor.rows_fetched++;
        for (int i=0; i < self->cursor.column_number; i++) {
            res = (char *)sqlite3_column_text(self->cursor.stmt, i);
            if (res) tuple[i] = mp_obj_new_str((char *)res, strlen(res));
            else tuple[i] = mp_const_none;
        }

        mp_obj_dict_store(dct,  mp_obj_new_int(rec_no), mp_obj_new_tuple(self->cursor.column_number, tuple));

        rec_no++;
        self->cursor.step_result = sqlite3_step(self->cursor.stmt);
    }

    sqlite3_finalize(self->cursor.stmt);
    self->cursor.stmt = NULL;
    return dct;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_sqlite3_cursor_fetchall_obj, mod_sqlite3_cursor_fetchall);

// Fetches the next set of rows of a query result, returning a list.
// An empty list is returned when no more rows are available.
//---------------------------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_cursor_fetchmany(size_t n_args, const mp_obj_t *args) {

    pysqlite_Cursor_t *self = MP_OBJ_TO_PTR(args[0]);

    cursor_check(self);

    int limit = 1;
    if (n_args > 1) limit = mp_obj_get_int(args[1]);
    if (limit == 1) {
        return mod_sqlite3_cursor_fetchone(self);
    }

    mp_obj_dict_t *dct = mp_obj_new_dict(0);

    if (self->cursor.step_result != SQLITE_ROW) {
        sqlite3_finalize(self->cursor.stmt);
        self->cursor.stmt = NULL;
        return dct;
    }

    char *res = NULL;
    mp_obj_t tuple[self->cursor.column_number];

    int rec_no = 0;
    while ((rec_no < limit) && (self->cursor.step_result == SQLITE_ROW)) {
        self->cursor.rows_fetched++;
        for (int i=0; i < self->cursor.column_number; i++) {
            res = (char *)sqlite3_column_text(self->cursor.stmt, i);
            if (res) tuple[i] = mp_obj_new_str((char *)res, strlen(res));
            else tuple[i] = mp_const_none;
        }

        mp_obj_dict_store(dct,  mp_obj_new_int(rec_no), mp_obj_new_tuple(self->cursor.column_number, tuple));

        rec_no++;
        self->cursor.step_result = sqlite3_step(self->cursor.stmt);
    }

    if (self->cursor.step_result == SQLITE_DONE) {
        sqlite3_finalize(self->cursor.stmt);
        self->cursor.stmt = NULL;
    }
    return dct;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sqlite3_cursor_fetchmany_obj, 1, 2, mod_sqlite3_cursor_fetchmany);

//----------------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_cursor_description(mp_obj_t self_in) {

    pysqlite_Cursor_t *self = (pysqlite_Cursor_t *)self_in;

    cursor_check(self);

    if (self->cursor.step_result != SQLITE_ROW) {
        sqlite3_finalize(self->cursor.stmt);
        self->cursor.stmt = NULL;
        return mp_const_none;
    }

    // Get column names into tuple
    const char *res = NULL;
    mp_obj_t tuple[self->cursor.column_number];

    for (int i=0; i<self->cursor.column_number; i++) {
        res = sqlite3_column_name(self->cursor.stmt, i);
        if (res) tuple[i] = mp_obj_new_str(res, strlen(res));
        else tuple[i] = mp_const_none;
    }

    return mp_obj_new_tuple(self->cursor.column_number, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_sqlite3_cursor_description_obj, mod_sqlite3_cursor_description);


// Connection class
//===========================================================================
STATIC const mp_rom_map_elem_t mod_sqlite3_connection_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_cursor),         MP_ROM_PTR(&mod_sqlite3_connection_cursor_obj) },
    { MP_ROM_QSTR(MP_QSTR_execute),        MP_ROM_PTR(&mod_sqlite3_connection_execute_obj) },
    { MP_ROM_QSTR(MP_QSTR_close),          MP_ROM_PTR(&mod_sqlite3_connection_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_open),           MP_ROM_PTR(&mod_sqlite3_connection_open_obj) },
};
STATIC MP_DEFINE_CONST_DICT(mod_sqlite3_connection_locals_dict, mod_sqlite3_connection_locals_dict_table);


//=================================================
const mp_obj_type_t mod_sqlite3_Connection_type = {
    { &mp_type_type },
    .name = MP_QSTR_connection,
    //.print = mod_sqlite3_connection_printinfo,
    .make_new = mod_sqlite3_connection_make_new,
    .locals_dict = (mp_obj_t)&mod_sqlite3_connection_locals_dict,
};

// Cursor class
//=======================================================================
STATIC const mp_rom_map_elem_t mod_sqlite3_cursor_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_execute),        MP_ROM_PTR(&mod_sqlite3_cursor_execute_obj) },
    { MP_ROM_QSTR(MP_QSTR_executescript),  MP_ROM_PTR(&mod_sqlite3_cursor_execute_script_obj) },
    { MP_ROM_QSTR(MP_QSTR_fetchone),       MP_ROM_PTR(&mod_sqlite3_cursor_fetchone_obj) },
    { MP_ROM_QSTR(MP_QSTR_fetchmany),      MP_ROM_PTR(&mod_sqlite3_cursor_fetchmany_obj) },
    { MP_ROM_QSTR(MP_QSTR_fetchall),       MP_ROM_PTR(&mod_sqlite3_cursor_fetchall_obj) },
    { MP_ROM_QSTR(MP_QSTR_description),    MP_ROM_PTR(&mod_sqlite3_cursor_description_obj) },
};
STATIC MP_DEFINE_CONST_DICT(mod_sqlite3_cursor_locals_dict, mod_sqlite3_cursor_locals_dict_table);

//=============================================
const mp_obj_type_t mod_sqlite3_Cursor_type = {
    { &mp_type_type },
    .name = MP_QSTR_cursor,
    //.print = mod_sqlite3_cursor_printinfo,
    .locals_dict = (mp_obj_t)&mod_sqlite3_cursor_locals_dict,
};


// usqlite3 module
//------------------------------------------------
STATIC mp_obj_t mod_sqlite3_debug(mp_obj_t dbg_in)
{
    sqlite3_debug = mp_obj_is_true(dbg_in);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_sqlite3_debug_obj, mod_sqlite3_debug);

#if USER_MEM_ALLOC
//------------------------------------------------------
STATIC mp_obj_t mod_sqlite3_alloc_debug(mp_obj_t dbg_in)
{
    sqlite3_debug = mp_obj_is_true(dbg_in);
    sqlite3_alloc_debug = mp_obj_is_true(dbg_in);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_sqlite3_alloc_debug_obj, mod_sqlite3_alloc_debug);
#endif

//===============================================================
STATIC const mp_rom_map_elem_t sqlite3_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_usqlite3) },
    { MP_ROM_QSTR(MP_QSTR_connect),     MP_ROM_PTR(&mod_sqlite3_Connection_type) },
    { MP_ROM_QSTR(MP_QSTR_debug),       MP_ROM_PTR(&mod_sqlite3_debug_obj) },
    #if USER_MEM_ALLOC
    { MP_ROM_QSTR(MP_QSTR_alloc_debug), MP_ROM_PTR(&mod_sqlite3_alloc_debug_obj) },
    #endif
};
STATIC MP_DEFINE_CONST_DICT(sqlite3_module_globals, sqlite3_module_globals_table);

//==========================================
const mp_obj_module_t mp_module_usqlite3 = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sqlite3_module_globals,
};

#endif
