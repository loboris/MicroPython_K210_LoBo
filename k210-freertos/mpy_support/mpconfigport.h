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


#include <sys/types.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <limits.h>

// ===========================================
// options to control how MicroPython is built
// ===========================================

#define MICRO_PY_DEFAULT_CPU_CLOCK              (400000000) // default cpu clock in Hz
#define MICRO_PY_DEFAULT_BAUDRATE               (115200)    // default REPL baudrate

#define USE_MICROPY_VM_HOOK_LOOP                (1)
#if USE_MICROPY_VM_HOOK_LOOP
extern void vm_loop_hook();
#define MICROPY_VM_HOOK_LOOP                    vm_loop_hook();
#endif

//-------------------------------------------------------------
// If set to 1, two main MicroPython tasks are created on boot,
// each running on its own K210 processor
// !!This feature is under development and does not work yet!!
#define MICROPY_USE_TWO_MAIN_TASKS              (0)
//-------------------------------------------------------------

// !MicroPython threads are always supported!
#define MICROPY_PY_THREAD                       (1)  // !DO NOT CHANGE!
#define MICROPY_PY_THREAD_GIL                   (1)  // !DO NOT CHANGE!
#define MICROPY_PY_THREAD_GIL_VM_DIVISOR        (32)
// stack entries are 64-bit, stack size in bytes is 8*MICROPY_THREAD_STACK_SIZE
#define MICROPY_THREAD_STACK_SIZE               (2048)
#define MICROPY_TASK_PRIORITY                   (8) // default thread priority
#define MICROPY_PY_THREAD_STATIC_TASK           (0)
//MicroPython heap size in bytes
#if MICROPY_USE_TWO_MAIN_TASKS
#define MICROPY_HEAP_SIZE                       (3 * 512 * 1024)
#else
#define MICROPY_HEAP_SIZE                       (3 * 1024 * 1024)
#endif

#define MICROPY_USE_DISPLAY                     (1)
#define MICROPY_USE_TFT                         (1)

// MicroPython main task stack size in bytes
#define MICROPY_TASK_STACK_SIZE                 (32 * 1024)
// MicroPython main task stack size in stack_type units (64-bits)
#define MICROPY_TASK_STACK_LEN                  (MICROPY_TASK_STACK_SIZE / sizeof(StackType_t))
#define MICROPY_ENABLE_PYSTACK                  (1)
#if MICROPY_ENABLE_PYSTACK
#define MICROPY_PYSTACK_SIZE                    (4096)
#else
#define MICROPY_PYSTACK_SIZE                    (0)
#endif
#define MICRO_PY_UARTHS_BUFFER_SIZE             (1280)

// object representation and NLR handling

//--------------------------------------------------------------------------------------------------
// A MicroPython object is a 64-bit word having the following form (called R):
//  - seeeeeee eeeeffff ffffffff ffffffff ffffffff ffffffff ffffffff ffffffff 64-bit fp, e != 0x7ff
//  - s1111111 11110000 00000000 00000000 00000000 00000000 00000000 00000000 +/- inf
//  - 01111111 11111000 00000000 00000000 00000000 00000000 00000000 00000000 normalised nan
//  - 01111111 11111101 iiiiiiii iiiiiiii iiiiiiii iiiiiiii iiiiiiii iiiiiii1 small int
//  - 01111111 11111110 00000000 00000000 qqqqqqqq qqqqqqqq qqqqqqqq qqqqqqq1 str
//  - 01111111 11111100 00000000 00000000 pppppppp pppppppp pppppppp pppppp00 ptr (4 byte alignment)
// Stored as O = R + 0x8004000000000000, retrieved as R = O - 0x8004000000000000.
// This makes pointers have all zeros in the top 32 bits.
// Small-ints and strs have 1 as LSB to make sure they don't look like pointers
// to the garbage collector.
#define MICROPY_OBJ_REPR                        (MICROPY_OBJ_REPR_D)
//--------------------------------------------------------------------------------------------------

#define MICROPY_NLR_SETJMP                      (1)
#define MICROPY_READER_VFS                      (1)

// MCU definition
#define MP_ENDIANNESS_LITTLE                    (1)
#define MICROPY_NO_ALLOCA                       (0)

// optimisations
#define MICROPY_OPT_COMPUTED_GOTO               (1)
#define MICROPY_OPT_MPZ_BITWISE                 (1)

#define MICROPY_USE_INTERNAL_PRINTF             (0)

#define MICROPY_ENABLE_COMPILER                 (1)

#define MICROPY_QSTR_BYTES_IN_HASH              (1)
#define MICROPY_ALLOC_PATH_MAX                  (128)
#define MICROPY_ALLOC_PARSE_CHUNK_INIT          (16)
#define MICROPY_EMIT_X64                        (0)
#define MICROPY_EMIT_THUMB                      (0)
#define MICROPY_EMIT_INLINE_THUMB               (0)
#define MICROPY_EMIT_INLINE_THUMB_ARMV7M        (0)

#define MICROPY_COMP_MODULE_CONST               (1)
#define MICROPY_COMP_CONST                      (1)
#define MICROPY_COMP_DOUBLE_TUPLE_ASSIGN        (1)
#define MICROPY_COMP_TRIPLE_TUPLE_ASSIGN        (10)

#define MICROPY_ENABLE_SCHEDULER                (1)
#define MICROPY_SCHEDULER_DEPTH                 (8)

#define MICROPY_ENABLE_FINALISER                (0)
#define MICROPY_STACK_CHECK                     (0)
#define MICROPY_ENABLE_EMERGENCY_EXCEPTION_BUF  (1)
#define MICROPY_KBD_EXCEPTION                   (1)
#define MICROPY_REPL_EMACS_KEYS                 (1)
#define MICROPY_REPL_AUTO_INDENT                (1)
#define MICROPY_HAL_HAS_VT100                   (1)

#define MICROPY_CPYTHON_COMPAT                  (1)
#define MICROPY_STREAMS_NON_BLOCK               (0)
#define MICROPY_STREAMS_POSIX_API               (1)
#define MICROPY_MODULE_BUILTIN_INIT             (1)
#define MICROPY_MODULE_WEAK_LINKS               (1)

#define MICROPY_PERSISTENT_CODE_LOAD            (1)
#define MICROPY_PERSISTENT_CODE_SAVE            (0)
#define MICROPY_OPT_CACHE_MAP_LOOKUP_IN_BYTECODE    (0)

#define MICROPY_COMP_RETURN_IF_EXPR             (1)

#define MICROPY_PY_COLLECTIONS                  (1)
#define MICROPY_PY_COLLECTIONS_DEQUE            (1)
#define MICROPY_PY_COLLECTIONS_ORDEREDDICT      (1)

extern const struct _mp_print_t mp_debug_print;
extern const struct _mp_print_t mp_debug_print;
#define MICROPY_DEBUG_VERBOSE                   (0)
#define MICROPY_DEBUG_PRINTER                   (&mp_debug_print)

#define MICROPY_MEM_STATS                       (1)
#define MICROPY_DEBUG_PRINTERS                  (0)
#define MICROPY_ENABLE_GC                       (1)
#define MICROPY_GC_ALLOC_THRESHOLD              (1)
#define MICROPY_REPL_EVENT_DRIVEN               (0)
#define MICROPY_MALLOC_USES_ALLOCATED_SIZE      200*1024
#define MICROPY_HELPER_REPL                     (1)
#define MICROPY_HELPER_LEXER_UNIX               (1)
#define MICROPY_ENABLE_SOURCE_LINE              (1)
#define MICROPY_ENABLE_DOC_STRING               (0)
#define MICROPY_ERROR_REPORTING                 (MICROPY_ERROR_REPORTING_TERSE)

#define MICROPY_BUILTIN_METHOD_CHECK_SELF_ARG   (0)
#define MICROPY_PY_ASYNC_AWAIT                  (0)

#define MICROPY_PY_BUILTINS_BYTEARRAY           (1)
#define MICROPY_PY_BUILTINS_MEMORYVIEW          (1)
#define MICROPY_PY_BUILTINS_FROZENSET           (1)
#define MICROPY_PY_BUILTINS_SET                 (1)
#define MICROPY_PY_BUILTINS_PROPERTY            (1)
#define MICROPY_PY_BUILTINS_MIN_MAX             (1)
#define MICROPY_PY_BUILTINS_STR_OP_MODULO       (1)
#define MICROPY_PY_GC                           (1)
#define MICROPY_MODULE_FROZEN_STR               (0)
#define MICROPY_MODULE_FROZEN_MPY               (1)
#define MICROPY_LONGINT_IMPL                    (MICROPY_LONGINT_IMPL_MPZ) //(MICROPY_LONGINT_IMPL_LONGLONG)

//-----------------------------
// control over Python builtins
//-----------------------------
#define MICROPY_FLOAT_IMPL                      (MICROPY_FLOAT_IMPL_DOUBLE)
#define MICROPY_PY_BUILTINS_HELP                (1)
#define MICROPY_PY_BUILTINS_HELP_TEXT           kendryte_k210_help_text
#define MICROPY_PY_BUILTINS_HELP_MODULES        (1)
#define MICROPY_PY_BUILTINS_COMPLEX             (1)
#define MICROPY_PY_BUILTINS_FLOAT               (1)

#define MICROPY_PY_STR_BYTES_CMP_WARN           (1)
#define MICROPY_PY_BUILTINS_STR_UNICODE         (1)
#define MICROPY_PY_BUILTINS_STR_CENTER          (1)
#define MICROPY_PY_BUILTINS_STR_PARTITION       (1)
#define MICROPY_PY_BUILTINS_STR_SPLITLINES      (1)
#define MICROPY_PY_BUILTINS_SLICE               (1)
#define MICROPY_PY_BUILTINS_SLICE_ATTRS         (1)
#define MICROPY_PY_BUILTINS_RANGE_ATTRS         (1)
#define MICROPY_PY_BUILTINS_ROUND_INT           (1)
#define MICROPY_PY_BUILTINS_TIMEOUTERROR        (1)
#define MICROPY_PY_ALL_SPECIAL_METHODS          (1)
#define MICROPY_PY_BUILTINS_COMPILE             (1)
#define MICROPY_PY_BUILTINS_ENUMERATE           (1)
#define MICROPY_PY_BUILTINS_EXECFILE            (1)
#define MICROPY_PY_BUILTINS_FILTER              (1)
#define MICROPY_PY_BUILTINS_REVERSED            (1)
#define MICROPY_PY_BUILTINS_NOTIMPLEMENTED      (1)
#define MICROPY_PY_BUILTINS_INPUT               (1)
#define MICROPY_PY_BUILTINS_POW3                (1)
#define MICROPY_PY___FILE__                     (1)
#define MICROPY_PY_MICROPYTHON_MEM_INFO         (1)
#define MICROPY_PY_ARRAY                        (1)
#define MICROPY_PY_ARRAY_SLICE_ASSIGN           (1)
#define MICROPY_PY_MATH                         (1)
#define MICROPY_PY_MATH_SPECIAL_FUNCTIONS       (1)
#define MICROPY_PY_CMATH                        (1)
#define MICROPY_PY_IO                           (1)
#define MICROPY_PY_IO_IOBASE                    (1)
#define MICROPY_PY_IO_FILEIO                    (1)
#define MICROPY_PY_IO_BYTESIO                   (1)
#define MICROPY_PY_IO_BUFFEREDWRITER            (1)
#define MICROPY_PY_STRUCT                       (1)
#define MICROPY_PY_SYS                          (1)
#define MICROPY_PY_SYS_MAXSIZE                  (1)
#define MICROPY_PY_SYS_MODULES                  (1)
#define MICROPY_PY_SYS_EXIT                     (1)
#define MICROPY_PY_SYS_STDFILES                 (1)
#define MICROPY_PY_SYS_STDIO_BUFFER             (1)
#define MICROPY_PY_UERRNO                       (1)
#define MICROPY_PY_USELECT                      (0)
#define MICROPY_PY_UTIME_MP_HAL                 (1)

#define mp_type_fileio                          mp_type_vfs_spiffs_fileio
#define mp_type_textio                          mp_type_vfs_spiffs_textio
#define MICROPY_VFS                             (1)
#define MICROPY_VFS_SPIFFS                      (1)
#define MICROPY_VFS_SDCARD                      (1)
// Vfs_FAT is not used!
#define MICROPY_VFS_FAT                         (0) // !do not change!
#define MICROPY_FATFS_REENTRANT                 (1)

//#define MP_SSIZE_MAX                            (0x7fffffff)
#define MP_SSIZE_MAX                            (0x7fffffffffffffff)

#define                                         _USE_MKFS 1
#define                                         _FS_READONLY 0

// use vfs's functions for import stat and builtin open
#define mp_import_stat mp_vfs_import_stat
#define mp_builtin_open mp_vfs_open
#define mp_builtin_open_obj mp_vfs_open_obj
#define MICROPY_PY_ATTRTUPLE                    (1)

#define MICROPY_PY_FUNCTION_ATTRS               (1)
#define MICROPY_PY_DESCRIPTORS                  (1)

//-----------------
// extended modules
//-----------------
#define MICROPY_PY_UCTYPES                      (1)
#define MICROPY_PY_UZLIB                        (1)
#define MICROPY_PY_UJSON                        (1)
#define MICROPY_PY_URE                          (1)
#define MICROPY_PY_URE_SUB                      (1)
#define MICROPY_PY_UHEAPQ                       (1)
#define MICROPY_PY_UTIMEQ                       (0)
#define MICROPY_PY_UTIMEQ_K210                  (1)
// MicroPython implementation of hash functions is not used!
#define MICROPY_PY_UHASHLIB                     (0) // !do not change!
#define MICROPY_PY_UHASHLIB_MD5                 (0) // !do not change!
#define MICROPY_PY_UHASHLIB_SHA1                (0) // !do not change!
#define MICROPY_PY_UHASHLIB_SHA256              (0) // !do not change!
// K210 specific implementation of hash functions
#define MICROPY_PY_UHASHLIB_K210                (1)
#define MICROPY_PY_UHASHLIB_MD5_K210            (1)
#define MICROPY_PY_UHASHLIB_SHA1_K210           (1)
#define MICROPY_PY_UHASHLIB_SHA256_K210         (1)

#define MICROPY_PY_UCRYPTOLIB                   (0) // !do not change!
#define MICROPY_PY_UCRYPTOLIB_K210              (1)

#define MICROPY_PY_UBINASCII                    (1)
#define MICROPY_PY_UBINASCII_CRC32              (1)
#define MICROPY_PY_URANDOM                      (1)
#define MICROPY_PY_URANDOM_EXTRA_FUNCS          (1)
#define MICROPY_PY_OS_DUPTERM                   (0)

#define MICROPY_PY_MACHINE_PULSE                (1)
#define MICROPY_PY_MACHINE_I2C                  (0)
#define MICROPY_PY_MACHINE_SPI                  (0)
#define MICROPY_PY_MACHINE_SPI_MSB              (0)
#define MICROPY_PY_MACHINE_SPI_LSB              (0)
#define MICROPY_PY_MACHINE_SPI_MAKE_NEW         machine_hw_spi_make_new
#define MICROPY_HW_SOFTSPI_MIN_DELAY            (0)
#define MICROPY_HW_SOFTSPI_MAX_BAUDRATE         (ets_get_cpu_frequency() * 1000000 / 200) // roughly
#define MICROPY_PY_USSL                         (0)
#define MICROPY_SSL_MBEDTLS                     (0)
#define MICROPY_PY_USSL_FINALISER               (0)
#define MICROPY_PY_WEBSOCKET                    (1)
#define MICROPY_PY_WEBREPL                      (0)
#define MICROPY_PY_FRAMEBUF                     (0)
#define MICROPY_PY_USOCKET_EVENTS               (MICROPY_PY_WEBREPL)

// disable ext str pool
#define MICROPY_QSTR_EXTRA_POOL                 mp_qstr_frozen_const_pool

// type definitions for the specific machine

#define MICROPY_MAKE_POINTER_CALLABLE(p) ((void*)((mp_uint_t)(p) | 1))

// This port is intended to be 32-bit, but unfortunately, int32_t for
// different targets may be defined in different ways - either as int
// or as long. This requires different printf formatting specifiers
// to print such value. So, we avoid int32_t and use int directly.
#define UINT_FMT "%u"
#define INT_FMT "%d"

typedef int64_t mp_int_t;   // must be pointer size
typedef uint64_t mp_uint_t; // must be pointer size
typedef int64_t mp_off_t;

#define MP_PLAT_PRINT_STRN(str, len) mp_hal_stdout_tx_strn_cooked(str, len)

// extra built in names to add to the global namespace
#define MICROPY_PORT_BUILTINS \
    { MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&mp_builtin_open_obj) },

extern const struct _mp_obj_module_t machine_module;
extern const struct _mp_obj_module_t uos_module;
extern const struct _mp_obj_module_t utime_module;
extern const struct _mp_obj_module_t mp_module_ymodem;

#ifdef MICROPY_PY_UHASHLIB_K210
extern const struct _mp_obj_module_t mp_module_uhashlib;
#define BUILTIN_MODULE_UHASHLIB_K210 { MP_ROM_QSTR(MP_QSTR_uhashlib), MP_ROM_PTR(&mp_module_uhashlib) },
#else
#define BUILTIN_MODULE_UHASHLIB_K210
#endif

#ifdef MICROPY_PY_UCRYPTOLIB_K210
extern const struct _mp_obj_module_t mp_module_ucryptolib;
#define BUILTIN_MODULE_UCRYPTOLIB_K210 { MP_ROM_QSTR(MP_QSTR_ucryptolib), MP_ROM_PTR(&mp_module_ucryptolib) },
#else
#define BUILTIN_MODULE_UCRYPTOLIB_K210
#endif

#ifdef MICROPY_USE_DISPLAY
extern const struct _mp_obj_module_t mp_module_display;
#define BUILTIN_MODULE_DISPLAY { MP_OBJ_NEW_QSTR(MP_QSTR_display), (mp_obj_t)&mp_module_display },
#else
#define BUILTIN_MODULE_DISPLAY
#endif

#ifdef MICROPY_PY_UTIMEQ_K210
extern const struct _mp_obj_module_t mp_module_utimeq;
#define BUILTIN_MODULE_UTIMEQ_K210 { MP_OBJ_NEW_QSTR(MP_QSTR_utimeq), (mp_obj_t)&mp_module_utimeq },
#else
#define BUILTIN_MODULE_UTIMEQ_K210
#endif

#define MICROPY_PORT_BUILTIN_MODULES \
    { MP_OBJ_NEW_QSTR(MP_QSTR_os),          (mp_obj_t)&uos_module }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_time),        (mp_obj_t)&utime_module }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_re),          (mp_obj_t)&mp_module_ure }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_io),          (mp_obj_t)&mp_module_io }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_binascii),    (mp_obj_t)&mp_module_ubinascii }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_collections), (mp_obj_t)&mp_module_collections }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_uos),         (mp_obj_t)&uos_module }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_utime),       (mp_obj_t)&utime_module }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_machine),     (mp_obj_t)&machine_module }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_ymodem),      (mp_obj_t)&mp_module_ymodem }, \
    BUILTIN_MODULE_UHASHLIB_K210 \
    BUILTIN_MODULE_UCRYPTOLIB_K210 \
    BUILTIN_MODULE_DISPLAY \
    BUILTIN_MODULE_UTIMEQ_K210 \

/*
#define MICROPY_PORT_BUILTIN_MODULE_WEAK_LINKS \
    { MP_OBJ_NEW_QSTR(MP_QSTR_os),          (mp_obj_t)&uos_module }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_time),        (mp_obj_t)&utime_module }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_re),          (mp_obj_t)&mp_module_ure }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_io),          (mp_obj_t)&mp_module_io }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_binascii),    (mp_obj_t)&mp_module_ubinascii }, \
    { MP_OBJ_NEW_QSTR(MP_QSTR_collections), (mp_obj_t)&mp_module_collections }, \
*/

#define MICROPY_PY_MACHINE                  (1)
#define MICROPY_PY_MACHINE_PIN_MAKE_NEW     mp_pin_make_new

// We need to provide a declaration/definition of alloca()
#include <alloca.h>

#define MICROPY_HW_BOARD_NAME       "Sipeed_board"
#define MICROPY_HW_MCU_NAME         "Kendryte-K210"
#define MICROPY_PY_SYS_PLATFORM     "Sipeed"

#define MP_STATE_PORT MP_STATE_VM

#define MICROPY_PORT_ROOT_POINTERS \
    const char *readline_hist[32];
