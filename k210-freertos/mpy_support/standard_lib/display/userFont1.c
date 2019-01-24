/*
 * This file is part of the MicroPython ESP32 project, https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 LoBo (https://github.com/loboris)
 *
*/

/*
 * This is dummy userFon
 * You can replace it with any font C file created by Font Creator tool
 * https://loboris.eu/ttf2fon/
 * !!! KEEP THE FILE NAME THE SAME !!!
 * !!! REPLACE THE FONT ARRAY NAME WITH THE NAME IN THIS FILE (userFontN[]) !!!
 */

#include "mpconfigport.h"

#ifdef MICROPY_USE_DISPLAY

const unsigned char userFont1[] =
{
0x00, 0x00, 0x00, 0x00,

// Terminator
0x00
};

#endif
