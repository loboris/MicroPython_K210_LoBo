/*
    Created by Chris Morgan based on the nodemcu project driver.
    Copyright 2017 Chris Morgan <chmorgan@gmail.com>
    Copyright (c) 2019 LoBo (https://github.com/loboris)

    Ported to Kendryte K210 for low-level signal generation by LoBo.

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the
    "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to
    the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
    LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
    WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    Much of the code was inspired by Derek Yerger's code, though I don't
    think much of that remains.  In any event that was..
        (copyleft) 2006 by Derek Yerger - Free to distribute freely.

    The CRC code was excerpted and inspired by the Dallas Semiconductor
    sample code bearing this copyright.
    //---------------------------------------------------------------------------
    // Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
    //
    // Permission is hereby granted, free of charge, to any person obtaining a
    // copy of this software and associated documentation files (the "Software"),
    // to deal in the Software without restriction, including without limitation
    // the rights to use, copy, modify, merge, publish, distribute, sublicense,
    // and/or sell copies of the Software, and to permit persons to whom the
    // Software is furnished to do so, subject to the following conditions:
    //
    // The above copyright notice and this permission notice shall be included
    // in all copies or substantial portions of the Software.
    //
    // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
    // OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    // MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    // IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
    // OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
    // ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    // OTHER DEALINGS IN THE SOFTWARE.
    //
    // Except as contained in this notice, the name of Dallas Semiconductor
    // shall not be used except as stated in the Dallas Semiconductor
    // Branding Policy.
    //--------------------------------------------------------------------------
*/

#include "owb_k210.h"

#include "syslog.h"
#include "gpiohs.h"
#include "mphalport.h"

#undef OW_DEBUG
#define  OW_DEBUG 1

// bus reset: duration of low phase [ns]
#define OW_DURATION_RESET 480000
// overall slot duration
#define OW_DURATION_SLOT 75000
// write 1 slot and read slot durations [ns]
#define OW_DURATION_1_LOW    1000
#define OW_DURATION_1_HIGH (OW_DURATION_SLOT - OW_DURATION_1_LOW)
// write 0 slot durations [ns]
#define OW_DURATION_0_LOW   62000
#define OW_DURATION_0_HIGH (OW_DURATION_SLOT - OW_DURATION_0_LOW)
// sample time for read slot
#define OW_DURATION_SAMPLE  (15000-OW_DURATION_1_LOW)
// RX idle threshold
// needs to be larger than any duration occurring during write slots
#define OW_DURATION_RX_IDLE (OW_DURATION_SLOT+25000)


volatile gpiohs_t* const gpiohs_1wire = (volatile gpiohs_t*)GPIOHS_BASE_ADDR;

static const char * TAG = "owb_rmt";
static owb_k210_driver_info *driver_info = NULL;
static double clk_ns = 1;

//-------------------------------
static void ow_delay(uint32_t ns)
{
    uint64_t start_clk = read_csr64(mcycle);
    uint64_t end_clk = start_clk - 50 + (uint64_t)((double)ns / clk_ns);
    while (read_csr64(mcycle) < end_clk) {
        ;
    }
}

//-------------------------------------------------------------
static void gpiohs_set_pin(uint8_t pin, gpio_pin_value_t value)
{
    uint32_t org = (*gpiohs_1wire->output_val.u32) & ~(1 << pin);
    *gpiohs_1wire->output_val.u32 = org | (value & (1 << pin));
}

//-------------------------------------
static bool gpiohs_get_pin(uint8_t pin)
{
    uint32_t org = (*gpiohs_1wire->input_val.u32) & (1 << pin);
    return (org != 0);
}

//-------------------------------------------------------------
static void gpiohs_set_dir(uint8_t pin, gpio_drive_mode_t mode)
{
    if (mode == GPIO_DM_OUTPUT) {
        uint32_t org = (*gpiohs_1wire->output_en.u32) & ~(1 << pin);
        *gpiohs_1wire->output_en.u32 = org | (1 << pin);
    }
    else {
        uint32_t org = (*gpiohs_1wire->output_en.u32) & ~(1 << pin);
        *gpiohs_1wire->output_en.u32 = org;
    }
}

//============================================
void ow_parasite_power(gpio_pin_value_t onoff)
{
    gpiohs_set_pin(driver_info->gpio, onoff);
    gpiohs_set_dir(driver_info->gpio, GPIO_DM_OUTPUT);
}

//-----------------------------------------------------------------
static owb_status _reset( const OneWireBus *bus, bool *is_present )
{
    int res = OWB_STATUS_OK;
    gpiohs_set_pin(driver_info->gpio, GPIO_PV_LOW);
    gpiohs_set_dir(driver_info->gpio, GPIO_DM_OUTPUT);
    ow_delay(OW_DURATION_RESET);
    gpiohs_set_dir(driver_info->gpio, GPIO_DM_INPUT);
    ow_delay(OW_DURATION_SLOT);
    bool _is_present = !gpiohs_get_pin(driver_info->gpio);

    *is_present = _is_present;

    vTaskDelay(2);
    LOGD(TAG, "%s(): _is_present %d", __func__, _is_present);

    return res;
}

//-------------------------------------------
static void _encode_write_slot( uint8_t val )
{
    taskENTER_CRITICAL();
    gpiohs_set_pin(driver_info->gpio, GPIO_PV_LOW);
    gpiohs_set_dir(driver_info->gpio, GPIO_DM_OUTPUT);
    if (val) {
        // write "1" slot
        ow_delay(OW_DURATION_1_LOW);
        gpiohs_set_dir(driver_info->gpio, GPIO_DM_INPUT);
        taskEXIT_CRITICAL();
        ow_delay(OW_DURATION_1_HIGH);
    } else {
        // write "0" slot
        ow_delay(OW_DURATION_0_LOW);
        gpiohs_set_dir(driver_info->gpio, GPIO_DM_INPUT);
        taskEXIT_CRITICAL();
        ow_delay(OW_DURATION_0_HIGH);
    }
}

/** NOTE: The data is shifted out of the low bits, eg. it is written in the order of lsb to msb */
//-----------------------------------------------------------------------------------------------
static owb_status _write_bits( const OneWireBus * bus, uint8_t out, int number_of_bits_to_write )
{
    if (number_of_bits_to_write > 8) return OWB_STATUS_TOO_MANY_BITS;

    // write requested bits as pattern to TX buffer
    for (int i = 0; i < number_of_bits_to_write; i++) {
        _encode_write_slot( out & 0x01 );
        out >>= 1;
    }

    return OWB_STATUS_OK;
}

//-----------------------------------
static bool _encode_read_slot( void )
{
    bool bitval = 1;
    taskENTER_CRITICAL();
    // shortly force 0
    gpiohs_set_pin(driver_info->gpio, GPIO_PV_LOW);
    gpiohs_set_dir(driver_info->gpio, GPIO_DM_OUTPUT);
    ow_delay(OW_DURATION_1_LOW);
    gpiohs_set_dir(driver_info->gpio, GPIO_DM_INPUT);
    ow_delay(OW_DURATION_SAMPLE);

    bitval = gpiohs_get_pin(driver_info->gpio);
    taskEXIT_CRITICAL();
    ow_delay(OW_DURATION_RX_IDLE);
    return bitval;
}

/** NOTE: Data is read into the high bits, eg. each bit read is shifted down before the next bit is read */
//---------------------------------------------------------------------------------------------
static owb_status _read_bits( const OneWireBus * bus, uint8_t *in, int number_of_bits_to_read )
{
    if (number_of_bits_to_read > 8)
    {
        LOGE(TAG, "_read_bits() OWB_STATUS_TOO_MANY_BITS");
        return OWB_STATUS_TOO_MANY_BITS;
    }

    uint8_t read_data = 0;
    // generate requested read slots
    for (int i = 0; i < number_of_bits_to_read; i++)
    {
        read_data |= (_encode_read_slot() << i);
    }

    *in = read_data;
    return OWB_STATUS_OK;
}

//----------------------------------------------------
static owb_status _uninitialize(const OneWireBus *bus)
{

    return OWB_STATUS_OK;
}

//-------------------------------------------
static struct owb_driver rmt_function_table =
{
    .name = "owb_rmt",
    .uninitialize = _uninitialize,
    .reset = _reset,
    .write_bits = _write_bits,
    .read_bits = _read_bits
};

//-------------------------------------------------------------------
static owb_status _init(owb_k210_driver_info *info, uint8_t gpio_num)
{
    clk_ns = (double)(1e9 / (double)sysctl_clock_get_freq(SYSCTL_CLOCK_CPU));
    driver_info = info;

    info->bus.driver = &rmt_function_table;
    info->gpio = gpio_num;

    // === Configure pin ===
    if (gpio_num >= FPIOA_NUM_IO) return OWB_STATUS_HW_ERROR;
    if (mp_used_pins[gpio_num].func != GPIO_FUNC_NONE) return OWB_STATUS_HW_ERROR;
    if (mpy_timers_used[11] != NULL) return OWB_STATUS_HW_ERROR;

    int pio_num = gpiohs_get_free();
    if (pio_num < 0) return OWB_STATUS_HW_ERROR;

    info->pin = gpio_num;
    info->gpio = pio_num;
    info->pin_mode = GPIO_DM_INPUT_PULL_UP;
    // configure the pin for gpio
    if (fpioa_set_function(info->pin, FUNC_GPIOHS0 + info->gpio) < 0) {
        gpiohs_set_free(info->gpio);
        return OWB_STATUS_HW_ERROR;
    }

    gpio_set_drive_mode(gpiohs_handle, info->gpio, info->pin_mode);

    mp_used_pins[info->pin].func = GPIO_FUNC_1WIRE;
    mp_used_pins[info->pin].usedas = GPIO_USEDAS_1WIRE;
    mp_used_pins[info->pin].gpio = info->gpio;
    mp_used_pins[info->pin].fpioa_func = FUNC_GPIOHS0 + info->gpio;

    // Disable gpio interrupt
    gpio_set_pin_edge(gpiohs_handle, info->gpio, GPIO_PE_NONE);

    return OWB_STATUS_OK;
}

//============================================================================
OneWireBus* owb_k210_initialize( owb_k210_driver_info *info, uint8_t gpio_num)
{
    LOGI(TAG, "%s(): gpio_num: %d", __func__, gpio_num);

    owb_status status = _init(info, gpio_num);
    if (status != OWB_STATUS_OK) {
        LOGE(TAG, "_init() failed with status %d", status);
    }

    return &(info->bus);
}
