/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "fpioa.h"
#include "gpiohs.h"
#include "sysctl.h"

#define GPIOHS_MAX_PINNO 32

volatile gpiohs_t *const gpiohs = (volatile gpiohs_t *)GPIOHS_BASE_ADDR;

/*
typedef struct _gpiohs_pin_instance
{
    size_t pin;
    gpio_pin_edge_t edge;
    void (*callback)();
    //plic_irq_callback_t gpiohs_callback;
    void *context;
} gpiohs_pin_instance_t;

static gpiohs_pin_instance_t pin_instance[32];
*/

void set_bit(volatile uint32_t *bits, uint32_t mask, uint32_t value)
{
    uint32_t org = (*bits) & ~mask;
    *bits = org | (value & mask);
}

void set_bit_offset(volatile uint32_t *bits, uint32_t mask, size_t offset, uint32_t value)
{
    set_bit(bits, mask << offset, value << offset);
}

void set_gpio_bit(volatile uint32_t *bits, size_t offset, uint32_t value)
{
    set_bit_offset(bits, 1, offset, value);
}

uint32_t get_bit(volatile uint32_t *bits, uint32_t mask, size_t offset)
{
    return ((*bits) & (mask << offset)) >> offset;
}

uint32_t get_gpio_bit(volatile uint32_t *bits, size_t offset)
{
    return get_bit(bits, 1, offset);
}




void gpiohs_set_drive_mode(uint8_t pin, gpio_drive_mode_t mode)
{
    int io_number = fpioa_get_io_by_function(FUNC_GPIOHS0 + pin);

    fpioa_pull_t pull = FPIOA_PULL_NONE;
    uint32_t dir = 0;

    switch(mode)
    {
        case GPIO_DM_INPUT:
            pull = FPIOA_PULL_NONE;
            dir = 0;
            break;
        case GPIO_DM_INPUT_PULL_DOWN:
            pull = FPIOA_PULL_DOWN;
            dir = 0;
            break;
        case GPIO_DM_INPUT_PULL_UP:
            pull = FPIOA_PULL_UP;
            dir = 0;
            break;
        case GPIO_DM_OUTPUT:
            pull = FPIOA_PULL_DOWN;
            dir = 1;
            break;
        default:
            break;
    }

    fpioa_set_io_pull(io_number, pull);
    volatile uint32_t *reg = dir ? gpiohs->output_en.u32 : gpiohs->input_en.u32;
    volatile uint32_t *reg_d = !dir ? gpiohs->output_en.u32 : gpiohs->input_en.u32;
    set_gpio_bit(reg_d, pin, 0);
    set_gpio_bit(reg, pin, 1);
}

gpio_pin_value_t gpiohs_get_pin(uint8_t pin)
{
    return get_gpio_bit(gpiohs->input_val.u32, pin);
}

void gpiohs_set_pin(uint8_t pin, gpio_pin_value_t value)
{
    set_gpio_bit(gpiohs->output_val.u32, pin, value);
}

