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
#include <FreeRTOS.h>
#include "task.h"
#include <fpioa.h>
#include <hal.h>
#include <kernel/driver_impl.hpp>
#include <plic.h>
#include <semphr.h>
#include <stdio.h>
#include <sysctl.h>
#include <timer.h>
#include <devices.h>

using namespace sys;

class k_pwm_driver : public pwm_driver, public static_object, public free_object_access
{
public:
    k_pwm_driver(uintptr_t base_addr, sysctl_clock_t clock)
        : pwm_(*reinterpret_cast<volatile kendryte_timer_t *>(base_addr)), clock_(clock)
    {
    }

    virtual void install() override
    {
        sysctl_clock_disable(clock_);
    }

    virtual void on_first_open() override
    {
        sysctl_clock_enable(clock_);
    }

    virtual void on_last_close() override
    {
        sysctl_clock_disable(clock_);
    }

    virtual uint32_t get_pin_count() override
    {
        return 4;
    }

    virtual double set_frequency(double frequency) override
    {
        uint32_t clk_freq = sysctl_clock_get_freq(clock_);

        /* frequency = clk_freq / periods */
        int64_t periods = (int64_t)(clk_freq / frequency);
        if ((periods <= 0) || (periods >= INT32_MAX)) return 0.0;

        frequency = clk_freq / (double)periods;
        periods_ = periods;
        return frequency;
    }

    // LoBo: modified
    virtual double set_active_duty_cycle_percentage(uint32_t pin, double duty_cycle_percentage, uint32_t *perc, uint32_t *periods) override
    {
        configASSERT(pin < get_pin_count());
        configASSERT(duty_cycle_percentage >= 0 && duty_cycle_percentage <= 1);

        uint32_t percent = (uint32_t)(duty_cycle_percentage * periods_);
        pwm_.channel[pin].load_count = periods_ - percent;
        pwm_.load_count2[pin] = percent;
        if (perc) *perc = percent;
        if (periods) * periods = periods_;
        return ((double)percent / (double)periods_); // LoBo
    }

    virtual void set_enable(uint32_t pin, bool enable) override
    {
        configASSERT(pin < get_pin_count());
        if (enable)
            pwm_.channel[pin].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
        else
            pwm_.channel[pin].control = TIMER_CR_INTERRUPT_MASK;
    }

    // LoBo: added function
    virtual uint32_t __attribute__((optimize("O0"))) set_enable_multi(uint32_t mask, bool enable, double delay_perc) override
    {
        if (mask == 0) return 0;
        // disable all requested pwm-s first
        for (int i=0; i<4; i++) {
            if ((mask >> i) & 1) {
                pwm_.channel[i].control = TIMER_CR_INTERRUPT_MASK;
            }
        }

        if (!enable) return mask; // disable requested, exit

        if (delay_perc == 0) {
            // === start requested pwm-s immediately ===
            if (mask & 1) {
                pwm_.channel[0].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
            }
            if (mask & 2) {
                pwm_.channel[1].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
            }
            if (mask & 4) {
                pwm_.channel[2].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
            }
            if (mask & 8) {
                pwm_.channel[3].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
            }
            return mask;
        }
        // === start requested pwm-s with delay ===
        uint64_t delay = (uint64_t)(delay_perc * periods_); // delay in timer clocks (~2.024 ns @494 MHz
        //only execute for limited delay range
        if ((delay < 100) || (delay > 200000000)) return 0;

        delay -= 62;
        if (delay < 200000) {
            taskENTER_CRITICAL();
        }

        _start_delay(delay);
        if (mask & 1) {
            _start_delay(delay);
            pwm_.channel[0].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
        }
        else {
            _start_delay(delay);
            __asm__ ("ADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\n");
        }
        if (mask & 2) {
            _start_delay(delay);
            pwm_.channel[1].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
        }
        else {
            _start_delay(delay);
            __asm__ ("ADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\n");
        }
        if (mask & 4) {
            _start_delay(delay);
            pwm_.channel[2].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
        }
        else {
            _start_delay(delay);
            __asm__ ("ADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\n");
        }
        if (mask & 8) {
            _start_delay(delay);
            pwm_.channel[3].control = TIMER_CR_INTERRUPT_MASK | TIMER_CR_PWM_ENABLE | TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
        }
        else {
            _start_delay(delay);
            __asm__ ("ADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\nADDI x0, x0, 0\n");
        }

        if (delay < 100000) {
            taskEXIT_CRITICAL();
        }
        return mask;
    }

private:
    // LoBo: added
    void _start_delay(uint32_t delay)
    {
        uint64_t end_time = read_csr64(mcycle) + delay;
        while (read_csr64(mcycle) < end_time) {
            ;
        }
    }

private:
    volatile kendryte_timer_t &pwm_;
    sysctl_clock_t clock_;

    uint32_t periods_;
};

static k_pwm_driver dev0_driver(TIMER0_BASE_ADDR, SYSCTL_CLOCK_TIMER0);
static k_pwm_driver dev1_driver(TIMER1_BASE_ADDR, SYSCTL_CLOCK_TIMER1);
static k_pwm_driver dev2_driver(TIMER2_BASE_ADDR, SYSCTL_CLOCK_TIMER2);

driver &g_pwm_driver_pwm0 = dev0_driver;
driver &g_pwm_driver_pwm1 = dev1_driver;
driver &g_pwm_driver_pwm2 = dev2_driver;
