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
#include <limits.h>
#include <plic.h>
#include <semphr.h>
#include <stdio.h>
#include <sysctl.h>
#include <timer.h>
#include <utility.h>

#define LOAD_COUNT_VALUE        1000000000UL
#define MAX_LOAD_COUNT_VALUE    (LOAD_COUNT_VALUE*5)

using namespace sys;

static void *irq_context[3][4];

class k_timer_driver : public timer_driver, public static_object, public exclusive_object_access
{
public:
    k_timer_driver(uintptr_t base_addr, sysctl_clock_t clock, plic_irq_t irq, uint32_t num, uint32_t channel)
        : timer_(*reinterpret_cast<volatile kendryte_timer_t *>(base_addr)), clock_(clock), irq_(irq), num_(num), channel_(channel)
    {
    }

    virtual void install() override
    {
        irq_context[num_][channel_] = this;
        if (channel_ == 0)
        {
            sysctl_clock_enable(clock_);

            readl(&timer_.eoi);
            size_t i;
            for (i = 0; i < 4; i++)
                timer_.channel[i].control = TIMER_CR_INTERRUPT_MASK;

            pic_set_irq_handler(irq_, timer_isr, irq_context[num_]);
            pic_set_irq_handler(irq_ + 1, timer_isr, irq_context[num_]);
            pic_set_irq_priority(irq_, 1);
            pic_set_irq_priority(irq_ + 1, 1);
            pic_set_irq_enable(irq_, 1);
            pic_set_irq_enable(irq_ + 1, 1);
            sysctl_clock_disable(clock_);
        }
    }

    virtual void on_first_open() override
    {
        sysctl_clock_enable(clock_);
        running_time_ = 0;
    }

    virtual void on_last_close() override
    {
        sysctl_clock_disable(clock_);
    }

    virtual size_t set_interval(size_t nanoseconds) override
    {
        remain_ = 0;
        uint32_t clk_freq = sysctl_clock_get_freq(clock_);
        resolution_ = 1e9 / clk_freq;
        load_count_1sec_ = (uint32_t)((double)LOAD_COUNT_VALUE / resolution_);

        // interval in timer increments
        uint64_t value = (uint64_t)((double)nanoseconds / resolution_);
        if (value < 1) return 0; // interval to small

        // LoBo: enable 64-bit intervals
        // actual interval in nanoseconds
        interval_ = (size_t)(resolution_ * (double)value);

        remain_ = 0;
        load_remain_ = 0;
        current_value_ = 0;
        if (interval_ > MAX_LOAD_COUNT_VALUE) {
            load_count_ = value % load_count_1sec_;
            remain_ = value / load_count_1sec_;
            if (load_count_ < (load_count_1sec_/2)) {
                load_count_ += load_count_1sec_;
                if (load_count_ == load_count_1sec_) load_count_++;
                remain_--;
            }
            load_remain_ = remain_;
            // interval = (remain_ * load_count_1sec_) + load_count_
        }
        else load_count_ = (uint32_t)value;

        timer_.channel[channel_].load_count = load_count_;
        //timer_.channel[channel_].current_value = load_count_;

        return interval_;
    }

    virtual void set_on_tick(timer_on_tick_t on_tick, void *userdata) override
    {
        ontick_data_ = userdata;
        on_tick_ = on_tick;
    }

    virtual void set_enable(bool enable) override
    {
        if (enable)
            timer_.channel[channel_].control = TIMER_CR_USER_MODE | TIMER_CR_ENABLE;
        else
            timer_.channel[channel_].control = TIMER_CR_INTERRUPT_MASK;
    }

    virtual size_t get_value(double *res, size_t *runtime) override
    {
        if (res) *res = resolution_;

        uint32_t current_value = 0;
        size_t val = 0;

        taskENTER_CRITICAL();
        if (interval_ > MAX_LOAD_COUNT_VALUE) {
            current_value = timer_.channel[channel_].load_count - timer_.channel[channel_].current_value;
            val = current_value_;
        }
        else {
            current_value = timer_.channel[channel_].load_count - timer_.channel[channel_].current_value;
        }
        taskEXIT_CRITICAL();

        if (runtime) {
            if (*runtime == 0) running_time_ = 0;
            *runtime = running_time_+current_value;
        }
        // returns current timer value in timer increments units
        return (val + current_value);
    }

private:
    static void timer_isr(void *userdata)
    {
        k_timer_driver **context = reinterpret_cast<k_timer_driver **>(userdata);
        auto &driver = *context[0];
        auto &timer = driver.timer_;

        uint32_t channel = timer.intr_stat;
        size_t i = 0;
        for (i = 0; i < 4; i++)
        {
            if (channel & 1)
            {
                auto &driver_ch = *context[i];
                if (driver_ch.interval_ > MAX_LOAD_COUNT_VALUE) {
                    // == long interval handling ==
                    if (driver_ch.remain_ == 0) {
                        // last 1-second interval ended
                        // start with 1st (partial second) interval
                        timer.channel[driver_ch.channel_].load_count = driver_ch.load_count_;
                        //timer.channel[driver_ch.channel_].current_value = driver_ch.load_count_;
                        // reload remaining full second intervals
                        driver_ch.remain_ = driver_ch.load_remain_;
                        // reset current value
                        driver_ch.current_value_ = 0;
                        driver_ch.running_time_ += driver_ch.load_count_1sec_;
                        if (driver_ch.on_tick_) {
                            // execute user 'on_tick' callback
                            driver_ch.on_tick_(driver_ch.ontick_data_);
                        }
                    }
                    else {
                        if (timer.channel[driver_ch.channel_].load_count != driver_ch.load_count_1sec_) {
                            // 1st (partial 1-second) interval ended
                            // next count for 1 second
                            timer.channel[driver_ch.channel_].load_count = driver_ch.load_count_1sec_;
                            driver_ch.current_value_ = driver_ch.load_count_;
                            driver_ch.running_time_ += driver_ch.load_count_;
                        }
                        else {
                            // 1-second interval ended
                            driver_ch.remain_--;
                            if (driver_ch.remain_) {
                                driver_ch.current_value_ += driver_ch.load_count_1sec_;
                                driver_ch.running_time_ += driver_ch.load_count_1sec_;
                            }
                            else {
                                // last 1-second interval begins
                                driver_ch.current_value_ += driver_ch.load_count_1sec_;
                                driver_ch.running_time_ += driver_ch.load_count_1sec_;
                            }
                        }
                    }
                }
                else {
                    // === normal interval handling ===
                    driver_ch.running_time_ += driver_ch.load_count_;
                    if (driver_ch.on_tick_) {
                        // execute user 'on_tick' callback
                        driver_ch.on_tick_(driver_ch.ontick_data_);
                    }
                }
            }

            channel >>= 1;
        }

        readl(&timer.eoi);
    }

private:
    volatile kendryte_timer_t &timer_;
    sysctl_clock_t clock_;
    plic_irq_t irq_;
    size_t num_;
    size_t channel_;
    size_t interval_;

    size_t remain_;
    size_t load_remain_;
    size_t current_value_;
    size_t running_time_;
    uint32_t load_count_;
    uint32_t load_count_1sec_;
    double resolution_;

    timer_on_tick_t on_tick_;
    void *ontick_data_;
};

/* clang-format off */
#define DEFINE_TIMER_DATA(i) \
{ TIMER##i##_BASE_ADDR, SYSCTL_CLOCK_TIMER##i, IRQN_TIMER##i##A_INTERRUPT, i, 0 },  \
{ TIMER##i##_BASE_ADDR, SYSCTL_CLOCK_TIMER##i, IRQN_TIMER##i##A_INTERRUPT, i, 1 },  \
{ TIMER##i##_BASE_ADDR, SYSCTL_CLOCK_TIMER##i, IRQN_TIMER##i##A_INTERRUPT, i, 2 },  \
{ TIMER##i##_BASE_ADDR, SYSCTL_CLOCK_TIMER##i, IRQN_TIMER##i##A_INTERRUPT, i, 3 }
/* clang format on */

#define INIT_TIMER_DRIVER(i) { { &dev_driver[i], timer_install, timer_open, timer_close }, timer_set_interval, timer_set_on_tick, timer_set_enable, timer_get_value }

static k_timer_driver dev_driver[12] =
{
    DEFINE_TIMER_DATA(0),
    DEFINE_TIMER_DATA(1),
    DEFINE_TIMER_DATA(2)
};

driver &g_timer_driver_timer0 = dev_driver[0];
driver &g_timer_driver_timer1 = dev_driver[1];
driver &g_timer_driver_timer2 = dev_driver[2];
driver &g_timer_driver_timer3 = dev_driver[3];
driver &g_timer_driver_timer4 = dev_driver[4];
driver &g_timer_driver_timer5 = dev_driver[5];
driver &g_timer_driver_timer6 = dev_driver[6];
driver &g_timer_driver_timer7 = dev_driver[7];
driver &g_timer_driver_timer8 = dev_driver[8];
driver &g_timer_driver_timer9 = dev_driver[9];
driver &g_timer_driver_timer10 = dev_driver[10];
driver &g_timer_driver_timer11 = dev_driver[11];
