
#ifndef INCLUDED_MPHALPORT_H
#define INCLUDED_MPHALPORT_H

#include "py/ringbuf.h"
#include "lib/utils/interrupt_char.h"


void mp_hal_uarths_setirqhandle(void *irq_handler);
void mp_hal_uarths_setirq_default();
int mp_hal_stdin_rx_chr(void);
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len);
void mp_hal_debug_tx_strn_cooked(void *env, const char *str, size_t len);
mp_uint_t mp_hal_ticks_cpu(void);
mp_uint_t mp_hal_ticks_us(void);
mp_uint_t mp_hal_ticks_ms(void);
void mp_hal_delay_us(mp_uint_t us);
void mp_hal_delay_ms(mp_uint_t ms);
void mp_hal_init(void);

#endif

