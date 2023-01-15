#include "general.h"
#include "platform.h"
#include "morse.h"

#include <libopencm3/cm3/systick.h>

static volatile uint32_t time_ms = 0;

static size_t morse_tick = 0;

void sys_tick_handler(void)
{
	time_ms += SYSTICKMS;
	if (morse_tick >= MORSECNT) {
		SET_ERROR_STATE(morse_update());
		morse_tick = 0;
	} else
		++morse_tick;
}

uint32_t platform_time_ms(void)
{
	return time_ms;
}

uint32_t platform_max_frequency_get(void)
{
	return FREQ_FIXED;
}

void platform_max_frequency_set(uint32_t freq)
{
	(void)freq;
}

void platform_delay(uint32_t ms)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, ms);
	while (!platform_timeout_is_expired(&timeout))
		continue;
}

void platform_timing_init(void)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(PLATFORM_CLOCK_FREQ / SYSTICKHZ);
	systick_clear();

	systick_interrupt_enable();
	systick_counter_enable();
}