/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by -----
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the native implementation. */

#include "general.h"
#include "usb.h"
#include "aux_serial.h"
#include "morse.h"
#include "platform.h"

//#include <libopencm3/sam/d/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
//#include <libopencm3/sam/d/exti.h>
//#include <libopencm3/sam/d/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/sam/d/gpio.h>
#include <libopencm3/sam/d/gclk.h>
#include <libopencm3/sam/d/nvmctrl.h>
//#include <libopencm3/sam/d/flash.h>

/* This is defined by the linker script */
extern char vector_table;

uint32_t swd_delay_cnt = 0;
uint8_t running_status;

void spi_debug(uint8_t n)
{
	for (int i = 0; i < 8; i++) {
		if (n & 1) {
			gpio_set(PORTA, GPIO19);
		} else {
			gpio_clear(PORTA, GPIO19);
		}
		gpio_toggle(PORTA, GPIO20);
		gpio_toggle(PORTA, GPIO20);
		n >>= 1;
	}
}

void platform_timing_init(void);

void platform_request_boot(void)
{
}

void platform_init(void)
{
	sysctrl_enable_dfll48m();
	nvmctrl_enable_rws();
	gclk_enable_gen(GCLK_GEN_ID0, GCLK_GEN_SRC_DFLL48M, GCLK_GEN_DIVSEL_DIRECT, 0);

	platform_timing_init();

	gpio_set_output(PORTA, GPIO6 | GPIO17 | GPIO19 | GPIO20);
	gpio_clear(PORTA, GPIO6 | GPIO17 | GPIO20);

	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_FLOAT, SWDIO_PIN);

	blackmagic_usb_init();
}

void platform_nrst_set_val(bool assert)
{
	(void)assert;
	// NRST not implemented yet...
}

bool platform_nrst_get_val(void)
{
	// NRST not implemented yet...
	return 0;
}

bool platform_target_get_power(void)
{
	// Target power not implemented.
	return 0;
}

void platform_target_set_power(const bool power)
{
	(void)power;
	// Target power not implemented.
}

uint32_t platform_target_voltage_sense(void)
{
	// Target voltage sense not implemented.
	return 0;
}

const char *platform_target_voltage(void)
{
	// Target voltage not implemented.
	return "N/A";
}

// MARK: IDK what this is
// void platform_request_boot(void)
// {
// 	/* Disconnect USB cable */
// 	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, 0, USB_PU_PIN);
//
// 	/* Drive boot request pin */
// 	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
// 	gpio_clear(GPIOB, GPIO12);
// }

void platform_target_clk_output_enable(bool enable)
{
	if (enable)
		gpio_set_output(TCK_PORT, TCK_PIN);
	else
		gpio_set_input(TCK_PORT, TCK_PIN);
}

// No clue what this is for
// void exti15_10_isr(void)
// {
// 	uint32_t usb_vbus_port;
// 	uint16_t usb_vbus_pin;
//
// 	if (platform_hwversion() < 5) {
// 		usb_vbus_port = USB_VBUS_PORT;
// 		usb_vbus_pin = USB_VBUS_PIN;
// 	} else {
// 		usb_vbus_port = USB_VBUS5_PORT;
// 		usb_vbus_pin = USB_VBUS5_PIN;
// 	}
//
// 	if (gpio_get(usb_vbus_port, usb_vbus_pin))
// 		/* Drive pull-up high if VBUS connected */
// 		gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
// 	else
// 		/* Allow pull-up to float if VBUS disconnected */
// 		gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);
//
// 	exti_reset_request(usb_vbus_pin);
// }

// Also no clue what this is for
// static void setup_vbus_irq(void)
// {
// 	uint32_t usb_vbus_port;
// 	uint16_t usb_vbus_pin;
//
// 	if (platform_hwversion() < 5) {
// 		usb_vbus_port = USB_VBUS_PORT;
// 		usb_vbus_pin = USB_VBUS_PIN;
// 	} else {
// 		usb_vbus_port = USB_VBUS5_PORT;
// 		usb_vbus_pin = USB_VBUS5_PIN;
// 	}
//
// 	nvic_set_priority(USB_VBUS_IRQ, IRQ_PRI_USB_VBUS);
// 	nvic_enable_irq(USB_VBUS_IRQ);
//
// 	gpio_set(usb_vbus_port, usb_vbus_pin);
// 	gpio_set(USB_PU_PORT, USB_PU_PIN);
//
// 	gpio_set_mode(usb_vbus_port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, usb_vbus_pin);
//
// 	/* Configure EXTI for USB VBUS monitor */
// 	exti_select_source(usb_vbus_pin, usb_vbus_port);
// 	exti_set_trigger(usb_vbus_pin, EXTI_TRIGGER_BOTH);
// 	exti_enable_request(usb_vbus_pin);
//
// 	exti15_10_isr();
// }
