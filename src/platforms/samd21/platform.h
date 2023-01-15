/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
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

/* This file provides the platform specific declarations for the SAMD21 implementation. */

#ifndef PLATFORMS_SAMD21_PLATFORM_H
#define PLATFORMS_SAMD21_PLATFORM_H

#include <libopencm3/sam/d/gpio.h>
#include <libopencm3/sam/d/nvic.h>
#include <timing.h>
#include "gpio.h"

void spi_debug(uint8_t n);

#define PLATFORM_HAS_USBUART
// #define PLATFORM_HAS_TRACESWO // Not implemented yet, but possible in the future.
#define TMP_AUX_DISABLE
#define PLATFORM_NO_JTAG

#define PLATFORM_CLOCK_FREQ 48000000

#define PLATFORM_IDENT " (SAMD21)"

// Currently, SAMD21 is SWD ONLY. JTAG support can be added at a future date.

#define TMS_PORT PORTA
#define TCK_PORT PORTA
#define TMS_PIN  GPIO7
#define TCK_PIN  GPIO16

#define SWDIO_PORT TMS_PORT
#define SWCLK_PORT TCK_PORT
#define SWDIO_PIN  TMS_PIN
#define SWCLK_PIN  TCK_PIN

// Also, no TRST pin at the moment.
// #define TRST_PORT GPIOC
// #define TRST_PIN  GPIO5

#define LED_PORT       PORTA
#define LED_PORT_UART  PORTA
#define LED_UART       GPIO17
#define LED_IDLE_RUN   GPIO17
#define LED_ERROR      GPIO6
#define LED_BOOTLOADER GPIO17

//#define TMS_SET_MODE()     gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_CNF_FLOAT, TMS_PIN);
#define SWDIO_MODE_FLOAT() gpio_set_input(SWDIO_PORT, SWDIO_PIN);

#define SWDIO_MODE_DRIVE() gpio_set_output(SWDIO_PORT, SWDIO_PIN);
#define UART_PIN_SETUP()                                                                             \
	do {                                                                                               \
		gpio_mode_setup(USBUSART_PORT, GPIO_MODE_OUTPUT, GPIO_CNF_AF, USBUSART_TX_PIN);                  \
		gpio_set_af(USBUSART_PORT, PORT_PMUX_FUN_C, USBUSART_TX_PIN);                                    \
		gpio_mode_setup(USBUSART_PORT, GPIO_MODE_INPUT, GPIO_CNF_AF | GPIO_CNF_PULLUP, USBUSART_RX_PIN); \
		gpio_set_af(USBUSART_PORT, PORT_PMUX_FUN_C, USBUSART_RX_PIN);                                    \
	} while (0)

#define USB_DRIVER samd21_usb_driver
#define USB_IRQ    NVIC_USB_IRQ
#define USB_ISR(x) usb_isr(x)

/*
 * Interrupt priorities. Low numbers are high priority.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB          (1U << 4U)
#define IRQ_PRI_USBUSART     (2U << 4U)
#define IRQ_PRI_USBUSART_DMA (2U << 4U)
#define IRQ_PRI_TRACE        (0U << 4U)

#define USBUSART        SERCOM1_BASE
#define USBUSART_PM     PM_SERCOM1
#define USBUSART_PORT   PORTA
#define USBUSART_TX_PIN GPIO18
#define USBUSART_RX_PIN GPIO19
#define USBUSART_RXPO   USART_TX_PAD2
#define USBUSART_TXPO   USART_RX_PAD3
#define USBUSART_ISR(x) sercom1_isr(x)

#define USBUSART_DMA_BUS       DMA2
#define USBUSART_DMA_CLK       RCC_DMA2
#define USBUSART_DMA_TX_CHAN   DMA_STREAM7
#define USBUSART_DMA_TX_IRQ    NVIC_DMA2_STREAM7_IRQ
#define USBUSART_DMA_TX_ISR(x) dma2_stream7_isr(x)
#define USBUSART_DMA_RX_CHAN   DMA_STREAM5
#define USBUSART_DMA_RX_IRQ    NVIC_DMA2_STREAM5_IRQ
#define USBUSART_DMA_RX_ISR(x) dma2_stream5_isr(x)
/* For STM32F4 DMA trigger source must be specified */
#define USBUSART_DMA_TRG DMA_SxCR_CHSEL_4

#define AUX_UART_BUFFER_SIZE 64U

#define TRACE_TIM          TIM3
#define TRACE_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM3)
#define TRACE_IRQ          NVIC_TIM3_IRQ
#define TRACE_ISR(x)       tim3_isr(x)

extern uint32_t swd_delay_cnt;
extern uint8_t running_status;
#define SET_RUN_STATE(state)      \
	{                             \
		running_status = (state); \
	}
#define SET_IDLE_STATE(state)                        \
	{                                                \
		gpio_set_val(LED_PORT, LED_IDLE_RUN, state); \
	}
#define SET_ERROR_STATE(state)                    \
	{                                             \
		gpio_set_val(LED_PORT, LED_ERROR, state); \
	}

static inline int platform_hwversion(void)
{
	return 0;
}

/* Use newlib provided integer-only stdio functions */

#ifdef sscanf
#undef sscanf
#endif
#define sscanf siscanf

#ifdef sprintf
#undef sprintf
#endif
#define sprintf siprintf

#ifdef vasprintf
#undef vasprintf
#endif
#define vasprintf vasiprintf

#ifdef snprintf
#undef snprintf
#endif
#define snprintf sniprintf

#endif /* PLATFORMS_SAMD_PLATFORM_H */
