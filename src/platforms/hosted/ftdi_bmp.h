/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2018  Uwe Bonnes (non@elektron.ikp.physik.tu-darmstadt.de)
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

#ifndef PLATFORMS_HOSTED_FTDI_BMP_H
#define PLATFORMS_HOSTED_FTDI_BMP_H

#include "cli.h"
#include "jtagtap.h"

#include "bmp_hosted.h"

typedef struct data_desc {
	int16_t data_low;
	int16_t ddr_low;
	int16_t data_high;
	int16_t ddr_high;
} data_desc_s;

typedef struct pin_settings {
	uint8_t set_data_low;
	uint8_t clr_data_low;
	uint8_t set_data_high;
	uint8_t clr_data_high;
} pin_settings_s;

typedef struct cable_desc {
	int vendor;
	int product;
	int interface;
	/* Initial (C|D)(Bus|Ddr) values for additional pins.
	 * MPSSE_CS|DI|DO|SK are initialized accordig to mode.*/
	data_desc_s init;
	/* MPSSE command to read TMS/SWDIO in bitbanging SWD.
	 * In many cases this is the TMS port, so then use "GET_PIN_LOW".*/
	uint8_t bb_swdio_in_port_cmd;
	/* bus bit to read TMS/SWDIO in bitbanging SWD.
	 * In many cases this is the TMS port, so then use "MPSSE_CS".*/
	uint8_t bb_swdio_in_pin;
	/* Bus data to allow bitbanging switched SWD read.
	 * TMS is routed to bb_swdio_in_port/pin.*/
	pin_settings_s bb_swd_read;
	/* Bus data to allow bitbanging switched SWD write.
	 * TMS is routed to MPSSE_CS.*/
	pin_settings_s bb_swd_write;
	/* dbus_data, dbus_ddr, cbus_data, cbus_ddr value to assert nRST.
	 *	E.g. with CBUS Pin 1 low,
	 *	give data_high = ~PIN1, ddr_high = PIN1 */
	data_desc_s assert_nrst;
	/*  Bus_data, dbus_ddr, cbus_data, cbus_ddr value to release nRST.
	 *	E.g. with CBUS Pin 1 floating with internal pull up,
	 *	give data_high = PIN1, ddr_high = ~PIN1 */
	data_desc_s deassert_nrst;
	/* Command to read back NRST. If 0, port from assert_nrst is used*/
	uint8_t nrst_get_port_cmd;
	/* PIN to read back as NRST. if 0 port from assert_nrst is ised.
	*  Use PINX if active high, use Complement (~PINX) if active low*/
	uint8_t nrst_get_pin;
	/* Bbus data for pure MPSSE SWD read.
	 * Use together with swd_write if by some bits on DBUS,
	 * SWDIO can be routed to TDI and TDO.
	 * If both mpsse_swd_read|write and
	 * bitbang_swd_dbus_read_data/bitbang_tms_in_port_cmd/bitbang_tms_in_pin
	 * are provided, pure MPSSE SWD is chosen.
	 * If neither a complete set of swd_read|write or
	 * bitbang_swd_dbus_read_data/bitbang_tms_in_port_cmd/bitbang_tms_in_pin
	 * are provided, SWD can not be done.
	 * swd_read.set_data_low ==  swd_write.set_data_low == MPSSE_DO
	 * indicated resistor SWD and inhibits Jtag.*/
	pin_settings_s mpsse_swd_read;
	/* dbus data for pure MPSSE SWD write.*/
	pin_settings_s mpsse_swd_write;
	/* dbus data for jtag.*/
	pin_settings_s jtag;
	/* Command to read port to check target voltage.*/
	uint8_t target_voltage_cmd;
	/* Pin to check target voltage.*/
	uint8_t target_voltage_pin;
	/* USB readable description of the device.*/
	char *description;
	/* Command line argument to -c option to select this device.*/
	char *name;
} cable_desc_s;

#define libftdi_buffer_write_arr(array) libftdi_buffer_write(array, sizeof(array))
#define libftdi_buffer_write_val(value) libftdi_buffer_write(&(value), sizeof(value))
#define libftdi_buffer_read_arr(array)  libftdi_buffer_read(array, sizeof(array))
#define libftdi_buffer_read_val(value)  libftdi_buffer_read(&(value), sizeof(value))

#if HOSTED_BMP_ONLY == 1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

int ftdi_bmp_init(bmda_cli_options_s *cl_opts, bmp_info_s *info)
{
	return -1;
}

int libftdi_swdptap_init(adiv5_debug_port_s *dp)
{
	return -1;
}

bool libftdi_jtagtap_init(void)
{
	return false;
}

void libftdi_buffer_flush(void)
{
}

size_t libftdi_buffer_write(const uint8_t *data, size_t size)
{
	return size;
}

size_t libftdi_buffer_read(uint8_t *data, size_t size)
{
	return size;
}

const char *libftdi_target_voltage(void)
{
	return "ERROR";
}

void libftdi_jtagtap_tdi_tdo_seq(
	uint8_t *const data_out, const bool final_tms, const uint8_t *const data_in, const size_t ticks)
{
}

bool libftdi_swd_possible(void)
{
	return false;
}

void libftdi_max_frequency_set(uint32_t freq)
{
}

uint32_t libftdi_max_frequency_get(void)
{
	return 0;
}

void libftdi_nrst_set_val(bool assert)
{
}

bool libftdi_nrst_get_val(void)
{
	return false;
}

#pragma GCC diagnostic pop
#else
#include <ftdi.h>
extern const cable_desc_s cable_desc[];
extern cable_desc_s active_cable;
extern ftdi_context_s *ftdic;
extern data_desc_s active_state;

int ftdi_bmp_init(bmda_cli_options_s *cl_opts, bmp_info_s *info);
int libftdi_swdptap_init(adiv5_debug_port_s *dp);
bool libftdi_jtagtap_init(void);
void libftdi_buffer_flush(void);
size_t libftdi_buffer_write(const uint8_t *data, size_t size);
size_t libftdi_buffer_read(uint8_t *data, size_t size);
const char *libftdi_target_voltage(void);
void libftdi_jtagtap_tdi_tdo_seq(uint8_t *data_out, bool final_tms, const uint8_t *data_in, size_t ticks);
bool libftdi_swd_possible(void);
void libftdi_max_frequency_set(uint32_t freq);
uint32_t libftdi_max_frequency_get(void);
void libftdi_nrst_set_val(bool assert);
bool libftdi_nrst_get_val(void);
#endif

#define MPSSE_SK 1
#define PIN0     1
#define MPSSE_DO 2
#define PIN1     2
#define MPSSE_DI 4
#define PIN2     4
#define MPSSE_CS 8
#define PIN3     8
#define PIN4     0x10
#define PIN5     0x20
#define PIN6     0x40
#define PIN7     0x80

#endif /* PLATFORMS_HOSTED_FTDI_BMP_H */
