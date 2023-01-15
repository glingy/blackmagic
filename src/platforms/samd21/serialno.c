/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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
#include "general.h"
#include "serialno.h"

#include <libopencm3/sam/d/serno.h>

char serial_no[DFU_SERIAL_LENGTH];

void read_serial_number(void)
{
#if DFU_SERIAL_LENGTH == 9
	serial_no[0] = 'P';
	serial_no[1] = 'L';
	serial_no[2] = 'A';
	serial_no[3] = 'Y';
	serial_no[4] = 'G';
	serial_no[5] = 'R';
	serial_no[6] = 'N';
	serial_no[7] = 'D';
#else
#warning "Unhandled DFU_SERIAL_LENGTH"
#endif
	serial_no[DFU_SERIAL_LENGTH - 1] = '\0';
}
