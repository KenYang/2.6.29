/*
 *  linux/include/asm-arm/arch-socle/serial.h
 *
 *  Copyright (C) 2006 Socle-tech Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H

#include <mach/platform.h>
/*
 * This assumes you have a 14.7456 MHz clock UART.
 */
#define BASE_BAUD 115200

#define RS_TABLE_SIZE	2
     /* UART CLK        PORT  IRQ     FLAGS        */
#define STD_SERIAL_PORT_DEFNS \
	{ 0, BASE_BAUD, IO_ADDRESS(LDK_UART0_BASE), 1, ASYNC_SKIP_TEST|ASYNC_BOOT_AUTOCONF  },	/* ttyS0 */	\
	{ 0, BASE_BAUD, IO_ADDRESS(LDK_UART1_BASE), 2, ASYNC_SKIP_TEST|ASYNC_BOOT_AUTOCONF  },	/* ttyS1 */

#define EXTRA_SERIAL_PORT_DEFNS

#endif
