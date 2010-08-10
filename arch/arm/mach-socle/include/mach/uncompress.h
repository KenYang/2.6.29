/*
 *  linux/include/asm-arm/arch-socle/uncompress.h
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

#include <mach/platform.h>

#define AMBA_UART_THR	(*(volatile unsigned char *)SOCLE_UART0)
#define AMBA_UART_LSR	(*(volatile unsigned char *)(SOCLE_UART0 + 0x14))

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	while (!(AMBA_UART_LSR & (1 << 5)))
		barrier();

	AMBA_UART_THR = c;
}

static inline void flush(void)
{
	while (AMBA_UART_LSR & (1 << 3))
		barrier();
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()