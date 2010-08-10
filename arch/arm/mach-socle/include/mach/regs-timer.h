/* linux/include/asm/arch-socle/socle-timer.h
 *
 * Copyright (c) 2006 Socle-tech Corp
 *		      http://www.socle-tech.com.tw/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * LDK Timer configuration
*/

#ifndef __ASM_ARCH_SOCLE_TIMER_H
#define __ASM_ARCH_SOCLE_TIMER_H	1

#include <mach/platform.h>


#define TIMER_BASS_ADDR			SOCLE_TIMER0

#define SOCLE_TIMER0_BASE          		TIMER_BASS_ADDR
#define SOCLE_TIMER1_BASE         			(TIMER_BASS_ADDR + 0x10)
#define SOCLE_TIMER2_BASE          		(TIMER_BASS_ADDR + 0x20)

#define TIMER0_VA_BASE IO_ADDRESS(SOCLE_TIMER0_BASE)
#define TIMER1_VA_BASE IO_ADDRESS(SOCLE_TIMER1_BASE)
#define TIMER2_VA_BASE IO_ADDRESS(SOCLE_TIMER2_BASE)

#endif /*  __ASM_ARCH_SOCLE_TIMER_H */


