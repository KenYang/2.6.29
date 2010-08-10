/* linux/include/asm/arch-socle/regs-wdt.h
 *
 * Copyright (c) 2006 Socle-tech Corp
 *		      http://www.socle-tech.com.tw/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#ifndef __ASM_ARCH_SOCLE_WDT_H
#define __ASM_ARCH_SOCLE_WDT_H	1

#define SOCLE_WDT_BASE			IO_ADDRESS(SOCLE_WDT0)

#define SOCLE_WTDOG_MAX_LR					(0xffffffff) /* 32-bit load register */
#define MAX_PRESCALE_SHIFT						(8)	/* Max. prescale 1/256 */
#define SOCLE_WTDOG_DISALLOWED_PRESCALE		(1)

#define SOCLE_WTDOG_LR			(SOCLE_WDT_BASE+0x0)
#define SOCLE_WTDOG_CVR		(SOCLE_WDT_BASE+0x4)
#define SOCLE_WTDOG_CON		(SOCLE_WDT_BASE+0x8)

/*
 * uPlatform WATCHDOG CON register definition
 */
#define WTDOG_ENABLE			(1<<3)
#define WTDOG_RESET_ENABLE	(1<<4)

#endif
