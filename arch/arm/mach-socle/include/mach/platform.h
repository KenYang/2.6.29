/*
 * linux/include/asm-arm/arch-ldk/platform.h
 *
 * Copyright (c) 2006 Socle-tech Corp.  All rights reserved.
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

#ifndef __SOCLE_MEM_MAPIO_H
#define __SOCLE_MEM_MAPIO_H                     1

#include <mach/hardware.h>

/*
 * SOCLE System Memory Map
 */


#if defined(CONFIG_ARCH_PDK_PC9223)
// memory mapped address
#define	SOCLE_MEMORY_ADDR_START			0x40000000
#define	SOCLE_MEMORY_ADDR_SIZE			0x04000000

#define SOCLE_MM_DDR_SDR_BANK0			0x00000000
#define SOCLE_MM_DDR_SDR_BANK1			0x40000000

#define SOCLE_NOR_FLASH0       		0x10000000
#define SOCLE_NOR_FLASH_SIZE            SZ_4M

// APB device base address define

#define SOCLE_ADC0		0x19120000
#define SOCLE_SCU0		0x19100000
#define SOCLE_WDT0		0x190F0000
#define SOCLE_RTC0		0x190E0000
#define SOCLE_GPIO3		0x190D0000
#define SOCLE_GPIO2		0x190C0000
#define SOCLE_GPIO1		0x190B0000
#define SOCLE_GPIO0		0x190A0000
#define SOCLE_TIMER_PWM0	0x19090000
#define SOCLE_TIMER0		0x19080000
#define SOCLE_I2S0		0x19060000
#define SOCLE_I2C0		0x19050000
#define SOCLE_SPI1		0x19040000
#define SOCLE_SPI0		0x19030000
#define SOCLE_UART2		0x19020000
#define SOCLE_UART1		0x19010000
#define SOCLE_UART0		0x19000000

// AHB device base address define

#define SOCLE_BRI0		0x19000000	
#define SOCLE_ARBITER1		0x18160000
#define SOCLE_NAND0		0x18140000	
#define SOCLE_VOP0		0x18100000
#define PANTHER7_HDMA0		0x180C0000
#define SOCLE_HDMA0		0x180C0000

#define SOCLE_MAC0		0x18060000
#define SOCLE_INTC0		0x18040000
#define SOCLE_ARBITER0		0x18020000
#define SOCLE_SDRSTMC0		0x18000000	

#define SOCLE_LCD0		0x18180000
#define SOCLE_OTG0		0x18080000
#define SOCLE_OTG1		0x180E0000
#define SOCLE_SDHC0		0x180A0000
#define SOCLE_VIP0		0x18120000

#define SOCLE_UHC0          SOCLE_OTG0
#define SOCLE_OTG_UDC0          SOCLE_OTG1

#endif /* 	END */
#endif
