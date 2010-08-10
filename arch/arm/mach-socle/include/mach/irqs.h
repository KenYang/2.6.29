/*
 *  linux/include/asm-arm/arch-ldk/irqs.h
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

#if defined (CONFIG_ARCH_PDK_PC9223)

#define 	NR_IRQS			31

#define	IRQ_EXT1			30
#define	IRQ_EXT0			29
#define	IRQ_GPIO3			28
#define	IRQ_NAND0			27
#define	IRQ_GPIO2			26
#define	IRQ_CLCD0			25
#define	IRQ_GPIO1			24
#define	IRQ_GPIO0			23
#define	IRQ_HDMA0			22

#define	IRQ_VOP0          		17
#define	IRQ_MAC0     	   		15
#define	IRQ_ADC0        		14
#define	IRQ_RTC0			13
#define	IRQ_PWM1         		12
#define	IRQ_PWM0      		  	11
#define	IRQ_TIMER2			10
#define	IRQ_TIMER1     			9
#define	IRQ_TIMER0        		8
#define	IRQ_I2S0        	 	6
#define	IRQ_I2C0         	 	5
#define	IRQ_SPI1       	 		4
#define	IRQ_SPI0      	 		3
#define	IRQ_UART2    	   		2
#define	IRQ_UART1        		1
#define	IRQ_UART0        		0
#define 	IRQ_PANTHER7_HDMA0 IRQ_HDMA0
#define IRQ_OTG0			19
#define IRQ_OTG1			20
#define	IRQ_VIP0      			18
#define	IRQ_SDHC0			21

#define IRQ_UHC0                        IRQ_OTG0
#define IRQ_OTG_UDC0                    IRQ_OTG1

#endif

