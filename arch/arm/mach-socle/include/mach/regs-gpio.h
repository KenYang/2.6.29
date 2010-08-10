/* linux/include/asm/arch-socle/socle-gpio.h
 *
 * Copyright (c) 2006 Socle-tech Corp
 *		      http://www.socle-tech.com.tw/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __SOCLE_GPIOREG_H
#define __SOCLE_GPIOREG_H                     1

#include <mach/platform.h>
#include <mach/irqs.h>

/* Set the IP's base address */
#ifndef SOCLE_GPIO0
	#error "GPIO IP base address is not defined"
#endif

#ifndef SOCLE_GPIO1
	#define SOCLE_GPIO1 SOCLE_GPIO0
	#define SOCLE_GPIO_GP1	0
#else
	#define SOCLE_GPIO_GP1	1
#endif

#ifndef SOCLE_GPIO2
	#define SOCLE_GPIO2 SOCLE_GPIO0
	#define SOCLE_GPIO_GP2	0
#else
	#define SOCLE_GPIO_GP2	1
#endif

#ifndef SOCLE_GPIO3
	#define SOCLE_GPIO3 SOCLE_GPIO0
	#define SOCLE_GPIO_GP3	0
#else
	#define SOCLE_GPIO_GP3	1
#endif

/* Set the IP's irq */
#ifndef IRQ_GPIO0
	#ifdef IRQ_GPIO
		#define IRQ_GPIO0	IRQ_GPIO
		#define SOCLE_GPIO_WITH_INT
	#endif
#else
	#define SOCLE_GPIO_WITH_INT
#endif


#ifndef IRQ_GPIO1
	#define IRQ_GPIO1		IRQ_GPIO0
#endif

#ifndef IRQ_GPIO2
	#define IRQ_GPIO2		IRQ_GPIO0
#endif

#ifndef IRQ_GPIO3
	#define IRQ_GPIO3		IRQ_GPIO0
#endif


#define GPIO_PER_PORT_PIN_NUM	0x8
#define GPIO_PORT_NUM			0x4

//Register
#define GPIO_PADR			0X0000
#define GPIO_PACON			0X0004
#define GPIO_PBDR			0X0008
#define GPIO_PBCON			0X000c
#define GPIO_PCDR			0X0010
#define GPIO_PCCON			0X0014
#define GPIO_PDDR			0X0018
#define GPIO_PDCON			0X001c
#define GPIO_TEST			0X0020

#define GPIO_IEA				0X0024
#define GPIO_IEB				0X0028
#define GPIO_IEC				0X002c
#define GPIO_IED				0X0030

#define GPIO_ISA				0X0034
#define GPIO_ISB				0X0038
#define GPIO_ISC				0X003c
#define GPIO_ISD				0X0040

#define GPIO_IBEA			0X0044
#define GPIO_IBEB			0X0048
#define GPIO_IBEC			0X004c
#define GPIO_IBED			0X0050

#define GPIO_IEVA			0X0054
#define GPIO_IEVB			0X0058
#define GPIO_IEVC			0X005c
#define GPIO_IEVD			0X0060

#define GPIO_ICA				0X0064
#define GPIO_ICB				0X0068
#define GPIO_ICC				0X006c
#define GPIO_ICD				0X0070

#define GPIO_ISR				0X0074


#endif

