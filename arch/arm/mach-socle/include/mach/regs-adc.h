/* linux/include/asm/arch-socle/cdk-adc.h
 *
 * Copyright (c) 2006 Socle-tech Corp
 *		      http://www.socle-tech.com.tw/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * CDK ADC configuration
*/

#ifndef __SOCLE_ADCREG_H
#define __SOCLE_ADCREG_H

#include <mach/platform.h>

#if 1
#define ADC_DATA		0x00
#define ADC_STATUS		0x04
#define ADC_CTRL		0x08

#define ADC_TOUCH_SWITCH_XM_ENABLE	(0x0<<10)
#define ADC_TOUCH_SWITCH_YM_ENABLE	(0x0<<9)
#define ADC_TOUCH_SWITCH_XP_ENABLE	(0x0<<8)
#define ADC_TOUCH_SWITCH_YP_ENABLE	(0x0<<7)
#define ADC_TOUCH_SWITCH_XM_DISABLE	(0x1<<10)
#define ADC_TOUCH_SWITCH_YM_DISABLE	(0x1<<9)
#define ADC_TOUCH_SWITCH_XP_DISABLE	(0x1<<8)
#define ADC_TOUCH_SWITCH_YP_DISABLE	(0x1<<7)
#define ADC_TOUCH_SWITCH_MASK	(0xf<<7)
#define ADC_INT_STATUS			(0x1<<6)
#define ADC_INT_ENABLE			(0x1<<5)
#define ADC_INT_DISABLE		(0x0<<5)
#define ADC_INT_MASK			(0x1<<5)
#define ADC_START_CONVERSION	(0x1<<4)
#define ADC_POWER_UP			(0x1<<3)

#define ADC_SOURCE_0			0
#define ADC_SOURCE_1			1
#define ADC_SOURCE_2			2
#define ADC_SOURCE_3			3
#define ADC_SOURCE_4			4
#define ADC_SOURCE_5			5
#define ADC_SOURCE_6			6
#define ADC_SOURCE_7			7
#define ADC_SOURCE_MASK		0x7
#else
// offset of regisgers

#define ADC_REG_BASE 			regs
#define SOCLE_ADC_DATA	        	(ADC_REG_BASE + 0x00)
#define SOCLE_ADC_STAS	       	(ADC_REG_BASE + 0x08)
#define SOCLE_ADC_CTRL	       	(ADC_REG_BASE + 0x04)


//SOCLE_ADC_CTRL
#define ADC_CTRL_INTR_CLR		(1<6)
#define ADC_CTRL_INTR_EN		(1<5)
#define ADC_CTRL_SOC			(1<4)
#define ADC_CTRL_PWM			(1<3)


#define ADC_GET_INPUT_CHANNEL			(0x7) & readl(SOCLE_ADC_CTRL)
#define ADC_SET_INPUT_CHANNEL(x)		x | ((~0x7) & readl(SOCLE_ADC_CTRL))

/*
* ioctl commands
*/
/* soft reset, master enable, and run bit is ignored, driver handles it itself. */
#define ADC_IOC_SET_PWM		_IO('s', 0)
#define ADC_IOC_GET_PWM		_IO('s', 1)
#define ADC_IOC_SET_CHANNEL	_IO('s', 2)
#define ADC_IOC_GET_CHANNEL	_IO('s', 3)
#define ADC_IOC_CONTINOUS		_IO('s', 4)
#endif
#endif

