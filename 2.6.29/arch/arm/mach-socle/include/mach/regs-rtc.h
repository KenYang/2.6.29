/* linux/include/asm/arch-socle/regs-timer.h
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

#ifndef __ASM_ARCH_SOCLE_RTC_H
#define __ASM_ARCH_SOCLE_RTC_H	1

#include <mach/platform.h>

#define SOCLE_VA_RTC 			rtc_regs
#define SOCLE_RTC_TIME			(SOCLE_VA_RTC+0x0)
#define SOCLE_RTC_DATE			(SOCLE_VA_RTC+0x4)
#define SOCLE_RTC_TALRM		(SOCLE_VA_RTC+0x8)
#define SOCLE_RTC_DALRM		(SOCLE_VA_RTC+0xC)
#define SOCLE_RTC_CTRL			(SOCLE_VA_RTC+0x10)


#define SOCLE_RTC_RESET    		(SOCLE_VA_RTC + 0x0014)
#define SOCLE_RTC_PWOFF    		(SOCLE_VA_RTC + 0x0018)
#define SOCLE_RTC_RTCPWFAIL		(SOCLE_VA_RTC + 0x001C)


#if defined(CONFIG_ARCH_PDK_PC9223) 
#define RTC_CLK_FREQ		(32768/2)
#else
#define RTC_CLK_FREQ		(32768)
#endif
#define RTC_DIVIDER			((RTC_CLK_FREQ /16)-1)

#define RTC_SET_DIVIDER(x)       (writel((readl(SOCLE_RTC_CTRL) & 0xf8000000) | (x), SOCLE_RTC_CTRL))

#define RTC_CTRL_SET_FLAG(x)     (writel(readl(SOCLE_RTC_CTRL) |  (x), SOCLE_RTC_CTRL))
#define RTC_CTRL_CLR_FLAG(x)     (writew(readw(SOCLE_RTC_CTRL) & ~(x), SOCLE_RTC_CTRL))

#define RTC_EN()                 RTC_CTRL_SET_FLAG( SOCLE_RTC_CTRL_EN )
#define RTC_DIS()                RTC_CTRL_CLR_FLAG( SOCLE_RTC_CTRL_EN )

#define RTC_IS_GOOD		(readl(SOCLE_RTC_PWOFF) & SOCLE_RTC_PWOFF_INDT)
#define RTC_IS_PWFAIL		(readl(SOCLE_RTC_RTCPWFAIL) & SOCLE_RTC_RTCPWFAIL_RTCPWFAIL)

#define RESET_RTC_CIRCUIT()	writel(readl(SOCLE_RTC_RESET) | SOCLE_RTC_RESET_CRESET, SOCLE_RTC_RESET)
#define RESET_RTC_COUNTER()	writel(readl(SOCLE_RTC_RESET) | SOCLE_RTC_RESET_RTCRESET, SOCLE_RTC_RESET)

#define EN_RTC_ALARM_POWER_ON()		writel(readl(SOCLE_RTC_CTRL) | SOCLE_RTC_CTRL_ALRM_PWON, SOCLE_RTC_CTRL)
#define ENTER_RTC_POWER_OFF_MODE()													\
		do {																		\
			writel(readl(SOCLE_RTC_CTRL) | SOCLE_RTC_CTRL_INT_OUT_EN, SOCLE_RTC_CTRL);	\
			writel(readl(SOCLE_RTC_PWOFF) | SOCLE_RTC_PWOFF_PWOFF, SOCLE_RTC_PWOFF);		\
		} while (0)


//SOCLE_RTC_TIME		0x00
#define SOCLE_RTC_TIME_SOS           	0
#define SOCLE_RTC_TIME_S             	4
#define SOCLE_RTC_TIME_TS            	8
#define SOCLE_RTC_TIME_M            	11
#define SOCLE_RTC_TIME_TM           	15
#define SOCLE_RTC_TIME_H            	18
#define SOCLE_RTC_TIME_TH           	22
#define SOCLE_RTC_TIME_DOW          	24

//SOCLE_RTC_DATE		0x04
#define SOCLE_RTC_DATE_D             	0
#define SOCLE_RTC_DATE_TD            	4
#define SOCLE_RTC_DATE_M             	6
#define SOCLE_RTC_DATE_TM           	10
#define SOCLE_RTC_DATE_Y            	11
#define SOCLE_RTC_DATE_TY           	15
#define SOCLE_RTC_DATE_C            	19
#define SOCLE_RTC_DATE_TC           	23

//SOCLE_RTC_TALRM		0x08
#define SOCLE_RTC_TALRM_CDOW		(1 << 31)
#define SOCLE_RTC_TALRM_CH			(1 << 30)
#define SOCLE_RTC_TALRM_CM			(1 << 29)
#define SOCLE_RTC_TALRM_CS			(1 << 28)
#define SOCLE_RTC_TALRM_CSOS			(1 << 27)

//SOCLE_RTC_DALRM		0x0C
#define SOCLE_RTC_DALRM_CC			(1 << 30)
#define SOCLE_RTC_DALRM_CY			(1 << 29)
#define SOCLE_RTC_DALRM_CM			(1 << 28)
#define SOCLE_RTC_DALRM_CD			(1 << 27)

//SOCLE_RTC_CTRL 		0x10
#define SOCLE_RTC_CTRL_DIV        			(1<<0)
#define SOCLE_RTC_CTRL_SOS	       		(1<<27)
#define SOCLE_RTC_CTRL_INT_OUT_EN		(1<<28)
#define SOCLE_RTC_CTRL_ALRM_PWON 		(1<<29)
#define SOCLE_RTC_CTRL_ALRM          		(1<<30)
#define SOCLE_RTC_CTRL_EN            			(1<<31)

//SOCLE_RTC_RESET		0x14
#define SOCLE_RTC_RESET_CRESET			(1<<0)
#define SOCLE_RTC_RESET_RTCRESET		(1<<1)

//SOCLE_RTC_PWOFF		0x18
#define SOCLE_RTC_PWOFF_INDT			(1<<0)
#define SOCLE_RTC_PWOFF_PWOFF			(1<<0)	// cyli fix

//SOCLE_RTC_PWFAIL	0x1c
#define SOCLE_RTC_RTCPWFAIL_RTCPWFAIL		(1<<0)


#define FEBRUARY			2
#define STARTOFTIME			1970
#define SECDAY				86400L
#define SECYR				(SECDAY * 365)
#define leapyear(year)		((year) % 4 == 0)
#define days_in_year(a)		(leapyear(a) ? 366 : 365)
#define days_in_month(a)		(month_days[(a) - 1])


//Socle RTC Ioctl
#define RTC_PWR_OFF		_IO('p', 0x23)

#endif /*  __ASM_ARCH_SOCLE_RTC_H */

