/*
 * linux/include/asm-arm/arch-socle/regs-irq.h
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
#ifndef __ASSEMBLY__
#include <asm/io.h>
#endif
//============== INTERRUPT========================================
#include <mach/platform.h>

#ifdef SOCLE_INTC0
#define INTC0_PHY_ADDR			SOCLE_INTC0
#else
#define INTC0_PHY_ADDR			SOCLE_VIC0
#endif


#define INTC0_VIR_ARRD		 IO_ADDRESS(INTC0_PHY_ADDR)


//#define INTERRUPTCTRL_BASE VA_VIC_BASE
#define INTC0_SCR(x) 		(INTC0_VIR_ARRD + 4 * (x))
#define INTC0_ISR    			(INTC0_VIR_ARRD + 0x104)
#define INTC0_IPR    			(INTC0_VIR_ARRD + 0x108)
#define INTC0_IMR    			(INTC0_VIR_ARRD + 0x10C)
#define INTC0_IECR   			(INTC0_VIR_ARRD + 0x114)
#define INTC0_ICCR   			(INTC0_VIR_ARRD + 0x118)
#define INTC0_ISCR   			(INTC0_VIR_ARRD + 0x11C)
#define INTC0_TEST   			(INTC0_VIR_ARRD + 0x124)


#define LO_LEVEL                 	0x00000000
#define HI_LEVEL                 	0x00000040
#define NEGATIVE_EDGE    	0x00000080
#define POSITIVE_EDGE    		0x000000C0
#define SRCTYPE_MASK     		0x000000C0
// define Macro Lib
#define INT0_SET_TYPE(i,type)     __raw_writel((__raw_readl(INTC0_SCR(i)) & ~SRCTYPE_MASK) | type , INTC0_SCR(i))
#define INT0_GET_TYPE(i)              __raw_readl(INTC0_SCR(i)) & SRCTYPE_MASK
#define INT0_SET_PRIORITY(i,p)    __raw_writel((__raw_readl(INTC0_SCR(i)) & 0xfffffff8) | (p), INTC0_SCR(i))
#define INT0_GET_PRIORITY(i)          __raw_readl(INTC0_SCR(i)) & BITMASK(3)

#define INT0_ENABLE(i)                __raw_writel(__raw_readl(INTC0_IECR) |  (0x1 << i), INTC0_IECR)
#define INT0_DISABLE(i)               __raw_writel(__raw_readl(INTC0_IECR) & ~(0x1 << i), INTC0_IECR)
#define INT0_SET_MASK(i)              __raw_writel(__raw_readl(INTC0_IMR)  |  (0x1 << i), INTC0_IMR)
#define INT0_CLR_MASK(i)              __raw_writel(__raw_readl(INTC0_IMR)  & ~(0x1 << i), INTC0_IMR)

#define INT0_TRIGGER(i)               __raw_writel(__raw_readl(INTC0_ISCR)| (0x1 << i), INTC0_ISCR)
#define INT0_SWTRIGGER_CLEAR()    __raw_writel(0X0, INTC0_ISCR)
//#define INT0_CLEAR(i)                 __raw_writel(getInterruptMask(i),        INTC0_ICCR)


#define VA_INTC1_BASE		 IO_ADDRESS(SOCLE_INTC1)

#define INTC1_SCR(x) 		(VA_INTC1_BASE + 4 * (x))
#define INTC1_ISR    			(VA_INTC1_BASE + 0x104)
#define INTC1_IPR    			(VA_INTC1_BASE + 0x108)
#define INTC1_IMR    			(VA_INTC1_BASE + 0x10C)
#define INTC1_IECR   			(VA_INTC1_BASE + 0x114)
#define INTC1_ICCR   			(VA_INTC1_BASE + 0x118)
#define INTC1_ISCR   			(VA_INTC1_BASE + 0x11C)
#define INTC1_TEST   			(VA_INTC1_BASE + 0x124)

// define Macro Lib
#define INT1_SET_TYPE(i,type)     __raw_writel((__raw_readl(INTC1_SCR(i)) & ~SRCTYPE_MASK) | type , INTC1_SCR(i))
#define INT1_GET_TYPE(i)              __raw_readl(INTC1_SCR(i)) & SRCTYPE_MASK
#define INT1_SET_PRIORITY(i,p)    __raw_writel((__raw_readl(INTC1_SCR(i)) & 0xfffffff8) | (p), INTC1_SCR(i))
#define INT1_GET_PRIORITY(i)          __raw_readl(INTC1_SCR(i)) & BITMASK(3)

#define INT1_ENABLE(i)                __raw_writel(__raw_readl(INTC1_IECR) |  (0x1 << i), INTC1_IECR)
#define INT1_DISABLE(i)               __raw_writel(__raw_readl(INTC1_IECR) & ~(0x1 << i), INTC1_IECR)
#define INT1_SET_MASK(i)              __raw_writel(__raw_readl(INTC1_IMR)  |  (0x1 << i), INTC1_IMR)
#define INT1_CLR_MASK(i)              __raw_writel(__raw_readl(INTC1_IMR)  & ~(0x1 << i), INTC1_IMR)

#define INT1_TRIGGER(i)               __raw_writel(__raw_readl(INTC1_ISCR)| (0x1 << i), INTC1_ISCR)
#define INT1_SWTRIGGER_CLEAR()    __raw_writel(0X0, INTC1_ISCR)
//#define INT0_CLEAR(i)                 __raw_writel(getInterruptMask(i),        INTC0_ICCR)

