/*
 *  linux/arch/arm/mach-socle/irq.c
 *
 * Copyright (C) Socle Tech. Corp.
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option) any later version. 
 * This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/regs-intr.h>

#include "generic.h"

extern void socle_init_pcie_irq(void);

#define VIC_IRQS	32

#if defined(CONFIG_ARCH_MDK_3D) || defined(CONFIG_ARCH_MDK_FHD) // todo: When arch/arm/common/vic.c support PL192 officially, and use them instead
#include <asm/hardware/vic.h>

static void vic_mask_irq(unsigned int irq)
{
	void __iomem *base = get_irq_chip_data(irq);
	irq &= 31;
	writel(1 << irq, base + VIC_INT_ENABLE_CLEAR);
}

static void vic_unmask_irq(unsigned int irq)
{
	void __iomem *base = get_irq_chip_data(irq);
	irq &= 31;
	writel(1 << irq, base + VIC_INT_ENABLE);
}

#if 0
static void vic_ack_irq(unsigned int irq)
{
	if(irq < VIC_IRQS)
		writel(64 , IO_ADDRESS(SOCLE_VIC0 +VIC_PL192_VECT_ADDR));
	else
	{
		writel(64 , IO_ADDRESS(SOCLE_VIC1 +VIC_PL192_VECT_ADDR));
		//	writel(1 , IO_ADDRESS(SOCLE_VIC0 +VIC_PL192_VECT_ADDR));
	}
}
#endif

// cyli add for PM
static int vic_set_wake_irq(unsigned int irq, unsigned int on)
{
	// Setup system wake irq
	return 0;
}

static struct irq_chip vic_chip = {
	.name	= "VIC",
//	.ack	= vic_ack_irq,
	.ack	= vic_mask_irq,
	.disable= vic_mask_irq,
	.mask	= vic_mask_irq,
	.unmask	= vic_unmask_irq,
	.set_wake	= vic_set_wake_irq,
};

void static vic_pl192_init(void __iomem *base, unsigned int irq_start,
		     u32 vic_sources)
{
	unsigned int i;

	/* Disable all interrupts initially. */

	writel(0, base + VIC_INT_SELECT);
	writel(0, base + VIC_INT_ENABLE);
	writel(~0, base + VIC_INT_ENABLE_CLEAR);
	writel(0, base + VIC_IRQ_STATUS);
	writel(~0, base + VIC_INT_SOFT_CLEAR);

	/*
	 * Make sure we clear all existing interrupts
	 */

	writel(0, base + VIC_PL192_VECT_ADDR);
#if 0
	for (i = 0; i < 32; i++)
		writel(irq_start + i, base + VIC_VECT_ADDR0 + i*4);

	for (i = 0; i < 32; i++) {
		writel(0, base + VIC_VECT_CNTL0 + i*4);
	}
#endif

	for (i = 0; i < 32; i++) {
		unsigned int irq = irq_start + i;


		if (vic_sources & (1 << i)) {
			set_irq_chip(irq, &vic_chip);
			set_irq_chip_data(irq, base);
			set_irq_handler(irq, handle_level_irq);
			set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
		}
	}
}
void __init socle_init_irq(void)
{
	/* initialise the pair of VICs */
	vic_pl192_init((void __iomem*)IO_ADDRESS(SOCLE_VIC0), 0, ~0);
	vic_pl192_init ((void __iomem *) IO_ADDRESS (SOCLE_VIC1), 32,
			( 1 << ( IRQ_EXT5 /* for FPGA */ - 32) | 1 << (IRQ_MALI200_MMU - 32) | 1 << (IRQ_MALI200_MALIGP2 - 32)
			 | 1 << (IRQ_MALI200_MALI200 - 32) | 1 << (IRQ_F75111 - 32) | 1 << (IRQ_EXT3 - 32)));

}
#else
void socle_mask_irq(unsigned int irq)
{
	INT0_DISABLE(irq);
	INT0_CLR_MASK(irq);
	writel(1 << (irq), INTC0_ICCR);
}

void socle_unmask_irq(unsigned int irq)
{
	INT0_SET_MASK(irq);
	INT0_ENABLE(irq);
}

static struct irq_chip socle_irq_chip = {
	.ack	= socle_mask_irq,
	.mask	= socle_mask_irq,
	.unmask	= socle_unmask_irq,
};

void __init socle_init_irq(void)
{
	unsigned int i;
	// disable all interrupt
	writel(0, INTC0_IECR);
	// clear all interrupt
	writel(0xFFFFFFFF, INTC0_ICCR);	
	/* Disable all interrupts initially. */

	for (i = 0; i < NR_IRQS; i++) {
#ifdef CONFIG_PCIE
		if (MAX_IRQS == i)
			break;
#endif

#if defined(CONFIG_ARCH_PDK_PC9002) || defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_SCDK)
		if((i >= 16) && (i <= 19)){
			INT0_SET_TYPE( i, LO_LEVEL);
		}else if ((i == 29)||(i == 30)) {                       //for ads7846 irq : 30
			INT0_SET_TYPE( i, LO_LEVEL);		//for touch screen tsc2000
		}else if ((i == 22) || (i == 23)) {		//for hdma
			INT0_SET_TYPE( i, POSITIVE_EDGE);
		}else{
			INT0_SET_TYPE( i, HI_LEVEL);
		}
#elif (defined(CONFIG_ARCH_PDK_PC7210) || defined(CONFIG_ARCH_P7DK))
		INT0_SET_TYPE( i, HI_LEVEL);
#elif defined(CONFIG_ARCH_LDK3V21)
		if((i >= 11) && (i <= 14)){
			INT0_SET_TYPE( i, LO_LEVEL);
		}else{
			INT0_SET_TYPE( i, HI_LEVEL);
		}
#elif defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
		if (i==30) {			
			INT0_SET_TYPE( i, NEGATIVE_EDGE);		//20100629 jerry+ for spi wifi
		} else if(i == 29) {
			INT0_SET_TYPE( i, LO_LEVEL);		//for touch screen tsc2000 and spi wifi
//		}else if (i == 27) {		//for hdma
//			INT0_SET_TYPE( i, POSITIVE_EDGE);
		}else{
			INT0_SET_TYPE( i, HI_LEVEL);
		}
#else
		INT0_SET_TYPE( i, HI_LEVEL);
#endif		
		set_irq_chip(i, &socle_irq_chip);
		set_irq_handler(i, handle_level_irq);	// 20090209 cyli fix from do_level_IRQ to handle_level_irq
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);	

	}
#ifdef CONFIG_PCIE
	socle_init_pcie_irq();
#endif
}
#endif

