/*
 * include/asm-arm/arch-socle/entry-macro.S
 *
 * Low-level IRQ helper macros for SOCLE platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifdef CONFIG_ARCH_MDK_3D
#include <asm/hardware/vic.h>
#endif
#include <mach/regs-intr.h>

		.macro	disable_fiq
		.endm

		@ 20090210 cyli add
		.macro  get_irqnr_preamble, base, tmp
		.endm

		@ 20090210 cyli add
		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

#ifdef CONFIG_ARCH_MDK_3D
        .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp

        @ check the vic0
	ldr	\base, =IO_ADDRESS(INTC0_PHY_ADDR)
        mov     \irqnr, #31
        ldr     \irqstat, [ \base, # VIC_IRQ_STATUS ]
        teq     \irqstat, #0

        @ otherwise try vic1
        addeq   \tmp, \base, #(SOCLE_VIC1 - SOCLE_VIC0)
        addeq   \irqnr, \irqnr, #32
        ldreq   \irqstat, [ \tmp, # VIC_IRQ_STATUS ]
        teqeq   \irqstat, #0

        clzne   \irqstat, \irqstat
        subne   \irqnr, \irqnr, \irqstat
        .endm
#else
		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
/* FIXME: should not be using soo many LDRs here */
		ldr	\base, =IO_ADDRESS(INTC0_PHY_ADDR)
        ldr     \irqnr,[\base,#260]   @ INTC0_PHY_ADDR + 0x104 ISR
     
        teq    \irqnr, #31
        .endm
#endif

		.macro	irq_prio_table
		.endm




