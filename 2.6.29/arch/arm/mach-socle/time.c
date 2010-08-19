/*
 * linux/arch/arm/mach-socle/time.c
 *
 *  Copyright (C) 2007 Socle
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

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/module.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/time.h>

#include <mach/regs-timer.h>

#include <mach/socle-scu.h>

#include "devs.h"

#define CALCULATE_CPU_FREQ

#ifdef CALCULATE_CPU_FREQ

#define TICKS2USECS(x)	((x) / (apb_clock/1000000))  
unsigned long apb_clock;
unsigned long TIMER_RELOAD;
#else

#define TICKS2USECS(x)	((x) / 50) 
#define TIMER_RELOAD	(500000)
unsigned long apb_clock = 500000;
#endif

/*
 * What does it look like?
 */
typedef struct TimerStruct {
	unsigned long TimerLoad;
	unsigned long TimerValue;
	unsigned long TimerControl;
} TimerStruct_t;

/*
 * Returns number of ms since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long socle_gettimeoffset(void)
{
	volatile TimerStruct_t *timer1 = (TimerStruct_t *)TIMER1_VA_BASE;
	unsigned long ticks1, ticks2, status;

	/*
	 * Get the current number of ticks.  Note that there is a race
	 * condition between us reading the timer and checking for
	 * an interrupt.  We get around this by ensuring that the
	 * counter has not reloaded between our two reads.
	 */
	ticks2 = timer1->TimerValue;;
	status = timer1->TimerControl ;

	/*
	 * Number of ticks since last interrupt.
	 */
	ticks1 = TIMER_RELOAD - ticks2;
	/*
	 * Interrupt pending?  If so, we've reloaded once already.
	 *
	 * FIXME: Need to check this is effectively timer 0 that expires
	 */
	 /*
	if (status & (1<<2))  //timer's pending bit
	{
		ticks1 += TIMER_RELOAD;
	}
	*/

	return TICKS2USECS(ticks1);

}

/*
 * IRQ handler for the timer
 */

static irqreturn_t socle_timer_interrupt(int irq, void *dev_id)
{
  	volatile TimerStruct_t *timer1 = (volatile TimerStruct_t *) TIMER1_VA_BASE;

// 20090213 cyli remove lock
//	write_seqlock (&xtime_lock);
	// Clear the interrupt
  	timer1->TimerControl = 0x182;
   	timer_tick ();
//	write_sequnlock (&xtime_lock);
	return IRQ_HANDLED;
}

static struct irqaction socle_timer_irq = {
	.name		= "SOCLE Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler		= socle_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init socle_init_timer(void)
{
	struct timespec tv;
	volatile TimerStruct_t *timer0 = (volatile TimerStruct_t *)TIMER0_VA_BASE;
	volatile TimerStruct_t *timer1 = (volatile TimerStruct_t *)TIMER1_VA_BASE;
	volatile TimerStruct_t *timer2 = (volatile TimerStruct_t *)TIMER2_VA_BASE;

	xtime.tv_nsec = 0;
	xtime.tv_sec  = 0;
	tv.tv_nsec = 0;
	tv.tv_sec  = 0;

	/*
	 * Initialise to a known state (all timers off)
	 */
	timer0->TimerControl = 0;
	timer1->TimerControl = 0;
	timer2->TimerControl = 0;

#ifdef CALCULATE_CPU_FREQ
	apb_clock = socle_scu_apb_clock_get();
	socle_scu_show_system_info();
	TIMER_RELOAD=apb_clock/HZ;
#endif

	timer1->TimerLoad = TIMER_RELOAD;
	tick_usec = TICKS2USECS(TIMER_RELOAD);
	/* 
	 * Make irqs happen for the system timer
	 */
	
	do_settimeofday(&tv);
	timer1->TimerControl = 0x182; /* periodic + IE ; prescale factor=1*/
	setup_irq(IRQ_TIMER1, &socle_timer_irq);

}

#ifdef CONFIG_PM

#define SOCLE_CLCD_REG_NUM	13

u32 socle_timer_save_addr[SOCLE_CLCD_REG_NUM]; 

static void socle_timer_suspend(void)
{
	u32 tmp;
	u32 addr_int = TIMER0_VA_BASE;
	
	for(tmp=0;tmp<0x30;tmp+=0x10){
		iowrite32(ioread32(addr_int+tmp), (socle_timer_save_addr+tmp));
		iowrite32(ioread32(addr_int+tmp+0x8), (socle_timer_save_addr+tmp+0x8));
	}
	
	return ;

}

static void socle_timer_resume(void)
{
	u32 tmp;
	u32 addr_int = TIMER0_VA_BASE;

	for(tmp=0;tmp<0x30;tmp+=0x10){
		iowrite32(ioread32(socle_timer_save_addr+tmp), (addr_int+tmp));
		iowrite32(ioread32(socle_timer_save_addr+tmp+0x8), (addr_int+tmp+0x8));
	}
	
	return ;

}

#else
#define socle_timer_suspend	NULL
#define socle_timer_resume NULL
#endif


struct sys_timer socle_timer = {
	.init			= socle_init_timer,
	.offset		= socle_gettimeoffset,
	.suspend		= socle_timer_suspend,
	.resume 		= socle_timer_resume,
};

