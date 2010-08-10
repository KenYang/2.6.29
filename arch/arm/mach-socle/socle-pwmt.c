/********************************************************************************
* File Name     : drivers/char/socle-pwmt.c 
* Author        : cyli
* Description   : Socle PWM Timer Driver
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
    
*   Version      : 2,0,0,1
*   History      : 
*      1. 2007/03/21 cyli create this file
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <mach/irqs.h>
//#include <linux/cdev.h>
//#include <asm/uaccess.h>		// for ioctl

#include <mach/regs-pwmt.h>

#if defined(CONFIG_ARCH_PDK_PC9220)
#include <mach/pc9220-scu.h>
#endif
#if defined(CONFIG_ARCH_PDK_PC9223)
#include <mach/pc9223-scu.h>
#endif

//#define CONFIG_SOCLE_PWMT_DEBUG
#ifdef CONFIG_SOCLE_PWMT_DEBUG
	#define PWMT_DBG(fmt, args...) printk(KERN_DEBUG "SOCLE_PWMT: " fmt, ## args)
#else
	#define PWMT_DBG(fmt, args...)
#endif


static DEFINE_SPINLOCK(pwmt_lock);

/* socle_pwmt_driver */
static void socle_pwmt_claim_lock(void);
static void socle_pwmt_release_lock(void);
static void socle_pwmt_reset(struct socle_pwmt *p_pwm_st);
static void socle_pwmt_output_enable(struct socle_pwmt *p_pwm_st, int en);
static void socle_pwmt_enable(struct socle_pwmt *p_pwm_st, int en);
static void socle_pwmt_capture_mode_enable(struct socle_pwmt *p_pwm_st, int en);
static void socle_pwmt_clear_interrupt(struct socle_pwmt *p_pwm_st);
static void socle_pwmt_enable_interrupt(struct socle_pwmt *p_pwm_st, int en);
static void socle_pwmt_single_counter_mode_enable(struct socle_pwmt *p_pwm_st, int en);
static void socle_pwmt_set_counter(struct socle_pwmt *p_pwm_st, unsigned int data);
static unsigned int socle_pwmt_read_hrc(struct socle_pwmt *p_pwm_st);
static unsigned int socle_pwmt_read_lrc(struct socle_pwmt *p_pwm_st);
static void socle_pwmt_write_hrc(struct socle_pwmt *p_pwm_st, unsigned int data);
static void socle_pwmt_write_lrc(struct socle_pwmt *p_pwm_st, unsigned int data);
static unsigned int socle_pwmt_read_prescale_factor(struct socle_pwmt *p_pwm_st);
static void socle_pwmt_write_prescale_factor(struct socle_pwmt *p_pwm_st, unsigned int data);
static void socle_pwmt_suspend(struct socle_pwmt *p_pwm_st);
static void socle_pwmt_resume(struct socle_pwmt *p_pwm_st);
#ifdef CONFIG_ARCH_PDK_PC7210
static void socle_pwmt_i2s_phy_clock_on(struct socle_pwmt *p_pwm_st);
static void socle_pwmt_i2s_phy_clock_off(struct socle_pwmt *p_pwm_st);
#endif

static struct socle_pwmt_driver socle_pwm_drv = {
	.claim_pwm_lock	= socle_pwmt_claim_lock,
	.release_pwm_lock	= socle_pwmt_release_lock,
	.reset	= socle_pwmt_reset,
	.output_enable	= socle_pwmt_output_enable,
	.enable	= socle_pwmt_enable,
	.capture_mode_enable	= socle_pwmt_capture_mode_enable,
	.clear_interrupt	= socle_pwmt_clear_interrupt,
	.enable_interrupt	= socle_pwmt_enable_interrupt,
	.single_counter_mode_enable	= socle_pwmt_single_counter_mode_enable,
	.set_counter	= socle_pwmt_set_counter,
	.read_hrc	= socle_pwmt_read_hrc,
	.read_lrc	= socle_pwmt_read_lrc,
	.write_hrc	= socle_pwmt_write_hrc,
	.write_lrc	= socle_pwmt_write_lrc,
	.read_prescale_factor	= socle_pwmt_read_prescale_factor,
	.write_prescale_factor	= socle_pwmt_write_prescale_factor,
	.suspend	= socle_pwmt_suspend,
	.resume	= socle_pwmt_resume,
#ifdef CONFIG_ARCH_PDK_PC7210
	.i2s_phy_clock_on	=	socle_pwmt_i2s_phy_clock_on,
	.i2s_phy_clock_off	=	socle_pwmt_i2s_phy_clock_off,
#endif
};

/* socle pwmt devie */
static struct socle_pwmt socle_pwm[SOCLE_PWM_NUM];


//static char __initdata banner[] = "SOCLE PWM Timer, (c) 2007 SOCLE Corp.\n";

static inline void
pwmt_read(unsigned int offset, unsigned int *data, unsigned int base)
{
	*data = ioread32(base + offset);

	PWMT_DBG("pwmt_read(): base:0x%08x, offset:0x%08x, data:0x%08x\n", base, offset, *data);
}

static inline void
pwmt_write(unsigned int offset, unsigned int data, unsigned int base)
{
	iowrite32(data, base + offset);

	PWMT_DBG("pwmt_write(): base:0x%08x, offset:0x%08x, data:0x%08x\n", base, offset, data);
}

static void
socle_pwmt_claim_lock(void)
{
	spin_lock(&pwmt_lock);
}

static void
socle_pwmt_release_lock(void)
{
	spin_unlock(&pwmt_lock);
}

static void
socle_pwmt_reset(struct socle_pwmt *p_pwm_st)
{
	pwmt_write(PWMT_CTRL, PWMT_CTRL_RST, p_pwm_st->base);
}

static void
socle_pwmt_output_enable(struct socle_pwmt *p_pwm_st, int en)
{
	unsigned int tmp;

	pwmt_read(PWMT_CTRL, &tmp, p_pwm_st->base);

	if (en)
		pwmt_write(PWMT_CTRL, tmp | PWMT_CTRL_OPT_EN, p_pwm_st->base);
	else
		pwmt_write(PWMT_CTRL, tmp & ~PWMT_CTRL_OPT_EN, p_pwm_st->base);
}

static void
socle_pwmt_enable(struct socle_pwmt *p_pwm_st, int en)
{
	unsigned int tmp;
	
	pwmt_read(PWMT_CTRL, &tmp, p_pwm_st->base);

	if (en)
		pwmt_write(PWMT_CTRL, tmp | PWMT_CTRL_EN, p_pwm_st->base);
	else
		pwmt_write(PWMT_CTRL, tmp & ~PWMT_CTRL_EN, p_pwm_st->base);
}

static void
socle_pwmt_capture_mode_enable(struct socle_pwmt *p_pwm_st, int en)
{
	unsigned int tmp;
	
	pwmt_read(PWMT_CTRL, &tmp, p_pwm_st->base);

	if (en)
		pwmt_write(PWMT_CTRL, tmp | PWMT_CTRL_CAP, p_pwm_st->base);
	else
		pwmt_write(PWMT_CTRL, tmp & ~PWMT_CTRL_CAP, p_pwm_st->base);
}

static void
socle_pwmt_clear_interrupt(struct socle_pwmt *p_pwm_st)
{
	u32 tmp;
	
	pwmt_read(PWMT_CTRL, &tmp, p_pwm_st->base);
	pwmt_write(PWMT_CTRL, tmp | PWMT_CTRL_INT_CLR, p_pwm_st->base);
}

static void
socle_pwmt_enable_interrupt(struct socle_pwmt *p_pwm_st, int en)
{
	unsigned int tmp;
	
	pwmt_read(PWMT_CTRL, &tmp, p_pwm_st->base);

	if (en)
		pwmt_write(PWMT_CTRL, tmp | PWMT_CTRL_INT_EN, p_pwm_st->base);
	else
		pwmt_write(PWMT_CTRL, tmp & ~PWMT_CTRL_INT_EN, p_pwm_st->base);
}

static void
socle_pwmt_single_counter_mode_enable(struct socle_pwmt *p_pwm_st, int en)
{
	unsigned int tmp;
	
	pwmt_read(PWMT_CTRL, &tmp, p_pwm_st->base);

	if (en)
		pwmt_write(PWMT_CTRL, tmp | PWMT_CTRL_SIG_CNTR, p_pwm_st->base);
	else
		pwmt_write(PWMT_CTRL, tmp & ~PWMT_CTRL_SIG_CNTR, p_pwm_st->base);
}

static void
socle_pwmt_set_counter(struct socle_pwmt *p_pwm_st, unsigned int data)
{
	pwmt_write(PWMT_CNTR, data, p_pwm_st->base);
}

static unsigned int
socle_pwmt_read_hrc(struct socle_pwmt *p_pwm_st)
{
	unsigned int data;

	pwmt_read(PWMT_HRC, &data, p_pwm_st->base);
	
	return data;
}

static unsigned int
socle_pwmt_read_lrc(struct socle_pwmt *p_pwm_st)
{
	unsigned int data;

	pwmt_read(PWMT_LRC, &data, p_pwm_st->base);
	
	return data;
}

static void
socle_pwmt_write_hrc(struct socle_pwmt *p_pwm_st, unsigned int data)
{
	pwmt_write(PWMT_HRC, data, p_pwm_st->base);
}

static void
socle_pwmt_write_lrc(struct socle_pwmt *p_pwm_st, unsigned int data)
{
	pwmt_write(PWMT_LRC, data, p_pwm_st->base);
}

static unsigned int
socle_pwmt_read_prescale_factor(struct socle_pwmt *p_pwm_st)
{
	unsigned int data;

	pwmt_read(PWMT_CTRL, &data, p_pwm_st->base);
	data = (data & PWMT_CTRL_PRESCALE_MSK) >> PWMT_CTRL_PRESCALE_S;

	return data;
}

static void
socle_pwmt_write_prescale_factor(struct socle_pwmt *p_pwm_st, unsigned int data)
{
	unsigned int tmp;

	pwmt_read(PWMT_CTRL, &tmp, p_pwm_st->base);
	pwmt_write(PWMT_CTRL, (tmp & PWMT_CTRL_PRESCALE_MSK) | (data << PWMT_CTRL_PRESCALE_S), p_pwm_st->base);
}

u32 pwmt_save_addr[SOCLE_PWM_NUM][4]; 

static void
socle_pwmt_suspend(struct socle_pwmt *p_pwm_st)
{
	int tmp;

	printk("socle_pwmt_suspend : index = %d\n", p_pwm_st->index);

	for(tmp=0;tmp<4;tmp++){
		iowrite32(ioread32((p_pwm_st->base)+(tmp*4)), &pwmt_save_addr[p_pwm_st->index][tmp]);
	}
}


static void
socle_pwmt_resume(struct socle_pwmt *p_pwm_st)
{
	int tmp;
	
	printk("socle_pwmt_resume : index = %d\n", p_pwm_st->index);

	for(tmp=0;tmp<4;tmp++){
		iowrite32(pwmt_save_addr[p_pwm_st->index][tmp], ((p_pwm_st->base)+(tmp*4)));
	}
}



#ifdef CONFIG_ARCH_PDK_PC7210
static void
socle_pwmt_i2s_phy_clock_on(struct socle_pwmt *p_pwm_st)
{
	/*	pull high to power on	*/
	pwmt_write(PWMT_CTRL, PWMT_CTRL_RST, p_pwm_st->base);
	pwmt_write(PWMT_HRC, 0x100, p_pwm_st->base);
	pwmt_write(PWMT_LRC, 0x100, p_pwm_st->base);
	pwmt_write(PWMT_CTRL, 0x100, p_pwm_st->base);
	pwmt_write(PWMT_CTRL, (PWMT_CTRL_EN|PWMT_CTRL_OPT_EN), p_pwm_st->base);
}

static void
socle_pwmt_i2s_phy_clock_off(struct socle_pwmt *p_pwm_st)
{			
	/*	pull low to power off	*/
	pwmt_write(PWMT_CTRL, PWMT_CTRL_RST, p_pwm_st->base);
	pwmt_write(PWMT_HRC, 0x0, p_pwm_st->base);
	pwmt_write(PWMT_LRC, 0x100, p_pwm_st->base);
	pwmt_write(PWMT_CTRL, 0x100, p_pwm_st->base);
	pwmt_write(PWMT_CTRL, PWMT_CTRL_EN, p_pwm_st->base);
	pwmt_write(PWMT_CTRL, PWMT_CTRL_OPT_EN, p_pwm_st->base);
}
#endif

extern struct socle_pwmt *
get_socle_pwmt_structure(int num)
{
	struct socle_pwmt *p = NULL;

	if (SOCLE_PWM_NUM <= num) {
		printk("Error request PWMT num = %d!, max num = %d\n", num, SOCLE_PWM_NUM - 1);
		return p;
	}

	socle_pwmt_claim_lock();

	if (socle_pwm[num].busy) {
		printk("PWMT[%d] is busy!\n", num);
	} else {
		socle_pwm[num].busy = 1;
		p = &socle_pwm[num];
		p->index = num;
	}

	PWMT_DBG("PWMT[%d] base = 0x%08x, irq = %d\n", num, p->base, p->irq);

	socle_pwmt_release_lock();

	return p;
}
EXPORT_SYMBOL(get_socle_pwmt_structure);

extern int
release_socle_pwmt_structure(int num)
{
	int ret = 0;

	if (SOCLE_PWM_NUM <= num) {
		printk("Error request PWMT num = %d!, max num = %d\n", num, SOCLE_PWM_NUM - 1);
		return -EINVAL;
	}

	socle_pwmt_claim_lock();

	socle_pwm[num].busy = 0;

	PWMT_DBG("Release PWMT[%d]!\n", num);

	socle_pwmt_release_lock();

	return ret;
}
EXPORT_SYMBOL(release_socle_pwmt_structure);

static int __init
socle_pwmt_init(void)
{
	int ret = 0, i;
	unsigned int base;
//	struct resource *pwmt_resource;
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_PWM0);
	socle_scu_dev_enable(SOCLE_DEVCON_PWM1);
#endif
	PWMT_DBG("pwmt probe\n");

	base = (int)ioremap(SOCLE_TIMER_PWM0, SZ_4K);

	for (i = 0; i < SOCLE_PWM_NUM; i++) {
		if (i)
			base += PWMT_BASE_OFFSET;

		socle_pwm[i].base = base;
		socle_pwm[i].irq = IRQ_PWM0+i;
		socle_pwm[i].drv = &socle_pwm_drv;
		socle_pwm[i].busy = 0;

//		socle_pwmt_reset(&socle_pwm[i]);
	}
	

	return ret;
}

core_initcall(socle_pwmt_init);

MODULE_DESCRIPTION("SOCLE PWMT Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("CY Li <cyli@socle-tech.com.tw>");

