/********************************************************************************
* File Name     : drivers/char/rtc-socle.c 
* Author         : ryan chen
* Description   : Socle Real Time Clock Driver (RTC)
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
*      1. 2006/08/25 ryan chen create this file
*	 2. 2006/12/25 cyli modify this file
*	 3. 2010/06/09 cyli add power management
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>

#include <asm/mach/time.h>

#include <asm/io.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/regs-rtc.h>

//#define CONFIG_RTC_DEBUG
#ifdef CONFIG_RTC_DEBUG
#define RTCDBUG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
#define RTCDBUG(fmt, args...)
#endif

static int rtc_busy = 0;

static void __iomem *socle_rtc_base;

static unsigned long rtc_ctrl_reg = 0;	/* year corresponding to 0x00	*/

spinlock_t socle_rtc_lock = SPIN_LOCK_UNLOCKED;

void __iomem            *rtc_regs;
int                     irq;

/* Time read/write */
static int
socle_rtc_get_time(struct device *dev, struct rtc_time *rtc_tm)
{
	unsigned long flags;
	u32 reg_time, reg_date;
	u32 year, mon, wday, day, hr, mn, sec;

	spin_lock_irqsave(&socle_rtc_lock, flags);

	reg_time	= readl(SOCLE_RTC_TIME);
	reg_date	= readl(SOCLE_RTC_DATE);

	RTCDBUG("reg_time=%08x, reg_date=%08x\n", reg_time, reg_date);

	/* get seconds */
	sec = (reg_time >> SOCLE_RTC_TIME_S) & 0x0f;
	/* get ten secondss */
	sec += ((reg_time >> SOCLE_RTC_TIME_TS) & 0x07) * 10;	
	rtc_tm->tm_sec = sec;

	/* get minutes */
	mn = (reg_time >> SOCLE_RTC_TIME_M) & 0x0f;
	/* get ten minutes */
	mn += ((reg_time >> SOCLE_RTC_TIME_TM) & 0x07) * 10;
	rtc_tm->tm_min = mn;

	/* get hours */
	hr = (reg_time >> SOCLE_RTC_TIME_H) & 0x0f;
	/* get ten hours */
	hr += ((reg_time >> SOCLE_RTC_TIME_TH) & 0x03) * 10;
	rtc_tm->tm_hour = hr;

	/* get day of week */
	wday = (reg_time >> SOCLE_RTC_TIME_DOW) & 0x07;
	rtc_tm->tm_wday= wday;

	/* get day */
	day = (reg_date >> SOCLE_RTC_DATE_D) & 0x0f;
	/* get ten day */
	day += ((reg_date >> SOCLE_RTC_DATE_TD) & 0x03) * 10;
	rtc_tm->tm_mday = day;

	/* get mon */
	mon = (reg_date >> SOCLE_RTC_DATE_M) & 0x0f;
	/* get ten mon */
	mon += ((reg_date >> SOCLE_RTC_DATE_TM) & 0x01) * 10;
	rtc_tm->tm_mon = mon-1;

	/* get year */
	year = (reg_date >> SOCLE_RTC_DATE_Y) & 0x0f;
	/* get ten year */
	year += ((reg_date >> SOCLE_RTC_DATE_TY) & 0x0f) * 10;
	/* get century */
	year += ((reg_date >> SOCLE_RTC_DATE_C) & 0x0f) * 100;
	/* get ten century */
	year += ((reg_date >> SOCLE_RTC_DATE_TC) & 0x0f) * 1000;
	rtc_tm->tm_year = year;

	RTCDBUG( "%04d/%02d/%02d %02d:%02d:%02d w(%d)\n", year+1900, mon, day, hr, mn, sec, wday);

	spin_unlock_irqrestore(&socle_rtc_lock, flags);

	return 0;
}


static int
socle_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags;
	u32 reg_time, reg_date;

	spin_lock_irqsave(&socle_rtc_lock, flags);

	tm->tm_mon++; /* conversion tm_mon(0..11) to (1..12)*/

	RTCDBUG("%04d/%02d/%02d %02d:%02d:%02d w(%d)\n", tm->tm_year+1900, tm->tm_mon, tm->tm_mday,
						tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_wday);
	
	/* get day of week */
	reg_time = (tm->tm_wday) << SOCLE_RTC_TIME_DOW;
	
	/* get ten hours */
	reg_time |= (tm->tm_hour / 10) << SOCLE_RTC_TIME_TH;
	/* get hours */
	reg_time |= (tm->tm_hour % 10) << SOCLE_RTC_TIME_H;
	
	/* get ten minutes */
	reg_time |= (tm->tm_min / 10) << SOCLE_RTC_TIME_TM;
	/* get minutes */
	reg_time |= (tm->tm_min % 10) << SOCLE_RTC_TIME_M;
	
	/* get ten secondss */
	reg_time |= (tm->tm_sec / 10) << SOCLE_RTC_TIME_TS;
	/* get secondss */
	reg_time |= (tm->tm_sec % 10) << SOCLE_RTC_TIME_S;


	/* get ten century */
	reg_date = (tm->tm_year / 1000) << SOCLE_RTC_DATE_TC;
	/* get century */
	reg_date |= ((tm->tm_year / 100) % 10) << SOCLE_RTC_DATE_C;
	/* get ten year */
	reg_date |= ((tm->tm_year / 10) % 10) << SOCLE_RTC_DATE_TY;
	/* get year */
	reg_date |= (tm->tm_year % 10) << SOCLE_RTC_DATE_Y;
	
	/* get ten mon */
	reg_date |= (tm->tm_mon / 10) << SOCLE_RTC_DATE_TM;
	/* get mon */
	reg_date |= (tm->tm_mon % 10) << SOCLE_RTC_DATE_M;
	
	/* get ten day */
	reg_date |= (tm->tm_mday / 10) << SOCLE_RTC_DATE_TD;
	/* get day */
	reg_date |= (tm->tm_mday % 10) << SOCLE_RTC_DATE_D;

	rtc_ctrl_reg &= ~SOCLE_RTC_CTRL_EN;
	writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);
	writel(reg_time, SOCLE_RTC_TIME);
	writel(reg_date, SOCLE_RTC_DATE);
	rtc_ctrl_reg |= SOCLE_RTC_CTRL_EN;
	writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);
	
	RTCDBUG("reg_time=%08x, reg_date=%08x\n", reg_time, reg_date);

	spin_unlock_irqrestore(&socle_rtc_lock, flags);

	return 0;
}


static int
socle_rtc_get_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	unsigned long flags;
	struct rtc_time *alm_tm = &alarm->time;
	u32 reg_time, reg_date;
	u32 year, mon, wday, day, hr, mn, sec;

	spin_lock_irqsave(&socle_rtc_lock, flags);

	reg_time = readl(SOCLE_RTC_TALRM);
	reg_date = readl(SOCLE_RTC_DALRM);

	RTCDBUG("reg_time= %08x, reg_date=%08x\n", reg_time, reg_date);

	/* get seconds */
	sec = (reg_time >> SOCLE_RTC_TIME_S) & 0x0f;
	/* get ten secondss */
	sec += ((reg_time >> SOCLE_RTC_TIME_TS) & 0x07) * 10;
	alm_tm->tm_sec = sec;
	
	/* get minutes */
	mn = (reg_time >> SOCLE_RTC_TIME_M) & 0x0f;
	/* get ten minutes */
	mn += ((reg_time >> SOCLE_RTC_TIME_TM) & 0x07) * 10;
	alm_tm->tm_min = mn;
	
	/* get hours */
	hr = (reg_time >> SOCLE_RTC_TIME_H) & 0x0f;
	/* get ten hours */
	hr += ((reg_time >> SOCLE_RTC_TIME_TH) & 0x03) * 10;
	alm_tm->tm_hour = hr;

	/* get day of week */
	wday = (reg_time >> SOCLE_RTC_TIME_DOW) & 0x07;
	alm_tm->tm_wday= wday;
	
	/* get day */
	day = (reg_date >> SOCLE_RTC_DATE_D) & 0x0f;
	/* get ten day */
	day += ((reg_date >> SOCLE_RTC_DATE_TD) & 0x03) * 10;
	alm_tm->tm_mday = day;
	
	/* get mon */
	mon = (reg_date >> SOCLE_RTC_DATE_M) & 0x0f;
	/* get ten mon */
	mon += ((reg_date >> SOCLE_RTC_DATE_TM) & 0x01) * 10;
	alm_tm->tm_mon = mon;
	
	/* get year */
	year = (reg_date >> SOCLE_RTC_DATE_Y) & 0x0f;
	/* get ten year */
	year += ((reg_date >> SOCLE_RTC_DATE_TY) & 0x0f) * 10;
	/* get century */
	year += ((reg_date >> SOCLE_RTC_DATE_C) & 0x0f) * 100;
	/* get ten century */
	year += ((reg_date >> SOCLE_RTC_DATE_TC) & 0x0f) * 1000;
	alm_tm->tm_year = year;

	RTCDBUG("%04d/%02d/%02d %02d:%02d:%02d w(%d)\n", year+1900, mon, day, hr, mn, sec, wday);

	spin_unlock_irqrestore(&socle_rtc_lock, flags);

	return 0;
}


static int
socle_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct rtc_time *tm = &alarm->time;
	unsigned long flags;
	u32 reg_time, reg_date;
	u32 talrm_en = 0 ,dalrm_en = 0;
	u32 rtc_talrm_reg = 0;	/* alarm time temp	*/
	u32 rtc_dalrm_reg = 0;	/* alarm date temp	*/

	spin_lock_irqsave(&socle_rtc_lock, flags);

	RTCDBUG("%04d/%02d/%02d %02d:%02d:%02d w(%d)\n", tm->tm_year+1900, tm->tm_mon, tm->tm_mday,
						tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_wday);

//DAY
	/* get day of week */
	if (tm->tm_wday < 8 && tm->tm_wday >= 1) {
		talrm_en |= SOCLE_RTC_TALRM_CDOW;
	}
	reg_time = (tm->tm_wday) << SOCLE_RTC_TIME_DOW;
//HR
	/* get ten hours */
	if (tm->tm_hour < 24 && tm->tm_hour >= 0) {
		talrm_en |= SOCLE_RTC_TALRM_CH;
	}
	reg_time |= (tm->tm_hour / 10) << SOCLE_RTC_TIME_TH;

	/* get hours */
	reg_time |= (tm->tm_hour % 10) << SOCLE_RTC_TIME_H;
//MIN
	/* get ten minutes */
	if (tm->tm_min < 60 && tm->tm_min >= 0) {
		talrm_en |= SOCLE_RTC_TALRM_CM;
	}
	reg_time |= (tm->tm_min / 10) << SOCLE_RTC_TIME_TM;
	/* get minutes */
	reg_time |= (tm->tm_min % 10) << SOCLE_RTC_TIME_M;
//SEC
	/* get ten secondss */
	if (tm->tm_sec < 60 && tm->tm_sec >= 0) {
		talrm_en |= SOCLE_RTC_TALRM_CS;
	}
	reg_time |= (tm->tm_sec / 10) << SOCLE_RTC_TIME_TS;
	/* get secondss */
	reg_time |= (tm->tm_sec % 10) << SOCLE_RTC_TIME_S;

//CC
	/* get ten century */
	if (tm->tm_year < 9999 && tm->tm_year >= 1) {
		dalrm_en |= SOCLE_RTC_DALRM_CC | SOCLE_RTC_DALRM_CY;
	}
	reg_date = (tm->tm_year / 1000) << SOCLE_RTC_DATE_TC;
	/* get century */
	reg_date |= ((tm->tm_year / 100) % 10) << SOCLE_RTC_DATE_C;
//YEAR
	/* get ten year */
	//if (tm->tm_year < 9999 && tm->tm_year >= 100) {
	//	alrm_en |= RTC_DALRM_CC;
	//}
	reg_date |= ((tm->tm_year / 10) % 10) << SOCLE_RTC_DATE_TY;
	/* get year */
	reg_date |= (tm->tm_year % 10) << SOCLE_RTC_DATE_Y;
//MON
	/* get ten mon */
	if (tm->tm_mon < 12 && tm->tm_mon >= 1) {
		dalrm_en |= SOCLE_RTC_DALRM_CM;
	}
	//tm->tm_mon++; /* conversion tm_mon(0..11) to (1..12)*/
	reg_date |= (tm->tm_mon / 10) << SOCLE_RTC_DATE_TM;
	/* get mon */
	reg_date |= (tm->tm_mon % 10) << SOCLE_RTC_DATE_M;
//DAY
	if (tm->tm_mday < 32 && tm->tm_mday >= 1) {
		dalrm_en |= SOCLE_RTC_DALRM_CD;
	}

	/* get ten day */
	reg_date |= (tm->tm_mday / 10) << SOCLE_RTC_DATE_TD;
	/* get day */
	reg_date |= (tm->tm_mday % 10) << SOCLE_RTC_DATE_D;

   	rtc_talrm_reg = (reg_time & ~0xf8000000) | talrm_en;
    	rtc_dalrm_reg = (reg_date & ~0xf8000000) |dalrm_en;
	writel(rtc_talrm_reg, SOCLE_RTC_TALRM);
	writel(rtc_dalrm_reg, SOCLE_RTC_DALRM);

	RTCDBUG("talrm_en=%08x , dalrm_en=%08x\n", talrm_en, dalrm_en);
	RTCDBUG("reg_time=%08x , reg_date=%08x\n", reg_time, reg_date);
	RTCDBUG("rtc_talrm_reg=%08x , rtc_dalrm_reg=%08x\n", rtc_talrm_reg, rtc_dalrm_reg);

	spin_unlock_irqrestore(&socle_rtc_lock, flags);
	
	return 0;
}

static int
socle_rtc_setfreq(struct device *dev, int freq)
{
	u32 tmp;

	if ((freq < 1) || (freq > RTC_CLK_FREQ))
		return -EINVAL;

	spin_lock_irq(&socle_rtc_lock);

	tmp = readl(SOCLE_RTC_CTRL);
	tmp = tmp & 0xf8000000;
	tmp |= ((RTC_CLK_FREQ / freq) - 1);
	RTCDBUG("RTC_CLK_FREQ=%d, freq=%d, tmp=%x \n", RTC_CLK_FREQ, freq, tmp);

	writel(tmp, SOCLE_RTC_CTRL);
	rtc_ctrl_reg = tmp;

	spin_unlock_irq(&socle_rtc_lock);

	return 0;
}

static int
socle_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	RTCDBUG("cmd=%08x, arg=%08lx\n", cmd, arg);
	
	switch (cmd)	{
/*
	case RTC_AIE_OFF:	// Mask alarm int. enab. bit
		RTCDBUG("RTC_AIE_OFF \n");
		rtc_ctrl_reg = rtc_ctrl_reg & ~SOCLE_RTC_CTRL_ALRM_PWON;
		writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);
		return 0;

	case RTC_AIE_ON:		// Allow alarm interrupts
		RTCDBUG("RTC_AIE_ON\n");
		rtc_ctrl_reg = rtc_ctrl_reg | SOCLE_RTC_CTRL_ALRM_PWON;
		writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);
		return 0;
*/
	case RTC_PWR_OFF:	/* Enter RTC Power Off Mode */
		RTCDBUG("RTC_PWR_OFF\n");
		EN_RTC_ALARM_POWER_ON();
		ENTER_RTC_POWER_OFF_MODE();
		return 0;
	}

	return -ENOIOCTLCMD;
}


static int
socle_rtc_proc(struct device *dev, struct seq_file *seq)
{
	u32 rtctalm, rtcdalm;

	spin_lock_irq(&socle_rtc_lock);

	rtctalm = readl(SOCLE_RTC_TALRM);
	rtcdalm = readl(SOCLE_RTC_DALRM);
	
	RTCDBUG("socle_rtc_proc\n");
	
	seq_printf(seq, "time almen \t: %s\n", (rtctalm ) ? "yes" : "no" );
	seq_printf(seq, "date almen \t: %s\n", (rtcdalm ) ? "yes" : "no" );
	//seq_printf(seq, "periodic_freq\t: %ld\n", socle_rtc_freq);

	spin_unlock_irq(&socle_rtc_lock);

	return 0;
}

#if 0
static void
rtc_print_reg(char *str)
{
	int i;
	void __iomem *reg;

	printk("-->    rtc_print_reg(): %s\n", str);
	for (i = 0; i < 7; i++) {
		reg = rtc_regs + i * 4;
		printk("0x%p = 0x%08x\n", reg, readl(reg));
	}
}
#endif

static irqreturn_t
//socle_rtc_irq(int this_irq, void *dev_id) 
socle_rtc_irq(int this_irq, void *rtc) 
{
//cyli add for workaround
//rtc_print_reg("isr");
	rtc_ctrl_reg = readl(SOCLE_RTC_CTRL);
	if (!(rtc_ctrl_reg & SOCLE_RTC_CTRL_ALRM)) {
		printk("socle_rtc_irq(): NOT RTC Alarm!!\n");
		rtc_ctrl_reg = rtc_ctrl_reg & ~SOCLE_RTC_CTRL_ALRM;
		writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);
		return IRQ_NONE;
	}

#ifdef CONFIG_LDK_SCU
	writel(0, LDK_SCU_CLKCFG);
#endif
	rtc_ctrl_reg = rtc_ctrl_reg & ~SOCLE_RTC_CTRL_ALRM;
	writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);

	rtc_update_irq(rtc, 1, RTC_AF | RTC_IRQF);

	printk("RTC Alarm!!\n");

	return IRQ_HANDLED;
}

static int
socle_rtc_open(struct device *dev)
{
	if (rtc_busy)
		return -EBUSY;

	RTCDBUG("RTC Open!\n");

	rtc_busy = 1;

	return 0;
}

static void
socle_rtc_release(struct device *dev)
{
	RTCDBUG("RTC Release!\n");

	rtc_busy = 0;
}

static struct rtc_class_ops socle_rtcops = {
	.open             = socle_rtc_open,
	.release          = socle_rtc_release,
	.ioctl	              = socle_rtc_ioctl,
	.read_time      = socle_rtc_get_time,
	.set_time        = socle_rtc_set_time,
	.read_alarm    = socle_rtc_get_alarm,
	.set_alarm      = socle_rtc_set_alarm,
	.proc              = socle_rtc_proc,
	.irq_set_freq   = socle_rtc_setfreq,
};

#if 0
static void 
socle_rtc_enable(struct platform_device *pdev, int en)
{
	unsigned long reg_date;

	RTCDBUG("%s RTC\n", en ? "Enable" : "Disable");

	if (socle_rtc_base == NULL)
		return;

	spin_lock_irq(&socle_rtc_lock);

	if (!en) {
		reg_date = readl(SOCLE_RTC_DATE);
		rtc_ctrl_reg &= ~SOCLE_RTC_CTRL_EN;
		writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);

	} else {
		/* re-enable the device, and check it is ok */
		reg_date = readl(SOCLE_RTC_DATE);
		rtc_ctrl_reg |= SOCLE_RTC_CTRL_EN;
		writel(rtc_ctrl_reg, SOCLE_RTC_CTRL);
	}

	spin_unlock_irq(&socle_rtc_lock);
}
#endif

static int
socle_rtc_remove(struct platform_device *dev)
{		
	struct rtc_device *rtc = platform_get_drvdata(dev);

	RTCDBUG("irq: %x\n", irq);

	free_irq(irq, dev);
	rtc_device_unregister(rtc);
	platform_set_drvdata(dev, NULL);

	return 0;
}

static int
socle_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	struct resource *res = NULL;
	int ret;
	struct rtc_time tm;
	int check;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "register resources unusable\n");
		ret = -ENXIO;
		goto  free_dev;
	}

	/* request irq resource & check */
	irq = platform_get_irq(pdev, 0);
	RTCDBUG("irq=%d\n", irq);
	if (irq < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = -ENXIO;
		goto free_dev;
	}

	if (!request_mem_region(res->start, (res->end - res->start) + 1, pdev->name)) {
		ret = -EBUSY;
		goto free_dev;
	}

	rtc_regs = ioremap(res->start, (res->end - res->start) + 1);
	if (!rtc_regs) {
		dev_err(&pdev->dev, "cannot map SocleDev registers\n");
		ret = -ENOMEM;
		goto release_mem;
	}

	socle_rtc_base = rtc_regs;
	RTCDBUG("socle_rtc_base=%p\n", socle_rtc_base);


	/*	20080324 leonid+ for pdk-pc7210 RTC initinal	*/
#if defined(CONFIG_ARCH_LDK5) || defined(CONFIG_ARCH_P7DK)  || defined(CONFIG_ARCH_PDK_PC7210) || defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
{
	u32 check_t;
	RESET_RTC_CIRCUIT();

/*	check if time has work	*/
	check_t = readl(SOCLE_RTC_TIME);
	msleep(1);
	//CY T. modify for Warning message 20081224
	if (check_t == readl(SOCLE_RTC_TIME)){
		printk("Resetting RTC counter...\n");
		RESET_RTC_COUNTER();
	}
}

//        if (RTC_IS_PWFAIL) {
//                printk("Warning: RTC Power Fail!!\n");
//                printk("\tPlease change your battery and then reset RTC again...\n");
//        }
#if 0
        while (!RTC_IS_GOOD) {
                printk("Resetting RTC control circuit...\n");
                RESET_RTC_CIRCUIT();
        }
#endif

	if (!RTC_IS_GOOD) {
		printk("Warning: RTC is using perpetual power!!\n");
		return 0;
	}
#endif

	rtc = rtc_device_register(pdev->name, &pdev->dev,
				&socle_rtcops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		goto unmap;
	}

	platform_set_drvdata(pdev, rtc);
	device_init_wakeup(&pdev->dev, 1);

	rtc_ctrl_reg = readl(SOCLE_RTC_CTRL);

	//set freq
	rtc->irq_freq = 16;
	socle_rtc_setfreq(&pdev->dev, rtc->irq_freq);

//	socle_rtc_enable(pdev, 1);

	//Initial time
	tm.tm_wday = 2;
	tm.tm_year = 1970 - 1900;
	tm.tm_mon = 1 - 1;
	tm.tm_mday = 1;
	tm.tm_hour = 0;
	tm.tm_min = 0;
	tm.tm_sec = 0;

	socle_rtc_set_time(&pdev->dev, &tm);

	//wait for set time really
	check = tm.tm_year;
#if !defined(CONFIG_SQ_GDR)	
	//demo board bug....channninglan
	do {
		socle_rtc_get_time(&pdev->dev, &tm);
	} while (check !=  tm.tm_year);
#endif 
	ret = request_irq(irq, socle_rtc_irq,
//			  IRQF_DISABLED,  "socle-rtc", pdev);
			  IRQF_DISABLED,  "socle-rtc", rtc);
	if (ret) {
		printk(KERN_ERR "IRQ%d already in use\n", irq);
		goto unregister;
	}

	return 0;

unregister:
	rtc_device_unregister(rtc);
unmap:
	iounmap(rtc_regs);
release_mem:
	release_mem_region(res->start, (res->end - res->start) + 1);
free_dev:
	
	return ret;
}


#ifdef CONFIG_PM

/* SOCLE RTC Power management control */
static int
socle_rtc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	RTCDBUG("RTC Suspend!\n");

	writel(SOCLE_RTC_PWOFF_PWOFF, SOCLE_RTC_PWOFF);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(platform_get_irq(pdev, 0));
	return 0;
}

static int
socle_rtc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	RTCDBUG("RTC Resume!\n");

	while(!readl(SOCLE_RTC_TIME));

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(platform_get_irq(pdev, 0));
	return 0;
}

static struct dev_pm_ops socle_rtc_dev_pm_ops = {
	.suspend	= socle_rtc_suspend,
	.resume	= socle_rtc_resume,
};

#endif

static struct platform_driver socle_rtcdrv = {
	.probe		= socle_rtc_probe,
	.remove		= socle_rtc_remove,
	.driver		= {
		.name	= "socle-rtc",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm		= &socle_rtc_dev_pm_ops,
#endif
	},
};

static char __initdata banner[] = "SOCLE-SOCLE RTC, (C) 2006 SOCLE Corp.\n";

static int __init
socle_rtc_init(void)
{
	printk(banner);

	return platform_driver_register(&socle_rtcdrv);
}

static void __exit
socle_rtc_exit(void)
{
	platform_driver_unregister(&socle_rtcdrv);
}

module_init(socle_rtc_init);
module_exit(socle_rtc_exit);

MODULE_DESCRIPTION("SOCLE SOCLE RTC Driver");
MODULE_LICENSE("GPL");

