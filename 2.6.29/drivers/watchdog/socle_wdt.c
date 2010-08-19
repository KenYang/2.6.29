/********************************************************************************
* File Name     : drivers/char/watchdog/socle_wdt.c 
* Author         : ryan chen
* Description   : Socle Watch Dog Timer (WDT)
* 
* Copyright (C) Socle Tech. Corp.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
*without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
    
*   Version      : 2,0,0,1
*   History      : 
*      1. 2006/09/03 ryan chen create this file 
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
//#include <linux/clk.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/platform.h>
#include <mach/regs-wdt.h>

#if (defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_PDK_PC9002) || defined(CONFIG_ARCH_SCDK))
#include <mach/cheetah-scu.h>
#endif

#include <mach/socle-scu.h>

#define PFX "socle-wdt: "

#define CONFIG_SOCLE_WATCHDOG_ATBOOT		(0)
#define CONFIG_SOCLE_WATCHDOG_DEFAULT_TIME	(15)

static int nowayout = WATCHDOG_NOWAYOUT;
static int tmr_margin	= CONFIG_SOCLE_WATCHDOG_DEFAULT_TIME;
static int tmr_atboot	= CONFIG_SOCLE_WATCHDOG_ATBOOT;
static int soft_noboot	= 0;
static int debug	= 0;

module_param(tmr_margin,  int, 0);
module_param(tmr_atboot,  int, 0);
module_param(nowayout,    int, 0);
module_param(soft_noboot, int, 0);
module_param(debug,	  int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. default=" __MODULE_STRING(CONFIG_SOCLE_WATCHDOG_DEFAULT_TIME) ")");

MODULE_PARM_DESC(tmr_atboot, "Watchdog is started at boot time if set to 1, default=" __MODULE_STRING(CONFIG_SOCLE_WATCHDOG_ATBOOT));

MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, 0 to reboot (default depends on ONLY_TESTING)");

MODULE_PARM_DESC(debug, "Watchdog debug, set to >1 for debug, (default 0)");


typedef enum close_state {
	CLOSE_STATE_NOT,
	CLOSE_STATE_ALLOW=0x4021
} close_state_t;

static DECLARE_MUTEX(open_lock);

//static struct resource	*wdt_mem;

static void __iomem	*wdt_base;
static close_state_t	 allow_close;

static unsigned long pre_scale; //modify from reg_t --> unsigned long

/* watchdog control routines */
//#define SOCLE_WDT_DEBUG

#ifdef SOCLE_WDT_DEBUG
#define WDTDBUG(fmt, args...) printk( KERN_DEBUG "SOCLE_WDT : " fmt, ## args)
#else
#define WDTDBUG(fmt, args...) \
	do { } while (0)
#endif

static int socle_wdt_keepalive(void)
{
	writel(tmr_margin, SOCLE_WTDOG_LR);
	WDTDBUG("socle_wdt_keepalive margin = %x\n",tmr_margin);
	return 0;
}

static int socle_wdt_stop(void)
{
	WDTDBUG("socle_wdt_stop \n");
	writel(readl(SOCLE_WTDOG_CON) & ~(WTDOG_RESET_ENABLE | WTDOG_ENABLE), SOCLE_WTDOG_CON);

	return 0;
}

static int socle_wdt_start(void)
{
	WDTDBUG("socle_wdt_start \n");

	socle_wdt_stop();

	if (soft_noboot) {
		writel(WTDOG_ENABLE | pre_scale, SOCLE_WTDOG_CON);
	} else {
		writel(WTDOG_RESET_ENABLE | WTDOG_ENABLE | pre_scale, SOCLE_WTDOG_CON);
	}
	
	WDTDBUG("%s: tmr_margin=0x%08x, pre_scale=%08lx\n",
	    __FUNCTION__, tmr_margin, pre_scale);


	return 0;
}

static int socle_wdt_set_heartbeat(int timeout)
{
	unsigned long apb_clock_freq;
	u32 prescale;	

	if (timeout < 1)
		return -EINVAL;

	apb_clock_freq = socle_scu_apb_clock_get();
	//printk("apb_clock %lx \n",apb_clock_freq);

	for (prescale=0; prescale <= MAX_PRESCALE_SHIFT; prescale++) {
		u32 max_period;

		max_period = SOCLE_WTDOG_MAX_LR / (apb_clock_freq >> prescale);
		if (max_period >= timeout)
			break;
		
	}

	if (prescale > MAX_PRESCALE_SHIFT) {
		/* Period cannot be supported by H/W. */
		return (-1);
	}

	/* skip unimplemented prescale factor*/
	if (prescale > SOCLE_WTDOG_DISALLOWED_PRESCALE)
		pre_scale = prescale-1;
	else
		pre_scale = prescale;

	tmr_margin = timeout * (apb_clock_freq >> prescale);
	
	return 0;

}

/*
 *	/dev/watchdog handling
 */

static int socle_wdt_open(struct inode *inode, struct file *file)
{
	WDTDBUG(" socle_wdt_open\n");

	if(down_trylock(&open_lock))
		return -EBUSY;

	if (nowayout) {
		__module_get(THIS_MODULE);
	} else {
		allow_close = CLOSE_STATE_ALLOW;
	}

	/* start the timer */
	socle_wdt_start();
	return nonseekable_open(inode, file);
}

static int socle_wdt_release(struct inode *inode, struct file *file)
{
	WDTDBUG(" socle_wdt_release\n");
	/*
	 *	Shut off the timer.
	 * 	Lock it in if it's a module and we set nowayout
	 */
	if (allow_close == CLOSE_STATE_ALLOW) {
		socle_wdt_stop();
	} else {
		WDTDBUG(KERN_CRIT PFX "Unexpected close, not stopping watchdog!\n");
		socle_wdt_keepalive();
	}

	allow_close = CLOSE_STATE_NOT;
	up(&open_lock);
	return 0;
}

static ssize_t socle_wdt_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	WDTDBUG(" socle_wdt_write\n");
	/*
	 *	Refresh the timer.
	 */
	if(len) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			allow_close = CLOSE_STATE_NOT;

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					allow_close = CLOSE_STATE_ALLOW;
			}
		}

		socle_wdt_keepalive();
	}
	return len;
}

#define OPTIONS WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE

static struct watchdog_info socle_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"SOCLE Watchdog",
};


static int socle_wdt_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_margin;

	switch (cmd) {
		default:
			return -ENOIOCTLCMD;

		case WDIOC_GETSUPPORT:
			return copy_to_user(argp, &socle_wdt_ident,
				sizeof(socle_wdt_ident)) ? -EFAULT : 0;

		case WDIOC_GETSTATUS:
		case WDIOC_GETBOOTSTATUS:
			return put_user(0, p);

		case WDIOC_KEEPALIVE:
			socle_wdt_keepalive();
			return 0;

		case WDIOC_SETTIMEOUT:
			if (get_user(new_margin, p))
				return -EFAULT;

			printk("new_margin = %d \n",new_margin);
			if (socle_wdt_set_heartbeat(new_margin))
				return -EINVAL;

			socle_wdt_keepalive();

			return put_user(tmr_margin, p);

		case WDIOC_GETTIMEOUT:
			return put_user(tmr_margin, p);

		// 20081103 cyli add
		case WDIOC_SETOPTIONS: {
			if (arg & WDIOS_DISABLECARD) {
				socle_wdt_stop();
				return 0;
			}
			if (arg & WDIOS_ENABLECARD) {
				socle_wdt_start();
				return 0;
			}

			return -EFAULT;
		}
	}
}

/* kernel interface */

static struct file_operations socle_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= socle_wdt_write,
	.ioctl		= socle_wdt_ioctl,
	.open		= socle_wdt_open,
	.release	= socle_wdt_release,
};

static struct miscdevice socle_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &socle_wdt_fops,
};

static int socle_wdt_probe(struct platform_device *pdev)
{
//	struct resource *res;
	int started = 0;
	int ret;
//	int size;

	WDTDBUG("%s: probe=%p\n", __FUNCTION__, pdev);

	/* get the memory region for the watchdog timer */
/*
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_INFO PFX "failed to get memory region resouce\n");
		return -ENOENT;
	}

	size = (res->end-res->start)+1;
	wdt_mem = request_mem_region(res->start, size, pdev->name);
	if (wdt_mem == NULL) {
		printk(KERN_INFO PFX "failed to get memory region\n");
		return -ENOENT;
	}

*/	
	wdt_base = (void *) SOCLE_WDT_BASE;
	if (wdt_base == 0) {
		printk(KERN_INFO PFX "failed to ioremap() region\n");
		return -EINVAL;
	}

	WDTDBUG("probe: mapped wdt_base=%p\n", wdt_base);
/*
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		printk(KERN_INFO PFX "failed to get irq resource\n");
		return -ENOENT;
	}


	wdt_clock = clk_get(&pdev->dev, "watchdog");
	if (wdt_clock == NULL) {
		printk(KERN_INFO PFX "failed to find watchdog clock source\n");
		return -ENOENT;
	}

	clk_enable(wdt_clock);
*/
	/* see if we can actually set the requested timer margin, and if
	 * not, try the default value */
#if (defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_PDK_PC9002) || defined(CONFIG_ARCH_SCDK))
	//init WDT Reset
	socle_scu_arm926ej_reset_wdt_reset_enable();
#endif

	if (socle_wdt_set_heartbeat(tmr_margin)) {
		started = socle_wdt_set_heartbeat(CONFIG_SOCLE_WATCHDOG_DEFAULT_TIME);

		if (started == 0) {
			printk(KERN_INFO PFX "tmr_margin value out of range, default %d used\n",
			       CONFIG_SOCLE_WATCHDOG_DEFAULT_TIME);
		} else {
			printk(KERN_INFO PFX "default timer value is out of range, cannot start\n");
		}
	}

	ret = misc_register(&socle_wdt_miscdev);
	if (ret) {
		printk (KERN_ERR PFX "cannot register miscdev on minor=%d (%d)\n",
			WATCHDOG_MINOR, ret);
		return ret;
	}

	if (tmr_atboot && started == 0) {
		printk(KERN_INFO PFX "Starting Watchdog Timer\n");
		socle_wdt_start();
	} else if (!tmr_atboot) {
		/* if we're not enabling the watchdog, then ensure it is
		 * disabled if it has been left running from the bootloader
		 * or other source */

		socle_wdt_stop();
	}

	return 0;
}

static int socle_wdt_remove(struct platform_device *dev)
{
/*
	if (wdt_mem != NULL) {
		release_resource(wdt_mem);
		kfree(wdt_mem);
		wdt_mem = NULL;
	}


	if (wdt_clock != NULL) {
		clk_disable(wdt_clock);
		clk_put(wdt_clock);
		wdt_clock = NULL;
	}
*/
	misc_deregister(&socle_wdt_miscdev);
	return 0;
}

static void socle_wdt_shutdown(struct platform_device *dev)
{
	socle_wdt_stop();	
}

#ifdef CONFIG_PM

static unsigned long wtcon_save;
static unsigned long wtdat_save;

static int socle_wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	pr_debug("leonid : socle_wdt_suspend \n");
	/* Save watchdog state, and turn it off. */
	
	wtcon_save = readl(SOCLE_WTDOG_CON);
	wtdat_save = readl(SOCLE_WTDOG_LR);

	/* Note that WTCNT doesn't need to be saved. */
	socle_wdt_stop();

	return 0;
}

static int socle_wdt_resume(struct platform_device *dev)
{
	/* Restore watchdog state. */
	pr_debug("leonid : socle_wdt_resume \n");

	writel(wtdat_save, SOCLE_WTDOG_LR);
	writel(wtcon_save, SOCLE_WTDOG_CON);

	return 0;
}

#else
#define socle_wdt_suspend NULL
#define socle_wdt_resume  NULL
#endif /* CONFIG_PM */


static struct platform_driver socle_wdt_driver = {
	.probe		= socle_wdt_probe,
	.remove		= socle_wdt_remove,
	.shutdown	= socle_wdt_shutdown,
	.suspend		= socle_wdt_suspend,
	.resume		= socle_wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "socle-wdt",
	},
};


static char banner[] __initdata = KERN_INFO "SOCLE Watchdog Timer, (c) 2006 SOCLE Corp.\n";

static int __init watchdog_init(void)
{
	printk(banner);

	return platform_driver_register(&socle_wdt_driver);
}

static void __exit watchdog_exit(void)
{
	platform_driver_unregister(&socle_wdt_driver);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Ryan Chen <ryanchen@socle-tech.com.tw>");
MODULE_DESCRIPTION("SOCLE Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
