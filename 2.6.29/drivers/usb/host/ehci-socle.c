/*
 * File Name     : drivers/usb/host/ehci-socle.c 
 * Author         : ryan chen
 * Description   : EHCI HCD (Host Controller Driver) for USB
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
 *      1. 2006/09/11 ryan chen create this file 
*/


#include <linux/platform_device.h>

#include <mach/hardware.h>




#ifdef CONFIG_SOCLE_EHCI_PERFORMANCE_MEASURE		//leonid+ host performance
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#endif

extern int usb_disabled(void);
/* SOCLE EHCI USB Host Controller */

#ifdef CONFIG_SOCLE_EHCI_PERFORMANCE_MEASURE		//leonid+ host performance

static u32 read_rate;

static void*
socle_otg_seq_start(struct seq_file *s, loff_t *pos)
{
     if (*pos > 0)
	  return NULL;
     else
	  return &read_rate;
}

static void*
socle_otg_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
     (*pos)++;
     return NULL;
}

static void 
socle_otg_seq_stop(struct seq_file *s, void *v)
{
     /* Actually, there's nothing to do here */
}

unsigned long tran_timers=0;
unsigned long tran_jiffies=0;
unsigned long tran_bytes=0;
unsigned long otg_timer_load=0;

static int socle_otg_seq_show(struct seq_file *s, void *v)
{	
	struct timespec ts;
	u32 m_sec;
	  u32 tran_speed_rate;

	jiffies_to_timespec(1, &ts);
	  m_sec = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;


	printk("tran_jiffies = %ld\n", tran_jiffies);
	printk("tran_timers = %ld\n", tran_timers);

	m_sec = (m_sec * tran_jiffies) + (m_sec * tran_timers / otg_timer_load);

	  
	  seq_printf(s, "Total time for tran: %10d ms\n", m_sec);
	  seq_printf(s, "Total bytes for tran: %10ld Bytes\n", tran_bytes);
	  if (0 == m_sec)
	       tran_speed_rate = 0;
	  else
	       tran_speed_rate = tran_bytes / m_sec;
	  seq_printf(s, "Speed rate of tran: %10d B/ms\n", tran_speed_rate);

	tran_timers=0;
	tran_jiffies=0;
	tran_bytes=0;	  
	
	return 0;  
}	

/*
 *  Tie the sequence operators up.
 *  */
static struct seq_operations socle_otg_seq_ops = {
     .start = socle_otg_seq_start,
     .next = socle_otg_seq_next,
     .stop = socle_otg_seq_stop,
     .show = socle_otg_seq_show
};

/*
 *  Now to implement the /proc file we need only make an open
 *  method which sets up the sequence operators.
 *  */
static int socle_otg_proc_open(struct inode *inode, struct file *file)
{
     return seq_open(file, &socle_otg_seq_ops);
}

/*
 *  Create a set of file operations for our proc file.
 *  */
static struct file_operations socle_otg_proc_ops = {
     .owner = THIS_MODULE,
     .open = socle_otg_proc_open,
     .read = seq_read,
     //.write = socle_otg_write,
     .llseek = seq_lseek,
     .release = seq_release
};

static struct proc_dir_entry *socle_otg_proc_entry;
#endif

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_ehci_socle_probe - initialize socle-based HCDs
 * Context: 
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int usb_hcd_ehci_socle_probe(const struct hc_driver *driver,
		      struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	//struct ehci_hcd *ehci;
	struct resource *res;
	int irq;
	int retval;
#ifdef CONFIG_SOCLE_EHCI_OTG
	unsigned int temp;
#endif

	pr_debug("initializing SOCLE USB Controller\n");

#ifdef CONFIG_SOCLE_EHCI_PERFORMANCE_MEASURE		//leonid+ host performance
	otg_timer_load = ioread32(IO_ADDRESS(SOCLE_TIMER0) + 0x10);
		printk("platform device NULL \n");

     /* Install the proc_fs entry */
     socle_otg_proc_entry = create_proc_entry("socle_otg", 
					       S_IRUGO | S_IFREG,
					       &proc_root);
     if (socle_otg_proc_entry) {
	  socle_otg_proc_entry->proc_fops = &socle_otg_proc_ops;
	  socle_otg_proc_entry->data = NULL;
     } else
	  return -ENOMEM;
#endif

	/*
	 * This is a host mode driver, verify that we're supposed to be
	 * in host mode.
	 */

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto err2;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto err2;
	}
	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto err3;
	}
#if 0
	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

#ifdef CONFIG_USB_EHCI_ROOT_HUB_TT
	ehci_to_hcd(ehci)->has_tt=1;	//leonid + 
#endif
#endif
#ifdef CONFIG_SOCLE_EHCI_OTG
	/* Set to Host mode */
	temp = readl(hcd->regs + 0x1a8);
	writel(temp | 0x3, hcd->regs + 0x1a8);
#endif

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0){
		printk("usb_add_hcd fail\n");
		goto err4;
	}
	return retval;

      err4:
	iounmap(hcd->regs);
      err3:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
      err2:
	usb_put_hcd(hcd);
      err1:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_socle_remove - shutdown processing for SOCLE-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_ehci_socle_probe().
 *
 */
void usb_hcd_socle_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
#ifndef CONFIG_SOCLE_USB_OTG
	iounmap(hcd->regs);
#endif
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}

#if 1
static void socle_usb_setup(struct usb_hcd *hcd)
{
#ifdef CONFIG_SOCLE_EHCI_OTG
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	void __iomem *non_ehci = hcd->regs;

	/* put controller in host mode. */
	ehci_writel(ehci, 0x00000003, non_ehci + 0x1a8);
#endif
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_socle_reinit(struct ehci_hcd *ehci)
{
	socle_usb_setup(ehci_to_hcd(ehci));
	ehci_port_power(ehci, 0);

	return 0;
}
#endif

/* called during probe() after chip reset completes */
static int ehci_socle_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs +
	    HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

#ifdef CONFIG_USB_EHCI_ROOT_HUB_TT
	hcd->has_tt = 1;
#endif

	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	retval = ehci_socle_reinit(ehci);
	return retval;
}

static const struct hc_driver ehci_socle_hc_driver = {
	.description = hcd_name,
	.product_desc = "SOCLE On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_socle_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,

};

static int ehci_socle_drv_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	/* FIXME we only want one one probe() not two */
	return usb_hcd_ehci_socle_probe(&ehci_socle_hc_driver, pdev);
}

static int ehci_socle_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	/* FIXME we only want one one remove() not two */
	usb_hcd_socle_remove(hcd, pdev);
	return 0;
}

 /*TBD*/
#ifdef CONFIG_PM
static int ehci_hcd_socle_drv_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);
	unsigned long		flags;
	int			rc = 0;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave (&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
 bail:
	spin_unlock_irqrestore (&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	return rc;
}
static int ehci_hcd_socle_drv_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd		*ehci = hcd_to_ehci(hcd);

	// maybe restore FLADJ

	if (time_before(jiffies, ehci->next_statechange))
		msleep(100);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	usb_root_hub_lost_power(hcd->self.root_hub);

	/* Else reset, to cope with power loss or flush-to-storage
	 * style "resume" having let BIOS kick in during reboot.
	 */
	(void) ehci_halt(ehci);
	(void) ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;
	return 0;
}
#endif

MODULE_ALIAS("platform:socle-ehci");

static struct platform_driver ehci_hcd_socle_driver = {
	.probe = ehci_socle_drv_probe,
	.remove = ehci_socle_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
#ifdef CONFIG_PM	
	.suspend      	= ehci_hcd_socle_drv_suspend,
	.resume       	= ehci_hcd_socle_drv_resume,
#endif	
	.driver = {
		.name	= "SOCLE_EHCI",
	},
};
