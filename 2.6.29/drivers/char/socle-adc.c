/********************************************************************************
* File Name     : drivers/char/cdk-adc.c 
* Author         : ryan chen
* Description   : Socle  CDK ADC Controller Driver (ADC)
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
    
*   Version      : a.0
*   History      : 
*      1. 2007/02/26 ryan chen create this file 
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/cdev.h>

#include <asm/hardware.h>
#include <asm/uaccess.h>


#include <asm/io.h>
#include <asm/arch/irqs.h>
#include <asm/arch/platform.h>
#include <asm/arch/regs-adc.h>

#define ADC_MAJOR 0
#define ADC_MINOR 0 

int adc_major = ADC_MAJOR;
int adc_minor = ADC_MINOR;

static struct class *adc_class;
static struct class_device *adc_class_dev;
struct cdev *adc_cdev;

//#define SOCLE_ADC_DEBUG
#ifdef SOCLE_ADC_DEBUG
#define ADCDBUG(fmt, args...) printk( KERN_DEBUG "SOCLE_ADC : " fmt, ## args)
#else
#define ADCDBUG(fmt, args...) \
	do { } while (0)
#endif

static int adc_busy = 0;

void __iomem            *regs;

static int socle_adc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	ADCDBUG("ADC_IOCTL cmd=%x arg=%ld \n", cmd, arg);

	switch (cmd)
	{
	case ADC_IOC_SET_PWM:
		ADCDBUG("Set ADC Power\n");
		writew(readw(SOCLE_ADC_CTRL) | ADC_CTRL_PWM , SOCLE_ADC_CTRL);
		return 0;
	case ADC_IOC_GET_PWM:
		ADCDBUG("Get ADC Power\n");
		arg = ((readw(SOCLE_ADC_CTRL) & (~ADC_CTRL_PWM)) >> 3);
		return 0;
	case ADC_IOC_SET_CHANNEL:
		ADCDBUG("SET ADC Channel \n");
		if (arg > 7)
			return -EINVAL;
		writew(arg |((~0x7) & readw(ADC_REG_BASE)),ADC_REG_BASE);
		return 0;
	case ADC_IOC_GET_CHANNEL:
		ADCDBUG("GET ADC Channel \n");
		if (arg > 7)
			return -EINVAL;
		arg = 0x7 & readw(ADC_REG_BASE);	
		return 0;
	
	}
	return -EINVAL;
}
#if 0
static irqreturn_t socle_adc_irq(int this_irq, struct pt_regs *regs) 
{
	//SCUDBUG("ADC irq stats=%x ctrl=%x\n", stats, ctrl);

	//clear intr
	writew(readw(SOCLE_ADC_CTRL) | ADC_CTRL_INTR_CLR, SOCLE_ADC_CTRL);
	
	if(readw(SOCLE_ADC_STAS))
	{
		printk("ADC Error !! \n");
		return IRQ_HANDLED;
	}
	
	return IRQ_HANDLED;
}
#endif
static int socle_adc_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	u32 adc_value;

	ADCDBUG("socle adc read\n");

	writew(readw(SOCLE_ADC_CTRL) | ADC_CTRL_SOC, SOCLE_ADC_CTRL);
	
	//polling 
	while (readw(SOCLE_ADC_STAS));
	
	adc_value = readw(SOCLE_ADC_DATA);
	*buf = adc_value;	
	copy_to_user(buf, &adc_value, sizeof(adc_value));
	ADCDBUG("adc value : %x \n",adc_value);


	return count;

}

static int socle_adc_open(struct inode *inode, struct file *file)
{
	if (adc_busy)
		return -EBUSY;

	ADCDBUG("adc open\n");

	adc_busy = 1;

	return 0;
	
}

static int socle_adc_release(struct inode *inode, struct file *file)
{
	ADCDBUG("socle adc release\n");
	adc_busy = 0;
	return 0;
}

static const struct file_operations adc_fops = {
        .owner =        THIS_MODULE,
        .llseek =       no_llseek,
        .ioctl =        socle_adc_ioctl,
        .open =         socle_adc_open,
        .read = 		socle_adc_read,
        .release =      socle_adc_release,
};

static int socle_adc_remove(struct platform_device *pdev)
{
	/* do not clear AIE here, it may be needed for wake */
	free_irq(IRQ_ADC0, pdev);

//20070515 cyli add
	class_device_unregister(adc_class_dev);
	cdev_del(adc_cdev);
	class_destroy(adc_class);
	unregister_chrdev_region(MKDEV(adc_major, adc_minor), 1);

	return 0;
}


static int socle_adc_probe(struct platform_device *pdev)
{
	int ret,err,result;
	struct resource *res = NULL;
	
	dev_t dev = MKDEV(adc_major, adc_minor);

	/*
	 * Register your major, and accept a dynamic number.
	 */
	if (ADC_MAJOR)
		result = register_chrdev_region(dev, 1, "socle-adc");
	else {
		result = alloc_chrdev_region(&dev, adc_minor, 1, "socle-adc");
		adc_major = MAJOR(dev);
		adc_minor = MINOR(dev);
	}
	if (result < 0)
		return result;

	adc_cdev = cdev_alloc();

	adc_cdev->ops = &adc_fops;
	adc_cdev->owner = THIS_MODULE;

	err = cdev_add (adc_cdev, dev, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "ADC Cdev Error %d adding ", err );

	
	adc_class = class_create(THIS_MODULE, "socle-adc");
	if (IS_ERR(adc_class)) {
		ret = PTR_ERR(adc_class);
	}
	
	adc_class_dev = class_device_create(adc_class, NULL, dev, NULL, "socle-adc");
        if (IS_ERR(adc_class_dev)) {
		ret = PTR_ERR(adc_class_dev);
        }

	/* request register map resource & check */
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res) {
                dev_err(&pdev->dev, "register resources unusable\n");
                ret = -ENXIO;
        }

	if (!request_mem_region(res->start, (res->end - res->start) + 1,
                                pdev->name)) {
                ret = -EBUSY;
        }
	
	regs = ioremap(res->start, (res->end - res->start) + 1);
	if (!regs) {
                dev_err(&pdev->dev, "cannot map SocleDev registers\n");
		ret = -ENOMEM;
                goto release_mem;
        }
	
	//power up ADC
	writew(ADC_CTRL_PWM |readw(ADC_REG_BASE),ADC_REG_BASE);
	//set Channel 0
	writew((~0x7) & readw(ADC_REG_BASE),ADC_REG_BASE);	
	// Sart to conversion
	writew(ADC_CTRL_SOC | readw(ADC_REG_BASE),ADC_REG_BASE);	
	//polling 
	while (readw(SOCLE_ADC_STAS));
	
//	printk("ADC value = %x \n",readw(SOCLE_ADC_DATA));


	return 0;

release_mem:
	release_mem_region(res->start, (res->end - res->start) + 1);


}

/* LDK SCU Power management control */
static struct platform_driver socle_adcdrv = {
	.probe		= socle_adc_probe,
	.remove		= socle_adc_remove,
	.driver		= {
		.name	= "socle-adc",
		.owner	= THIS_MODULE,
	},
};

static char __initdata banner[] = "SOCLE ADC, (c) 2007 SOCLE Corp. \n";

static int __init socle_adc_init(void)
{
	printk(banner);

	return platform_driver_register(&socle_adcdrv);
}

static void __exit socle_adc_exit(void)
{
	platform_driver_unregister(&socle_adcdrv);
}

module_init(socle_adc_init);
module_exit(socle_adc_exit);

MODULE_DESCRIPTION("SOCLE ADC Driver");
MODULE_LICENSE("GPL");

