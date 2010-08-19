/********************************************************************************
* File Name     : drivers/input/keyboard/pc9223-keypad.c
* Author        : Jerry Hsieh
* Description   : Socle PC9223 KeyPad Driver
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
* 
*   Version      : 1,0,0,0
*   History      :
*      1. 2010/07/01 jerry create this file
*
********************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>

#include <linux/delay.h>

#include <mach/gpio.h>


//#define CONFIG_PDK_KPD_DEBUG
#ifdef CONFIG_PDK_KPD_DEBUG
	#define DBG(fmt, args...) printk("PC9223_KPD: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

#define KPD_NUM	6

static struct input_dev *pdk_kpd;
static unsigned char *pdkkpd_keycode;
static int kpd_irq;

static void pdk_keypad_input_report_key(struct work_struct *work);
static void pdk_keypad_input_delay_report_key(struct work_struct *work);

static DECLARE_WORK(work, pdk_keypad_input_report_key);
static DECLARE_DELAYED_WORK(dwork, pdk_keypad_input_delay_report_key);

static char __initdata banner[] = "SOCLE PC9223 KeyPad Driver, (c) 2010 SOCLE Corp.\n";

static void
pdk_keypad_report_key(int key_value)
{

	if (key_value < 0) {
		DBG(KERN_ERR "Invalid argument: key_value = %d\n", key_value);
		return;
	}

	input_report_key(pdk_kpd, pdkkpd_keycode[key_value], 1);	//press

	DBG("key_value = %d ; pdkkpd_keycode = %d\n", key_value, pdkkpd_keycode[key_value]);

	input_sync(pdk_kpd);
}


static void
gpio_kpd_init(void)
{
	socle_gpio_claim_lock();

	// normal mode
	socle_gpio_test_mode_en(PN, 0);

	//single low edge trigger
	// PN2, PN3, PN4, PN5, PN6, PN7
	socle_gpio_set_interrupt_sense_with_mask(PN, 0x0, 0xfc);
	socle_gpio_set_interrupt_both_edges_with_mask(PN, 0x0, 0xfc);
	socle_gpio_set_interrupt_event_with_mask(PN, 0x0, 0xfc);

	//set gpio as input
	socle_gpio_get_value_with_mask(PN, 0xfc);

	// enable all interrupt
	socle_gpio_set_interrupt_mask_with_mask(PN, 0xfc, 0xfc);
	
	socle_gpio_release_lock();
}

static int
kpd_value_convert(int val)
{
	int ret;

	switch (val) {
	case 1:
		ret = 0;
		break;
	case 2:
		ret = 1;
		break;
	case 4:
		ret = 2;
		break;
	case 8:
		ret = 3;
		break;
	case 16:
		ret = 4;
		break;
	case 32:
		ret = 5;
		break;
	case 64:
		ret = 6;
		break;
	case 128:
		ret = 7;
		break;
	default:
		DBG("Warning! val = 0x%02x\n", val);
		ret = -1;
	}

	return ret;
}

static int tmp1, tmp_old, key_value;

static void pdk_keypad_input_delay_report_key(struct work_struct *work)
{
	DBG("---start\n");
	
	socle_gpio_claim_lock();

	// read pins (PN2, PN3, PN4, PN5, PN6, PN7)
	tmp1 = socle_gpio_get_value_with_mask(PN, 0xfc);

	socle_gpio_release_lock();

	if (tmp1 == tmp_old)
		schedule_delayed_work (&dwork, HZ / 10);
	else {
		DBG("---Release\n");
		
		input_report_key(pdk_kpd, pdkkpd_keycode[key_value], 0);	//release
		input_sync(pdk_kpd);

		socle_gpio_claim_lock();

		// enable all interrupt (PN2, PN3, PN4, PN5, PN6, PN7)
		socle_gpio_set_interrupt_mask_with_mask(PN, 0xfc, 0xfc);

		socle_gpio_release_lock();
	}
}

static void pdk_keypad_input_report_key(struct work_struct *work)
{

	DBG("---start\n");
	socle_gpio_claim_lock();

	// read pins (PN2, PN3, PN4, PN5, PN6, PN7)
	tmp1 = socle_gpio_get_value_with_mask(PN, 0xfc);

	tmp_old = tmp1;
		
	socle_gpio_release_lock();

	DBG("tmp1 = 0x%x\n", tmp1);

	key_value = kpd_value_convert((~tmp1) & 0xfc) - 2;

	DBG("key_value= %d\n", key_value);

	pdk_keypad_report_key(key_value);;

	schedule_delayed_work (&dwork, HZ / 10);
	
}

static irqreturn_t
pdk_keypad_isr(int irq, void *dev)
{
	DBG("pdk_keypad_isr\n");

	// read row pins
	tmp1 = socle_gpio_get_interrupt_status_with_port(PN);

	DBG("tmp1 = 0x%x\n", tmp1);
	
	if (0xfc & tmp1) {
		// disable all interrupt (PN2, PN3, PN4, PN5, PN6, PN7)
		socle_gpio_set_interrupt_mask_with_mask(PN, 0x0, 0xfc);
	} else {
		return IRQ_HANDLED;
	}

	schedule_work(&work);

	return IRQ_HANDLED;
}

static int __devinit
pdk_keypad_probe(struct platform_device *pdev)
{
	int ret, i;

	pdkkpd_keycode = (unsigned char *)pdev->dev.platform_data;
	if (!pdkkpd_keycode) {
		printk(KERN_ERR "Unable to find keycode\n");
		return -ENOMEM;
	}

	pdk_kpd = input_allocate_device();
	if (!pdk_kpd) {
		printk(KERN_ERR "Unable to allocate input device\n");
		return -ENOMEM;
	}

	pdk_kpd->evbit[0] = BIT(EV_KEY);
	pdk_kpd->name = "PC9223 KeyPad";
	pdk_kpd->phys = "pdkkpd/input0";
	pdk_kpd->id.bustype = BUS_HOST;
	pdk_kpd->id.vendor = 0x0001;
	pdk_kpd->id.product = 0x0001;
	pdk_kpd->id.version = 0x0001;

	for (i = 0; i < KPD_NUM; i++) {
		set_bit(pdkkpd_keycode[i], pdk_kpd->keybit);
		DBG("pdkkpd_keycode[%d] = %d\n", i, pdkkpd_keycode[i]);
	}

	gpio_kpd_init();

	kpd_irq = socle_gpio_get_irq(PN);
	if (kpd_irq < 0) {
		printk(KERN_ERR "Unable to get IRQ(%d)\n", kpd_irq);
		ret = -ENOENT;
		goto err_no_irq;
	}

	ret = request_irq(kpd_irq, pdk_keypad_isr, IRQF_SHARED, pdk_kpd->name, pdk_kpd);
	if (ret) {
		printk(KERN_ERR "Unable to claim IRQ(%d)\n", kpd_irq);
		goto err_no_irq;
	}

	ret = input_register_device(pdk_kpd);
	if (ret) {
		printk(KERN_ERR "Unable to register %s input device\n", pdk_kpd->name);
		goto err_reg_dev;
	}

	return ret;

err_reg_dev:
	free_irq(kpd_irq, NULL);
err_no_irq:
	input_free_device(pdk_kpd);
	return ret;
}

static int __devexit
pdk_keypad_remove(struct platform_device *pdev)
{
	DBG("\n");

	free_irq(kpd_irq, pdk_kpd);
	flush_scheduled_work();
	input_unregister_device(pdk_kpd);

	return 0;
}

static struct platform_driver pdk_keypad_device_driver = {
	.probe		= pdk_keypad_probe,
	.remove		= __devexit_p(pdk_keypad_remove),
	.driver		= {
		.name	= "pc9223-keypad",
	}
};

static int __init
pdk_keypad_init(void)
{
	printk(banner);
	return platform_driver_register(&pdk_keypad_device_driver);
}

static void __exit
pdk_keypad_exit(void)
{
	platform_driver_unregister(&pdk_keypad_device_driver);
}

module_init(pdk_keypad_init);
module_exit(pdk_keypad_exit);


MODULE_DESCRIPTION("SOCLE PC9223 KeyPad Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jerry Hsieh");
