#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>			/* kmalloc() */
#include <linux/interrupt.h>
#include <linux/miscdevice.h>

#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/wait.h>			//use to sleep task
#include <asm/uaccess.h>  		/* put_user() */ 
#include <asm/io.h>			/* phy_to_bus() swab() */
#include <linux/leds.h>
#include <linux/platform_device.h>

#include <mach/regs-pwmt.h>
#include <mach/socle-scu.h>
#include "socle-backlight.h"
#include <linux/fb.h>

//#define CONFIG_BACKLIGHT_DEBUG
#ifdef CONFIG_BACKLIGHT_DEBUG
	#define DBG(fmt, args...) printk("backlight: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

static char __initdata banner[] = "SOCLE BACKLIGHT, (c) 2009 SOCLE Corp.\n";

struct socle_backlight {
	struct led_classdev	cdev;
	struct socle_pwmt *p_pwm;
	enum lcd_type {
		panel_48 = 0,
		panel_70,
		panel_7a
	}lcd;
	int lrc_value;
	int hrc_value;
};

static int
hrc_value_caculate(enum lcd_type lcd, int lrc_value, int brightness)
{
	int hrc_max_value = 0;
	int hrc_value;
	
	if (lcd == panel_48) {
		DBG("panel type is panel_48\n");
		if (brightness < 200)
			hrc_max_value = 80 * lrc_value/100;
		else
			hrc_max_value = lrc_value;
	} else if (lcd == panel_7a)  {
		DBG("panel type is panel_7a\n");
		hrc_max_value = lrc_value;
	} else if (lcd == panel_70) {
		DBG("panel type is panel_70\n");
		if (brightness < 170)
			hrc_max_value = 40 * lrc_value/100;
		else if (brightness >= 170 && brightness < 200)
			hrc_max_value = 60 * lrc_value/100;
		else
			hrc_max_value = lrc_value;
	}

	DBG("hrc_max_value = %d\n", hrc_max_value);

	hrc_value = (brightness * hrc_max_value)/255;
	
	return hrc_value;

}

static void
socle_backlight_init_by_pwm(struct socle_backlight *socle_bl)
{
	struct socle_pwmt_driver *pwm_drv = socle_bl->p_pwm->drv;

	int clk_apb;
	int lrc_value;
	int hrc_value;
	int lcd_pwm_freq;
	
	clk_apb = socle_scu_apb_clock_get();
	DBG("clk_apb = %d\n", clk_apb);

	if (socle_bl->lcd == panel_48) {
		lcd_pwm_freq = 70000;
	} else if (socle_bl->lcd == panel_7a)  {
		lcd_pwm_freq = 20000;
	} else if (socle_bl->lcd == panel_70) {
		lcd_pwm_freq = 70000;
	}
	
	lrc_value = clk_apb / lcd_pwm_freq;

	socle_bl->lrc_value = lrc_value;

	DBG("lrc_value = %d\n", lrc_value);

	DBG("socle_bl->cdev.brightness = %d\n", socle_bl->cdev.brightness);

	hrc_value = hrc_value_caculate(socle_bl->lcd, lrc_value, socle_bl->cdev.brightness);
	socle_bl->hrc_value = hrc_value;

	DBG("hrc_value = %d\n", hrc_value);

	pwm_drv->claim_pwm_lock();

	pwm_drv->reset(socle_bl->p_pwm);
	pwm_drv->write_prescale_factor(socle_bl->p_pwm, DEFAULT_PRE_SCL);
	pwm_drv->write_hrc(socle_bl->p_pwm, hrc_value);
	pwm_drv->write_lrc(socle_bl->p_pwm, lrc_value);
	pwm_drv->output_enable(socle_bl->p_pwm, 1);
	pwm_drv->enable(socle_bl->p_pwm, 1);

	pwm_drv->release_pwm_lock();

}

static void
socle_backlight_set_brightness(struct led_classdev *cdev, enum led_brightness b)
{
	struct socle_pwmt_driver *pwm_drv;
	struct socle_backlight *socle_bl;
	int lrc_value;
	int hrc_value;

	socle_bl = container_of(cdev, struct socle_backlight, cdev);
	
	lrc_value = socle_bl->lrc_value;

	DBG("lrc_value = %d\n", lrc_value);

	DBG("socle_bl->cdev.brightness = %d\n", b);

	hrc_value = hrc_value_caculate(socle_bl->lcd, lrc_value, b);
	socle_bl->hrc_value = hrc_value;

	DBG("hrc_value = %d\n", hrc_value);

	pwm_drv= socle_bl->p_pwm->drv;
	
	pwm_drv->claim_pwm_lock();
	pwm_drv->write_hrc(socle_bl->p_pwm, hrc_value);
	pwm_drv->release_pwm_lock();
	
}

static int 
socle_backlight_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct socle_backlight *socle_bl;
	char *options = NULL;
	char *this_opt;
	char tmp[128];
	char *tmp_opt;
	
	socle_bl = kzalloc(sizeof(struct socle_backlight), GFP_KERNEL);
	if (NULL == socle_bl) {
		printk("error : socle_bl strcuturecan't kmalloc\n");
		ret = -ENOMEM;
		goto out_return;
	}

	platform_set_drvdata(pdev, socle_bl);

	socle_bl->p_pwm = get_socle_pwmt_structure(USE_PWM_NUM);
	if (NULL == socle_bl->p_pwm) {
		printk("Socle Backlight can't get PWMT structure!!\n");
		ret = -ENXIO;
		goto out_kree;
	}
	
	socle_bl->cdev.name = "lcd-backlight";		//for android backlight support
	socle_bl->cdev.brightness = LED_HALF;
	socle_bl->cdev.brightness_set = socle_backlight_set_brightness;
	socle_bl->cdev.default_trigger = "backlight";

	ret = led_classdev_register(&pdev->dev, &socle_bl->cdev);
	if (ret < 0) {
		goto out_release_socle_pwmt_structure;
	}

	// set default lcd panel type
	socle_bl->lcd = panel_70;

	// get the video boot argument
	fb_get_options("soclefb",&options);
	if(options) {
		strcpy(tmp, options);
		// get the video boot argument lcd panel type
		tmp_opt=tmp;
		while ((this_opt = strsep(&tmp_opt, ",")) != NULL) {
	  		if (!strncmp(this_opt, "type=", 5)) {
	  			if(!strncmp(this_opt+5, "PANEL48", 7)) {
						DBG("panel type is PANEL48\n");
	  				socle_bl->lcd = panel_48;
					} 
					else if(!strncmp(this_opt+5, "PANEL7a", 7)) {
						DBG("panel type is PANEL7a\n");
	  				socle_bl->lcd = panel_7a;
					} 
					else if(!strncmp(this_opt+5, "PANEL70", 7)) {
						DBG("panel type is panel_70\n");
	  				socle_bl->lcd = panel_70;
					}
					break;
	  		}
	  	}
	}

	DBG("socle_bl->lcd = %d\n", socle_bl->lcd);
	
	socle_backlight_init_by_pwm(socle_bl);

	return ret;

out_release_socle_pwmt_structure:
	release_socle_pwmt_structure(USE_PWM_NUM);

out_kree:
	kfree(socle_bl);

out_return:
	return ret;
}


static int
socle_backlight_remove(struct platform_device *pdev)
{
	struct socle_backlight *socle_bl;

	socle_bl = platform_get_drvdata(pdev);
	
	led_classdev_unregister(&socle_bl->cdev);

	release_socle_pwmt_structure(USE_PWM_NUM);
	
	kfree(socle_bl);
	
	return 0;
}

#ifdef CONFIG_PM
static int
socle_backlight_suspend(struct platform_device *pdev, pm_message_t msg)
{	
	struct socle_backlight *socle_bl;
	struct socle_pwmt_driver *pwm_drv;

	socle_bl = platform_get_drvdata(pdev);

	pwm_drv = socle_bl->p_pwm->drv;
	
	//pwm_drv->suspend(socle_bl->p_pwm);

        return 0;
}

static int 
socle_backlight_resume(struct platform_device *pdev)
{	
	struct socle_backlight *socle_bl;
	struct socle_pwmt_driver *pwm_drv;

	socle_bl = platform_get_drvdata(pdev);

	pwm_drv = socle_bl->p_pwm->drv;
	
	//pwm_drv->resume(socle_bl->p_pwm);

	pwm_drv->claim_pwm_lock();

	pwm_drv->reset(socle_bl->p_pwm);
	pwm_drv->write_prescale_factor(socle_bl->p_pwm, DEFAULT_PRE_SCL);
	pwm_drv->write_hrc(socle_bl->p_pwm, socle_bl->hrc_value);
	pwm_drv->write_lrc(socle_bl->p_pwm, socle_bl->lrc_value);
	pwm_drv->output_enable(socle_bl->p_pwm, 1);
	pwm_drv->enable(socle_bl->p_pwm, 1);

	pwm_drv->release_pwm_lock();
	
  	return 0;
}
#else
#define socle_backlight_suspend	NULL
#define socle_backlight_resume		NULL
#endif

static struct platform_driver socle_led_backlight_drv = {
	.probe		= socle_backlight_probe,
	.remove		= socle_backlight_remove,
	.suspend		= socle_backlight_suspend,
	.resume		= socle_backlight_resume,
	.driver		= {
		.name	= "socle-led-backlight",	
		.owner	= THIS_MODULE,
	},
};

static int __init socle_backlight_init(void)
{
	printk(banner);
	return platform_driver_register(&socle_led_backlight_drv);
}

static void __exit socle_backlight_exit(void)
{
	platform_driver_unregister(&socle_led_backlight_drv);
}
module_init(socle_backlight_init);
module_exit(socle_backlight_exit);

MODULE_AUTHOR ("Jerryh Hsieh");
MODULE_LICENSE ("Dual BSD/GPL");	/* gets rid of that tainting message */
MODULE_DESCRIPTION("Socle Backlight Driver");
