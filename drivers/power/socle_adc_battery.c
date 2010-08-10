#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/io.h>

#include <asm/mach-types.h>
#include <mach/regs-adc.h>
#include <mach/socle-adc.h>

//#define CONFIG_BAT_DEBUG
#ifdef CONFIG_BAT_DEBUG
	#define DBG(fmt, args...) printk("Battery: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

/* Battery Define */
#define SOCLE_ADC_BAT_MAX_VOLTAGE	3300	// 3.3V maximum voltage
#define SOCLE_ADC_BAT_MIN_VOLTAGE	0		// 0V minimum voltage

static char __initdata banner[] = "SOCLE ADC Battery Driver, (c) 2010 SOCLE Corp.\n";

struct socle_battery_dev_info {
	int voltage_mV;
	int rem_capacity;
	struct device *dev;
	struct power_supply bat;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

static int socle_adc_battery_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
#ifndef CONFIG_ARCH_MDK_3D
	struct socle_battery_dev_info *bat_di = container_of(psy,
		struct socle_battery_dev_info, bat);
#endif	

	DBG("###---start\n");
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CONFIG_ARCH_MDK_3D
		val->intval = 4000;
#else
		val->intval = bat_di->voltage_mV;
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = SOCLE_ADC_BAT_MAX_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = SOCLE_ADC_BAT_MIN_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 25;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_ARCH_MDK_3D
		val->intval = 80;
#else
		val->intval = bat_di->rem_capacity;
#endif
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void socle_adc_batery_external_power_changed(struct power_supply *bat_ps)
{
	DBG("###---start\n");
}

static void socle_adc_battery_work(struct work_struct *work)
{
	struct socle_battery_dev_info *bat_di = container_of(work,
		struct socle_battery_dev_info, monitor_work.work);
	const int interval = HZ / 1;
	u32 data;
	
	//DBG("###---start\n");
#ifdef CONFIG_ARCH_MDK_3D
	data = 0;
#else
	data = socle_adc_read_data(ADC_SOURCE_0);
	if(data < 0) {
		printk("warning : adc read data timeout\n ");
		data = 0;
	}
#endif
	DBG("data = %d\n", data);
	bat_di->voltage_mV = (data*1000)/312;
	bat_di->rem_capacity = (bat_di->voltage_mV*100) / SOCLE_ADC_BAT_MAX_VOLTAGE;
	
	queue_delayed_work(bat_di->monitor_wqueue, &bat_di->monitor_work, interval);
}

#ifdef CONFIG_PM
static int socle_adc_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int socle_adc_battery_resume(struct platform_device *dev)
{
	return 0;
}
#else
#define socle_adc_battery_suspend NULL
#define socle_adc_battery_resume NULL
#endif

static enum power_supply_property socle_adc_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int __devinit socle_adc_battery_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct socle_battery_dev_info *bat_di;
	
	DBG("###---start\n");

	bat_di = kzalloc(sizeof(*bat_di), GFP_KERNEL);
	if (!bat_di) {
		printk("Error : failed to allocate socle_battery_dev_info structure\n");
		ret = -ENOMEM;
		return ret;;
	}

	platform_set_drvdata(pdev, bat_di);

	bat_di->dev			= &pdev->dev;
	bat_di->bat.name		= dev_name(&pdev->dev);
	bat_di->bat.type		= POWER_SUPPLY_TYPE_BATTERY;
	bat_di->bat.properties	= socle_adc_battery_props;
	bat_di->bat.num_properties	= ARRAY_SIZE(socle_adc_battery_props);
	bat_di->bat.get_property	= socle_adc_battery_get_property;
	bat_di->bat.external_power_changed = socle_adc_batery_external_power_changed;
	bat_di->bat.use_for_apm = 1;
	
	ret = power_supply_register(&pdev->dev, &bat_di->bat);
	if (ret) {
		printk("Error : failed to register battery\n");
		goto out_no_resource;
	}

	INIT_DELAYED_WORK(&bat_di->monitor_work, socle_adc_battery_work);
	
	bat_di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!bat_di->monitor_wqueue) {
		ret = -ESRCH;
		goto out_power_supply_unregister;
	}
	
	queue_delayed_work(bat_di->monitor_wqueue, &bat_di->monitor_work, HZ * 1);
	
	return 0;

out_power_supply_unregister:
	power_supply_unregister(&bat_di->bat);
out_no_resource:
	kfree(bat_di);
	return ret;
	
}

static int __devexit socle_adc_battery_remove(struct platform_device *pdev)
{
	struct socle_battery_dev_info *bat_di = platform_get_drvdata(pdev);

	DBG("###---start\n");

	destroy_workqueue(bat_di->monitor_wqueue);

	cancel_rearming_delayed_workqueue(bat_di->monitor_wqueue,
					  &bat_di->monitor_work);
	
	power_supply_unregister(&bat_di->bat);
	
	kfree(bat_di);
	return 0;
}

static struct platform_driver socle_adc_battery_driver = {
	.driver		= {
		.name	= "socle-adc-battery",
	},
	.probe		= socle_adc_battery_probe,
	.remove		= __devexit_p(socle_adc_battery_remove),
	.suspend		= socle_adc_battery_suspend,
	.resume		= socle_adc_battery_resume,
};

static int __init socle_adc_battery_init(void)
{
	printk(banner);
	return platform_driver_register(&socle_adc_battery_driver);
}

static void __exit socle_adc_battery_exit(void)
{
	platform_driver_unregister(&socle_adc_battery_driver);
}

module_init(socle_adc_battery_init);
module_exit(socle_adc_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jerry Hsieh");
MODULE_DESCRIPTION("adc battery driver");
