#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include <asm/mach-types.h>
#include <mach/regs-adc.h>
#include <mach/socle-adc.h>

//#define CONFIG_ADC_DEBUG
#ifdef CONFIG_ADC_DEBUG
	#define DBG(fmt, args...) printk("ADC: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

static char __initdata banner[] = "SOCLE ADC Driver, (c) 2009 SOCLE Corp.\n";

struct socle_adc_device {
	u32	base;
	int	irq;
	wait_queue_head_t	*wait;
	int	 result;
};

//static DEFINE_SPINLOCK(adc_lock);

static struct socle_adc_device *adc_dev;

static inline void
socle_adc_write(u32 reg, u32 val, u32 base)
{
	//DBG("reg=0x%04x, val=0x%08x, base=0x%08x\n", reg, val, base);
	iowrite32(val, base + reg);
}

static inline u32
socle_adc_read(u32 reg, u32 base)
{
	u32 val = ioread32(base + reg);
	//DBG("reg=0x%04x, val=0x%08x, base=0x%08x\n", reg, val, base);
	return val;
}

static int
socle_adc_start(unsigned int channel)
{
	int temp;

	DBG("###---start\n");

	temp = socle_adc_read(ADC_CTRL, adc_dev->base);
	temp = temp | channel | ADC_START_CONVERSION;
	
	socle_adc_write(ADC_CTRL, temp, adc_dev->base);

	return 0;
}

static void
socle_adc_convert_done(unsigned value)
{
	DBG("###---start\n");
	
	adc_dev->result = value;
	wake_up(adc_dev->wait);
}

int
socle_adc_read_data(unsigned int ch)
{
	//DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wake);
	DECLARE_WAIT_QUEUE_HEAD(wake);
	int ret;

	DBG("###---start\n");
	
	//spin_lock(&adc_lock);
		
	adc_dev->wait = &wake;
	adc_dev->result = -1;
	ret = socle_adc_start(ch);
	if (ret < 0)
		goto err;

	ret = wait_event_timeout(wake, adc_dev->result >= 0, HZ / 2);
	if (adc_dev->result < 0) {
		ret = -ETIMEDOUT;
		goto err;
	}

	//spin_unlock(&adc_lock);

	DBG("adc_dev->result = %d\n", adc_dev->result);
		
	return adc_dev->result;

err:
	return ret;
}

EXPORT_SYMBOL_GPL(socle_adc_read_data);

#ifdef CONFIG_ARCH_MDK_FHD
void
socle_adc_touch_switch(int mode)
{
	int temp;
	
	switch(mode) {

		case XP:
			temp = socle_adc_read(ADC_CTRL, adc_dev->base);
			temp = temp & (~ADC_TOUCH_SWITCH_MASK);
			temp = temp | ADC_TOUCH_SWITCH_XP_ENABLE
					| ADC_TOUCH_SWITCH_XM_ENABLE
					| ADC_TOUCH_SWITCH_YP_DISABLE
					| ADC_TOUCH_SWITCH_YM_DISABLE;
			
			socle_adc_write(ADC_CTRL, temp, adc_dev->base);
			break;

		case YP:
			temp = socle_adc_read(ADC_CTRL, adc_dev->base);
			temp = temp & (~ADC_TOUCH_SWITCH_MASK);
			temp = temp | ADC_TOUCH_SWITCH_XP_DISABLE
					| ADC_TOUCH_SWITCH_XM_DISABLE
					| ADC_TOUCH_SWITCH_YP_ENABLE
					| ADC_TOUCH_SWITCH_YM_ENABLE;
			
			socle_adc_write(ADC_CTRL, temp, adc_dev->base);
			break;

		case ZX:
			temp = socle_adc_read(ADC_CTRL, adc_dev->base);
			temp = temp & (~ADC_TOUCH_SWITCH_MASK);
			temp = temp | ADC_TOUCH_SWITCH_XP_DISABLE
					| ADC_TOUCH_SWITCH_XM_ENABLE
					| ADC_TOUCH_SWITCH_YP_ENABLE
					| ADC_TOUCH_SWITCH_YM_DISABLE;
			
			socle_adc_write(ADC_CTRL, temp, adc_dev->base);
			break;

		case ZY:
			temp = socle_adc_read(ADC_CTRL, adc_dev->base);
			temp = temp & (~ADC_TOUCH_SWITCH_MASK);
			temp = temp | ADC_TOUCH_SWITCH_XP_DISABLE
					| ADC_TOUCH_SWITCH_XM_ENABLE
					| ADC_TOUCH_SWITCH_YP_ENABLE
					| ADC_TOUCH_SWITCH_YM_DISABLE;
			
			socle_adc_write(ADC_CTRL, temp, adc_dev->base);
			break;

		default:
			printk("touch screen mode not defined\n");
	}
}

EXPORT_SYMBOL_GPL(socle_adc_touch_switch);
#endif

static irqreturn_t
socle_adc_isr(int irq, void *dev)
{
	int temp;
	int data;
	
	struct socle_adc_device *adc_dev = dev;

	DBG("###---start\n");
	
	temp = socle_adc_read(ADC_CTRL,  adc_dev->base);
	temp = temp & (~ADC_INT_STATUS);
	socle_adc_write(ADC_CTRL, temp, adc_dev->base);

	data = socle_adc_read(ADC_DATA, adc_dev->base);
//printk("@@--%s--data = 0x%x\n", __func__, data);
	
	socle_adc_convert_done(data);
	
	return IRQ_HANDLED;
}

static void
adc_initial(struct socle_adc_device *adc_dev)
{
	int temp;

	DBG("###---start\n");
	
	temp = socle_adc_read(ADC_CTRL, adc_dev->base);
	temp = temp & (~ADC_INT_MASK) & (~ADC_SOURCE_MASK);
#if defined(CONFIG_ARCH_PDK_PC9223)
	temp = (temp | ADC_INT_ENABLE) & (~ADC_POWER_UP);
#else
	temp = temp | ADC_POWER_UP | ADC_INT_ENABLE;
#endif
	socle_adc_write(ADC_CTRL, temp, adc_dev->base);

}

static int __init
socle_adc_init(void)
{
	int ret = 0;

	printk(banner);
	
	DBG("###---start\n");

	adc_dev = kzalloc(sizeof(adc_dev), GFP_KERNEL);
	if (adc_dev == NULL) {
		printk("Error : failed to allocate adc_device\n");
		return -ENOMEM;
	}
	
	adc_dev->base = (u32)ioremap(SOCLE_ADC0, SZ_4K);
	if(!adc_dev->base) {
		printk("Error : adc base ioremap fail\n");
		ret = -ENXIO;
		goto out_kree;
	}

	DBG("adc_dev->base = 0x%x\n", adc_dev->base);
		
	adc_dev->irq = IRQ_ADC0;
		
	ret = request_irq(adc_dev->irq, socle_adc_isr, IRQF_DISABLED, "socle-adc", adc_dev);
	if (ret) {
		printk("Error : request_irq fail\n\n");
		goto out_iounmap;
	}

	adc_initial(adc_dev);

	return 0;
	
out_iounmap:
	iounmap((void __iomem *)adc_dev->base);
out_kree:
	kfree(adc_dev);

	return ret;
}

//20100610 jerry+ for suspend/resume
static void
socle_adc_resume(void)
{
	adc_initial(adc_dev);
}
EXPORT_SYMBOL_GPL(socle_adc_resume);

core_initcall(socle_adc_init);

MODULE_DESCRIPTION("SOCLE ADC Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jerry Hsieh");

