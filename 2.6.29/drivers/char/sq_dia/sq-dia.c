#include <linux/fs.h>

#include <linux/module.h>
#include <linux/cdev.h>

//#define CONFIG_SQ_DIA_DEBUG
#ifdef CONFIG_SQ_DIA_DEBUG
	#define DIA_DBG(fmt, args...) printk("\nSQ DIA: " fmt, ## args)
#else
	#define DIA_DBG(fmt, args...)
#endif


static struct cdev sq_dia_cdev;

#define NAME		"sq_dia"
#define sq_dia_COUNT	32
#define sq_dia_SIZE	256


	
	
static int major;
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static ulong mask;
module_param(mask, ulong, 0);
MODULE_PARM_DESC(mask, "GPIO channel mask");

//----------------------------------------------------------------------
//sq_dia_write
//----------------------------------------------------------------------

static ssize_t sq_dia_write(struct file *file, const char __user *data,
				 size_t len, loff_t *ppos)
{
	DIA_DBG(" write len=0x%x \n",len);
	return 0;
}

//----------------------------------------------------------------------
//sq_dia_read
//----------------------------------------------------------------------

static ssize_t sq_dia_read(struct file *file, char __user *buf,
				size_t len, loff_t *ppos)
{
	DIA_DBG(" read len=0x%x \n",len);
	return 0;
}


//----------------------------------------------------------------------
//sq_dia_close
//----------------------------------------------------------------------

static ssize_t sq_dia_close(struct inode *inode, struct file *filp)
{
//	DIA_DBG(" close!\n");
	return 0;
}

//----------------------------------------------------------------------
//sq_dia_open
//----------------------------------------------------------------------

static ssize_t sq_dia_open(struct inode *inode, struct file *file)
{
//	DIA_DBG(" open!\n");
	return 0;
}

//----------------------------------------------------------------------
//sq_dia_ioctrl
//----------------------------------------------------------------------

enum dia_item {
DIA_GPIO,
DIA_SDR,
DIA_INTR,
DIA_UART,
DIA_TIMER,
DIA_RTC,
DIA_SPI,
DIA_UDC,
DIA_UHC,
DIA_MAC,
DIA_HDMA,
DIA_I2S,
DIA_I2C,
DIA_SDMMC,
DIA_NAND,
DIA_LCD,
DIA_PWMT,
DIA_VOP,
DIA_SCU,
DIA_ADC,
};

#define DIA_GET_IP_NUM(cmd)	((cmd >> 8)& 0xff)
#define DIA_GET_FUNC_NUM(cmd)	((cmd >> 0)& 0xff)
#define DIA_SET_IP_NUM(x)	((cmd & 0xff) << 8)
#define DIA_SET_FUNC_NUM(x)	((cmd & 0xff) << 0)


#define BROOK_PWM_MAGIC     'm'
#define BROOK_SPI_MAGIC     'i'
#define BROOK_SCU_MAGIC     'u'
#define BROOK_GPIO_MAGIC     'o'
#define BROOK_PWM_GET_VERSION    _IOR(BROOK_PWM_MAGIC, 1, int)

#define SQ_DIA_PWM_EN	1
#define SQ_DIA_SCU_EN	0
#define SQ_DIA_ADC_EN	1

#if (SQ_DIA_PWM_EN == 1)
#include "sq-dia-pwm.c"
#endif
#if (SQ_DIA_SCU_EN == 1)
#include "sq-dia-scu.c"
#endif
#if (SQ_DIA_ADC_EN == 1)
#include "sq-dia-adc.c"
#endif


static ssize_t sq_dia_ioctrl(struct inode *inode,struct file *filp, 
			     unsigned int cmd, unsigned long arg)
{
	int ip_sel = DIA_GET_IP_NUM(cmd);
	
//	DIA_DBG("\ninode=0x%p filp=0x%p cmd=0x%x arg=0x%lx \n",inode,filp,cmd,arg);


	switch (ip_sel){
		case DIA_GPIO:	DIA_DBG("DIA_GPIO:");	break;
		case DIA_SDR:   DIA_DBG("DIA_SDR:");	break;
		case DIA_INTR:  DIA_DBG("DIA_INTR:");	break;
		case DIA_UART:  DIA_DBG("DIA_UART:");	break;
		case DIA_TIMER: DIA_DBG("DIA_TIMER:");	break;
		case DIA_RTC:   DIA_DBG("DIA_RTC:");	break;
		case DIA_SPI:   DIA_DBG("DIA_SPI:");	break;
		case DIA_UDC:   DIA_DBG("DIA_UDC:");	break;
		case DIA_UHC:   DIA_DBG("DIA_UHC:");	break;
		case DIA_MAC:   DIA_DBG("DIA_MAC:");	break;
		case DIA_HDMA:  DIA_DBG("DIA_HDMA:");	break;
		case DIA_I2S:   DIA_DBG("DIA_I2S:");	break;
		case DIA_I2C:   DIA_DBG("DIA_I2C:");	break;
		case DIA_SDMMC: DIA_DBG("DIA_SDMMC:");	break;
		case DIA_NAND:  DIA_DBG("DIA_NAND:");	break;
		case DIA_LCD:   DIA_DBG("DIA_LCD:");	break;
		case DIA_PWMT:  DIA_DBG("DIA_PWMT:");	
			#if (SQ_DIA_PWM_EN == 1)
			sq_dia_pwm(DIA_GET_FUNC_NUM(cmd),arg);
			#endif	
			break;
		case DIA_VOP:   DIA_DBG("DIA_VOP:");	break;
		case DIA_SCU:   DIA_DBG("DIA_SCU:");	
			#if (SQ_DIA_SCU_EN == 1)		
			sq_dia_scu(DIA_GET_FUNC_NUM(cmd),arg);
			#endif
			break;	
		case DIA_ADC:   DIA_DBG("DIA_ADC:");	
			#if (SQ_DIA_ADC_EN == 1)		
			sq_dia_adc(DIA_GET_FUNC_NUM(cmd),arg);
			#endif
			break;					
		default:
			DIA_DBG("Not Support IP\n");
			return -1;
	}



	return 0;
}

static const struct file_operations sq_dia_fops = {
	.owner	= THIS_MODULE,
	.write	= sq_dia_write,
	.read	= sq_dia_read,
	.open	= sq_dia_open,
	.release = sq_dia_close,
	.ioctl	= sq_dia_ioctrl,
};

//----------------------------------------------------------------------
//sq_dia_init
//----------------------------------------------------------------------
int __init sq_dia_init(void)
{
	dev_t	dev_id;
	int	retval;

	DIA_DBG(" init\n");

	if (major) {
		dev_id = MKDEV(major, 0);
		retval = register_chrdev_region(dev_id, sq_dia_COUNT,
						NAME);
	} else {
		retval = alloc_chrdev_region(&dev_id, 0, sq_dia_COUNT,
					     NAME);
		major = MAJOR(dev_id);
	}

	DIA_DBG(":  mask=%#lx major=%d\n", mask, major);

	cdev_init(&sq_dia_cdev, &sq_dia_fops);
	cdev_add(&sq_dia_cdev, dev_id, sq_dia_COUNT);


	return 0;
}

//----------------------------------------------------------------------
//sq_dia_exit
//----------------------------------------------------------------------

static void __exit sq_dia_exit(void)
{
	dev_t dev_id = MKDEV(major, 0);

	DIA_DBG(" exit\n");


	cdev_del(&sq_dia_cdev);
	unregister_chrdev_region(dev_id, sq_dia_COUNT);
//	release_region(gpio_base, sq_dia_SIZE);
}







module_init(sq_dia_init);
module_exit(sq_dia_exit);



MODULE_AUTHOR("Channing Lan");
MODULE_DESCRIPTION("SQ driver dia");
MODULE_LICENSE("GPL");
