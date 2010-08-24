#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <mach/platform.h>
#include <linux/delay.h>


#include <linux/poll.h>




//#define CONFIG_SQ_SPI_DOWNLOAD_DEBUG
#ifdef CONFIG_SQ_SPI_DOWNLOAD_DEBUG
	#define SPI_DOWNLOAD_DBG(fmt, args...) printk("\n[spi_download]: " fmt, ## args)
#else
	#define SPI_DOWNLOAD_DBG(fmt, args...)
#endif

#define USE_CDEV	0
//#if (USE_CDEV == 1)
static struct cdev sq_spi_download_cdev;
//#endif
#define NAME		"sq_spi_download"
#define sq_spi_download_COUNT	32
#define sq_spi_download_SIZE	256


#define SPI_CTRL_GPIO_EN 0




#define sq_gpio_claim_lock			socle_gpio_claim_lock                   
#define sq_gpio_get_value_with_mask             socle_gpio_get_value_with_mask          
#define sq_gpio_release_lock	                socle_gpio_release_lock	             
#define sq_gpio_direction_output_with_mask      socle_gpio_direction_output_with_mask   
#define sq_gpio_set_value_with_mask             socle_gpio_set_value_with_mask          









struct spi_device *sq_spi_download_device;
	
static int major;
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static ulong mask;
module_param(mask, ulong, 0);
MODULE_PARM_DESC(mask, "GPIO channel mask");

//----------------------------------------------------------------------
//sq_spi_download_write
//----------------------------------------------------------------------
#if 1
#define TST_GPIO_G			PM//PN//PI

#define DL_PROGB			(0x01 << 3)
#define DL_DONE				(0x01 << 1)
#define DL_RST				(0x01 << 4)
#define DL_INITB			(0x01 << 0)


#define GPIO_2				(0x01 << 2)
#else
#define TST_GPIO_G			PJ//PN//PI

#define DL_PROGB			(0x01 << 7)
#define DL_DONE				(0x01 << 6)
#define DL_RST				(0x01 << 4)
#define DL_INITB			(0x01 << 5)
#define GPIO_2				(0x01 << 2)
#endif

#define TST_GPIO_N			PN//PI



#include "mach/gpio.h"

int sq_gpio_in(int port,int pin)
{
	int ret = 0;
	
	sq_gpio_claim_lock();	
	ret = sq_gpio_get_value_with_mask(port,pin);
	sq_gpio_release_lock();	

	udelay(100);
	//ret = 1;  //test
	SPI_DOWNLOAD_DBG(" sq_gpio_in port=0x%x  pin=0x%x ret=0x%x",port,pin,ret);	
	return ret;
	
	
}

void sq_gpio_set_high(int port,int pin)
{
	sq_gpio_claim_lock();	
	sq_gpio_direction_output_with_mask(port,pin);
	sq_gpio_set_value_with_mask(port,pin,pin);	
	sq_gpio_release_lock();
	
}
void sq_gpio_set_low(int port,int pin)
{
	sq_gpio_claim_lock();	
	sq_gpio_direction_output_with_mask(port,pin);
	sq_gpio_set_value_with_mask(port,0,pin);	
	sq_gpio_release_lock();	
}


void sq_spi_download_write_rst(void)
{
	sq_gpio_set_low(TST_GPIO_G,DL_RST);
		
	mdelay(100);	
	SPI_DOWNLOAD_DBG(" write_rst");				
	sq_gpio_set_high(TST_GPIO_G,DL_RST);


}

/*
Step1:   
     PROG_B  先送Low ( 大於 1us ) 再送high.
Step2:
     Check INIT_B 直到它為High.
Step3:
     讀檔案, 轉換成SPI data送出. MSB first ?  byte or word ?  忘了, 再請你試一下.
*/
void sq_spi_download_write_before(void)
{
//	unsigned long timeout=0xff;
	
	
	//sq_gpio_set_low(TST_GPIO_G,DL_PROGB);	
	//sq_gpio_set_low(TST_GPIO_G,DL_INITB);					
	//sq_gpio_set_low(TST_GPIO_G,DL_DONE);		
	//sq_gpio_set_low(TST_GPIO_G,DL_RST);
	sq_gpio_get_value_with_mask(TST_GPIO_G,DL_DONE);	
	sq_gpio_get_value_with_mask(TST_GPIO_G,DL_INITB);	
//	sq_gpio_in(TST_GPIO_G,DL_DONE);
//	sq_gpio_in(TST_GPIO_G,DL_INITB);
	
//	sq_spi_download_write_rst();		
	sq_gpio_set_low(TST_GPIO_G,DL_PROGB);
		
	udelay(50);	
	
	sq_gpio_set_high(TST_GPIO_G,DL_PROGB);
	

	while(sq_gpio_in(TST_GPIO_G,DL_INITB) == 0) {}	//wait 80us
}



/*
Step4:
     送完後,  check DONE 為high 代表完成, 若為Low, 回到 Step1 retry
Step5:
     FPGA_rst 送Low, 維持約100ms後, 送High. 然後大功告成!!
*/




int sq_spi_download_write_chk(void)
{

	//sq_gpio_in(TST_GPIO_G,DL_DONE);
	if(sq_gpio_in(TST_GPIO_G,DL_DONE) != 0) {
		sq_spi_download_write_rst();
		return 0;	
	}
	return 1;
}




#define SET_TX_RX_LEN(tx, rx)	(((tx) << 16) | (rx))
static ssize_t sq_spi_download_write(struct file *file, const char __user *data,
				 size_t len, loff_t *ppos)
{
	int err = 0;
	//char *p;
	struct spi_message msg;
	struct spi_transfer xfer;
	//char buf[128];
	char *buf;

	buf = kmalloc(len,GFP_KERNEL);
	if(!buf){
		return -ENOMEM;
	}
	
	
	if(copy_from_user(buf,data,len)) {
		return -EFAULT;
	}
	
	SPI_DOWNLOAD_DBG(" write len=0x%x =>",len);

#if (SPI_CTRL_GPIO_EN == 1 )
	do {
#endif

	/*
	i=0;
	while(1) {
		sq_gpio_set_low(TST_GPIO_G,1<<i);	
		sq_gpio_set_high(TST_GPIO_G,1<<i);	
		i++;
		if(i>0x07)
			i=0;	
	}
	*/
	


#if 0
	i=1;
	while(i--)
	{
	
	sq_gpio_set_low(TST_GPIO_G,DL_PROGB);	
	udelay(20);
	sq_gpio_set_high(TST_GPIO_G,DL_PROGB);						
	/*	
	sq_gpio_set_low(TST_GPIO_G,DL_RST);	
	udelay(20);
	sq_gpio_set_high(TST_GPIO_G,DL_RST);	
	
	
	sq_gpio_set_low(TST_GPIO_G,DL_INITB);
	udelay(20);
	sq_gpio_set_high(TST_GPIO_G,DL_INITB);
	
	
	sq_gpio_set_low(TST_GPIO_G,DL_DONE);
	udelay(20);
	sq_gpio_set_high(TST_GPIO_G,DL_DONE);
	*/	
	
	//sq_gpio_in(TST_GPIO_G,DL_INITB);
	//sq_gpio_in(TST_GPIO_G,DL_DONE);
	//sq_spi_download_write_rst();
	
	}
#endif
		
	
/*
	c = len;
	p = data;
	while(c--) {
	printk("%c",*p++);
	}	
	printk("\n");	
*/	


#if 1
	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(struct spi_transfer));
	
	//SPI_DOWNLOAD_DBG(" bits_per_word= %d",msg.spi->bits_per_word);	
	xfer.tx_buf = buf;	
	

	
//	xfer.tx_buf = tx_buf;	
	xfer.len = SET_TX_RX_LEN(len, 0);
	spi_message_add_tail(&xfer, &msg);
#if (SPI_CTRL_GPIO_EN == 1 )
	sq_spi_download_write_before();
#endif
	
	err = spi_sync(sq_spi_download_device, &msg);
	
	
#if (SPI_CTRL_GPIO_EN == 1 )
	//sq_spi_download_write_chk();
	
	} while (sq_spi_download_write_chk() != 0 );
#endif
#endif

	kfree(buf);

//	SPI_DOWNLOAD_DBG(" <============== sq_spi_download_write end ===============>");
	return 0;	
	
}

//----------------------------------------------------------------------
//sq_spi_download_read
//----------------------------------------------------------------------

static ssize_t sq_spi_download_read(struct file *file, char __user *buf,
				size_t len, loff_t *ppos)
{
	SPI_DOWNLOAD_DBG(" read len=0x%x \n",len);
	
	
	return 0;
}


//----------------------------------------------------------------------
//sq_spi_download_close
//----------------------------------------------------------------------

static ssize_t sq_spi_download_close(struct inode *inode, struct file *filp)
{
	//SPI_DOWNLOAD_DBG(" close!\n");
	return 0;
}

//----------------------------------------------------------------------
//sq_spi_download_open
//----------------------------------------------------------------------

static ssize_t sq_spi_download_open(struct inode *inode, struct file *file)
{
	//SPI_DOWNLOAD_DBG(" open!\n");
	return 0;
}

//----------------------------------------------------------------------
//sq_spi_download_ioctrl
//----------------------------------------------------------------------

/*
Step1:   
     PROG_B  先送Low ( 大於 1us ) 再送high.
Step2:
     Check INIT_B 直到它為High.
Step3:
     讀檔案, 轉換成SPI data送出. MSB first ?  byte or word ?  忘了, 再請你試一下.
Step4:
     送完後,  check DONE 為high 代表完成, 若為Low, 回到 Step1 retry
Step5:
     FPGA_rst 送High, 維持約100ms後, 送Low. 然後大功告成!!
*/
#define SET_PROB_LOW	1
#define SET_PROB_HIGH	2
#define GET_INITB	3
#define GET_DONE	4
#define SET_RST_LOW	5
#define SET_RST_HIGH	6
#define SET_RST		7
#define GET_MAX_BUF     8
#define SET_PROB_LH	9

#define SPI_BUF_SIZE  64

static ssize_t sq_spi_download_ioctrl(struct inode *inode,struct file *filp, 
			     unsigned int cmd, unsigned long arg)
{
	unsigned int *p;
	
	SPI_DOWNLOAD_DBG("IOCTL: cmd=%d!",cmd);	
	
	p=(unsigned int  *)arg;
	
	switch(cmd&0xff){
	case SET_PROB_LOW:
		sq_gpio_set_low(TST_GPIO_G,DL_PROGB);		
		break;
		
	case SET_PROB_HIGH:
		sq_gpio_set_high(TST_GPIO_G,DL_PROGB);
		break;
		
	case SET_RST_HIGH:
		sq_gpio_set_high(TST_GPIO_G,DL_RST);
		break;
		
	case SET_RST_LOW:
		sq_gpio_set_low(TST_GPIO_G,DL_RST);
		break;	
						
	case GET_INITB:
		*p = sq_gpio_in(TST_GPIO_G,DL_INITB);
		break;
		
	case GET_DONE:
		*p = sq_gpio_in(TST_GPIO_G,DL_DONE);
		break;		
		
	case SET_RST:	
		SPI_DOWNLOAD_DBG("RST");		
		sq_spi_download_write_rst();
		break;
		
	case GET_MAX_BUF:
		*p = SPI_BUF_SIZE;
		break;	
		
	case SET_PROB_LH:
		sq_gpio_set_low(TST_GPIO_G,DL_PROGB);	
		udelay(100);			
		sq_gpio_set_high(TST_GPIO_G,DL_PROGB);
		break;			
	}

	return 0;
	
}

static const struct file_operations sq_spi_download_fops = {
	.owner	= THIS_MODULE,
	.write	= sq_spi_download_write,
	.read	= sq_spi_download_read,
	.open	= sq_spi_download_open,
	.release = sq_spi_download_close,
	.ioctl	= sq_spi_download_ioctrl,
};



static int __devinit 
sq_spi_download_probe(struct spi_device *spi)
{
	int err = 0;
	SPI_DOWNLOAD_DBG(" sq_spi_download_probe!\n");	
		
	sq_spi_download_device = spi;
	sq_spi_download_device->bits_per_word = 8;
	spi_setup(sq_spi_download_device);
	

	return err;
}

static int __devexit
sq_spi_download_remove(struct spi_device *spi)
{

	return 0;
}

static int sq_spi_download_suspend(struct spi_device *spi)
{
	return 0;
}
static int sq_spi_download_resume(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver sq_spi_download_driver = {
	.driver = {
		.name = "sq_spi_download",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = sq_spi_download_probe,
	.remove = __devexit_p(sq_spi_download_remove),
	.suspend	= sq_spi_download_suspend,
	.resume		= sq_spi_download_resume,	
};


//----------------------------------------------------------------------
//sq_spi_download_init
//----------------------------------------------------------------------
int __init sq_spi_download_init(void)
{
	int	err=0,i;
//#if (USE_CDEV == 1)
	dev_t	dev_id;
	int	retval;

	SPI_DOWNLOAD_DBG(" sq_spi_download_init\n");
	err=spi_register_driver(&sq_spi_download_driver);
	if(err)
		printk("sq_spi_download_driver init  fail\n");
		

/*
	err = platform_driver_register(&sq_spi_download_driver);
	if(err)
		printk("sq_spi_download_driver init  fail\n");
	
*/	

//#else

	if (major) {
		dev_id = MKDEV(major, 0);
		retval = register_chrdev_region(dev_id, sq_spi_download_COUNT,NAME);
	} else {
		retval = alloc_chrdev_region(&dev_id, 0, sq_spi_download_COUNT,NAME);
		major = MAJOR(dev_id);
	}

	SPI_DOWNLOAD_DBG(":  mask=%#lx major=%d\n", mask, major);

	cdev_init(&sq_spi_download_cdev, &sq_spi_download_fops);
	cdev_add(&sq_spi_download_cdev, dev_id, sq_spi_download_COUNT);


//#endif		
	SPI_DOWNLOAD_DBG(" sq_spi_download_init end\n");	
	/*
#define DL_PROGB			(0x01 << 7)
#define DL_DONE				(0x01 << 6)
#define DL_RST				(0x01 << 5)
#define DL_INITB			(0x01 << 4)
*/
	i=10;
	

			
	return err;

}

//----------------------------------------------------------------------
//sq_spi_download_exit
//----------------------------------------------------------------------

static void __exit sq_spi_download_exit(void)
{
//#if (USE_CDEV == 1)
	spi_unregister_driver(&sq_spi_download_driver);
	
	
/*
	platform_driver_unregister(&sq_spi_download_driver);	
*/	

//#else
	dev_t dev_id = MKDEV(major, 0);

	SPI_DOWNLOAD_DBG(" exit\n");


	cdev_del(&sq_spi_download_cdev);
	unregister_chrdev_region(dev_id, sq_spi_download_COUNT);
//	release_region(gpio_base, sq_spi_download_SIZE);

//#endif	
}



module_init(sq_spi_download_init);
//arch_initcall(sq_spi_download_init);

//late_initcall(sq_spi_download_init);
module_exit(sq_spi_download_exit);



MODULE_AUTHOR("Channing Lan");
MODULE_DESCRIPTION("SQ driver spi_download");
MODULE_LICENSE("GPL");
