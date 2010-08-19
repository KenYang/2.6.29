
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <linux/delay.h>
#include <linux/input.h>
#include <mach/irqs.h>
#include <mach/cma3000_d0x-regs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>

//#define CONFIG_CMA3000_DEBUG
#ifdef CONFIG_CMA3000_DEBUG
	#define DBG(fmt, args...) printk("CMA3000: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

#define SENSOR_ACTIVATE	0x01
#define SENSOR_DEACTIVATE	0x02

static unsigned short normal_i2c[] = {
	CMA3000_D0X_I2C_CLIENT_ADDR,
	I2C_CLIENT_END,
};
int test_count = 0;
I2C_CLIENT_INSMOD;

static char __initdata banner[] = "SOCLE 3-AXIS ACCELEROMETER, (c) 2009 SOCLE Corp.\n";

static void cma3000_xyz_report(struct work_struct *param);

//static DECLARE_WORK(work, cma3000_xyz_report);
static DECLARE_DELAYED_WORK(dwork, cma3000_xyz_report);

static struct cma3000_dev_st {
	struct input_dev *idev;
	struct i2c_client *cma3000_client;
	int x;
	int y;
	int z;
}cma3000_dev;

static int sensor_disable = 0;

static int inline
cma3000_d0x_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;
	
	ret = i2c_master_send(client, (char *)buf, 2);
	
	if (ret != 2)
		return -1;
	else
		return 0;

}

static int inline
cma3000_d0x_read(struct i2c_client *client, u8 reg)
{
	int ret;
	struct i2c_msg msg[2];
	u8 buf = reg;
	u8 ret_buf;
	int ret_val;
       
	memset((void *)msg, 0x00, 2*sizeof(struct i2c_msg));
	msg[0].addr = client->addr;
	msg[0].buf = &buf;
	msg[0].len = 1;
	
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &ret_buf;
	msg[1].len = 1;
	ret = i2c_transfer(client->adapter, msg, 2);
	
	//if (ret != 2)
	//	return -1;
	ret_val = ret_buf;
	
	return ret_val;

}

static void
cma3000_get_xyz(int *x, int *y, int *z)
{
	*x = cma3000_d0x_read(cma3000_dev.cma3000_client, CMA3000_D0X_DOUTX);
	*y = cma3000_d0x_read(cma3000_dev.cma3000_client, CMA3000_D0X_DOUTY);
	*z = cma3000_d0x_read(cma3000_dev.cma3000_client, CMA3000_D0X_DOUTZ);
}

static int
cma3000_idev_open(struct input_dev *dev)
{
	DBG("---start\n");
	return 0;
}

static void
cma3000_idev_close(struct input_dev *dev)
{
	DBG("---start\n");
}

static int
cma3000_input_device_enable(struct i2c_client *client)
{
	int ret;

	if (cma3000_dev.idev)
		return -EINVAL;

	cma3000_dev.idev = input_allocate_device();
	if (!cma3000_dev.idev)
		return -ENOMEM;

	cma3000_dev.idev->name       = "CMA3000 Accelerometer";
	cma3000_dev.idev->phys       = "cma3000/input1";
	cma3000_dev.idev->id.bustype = BUS_HOST;
	cma3000_dev.idev->id.vendor = 0x0001;
	cma3000_dev.idev->id.product = 0x0001;
	cma3000_dev.idev->id.version = 0x0001;
	cma3000_dev.idev->open       = cma3000_idev_open;
	cma3000_dev.idev->close      = cma3000_idev_close;
	
	set_bit(EV_ABS, cma3000_dev.idev->evbit);

	input_set_abs_params(cma3000_dev.idev, ABS_X, -9070, 9070, 3, 3);
	input_set_abs_params(cma3000_dev.idev, ABS_Y, -9070, 9070, 3, 3);
	input_set_abs_params(cma3000_dev.idev, ABS_Z, -9070, 9070, 3, 3);

	ret = input_register_device(cma3000_dev.idev);
	if (ret) {
		input_free_device(cma3000_dev.idev);
		cma3000_dev.idev= NULL;
	}

	return ret;
	

}

static void
cma3000_xyz_report(struct work_struct *param)
{
	int x_value, y_value, z_value;

	cma3000_get_xyz(&cma3000_dev.x, &cma3000_dev.y, &cma3000_dev.z);
	
	x_value = cma3000_dev.x;
	y_value = cma3000_dev.y;
	z_value = cma3000_dev.z;

	input_report_abs(cma3000_dev.idev, ABS_X, x_value);
	input_report_abs(cma3000_dev.idev, ABS_Y, y_value);
	input_report_abs(cma3000_dev.idev, ABS_Z, z_value);

	input_sync(cma3000_dev.idev);

	if (!sensor_disable)
		schedule_delayed_work (&dwork, HZ / 10);
	//enable_irq(IRQ_CMA3000);
}

#if 0
static irqreturn_t
cma3000_d0x_isr(int irq, void *dev)
{
	disable_irq(IRQ_CMA3000);
	schedule_work(&work);
	return IRQ_HANDLED;
}
#endif

static void
cma3000_d0x_chip_init(struct i2c_client *client)
{
	int data;
	
	//initial the client device
	cma3000_d0x_write(client, CMA3000_D0X_CTRL, 
		CMA3000_D0X_CTRL_MODE_PWR_DOWN_DEF
		| CMA3000_D0X_CTRL_G_RANGE_2G
		| CMA3000_D0X_CTRL_MDET_EXIT_EXIT
		| CMA3000_D0X_CTRL_INT_LEVEL_L
		| CMA3000_D0X_CTRL_I2C_EN
		| CMA3000_D0X_CTRL_INT_DIS);

	data = cma3000_d0x_read(client, CMA3000_D0X_CTRL);
}

static void
cma3000_d0x_chip_active_without_int(struct i2c_client *client)
{
	int data;
	
	//initial the client device
	cma3000_d0x_write(client, CMA3000_D0X_CTRL, 
		CMA3000_D0X_CTRL_MODE_MEASURE_100
		| CMA3000_D0X_CTRL_G_RANGE_2G
		| CMA3000_D0X_CTRL_MDET_EXIT_EXIT
		| CMA3000_D0X_CTRL_INT_LEVEL_L
		| CMA3000_D0X_CTRL_I2C_EN
		| CMA3000_D0X_CTRL_INT_DIS);

	data = cma3000_d0x_read(client, CMA3000_D0X_CTRL);
}

static void
cma3000_d0x_chip_active_with_int(struct i2c_client *client)
{
	int data;
	
	cma3000_d0x_write(client, CMA3000_D0X_CTRL, 
		CMA3000_D0X_CTRL_MODE_MEASURE_100
		| CMA3000_D0X_CTRL_G_RANGE_2G
		| CMA3000_D0X_CTRL_MDET_EXIT_EXIT
		| CMA3000_D0X_CTRL_INT_LEVEL_L
		| CMA3000_D0X_CTRL_I2C_EN
		| CMA3000_D0X_CTRL_INT_EN);

	data = cma3000_d0x_read(client, CMA3000_D0X_CTRL);
}

static void
cma3000_d0x_chip_deactive(struct i2c_client *client)
{
	int data;
	
	cma3000_d0x_write(client, CMA3000_D0X_CTRL, 
		CMA3000_D0X_CTRL_MODE_PWR_DOWN
		| CMA3000_D0X_CTRL_G_RANGE_2G
		| CMA3000_D0X_CTRL_MDET_EXIT_EXIT
		| CMA3000_D0X_CTRL_INT_LEVEL_L
		| CMA3000_D0X_CTRL_I2C_EN
		| CMA3000_D0X_CTRL_INT_DIS);

	data = cma3000_d0x_read(client, CMA3000_D0X_CTRL);
}


static ssize_t
cma3000_cdev_read(struct file *flip, char __user *buf, size_t count, loff_t *f_pos)
{
	DBG("---start\n");
	return 0;
}


static ssize_t 
cma3000_cdev_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos) 
{
	DBG("---start\n");
	return count;
}

static int
cma3000_cdev_ioctl(struct inode *inode, struct file *file, unsigned int ioctl_code, unsigned long arg)
{
	DBG("---start\n\n");

	if (ioctl_code == SENSOR_ACTIVATE) {
		DBG("--SENSOR_ACTIVATE\n");
		cma3000_d0x_chip_active_without_int(cma3000_dev.cma3000_client);
		sensor_disable = 0;
		schedule_delayed_work (&dwork, HZ / 10);
	} else if (ioctl_code == SENSOR_DEACTIVATE) {
		DBG("--SENSOR_DEACTIVATE\n");
		cma3000_d0x_chip_deactive(cma3000_dev.cma3000_client);
		sensor_disable = 1;
	} else
		printk("Error : ioctl code is not define\n");
	return 0;
}

static int
cma3000_cdev_open(struct inode *inode, struct file *file)
{
	DBG("---start\n\n");
	return 0;
}

static int
cma3000_cdev_release(struct inode *inode, struct file *file)
{
	DBG("---start\n\n");
	return 0;
}

static struct file_operations cma3000_cdev_fops = {
	.owner = THIS_MODULE,
	.read = cma3000_cdev_read,
	.write = cma3000_cdev_write,
	.ioctl = cma3000_cdev_ioctl,
	.open = cma3000_cdev_open,
	.release = cma3000_cdev_release
};

static struct miscdevice misc_cma3000_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cma3000_device",
	.fops = &cma3000_cdev_fops,
};

static int
cma3000_d0x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int data;
	int ret;
	int i;
	
	printk(banner);
		
	cma3000_dev.cma3000_client = client;
	
	data = cma3000_d0x_read(client, CMA3000_D0X_REVID);
	if (data != 0x10) {
		printk("CMA3000 reversion ID not correct\n");
		ret = -1;
		return ret;
	}

	ret = cma3000_input_device_enable(client);
	if (ret)
		printk("cma3000 input device enable failed\n");

	ret = misc_register(&misc_cma3000_dev);
	if (ret) {
		printk("Error : can't register misc device\n");
		goto out_input_unregister_device;
	}
#if 0
	ret = request_irq(IRQ_CMA3000, cma3000_d0x_isr, IRQF_SHARED, cma3000_dev.idev->name, cma3000_dev.idev);
	if (ret) {
		printk(KERN_ERR "Unable to claim IRQ(%d), ret = %d\n", IRQ_CMA3000, ret);
		goto out_misc_deregister;
	}
#endif
	cma3000_d0x_chip_init(client);
		
	return 0;

out_misc_deregister:
	misc_deregister(&misc_cma3000_dev);
	
out_input_unregister_device:
	input_unregister_device(cma3000_dev.idev);
	return ret;
}

static int
cma3000_d0x_remove(struct i2c_client *client)
{

	free_irq(IRQ_CMA3000, cma3000_dev.idev);
	flush_scheduled_work();
	misc_deregister(&misc_cma3000_dev);
	input_unregister_device(cma3000_dev.idev);
	
	return 0;
}

static int
cma3000_d0x_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE
				     | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	strlcpy(info->type, "cma3000_d0x", I2C_NAME_SIZE);
	return 0;
}

static const struct i2c_device_id cma3000_d0x_id[] = {
	{ "cma3000_d0x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cma3000_d0x_id);

static struct i2c_driver cma3000_d0x_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name 	= "cma3000_d0x"
	},
	.probe		= cma3000_d0x_probe,
	.remove		= cma3000_d0x_remove,
	.id_table		= cma3000_d0x_id,
	.detect		= cma3000_d0x_detect,
	.address_data	= &addr_data,
};

static int __init
cma3000_d0x_init(void)
{
	return i2c_add_driver(&cma3000_d0x_driver);
}

static void __exit
cma3000_d0x_exit(void)
{
	i2c_del_driver(&cma3000_d0x_driver);
}

module_init(cma3000_d0x_init);
module_exit(cma3000_d0x_exit);


MODULE_DESCRIPTION("SOCLE CMA3000 3-AXIS ACCELEROMETER Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jerry hsieh");
