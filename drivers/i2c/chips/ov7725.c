#include <linux/i2c.h>
#include <mach/gpio.h>
#include <linux/delay.h>

//#define	DEBUG

#ifdef DEBUG
	#define	dbg(fmt, arg...)	printk(fmt, ##arg)
#else
	#define	dbg(fmt, arg...)	
#endif

#define	OV7725_SLAVE_ADDRESS	0x21

#define	OV7725_RESET1_GPIO	PM
#define	OV7725_RESET1_MASK	7

static const unsigned short normal_i2c[] = { OV7725_SLAVE_ADDRESS, I2C_CLIENT_END };

I2C_CLIENT_INSMOD_1(ov7725);

struct ov7725_data {
	struct i2c_client	*fake_client;
	struct mutex		updata_lock;
};

static int ov7725_reset(void)
{
	dbg("ov7725 reset signal\n");
	socle_gpio_claim_lock();
	socle_gpio_direction_output_with_mask(OV7725_RESET1_GPIO, OV7725_RESET1_MASK);
	socle_gpio_set_value_with_mask(OV7725_RESET1_GPIO, 0<<OV7725_RESET1_MASK, 1<<OV7725_RESET1_MASK);
	socle_gpio_release_lock();

	mdelay(10);

	socle_gpio_claim_lock();
	socle_gpio_set_value_with_mask(OV7725_RESET1_GPIO, 1<<OV7725_RESET1_MASK, 1<<OV7725_RESET1_MASK);
	socle_gpio_release_lock();

	return 0;
}

static int ov7725_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter=client->adapter;

	dbg("ov7725 detect\n");

	if(!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;
	
	if(!(client->addr && OV7725_SLAVE_ADDRESS))
		return -ENODEV;

	strlcpy(info->type, "ov7725", I2C_NAME_SIZE);

	return 0;
}

static int ov7725_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov7725_data *data;
	int err;

	dbg("ov7725 probe start\n");

	if (!(data=kzalloc(sizeof(struct ov7725_data), GFP_KERNEL)))
		return -ENOMEM;
	
	data->fake_client = i2c_new_dummy(client->adapter, client->addr + 1);
	if (!data->fake_client) {
		err = -ENOMEM;
		goto exit_kfree;
	}

	i2c_set_clientdata(client ,data);
	mutex_init(&data->updata_lock);

	return 0;

exit_kfree:
	kfree(data);
	return err;
}

static int ov7725_remove(struct i2c_client *client)
{
	struct ov7725_data *data=i2c_get_clientdata(client);

	i2c_unregister_device(data->fake_client);

	kfree(data);

	return 0;
}

static const struct i2c_device_id ov7725_id[] = {
	{ "ov7725", 0 },
	{ }
};

static struct i2c_driver ov7725_driver = {
	.driver = {
		.name = "ov7725",
	},
	.probe	= ov7725_probe,
	.remove = ov7725_remove,
	.id_table = ov7725_id,
	.detect = ov7725_detect,
	.address_data = &addr_data,
};

static int __init ov7725_init(void)
{
	dbg("ov7725 init\n");
	ov7725_reset();
	return i2c_add_driver(&ov7725_driver);
}

static void __exit ov7725_exit(void)
{
	i2c_del_driver(&ov7725_driver);
}

MODULE_LICENSE("GPL");

module_init(ov7725_init);
module_exit(ov7725_exit);

