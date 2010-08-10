#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/initval.h>
#include <sound/info.h>
#include <sound/soc.h>

#include <mach/asoc-codec.h>
#include <mach/ms6335.h>

//#define CONFIG_MS6335_DEBUG
#ifdef CONFIG_MS6335_DEBUG
#define MS6335_DEBUG(x) (x)
#else
#define MS6335_DEBUG(x)
#endif

/*
 * ms6335 register cache
 */
static const u16 ms6335_reg[0x8] = {
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000,
};

#define MS6335_VOLUME_CTRL	0
#define MS6335_VOLUME_CTRL_L	1
#define MS6335_VOLUME_CTRL_R	2
#define MS6335_POWER_CTRL		3
#define MS6335_AUDIO_FORMAT	4

static unsigned int reg_mapping_table[] = {
	MS6335_VOLUME_CTRL,
	MS6335_VOLUME_CTRL_L,
	MS6335_VOLUME_CTRL_R,
	MS6335_POWER_CTRL,
	MS6335_AUDIO_FORMAT,
};

/* DEBUG */
void uda342ts_show_reg_cache(struct snd_soc_codec *codec)
{
	u16 *cache = codec->reg_cache;
	u32 i;

	printk("cache\n");
	for( i=0; i<ARRAY_SIZE(ms6335_reg); i++) {
		printk("%04X ", cache[i]);
		if(i%4==3) printk("\n");
	}
	printk("\n");
}
#if 0	/* FOR DEBUG */
static int inline
ms6335_read(struct snd_soc_codec *codec, u8 reg)
{
	struct i2c_client *client = codec->control_data;
	int ret;
	struct i2c_msg msg[2];
	u8 buf =reg;
	u8 ret_buf[2];
	int ret_val;

	memset((void *)msg, 0x00, 2*sizeof(struct i2c_msg));
	msg[0].addr = client->addr;
	msg[1].addr = client->addr;
	msg[0].buf = &buf;
	msg[0].len = 1;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = ret_buf;
	msg[1].len = 2;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2)
		return -1;
	ret_val = (ret_buf[0] << 8) | ret_buf[1];
	return ret_val; 
}

void uda342ts_show_reg(struct snd_soc_codec *codec)
{
	u16 *cache = codec->reg_cache;
	u32 i;

	printk("register\n");
	for( i=0; i<ARRAY_SIZE(ms6335_reg); i++) {
		printk("%04X ", ms6335_read(codec, i));
		if(i%4==3) printk("\n");
	}
	printk("\n");
}
#endif

/*
 * read ms6335 register cache
 */
static inline unsigned int ms6335_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int index)
{
	u16 *cache = codec->reg_cache;
	unsigned int reg = reg_mapping_table[index];
	return cache[reg];
}

/*
 * write ms6335 register cache
 */
static inline void ms6335_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	cache[reg] = value;
}


/*
 * write to the MS6335 register space
 */
static int ms6335_write(struct snd_soc_codec *codec, unsigned int numid,
	unsigned int value)
{
	u8 data;
	unsigned int reg = reg_mapping_table[numid];
	struct i2c_client *client = codec->control_data;
	int ret;
MS6335_DEBUG(printk("%s: reg %08X, val 0x%08X\n", __func__, reg, value));
//printk("client->addr 0x%04X\n", client->addr);
 
	data = (u8)(reg << 5 | value);

	/* the interpolator & decimator regs must only be written when the
	 * codec DAI is active.
	 */
	pr_debug("ms6335: hw write %x val %x\n", reg, value);
	if ( (ret=codec->hw_write(client, &data, 1)) != 1) {
//printk("-----%s ERROR ret %d\n", __func__, ret);			
			return -1;
	}

	ms6335_write_reg_cache(codec, reg, value);
	
 MS6335_DEBUG(uda342ts_show_reg_cache(codec));
 //MS6335_DEBUG(uda342ts_show_reg(codec));
	return 0;
}

static const struct snd_kcontrol_new ms6335_snd_controls[] = {
	SOC_SINGLE("Playback Switch", 3, 2, 1, 0),
	SOC_SINGLE("PCM Playback Volume", 0, 0, 31, 0),
	SOC_SINGLE("Mute", 3, 0, 1, 0),
};

/* add non dapm controls */
static int ms6335_add_controls(struct snd_soc_codec *codec)
{
	int err, i;
//printk("%s: codec ptr %x\n", __func__, (u32)codec);

	for (i = 0; i < ARRAY_SIZE(ms6335_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&ms6335_snd_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

#if 0
static int ms6335_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, ms6335_dapm_widgets,
				  ARRAY_SIZE(ms6335_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}
#endif

#if 0
static int ms6335_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int iface;

	/* set up DAI based upon fmt */
	iface = ms6335_read_reg_cache(codec, MS6335_IFACE);
	iface &= ~(R01_SFORI_MASK | R01_SIM | R01_SFORO_MASK);

	/* FIXME: how to select I2S for DATAO and MSB for DATAI correctly? */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= R01_SFORI_I2S | R01_SFORO_I2S;
		break;
	case SND_SOC_DAIFMT_LSB:
		iface |= R01_SFORI_LSB16 | R01_SFORO_I2S;
		break;
	case SND_SOC_DAIFMT_MSB:
		iface |= R01_SFORI_MSB | R01_SFORO_I2S;
	}

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBM_CFM)
		iface |= R01_SIM;

	ms6335_write(codec, MS6335_IFACE, iface);

	return 0;
}
#endif

/*
 * Flush reg cache
 * We can only write the interpolator and decimator registers
 * when the DAI is being clocked by the CPU DAI. It's up to the
 * machine and cpu DAI driver to do this before we are called.
 */
static int ms6335_pcm_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int reg, reg_start, reg_end, clk;
//printk("Entered %s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		reg_start = MS6335_MVOL;
		reg_end = MS6335_MIXER;
	} else {
		reg_start = MS6335_DEC;
		reg_end = MS6335_AGC;
	}

	/* FIXME disable DAC_CLK */
	clk = ms6335_read_reg_cache(codec, MS6335_CLK);
	ms6335_write(codec, MS6335_CLK, clk & ~R00_DAC_CLK);

	for (reg = reg_start; reg <= reg_end; reg++) {
		pr_debug("ms6335: flush reg %x val %x:", reg,
				ms6335_read_reg_cache(codec, reg));
		ms6335_write(codec, reg, ms6335_read_reg_cache(codec, reg));
	}

	/* FIXME enable DAC_CLK */
	ms6335_write(codec, MS6335_CLK, clk | R00_DAC_CLK);
#endif
//printk("Exit %s\n", __func__);
	return 0;
}

static int ms6335_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	return 0;
}

static void ms6335_pcm_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	//ms6335_write(codec, 4, 3);
#endif
}

static int ms6335_mute(struct snd_soc_dai *codec_dai, int mute)
{
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;
	//ms6335_write(codec, 3, 1);
#endif
	return 0;
}

static int ms6335_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
//printk("-----codec pointer %08X\n", (u32) codec);
	switch (level) {
		case SND_SOC_BIAS_ON:
		case SND_SOC_BIAS_PREPARE:
			break;
		case SND_SOC_BIAS_STANDBY:
			break;
		case SND_SOC_BIAS_OFF:
			break;
	}
	codec->bias_level = level;
	return 0;
}

#define MS6335_RATES SNDRV_PCM_RATE_44100

struct snd_soc_dai ms6335_dai[] = {
	{ /* playback only */
		.name = "MS6335",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MS6335_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MS6335_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = {
			.hw_params = ms6335_pcm_hw_params,
			.shutdown = ms6335_pcm_shutdown,
			.prepare = ms6335_pcm_prepare,
			.digital_mute = ms6335_mute,
			//.set_fmt = ms6335_set_dai_fmt,
		},
	},
};
EXPORT_SYMBOL_GPL(ms6335_dai);

static int ms6335_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 data = 0x83;
	if( codec->hw_write(codec->control_data, &data, 1) != 1)
		printk("----line %d----I2C WRITE ERROR\n", __LINE__);
	data = 0x6F;
	if( codec->hw_write(codec->control_data, &data, 1) != 1)
		printk("----line %d----I2C WRITE ERROR\n", __LINE__);
	ms6335_set_bias_level(codec, SND_SOC_BIAS_OFF);
#endif
	return 0;
}

static int ms6335_resume(struct platform_device *pdev)
{
#if 1
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data = 0x81;
	u16 *cache = codec->reg_cache;

//printk("----%s func----line %d----\n", __func__, __LINE__);
	ms6335_write(codec, 4, 1);
        ms6335_write(codec, 3, 0);
#if 1
	/* Sync reg_cache with the hardware */
	for (i = 0; i < 1; i++) {
		data = (u8)(cache[i] & 0x0ff);
		codec->hw_write(codec->control_data, &data, 1);
	}
	//ms6335_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	//ms6335_set_bias_level(codec, codec->suspend_bias_level);
#endif
#endif
	return 0;
}

/*
 * initialise the MS6335 driver
 * register mixer and dsp interfaces with the kernel
 */
static int ms6335_init(struct snd_soc_device *socdev, int dac_clk)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;
//printk("Entered %s, codec ptr 0x%08X\n", __func__, (u32)codec);
	codec->name = "MS6335 I2C CODEC";
	codec->owner = THIS_MODULE;
	codec->read = ms6335_read_reg_cache;
	codec->write = ms6335_write;
	codec->set_bias_level = ms6335_set_bias_level;
	codec->dai = ms6335_dai;
	codec->num_dai = ARRAY_SIZE(ms6335_dai);
	codec->reg_cache = (void *)ms6335_reg;
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = ARRAY_SIZE(ms6335_reg);
	codec->reg_cache_step = 1;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		pr_err("ms6335: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	ms6335_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
#if 0
	/* set clock input */
	switch (dac_clk) {
	case MS6335_DAC_CLK_SYSCLK:
		ms6335_write(codec, MS6335_CLK, 0);
		break;
	case MS6335_DAC_CLK_WSPLL:
		ms6335_write(codec, MS6335_CLK, R00_DAC_CLK);
		break;
	}
#endif
#if 1
	ms6335_write(codec, 4, 1);
	ms6335_write(codec, 3, 0);
	ms6335_write(codec, 0, 0xf);
#endif
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		pr_err("ms6335: failed to register card\n");
		goto card_err;
	}
//printk("Exit %s\n", __func__);
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
#if 0
	snd_soc_dapm_free(socdev);
#endif
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static const struct i2c_device_id ms6335_id[] = {
	{ "MS6335 CODEC", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ms6335_id);

static struct snd_soc_device *ms6335_socdev;

static int ms6335_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = ms6335_socdev;
	struct asoc_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;
//printk("Entered %s, id->name %s\n", __func__, id->name);

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = ms6335_init(socdev, setup->dac_clk);
	if (ret < 0)
		pr_err("ms6335: failed to initialise MS6335\n");
//printk("Exit %s\n", __func__);
	return ret;
}

static int ms6335_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	kfree(codec->reg_cache);
	return 0;
}

#if 0
static int ms6335_i2c_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
#if 1
	struct i2c_adapter *adapter = client->adapter;
	const char *name = "";
//printk("Entered %s, client addr 0x%08X\n", __func__, (u32)client);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	//name = "MS6335 CODEC";
	name = ms6335_id[0].name;

	strlcpy(info->type, name, I2C_NAME_SIZE);
#endif
	return 0;
}
#endif

static struct i2c_driver ms6335_i2c_driver = {
	.driver = {
		.name =  "MS6335 I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe =    ms6335_i2c_probe,
	.remove =   ms6335_i2c_remove,

	.id_table = ms6335_id,
	//.detect = ms6335_i2c_detect,
};
static int ms6335_add_i2c_device(struct platform_device *pdev,
				  const struct asoc_setup_data *setup)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;
	
	ret = i2c_add_driver(&ms6335_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	//strlcpy(info.type, "MS6335 CODEC", I2C_NAME_SIZE);
	strlcpy(info.type, ms6335_id[0].name, I2C_NAME_SIZE);

	adapter = i2c_get_adapter(setup->i2c_bus);
	if (!adapter) {
		dev_err(&pdev->dev, "can't get i2c adapter %d\n",
			setup->i2c_bus);
		goto err_driver;
	}

	client = i2c_new_device(adapter, &info);
	i2c_put_adapter(adapter);
	if (!client) {
		dev_err(&pdev->dev, "can't add i2c device at 0x%x\n",
			(unsigned int)info.addr);
		goto err_driver;
	}
	
	return 0;

err_driver:
	i2c_del_driver(&ms6335_i2c_driver);
	return -ENODEV;
}
#endif

static int ms6335_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct asoc_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret;

	//pr_info("MS6335 Audio Codec %s", MS6335_VERSION);
//printk("Entered %s\n", __func__);
	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;
//printk("%s: codec ptr 0x%08X\n", __func__, (u32)codec);
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ms6335_socdev = socdev;

	ret = -ENODEV;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		codec->hw_write = (hw_write_t)i2c_master_send;
		ret = ms6335_add_i2c_device(pdev, setup);
	}
#endif

	/* ms6335 init */
	ms6335_add_controls(codec);
#if 0
	ms6335_add_widgets(codec);
#endif

	if (ret != 0)
		kfree(codec);
//printk("Exit %s\n", __func__);
	return ret;
}

/* power down chip */
static int ms6335_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		ms6335_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
#if 0
	snd_soc_dapm_free(socdev);
#endif
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_unregister_device(codec->control_data);
	i2c_del_driver(&ms6335_i2c_driver);
#endif
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_ms6335 = {
	.probe	= 	ms6335_probe,
	.remove	= 	ms6335_remove,
	.suspend	= 	ms6335_suspend,
	.resume	=	ms6335_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ms6335);

static int __init ms6335_modinit(void)
{
	return snd_soc_register_dais(ms6335_dai, ARRAY_SIZE(ms6335_dai));
}
module_init(ms6335_modinit);

static void __exit ms6335_exit(void)
{
	snd_soc_unregister_dais(ms6335_dai, ARRAY_SIZE(ms6335_dai));
}
module_exit(ms6335_exit);

MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("Audio support for codec Philips MS6335");
MODULE_LICENSE("GPL");
