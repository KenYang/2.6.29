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
#include <sound/tlv.h>

#include <mach/asoc-codec.h>
#include <mach/uda1342ts.h>

#ifdef CONFIG_UDA1342TS_DEBUG
#define UDA1342TS_DEBUG(x) (x)
#else
#define UDA1342TS_DEBUG(x)
#endif

/*
 * uda1342ts register cache
 */
static const u16 uda1342ts_reg[0x30] = {
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000,
};

static unsigned int reg_mapping_table[] = {
	/* Playback */
	UDA1342TS_FUNC_SYSTEM,
	UDA1342TS_FUNC_SUB_SYSTEM,
	UDA1342TS_FUNC_DAC_FEATURES,
	UDA1342TS_FUNC_DAC_MASTER_VOLUME,
	UDA1342TS_FUNC_DAC_MIXER_VOLUME,
	UDA1342TS_FUNC_ADC_INPUT_MIXER_GAIN_CH1,
	UDA1342TS_FUNC_ADC_INPUT_MIXER_GAIN_CH2,
	UDA1342TS_FUNC_EVALUATION,
	/* Caputre */
	UDA1342TS_FUNC_SYSTEM,
	UDA1342TS_FUNC_SUB_SYSTEM,
	UDA1342TS_FUNC_DAC_FEATURES,
	UDA1342TS_FUNC_DAC_MASTER_VOLUME,
	UDA1342TS_FUNC_DAC_MIXER_VOLUME,
	UDA1342TS_FUNC_ADC_INPUT_MIXER_GAIN_CH1,
	UDA1342TS_FUNC_ADC_INPUT_MIXER_GAIN_CH2,
	UDA1342TS_FUNC_EVALUATION,
};

#if 0	/* FOR DEBUG */
void uda342ts_show_reg_cache(struct snd_soc_codec *codec)
{
	u16 *cache = codec->reg_cache;
	u32 i;

	printk("cache\n");
	for( i=0; i<ARRAY_SIZE(uda1342ts_reg); i++) {
		printk("%04X ", cache[i]);
		if(i%4==3) printk("\n");
	}
	printk("\n");
}

static int inline
uda1342ts_read(struct snd_soc_codec *codec, u8 reg)
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
	for( i=0; i<ARRAY_SIZE(uda1342ts_reg); i++) {
		printk("%04X ", uda1342ts_read(codec, i));
		if(i%4==3) printk("\n");
	}
	printk("\n");
}
#endif

/*
 * read uda1342ts register cache
 */
static inline unsigned int uda1342ts_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int index)
{
	u16 *cache = codec->reg_cache;
	unsigned int reg = reg_mapping_table[index];
	return cache[reg];
}

/*
 * write uda1342ts register cache
 */
static inline void uda1342ts_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	cache[reg] = value;
}


//static struct i2c_client *client_dac, *client_adc;

/*
 * write to the UDA1342TS register space
 */
static int uda1342ts_write(struct snd_soc_codec *codec, unsigned int index,
	unsigned int value)
{
	u8 data[3];
	unsigned int reg = reg_mapping_table[index];
	struct i2c_client *client = codec->control_data;
 UDA1342TS_DEBUG(printk("%s: reg %08X, val 0x%08X\n", __func__, reg, value));
	/* data is
	 *   data[0] is register offset
	 *   data[1] is MS byte
	 *   data[2] is LS byte
	 */
	data[0] = reg;
	data[1] = (value & 0xff00) >> 8;
	data[2] = value & 0x00ff;


	if( index >7)
		client->addr = UDA1342TS_SLAVE_ADDR_ADC;
	else
		client->addr = UDA1342TS_SLAVE_ADDR_DAC;
	
	/* the interpolator & decimator regs must only be written when the
	 * codec DAI is active.
	 */
	pr_debug("uda1342ts: hw write %x val %x\n", reg, value);
	if (codec->hw_write(client, data, 3) != 3) {
			printk("%s fucn: I2C WRITE ERROR\n", __func__);			
			return -1;
	}
	
	uda1342ts_write_reg_cache(codec, reg, value);
	
 UDA1342TS_DEBUG(uda342ts_show_reg_cache(codec));
// UDA1342TS_DEBUG(uda342ts_show_reg(codec));
	return 0;
}

static int uda1342ts_check_write(struct snd_soc_codec *codec, unsigned int index,
	unsigned int value)
{
	unsigned int val;
	val = uda1342ts_read_reg_cache(codec, index);
	if( value != val)
		uda1342ts_write(codec, index, value);
	return 0;
}

#define uda1342ts_dac_reset(c) uda1342ts_write(c, 0, 0x8000)
#define uda1342ts_adc_reset(c) uda1342ts_write(c, 8, 0x8000)

/* from -63 to 24 dB in 0.5 dB steps (-128...48) */
static DECLARE_TLV_DB_SCALE(dec_tlv, -6400, 50, 1);

static const struct snd_kcontrol_new uda1342ts_snd_controls[] = {
	SOC_SINGLE("Playback Switch", 0, 1, 1, 0),
	SOC_DOUBLE("PCM Playback Volume", 3, 0, 8, 224, 1),
	SOC_SINGLE("Mute", 2, 4, 1, 0),
	SOC_SINGLE("Tone Control - Switch", 2, 14, 3, 0),
	SOC_SINGLE("Tone Control - Bass", 2, 10, 0xf, 0),
	SOC_SINGLE("Tone Control - Treble", 2, 8, 3, 0),
	SOC_SINGLE("Capture Switch", 8, 9, 7, 0),
	SOC_DOUBLE_S8_TLV("Caputrue Volume", 13, -128, 48, dec_tlv),
	//SOC_DOUBLE_S8_TLV("Caputrue Volume CH2", 14, -128, 48, dec_tlv),
};

/* add non dapm controls */
static int uda1342ts_add_controls(struct snd_soc_codec *codec)
{
	int err, i;
//printk("%s: codec ptr %x\n", __func__, (u32)codec);

	for (i = 0; i < ARRAY_SIZE(uda1342ts_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&uda1342ts_snd_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

#if 0
static int uda1342ts_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, uda1342ts_dapm_widgets,
				  ARRAY_SIZE(uda1342ts_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}
#endif

#if 0
static int uda1342ts_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int iface;

	/* set up DAI based upon fmt */
	iface = uda1342ts_read_reg_cache(codec, UDA1342TS_IFACE);
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

	uda1342ts_write(codec, UDA1342TS_IFACE, iface);

	return 0;
}
#endif

/*
 * Flush reg cache
 * We can only write the interpolator and decimator registers
 * when the DAI is being clocked by the CPU DAI. It's up to the
 * machine and cpu DAI driver to do this before we are called.
 */
static int uda1342ts_pcm_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	//int reg, reg_start, reg_end, clk;
printk("Entered %s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		reg_start = UDA1342TS_MVOL;
		reg_end = UDA1342TS_MIXER;
	} else {
		reg_start = UDA1342TS_DEC;
		reg_end = UDA1342TS_AGC;
	}

	/* FIXME disable DAC_CLK */
	clk = uda1342ts_read_reg_cache(codec, UDA1342TS_CLK);
	uda1342ts_write(codec, UDA1342TS_CLK, clk & ~R00_DAC_CLK);

	for (reg = reg_start; reg <= reg_end; reg++) {
		pr_debug("uda1342ts: flush reg %x val %x:", reg,
				uda1342ts_read_reg_cache(codec, reg));
		uda1342ts_write(codec, reg, uda1342ts_read_reg_cache(codec, reg));
	}

	/* FIXME enable DAC_CLK */
	uda1342ts_write(codec, UDA1342TS_CLK, clk | R00_DAC_CLK);
#endif
//printk("Exit %s\n", __func__);
	return 0;
}

static int uda1342ts_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	u16 clk = uda1342ts_read_reg_cache(codec, UDA1342TS_CLK);

	/* set WSPLL power and divider if running from this clock */
	if (clk & R00_DAC_CLK) {
		int rate = params_rate(params);
		u16 pm = uda1342ts_read_reg_cache(codec, UDA1342TS_PM);
		clk &= ~0x3; /* clear SEL_LOOP_DIV */
		switch (rate) {
		case 6250 ... 12500:
			clk |= 0x0;
			break;
		case 12501 ... 25000:
			clk |= 0x1;
			break;
		case 25001 ... 50000:
			clk |= 0x2;
			break;
		case 50001 ... 100000:
			clk |= 0x3;
			break;
		}
		uda1342ts_write(codec, UDA1342TS_PM, R02_PON_PLL | pm);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		clk |= R00_EN_DAC | R00_EN_INT;
	else
		clk |= R00_EN_ADC | R00_EN_DEC;

	uda1342ts_write(codec, UDA1342TS_CLK, clk);
#endif
	return 0;
}

static void uda1342ts_pcm_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;

	u16 clk = uda1342ts_read_reg_cache(codec, UDA1342TS_CLK);

	/* shut down WSPLL power if running from this clock */
	if (clk & R00_DAC_CLK) {
		u16 pm = uda1342ts_read_reg_cache(codec, UDA1342TS_PM);
		uda1342ts_write(codec, UDA1342TS_PM, ~R02_PON_PLL & pm);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		clk &= ~(R00_EN_DAC | R00_EN_INT);
	else
		clk &= ~(R00_EN_ADC | R00_EN_DEC);

	uda1342ts_write(codec, UDA1342TS_CLK, clk);
#endif
}

static int uda1342ts_mute(struct snd_soc_dai *codec_dai, int mute)
{
#if 0
	struct snd_soc_codec *codec = codec_dai->codec;

	u16 mute_reg = uda1342ts_read_reg_cache(codec, UDA1342TS_DEEMP) & ~R13_MTM;

	/* FIXME: mute(codec,0) is called when the magician clock is already
	 * set to WSPLL, but for some unknown reason writing to interpolator
	 * registers works only when clocked by SYSCLK */
	u16 clk = uda1342ts_read_reg_cache(codec, UDA1342TS_CLK);
	uda1342ts_write(codec, UDA1342TS_CLK, ~R00_DAC_CLK & clk);
	if (mute)
		uda1342ts_write(codec, UDA1342TS_DEEMP, mute_reg | R13_MTM);
	else
		uda1342ts_write(codec, UDA1342TS_DEEMP, mute_reg);
	uda1342ts_write(codec, UDA1342TS_CLK, clk);
#endif
	return 0;
}

static int uda1342ts_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	int pm = uda1342ts_read_reg_cache(codec, 0);

	switch (level) {
		case SND_SOC_BIAS_ON:
		case SND_SOC_BIAS_PREPARE:
			uda1342ts_check_write(codec, 0, 
					pm | UDA1342TS_FUNC_SYSTEM_DAC_POWER_ON);
			break;
		case SND_SOC_BIAS_STANDBY:
			//uda342ts_show_reg_cache(codec);
			break;
		case SND_SOC_BIAS_OFF:
			uda1342ts_check_write(codec, 0, 
					pm & ~UDA1342TS_FUNC_SYSTEM_DAC_POWER_ON);
			break;
	}
	codec->bias_level = level;
	return 0;
}

#define UDA1342TS_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		       SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		       SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

struct snd_soc_dai uda1342ts_dai[] = {
	{
		.name = "UDA1342TS",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = UDA1342TS_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			//.formats = SNDRV_PCM_FMTBIT_S8,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = UDA1342TS_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = {
			.hw_params = uda1342ts_pcm_hw_params,
			.shutdown = uda1342ts_pcm_shutdown,
			.prepare = uda1342ts_pcm_prepare,
			.digital_mute = uda1342ts_mute,
			//.set_fmt = uda1342ts_set_dai_fmt,
		},
	},
	{ /* playback only - dual interface */
		.name = "UDA1342TS DAC",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = UDA1342TS_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = {
			.hw_params = uda1342ts_pcm_hw_params,
			.shutdown = uda1342ts_pcm_shutdown,
			.prepare = uda1342ts_pcm_prepare,
			.digital_mute = uda1342ts_mute,
			//.set_fmt = uda1342ts_set_dai_fmt,
		},
	},
	{ /* capture only - dual interface*/
		.name = "UDA1342TS ADC",
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = UDA1342TS_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = {
			.hw_params = uda1342ts_pcm_hw_params,
			.shutdown = uda1342ts_pcm_shutdown,
			.prepare = uda1342ts_pcm_prepare,
			//.set_fmt = uda1342ts_set_dai_fmt,
		},
	},
};
EXPORT_SYMBOL_GPL(uda1342ts_dai);

static int uda1342ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	uda1342ts_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int uda1342ts_resume(struct platform_device *pdev)
{
#if 0
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(uda1342ts_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}
	uda1342ts_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	uda1342ts_set_bias_level(codec, codec->suspend_bias_level);
#endif
	return 0;
}

/*
 * initialise the UDA1342TS driver
 * register mixer and dsp interfaces with the kernel
 */
static int uda1342ts_init(struct snd_soc_device *socdev, int dac_clk)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;
//printk("Entered %s, codec ptr 0x%08X\n", __func__, (u32)codec);
	codec->name = "UDA1342TS I2C CODEC";
	codec->owner = THIS_MODULE;
	codec->read = uda1342ts_read_reg_cache;
	codec->write = uda1342ts_write;
	codec->set_bias_level = uda1342ts_set_bias_level;
	codec->dai = uda1342ts_dai;
	codec->num_dai = ARRAY_SIZE(uda1342ts_dai);
	codec->reg_cache = kmemdup(uda1342ts_reg, sizeof(uda1342ts_reg),
				   GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = ARRAY_SIZE(uda1342ts_reg);
	codec->reg_cache_step = 1;
	uda1342ts_dac_reset(codec);
	uda1342ts_adc_reset(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		pr_err("uda1342ts: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	uda1342ts_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	uda1342ts_write(codec, 0, 0x1a62);
	uda1342ts_write(codec, 2, 0xc300);
	uda1342ts_write(codec, 5, 0x30);
	uda1342ts_write(codec, 8, 0x1a62);
	uda1342ts_write(codec, 10, 0xc300);
	uda1342ts_write(codec, 13, 0x30);
	
#if 0
	/* set clock input */
	switch (dac_clk) {
	case UDA1342TS_DAC_CLK_SYSCLK:
		uda1342ts_write(codec, UDA1342TS_CLK, 0);
		break;
	case UDA1342TS_DAC_CLK_WSPLL:
		uda1342ts_write(codec, UDA1342TS_CLK, R00_DAC_CLK);
		break;
	}
#endif

	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		pr_err("uda1342ts: failed to register card\n");
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

static const struct i2c_device_id uda1342ts_id[] = {
	{ "UDA1342TS CODEC", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, uda1342ts_id);

static struct snd_soc_device *uda1342ts_socdev;

static int uda1342ts_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = uda1342ts_socdev;
	struct asoc_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	int ret;
//printk("Entered %s, id->name %s\n", __func__, id->name);

	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = uda1342ts_init(socdev, setup->dac_clk);
	if (ret < 0)
		pr_err("uda1342ts: failed to initialise UDA1342TS\n");
//printk("Exit %s\n", __func__);
	return ret;
}

static int uda1342ts_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	kfree(codec->reg_cache);
	return 0;
}

#if 0
static int uda1342ts_i2c_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	const char *name = "";
printk("Entered %s, client addr 0x%08X\n", __func__, (u32)client);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if (kind < 0) {
		if( client->addr == UDA1342TS_SLAVE_ADDR_ADC)
			name = "UDA1342TS ADC";
		else if(client->addr == UDA1342TS_SLAVE_ADDR_DAC)
			name = "UDA1342TS DAC";
		else
			return -ENODEV;
	}

	strlcpy(info->type, name, I2C_NAME_SIZE);

	return 0;
}
#endif

static struct i2c_driver uda1342ts_i2c_driver = {
	.driver = {
		.name =  "UDA1342TS I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe =    uda1342ts_i2c_probe,
	.remove =   uda1342ts_i2c_remove,

	.id_table = uda1342ts_id,
	//.detect = uda1342ts_i2c_detect,
};
static int uda1342ts_add_i2c_device(struct platform_device *pdev,
				  const struct asoc_setup_data *setup)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;
	
	ret = i2c_add_driver(&uda1342ts_i2c_driver);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't add i2c driver\n");
		return ret;
	}

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	strlcpy(info.type, "UDA1342TS CODEC", I2C_NAME_SIZE);

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
	i2c_del_driver(&uda1342ts_i2c_driver);
	return -ENODEV;
}
#endif

static int uda1342ts_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct asoc_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret;

	//pr_info("UDA1342TS Audio Codec %s", UDA1342TS_VERSION);
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

	uda1342ts_socdev = socdev;

	ret = -ENODEV;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		codec->hw_write = (hw_write_t)i2c_master_send;
		ret = uda1342ts_add_i2c_device(pdev, setup);
	}
#endif

	/* uda1342ts init */
	uda1342ts_add_controls(codec);
#if 0
	uda1342ts_add_widgets(codec);
#endif

	if (ret != 0)
		kfree(codec);
	//printk("Exit %s\n", __func__);
	return ret;
}

/* power down chip */
static int uda1342ts_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		uda1342ts_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
#if 0
	snd_soc_dapm_free(socdev);
#endif
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_unregister_device(codec->control_data);
	i2c_del_driver(&uda1342ts_i2c_driver);
#endif
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_uda1342ts = {
	.probe	= 	uda1342ts_probe,
	.remove	= 	uda1342ts_remove,
	.suspend	= 	uda1342ts_suspend,
	.resume	=	uda1342ts_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_uda1342ts);

static int __init uda1342ts_modinit(void)
{
	return snd_soc_register_dais(uda1342ts_dai, ARRAY_SIZE(uda1342ts_dai));
}
module_init(uda1342ts_modinit);

static void __exit uda1342ts_exit(void)
{
	snd_soc_unregister_dais(uda1342ts_dai, ARRAY_SIZE(uda1342ts_dai));
}
module_exit(uda1342ts_exit);

MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("Audio support for codec Philips UDA1342TS");
MODULE_LICENSE("GPL");
