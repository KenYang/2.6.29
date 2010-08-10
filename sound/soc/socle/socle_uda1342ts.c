#if 1
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "socle-pcm.h"

#include <mach/asoc-codec.h>
#include <mach/uda1342ts.h>

extern struct snd_soc_dai socle_i2s_dai;
extern struct snd_soc_dai uda1342ts_dai[];

static struct snd_soc_dai_link socle_dai[] = {
	{
		.name = "UDA1342TS",
		.stream_name = "UDA1342TS HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &uda1342ts_dai[0],
	},
#if 0
	{
		.name = "UDA1342TS DAC",
		.stream_name = "UDA1342TS DAC HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &uda1342ts_dai[1],
	},
	{
		.name = "UDA1342TS ADC",
		.stream_name = "UDA1342TS ADC HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &uda1342ts_dai[2],
	},
#endif
};

static struct snd_soc_card socle_uda1342ts = {
	.name = "Socle UDA1342TS",
	.platform = &socle_snd_soc_platform,
	.dai_link = socle_dai,
	.num_links = ARRAY_SIZE(socle_dai),
};

struct asoc_setup_data uda1342ts_codec_data[] = {
	{
#if defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_SCDK)
		.i2c_bus		= 0,
#else
		.i2c_bus		= -1,
#endif
		.i2c_address	= UDA1342TS_SLAVE_ADDR_DAC,
		.dac_clk		= 0,
	},
#if 0
	{
		.i2c_bus		= 0,
		.i2c_address	= UDA1342TS_SLAVE_ADDR_ADC,
		.dac_clk		= 0,
	},
#endif
};

extern struct snd_soc_codec_device soc_codec_dev_uda1342ts;

static struct snd_soc_device socle_snd_uda1342ts_devdata = {
	.card = &socle_uda1342ts,
	.codec_dev = &soc_codec_dev_uda1342ts,
};

static struct platform_device *socle_snd_uda1342ts_device;

static int __init socle_asoc_init(void)
{
	int ret;
	
	printk("Socle UDA1342TS ALSA SoC\n");
	socle_snd_uda1342ts_device = platform_device_alloc("soc-audio", -1);
	if (!socle_snd_uda1342ts_device)
		return -ENOMEM;

	platform_set_drvdata(socle_snd_uda1342ts_device,
				&socle_snd_uda1342ts_devdata);
	socle_snd_uda1342ts_devdata.dev = &socle_snd_uda1342ts_device->dev;
	socle_snd_uda1342ts_devdata.codec_data = (void *)(&uda1342ts_codec_data);
	
	ret = platform_device_add(socle_snd_uda1342ts_device);

	if (ret)
		platform_device_put(socle_snd_uda1342ts_device);

	return ret;
}

static void __exit socle_asoc_exit(void)
{
	platform_device_unregister(socle_snd_uda1342ts_device);
}

module_init(socle_asoc_init);
module_exit(socle_asoc_exit);

/* Module information */
MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("Socle uda1342ts ALSA SoC");
MODULE_LICENSE("GPL");

#else
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "socle-pcm.h"

#include <mach/uda1342ts.h>

extern struct snd_soc_dai socle_i2s_dai;
extern struct snd_soc_dai uda1342ts_dai[];

static struct snd_soc_dai_link socle_dai[] = {
	{
		.name = "UDA1342TS",
		.stream_name = "UDA1342TS HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &uda1342ts_dai[0],
	},
	{
		.name = "UDA1342TS DAC",
		.stream_name = "UDA1342TS DAC HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &uda1342ts_dai[1],
	},
	{
		.name = "UDA1342TS ADC",
		.stream_name = "UDA1342TS ADC HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &uda1342ts_dai[2],
	},
};

static struct snd_soc_card socle_uda1342ts_dac = {
	.name = "Socle UDA1342TS",
	.platform = &socle_snd_soc_platform,
	.dai_link = &socle_dai[1],
	.num_links = 1,
};

static struct snd_soc_card socle_uda1342ts_adc = {
	.name = "Socle UDA1342TS",
	.platform = &socle_snd_soc_platform,
	.dai_link = &socle_dai[2],
	.num_links = 1,
};

struct uda1342ts_setup_data uda1342ts_codec_data[] = {
	{
		.i2c_bus		= 0,
		.i2c_address	= 0x1b,
		.dac_clk		= 0,
	},
	{
		.i2c_bus		= 0,
		.i2c_address	= 0x1a,
		.dac_clk		= 0,
	},
};

extern struct snd_soc_codec_device soc_codec_dev_uda1342ts;

static struct snd_soc_device socle_snd_uda1342ts_dac_devdata = {
	.card = &socle_uda1342ts_dac,
	.codec_dev = &soc_codec_dev_uda1342ts,
};

static struct snd_soc_device socle_snd_uda1342ts_adc_devdata = {
	.card = &socle_uda1342ts_adc,
	.codec_dev = &soc_codec_dev_uda1342ts,
};

static struct platform_device *socle_snd_uda1342ts_dac_device;
static struct platform_device *socle_snd_uda1342ts_adc_device;

static int __init socle_asoc_init(void)
{
	int ret;
	
	printk("Socle UDA1342TS ALSA SoC 1\n");
	socle_snd_uda1342ts_dac_device = platform_device_alloc("soc-audio", -1);
	if (!socle_snd_uda1342ts_dac_device)
		return -ENOMEM;
	printk("Socle UDA1342TS ALSA SoC 2\n");
	socle_snd_uda1342ts_adc_device = platform_device_alloc("soc-audio", -1);
	if (!socle_snd_uda1342ts_adc_device)
		return -ENOMEM;

	platform_set_drvdata(socle_snd_uda1342ts_dac_device,
				&socle_snd_uda1342ts_dac_devdata);
	socle_snd_uda1342ts_dac_devdata.codec_data = (void *)(&uda1342ts_codec_data[0]);

	platform_set_drvdata(socle_snd_uda1342ts_adc_device,
				&socle_snd_uda1342ts_adc_devdata);
	socle_snd_uda1342ts_adc_devdata.codec_data = (void *)(&uda1342ts_codec_data[1]);
	
	ret = platform_device_add(socle_snd_uda1342ts_dac_device);
	if (ret)
		platform_device_put(socle_snd_uda1342ts_dac_device);

	ret = platform_device_add(socle_snd_uda1342ts_adc_device);
	if (ret)
		platform_device_put(socle_snd_uda1342ts_adc_device);

	return ret;
}

static void __exit socle_asoc_exit(void)
{
	platform_device_unregister(socle_snd_uda1342ts_dac_device);
	platform_device_unregister(socle_snd_uda1342ts_adc_device);
}

module_init(socle_asoc_init);
module_exit(socle_asoc_exit);

/* Module information */
MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("Socle uda1342ts ALSA SoC");
MODULE_LICENSE("GPL");

#endif
