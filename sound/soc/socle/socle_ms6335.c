#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "socle-pcm.h"

#include <mach/asoc-codec.h>
#include <mach/ms6335.h>

extern struct snd_soc_dai socle_i2s_dai;
extern struct snd_soc_dai ms6335_dai[];

static struct snd_soc_dai_link socle_dai[] = {
	{
		.name = "MS6335",
		.stream_name = "MS6335 HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &ms6335_dai[0],
	},
};

static struct snd_soc_card socle_ms6335 = {
	.name = "Socle MS6335",
	.platform = &socle_snd_soc_platform,
	.dai_link = socle_dai,
	.num_links = ARRAY_SIZE(socle_dai),
};

struct asoc_setup_data ms6335_codec_data[] = {
	{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
		.i2c_bus		= 0,
#elif defined(CONFIG_ARCH_MDK_3D)
		.i2c_bus                = 1,
#else
		.i2c_bus                = -1, 
#endif
		.i2c_address	= MS6335_SLAVE_ADDR_DAC,
		.dac_clk		= 0,
	},
};

extern struct snd_soc_codec_device soc_codec_dev_ms6335;

static struct snd_soc_device socle_snd_ms6335_devdata = {
	.card = &socle_ms6335,
	.codec_dev = &soc_codec_dev_ms6335,
};

static struct platform_device *socle_snd_ms6335_device;

static int __init socle_asoc_init(void)
{
	int ret;
	
	printk("Socle MS6335 ALSA SoC\n");
	socle_snd_ms6335_device = platform_device_alloc("soc-audio", -1);
	if (!socle_snd_ms6335_device)
		return -ENOMEM;

	platform_set_drvdata(socle_snd_ms6335_device,
				&socle_snd_ms6335_devdata);
	socle_snd_ms6335_devdata.dev = &socle_snd_ms6335_device->dev;
	socle_snd_ms6335_devdata.codec_data = (void *)(&ms6335_codec_data);
	
	ret = platform_device_add(socle_snd_ms6335_device);

	if (ret)
		platform_device_put(socle_snd_ms6335_device);

	return ret;
}

static void __exit socle_asoc_exit(void)
{
	platform_device_unregister(socle_snd_ms6335_device);
}

module_init(socle_asoc_init);
module_exit(socle_asoc_exit);

/* Module information */
MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("Socle ms6335 ALSA SoC");
MODULE_LICENSE("GPL");
