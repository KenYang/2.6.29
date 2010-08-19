#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "socle-pcm.h"

#include <mach/asoc-codec.h>

extern struct snd_soc_dai socle_i2s_dai;
extern struct snd_soc_dai wm8961_dai[];

static struct snd_soc_dai_link socle_dai[] = {
	{
		.name = "WM8961",
		.stream_name = "WM8961 HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &wm8961_dai[0],
	},
};

static struct snd_soc_card socle_wm8961 = {
	.name = "Socle WM8961",
	.platform = &socle_snd_soc_platform,
	.dai_link = socle_dai,
	.num_links = ARRAY_SIZE(socle_dai),
};

struct asoc_setup_data wm8961_codec_data[] = {
	{
		.i2c_bus	= 0,
		.i2c_address	= 0x94,
		.dac_clk	= 0,
	},
};

extern struct snd_soc_codec_device soc_codec_dev_wm8961;

static struct snd_soc_device socle_snd_wm8961_devdata = {
	.card = &socle_wm8961,
	.codec_dev = &soc_codec_dev_wm8961,
};

static struct platform_device *socle_snd_wm8961_device;

static int __init socle_asoc_init(void)
{
	int ret;
	
	printk("Socle WM8961 ALSA SoC\n");
	socle_snd_wm8961_device = platform_device_alloc("soc-audio", -1);
	if (!socle_snd_wm8961_device)
		return -ENOMEM;

	platform_set_drvdata(socle_snd_wm8961_device,
				&socle_snd_wm8961_devdata);
	socle_snd_wm8961_devdata.dev = &socle_snd_wm8961_device->dev;
	socle_snd_wm8961_devdata.codec_data = (void *)(&wm8961_codec_data);
	
	ret = platform_device_add(socle_snd_wm8961_device);

	if (ret)
		platform_device_put(socle_snd_wm8961_device);

	return ret;
}

static void __exit socle_asoc_exit(void)
{
	platform_device_unregister(socle_snd_wm8961_device);
}

module_init(socle_asoc_init);
module_exit(socle_asoc_exit);

/* Module information */
MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("Socle wm8961 ALSA SoC");
MODULE_LICENSE("GPL");
