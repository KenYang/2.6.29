#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "socle-pcm.h"

#include <mach/uda1342ts.h>

extern struct snd_soc_dai socle_i2s_dai;
extern struct snd_soc_dai wm9714_dai;

static struct snd_soc_dai_link socle_dai[] = {
	{
		.name = "WM9714",
		.stream_name = "WM9714 HiFi",
		.cpu_dai = &socle_i2s_dai,
		.codec_dai = &wm9714_dai,
	},
};

static struct snd_soc_card socle_wm9714 = {
	.name = "Socle WM9714",
	.platform = &socle_snd_soc_platform,
	.dai_link = socle_dai,
	.num_links = ARRAY_SIZE(socle_dai),
};

extern struct snd_soc_codec_device soc_codec_dev_wm9714;

static struct snd_soc_device socle_snd_wm9714_devdata = {
	.card = &socle_wm9714,
	.codec_dev = &soc_codec_dev_wm9714,
};

static struct platform_device *socle_snd_wm9714_device;

static int __init socle_asoc_init(void)
{
	int ret;
	
	printk("Socle WM9714 ALSA SoC\n");
	socle_snd_wm9714_device = platform_device_alloc("soc-audio", -1);
	if (!socle_snd_wm9714_device)
		return -ENOMEM;

	platform_set_drvdata(socle_snd_wm9714_device,
				&socle_snd_wm9714_devdata);
	socle_snd_wm9714_devdata.dev = &socle_snd_wm9714_device->dev;
	
	ret = platform_device_add(socle_snd_wm9714_device);

	if (ret)
		platform_device_put(socle_snd_wm9714_device);

	return ret;
}

static void __exit socle_asoc_exit(void)
{
	platform_device_unregister(socle_snd_wm9714_device);
}

module_init(socle_asoc_init);
module_exit(socle_asoc_exit);

/* Module information */
MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("Socle uda1342ts ALSA SoC");
MODULE_LICENSE("GPL");
