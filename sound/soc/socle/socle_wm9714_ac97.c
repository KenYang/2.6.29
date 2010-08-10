#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "socle-pcm.h"
#include "../codecs/wm9714.h"
#include "../codecs/alc658.h"

//#define CONFIG_AC97_PLATFORM_DEBUG
#ifdef CONFIG_AC97_PLATFORM_DEBUG
	#define DBG(fmt, args...) printk("socle-ac97-platform : %s(): " fmt, __FUNCTION__, ## args)
#else	
	#define DBG(fmt, args...)
#endif

//static struct snd_soc_card socle_soc_card;

static char __initdata banner[] = "\n#######SOCLE AC97 SOC Platform Driver, (c) 2009 SOCLE Corp.\n\n";

extern struct snd_soc_dai socle_ac97_dai[3];

static struct snd_soc_dai_link socle_dai[] = {

#if 0
	{
		.name = "WM9714 Voice",
		.stream_name = "WM9714 Voice",
		.cpu_dai = &socle_ac97_dai[0],
		.codec_dai = &wm9714_dai[0],
	},

#else
	{
		.name = "WM9714 Voice",
		.stream_name = "WM9714 Voice",
		.cpu_dai = &socle_ac97_dai[0],
		//.codec_dai = &wm9714_dai,
		.codec_dai = &alc658_dai[0],
	},
	{
		.name = "WM9714 Surround51",
		.stream_name = "WM9714 Surround51",
		.cpu_dai = &socle_ac97_dai[1],
		//.codec_dai = &wm9714_dai,
		.codec_dai = &alc658_dai[1],
	},
	{
		.name = "WM9714 Spdif",
		.stream_name = "WM9714 Spdif",
		.cpu_dai = &socle_ac97_dai[2],
		//.codec_dai = &wm9714_dai,
		.codec_dai = &alc658_dai[2],
	},
#endif
};


static struct snd_soc_card socle_wm9714ts = {
	.name = "Socle WM9714TS",
	.platform = &socle_snd_soc_platform,
	.dai_link = socle_dai,
	.num_links = ARRAY_SIZE(socle_dai),
};

extern struct snd_soc_codec_device soc_codec_dev_wm9714;
extern struct snd_soc_codec_device soc_codec_dev_alc658;

static struct snd_soc_device socle_snd_wm9714ts_devdata = {
	.card = &socle_wm9714ts,
#if 0
	.codec_dev = &soc_codec_dev_wm9714,
#else
	.codec_dev = &soc_codec_dev_alc658,
#endif
};

static struct platform_device *socle_snd_wm9714ts_device;

static int __init socle_asoc_init(void)
{
	int ret;
	printk(banner);
	printk("Socle WM9714TS ALSA SoC\n");
	socle_snd_wm9714ts_device = platform_device_alloc("soc-audio", -1);
	if (!socle_snd_wm9714ts_device)
		return -ENOMEM;

	platform_set_drvdata(socle_snd_wm9714ts_device,
				&socle_snd_wm9714ts_devdata);
	//socle_snd_wm9714ts_devdata.dev = &socle_snd_wm9714ts_device->dev;
	ret = platform_device_add(socle_snd_wm9714ts_device);

	if (ret)
		platform_device_put(socle_snd_wm9714ts_device);

	return ret;
}

static void __exit socle_asoc_exit(void)
{
	platform_device_unregister(socle_snd_wm9714ts_device);
}

module_init(socle_asoc_init);
module_exit(socle_asoc_exit);

/* Module information */
MODULE_AUTHOR("Jerry Hsieh");
MODULE_DESCRIPTION("Socle wm9714ts ALSA SoC");
MODULE_LICENSE("GPL");

