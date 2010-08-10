/*
 * wm9714.c  --  ALSA Soc WM9714 codec support
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood <lrg@slimlogic.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Features:-
 *
 *   o Support for AC97 Codec, Voice DAC and Aux DAC
 *   o Support for DAPM
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "wm9714.h"

#define WM9714_VERSION "0.15"

//#define CONFIG_AC97_CODEC_DEBUG
#ifdef CONFIG_AC97_CODEC_DEBUG
	#define DBG(fmt, args...) printk("Wm9714 : %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

struct wm9714_priv {
	u32 pll_in; /* PLL input frequency */
	u32 pll_out; /* PLL output frequency */
};

static unsigned int ac97_read(struct snd_soc_codec *codec,
	unsigned int reg);
static int ac97_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val);

/*
 * WM9714 register cache
 * Reg 0x3c bit 15 is used by touch driver.
 */
static const u16 wm9714_reg[] = {
	0x6174, 0x8080, 0x8080, 0x8080,
	0xc880, 0xe808, 0xe808, 0x0808,
	0x00da, 0x8000, 0xd600, 0xaaa0,
	0xaaa0, 0xaaa0, 0x0000, 0x0000,
	0x0f0f, 0x0040, 0x0000, 0x7f00,
	0x0405, 0x0410, 0xbb80, 0xbb80,
	0x0000, 0xbb80, 0x0000, 0x4523,
	0x0000, 0x2000, 0x7eff, 0xffff,
	0x0000, 0x0000, 0x0080, 0x0000,
	0x0000, 0x0000, 0xfffe, 0xffff,
	0x0000, 0x0000, 0x0000, 0xfffe,
	0x4000, 0x0000, 0x0000, 0x0000,
	0xb032, 0x3e00, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0006,
	0x0001, 0x0000, 0x574d, 0x4c13,
	0x0000, 0x0000, 0x0000
};

/* virtual HP mixers regs */
#define HPL_MIXER	0x80
#define HPR_MIXER	0x82
#define MICB_MUX	0x82

static const char *wm9714_mic_mixer[] = {"Stereo", "Mic 1", "Mic 2", "Mute"};
static const char *wm9714_rec_mux[] = {"Stereo", "Left", "Right", "Mute"};
static const char *wm9714_rec_src[] =
	{"Mic 1", "Mic 2", "Line", "Mono In", "Headphone", "Speaker",
	"Mono Out", "Zh"};
static const char *wm9714_rec_gain[] = {"+1.5dB Steps", "+0.75dB Steps"};
static const char *wm9714_alc_select[] = {"None", "Left", "Right", "Stereo"};
static const char *wm9714_mono_pga[] = {"Vmid", "Zh", "Mono", "Inv",
	"Mono Vmid", "Inv Vmid"};
static const char *wm9714_spk_pga[] =
	{"Vmid", "Zh", "Headphone", "Speaker", "Inv", "Headphone Vmid",
	"Speaker Vmid", "Inv Vmid"};
static const char *wm9714_hp_pga[] = {"Vmid", "Zh", "Headphone",
	"Headphone Vmid"};
static const char *wm9714_out3_pga[] = {"Vmid", "Zh", "Inv 1", "Inv 1 Vmid"};
static const char *wm9714_out4_pga[] = {"Vmid", "Zh", "Inv 2", "Inv 2 Vmid"};
static const char *wm9714_dac_inv[] =
	{"Off", "Mono", "Speaker", "Left Headphone", "Right Headphone",
	"Headphone Mono", "NC", "Vmid"};
static const char *wm9714_bass[] = {"Linear Control", "Adaptive Boost"};
static const char *wm9714_ng_type[] = {"Constant Gain", "Mute"};
static const char *wm9714_mic_select[] = {"Mic 1", "Mic 2 A", "Mic 2 B"};
static const char *wm9714_micb_select[] = {"MPB", "MPA"};

static const struct soc_enum wm9714_enum[] = {
SOC_ENUM_SINGLE(AC97_LINE, 3, 4, wm9714_mic_mixer), /* record mic mixer 0 */
SOC_ENUM_SINGLE(AC97_VIDEO, 14, 4, wm9714_rec_mux), /* record mux hp 1 */
SOC_ENUM_SINGLE(AC97_VIDEO, 9, 4, wm9714_rec_mux),  /* record mux mono 2 */
SOC_ENUM_SINGLE(AC97_VIDEO, 3, 8, wm9714_rec_src),  /* record mux left 3 */
SOC_ENUM_SINGLE(AC97_VIDEO, 0, 8, wm9714_rec_src),  /* record mux right 4*/
SOC_ENUM_DOUBLE(AC97_CD, 14, 6, 2, wm9714_rec_gain), /* record step size 5 */
SOC_ENUM_SINGLE(AC97_PCI_SVID, 14, 4, wm9714_alc_select), /* alc source select 6*/
SOC_ENUM_SINGLE(AC97_REC_GAIN, 14, 4, wm9714_mono_pga), /* mono input select 7 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 11, 8, wm9714_spk_pga), /* speaker left input select 8 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 8, 8, wm9714_spk_pga), /* speaker right input select 9 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 6, 3, wm9714_hp_pga), /* headphone left input 10 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 4, 3, wm9714_hp_pga), /* headphone right input 11 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 2, 4, wm9714_out3_pga), /* out 3 source 12 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 0, 4, wm9714_out4_pga), /* out 4 source 13 */
SOC_ENUM_SINGLE(AC97_REC_GAIN_MIC, 13, 8, wm9714_dac_inv), /* dac invert 1 14 */
SOC_ENUM_SINGLE(AC97_REC_GAIN_MIC, 10, 8, wm9714_dac_inv), /* dac invert 2 15 */
SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 15, 2, wm9714_bass), /* bass control 16 */
SOC_ENUM_SINGLE(AC97_PCI_SVID, 5, 2, wm9714_ng_type), /* noise gate type 17 */
SOC_ENUM_SINGLE(AC97_3D_CONTROL, 12, 3, wm9714_mic_select), /* mic selection 18 */
SOC_ENUM_SINGLE(MICB_MUX, 0, 2, wm9714_micb_select), /* mic selection 19 */
};

static const struct snd_kcontrol_new wm9714_snd_ac97_controls[] = {
SOC_DOUBLE("Speaker Playback Volume", AC97_MASTER, 8, 0, 31, 1),
SOC_DOUBLE("Speaker Playback Switch", AC97_MASTER, 15, 7, 1, 1),
SOC_DOUBLE("Headphone Playback Volume", AC97_HEADPHONE, 8, 0, 31, 1),
SOC_DOUBLE("Headphone Playback Switch", AC97_HEADPHONE, 15, 7, 1, 1),
SOC_DOUBLE("Line In Volume", AC97_PC_BEEP, 8, 0, 31, 1),
SOC_DOUBLE("PCM Playback Volume", AC97_PHONE, 8, 0, 31, 1),
SOC_SINGLE("Mic 1 Volume", AC97_MIC, 8, 31, 1),
SOC_SINGLE("Mic 2 Volume", AC97_MIC, 0, 31, 1),

SOC_SINGLE("Mic Boost (+20dB) Switch", AC97_LINE, 5, 1, 0),
SOC_SINGLE("Mic Headphone Mixer Volume", AC97_LINE, 0, 7, 1),

SOC_SINGLE("Capture Switch", AC97_CD, 15, 1, 1),
SOC_ENUM("Capture Volume Steps", wm9714_enum[5]),
SOC_DOUBLE("Capture Volume", AC97_CD, 8, 0, 31, 0),
SOC_SINGLE("Capture ZC Switch", AC97_CD, 7, 1, 0),

SOC_SINGLE("Capture to Headphone Volume", AC97_VIDEO, 11, 7, 1),
SOC_SINGLE("Capture to Mono Boost (+20dB) Switch", AC97_VIDEO, 8, 1, 0),
SOC_SINGLE("Capture ADC Boost (+20dB) Switch", AC97_VIDEO, 6, 1, 0),

SOC_SINGLE("ALC Target Volume", AC97_CODEC_CLASS_REV, 12, 15, 0),
SOC_SINGLE("ALC Hold Time", AC97_CODEC_CLASS_REV, 8, 15, 0),
SOC_SINGLE("ALC Decay Time", AC97_CODEC_CLASS_REV, 4, 15, 0),
SOC_SINGLE("ALC Attack Time", AC97_CODEC_CLASS_REV, 0, 15, 0),
SOC_ENUM("ALC Function", wm9714_enum[6]),
SOC_SINGLE("ALC Max Volume", AC97_PCI_SVID, 11, 7, 0),
SOC_SINGLE("ALC ZC Timeout", AC97_PCI_SVID, 9, 3, 0),
SOC_SINGLE("ALC ZC Switch", AC97_PCI_SVID, 8, 1, 0),
SOC_SINGLE("ALC NG Switch", AC97_PCI_SVID, 7, 1, 0),
SOC_ENUM("ALC NG Type", wm9714_enum[17]),
SOC_SINGLE("ALC NG Threshold", AC97_PCI_SVID, 0, 31, 0),

SOC_DOUBLE("Speaker Playback ZC Switch", AC97_MASTER, 14, 6, 1, 0),
SOC_DOUBLE("Headphone Playback ZC Switch", AC97_HEADPHONE, 14, 6, 1, 0),

SOC_SINGLE("Out4 Playback Switch", AC97_MASTER_MONO, 15, 1, 1),
SOC_SINGLE("Out4 Playback ZC Switch", AC97_MASTER_MONO, 14, 1, 0),
SOC_SINGLE("Out4 Playback Volume", AC97_MASTER_MONO, 8, 63, 1),

SOC_SINGLE("Out3 Playback Switch", AC97_MASTER_MONO, 7, 1, 1),
SOC_SINGLE("Out3 Playback ZC Switch", AC97_MASTER_MONO, 6, 1, 0),
SOC_SINGLE("Out3 Playback Volume", AC97_MASTER_MONO, 0, 63, 1),

SOC_SINGLE("Mono Capture Volume", AC97_MASTER_TONE, 8, 31, 1),
SOC_SINGLE("Mono Playback Switch", AC97_MASTER_TONE, 7, 1, 1),
SOC_SINGLE("Mono Playback ZC Switch", AC97_MASTER_TONE, 6, 1, 0),
SOC_SINGLE("Mono Playback Volume", AC97_MASTER_TONE, 0, 31, 1),

SOC_SINGLE("PC Beep Playback Headphone Volume", AC97_AUX, 12, 7, 1),
SOC_SINGLE("PC Beep Playback Speaker Volume", AC97_AUX, 8, 7, 1),
SOC_SINGLE("PC Beep Playback Mono Volume", AC97_AUX, 4, 7, 1),

SOC_SINGLE("Voice Playback Headphone Volume", AC97_PCM, 12, 7, 1),
SOC_SINGLE("Voice Playback Master Volume", AC97_PCM, 8, 7, 1),
SOC_SINGLE("Voice Playback Mono Volume", AC97_PCM, 4, 7, 1),

SOC_SINGLE("Aux Playback Headphone Volume", AC97_REC_SEL, 12, 7, 1),
SOC_SINGLE("Aux Playback Master Volume", AC97_REC_SEL, 8, 7, 1),
SOC_SINGLE("Aux Playback Mono Volume", AC97_REC_SEL, 4, 7, 1),

SOC_ENUM("Bass Control", wm9714_enum[16]),
SOC_SINGLE("Bass Cut-off Switch", AC97_GENERAL_PURPOSE, 12, 1, 1),
SOC_SINGLE("Tone Cut-off Switch", AC97_GENERAL_PURPOSE, 4, 1, 1),
SOC_SINGLE("Playback Attenuate (-6dB) Switch", AC97_GENERAL_PURPOSE, 6, 1, 0),
SOC_SINGLE("Bass Volume", AC97_GENERAL_PURPOSE, 8, 15, 1),
SOC_SINGLE("Tone Volume", AC97_GENERAL_PURPOSE, 0, 15, 1),

SOC_SINGLE("3D Upper Cut-off Switch", AC97_REC_GAIN_MIC, 5, 1, 0),
SOC_SINGLE("3D Lower Cut-off Switch", AC97_REC_GAIN_MIC, 4, 1, 0),
SOC_SINGLE("3D Depth", AC97_REC_GAIN_MIC, 0, 15, 1),
};

/* add non dapm controls */
static int wm9714_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm9714_snd_ac97_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm9714_snd_ac97_controls[i],
					codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static unsigned int ac97_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	if (reg == AC97_RESET || reg == AC97_GPIO_STATUS ||
		reg == AC97_VENDOR_ID1 || reg == AC97_VENDOR_ID2 ||
		reg == AC97_CD)
		return soc_ac97_ops.read(codec->ac97, reg);
	else {
		reg = reg >> 1;

		if (reg > (ARRAY_SIZE(wm9714_reg)))
			return -EIO;

		return cache[reg];
	}
}

static int ac97_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u16 *cache = codec->reg_cache;
	if (reg < 0x7c)
		soc_ac97_ops.write(codec->ac97, reg, val);
	reg = reg >> 1;
	if (reg <= (ARRAY_SIZE(wm9714_reg)))
		cache[reg] = val;

	return 0;
}

static int ac97_hifi_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int reg;
	u16 vra;

	DBG("----start\n");
	
	//DBG("rate = %d\n", runtime->rate);
	vra = ac97_read(codec, AC97_EXTENDED_STATUS);
	ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = AC97_PCM_FRONT_DAC_RATE;
	else
		reg = AC97_PCM_LR_ADC_RATE;

	return ac97_write(codec, reg, runtime->rate);

	return 0;
}

static int wm9714_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	//struct snd_soc_codec *codec = dai->codec;

	DBG("----start\n");
	
	return 0;
}

#define WM9714_RATES \
	(SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define WM9714_PCM_FORMATS \
	(SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE)

struct snd_soc_dai wm9714_dai[] = {
{
	.name = "AC97 HiFi",
	.ac97_control = 1,
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM9714_RATES,
		.formats = WM9714_PCM_FORMATS,},
	.capture = {
		.stream_name = "HiFi Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM9714_RATES,
		.formats = WM9714_PCM_FORMATS,},
	.ops = {
		.prepare = ac97_hifi_prepare,
		.hw_params = wm9714_pcm_hw_params,},
},
};
EXPORT_SYMBOL_GPL(wm9714_dai);

int wm9714_reset(struct snd_soc_codec *codec, int try_warm)
{
	if (try_warm && soc_ac97_ops.warm_reset) {
		soc_ac97_ops.warm_reset(codec->ac97);
		if (ac97_read(codec, 0) == wm9714_reg[0])
			return 1;
	}

	soc_ac97_ops.reset(codec->ac97);
	if (soc_ac97_ops.warm_reset)
		soc_ac97_ops.warm_reset(codec->ac97);
	if (ac97_read(codec, 0) != wm9714_reg[0])
		return -EIO;
	return 0;
}
EXPORT_SYMBOL_GPL(wm9714_reset);

static int wm9714_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	
	switch (level) {
	case SND_SOC_BIAS_ON:
		/* enable thermal shutdown */
		//reg = ac97_read(codec, AC97_EXTENDED_MID) & 0x1bff;
		//ac97_write(codec, AC97_EXTENDED_MID, reg);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:

#ifdef CONFIG_SND_SOC_SOCLE_WM9714_AC97
		// setup the codec initial status
		ac97_write(codec, AC97_HEADPHONE, 0x0);	//0x04
		ac97_write(codec, AC97_PHONE, 0x6808);	//0x0c
		ac97_write(codec, AC97_CD, 0x0);			//0x12
		ac97_write(codec, AC97_VIDEO, 0xd612);	//0x14
		ac97_write(codec, AC97_REC_GAIN, 0xa0);	//0x1c
		ac97_write(codec, AC97_POWERDOWN, 0x0);	//0x26
		ac97_write(codec, AC97_EXTENDED_MID, 0xf803);	//0x3c
		ac97_write(codec, AC97_EXTENDED_MSTATUS, 0xf99f);	//0x3e
		ac97_write(codec, AC97_LINE1_RATE, 0x2000);	//0x40
		ac97_write(codec, AC97_PCM_FRONT_DAC_RATE, 0xbb80);	//0x2c
		ac97_write(codec, AC97_PCM_LR_ADC_RATE, 0xbb80);	//0x32
		//ac97_write(codec, AC97_PC_BEEP, 0x6808);	//0x0a
#else
		// setup the codec initial status
		ac97_write(codec, AC97_HEADPHONE, 0x0);	//0x04
		ac97_write(codec, AC97_CD, 0x0);			//0x12
		ac97_write(codec, AC97_VIDEO, 0xd612);	//0x14
		ac97_write(codec, AC97_PCM, 0x2aa0);	//0x18
		ac97_write(codec, AC97_REC_GAIN, 0xa0);	//0x1c
		ac97_write(codec, AC97_POWERDOWN, 0x0);	//0x26
		//ac97_write(codec, AC97_CENTER_LFE_MASTER, 0xc022);	//0x36	MASTER
		ac97_write(codec, AC97_CENTER_LFE_MASTER, 0xa022);	//0x36	SLAVE 
		ac97_write(codec, AC97_EXTENDED_MID, 0xea03);	//0x3c
		ac97_write(codec, AC97_EXTENDED_MSTATUS, 0xf99f);	//0x3e
		//ac97_write(codec, AC97_PC_BEEP, 0x6808);	//0x0a
		//ac97_write(codec, AC97_GPIO_CFG, 0xffdc);	//0x0a	MASTE
		ac97_write(codec, AC97_GPIO_CFG, 0xffde);	//0x0a	SLAVER

#endif
	
		break;
	case SND_SOC_BIAS_OFF:
		/* disable everything including AC link */
		ac97_write(codec, AC97_EXTENDED_MID, 0xffff);
		ac97_write(codec, AC97_EXTENDED_MSTATUS, 0xffff);
		ac97_write(codec, AC97_POWERDOWN, 0xffff);
		
		break;
	}
	codec->bias_level = level;
	return 0;
}

static int wm9714_soc_suspend(struct platform_device *pdev,
	pm_message_t state)
{

	return 0;
}

static int wm9714_soc_resume(struct platform_device *pdev)
{

	return 0;
}

static int wm9714_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	printk(KERN_INFO "WM9714/WM9714 SoC Audio Codec %s\n", WM9714_VERSION);
		
	socdev->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (socdev->codec == NULL)
		return -ENOMEM;
	codec = socdev->codec;
	mutex_init(&codec->mutex);

	codec->reg_cache = kmemdup(wm9714_reg, sizeof(wm9714_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		ret = -ENOMEM;
		goto cache_err;
	}
	codec->reg_cache_size = sizeof(wm9714_reg);
	codec->reg_cache_step = 2;

	codec->private_data = kzalloc(sizeof(struct wm9714_priv), GFP_KERNEL);
	if (codec->private_data == NULL) {
		ret = -ENOMEM;
		goto priv_err;
	}

	codec->name = "WM9714";
	codec->owner = THIS_MODULE;
	codec->dai = wm9714_dai;
	codec->num_dai = ARRAY_SIZE(wm9714_dai);
	codec->write = ac97_write;
	codec->read = ac97_read;
	codec->set_bias_level = wm9714_set_bias_level;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ret = snd_soc_new_ac97_codec(codec, &soc_ac97_ops, 0);
	if (ret < 0)
		goto codec_err;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0)
		goto pcm_err;
		
	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */

	wm9714_reset(codec, 0);
	ret = wm9714_reset(codec, 1);
	if (ret < 0) {
		printk(KERN_ERR "Failed to reset WM9714: AC97 link error\n");
		goto reset_err;
	}

	wm9714_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	wm9714_add_controls(codec);

	ret = snd_soc_init_card(socdev);
	if (ret < 0)
		goto reset_err;
	return 0;

reset_err:
	snd_soc_free_pcms(socdev);

pcm_err:
	snd_soc_free_ac97_codec(codec);

codec_err:
	kfree(codec->private_data);

priv_err:
	kfree(codec->reg_cache);

cache_err:
	kfree(socdev->codec);
	socdev->codec = NULL;
	return ret;
}

static int wm9714_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec == NULL)
		return 0;

	snd_soc_free_pcms(socdev);
	snd_soc_free_ac97_codec(codec);
	kfree(codec->private_data);
	kfree(codec->reg_cache);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm9714 = {
	.probe = 	wm9714_soc_probe,
	.remove = 	wm9714_soc_remove,
	.suspend =	wm9714_soc_suspend,
	.resume = 	wm9714_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wm9714);

MODULE_DESCRIPTION("ASoC WM9714/WM9714 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
