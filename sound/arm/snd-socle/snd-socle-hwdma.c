#ifdef CONFIG_SND_DEBUG
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <sound/asound.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <mach/i2s-regs.h>
#include <mach/dma.h>
#include <mach/i2s.h>

#ifdef CONFIG_UDA1342TS
#include <mach/uda1342ts.h>
#endif
#ifdef CONFIG_MS6335
#include <mach/ms6335.h>
#endif

#ifdef CONFIG_UDA1342TS	
//#define 	EXT_CLOCK  33868800
#define MAX_VOLUME 224
#else //MOSA 6553
//#define 	EXT_CLOCK  16934400
//#define MAX_VOLUME 31
#define MAX_VOLUME 400
#endif

static int socle_i2s_base;
/*
 *  Read register
 *  */
static u32 inline
socle_reg_read(u32 reg)
{
	return ioread32(socle_i2s_base+reg);
}

/*
 *  Write register
 *  */
static void inline
socle_reg_write(u32 val, u32 reg)
{
	iowrite32(val, socle_i2s_base+reg);
}
 
struct socle_snd_pcm {
	dma_addr_t dma_addr;
	u32 *buf;
	u32 buf_pos;
	u32 buf_size;
	u32 period_size;
	u32 periods;
};

struct socle_snd_control {
	int playback_switch;
	int capture_switch;
	int tone_switch;
	int playback_volume[2];
	int capture_volume[2];
	int tone_bass;
	int tone_treble;
};

struct socle_snd_host {
	struct resource *io_area;
	u32 va_base;
	int irq;
	struct socle_i2s_platform_data *i2s_data;
	struct socle_dma_notifier socle_snd_tx_dma_notifier;
	struct socle_dma_notifier socle_snd_rx_dma_notifier;
	struct socle_snd_pcm playback_pcm;
	struct socle_snd_pcm capture_pcm;
	u8 playback_stop;
	u8 capture_stop;
	struct socle_snd_control control;
	struct device *dev;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *playback_substream;
	struct snd_pcm_substream *capture_substream;
};

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX; /* index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR; /* ID for this card */
static u32 socle_i2s_conf = SOCLE_I2S_TX_N_RST |
	SOCLE_I2S_RX_N_RST |
	SOCLE_I2S_HDMA_REQ_1_EN |
	SOCLE_I2S_HDMA_REQ_2_EN |
	SOCLE_I2S_HDMA_IF_1_TX |
	SOCLE_I2S_HDMA_IF_2_RX |
	SOCLE_I2S_OP_NORMAL |
	SOCLE_I2S_TX_OP_STP |
	SOCLE_I2S_RX_OP_STP;

static void socle_snd_tx_dma_notifier_complete(void *data);
static void socle_snd_rx_dma_notifier_complete(void *data);
static int socle_snd_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params);
static int socle_snd_pcm_hw_free(struct snd_pcm_substream *substream);
static int socle_snd_pcm_prepare(struct snd_pcm_substream *substream, u8 mode);
static int socle_snd_playback_open(struct snd_pcm_substream *substream);
static int socle_snd_playback_close(struct snd_pcm_substream *substream);
static int socle_snd_playback_prepare(struct snd_pcm_substream *substream);
static int socle_snd_playback_trigger(struct snd_pcm_substream *substream, int cmd);
static snd_pcm_uframes_t socle_snd_playback_pointer(struct snd_pcm_substream *substream);
static int socle_snd_capture_open(struct snd_pcm_substream *substream);
static int socle_snd_capture_close(struct snd_pcm_substream *substream);
static int socle_snd_capture_prepare(struct snd_pcm_substream *substream);
static int socle_snd_capture_trigger(struct snd_pcm_substream *substream, int cmd);
static snd_pcm_uframes_t socle_snd_capture_pointer(struct snd_pcm_substream *substream);
static void socle_snd_control_initialize(struct socle_snd_control *control);
static int socle_snd_playback_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int socle_snd_playback_switch_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_playback_switch_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_playback_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int socle_snd_playback_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_playback_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
#ifdef CONFIG_UDA1342TS
static int socle_snd_capture_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int socle_snd_capture_switch_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_capture_switch_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_capture_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int socle_snd_capture_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_capture_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_tone_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int socle_snd_tone_switch_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_tone_switch_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_tone_bass_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int socle_snd_tone_bass_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_tone_bass_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_tone_treble_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
static int socle_snd_tone_treble_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
static int socle_snd_tone_treble_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
#endif

/*
 *  Hardware definition
 *  */
static struct snd_pcm_hardware socle_snd_playback_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
#ifdef CONFIG_SOCLE_INR
	.rates = SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000,
	.rate_min = 11025,
	.rate_max = 96000,
#else
	.rates = SNDRV_PCM_RATE_44100,
	.rate_min = 44100,
	.rate_max = 44100,
#endif
	.channels_min = 1,
	.channels_max = 2,
#if 1	
	.buffer_bytes_max = (64 * 1024),
	.period_bytes_min = 8*1024,
	.period_bytes_max = (8 * 1024),
	.periods_min = 1,
	.periods_max = 8,
#else
 	.buffer_bytes_max = (64 * 1024),
	.period_bytes_min = 64,
	.period_bytes_max = (64 * 1024) ,
	.periods_min = 1,
	.periods_max = 1024,
#endif
};

static struct snd_pcm_hardware socle_snd_capture_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
#ifdef CONFIG_SOCLE_INR
	.rates = SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000,
	.rate_min = 11025,
	.rate_max = 96000,
#else
	.rates = SNDRV_PCM_RATE_44100,
	.rate_min = 44100,
	.rate_max = 44100,
#endif
	.channels_min = 1,
	.channels_max = 2,
#if 1	
	.buffer_bytes_max = (64 * 1024),
	.period_bytes_min = 8*1024,
	.period_bytes_max = (8 * 1024),
	.periods_min = 1,
	.periods_max = 8,
#else
	.buffer_bytes_max = (64 * 1024),
	.period_bytes_min = 64,
	.period_bytes_max = (64 * 1024),
	.periods_min = 1,
	.periods_max = 1024,
#endif
};

static struct snd_kcontrol_new socle_snd_controls[] __devinitdata = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Playback Switch",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = 0xffff,
		.info = socle_snd_playback_switch_info,
		.get = socle_snd_playback_switch_get,
		.put = socle_snd_playback_switch_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Playback Volume",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = 0xffff,
		.info = socle_snd_playback_volume_info,
		.get = socle_snd_playback_volume_get,
		.put = socle_snd_playback_volume_put,
#ifdef CONFIG_UDA1342TS		
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Capture Switch",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = 0xffff,
		.info = socle_snd_capture_switch_info,
		.get = socle_snd_capture_switch_get,
		.put = socle_snd_capture_switch_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Capture Volume",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = 0xffff,
		.info = socle_snd_capture_volume_info,
		.get = socle_snd_capture_volume_get,
		.put = socle_snd_capture_volume_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Tone Control - Switch",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = 0xffff,
		.info = socle_snd_tone_switch_info,
		.get = socle_snd_tone_switch_get,
		.put = socle_snd_tone_switch_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Tone Control - Bass",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = 0xffff,
		.info = socle_snd_tone_bass_info,
		.get = socle_snd_tone_bass_get,
		.put = socle_snd_tone_bass_put,
	}, {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Tone Control - Treble",
		.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.private_value = 0xffff,
		.info = socle_snd_tone_treble_info,
		.get = socle_snd_tone_treble_get,
		.put = socle_snd_tone_treble_put,
	},
#else
	},
#endif
};

static 
void socle_snd_tx_dma_notifier_complete(void *data)
{
	struct socle_snd_host *host = data;	
	u32 flags;

	if (host->playback_stop) {
		/* Disable the hardware dma */
		flags = socle_claim_dma_lock();
		socle_disable_dma(host->i2s_data->tx_dma_ch);
		socle_release_dma_lock(flags);
		return;
	}
	host->playback_pcm.buf_pos += host->playback_pcm.period_size;
	host->playback_pcm.buf_pos %= host->playback_pcm.buf_size;
	snd_pcm_period_elapsed(host->playback_substream);	

#if 1
	/* Set and re-enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_set_dma_page_number(host->i2s_data->tx_dma_ch, 1);
	socle_release_dma_lock(flags);
#else
	/* Set and enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_disable_dma(host->i2s_data->tx_dma_ch);
	socle_set_dma_mode(host->i2s_data->tx_dma_ch, SOCLE_DMA_MODE_SLICE);
	socle_set_dma_ext_hdreq_number(host->i2s_data->tx_dma_ch, host->i2s_data->tx_dma_hdreq);
	socle_set_dma_burst_type(host->i2s_data->tx_dma_ch, host->i2s_data->burst_type);
	socle_set_dma_source_address(host->i2s_data->tx_dma_ch, host->playback_pcm.dma_addr+host->playback_pcm.buf_pos);
	socle_set_dma_destination_address(host->i2s_data->tx_dma_ch, host->io_area->start+SOCLE_I2S_TXR);
	socle_set_dma_source_direction(host->i2s_data->tx_dma_ch, SOCLE_DMA_DIR_INCR);
	socle_set_dma_destination_direction(host->i2s_data->tx_dma_ch, SOCLE_DMA_DIR_FIXED);
	socle_set_dma_data_size(host->i2s_data->tx_dma_ch, SOCLE_DMA_DATA_WORD);
	socle_set_dma_transfer_count(host->i2s_data->tx_dma_ch, host->playback_pcm.period_size);
	socle_set_dma_slice_count(host->i2s_data->tx_dma_ch, host->i2s_data->fifo_depth>>1);
	socle_set_dma_page_number(host->i2s_data->tx_dma_ch, host->playback_pcm.periods);
	socle_set_dma_buffer_size(host->i2s_data->tx_dma_ch, host->playback_pcm.buf_size);
	socle_enable_dma(host->i2s_data->tx_dma_ch);
	socle_release_dma_lock(flags);
#endif
}

static 
void socle_snd_rx_dma_notifier_complete(void *data)
{
	struct socle_snd_host *host = data;	
	u32 flags;

	if (host->capture_stop) {
		/* Disable the hardware dma */
		flags = socle_claim_dma_lock();
		socle_disable_dma(host->i2s_data->rx_dma_ch);
		socle_release_dma_lock(flags);

		return;
	}
	host->capture_pcm.buf_pos += host->capture_pcm.period_size;
	host->capture_pcm.buf_pos %= host->capture_pcm.buf_size;
	snd_pcm_period_elapsed(host->capture_substream);	

#if 0
	/* Set and re-enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_set_dma_page_number(host->i2s_data->rx_dma_ch, 1);
	socle_release_dma_lock(flags);
#else
	/* Set and enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_disable_dma(host->i2s_data->rx_dma_ch);
	socle_set_dma_mode(host->i2s_data->rx_dma_ch, SOCLE_DMA_MODE_SLICE);
	socle_set_dma_ext_hdreq_number(host->i2s_data->rx_dma_ch, host->i2s_data->rx_dma_hdreq);
	socle_set_dma_burst_type(host->i2s_data->rx_dma_ch, host->i2s_data->burst_type);
	socle_set_dma_source_address(host->i2s_data->rx_dma_ch, host->io_area->start+SOCLE_I2S_RXR);
	socle_set_dma_destination_address(host->i2s_data->rx_dma_ch, host->capture_pcm.dma_addr+host->capture_pcm.buf_pos);
	socle_set_dma_source_direction(host->i2s_data->rx_dma_ch, SOCLE_DMA_DIR_FIXED);
	socle_set_dma_destination_direction(host->i2s_data->rx_dma_ch, SOCLE_DMA_DIR_INCR);
	socle_set_dma_data_size(host->i2s_data->rx_dma_ch, SOCLE_DMA_DATA_WORD);
	socle_set_dma_transfer_count(host->i2s_data->rx_dma_ch, host->capture_pcm.period_size);
	socle_set_dma_slice_count(host->i2s_data->rx_dma_ch, host->i2s_data->fifo_depth>>1);
	socle_set_dma_page_number(host->i2s_data->rx_dma_ch, host->capture_pcm.periods);
	socle_set_dma_buffer_size(host->i2s_data->rx_dma_ch, host->capture_pcm.buf_size);
	socle_enable_dma(host->i2s_data->rx_dma_ch);
	socle_release_dma_lock(flags);
#endif
}

static void
socle_snd_control_initialize(struct socle_snd_control *control)
{
	int i;

	control->playback_switch = 1;
	control->capture_switch = 1;
	control->tone_switch = 0;
	control->tone_bass = 0;
	control->tone_treble = 0;
	for (i = 0 ; i < 2; i++) {
//		control->playback_volume[i] = MAX_VOLUME / 2;
		control->playback_volume[i] = 15;
		control->capture_volume[i] = 0;
	}
}

static int 
socle_snd_playback_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int 
socle_snd_playback_switch_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = host->control.playback_switch;
	return 0;
}

static int 
socle_snd_playback_switch_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	int err;

	if (host->control.playback_switch != ucontrol->value.integer.value[0]) {
		host->control.playback_switch = ucontrol->value.integer.value[0];
#ifdef CONFIG_UDA1342TS		
		err = uda1342ts_dac_power_switch(host->control.playback_switch);
		if (err) {
			dev_err(host->dev, "switching UDA1342TS DAC power is fail\n");
		} else
			changed = 1;
#elif CONFIG_MS6335
		err = ms6335_dac_power_switch(host->control.playback_switch);
		if (err) {
			dev_err(host->dev, "switching UDA1342TS DAC power is fail\n");
		} else
			changed = 1;
#else
		changed = 1;
#endif
	}
	return changed;
}

static int 
socle_snd_playback_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = MAX_VOLUME / 2;
	return 0;
}

static int 
socle_snd_playback_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int i;

	for (i = 0; i < 2; i++)
		ucontrol->value.integer.value[i] = host->control.playback_volume[i];
	return 0;
}

static int 
socle_snd_playback_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	int err;

	if (host->control.playback_volume[0] != ucontrol->value.integer.value[0]) {
		host->control.playback_volume[0] = ucontrol->value.integer.value[0];
#ifdef CONFIG_UDA1342TS				
		err = uda1342ts_dac_master_volume_l(host->control.playback_volume[0]);
#elif CONFIG_MS6335				
		err = ms6335_dac_master_volume_l(host->control.playback_volume[0]);
#else
		err = 0;
#endif
		if (err) {
			dev_err(host->dev, "changing UDA1342TS DAC master left volume is fail\n");
		} else
			changed |= 1;
	}
	if (host->control.playback_volume[1] != ucontrol->value.integer.value[1]) {
		host->control.playback_volume[1] = ucontrol->value.integer.value[1];
#ifdef CONFIG_UDA1342TS						
		err = uda1342ts_dac_master_volume_r(host->control.playback_volume[1]);
#elif CONFIG_MS6335				
		err = ms6335_dac_master_volume_r(host->control.playback_volume[1]);
#else
		err = 0;
#endif
		if (err) {
			dev_err(host->dev, "changing UDA1342TS DAC master right volume is fail\n");
		} else
			changed |= 1;
	}
	return changed;
}

#ifdef CONFIG_UDA1342TS
static int 
socle_snd_capture_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int 
socle_snd_capture_switch_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = host->control.capture_switch;
	return 0;
}

static int 
socle_snd_capture_switch_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	int err;

	if (host->control.capture_switch != ucontrol->value.integer.value[0]) {
		host->control.capture_switch = ucontrol->value.integer.value[0];
		err = uda1342ts_adc_power_switch(host->control.capture_switch);
		if (err) {
			dev_err(host->dev, "switching UDA1342TS ADC power is fail\n");
		} else
			changed = 1;
	}
	return changed;
}

static int 
socle_snd_capture_volume_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 8;
	return 0;
}

static int 
socle_snd_capture_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int i;

	for (i = 0; i < 2; i++)
		ucontrol->value.integer.value[i] = host->control.capture_volume[i];
	return 0;
}

static int 
socle_snd_capture_volume_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	int err;

	if (host->control.capture_volume[0] != ucontrol->value.integer.value[0]) {
		host->control.capture_volume[0] = ucontrol->value.integer.value[0];
		err = uda1342ts_adc_input_amplifier_gain_channel_1(host->control.capture_volume[0]);
		if (err) {
			dev_err(host->dev, "changing UDA1342TS ADC input1 amplifier gain is fail\n");
		} else
			changed |= 1;
	}
	if (host->control.capture_volume[1] != ucontrol->value.integer.value[1]) {
		host->control.capture_volume[1] = ucontrol->value.integer.value[1];
		err = uda1342ts_adc_input_amplifier_gain_channel_2(host->control.capture_volume[1]);
		if (err) {
			dev_err(host->dev, "changing UDA1342TS ADC input2 amplifier gain is fail\n");
		} else
			changed |= 1;
	}

	return changed;
}

static int 
socle_snd_tone_switch_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int 
socle_snd_tone_switch_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = host->control.tone_switch;
	return 0;
}

static int 
socle_snd_tone_switch_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	int err;

	if (host->control.tone_switch != ucontrol->value.integer.value[0]) {
		host->control.tone_switch = ucontrol->value.integer.value[0];
		err = uda1342ts_dac_tone_switch(host->control.tone_switch);
		if (err) {
			dev_err(host->dev, "switching UDA1342TS DAC tone is fail\n");
		} else
			changed = 1;
	}
	return changed;
}

static int 
socle_snd_tone_bass_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 15;
	return 0;
}

static int 
socle_snd_tone_bass_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = host->control.tone_bass;
	return 0;
}

static int 
socle_snd_tone_bass_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	int err;

	if (host->control.tone_bass != ucontrol->value.integer.value[0]) {
		host->control.tone_bass = ucontrol->value.integer.value[0];
		err = uda1342ts_dac_tone_bass(host->control.tone_bass);
		if (err) {
			dev_err(host->dev, "changing UDA1342TS DAC tone bass is fail\n");
		} else
			changed |= 1;
	}
	return changed;
}

static int
socle_snd_tone_treble_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 3;
	return 0;
}

static int
socle_snd_tone_treble_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = host->control.tone_treble;
	return 0;
}

static int
socle_snd_tone_treble_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct socle_snd_host *host = snd_kcontrol_chip(kcontrol);
	int changed = 0;
	int err;

	if (host->control.tone_treble != ucontrol->value.integer.value[0]) {
		host->control.tone_treble = ucontrol->value.integer.value[0];
		err = uda1342ts_dac_tone_treble(host->control.tone_treble);
		if (err) {
			dev_err(host->dev, "changing UDA1342TS DAC tone bass is fail\n");
		} else
			changed |= 1;
	}
	return changed;
}
#endif

static int
socle_snd_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);

	dev_dbg(host->dev, "socle_snd_pcm_hw_params()\n");	
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
}

static int
socle_snd_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);

	dev_dbg(host->dev, "socle_snd_pcm_hw_free()\n");	
	return snd_pcm_lib_free_pages(substream);
}

static int
socle_snd_pcm_prepare(struct snd_pcm_substream *substream, u8 mode)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u32 sample_res, oversampling_rate, channel_mode, format_width, ratio;
 
#if 0
	printk("socle_snd_pcm_prepare()\n");
	printk("channels:%d\n", runtime->channels);
	printk("rate:%d\n", runtime->rate);
	printk("format:%d\n", runtime->format);
	printk("format width:%d\n", snd_pcm_format_width(runtime->format));
	printk("buffer size: %d\n", snd_pcm_lib_buffer_bytes(substream));
	printk("period size: %d\n", snd_pcm_lib_period_bytes(substream));
	printk("periods:%d\n", runtime->periods);
	printk("dma_area:0x%08x\n", runtime->dma_area);
	printk("dma_addr:0x%08x\n", runtime->dma_addr);
	printk("dma_bytes:%d\n", runtime->dma_bytes);
	printk("dma buffer.area:0x%08x\n", runtime->dma_buffer_p->area);
	printk("dma buffer.addr:0x%08x\n", runtime->dma_buffer_p->addr);
	printk("dma buffer.bytes:%d\n", runtime->dma_buffer_p->bytes);
	printk("frame bits:%d\n", runtime->frame_bits);
#endif

#if 1
	dev_dbg(host->dev, "socle_snd_pcm_prepare()\n");
	dev_dbg(host->dev, "channels:%d\n", runtime->channels);
	dev_dbg(host->dev, "rate:%d\n", runtime->rate);
	dev_dbg(host->dev, "format:%d\n", runtime->format);
	dev_dbg(host->dev, "format width:%d\n", snd_pcm_format_width(runtime->format));
	dev_dbg(host->dev, "buffer size: %d\n", snd_pcm_lib_buffer_bytes(substream));
	dev_dbg(host->dev, "period size: %d\n", snd_pcm_lib_period_bytes(substream));
	dev_dbg(host->dev, "periods:%d\n", runtime->periods);
	dev_dbg(host->dev, "dma_area:0x%08x\n", (unsigned int)runtime->dma_area);
	dev_dbg(host->dev, "dma_addr:0x%08x\n", runtime->dma_addr);
	dev_dbg(host->dev, "dma_bytes:%d\n", runtime->dma_bytes);
	dev_dbg(host->dev, "dma buffer.area:0x%08x\n", (unsigned int)runtime->dma_buffer_p->area);
	dev_dbg(host->dev, "dma buffer.addr:0x%08x\n", runtime->dma_buffer_p->addr);
	dev_dbg(host->dev, "dma buffer.bytes:%d\n", runtime->dma_buffer_p->bytes);
	dev_dbg(host->dev, "frame bits:%d\n", runtime->frame_bits);
#else
	dev_info(host->dev, "socle_snd_pcm_prepare()\n");
	dev_info(host->dev, "channels:%d\n", runtime->channels);
	dev_info(host->dev, "rate:%d\n", runtime->rate);
	dev_info(host->dev, "format:%d\n", runtime->format);
	dev_info(host->dev, "format width:%d\n", snd_pcm_format_width(runtime->format));
	dev_info(host->dev, "buffer size: %d\n", snd_pcm_lib_buffer_bytes(substream));
	dev_info(host->dev, "period size: %d\n", snd_pcm_lib_period_bytes(substream));
	dev_info(host->dev, "periods:%d\n", runtime->periods);
	dev_info(host->dev, "dma_area:0x%08x\n", runtime->dma_area);
	dev_info(host->dev, "dma_addr:0x%08x\n", runtime->dma_addr);
	dev_info(host->dev, "dma_bytes:%d\n", runtime->dma_bytes);
	dev_info(host->dev, "dma buffer.area:0x%08x\n", runtime->dma_buffer_p->area);
	dev_info(host->dev, "dma buffer.addr:0x%08x\n", runtime->dma_buffer_p->addr);
	dev_info(host->dev, "dma buffer.bytes:%d\n", runtime->dma_buffer_p->bytes);
	dev_info(host->dev, "frame bits:%d\n", runtime->frame_bits);
#endif

	if (1 == runtime->channels)
		channel_mode = SOCLE_I2S_MONO;
	else if (2 == runtime->channels)
		channel_mode = SOCLE_I2S_STEREO;
	else {
		dev_err(host->dev, "unsupported channel number %d\n", runtime->channels);
		return -EINVAL;
	}

//	fs = EXT_CLOCK / runtime->rate;

//	ratio = 12;

#ifdef CONFIG_UDA1342TS
	switch(runtime->rate) {
		case 16000:
			ratio = 12;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
			break;
		case 44100:
			ratio = 12;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
			break;
		case 88200:
			ratio = 6;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
			break;
		default:
			printk("not support sample rate %d\n", runtime->rate);
			return -EINVAL;
	}
#else

#ifdef CONFIG_SOCLE_INR
	//MOSA 6335 clk = 16934400 * 4
	switch(runtime->rate) {
		case 11025:
			ratio = 48;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_128;
			break;
		case 16000:
			ratio = 66;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
			break;
		case 22050:
			ratio = 48;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
			break;
		case 32000:
			ratio = 66;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		case 44100:
			ratio = 48;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		case 48000:
			ratio = 44;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		case 88200:
			ratio = 24;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		case 96000:
			ratio = 22;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		default:
			printk("not support sample rate %d\n", runtime->rate);
			return -EINVAL;
	}
#else
	//MOSA 6335 clk = 16934400
	switch(runtime->rate) {
		case 16000:
			ratio = 16;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
			break;
		case 44100:
			ratio = 12;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		case 48000:
			ratio = 6;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
			break;
		case 88200:
			ratio = 6;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		case 96000:
			ratio = 6;
			oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
			break;
		default:
			printk("not support sample rate %d\n", runtime->rate);
			return -EINVAL;
	}
#endif	// end of CONFIG_SOCLE_INR

#endif

/*
	switch(fs/ratio) {
	case 32:
		oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_32;
		break;		
	case 64:
		oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
		break;		
	case 128:
		oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_128;
		break;
	default:
		printk("not support oversampling rate %d\n", fs/ratio);
		return -EINVAL;
	}
*/	
	format_width = snd_pcm_format_width(runtime->format);
//	printk("format_width : %x, runtime->frame_bits : %x \n",format_width,runtime->frame_bits);
	switch (format_width) {
	case 8:
		sample_res = SOCLE_I2S_SAMPLE_RES_8;
		break;
	case 16:
		sample_res = SOCLE_I2S_SAMPLE_RES_16;
		break;
	case 20:
		sample_res = SOCLE_I2S_SAMPLE_RES_20;
		break;
	case 24:
		sample_res = SOCLE_I2S_SAMPLE_RES_24;
		break;
	default:
		dev_err(host->dev, "unsupported format width %d\n", format_width);
		return -EINVAL;
	}

	if (0 == mode) {	/* playback mode */
		host->playback_pcm.buf = (u32 *)runtime->dma_area;
		host->playback_pcm.dma_addr = runtime->dma_addr;
		host->playback_pcm.buf_size = snd_pcm_lib_buffer_bytes(substream);
		host->playback_pcm.period_size = snd_pcm_lib_period_bytes(substream);
		host->playback_pcm.buf_pos = 0;
		host->playback_pcm.periods = runtime->periods;

		/* Set the transmitter */
		socle_reg_write(
				SOCLE_I2S_TX_DEV_SEL_0 |
				oversampling_rate |
				SOCLE_I2S_RATIO(ratio) |
				sample_res |
				channel_mode |
				SOCLE_I2S_BUS_IF_I2S |
				SOCLE_I2S_MASTER,SOCLE_I2S_TXCTL);

	} else {		/* capture mode */
		host->capture_pcm.buf = (u32 *)runtime->dma_area;
		host->capture_pcm.dma_addr = runtime->dma_addr;
		host->capture_pcm.buf_size = snd_pcm_lib_buffer_bytes(substream);
		host->capture_pcm.period_size = snd_pcm_lib_period_bytes(substream);
		host->capture_pcm.buf_pos = 0;
		host->capture_pcm.periods = runtime->periods;

		/* Set the receiver */
		socle_reg_write(
				SOCLE_I2S_RX_FIFO_CLR |
				oversampling_rate |
				SOCLE_I2S_RATIO(ratio) |
				sample_res |
				channel_mode |
				SOCLE_I2S_BUS_IF_I2S |
				SOCLE_I2S_MASTER,SOCLE_I2S_RXCTL);

	}
	return 0;
}

static int
socle_snd_playback_open(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	dev_dbg(host->dev, "socle_snd_playback_open()\n");
	host->playback_substream = substream;
	runtime->hw = socle_snd_playback_hw;

	/* Stop the tx operation */
	socle_reg_write(
			socle_i2s_conf |
			(socle_reg_read(SOCLE_I2S_OPR) & (~SOCLE_I2S_TX_OP_STR)),SOCLE_I2S_OPR);

	/* Reset the Tx logic and FSM */
	socle_reg_write(
			socle_i2s_conf |
			SOCLE_I2S_TX_RST |
			(socle_reg_read(SOCLE_I2S_OPR) & (~SOCLE_I2S_RX_OP_STR)),SOCLE_I2S_OPR);

	return 0;
}

static int
socle_snd_playback_close(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
 
	dev_dbg(host->dev, "socle_snd_playback_close()\n");
	return 0;
}

static int
socle_snd_playback_prepare(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);

	dev_dbg(host->dev, "socle_snd_playback_prepare()\n");
	return socle_snd_pcm_prepare(substream, 0);
}

static int
socle_snd_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	u32 flags;

	dev_dbg(host->dev, "socle_snd_playback_trigger(),cmd:%d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		host->playback_stop = 0;

		/* Start to transfer */
		socle_reg_write(
				socle_i2s_conf |
				SOCLE_I2S_TX_OP_STR |
				(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_RX_OP_STR),SOCLE_I2S_OPR);

		/* Set and enable the hardware dma */
		flags = socle_claim_dma_lock();
		socle_disable_dma(host->i2s_data->tx_dma_ch);
		socle_set_dma_mode(host->i2s_data->tx_dma_ch, SOCLE_DMA_MODE_SLICE);
		socle_set_dma_ext_hdreq_number(host->i2s_data->tx_dma_ch, host->i2s_data->tx_dma_hdreq);
		socle_set_dma_burst_type(host->i2s_data->tx_dma_ch, host->i2s_data->burst_type);
		socle_set_dma_source_address(host->i2s_data->tx_dma_ch, host->playback_pcm.dma_addr);
		socle_set_dma_destination_address(host->i2s_data->tx_dma_ch, host->io_area->start+SOCLE_I2S_TXR);
		socle_set_dma_source_direction(host->i2s_data->tx_dma_ch, SOCLE_DMA_DIR_INCR);
		socle_set_dma_destination_direction(host->i2s_data->tx_dma_ch, SOCLE_DMA_DIR_FIXED);
		socle_set_dma_data_size(host->i2s_data->tx_dma_ch, SOCLE_DMA_DATA_WORD);
		socle_set_dma_transfer_count(host->i2s_data->tx_dma_ch, host->playback_pcm.period_size);
		socle_set_dma_slice_count(host->i2s_data->tx_dma_ch, host->i2s_data->fifo_depth>>1);
		socle_set_dma_page_number(host->i2s_data->tx_dma_ch, host->playback_pcm.periods);
		socle_set_dma_buffer_size(host->i2s_data->tx_dma_ch, host->playback_pcm.buf_size);
		socle_enable_dma(host->i2s_data->tx_dma_ch);
		socle_release_dma_lock(flags);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		host->playback_stop = 1;

		/* Stop the transmission */
		socle_reg_write(
				socle_i2s_conf |
				(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_RX_OP_STR),SOCLE_I2S_OPR);
		
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t 
socle_snd_playback_pointer(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	
	dev_dbg(host->dev, "socle_snd_playback_pointer(),frame:%d\n", (int)bytes_to_frames(runtime, host->playback_pcm.buf_pos));
	return bytes_to_frames(runtime, host->playback_pcm.buf_pos);
}

static struct snd_pcm_ops socle_snd_playback_ops = {
	.open = socle_snd_playback_open,
	.close = socle_snd_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = socle_snd_pcm_hw_params,
	.hw_free = socle_snd_pcm_hw_free,
	.prepare = socle_snd_playback_prepare,
	.trigger = socle_snd_playback_trigger,
	.pointer = socle_snd_playback_pointer,
};

#ifdef CONFIG_ARCH_SCDK
static int
socle_snd_capture1_open(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	dev_dbg(host->dev, "socle_snd_capture_open()\n");
	
	uda1342ts_set_input_channel(2);
	
	host->capture_substream = substream;
	runtime->hw = socle_snd_capture_hw;

	/* Stop the rx operation */
	socle_reg_write(
			socle_i2s_conf |
			(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_TX_OP_STR),SOCLE_I2S_OPR);

	/* Reset the Rx logic and FSM */
	socle_reg_write(
			socle_i2s_conf |
			SOCLE_I2S_RX_RST |
			(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_TX_OP_STR),SOCLE_I2S_OPR);

	return 0;
}

static int
socle_snd_capture1_close(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);

	uda1342ts_set_input_channel(1);
	
	dev_dbg(host->dev, "socle_snd_capture_close()\n");
	return 0;
}
#endif
static int
socle_snd_capture_open(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	dev_dbg(host->dev, "socle_snd_capture_open()\n");

	host->capture_substream = substream;
	runtime->hw = socle_snd_capture_hw;

	/* Stop the rx operation */
	socle_reg_write(
			socle_i2s_conf |
			(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_TX_OP_STR),SOCLE_I2S_OPR);

	/* Reset the Rx logic and FSM */
	socle_reg_write(
			socle_i2s_conf |
			SOCLE_I2S_RX_RST |
			(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_TX_OP_STR),SOCLE_I2S_OPR);

	return 0;
}

static int
socle_snd_capture_close(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);

	dev_dbg(host->dev, "socle_snd_capture_close()\n");
	return 0;
}

static int
socle_snd_capture_prepare(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);

	dev_dbg(host->dev, "socle_snd_capture_prepare()\n");
	return socle_snd_pcm_prepare(substream, 1);
}

static int
socle_snd_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	u32 flags;

	dev_dbg(host->dev, "socle_snd_capture_trigger(),cmd:%d\n",cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		host->capture_stop = 0;

		/* Start to receive */
		socle_reg_write(
				socle_i2s_conf |
				SOCLE_I2S_RX_OP_STR |
				(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_TX_OP_STR),SOCLE_I2S_OPR);

		/* Set and enable the hardware dma */
		flags = socle_claim_dma_lock();
		socle_disable_dma(host->i2s_data->rx_dma_ch);
		socle_set_dma_mode(host->i2s_data->rx_dma_ch, SOCLE_DMA_MODE_SLICE);
		socle_set_dma_ext_hdreq_number(host->i2s_data->rx_dma_ch, host->i2s_data->rx_dma_hdreq);
		socle_set_dma_burst_type(host->i2s_data->rx_dma_ch, host->i2s_data->burst_type);
		socle_set_dma_source_address(host->i2s_data->rx_dma_ch, host->io_area->start+SOCLE_I2S_RXR);
		socle_set_dma_destination_address(host->i2s_data->rx_dma_ch, host->capture_pcm.dma_addr);
		socle_set_dma_source_direction(host->i2s_data->rx_dma_ch, SOCLE_DMA_DIR_FIXED);
		socle_set_dma_destination_direction(host->i2s_data->rx_dma_ch, SOCLE_DMA_DIR_INCR);
		socle_set_dma_data_size(host->i2s_data->rx_dma_ch, SOCLE_DMA_DATA_WORD);
		socle_set_dma_transfer_count(host->i2s_data->rx_dma_ch, host->capture_pcm.period_size);
		socle_set_dma_slice_count(host->i2s_data->rx_dma_ch, host->i2s_data->fifo_depth>>1);
		socle_set_dma_page_number(host->i2s_data->rx_dma_ch, host->capture_pcm.periods);
		socle_set_dma_buffer_size(host->i2s_data->rx_dma_ch, host->capture_pcm.buf_size);
		socle_enable_dma(host->i2s_data->rx_dma_ch);
		socle_release_dma_lock(flags);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		host->capture_stop = 1;

		/* Stop the transmission */
		socle_reg_write(
				socle_i2s_conf |
				(socle_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_TX_OP_STR),SOCLE_I2S_OPR);

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t 
socle_snd_capture_pointer(struct snd_pcm_substream *substream)
{
	struct socle_snd_host *host = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	
	dev_dbg(host->dev, "socle_snd_capture_pointer(),frame:%d\n", (int)bytes_to_frames(runtime, host->capture_pcm.buf_pos));
	return bytes_to_frames(runtime, host->capture_pcm.buf_pos);
}

static struct snd_pcm_ops socle_snd_capture_ops = {
	.open = socle_snd_capture_open,
	.close = socle_snd_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = socle_snd_pcm_hw_params,
	.hw_free = socle_snd_pcm_hw_free,
	.prepare = socle_snd_capture_prepare,
	.trigger = socle_snd_capture_trigger,
	.pointer = socle_snd_capture_pointer,
};

#ifdef CONFIG_ARCH_SCDK
static struct snd_pcm_ops socle_snd_capture1_ops = {
	.open = socle_snd_capture1_open,
	.close = socle_snd_capture1_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = socle_snd_pcm_hw_params,
	.hw_free = socle_snd_pcm_hw_free,
	.prepare = socle_snd_capture_prepare,
	.trigger = socle_snd_capture_trigger,
	.pointer = socle_snd_capture_pointer,
};
#endif

__devinit static int
socle_snd_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct socle_snd_host *host;
	int err;
	int dev = pdev->id;
	int i;

	dev_dbg(&pdev->dev, "socle_snd_probe()\n");
	card = snd_card_new(index[dev], id[dev], THIS_MODULE, sizeof(struct socle_snd_host));
	if (NULL == card) {
		dev_err(&pdev->dev, "cannot allocate memory to soundcard\n");
		err = -ENOMEM;
		goto err_no_mem;
	}
	host = (struct socle_snd_host *)card->private_data;
	host->card = card;
	host->dev = &pdev->dev;
	host->i2s_data = pdev->dev.platform_data;

	/* Find and claim our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_io_res;
	}
	
	host->io_area = request_mem_region(res->start, resource_size(res), pdev->name);
	if (NULL == host->io_area) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		err = -ENXIO;
		goto err_no_io_res;
	}
//	host->va_base = IO_ADDRESS(host->io_area->start);
//	socle_i2s_base = IO_ADDRESS(host->io_area->start);

       socle_i2s_base = (int)ioremap(res->start,  resource_size(res));
        if (!socle_i2s_base) {
                dev_err(&pdev->dev, "cannot map I2S registers\n");
		err = -ENOMEM;
                goto release_mem;
        }
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	/* Allocate the dma channel */
	host->socle_snd_tx_dma_notifier.data = host;
	host->socle_snd_rx_dma_notifier.data = host;
	host->socle_snd_tx_dma_notifier.complete = socle_snd_tx_dma_notifier_complete;
	host->socle_snd_rx_dma_notifier.complete = socle_snd_rx_dma_notifier_complete;
	err = socle_request_dma(host->i2s_data->tx_dma_ch, pdev->name, &host->socle_snd_tx_dma_notifier);
	if (err) {
		dev_err(&pdev->dev, "cannot claim dma channel for tx channel,channel:%d\n", host->i2s_data->tx_dma_ch);
		goto err_no_dma;
	}
	err = socle_request_dma(host->i2s_data->rx_dma_ch, pdev->name, &host->socle_snd_rx_dma_notifier);
	if (err) {
		dev_err(&pdev->dev, "cannot claim dma channel for rx channel,channel:%d\n", host->i2s_data->rx_dma_ch);
		goto err_no_dma;
	}

	strcpy(card->driver, pdev->name);
	strcpy(card->shortname, pdev->name);
	sprintf(card->longname, "%s.%d", pdev->name, dev);
	snd_card_set_dev(card, &pdev->dev);

	/* Create a pcm device */
	err = snd_pcm_new(card, "Socle PCM", dev, 1, 1, &pcm);
	if (err) {
		dev_err(&pdev->dev, "cannot create a pcm device\n");
		goto err_create_pcm;
	}
	host->pcm = pcm;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &socle_snd_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &socle_snd_capture_ops);
#ifdef CONFIG_ARCH_SCDK	
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &socle_snd_capture1_ops);	//SCDK Line in
#endif
	pcm->private_data = host;
	pcm->info_flags = 0;
	sprintf(pcm->name, "Socle PCM.%d", dev);
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, &pdev->dev,
					      64*1024, 64*1024);
	sprintf(card->mixername, "Socle Mixer.%d", dev);
	for (i = 0; i < ARRAY_SIZE(socle_snd_controls); i++) {
		err = snd_ctl_add(card, snd_ctl_new1(&socle_snd_controls[i], host));
		if (err) {
			dev_err(&pdev->dev, "cannot add the soundcard control[%d]:%s", i, socle_snd_controls[i].name);
			goto err_add_snd_ctl;
		}
	}

	/* Register the soundcard */
	err = snd_card_register(card);
	if (err) {
		dev_err(&pdev->dev, "cannot register the soundcard\n");
		goto err_reg_snd;
	}
	platform_set_drvdata(pdev, card);

#ifdef CONFIG_UDA1342TS
	/* Initialize the audio codec */
	if (uda1342ts_dac_reset())
		goto err_add_snd_ctl;
	if (uda1342ts_adc_reset())
		goto err_add_snd_ctl;
	if (uda1342ts_dac_set_system_frequency(UDA1342TS_FUNC_SYSTEM_768FS))
		goto err_add_snd_ctl;
	if (uda1342ts_adc_set_system_frequency(UDA1342TS_FUNC_SYSTEM_768FS))
		goto err_add_snd_ctl;
#endif

	socle_snd_control_initialize(&host->control);

	/* Disable the interrupt for I2S */
	socle_reg_write(
			SOCLE_I2S_TX_FIFO_TRIG_INT_DIS |
			SOCLE_I2S_RX_FIFO_TRIG_INT_DIS |
			SOCLE_I2S_RX_FIFO_OVR_INT_DIS,SOCLE_I2S_IER);

	return 0;
err_add_snd_ctl:
err_reg_snd:
err_create_pcm:
	socle_free_dma(host->i2s_data->tx_dma_ch);
	socle_free_dma(host->i2s_data->rx_dma_ch);
err_no_dma:
err_no_irq:
//	release_resource(host->io_area);
release_mem:
	release_mem_region(res->start, resource_size(res));
err_no_io_res:
	snd_card_free(card);
err_no_mem:
	return err;
}

static int
socle_snd_remove(struct platform_device *pdev)
{
	struct snd_card *card;
	struct socle_snd_host *host;

	dev_dbg(&pdev->dev, "socle_snd_remove()\n");
	card = (struct snd_card *)platform_get_drvdata(pdev);
	host = (struct socle_snd_host *)card->private_data;
	free_irq(host->irq, host);
	release_resource(host->io_area);
	socle_free_dma(host->i2s_data->tx_dma_ch );
	socle_free_dma(host->i2s_data->rx_dma_ch);
	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int
socle_snd_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int 
socle_snd_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define socle_snd_suspend NULL
#define socle_snd_resume NULL
#endif

static struct platform_driver socle_snd_driver = 
{
	.probe = socle_snd_probe,
	.remove = socle_snd_remove,
	.suspend = socle_snd_suspend,
	.resume = socle_snd_resume,
	.driver = {
		.name = "socle_snd",
		.owner = THIS_MODULE,
	},
};

static int __init
socle_snd_init(void)
{
	return platform_driver_register(&socle_snd_driver);
}

static void __exit
socle_snd_exit(void)
{
	platform_driver_unregister(&socle_snd_driver);
}

module_init(socle_snd_init);
module_exit(socle_snd_exit);

MODULE_DESCRIPTION("Socle ALSA soundcard driver");
MODULE_AUTHOR("Obi Hsieh");
MODULE_LICENSE("GPL");

