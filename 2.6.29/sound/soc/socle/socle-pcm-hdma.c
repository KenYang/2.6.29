#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/i2s.h>
#include <mach/platform.h>


#include "socle-pcm.h"

struct snd_pcm_substream *playback_substream;
struct snd_pcm_substream *capture_substream;
u32 playback_buf_pos;
u32 capture_buf_pos;


static struct snd_pcm_hardware socle_snd_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =	SNDRV_PCM_FMTBIT_S8 | 
				SNDRV_PCM_FMTBIT_S16_LE | 
				SNDRV_PCM_FMTBIT_S20_3LE | 
				SNDRV_PCM_FMTBIT_S24_LE,

	.rates = SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | \
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200,

	.rate_min = 44100,
	.rate_max = 44100,
	
	.channels_min = 1,
	.channels_max = 2,
	
	.buffer_bytes_max = (64 * 1024),
	.period_bytes_min = 8*1024,
	.period_bytes_max = (8 * 1024),
	.periods_min = 8,
	.periods_max = 8,
};

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

struct socle_snd_host *host;

int playback_stop;
int capture_stop;

#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
static 
void socle_snd_tx_dma_notifier_complete(void *data)
{
	u32 flags, ch = host->i2s_data->tx_dma_ch;
	struct snd_dma_buffer *dma_buffer;
//printk("Entered %s\n", __func__);
	if (playback_stop) {
		/* Disable the hardware dma */
		flags = socle_claim_dma_lock();
		socle_disable_dma(ch);
		socle_release_dma_lock(flags);
		return;
	}

	dma_buffer = &host->playback_substream->dma_buffer;
	playback_buf_pos += snd_pcm_lib_period_bytes(host->playback_substream);
	playback_buf_pos %= dma_buffer->bytes;
	snd_pcm_period_elapsed(host->playback_substream);	

#if 0
	/* Set and re-enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_set_dma_page_number(ch, 1);
	socle_release_dma_lock(flags);
#else
	/* Set and enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_disable_dma(ch);
	socle_set_dma_mode(ch, SOCLE_DMA_MODE_SLICE);
	socle_set_dma_ext_hdreq_number(ch, host->i2s_data->tx_dma_hdreq);
	socle_set_dma_burst_type(ch, host->i2s_data->burst_type);
	socle_set_dma_source_address(ch, playback_substream->dma_buffer.addr+playback_buf_pos);
	socle_set_dma_destination_address(ch, SOCLE_I2S0+0x04);
	socle_set_dma_source_direction(ch, SOCLE_DMA_DIR_INCR);
	socle_set_dma_destination_direction(ch, SOCLE_DMA_DIR_FIXED);
	socle_set_dma_data_size(ch, SOCLE_DMA_DATA_WORD);
	socle_set_dma_transfer_count(ch, socle_snd_hw.period_bytes_min);
	socle_set_dma_slice_count(ch, host->i2s_data->fifo_depth>>1);
	socle_set_dma_page_number(ch, socle_snd_hw.periods_max);
	socle_set_dma_buffer_size(ch, socle_snd_hw.buffer_bytes_max);
	socle_enable_dma(host->i2s_data->tx_dma_ch);
	socle_release_dma_lock(flags);
#endif
}

static 
void socle_snd_rx_dma_notifier_complete(void *data)
{
	u32 flags, ch = host->i2s_data->rx_dma_ch;
	struct snd_dma_buffer *dma_buffer;
//printk("Entered %s\n", __func__);
	if (capture_stop) {
		/* Disable the hardware dma */
		flags = socle_claim_dma_lock();
		socle_disable_dma(ch);
		socle_release_dma_lock(flags);
		return;
	}

	dma_buffer = &host->capture_substream->dma_buffer;
	capture_buf_pos += snd_pcm_lib_period_bytes(host->capture_substream);
	capture_buf_pos %= dma_buffer->bytes;
	snd_pcm_period_elapsed(host->capture_substream);	

#if 0
	/* Set and re-enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_set_dma_page_number(ch, 1);
	socle_release_dma_lock(flags);
#else
	/* Set and enable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_disable_dma(ch);
	socle_set_dma_mode(ch, SOCLE_DMA_MODE_SLICE);
	socle_set_dma_ext_hdreq_number(ch, host->i2s_data->rx_dma_hdreq);
	socle_set_dma_burst_type(ch, host->i2s_data->burst_type);
	socle_set_dma_source_address(ch, SOCLE_I2S0+0x8);
	socle_set_dma_destination_address(ch, host->capture_substream->dma_buffer.addr+capture_buf_pos);
	socle_set_dma_source_direction(ch, SOCLE_DMA_DIR_FIXED);
	socle_set_dma_destination_direction(ch, SOCLE_DMA_DIR_INCR);
	socle_set_dma_data_size(ch, SOCLE_DMA_DATA_WORD);
	socle_set_dma_transfer_count(ch, socle_snd_hw.period_bytes_min);
	socle_set_dma_slice_count(ch, host->i2s_data->fifo_depth>>1);
	socle_set_dma_page_number(ch, socle_snd_hw.periods_max);
	socle_set_dma_buffer_size(ch, socle_snd_hw.buffer_bytes_max);
	socle_enable_dma(ch);
	socle_release_dma_lock(flags);
#endif
}
#endif

static int
socle_pcm_hdma_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int
socle_pcm_hdma_hw_free(struct snd_pcm_substream *substream)
{	
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int socle_pcm_hdma_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0, ch, flags;

//printk("Entered %s\n", __func__);

	if(!host)
		return ret;

	/* channel needs configuring for mem=>device, increment memory addr,
	 * sync to pclk, half-word transfers to the IIS-FIFO. */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ch = host->i2s_data->tx_dma_ch;
		flags = socle_claim_dma_lock();
		socle_disable_dma(ch);
		socle_set_dma_mode(ch, SOCLE_DMA_MODE_SLICE);
		socle_set_dma_ext_hdreq_number(ch, host->i2s_data->tx_dma_hdreq);
		socle_set_dma_burst_type(ch, host->i2s_data->burst_type);
		socle_set_dma_source_address(ch, substream->dma_buffer.addr);
		socle_set_dma_destination_address(ch, SOCLE_I2S0+0x4);
		socle_set_dma_source_direction(ch, SOCLE_DMA_DIR_INCR);
		socle_set_dma_destination_direction(ch, SOCLE_DMA_DIR_FIXED);
		socle_set_dma_data_size(ch, SOCLE_DMA_DATA_WORD);
		socle_set_dma_transfer_count(ch, socle_snd_hw.period_bytes_min);
		socle_set_dma_slice_count(ch, host->i2s_data->fifo_depth>>1);
		socle_set_dma_page_number(ch, socle_snd_hw.periods_max);
		socle_set_dma_buffer_size(ch, socle_snd_hw.buffer_bytes_max);
		socle_release_dma_lock(flags);
	} else {
		ch = host->i2s_data->rx_dma_ch;
		flags = socle_claim_dma_lock();
		socle_disable_dma(ch);
		socle_set_dma_mode(ch, SOCLE_DMA_MODE_SLICE);
		socle_set_dma_ext_hdreq_number(ch, host->i2s_data->rx_dma_hdreq);
		socle_set_dma_burst_type(ch, host->i2s_data->burst_type);
		socle_set_dma_source_address(ch, SOCLE_I2S0+0x8);
		socle_set_dma_destination_address(ch, substream->dma_buffer.addr);
		socle_set_dma_source_direction(ch, SOCLE_DMA_DIR_FIXED);
		socle_set_dma_destination_direction(ch, SOCLE_DMA_DIR_INCR);
		socle_set_dma_data_size(ch, SOCLE_DMA_DATA_WORD);
		socle_set_dma_transfer_count(ch, socle_snd_hw.period_bytes_min);
		socle_set_dma_slice_count(ch, host->i2s_data->fifo_depth>>1);
		socle_set_dma_page_number(ch, socle_snd_hw.periods_max);
		socle_set_dma_buffer_size(ch, socle_snd_hw.buffer_bytes_max);
		socle_release_dma_lock(flags);
	}
	return ret;
}

static int socle_pcm_hdma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	u32 ret = 0, flags, ch;

//printk("Entered %s\n", __func__);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if(!host)
				break;
//printk("Start HDMA\n");				
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				playback_stop = 0;
				ch = host->i2s_data->tx_dma_ch;
				flags = socle_claim_dma_lock();
				socle_enable_dma(ch);
				socle_release_dma_lock(flags);
			} else {
				capture_stop = 0;
				ch = host->i2s_data->rx_dma_ch;
				flags = socle_claim_dma_lock();	
				socle_enable_dma(ch);
				socle_release_dma_lock(flags);
			}
			break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if(!host)
				break;
//printk("Stop HDMA\n");					
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				playback_stop = 1;
				//ch = host->i2s_data->tx_dma_ch;
				//memset(substream->dma_buffer.area, 0, substream->dma_buffer.bytes);
			} else {
				capture_stop = 1;
				//ch = host->i2s_data->rx_dma_ch;
				//memset(substream->dma_buffer.area, 0, substream->dma_buffer.bytes);
			}
			
			break;

		default:
			ret = -EINVAL;
			break;
	}


	return ret;
}

static snd_pcm_uframes_t 
socle_pcm_hdma_pointer(struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return bytes_to_frames(substream->runtime,capture_buf_pos);
	else
		return bytes_to_frames(substream->runtime,playback_buf_pos);
}

static int socle_pcm_open(struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &socle_snd_hw);
	return 0;
}

static int socle_pcm_close(struct snd_pcm_substream *substream)
{
	return 0;
}


static struct snd_pcm_ops socle_pcm_ops = {
	.open		= socle_pcm_open,
	.close		= socle_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= socle_pcm_hdma_hw_params,
	.hw_free		= socle_pcm_hdma_hw_free,
	.prepare		= socle_pcm_hdma_prepare,
	.trigger		= socle_pcm_hdma_trigger,
	.pointer		= socle_pcm_hdma_pointer,
};

static int socle_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = socle_snd_hw.buffer_bytes_max;

//printk("Entered %s\n", __func__);

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;

//printk("area 0x%08X, size 0x%08X\n", (int) buf->area, buf->bytes);
	return 0;
}

static void socle_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	//DBG("Entered %s\n", __func__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	socle_free_dma(host->i2s_data->tx_dma_ch);
	socle_free_dma(host->i2s_data->rx_dma_ch);
#endif
}

#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
static struct socle_i2s_platform_data socle_i2s_data = {
#if (defined(CONFIG_ARCH_P7DK) || defined(CONFIG_ARCH_PDK_PC7210) || defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223))
	.tx_dma_ch = 0,
	.rx_dma_ch = 1,
	.tx_dma_hdreq = 6,
	.rx_dma_hdreq = 7,
	.fifo_depth = 8,
	.burst_type = SOCLE_DMA_BURST_INCR4,
#elif defined(CONFIG_ARCH_MDK_3D)
        .tx_dma_ch = 0,
        .rx_dma_ch = 2,
        .tx_dma_hdreq = 5,
        .rx_dma_hdreq = 5,
        .fifo_depth = 8,
        .burst_type = SOCLE_DMA_BURST_INCR4,
#else
	.tx_dma_ch = 4,
	.rx_dma_ch = 5,
	.tx_dma_hdreq = 0,
	.rx_dma_hdreq = 1,
	.fifo_depth = 32,
	.burst_type = SOCLE_DMA_BURST_INCR16,
#endif
};
#endif

static u64 socle_pcm_dmamask = 0xffffffff;
static int socle_pcm_new(struct snd_card *card,
	struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;
	if (!card->dev->dma_mask)
                card->dev->dma_mask = &socle_pcm_dmamask;
        if (!card->dev->coherent_dma_mask)
                card->dev->coherent_dma_mask = 0xffffffff;


	if (dai->playback.channels_min) {
		playback_substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		ret = socle_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		capture_substream = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
		ret = socle_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	host = kmalloc(sizeof(struct socle_snd_host), GFP_KERNEL);
	if(!host)
		goto out;

	host->i2s_data = &socle_i2s_data;
	host->socle_snd_tx_dma_notifier.data = host;
	host->socle_snd_rx_dma_notifier.data = host;
	host->socle_snd_tx_dma_notifier.complete = socle_snd_tx_dma_notifier_complete;
	host->socle_snd_rx_dma_notifier.complete = socle_snd_rx_dma_notifier_complete;
	host->playback_substream = playback_substream;
	host->capture_substream = capture_substream;
	ret = socle_request_dma(host->i2s_data->tx_dma_ch, "socle i2s tx hdma", &host->socle_snd_tx_dma_notifier);
	if(ret)
		goto err_no_tx_dma;
	ret = socle_request_dma(host->i2s_data->rx_dma_ch, "socle i2s rx hdma", &host->socle_snd_rx_dma_notifier);
	if(ret)
		goto err_no_dma;
#else
	host = NULL;
#endif

 out:
	return ret;
	
#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
err_no_dma:
	socle_free_dma(host->i2s_data->rx_dma_ch);
err_no_tx_dma:
	 socle_free_dma(host->i2s_data->tx_dma_ch);
#endif	 
	return ret;
	 
}

struct snd_soc_platform socle_snd_soc_platform = {
	.name		= "socle-audio",
	.pcm_ops 	= &socle_pcm_ops,
	.pcm_new	= socle_pcm_new,
	.pcm_free	= socle_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(socle_snd_soc_platform);

static int __init socle_soc_platform_init(void)
{
	return snd_soc_register_platform(&socle_snd_soc_platform);
}
module_init(socle_soc_platform_init);

static void __exit socle_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&socle_snd_soc_platform);
}
module_exit(socle_soc_platform_exit);

MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("socle PCM HDMA module");
MODULE_LICENSE("GPL");


