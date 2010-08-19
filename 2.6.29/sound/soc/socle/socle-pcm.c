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
#include <mach/pl080_dma.h>
#ifndef CONFIG_PL080_DMA
#include <mach/i2s.h>
#endif


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
	
	.channels_min = 1,
	.channels_max = 2,
	
	.buffer_bytes_max = (64 * 1024),
#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA) && !defined (CONFIG_AC97_COMPACT_MODE) && defined(CONFIG_SND_SOC_SOCLE_WM9714_AC97) 
	.period_bytes_min = 4*1024,
        .period_bytes_max = (4 * 1024),
        .periods_min = 16,
        .periods_max = 16,
#else
	.period_bytes_min = 8*1024,
        .period_bytes_max = (8 * 1024),
        .periods_min = 8,
        .periods_max = 8,
#endif
};

struct socle_runtime_data {
	spinlock_t lock;
	int state;
	unsigned int dma_loaded;
	unsigned int dma_limit;
	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	struct socle_pcm_dma_params *params;
};

#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
/* socle_pcm_enqueue
 *
 * place a dma buffer onto the queue for the dma system
 * to handle.
*/
static void socle_pcm_enqueue(struct snd_pcm_substream *substream)
{
	struct socle_runtime_data *prtd = substream->runtime->private_data;
	dma_addr_t pos = prtd->dma_pos;
	int ret;

	//DBG("Entered %s\n", __func__);

	while (prtd->dma_loaded < prtd->dma_limit) {
		unsigned long len = prtd->dma_period;
//printk("len 0x%08X\n", (u32)len);
		//DBG("dma_loaded: %d\n", prtd->dma_loaded);

		if ((pos + len) > prtd->dma_end) {
			len  = prtd->dma_end - pos;
			//DBG(KERN_DEBUG "%s: corrected dma len %ld\n", __func__, len);
		}

		ret = pl080_dma_enqueue(prtd->params->channel,
			substream, pos, len);

		if (ret == 0) {
			prtd->dma_loaded++;
//printk("%s: dma_loaded %d\n", __func__, prtd->dma_loaded);
			pos += prtd->dma_period;
			if (pos >= prtd->dma_end)
				pos = prtd->dma_start;
		} else
			break;
	}
//printk("%s: pos 0x%08x\n", __func__, pos);
	prtd->dma_pos = pos;
}

static void socle_audio_buffdone(struct pl080_dma_chan *channel,
				void *dev_id, int size,
				enum pl080_dma_buffresult result)
{
	struct snd_pcm_substream *substream = dev_id;
	struct socle_runtime_data *prtd;
	struct snd_dma_buffer *dma_buffer;
	//DBG("Entered %s\n", __func__);

	if (result == PL080_RES_ABORT || result == PL080_RES_ERR)
		return;

	dma_buffer = &substream->dma_buffer;
	if( substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		playback_buf_pos += snd_pcm_lib_period_bytes(substream);
		playback_buf_pos %= dma_buffer->bytes;
	} else {
		capture_buf_pos += snd_pcm_lib_period_bytes(substream);
		capture_buf_pos %= dma_buffer->bytes;
	}
	prtd = substream->runtime->private_data;

	if (substream)
		snd_pcm_period_elapsed(substream);

	spin_lock(&prtd->lock);
	if (prtd->state & ST_RUNNING) {
		prtd->dma_loaded--;
//printk("%s: dma_loaded %d\n", __func__, prtd->dma_loaded);
		socle_pcm_enqueue(substream);
	}

	spin_unlock(&prtd->lock);
}
#endif
static int socle_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct socle_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct socle_pcm_dma_params *dma = rtd->dai->cpu_dai->dma_data;
	unsigned long totbytes = params_buffer_bytes(params);
	int ret = 0;

//printk("Entered %s\n", __func__);
#if !defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if(!dma) {
		snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
		return 0;
	}
#else
	/* this may get called several times by oss emulation
	 * with different params -HW */
	if (prtd->params == NULL) {
		/* prepare DMA */
		prtd->params = dma;

		/*DBG("params %p, client %p, channel %d\n", prtd->params,
			prtd->params->client, prtd->params->channel);*/

		ret = pl080_dma_request(prtd->params->channel,
					  (void *)prtd->params->client, NULL);

		if (ret < 0) {
			//DBG(KERN_ERR "failed to get dma channel\n");
			return ret;
		}
	}

	pl080_dma_set_buffdone_fn(prtd->params->channel,
				    socle_audio_buffdone);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	runtime->dma_bytes = totbytes;

	spin_lock_irq(&prtd->lock);
	prtd->dma_loaded = 0;
	prtd->dma_limit = runtime->hw.periods_min;
	prtd->dma_period = params_period_bytes(params);
	prtd->dma_start = runtime->dma_addr;
	prtd->dma_pos = prtd->dma_start;
	prtd->dma_end = prtd->dma_start + totbytes;
//printk("dma_end 0x%08X\n", (u32)prtd->dma_end);
	spin_unlock_irq(&prtd->lock);
#endif
	return 0;
}

static int socle_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct socle_runtime_data *prtd = substream->runtime->private_data;

//printk("Entered %s\n", __func__);

	/* TODO - do we need to ensure DMA flushed */
	snd_pcm_set_runtime_buffer(substream, NULL);
#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	if (prtd->params) {
		pl080_dma_free(prtd->params->channel, (void *)prtd->params->client);
		prtd->params = NULL;
	}
#endif
	return 0;
}

static int socle_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct socle_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

//printk("Entered %s\n", __func__);
#if  !defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if (!prtd->params)
		return 0;
#else
	/* channel needs configuring for mem=>device, increment memory addr,
	 * sync to pclk, half-word transfers to the IIS-FIFO. */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pl080_dma_devconfig(prtd->params->channel,
					MEM_TO_PERIPHERAL, prtd->params->dma_addr);

		pl080_dma_config(prtd->params->channel,
				prtd->params->data_width, prtd->params->burst);
	} else {
		pl080_dma_devconfig(prtd->params->channel,
					PERIPHERAL_TO_MEM, prtd->params->dma_addr);
		pl080_dma_config(prtd->params->channel,
				prtd->params->data_width, prtd->params->burst);
	}

	/* flush the DMA channel */
	pl080_dma_ctrl(prtd->params->channel, PL080_DMAOP_FLUSH);
	prtd->dma_loaded = 0;
	prtd->dma_pos = prtd->dma_start;

	/* enqueue dma buffers */
	socle_pcm_enqueue(substream);
#endif
	return ret;
}

static int socle_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct socle_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

//printk("Entered %s\n", __func__);

#if  !defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	if (!prtd->params)
		return 0;
#else	
	spin_lock(&prtd->lock);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			prtd->state |= ST_RUNNING;
			
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				playback_buf_pos = 0;
			else
				capture_buf_pos = 0;
			
			pl080_dma_ctrl(prtd->params->channel, PL080_DMAOP_START);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			prtd->state &= ~ST_RUNNING;
			pl080_dma_ctrl(prtd->params->channel, PL080_DMAOP_STOP);
			break;

		default:
			ret = -EINVAL;
			break;
	}

	spin_unlock(&prtd->lock);
#endif
	return ret;
}

static snd_pcm_uframes_t
socle_pcm_pointer(struct snd_pcm_substream *substream)
{
	//struct snd_pcm_runtime *runtime = substream->runtime;
	//struct socle_runtime_data *prtd = runtime->private_data;
	//unsigned long res;
	//dma_addr_t src, dst;

	
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return bytes_to_frames(substream->runtime, capture_buf_pos);
	else
		return bytes_to_frames(substream->runtime, playback_buf_pos);
#if 0
//printk("Entered %s\n", __func__);
mdelay(10);

	spin_lock(&prtd->lock);
	pl080_dma_getposition(prtd->params->channel, &src, &dst);
//printk("src 0x%08X, dst 0x%08X, dma_start 0x%08X\n", (u32)src, (u32)dst, (u32)prtd->dma_start);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		res = dst - prtd->dma_start;
	else
		res = src - prtd->dma_start;

	spin_unlock(&prtd->lock);

	//DBG("Pointer %x %x\n", src, dst);

	/* we seem to be getting the odd error from the pcm library due
	 * to out-of-bounds pointers. this is maybe due to the dma engine
	 * not having loaded the new values for the channel before being
	 * callled... (todo - fix )
	 */

	if (res >= snd_pcm_lib_buffer_bytes(substream)) {
		if (res == snd_pcm_lib_buffer_bytes(substream))
			res = 0;
	}

	return bytes_to_frames(substream->runtime, res);
#endif
}


static int socle_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct socle_runtime_data *prtd;

	//DBG("Entered %s\n", __func__);

	snd_soc_set_runtime_hwparams(substream, &socle_snd_hw);

	prtd = kzalloc(sizeof(struct socle_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);

	runtime->private_data = prtd;
	return 0;
}

static int socle_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct socle_runtime_data *prtd = runtime->private_data;
	
	//DBG("Entered %s\n", __func__);
	
	if (!prtd)
		kfree(prtd);

	return 0;
}
#if 0
static int socle_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

//printk("Entered %s\n", __func__);

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}
#endif
static struct snd_pcm_ops socle_pcm_ops = {
	.open		= socle_pcm_open,
	.close		= socle_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= socle_pcm_hw_params,
	.hw_free		= socle_pcm_hw_free,
	.prepare		= socle_pcm_prepare,
	.trigger		= socle_pcm_trigger,
	.pointer		= socle_pcm_pointer,
	//.mmap		= socle_pcm_mmap,
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
}

static u64 socle_pcm_dmamask = 0xffffffff;

static int socle_pcm_new(struct snd_card *card,
	struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;

//printk("Entered %s\n", __func__);
#if 1
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &socle_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;
#endif

/*printk("playback %x, capture %x\n",
	(u32)pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream,
	(u32)pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream);*/
	if (dai->playback.channels_min) {
		playback_substream	= pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		ret = socle_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		capture_substream	= pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
		ret = socle_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

/*printk("playback_substream ptr 0x%08X, capture_substream ptr 0x%08X\n", 
		(u32)playback_substream, (u32)capture_substream);*/


 out:
//printk("Exit %s\n", __func__);
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
MODULE_DESCRIPTION("socle PCM ARM DMA module");
MODULE_LICENSE("GPL");

