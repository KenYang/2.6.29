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
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <mach/i2s-regs.h>

#include "socle-pcm.h"
#if  defined(CONFIG_SND_SOC_SOCLE_HW_DMA) && defined(CONFIG_SOCLE_DMA_PL080)
#include <mach/pl080_ch.h>
#endif

#if defined(CONFIG_SND_SOC_SOCLE_UDA1342TS)
#define 	EXT_CLOCK  33868800
#elif defined(CONFIG_SND_SOC_SOCLE_MS6335) /* MOSA 6335 */
#define 	EXT_CLOCK  16934400
#else
#define 	EXT_CLOCK  33868800
#endif

static void __iomem *socle_i2s_base;

/*
 *  Read register
 */
 static u32 inline
socle_i2s_reg_read(u32 reg)
{
	return ioread32(socle_i2s_base+reg);
}

/*
 *  Write register
 */
static void inline
socle_i2s_reg_write(u32 val, u32 reg)
{
//printk("%s: reg 0x%08X, value 0x%08X\n", __func__, reg, val);
	iowrite32(val, socle_i2s_base+reg);
}

#if  defined(CONFIG_SND_SOC_SOCLE_HW_DMA) && defined(CONFIG_SOCLE_DMA_PL080)
static struct socle_dma_client socle_dma_client_out = {
	.name = "I2S PCM Stereo out"
};

static struct socle_dma_client socle_dma_client_in = {
	.name = "I2S PCM Stereo in"
};

static struct socle_pcm_dma_params socle_i2s_pcm_stereo_out = {
	.client		= &socle_dma_client_out,
	.channel		= DMACH_I2S_TX,								/* request number */
	.dma_addr	= SOCLE_I2S0 + SOCLE_I2S_TXR,	/* physical address */
	.data_width	= 4,
#if defined(CONFIG_ARCH_MDK_FHD)
	.burst = 16,
#endif
};

static struct socle_pcm_dma_params socle_i2s_pcm_stereo_in = {
	.client		= &socle_dma_client_in,
	.channel		= DMACH_I2S_RX,
	.dma_addr	= SOCLE_I2S0 + SOCLE_I2S_RXR,
	.data_width	= 4,
#if defined(CONFIG_ARCH_MDK_FHD)
	.burst = 16,
#endif
};
#endif

static u32 socle_i2s_conf = SOCLE_I2S_TX_N_RST |
	SOCLE_I2S_RX_N_RST |
#if  !defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	SOCLE_I2S_HDMA_REQ_1_DIS |
	SOCLE_I2S_HDMA_REQ_2_DIS |
#else
	SOCLE_I2S_HDMA_REQ_1_EN|
	SOCLE_I2S_HDMA_REQ_2_EN |
#endif
	SOCLE_I2S_HDMA_IF_1_TX |
	SOCLE_I2S_HDMA_IF_2_RX |
	SOCLE_I2S_OP_NORMAL |
	SOCLE_I2S_TX_OP_STP |
	SOCLE_I2S_RX_OP_STP;

extern struct snd_pcm_substream *playback_substream;
extern struct snd_pcm_substream *capture_substream;
extern u32 playback_buf_pos;
extern u32 capture_buf_pos;

#if !defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
static irqreturn_t
socle_snd_isr(int irq_no, struct snd_soc_dai *soc_dai)
{
	u32 int_stat;
	u32  *buf;
	struct snd_dma_buffer *dma_buffer;
	
	int_stat = socle_i2s_reg_read(SOCLE_I2S_ISR);
//printk("Entered %s int_stat 0x%08X\n", __func__, int_stat);

	/* Check wether is is the rx fifo overrun interrupt*/
	if (int_stat & SOCLE_I2S_RX_FIFO_OVR_INT) {
printk("rx overrun\n");		
		/* Clear the rx fifo */
		socle_i2s_reg_write(
				socle_i2s_reg_read(SOCLE_I2S_RXCTL) |
				SOCLE_I2S_RX_FIFO_CLR,SOCLE_I2S_RXCTL);

		/* Disable the interrupt */
		socle_i2s_reg_write(
				socle_i2s_reg_read(SOCLE_I2S_IER) &
				~(SOCLE_I2S_RX_FIFO_TRIG_INT_EN |
				  SOCLE_I2S_RX_FIFO_OVR_INT_EN),SOCLE_I2S_IER);

		/* Stop the rx transmission */
		socle_i2s_reg_write(
				socle_i2s_conf |
				(socle_i2s_reg_read(SOCLE_I2S_OPR) & SOCLE_I2S_TX_OP_STR),SOCLE_I2S_OPR);

		goto out;
	}

	/* Check whether it is the tx fifo data trigger interrupt */
	if (int_stat & SOCLE_I2S_TX_FIFO_TRIG_INT) {
//printk("%s:		playback_substream %x\n", __func__, (u32)playback_substream);
		dma_buffer = &(playback_substream->dma_buffer);
#if 0
		while ((socle_i2s_reg_read(SOCLE_I2S_FIFOSTS) & SOCLE_I2S_TX_FIFO_FULL) != SOCLE_I2S_TX_FIFO_FULL) {
			buf = (u32 *) &(dma_buffer->area[playback_buf_pos]);
			socle_i2s_reg_write(	*buf, SOCLE_I2S_TXR);
			playback_buf_pos += 4;
			playback_buf_period_pos +=4;
		}
		playback_buf_pos %= dma_buffer->bytes;
		if( playback_buf_period_pos > playback_substream->runtime->period_size) {
//printk("period_size 0x%08X, buf_size 0x%08X\n",(u32) playback_substream->runtime->period_size, dma_buffer->bytes);
			playback_buf_period_pos %= playback_substream->runtime->period_size;
			snd_pcm_period_elapsed(playback_substream);
		}
#else
//printk("dma_buffer->area %x\n", (u32)dma_buffer->area);
		while ((socle_i2s_reg_read(SOCLE_I2S_FIFOSTS) & SOCLE_I2S_TX_FIFO_FULL) != SOCLE_I2S_TX_FIFO_FULL) {
			buf = (u32 *) &(dma_buffer->area[playback_buf_pos]);
			socle_i2s_reg_write(	*buf, SOCLE_I2S_TXR);
			playback_buf_pos += 4;
			playback_buf_pos %= dma_buffer->bytes;
			if( playback_buf_pos % snd_pcm_lib_period_bytes(playback_substream) == 0) {
//printk("%s: playback_buf_pos 0x%08X\n", __func__, playback_buf_pos);
				snd_pcm_period_elapsed(playback_substream);
			}
		}
#endif
	}

	/* Check whether it is the rx fifo data trigger interrupt */
	if (int_stat & SOCLE_I2S_RX_FIFO_TRIG_INT) {
		dma_buffer = &(capture_substream->dma_buffer);
		while ((socle_i2s_reg_read(SOCLE_I2S_FIFOSTS) & SOCLE_I2S_RX_FIFO_EMPTY) != SOCLE_I2S_RX_FIFO_EMPTY) {
			buf = (u32 *) &(dma_buffer->area[capture_buf_pos]);
			*buf = socle_i2s_reg_read(SOCLE_I2S_RXR);
			capture_buf_pos += 4;
			capture_buf_pos %= dma_buffer->bytes;
			if( capture_buf_pos % snd_pcm_lib_period_bytes(capture_substream) == 0) {
				snd_pcm_period_elapsed(capture_substream);
			}
		}
	}
out:
//printk("Exit %s\n", __func__);
	return IRQ_HANDLED;
}
#endif

static int socle_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
#if defined(CONFIG_SOCLE_DMA_PL080) && defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#endif
	u32 iisctl = 0, fs, oversampling_rate, ratio;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		socle_i2s_reg_write(	socle_i2s_conf|
					SOCLE_I2S_TX_RST, SOCLE_I2S_OPR);
	} else {
		socle_i2s_reg_write(	socle_i2s_conf|
					SOCLE_I2S_RX_RST, SOCLE_I2S_OPR);
	}

//printk("Entered %s\n", __func__);
#if defined(CONFIG_SOCLE_DMA_PL080) && defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rtd->dai->cpu_dai->dma_data = &socle_i2s_pcm_stereo_out;
	}
	else {
		rtd->dai->cpu_dai->dma_data = &socle_i2s_pcm_stereo_in;
	}
#endif
	iisctl &= ~SOCLE_I2S_SAMPLE_RES_MASK;

	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S8:
			iisctl |= SOCLE_I2S_SAMPLE_RES_8;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			iisctl |= SOCLE_I2S_SAMPLE_RES_16;
			break;
		case SNDRV_PCM_FORMAT_S20_3LE:
			iisctl |= SOCLE_I2S_SAMPLE_RES_20;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			iisctl |= SOCLE_I2S_SAMPLE_RES_24;
			break;
		default:
			return -EINVAL;
	}

	iisctl &= ~SOCLE_I2S_OVERSAMPLING_RATE_MASK;
	iisctl &= ~SOCLE_I2S_RATIO_MASK;
#if 0	
	fs = EXT_CLOCK / params_rate(params);
	ratio = 6;

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
#else
	fs = EXT_CLOCK / params_rate(params);
	oversampling_rate = SOCLE_I2S_OVERSAMPLING_RATE_64;
	ratio = (fs/64+1)/2*2;
	//printk("ratio = %x\n", ratio);
#endif
//printk("oversampling_rate = %d\n", oversampling_rate);
	iisctl |= oversampling_rate;
	iisctl |= SOCLE_I2S_RATIO(ratio);
	iisctl |= SOCLE_I2S_BUS_IF_I2S;

	switch(runtime->channels) {
		case 1:
			iisctl |= SOCLE_I2S_MONO;
			break;
		case 2:
			iisctl |= SOCLE_I2S_STEREO;
			break;
		default:
			break;
	}
	

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		socle_i2s_reg_write(socle_i2s_reg_read(SOCLE_I2S_RXCTL) & ~SOCLE_I2S_MASTER,
										SOCLE_I2S_RXCTL);
#if 1
		socle_i2s_reg_write(iisctl |SOCLE_I2S_MASTER, SOCLE_I2S_TXCTL);
#else
		socle_i2s_reg_write(iisctl, SOCLE_I2S_TXCTL);
#endif
	} else {
		socle_i2s_reg_write(socle_i2s_reg_read(SOCLE_I2S_TXCTL) & ~SOCLE_I2S_MASTER,
										SOCLE_I2S_TXCTL);
#if 1
		socle_i2s_reg_write(iisctl |SOCLE_I2S_MASTER, SOCLE_I2S_RXCTL);
#else
		socle_i2s_reg_write(iisctl, SOCLE_I2S_RXCTL);
#endif
	}

//printk("%s IISCTL: 0x%08X\n", __func__, iisctl | SOCLE_I2S_MASTER);
	return 0;
}


static int socle_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	int ret = 0;
//printk("Entered %s\n", __func__);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
//printk("Trigger Start\n");
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				capture_buf_pos = 0;
#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	#if 0
					i2s_conf = (socle_i2s_conf & ~SOCLE_I2S_HDMA_REQ_2_DIS)
											| SOCLE_I2S_HDMA_REQ_2_EN;
	#endif
#else
//printk("enable rx interrupt\n");
					/* Set the interrupt enable */
					socle_i2s_reg_write(socle_i2s_reg_read(SOCLE_I2S_IER) |
									SOCLE_I2S_RX_FIFO_TRIG_INT_EN |
									SOCLE_I2S_RX_FIFO_OVR_INT_EN,SOCLE_I2S_IER);
#endif
//printk("i2s_conf %08X\n", i2s_conf);
				/* Start to receive */
				socle_i2s_reg_write(socle_i2s_conf |SOCLE_I2S_RX_OP_STR, SOCLE_I2S_OPR);
			} else {
				playback_buf_pos = 0;
#if defined(CONFIG_SND_SOC_SOCLE_HW_DMA)
	#if 0
					i2s_conf = (socle_i2s_conf & ~SOCLE_I2S_HDMA_REQ_1_DIS)
											| SOCLE_I2S_HDMA_REQ_1_EN;
	#endif
#else
//printk("enable tx interrupt\n");
					/* Set the interrupt enable */
					socle_i2s_reg_write(socle_i2s_reg_read(SOCLE_I2S_IER) | 
									SOCLE_I2S_TX_FIFO_TRIG_INT_EN,SOCLE_I2S_IER);
#endif
//printk("i2s_conf %08X\n", i2s_conf);
				/* Start to transmission */
				socle_i2s_reg_write(socle_i2s_conf |SOCLE_I2S_TX_OP_STR, SOCLE_I2S_OPR);
//printk("%s: i2s_conf 0x%08X\n", __func__, socle_i2s_conf|SOCLE_I2S_TX_OP_STR);
			}
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
//printk("Trigger Stop\n");
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
					/* Disable the interrupt */
					socle_i2s_reg_write(socle_i2s_reg_read(SOCLE_I2S_IER) &
									~(SOCLE_I2S_RX_FIFO_TRIG_INT_EN |
									SOCLE_I2S_RX_FIFO_OVR_INT_EN),SOCLE_I2S_IER);
#endif
				
				/* Stop the transmission */
				socle_i2s_reg_write(socle_i2s_conf, SOCLE_I2S_OPR);
			} else {
#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
					/* Disable the interrupt */
					socle_i2s_reg_write(
					socle_i2s_reg_read(SOCLE_I2S_IER) &
									~SOCLE_I2S_TX_FIFO_TRIG_INT_EN,SOCLE_I2S_IER);
#endif
				
				/* Stop the transmission */
				socle_i2s_reg_write( socle_i2s_conf, SOCLE_I2S_OPR);
//printk("%s: socle_i2s_conf 0x%08X\n", __func__, socle_i2s_conf);
			}
			break;
		default:
			break;
	}


//printk("Exit %s\n", __func__);
	return ret;
}


static int socle_i2s_probe(struct platform_device *pdev,
			     struct snd_soc_dai *dai)
{
	int ret = 0;
//printk("Entered %s\n", __func__);

	socle_i2s_base = ioremap(SOCLE_I2S0, 0x100);
	if (!socle_i2s_base) {
		ret = -ENOMEM;
		goto release_mem;
	}
#if 0
	irq = platform_get_irq(pdev, 0);
	if(irq < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = -ENXIO;
		goto release_mem;/*FIXME*/
	}
#endif

#if defined(CONFIG_SND_SOC_SOCLE_NO_DMA)
	ret = request_irq(IRQ_I2S0, (irq_handler_t)socle_snd_isr, IRQF_DISABLED, pdev->name, dai);
	if(ret) {
		goto release_mem;/*FIXME*/
	}
#endif

	/* Set the interrupt trigger level */
	socle_i2s_reg_write(
			SOCLE_I2S_TX_INT_TRIG_LEV_ALMOST_EMPTY |
			SOCLE_I2S_RX_INT_TRIG_LEV_HALF_FULL, SOCLE_I2S_FIFOSTS);

	socle_i2s_reg_write(	socle_i2s_conf|
					SOCLE_I2S_TX_RST| 
					SOCLE_I2S_RX_RST, SOCLE_I2S_OPR);
	
release_mem:
//printk("Exit %s\n", __func__);
	return ret;
}

#ifdef CONFIG_PM
static u32 i2s_opr;
static u32 i2s_txctl;
static u32 i2s_rxctl;
static u32 i2s_fifosts;
static u32 i2s_ier;
static u32 i2s_isr;
static int socle_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
#if 0
	socle_i2s_reg_write(0, SOCLE_I2S_OPR);
	socle_i2s_reg_write(0, SOCLE_I2S_TXCTL);
	socle_i2s_reg_write(0, SOCLE_I2S_RXCTL);
	socle_i2s_reg_write(0, SOCLE_I2S_FIFOSTS);
	socle_i2s_reg_write(0, SOCLE_I2S_IER);
	socle_i2s_reg_write(0, SOCLE_I2S_ISR);
#else
	i2s_opr = socle_i2s_reg_read(SOCLE_I2S_OPR);
	i2s_txctl = socle_i2s_reg_read(SOCLE_I2S_TXCTL);
	i2s_rxctl = socle_i2s_reg_read(SOCLE_I2S_RXCTL);
        i2s_fifosts = socle_i2s_reg_read(SOCLE_I2S_FIFOSTS);
        i2s_ier = socle_i2s_reg_read(SOCLE_I2S_IER);
        i2s_isr = socle_i2s_reg_read(SOCLE_I2S_ISR);
	socle_i2s_reg_write(socle_i2s_conf, SOCLE_I2S_OPR);
#endif
	return 0;
}

static int socle_i2s_resume(struct snd_soc_dai *cpu_dai)
{
#if 0
	socle_i2s_default_set();
#else
	socle_i2s_reg_write(i2s_opr, SOCLE_I2S_OPR);
        socle_i2s_reg_write(i2s_txctl, SOCLE_I2S_TXCTL);
        socle_i2s_reg_write(i2s_rxctl, SOCLE_I2S_RXCTL);
        socle_i2s_reg_write(i2s_fifosts, SOCLE_I2S_FIFOSTS);
        socle_i2s_reg_write(i2s_ier, SOCLE_I2S_IER);
        socle_i2s_reg_write(i2s_isr, SOCLE_I2S_ISR);
#endif
	return 0;
}
#else
#define socle_i2s_suspend NULL
#define socle_i2s_resume NULL
#endif
#define SOCLE_I2S_RATE  (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

struct snd_soc_dai socle_i2s_dai = {
#ifdef CONFIG_SND_SOC_WM9714
	.ac97_control = 1,
#endif
	.name = "socle-i2s",
	.id = 0,
	.probe = socle_i2s_probe,
	.suspend = socle_i2s_suspend,
	.resume = socle_i2s_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SOCLE_I2S_RATE,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE /*| SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE*/,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SOCLE_I2S_RATE,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = {
		.hw_params      = socle_i2s_hw_params,
		.trigger                = socle_i2s_trigger,
	},
};
EXPORT_SYMBOL_GPL(socle_i2s_dai);

static int __init socle_i2s_init(void)
{
	return snd_soc_register_dai(&socle_i2s_dai);
}
module_init(socle_i2s_init);

static void __exit socle_i2s_exit(void)
{
	snd_soc_unregister_dai(&socle_i2s_dai);
}
module_exit(socle_i2s_exit);

/* Module information */
MODULE_AUTHOR("CY Chen");
MODULE_DESCRIPTION("socle I2S SoC Interface");
MODULE_LICENSE("GPL");
