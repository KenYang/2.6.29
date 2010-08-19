/*
 * socle-ac97.c  --  ALSA Soc Audio Layer
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/hardware.h>
#include <asm/dma.h>
#include <mach/dma.h>
#include <linux/vmalloc.h>

#include "socle-ac97.h"
#include "socle-pcm.h"

//#define CONFIG_AC97_DEBUG
#ifdef CONFIG_AC97_DEBUG
	#define DBG(fmt, args...) printk("socle-ac97: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif


struct socle_ac97_st {
	u32 base;
	u32 irq;
	u32 maincr;
	struct aaci_runtime {
		u32 base;
		void *fifo;
		u32 	cr;
		void	*start;
		void	*end;
		void	*ptr;
		u32 period;
		u32 period_byes;
		int bytes;
		unsigned int fifosz;
	}playback, capture;
	
};

extern u32 playback_buf_pos;
extern u32 capture_buf_pos;

static struct socle_ac97_st socle_ac97;

#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
static struct socle_dma_client socle_dma_client_out = {
	.name = "AC97 PCM Stereo out"
};

static struct socle_dma_client socle_dma_client_in = {
	.name = "AC97 PCM Stereo in"
};

static struct socle_pcm_dma_params socle_ac97_pcm_stereo_out = {
	.client		= &socle_dma_client_out,
	.channel		= 3,								/* request number */
	.dma_addr	= AC97_BASE + AACI_DR1,			/* physical address */
#ifdef CONFIG_AC97_COMPACT_MODE
	.data_width	= 4,
	.burst = 1,
#else
	.data_width	= 2,
	.burst = 4,
#endif
};

static struct socle_pcm_dma_params socle_ac97_pcm_stereo_in = {
	.client		= &socle_dma_client_in,
	.channel		= 2,								/* request number */
	.dma_addr	= AC97_BASE + AACI_DR1,			/* physical address */
#ifdef CONFIG_AC97_COMPACT_MODE
	.data_width	= 4,
	.burst = 1,
#else
	.data_width	= 2,
	.burst = 4,
#endif
};
#endif

static char __initdata banner[] = "\n#######SOCLE AC97 Host Driver, (c) 2009 SOCLE Corp.\n\n";

static inline void
socle_ac97_write(u32 reg, u32 val, u32 base)
{
	//DBG("reg=0x%04x, val=0x%08x, base=0x%08x\n", reg, val, base);
	iowrite32(val, base + reg);
}

static inline u32
socle_ac97_read(u32 reg, u32 base)
{
	u32 val = ioread32(base + reg);
	//DBG("reg=0x%04x, val=0x%08x, base=0x%08x\n", reg, val, base);
	return val;
}

static void socle_ac97_codec_write(struct snd_ac97 *ac97, unsigned short reg,
			    unsigned short val)
{
	u32 v;
	int timeout = 5000;
	u32 io_base;
	
	DBG("reg = 0x%x  ;  value = 0x%x\n", reg, val);
	
	io_base = socle_ac97.base;
	
	if (ac97->num >= 4)
		return;

	/*
	 * P54: You must ensure that AACI_SL2TX is always written
	 * to, if required, before data is written to AACI_SL1TX.
	 */
	socle_ac97_write(AACI_SL2TX, val << 4, io_base);
	socle_ac97_write(AACI_SL1TX, reg << 12, io_base);
	
	/*
	 * Wait for the transmission of both slots to complete.
	 */
	do {
		v = socle_ac97_read(AACI_SLFR, io_base);
	} while ((v & (SLFR_1TXB|SLFR_2TXB)) && --timeout);

	if (!timeout)
		printk("timeout waiting for write to complete\n");

}

/*
 * Read an AC'97 register.
 */
static unsigned short socle_ac97_codec_read(struct snd_ac97 *ac97, unsigned short reg)
{
	u32 v;
	int timeout = 5000;
	int retries = 10;
	u32 io_base;
	
	io_base = socle_ac97.base;

	if (ac97->num >= 4)
		return ~0;

	/*
	 * Write the register address to slot 1.
	 */
	socle_ac97_write(AACI_SL1TX, (reg << 12) | (1 << 19), io_base);

	/*
	 * Wait for the transmission to complete.
	 */
	do {
		v = socle_ac97_read(AACI_SLFR, io_base);
	} while ((v & SLFR_1TXB) && --timeout);

	if (!timeout) {
		printk("timeout on slot 1 TX busy\n");
		v = ~0;
		goto out;
	}

	/*
	 * Give the AC'97 codec more than enough time
	 * to respond. (42us = ~2 frames at 48kHz.)
	 */
	udelay(42);

	/*
	 * Wait for slot 2 to indicate data.
	 */
	timeout = 5000;
	do {
		cond_resched();
		v = socle_ac97_read(AACI_SLFR, io_base) & (SLFR_1RXV|SLFR_2RXV);
	} while ((v != (SLFR_1RXV|SLFR_2RXV)) && --timeout);

	if (!timeout) {
		printk("timeout on RX valid\n");
		v = ~0;
		goto out;
	}

	do {
		v = socle_ac97_read(AACI_SL1RX, io_base) >> 12;
		if (v == reg) {
			v = socle_ac97_read(AACI_SL2RX, io_base) >> 4;
			break;
		} else if (--retries) {
			printk("ac97 read back fail.  retry\n");
			continue;
		} else {
			printk("wrong ac97 register read back\n");
			v = ~0;
		}
	} while (retries);

	DBG("reg = 0x%x  ;  value = 0x%x\n", reg, v);
 out:
	return v;
}

static inline void aaci_chan_wait_ready(struct aaci_runtime *aacirun)
{
	u32 val;
	int timeout = 5000;

	do {
		val = socle_ac97_read(AACI_SR, aacirun->base);
	} while (val & (SR_TXB|SR_RXB) && timeout--);
}


static void socle_ac97_warm_reset(struct snd_ac97 *ac97)
{
	socle_ac97_write(AACI_SYNC, 1, socle_ac97.base);
	udelay(100);
	socle_ac97_write(AACI_SYNC, 0, socle_ac97.base);
}

static void socle_ac97_cold_reset(struct snd_ac97 *ac97)
{
	socle_ac97_write(AACI_RESET, 0, socle_ac97.base);
	udelay(100);
	socle_ac97_write(AACI_RESET, 1, socle_ac97.base);
}

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
static irqreturn_t socle_ac97_irq(int irq, void *devid)
{
	u32 tmp;
	
	tmp = socle_ac97_read(AACI_ISR, socle_ac97.base);
	
	if (tmp & ISR_ORINTR) {
		//printk("RX overrun\n");
		socle_ac97_write(AACI_INTCLR, ICLR_RXOEC1, socle_ac97.base);
	}

	if (tmp & ISR_RXTOINTR) {
		//printk("RX timeout\n");
		socle_ac97_write(AACI_INTCLR, ICLR_RXTOFEC1, socle_ac97.base);
	}

	if (tmp & ISR_RXINTR) {
		u16 *ptr = socle_ac97.capture.ptr;

		do {
			unsigned int len = socle_ac97.capture.fifosz;
			u32 val;

			if (!(socle_ac97.capture.cr & CR_EN))
				break;

			val = socle_ac97_read(AACI_SR, socle_ac97.base);
			if (!(val & SR_RXHF))
				break;

			if (!(val & SR_RXFF))
				len >>= 1;
				
			for( ; len > 0; len -= 16) {

				asm(
					"ldmia	%1, {r0, r1, r2, r3}\n\t"
					"strh		r0, [%0], #2\n\t"
					"strh		r1, [%0], #2\n\t"
					"strh		r2, [%0], #2\n\t"
					"strh		r3, [%0], #2"
					: "=r" (ptr)
					: "r" (socle_ac97.capture.fifo)
					: "r0", "r1", "r2", "r3", "cc");

				capture_buf_pos += 0x08;
				socle_ac97.capture.period += 0x08;

				capture_buf_pos %= capture_substream->dma_buffer.bytes;

				if(socle_ac97.capture.period >= (capture_substream->runtime->period_size * 4)) {
					socle_ac97.capture.period %= (capture_substream->runtime->period_size * 4);
					snd_pcm_period_elapsed(capture_substream);
				}
			
				if (ptr >= (u16 *)socle_ac97.capture.end) {
					ptr = (u16 *)socle_ac97.capture.start;
				}
			}
		} while(1);
		socle_ac97.capture.ptr = ptr;
	}

	if (tmp & ISR_URINTR) {
		//printk("TX underrun\n");
		socle_ac97_write(AACI_INTCLR, ICLR_TXUEC1, socle_ac97.base);
	}

	if (tmp & ISR_TXINTR) {
		u16 *ptr = socle_ac97.playback.ptr;

		do {
			unsigned int len = socle_ac97.playback.fifosz;
			u32 val;

			if (!(socle_ac97.playback.cr & CR_EN))
				break;
			
			val = socle_ac97_read(AACI_SR, socle_ac97.base);
			
			if (!(val & SR_TXHE))
				break;
			if (!(val & SR_TXFE))
				len >>= 1;
			
			/* writing 16 bytes at a time */
			for ( ; len > 0; len -= 16) {
				asm(
					"ldrh		r0, [%0], #2\n\t"
					"ldrh		r1, [%0], #2\n\t"
					"ldrh		r2, [%0], #2\n\t"
					"ldrh		r3, [%0], #2\n\t"
					"stmia	%1, {r0, r1, r2, r3}"
					: "=r" (ptr)
					: "r" (socle_ac97.playback.fifo)
					: "r0", "r1", "r2", "r3", "cc");

				playback_buf_pos += 0x8;
				socle_ac97.playback.period += 0x8;
				
				playback_buf_pos %= playback_substream->dma_buffer.bytes;
				
				if(socle_ac97.playback.period >= (playback_substream->runtime->period_size * 4)) {
					socle_ac97.playback.period %= (playback_substream->runtime->period_size * 4);
					snd_pcm_period_elapsed(playback_substream);
				}
		
				if (ptr >= (u16 *)socle_ac97.playback.end) {
					ptr = (u16 *)socle_ac97.playback.start;
				}

			}
		} while (1);

		socle_ac97.playback.ptr = ptr;
	}

	return tmp ? IRQ_HANDLED : IRQ_NONE;
}
#endif

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read	= socle_ac97_codec_read,
	.write	= socle_ac97_codec_write,
	.warm_reset	= socle_ac97_warm_reset,
	.reset	= socle_ac97_cold_reset,
};

static void socle_pcm_playback_stop(struct aaci_runtime *aacirun)
{

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
	{
		u32 ie;
		printk("###---Disable playback interrupt\n");
		ie = socle_ac97_read(AACI_IE, aacirun->base);
		ie &= ~(IE_URIE|IE_TXIE);
		socle_ac97_write(AACI_IE, ie, aacirun->base);
	}
#endif
	aacirun->cr &= ~CR_EN;
	aaci_chan_wait_ready(aacirun);
	socle_ac97_write(AACI_TXCR, aacirun->cr, aacirun->base);

#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
	printk("###---Disable playback HDMA\n");
	socle_ac97_write(AACI_MAINCR, socle_ac97_read(AACI_MAINCR ,  aacirun->base)
			&(~MAINCR_DMAEN),  aacirun->base);
#endif
}

static void socle_pcm_playback_start(struct aaci_runtime *aacirun)
{
	
	aaci_chan_wait_ready(aacirun);

	aacirun->cr |= CR_EN;
	socle_ac97_write(AACI_TXCR, aacirun->cr, aacirun->base);

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
	{
		u32 ie;
		printk("###---Enable playback interrupt\n");
		ie = socle_ac97_read(AACI_IE, aacirun->base);
		ie |= IE_URIE | IE_TXIE;
		socle_ac97_write(AACI_IE, ie, aacirun->base);
	}
#endif

#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
	printk("###---Enable playback HDMA\n");
	socle_ac97_write(AACI_MAINCR, socle_ac97_read(AACI_MAINCR , aacirun->base)
			| MAINCR_DMAEN, aacirun->base);
#endif

}

static void socle_pcm_capture_stop(struct aaci_runtime *aacirun)
{
	
	aaci_chan_wait_ready(aacirun);

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
	{
		u32 ie;
		printk("###---Disable capture interrupt\n");
		ie = socle_ac97_read(AACI_IE, aacirun->base);
		ie &= ~(IE_ORIE | IE_RXIE);
		socle_ac97_write(AACI_IE, ie, aacirun->base);
	}
#endif

	aacirun->cr &= ~CR_EN;
	socle_ac97_write(AACI_RXCR, aacirun->cr, aacirun->base);

#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
	printk("###---Disable capture HDMA\n");
	socle_ac97_write(AACI_MAINCR, socle_ac97_read(AACI_MAINCR , aacirun->base)
			&(~MAINCR_DMAEN), aacirun->base);
#endif
}

static void socle_pcm_capture_start(struct aaci_runtime *aacirun)
{
	
	aaci_chan_wait_ready(aacirun);

	aacirun->cr |= 0xf << 17;

	aacirun->cr |= CR_EN;
	socle_ac97_write(AACI_RXCR, aacirun->cr, aacirun->base);

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
	{
		u32 ie;
		printk("###---Enable capture interrupt\n");
		ie = socle_ac97_read(AACI_IE, aacirun->base);
		ie |= IE_ORIE |IE_RXIE; // overrun and rx interrupt -- half full
		socle_ac97_write(AACI_IE, ie, aacirun->base);
	}
#endif

#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
	//mdelay(10);		//there have a bug, it must set a delay befor dma enable
	printk("###---Enable capture HDMA\n");
	socle_ac97_write(AACI_MAINCR, socle_ac97_read(AACI_MAINCR , aacirun->base)
			| MAINCR_DMAEN, aacirun->base);
#endif
}


static int socle_ac97_probe(struct platform_device *pdev,
			      struct snd_soc_dai *dai)
{
	int ret, i;
	u32 io_base;
	
		
	socle_ac97.base = (u32)ioremap(AC97_BASE, SZ_4K);
	if (!socle_ac97.base) {
		printk("Error : can't ioremap socle_ac97 base\n");
		ret = -ENOMEM;
		goto out_error;
	}

	io_base = socle_ac97.base;

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
	ret = request_irq(AC97_IRQ, socle_ac97_irq, IRQF_DISABLED, "AC97", pdev);
	if (ret) {
		printk("Error : AC97 interrupt request fail\n");
		goto out_ioremap;
	}
#endif

	socle_ac97.maincr = MAINCR_IE | MAINCR_SL1RXEN | MAINCR_SL1TXEN |
		       MAINCR_SL2RXEN | MAINCR_SL2TXEN;
	
	/*
	 * Playback uses AACI channel 0
	 */
	socle_ac97.playback.base = io_base + AACI_CSCH1;
	socle_ac97.playback.fifo = (u32 *)(io_base + AACI_DR1);

	/*
	 * Capture uses AACI channel 0
	 */
	socle_ac97.capture.base = io_base + AACI_CSCH1;
	socle_ac97.capture.fifo = (u32 *)(io_base + AACI_DR1);

	for (i = 0; i < 4; i++) {
		u32 base = io_base + i * 0x14;

		socle_ac97_write(AACI_IE, 0, base);
		socle_ac97_write(AACI_TXCR, 0, base);
		socle_ac97_write(AACI_RXCR, 0, base);
	}

	socle_ac97_write(AACI_INTCLR, 0x1ffff, io_base);
	socle_ac97_write(AACI_MAINCR, socle_ac97.maincr, io_base);
	
	return 0;

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
out_ioremap:
	iounmap((void __iomem *)socle_ac97.base);
#endif
out_error:
	return ret;
}

static void socle_ac97_remove(struct platform_device *pdev,
				struct snd_soc_dai *dai)
{
	free_irq(AC97_IRQ, pdev);
	iounmap((void __iomem *)socle_ac97.base);
}

#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
int socle_ac97_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		//DBG("dma_area = 0x%x\n", (u32)runtime->dma_area);
		//DBG("dma_bytes = 0x%x\n", (u32)runtime->dma_bytes);
		//DBG("dma_period = 0x%x\n", (u32)runtime->period_size);

		socle_ac97.playback.start = (void *)runtime->dma_area;
		socle_ac97.playback.end = socle_ac97.playback.start + runtime->dma_bytes;
		socle_ac97.playback.ptr = socle_ac97.playback.start;
		socle_ac97.playback.period = 0;
		socle_ac97.playback.period_byes = runtime->period_size*4;
		playback_buf_pos = 0;

	} else {

		//DBG("dma_area = 0x%x\n", (u32)runtime->dma_area);
		//DBG("dma_bytes = 0x%x\n", (u32)runtime->dma_bytes);
		//DBG("dma_period = 0x%x\n", (u32)runtime->period_size);
		
		socle_ac97.capture.start = (void *)runtime->dma_area;
		socle_ac97.capture.end = socle_ac97.capture.start + runtime->dma_bytes;
		socle_ac97.capture.ptr = socle_ac97.capture.start;
		socle_ac97.capture.period = 0;
		socle_ac97.capture.period_byes = runtime->period_size*4;
		capture_buf_pos = 0;
	}

	return 0;
	
}
#endif

static int socle_ac97_hifi_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rtd->dai->cpu_dai->dma_data = &socle_ac97_pcm_stereo_out;
	}
	else {
		rtd->dai->cpu_dai->dma_data = &socle_ac97_pcm_stereo_in;
	}
#endif

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#ifdef CONFIG_AC97_COMPACT_MODE
		socle_ac97.playback.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4 | CR_COMPACT;
#else
		socle_ac97.playback.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4;
#endif
		socle_ac97.playback.fifosz = 32;	//32Bytes
		socle_ac97_write(AACI_TXCR, socle_ac97.playback.cr, socle_ac97.base);
	} else {
#ifdef CONFIG_AC97_COMPACT_MODE
		socle_ac97.capture.cr = CR_FEN |CR_SZ16 | CR_SL3 | CR_SL4 | CR_COMPACT;
#else
		socle_ac97.capture.cr = CR_FEN |CR_SZ16 | CR_SL3 | CR_SL4;
#endif
		socle_ac97.capture.fifosz = 32;	//32Bytes
		socle_ac97_write(AACI_RXCR, socle_ac97.capture.cr, socle_ac97.base);
	}

	return 0;
}

static int socle_ac97_surr51_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rtd->dai->cpu_dai->dma_data = &socle_ac97_pcm_stereo_out;
	}
	else {
		rtd->dai->cpu_dai->dma_data = &socle_ac97_pcm_stereo_in;
	}
#endif

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#ifdef CONFIG_AC97_COMPACT_MODE
		socle_ac97.playback.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4 | \
							CR_SL5 | CR_SL6 | CR_SL7 | CR_SL8 | CR_COMPACT;
#else
		socle_ac97.playback.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4 | \
							CR_SL5 | CR_SL6 | CR_SL7 | CR_SL8;
#endif
		socle_ac97.playback.fifosz = 32;	//32Bytes
		
		socle_ac97_write(AACI_TXCR, socle_ac97.playback.cr, socle_ac97.base);
	} else {
#ifdef CONFIG_AC97_COMPACT_MODE
		socle_ac97.capture.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4 | \
							CR_SL5 | CR_SL6 | CR_SL7 | CR_SL8 | CR_COMPACT;
#else
		socle_ac97.capture.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4 | \
							CR_SL5 | CR_SL6 | CR_SL7 | CR_SL8;
#endif
		
		socle_ac97.capture.fifosz = 32;	//32Bytes
		
		socle_ac97_write(AACI_RXCR, socle_ac97.capture.cr, socle_ac97.base);
	}

	return 0;
}

static int socle_ac97_spdif_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
#ifdef CONFIG_SND_SOC_SOCLE_HW_DMA
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rtd->dai->cpu_dai->dma_data = &socle_ac97_pcm_stereo_out;
	}
	else {
		rtd->dai->cpu_dai->dma_data = &socle_ac97_pcm_stereo_in;
	}
#endif

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#ifdef CONFIG_AC97_COMPACT_MODE
		socle_ac97.playback.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4 | CR_COMPACT;
#else
		socle_ac97.playback.cr = CR_FEN | CR_SZ16 | CR_SL3 | CR_SL4;
#endif
		socle_ac97.playback.fifosz = 32;	//32Bytes
		socle_ac97_write(AACI_TXCR, socle_ac97.playback.cr, socle_ac97.base);
	} else {
#ifdef CONFIG_AC97_COMPACT_MODE
		socle_ac97.capture.cr = CR_FEN |CR_SZ16 | CR_SL3 | CR_SL4 | CR_COMPACT;
#else
		socle_ac97.capture.cr = CR_FEN |CR_SZ16 | CR_SL3 | CR_SL4;
#endif
		socle_ac97.capture.fifosz = 32;	//32Bytes
		socle_ac97_write(AACI_RXCR, socle_ac97.capture.cr, socle_ac97.base);
	}
	
	return 0;
}

static int socle_ac97_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			socle_pcm_playback_start(&socle_ac97.playback);
		else
			socle_pcm_capture_start(&socle_ac97.capture);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			socle_pcm_playback_stop(&socle_ac97.playback);
		else
			socle_pcm_capture_stop(&socle_ac97.capture);
		break;
	}

	return 0;
}

#define SOCLE_AC97_HIFI_RATES	\
	(SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define SOCLE_AC97_HIFI_PCM_FORMATS \
	(SNDRV_PCM_FORMAT_S8 | SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE)

#define SOCLE_AC97_SURR51_RATES	\
	(SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define SOCLE_AC97_SURR51_PCM_FORMATS \
	(SNDRV_PCM_FORMAT_S8 | SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE)

#define SOCLE_AC97_SPDIF_RATES	\
	(SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define SOCLE_AC97_SPDIF_PCM_FORMATS \
	(SNDRV_PCM_FORMAT_S8 | SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE)

struct snd_soc_dai socle_ac97_dai[] = {
{
	.name = "socle-ac97-hifi",
	.id = 0,
	.ac97_control = 1,
	.probe = socle_ac97_probe,
	.remove = socle_ac97_remove,
	.playback = {
		.stream_name = "AC97_hifi Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SOCLE_AC97_HIFI_RATES,
		.formats = SOCLE_AC97_HIFI_PCM_FORMATS},
	.capture = {
		.stream_name = "AC97_hifi Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SOCLE_AC97_HIFI_RATES,
		.formats = SOCLE_AC97_HIFI_PCM_FORMATS,},
	.ops = {
		.hw_params = socle_ac97_hifi_hw_params,
#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
		.prepare = socle_ac97_prepare,
#endif
		.trigger = socle_ac97_trigger},
},
{
	.name = "socle-ac97-surround51",
	.id = 1,
	.ac97_control = 1,
	.playback = {
		.stream_name = "AC97_surr51 Playback",
		.channels_min = 1,
		.channels_max = 6,
		.rates = SOCLE_AC97_SURR51_RATES,
		.formats = SOCLE_AC97_SURR51_PCM_FORMATS,},
	.capture = {
		.stream_name = "AC97_surr51 Capture",
		.channels_min = 1,
		.channels_max = 6,
		.rates = SOCLE_AC97_SURR51_RATES,
		.formats = SOCLE_AC97_SURR51_PCM_FORMATS,},
	.ops = {
		.hw_params = socle_ac97_surr51_hw_params,
#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
		.prepare = socle_ac97_prepare,
#endif
		.trigger = socle_ac97_trigger},
},
{
	.name = "socle-ac97-spdif",
	.id = 2,
	.ac97_control = 1,
	.playback = {
		.stream_name = "AC97_spdif Playback",
		.channels_min = 1,
		.channels_max = 6,
		.rates = SOCLE_AC97_SPDIF_RATES,
		.formats = SOCLE_AC97_SPDIF_PCM_FORMATS,},
	.capture = {
		.stream_name = "AC97_spdif Capture",
		.channels_min = 1,
		.channels_max = 6,
		.rates = SOCLE_AC97_SPDIF_RATES,
		.formats = SOCLE_AC97_SPDIF_PCM_FORMATS,},
	.ops = {
		.hw_params = socle_ac97_spdif_hw_params,
#ifdef CONFIG_SND_SOC_SOCLE_NO_DMA
		.prepare = socle_ac97_prepare,
#endif
		.trigger = socle_ac97_trigger},
},
};

EXPORT_SYMBOL_GPL(socle_ac97_dai);
EXPORT_SYMBOL_GPL(soc_ac97_ops);

static int __init socle_ac97_init(void)
{
	printk(banner);
	return snd_soc_register_dais(socle_ac97_dai, ARRAY_SIZE(socle_ac97_dai));
}
module_init(socle_ac97_init);

static void __exit socle_ac97_exit(void)
{
	snd_soc_unregister_dais(socle_ac97_dai, ARRAY_SIZE(socle_ac97_dai));
}
module_exit(socle_ac97_exit);


MODULE_AUTHOR("Jerry Hsieh");
MODULE_DESCRIPTION("AC97 driver for the Socle chip");
MODULE_LICENSE("GPL");