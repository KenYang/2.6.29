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

struct socle_ac97_st {
	u32 base;
	u32 maincr;
}socle_ac97;

#undef DBG
#define DBG(fmt...)

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

static void socle_ac97_warm_reset(struct snd_ac97 *ac97)
{
printk("Entered %s, line %d\n", __func__, __LINE__);
	socle_ac97_write(AACI_SYNC, 1, socle_ac97.base);
	udelay(100);
	socle_ac97_write(AACI_SYNC, 0, socle_ac97.base);
}

static void socle_ac97_cold_reset(struct snd_ac97 *ac97)
{
printk("Entered %s, line %d\n", __func__, __LINE__);
	socle_ac97_write(AACI_RESET, 0, socle_ac97.base);
	udelay(100);
	socle_ac97_write(AACI_RESET, 1, socle_ac97.base);
}

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read	= socle_ac97_codec_read,
	.write	= socle_ac97_codec_write,
	.warm_reset	= socle_ac97_warm_reset,
	.reset	= socle_ac97_cold_reset,
};

static int socle_ac97_init(void)
{
	int ret, i;
printk("Entered %s, line %d\n", __func__, __LINE__);
	socle_ac97.base = (u32)ioremap(AC97_BASE, SZ_4K);
	if (!socle_ac97.base) {
		printk("Error : can't ioremap socle_ac97 base\n");
		ret = -ENOMEM;
		goto out_error;
	}

	socle_ac97.maincr = MAINCR_IE | MAINCR_SL1RXEN | MAINCR_SL1TXEN |
		       MAINCR_SL2RXEN | MAINCR_SL2TXEN;

	socle_ac97_write(AACI_MAINCR, socle_ac97.maincr, socle_ac97.base);
	
	return 0;
	
out_ioremap:
	iounmap((void __iomem *)socle_ac97.base);
	
out_error:
	return ret;
}

static void socle_ac97_exit(struct platform_device *pdev,
				struct snd_soc_dai *dai)
{
	iounmap((void __iomem *)socle_ac97.base);
}

EXPORT_SYMBOL_GPL(soc_ac97_ops);
arch_initcall(socle_ac97_init);
module_exit(socle_ac97_exit);

