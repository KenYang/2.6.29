/*
 * drivers/mtd/nand/socle_nand.c
 *
 * Copyright 2005 Socle

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <asm/dma.h>

#include <mach/socle_nand_nfc_regs.h>

#define MOVE_ECC_REG

//#define NFC_DEBUG
#ifdef NFC_DEBUG
#define NDEBUG(fmt, args...) printk(fmt, ## args)
#else
#define NDEBUG(fmt, args...)
#endif

enum {
	HM = 0,
	BCH4 = 1,
	BCH_8= 2,
	BCH_16 = 3,
};
#define ECC_MODE BCH4 

enum {
	ECC_NONE = 0,
	ECC_SOFTWARE	= 1,
	ECC_HARDWARE	= 2, // Need under BCH4 cond
};

#if defined(CONFIG_MTD_NAND_SOCLE_HWECC)
#define ECC_TYPE  ECC_HARDWARE
#elif defined(CONFIG_MTD_NAND_SOCLE_SWECC)
#define ECC_TYPE ECC_SOFTWARE
#elif defined(CONFIG_MTD_NAND_SOCLE_NOECC)
#define ECC_TYPE ECC_NONE
#else
#error unknow ecc type!
#endif

/* error code and state */
enum {
	ERR_NONE	= 0,
	ERR_ECC	= -1,
	ERR_ERASE	= -2,
};

/*
 * Default NAND flash controller configuration setup by the
 * bootloader. This configuration is used only when pdata->keep_config is set
 */

struct socle_nand_timing {
    u8 tWBns;    /** WE# HIGH to busy in nano second */
    u8 tWHRns;   /** WE# HIGH to RE# LOW in nanosecond */
    u8 tWPns;    /** WE# pulse width in nano second */
    u8 tRPns;    /** RE# pulse width in nano second */
    u8 tWHns;    /** WE# pulse width HIGH in nano second */
};

struct socle_nand_flash {
	struct socle_nand_timing *timing; /* NAND Flash timing */

	u32 flash_width;   /* Width of Flash memory (DWIDTH_M) */
 	u32 page_per_block;/* Pages per block (PG_PER_BLK) */
	u32 page_size;	   /* Page size in bytes (PAGE_SZ) */
       u32 sp_size;       /* SP size in byte*/
	u32 num_blocks;	   /* Number of physical blocks in Flash */
	u32 chip_id;
};

struct socle_nand_info {
	struct nand_chip	nand_chip;
	
	struct socle_nand_flash *flash_info;

	void __iomem	*regs;
#ifdef CONFIG_NFC_INT
	int irq;
#endif
	struct platform_device	 *pdev;

	unsigned int buf_start;
	unsigned int buf_count;
	unsigned char *buf_addr;

	uint32_t reg_flctr;

	/* saved column/page_addr during CMD_SEQIN */
	int seqin_column;
	int seqin_page_addr;

	int use_hwecc;	/* use HW ECC ? */
	u8 bypass_check_ecc;

#ifdef CONFIG_NFC_DMA
	u16 dma_type;		/** type of DMA transfer */
	u16 dma_width;		/** size of DMA transfer */
#endif
	int retcode;

	u32 reg_data;

	/* calculated from socle_nand_flash data */
	size_t oob_size;

	unsigned int col_addr_cycles;
	unsigned int row_addr_cycles;
#ifdef CONFIG_NFC_IP_PROTECT
	u8 use_ip_protect;
#endif	
};

static int socle_nfc_readid(struct socle_nand_info *socle_nand, uint32_t *id);

/******************************************************************************/
struct socle_nand_timing st_D08GW3C2C_timing = {
	.tWBns	= 100,
	.tWHRns	= 60,
	.tWPns	= 12,
	.tRPns	= 12,
	.tWHns	= 10,
};

static struct socle_nand_flash D08GW3C2C = {
	.timing		= &st_D08GW3C2C_timing,

	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 8,
	.num_blocks	= 4096,
	.chip_id	= 0x9551d320,
};

/******************************************************************************/
struct socle_nand_timing st_D08GW3C2B_timing = {

	.tWBns	= 100,
	.tWHRns	= 60,
	.tWPns	= 12,
	.tRPns	= 12,
	.tWHns	= 10,
};

static struct socle_nand_flash D08GW3C2B = {
	.timing		= &st_D08GW3C2B_timing,

	.page_per_block	= 128,
	.page_size	= 2048,
	.flash_width	= 8,
	.num_blocks	= 8192,
	.chip_id	= 0xa514d320,
};
/******************************************************************************/
struct socle_nand_timing samsung_K9GAG08U0M_timing = {
	.tWBns	= 255,//100,
	.tWHRns	= 60,
	.tWPns	= 12,
	.tRPns	= 12,
	.tWHns	= 10,
};

static struct socle_nand_flash K9GAG08U0M = {
	.timing		= &samsung_K9GAG08U0M_timing,

	.page_per_block	= 128,
	.page_size	= 4096,
	.flash_width	= 8,
	.num_blocks	= 4096,
	.chip_id	= 0xb614d5ec,
};

/******************************************************************************/
struct socle_nand_timing samsung_K9K8G08U0B_timing = {
        .tWBns  = 255,//100,
        .tWHRns = 60,
        .tWPns  = 12,
        .tRPns  = 12,
        .tWHns  = 10,
};

static struct socle_nand_flash K9K8G08U0B = {
        .timing         = &samsung_K9K8G08U0B_timing,

        .page_per_block = 64,
        .page_size      = 2048,
        .flash_width    = 8,
        .num_blocks     = 8192,
        .chip_id        = 0x9551d3ec,
};

/******************************************************************************/
struct socle_nand_timing hynix_HY27UF081G2A_timing = {
	.tWBns	= 100,
	.tWHRns	= 60,
	.tWPns	= 12,
	.tRPns	= 12,
	.tWHns	= 10,
};

static struct socle_nand_flash HY27UF081G2A = {
	.timing		= &hynix_HY27UF081G2A_timing,

	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 8,
	.num_blocks	= 1024,
	.chip_id	= 0x1d80f1ad,
};


/******************************************************************************/
struct socle_nand_timing hynix_HY27G088G5M_timing = {
	.tWBns	= 255,
	.tWHRns	= 60,
	.tWPns	= 12,
	.tRPns	= 12,
	.tWHns	= 10,
};

static struct socle_nand_flash HY27G088G5M = {
	.timing		= &hynix_HY27G088G5M_timing,

	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 8,
	.num_blocks	= 8192,
	.chip_id	= 0x9580dcad,
};

/******************************************************************************/
static struct socle_nand_flash *builtin_flash_types[] = {
	&K9GAG08U0M,
	&K9K8G08U0B,
	&D08GW3C2C,
	&D08GW3C2B,
	&HY27UF081G2A,
	&HY27G088G5M,
};

static void socle_nfc_write(struct socle_nand_info *socle_nand,u32 val, u32 reg_addr)
{
	iowrite32(val, reg_addr +  socle_nand->regs + 0x00001300);
}
static u32 socle_nfc_read(struct socle_nand_info *socle_nand,u32 reg_addr)
{  
	return (ioread32(reg_addr +  socle_nand->regs + 0x00001300));
}

u32 socle_nfc_select_timing_mask(u8 mask_length)
{
	u32 mask = 0;
	switch (mask_length)	{
	        case 3:
	            mask = (BIT0 | BIT1 | BIT2);
	            break;
	        case 4:
	            mask = (BIT0 | BIT1 | BIT2 | BIT3);
	            break;
	        case 5:
	            mask = (BIT0 | BIT1 | BIT2 | BIT3 | BIT4);
	            break;
	        case 6:
	            mask = (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
	            break;
	        case 7:
	            mask = (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
	            break;
	        default:
	            break;
	}
	return mask;
}

#define NANO_SECOND 1000000000
#define TIMER_BITS 7
extern unsigned long socle_scu_cpu_clock_get(void);
extern unsigned long socle_scu_ahb_clock_get(void);
extern void 
socle_nfc_configure_timing(struct socle_nand_info *socle_nand,struct socle_nand_timing *timings, u8 check_device_done)
{
	long int TWB, TWHR, TWP, TRP, TWH;
	u32 thclk =0; 

	u8 trp_shift = TIMER_BITS - 2;
	u8 twp_shift = trp_shift + TIMER_BITS - 1;
	u8 twhr_shift = twp_shift + TIMER_BITS - 1;
	u8 twb_shift = twhr_shift + TIMER_BITS - 1;

	u8 twh_mask =  TIMER_BITS - 2;
	u8 trp_mask = TIMER_BITS - 1;
	u8 twp_mask =  TIMER_BITS - 1;
	u8 twhr_mask = TIMER_BITS - 1;
	u8 twb_mask =  TIMER_BITS ;
	u32 mask_temp = 0;

	if (check_device_done == 0) {
		socle_nfc_write(socle_nand,0x3FFFFFFF, NF_SFR_FLCONF);
		NDEBUG("\nUse Default Timing " );
		NDEBUG("\nNF_SFR_FLCONF= %x",socle_nfc_read(socle_nand,NF_SFR_FLCONF));
		return;
	}
	NDEBUG("\ncpu clock = %x",socle_scu_cpu_clock_get() );
	NDEBUG("\nahp clock = %d",socle_scu_ahb_clock_get() );
	thclk = NANO_SECOND / socle_scu_ahb_clock_get();  //Need Measure from Board

	mask_temp = socle_nfc_select_timing_mask(twb_mask);
	if ((TWB = timings->tWBns / thclk - 1) < 0 )
		TWB = 0;
	if (timings->tWBns > ((TWB + 1) * thclk))
		++TWB;

	TWB = (TWB &  mask_temp);
	NDEBUG("\n TWB = %x",TWB );

	mask_temp = socle_nfc_select_timing_mask(twhr_mask);
	if((TWHR = timings->tWHRns / thclk - 1) < 0)
		TWHR = 0;
	if (timings->tWHRns > ((TWHR + 1) * thclk))
		++TWHR;

	TWHR = (TWHR &  mask_temp);
	NDEBUG("\n TWHR = %x",TWHR);

	mask_temp = socle_nfc_select_timing_mask(twp_mask);
	if((TWP = timings->tWPns / thclk - 2) < 0)
		TWP = 0;
	if (timings->tWPns > ((TWP + 1) * thclk))
		++TWP;

	TWP = (TWP &  mask_temp);
	NDEBUG("\n TWP = %x",TWP );

	mask_temp = socle_nfc_select_timing_mask(trp_mask);
	if ((TRP = timings->tRPns / thclk - 1) < 0)
		TRP = 0;
	if (timings->tRPns > ((TRP + 1) * thclk))
		++TRP;

	TRP = (TRP &  mask_temp);
	NDEBUG("\n TRP = %x",TRP );

	mask_temp = socle_nfc_select_timing_mask(twh_mask);
	if ((TWH = timings->tWHns / thclk - 1) < 0)
		TWH = 0;
	if (timings->tWHns > ((TWH + 1) * thclk))
		++TWH;

	TWH = (TWH &  mask_temp);
	NDEBUG("\n TWH = %x",TWH );

	socle_nfc_write(socle_nand,((u32)TWB << twb_shift)
		| ((u32)TWHR << twhr_shift) | ((u32)TWP << twp_shift) | ((u32)TRP << trp_shift)
		| ((u32)TWH), NF_SFR_FLCONF);

	NDEBUG("NF_SFR_FLCONF= %x\n",socle_nfc_read(socle_nand,NF_SFR_FLCONF));
}

extern int 
socle_nfc_busy_check(struct socle_nand_info *socle_nand)
{
	volatile u16 timeout = 0xFFFF;
	/* wait until controller is busy*/
	while (socle_nfc_read(socle_nand,NF_SFR_FLSTATE) & NF_STATE_FSM_BUSY);
	/* wait until memory is busy*/
	do {
		if (timeout == 0)
			return -1;
		timeout--;
	} while (!(socle_nfc_read(socle_nand,NF_SFR_FLSTATE) & NF_STATE_RNB_ST));
	return 0; 
}
#ifdef CONFIG_NFC_DMA
static void socle_nand_dma_configure(struct socle_nand_info *socle_nand,u8 command)
{
	switch (command) {
		case NF_DMA_SET_SINGLE_ADRR_DEC:
		case NF_DMA_SET_BURST_ADRR_DEC:
		case NF_DMA_SET_SINGLE_ADRR_INC:
		case NF_DMA_SET_BURST_ADRR_INC:
		case NF_DMA_SET_BURST_4_ADDR_INC:
		case NF_DMA_SET_BURST_8_ADDR_INC:
		case NF_DMA_SET_BURST_16_ADDR_INC:
		case NF_DMA_SET_STREAM_BURST:
			socle_nand->dma_type = command;
			break;
		case NF_DMA_SET_SIZE_8:
		case NF_DMA_SET_SIZE_16:
		case NF_DMA_SET_SIZE_32:
			socle_nand->dma_width = command;
			break;
	}
}

static u8 socle_nand_dma_pre_transfer(struct socle_nand_info *socle_nand,u8 *buffer, u16 size, u16 offset, u8 dir)
{
	u32 dma_ctrl;

	socle_nfc_write(socle_nand,0, NF_SFR_FLDMACTRL);
	/*  Wait for DMA Ready */
	while((socle_nfc_read(socle_nand,NF_SFR_FLSTATE) & NF_STATE_DMA_BUSY));
	while ((socle_nfc_read(socle_nand,NF_SFR_FLSTATE) & NF_STATE_FSM_BUSY));

	switch(socle_nand->dma_width) {
		case NF_DMA_SIZE_32:
			size = size >> 2;
			if (((u32)buffer & 3) != 0)
				return -1;
			break;
		case NF_DMA_SIZE_16:
			size = size >> 1;
			if (((u32)buffer & 1) != 0)
				return -1;
			break;
	}
	/* Prepare DMA */
	socle_nfc_write(socle_nand,(u32) buffer, NF_SFR_FLDMAADDR);
	socle_nfc_write(socle_nand,(((u32)(size) << 16) & 0xFFFF0000) | ((u32)offset & 0xFFFF), NF_SFR_FLDMACNTR);
	socle_nfc_write(socle_nand,0, NF_SFR_FLSTATE);

	/* Start transmitting */
	dma_ctrl =
		(dir ? NF_DMA_READ : NF_DMA_WRITE)      /* Read or write data */
		| socle_nand->dma_type
		| socle_nand->dma_width;
	socle_nfc_write(socle_nand,dma_ctrl, NF_SFR_FLDMACTRL);
	return 0;
}

static u8 socle_nand_dma_finish_transfer(struct socle_nand_info *socle_nand)
{
	/* Wait for transfer finish */
 	while((socle_nfc_read(socle_nand,NF_SFR_FLSTATE) & NF_STATE_DMA_BUSY));
	while ((socle_nfc_read(socle_nand,NF_SFR_FLSTATE) & NF_STATE_FSM_BUSY));
	while ((socle_nfc_read(socle_nand,NF_SFR_FLDMACTRL) & NF_DMACTRL_DMA_BUSY));

	/* check if there are any errors of DMA transmission */
	if (socle_nfc_read(socle_nand,NF_SFR_FLDMACTRL) & NF_DMA_ERR_FLAG)
		return -1;

	socle_nfc_write(socle_nand,0, NF_SFR_FLDMACTRL);
	return 0;
}


static u8 socle_nand_dma_transfer(struct socle_nand_info *socle_nand,u8 *buffer, u16 size, u16 offset, u8 dir)
{
	socle_nand_dma_pre_transfer(socle_nand,buffer, size, offset, dir);
	socle_nfc_write(socle_nand,socle_nfc_read(socle_nand,NF_SFR_FLDMACTRL) |NF_DMA_START_FLAG, NF_SFR_FLDMACTRL);
	return socle_nand_dma_finish_transfer(socle_nand);
}
#endif

#ifdef CONFIG_NFC_INT
static irqreturn_t socle_nand_irq(int irq, void *devid)
{
	struct socle_nand_info *socle_nand = devid;

	//socle_nfc_int_flag = 1;
	//socle_nfc_int_stat = socle_nfc_read(info,NF_SFR_FLSTATE);
	socle_nfc_write(socle_nand,0, NF_SFR_FLSTATE);	

	return IRQ_HANDLED;
}
#endif

static int socle_nand_dev_ready(struct mtd_info *mtd)
{
	struct socle_nand_info *socle_nand = mtd->priv;
	return (socle_nfc_read(socle_nand, NF_SFR_FLSTATE) & NF_STATE_RNB_ST) ? 1 : 0;
}

static inline int is_buf_blank(uint8_t *buf, size_t len)
{
	for (; len > 0; len--) {
		if (*buf++ != 0xff)
			return 0;
	}
	return 1;
}
static void socle_nand_ecc_ctrl(struct socle_nand_info *socle_nand, int hwecc)
{
	u32 flctr = socle_nfc_read(socle_nand,NF_SFR_FLCTRL);

	if (hwecc) 
		flctr |=  NF_CTRL_ECC_EN;
	else
		flctr &= ~(NF_CTRL_ECC_EN);

	socle_nand->reg_flctr = flctr;
	socle_nfc_write(socle_nand,flctr, NF_SFR_FLCTRL);
}
#if 1	//for test
unsigned char tmp_buf[0x1080];
int by_pass = 0;
#endif
static void socle_nand_cmdfunc(struct mtd_info *mtd, unsigned command,
				int column, int page_addr)
{
	struct socle_nand_info *socle_nand = mtd->priv;
#ifdef CONFIG_MTD_NAND_SOCLE_HWECC
	u32 ecc_status;
#endif
	uint32_t id;
	int hwecc;
	//int i, data_exist;

	socle_nfc_busy_check(socle_nand); 
	switch (command) {
		case NAND_CMD_READOOB:
			NDEBUG("_RB:c=0x%x,p=0x%x\n",column,page_addr);
			socle_nand->buf_count = mtd->writesize + mtd->oobsize;
			socle_nand->buf_start = mtd->writesize + column;

			socle_nfc_write(socle_nand,page_addr << (socle_nand->nand_chip.page_shift+1), NF_SFR_FLADDR0L);
			socle_nfc_write(socle_nand,page_addr >> 16, NF_SFR_FLADDR0H);
			socle_nfc_write(socle_nand,NFG_CMD_PAGE_READ, NF_SFR_FLCOMM);
			socle_nfc_busy_check(socle_nand); 
			break;
			
		case NAND_CMD_READ0:
		case NAND_CMD_READ1:
			//printk("Read page 0x%08x\n", page_addr);
			NDEBUG("_R0:c=0x%x, p=0x%x\n",column,page_addr);
			socle_nand->bypass_check_ecc = 0;
			socle_nand->retcode = ERR_NONE;
			socle_nand->buf_start = column;
			socle_nand->buf_count = mtd->writesize + mtd->oobsize;
			memset(socle_nand->buf_addr, 0xFF, socle_nand->buf_count);
			
			socle_nfc_write(socle_nand,page_addr << (socle_nand->nand_chip.page_shift+1), NF_SFR_FLADDR0L);
			socle_nfc_write(socle_nand,page_addr >> 16, NF_SFR_FLADDR0H);
			socle_nfc_write(socle_nand,NFG_CMD_PAGE_READ, NF_SFR_FLCOMM);
			socle_nfc_busy_check(socle_nand); 
#if 0	//for test
	if ( page_addr == 0x7ff80) {
		printk("TEST read 0x7ff8 page empty\n");
		memset(socle_nand->buf_addr, 0xFF, 0x1080);
	}
#endif 
#ifdef MOVE_ECC_REG
    #if 1
    	if (by_pass) goto skip_check_ecc;
    #endif
    #ifdef  CONFIG_MTD_NAND_SOCLE_HWECC
	ecc_status = socle_nfc_read(socle_nand,NF_SFR_FLECCSTATUS);
	if ( ((ecc_status >> 16) != 0x00FF) && (((ecc_status >> 16) & 0x00FF) != 0x0) && 
		(((ecc_status >> 16) & 0x00FF) != (ecc_status & 0x00FF))) {
		printk("err fail: ecc_status = 0x%08X, page_addr = 0x%08X\n", ecc_status, page_addr);
	}
    #endif
    skip_check_ecc:
#endif
#if 0
		if (is_buf_blank(socle_nand->buf_addr, mtd->writesize))
		{
			socle_nand->bypass_check_ecc = 1;
			NDEBUG("ECC bypass\n");
		}
#ifdef MOVE_ECC_REG
		else
		{
			ecc_status = socle_nfc_read(socle_nand,NF_SFR_FLECCSTATUS);
			if (( (u8)((ecc_status >> 16) & 0xFF)) != ((u8)(ecc_status & 0xFF))) {
				socle_nand->retcode = ERR_ECC;
				printk("_R0:c=0x%x, p=0x%x\n",column,page_addr);
				printk("ecc fail: use_ecc=0x%x,Status =0x%x\n",socle_nand->use_hwecc ,ecc_status);
			}
		}
#endif
#endif
			break;

		case NAND_CMD_RNDOUT:
			NDEBUG("_Out:c=0x%x,p=0x%x\n",column,page_addr);
			socle_nand->buf_start = column;
			socle_nand->buf_count = mtd->writesize + mtd->oobsize;
			break;		

		case NAND_CMD_SEQIN:
			NDEBUG("_SN:c=0x%x,p=0x%x\n",column,page_addr);
			socle_nand->buf_start = column;
			socle_nand->buf_count = mtd->writesize + mtd->oobsize;
			memset(socle_nand->buf_addr, 0xff, socle_nand->buf_count);

			socle_nfc_write(socle_nand,0, NF_SFR_FLSTATE);
			socle_nfc_write(socle_nand,page_addr << (socle_nand->nand_chip.page_shift+1), NF_SFR_FLADDR1L);
			socle_nfc_write(socle_nand,page_addr >> 16, NF_SFR_FLADDR1H);
			/* save column/page_addr for next CMD_PAGEPROG */
			socle_nand->seqin_column = column;
			socle_nand->seqin_page_addr = page_addr;
			break;

		case NAND_CMD_PAGEPROG:
#if 0	//for test
	printk("Program page 0x%08X\n", socle_nand->seqin_page_addr);
	by_pass = 1;
	memcpy(tmp_buf, socle_nand->buf_addr, 0x1080);
	socle_nand_cmdfunc(mtd, NAND_CMD_READ0, socle_nand->seqin_column, socle_nand->seqin_page_addr);
	data_exist = 0;
	for (i=0; i<0x1000; i++) {
		if( socle_nand->buf_addr[i] != 0xff ) {
			data_exist = 1;
			break;
		}
	}
	if ( data_exist) {
		printk("Program page, but data exist in page 0x%08x\n", socle_nand->seqin_page_addr);
	}
	by_pass = 0;
	memcpy(socle_nand->buf_addr, tmp_buf, 0x1080);
#endif 
			hwecc = socle_nand->seqin_column >= mtd->writesize ? 0 : socle_nand->use_hwecc;
			socle_nand_ecc_ctrl(socle_nand, hwecc);
			NDEBUG("PG:c=%x,p=%x,HWECC=%x\n",socle_nand->seqin_column,socle_nand->seqin_page_addr,socle_nand->use_hwecc );
			socle_nfc_write(socle_nand,NFG_CMD_PROGRAM_PAGE, NF_SFR_FLCOMM);
			socle_nfc_busy_check(socle_nand);
			if( hwecc != socle_nand->use_hwecc )
				socle_nand_ecc_ctrl(socle_nand, socle_nand->use_hwecc);
			break;

		case NAND_CMD_ERASE1:
			NDEBUG("_E1:c=0x%x, p=0x%x\n",column,page_addr);
			socle_nfc_write(socle_nand,0, NF_SFR_FLSTATE);
			socle_nfc_write(socle_nand,page_addr <<(socle_nand->nand_chip.page_shift+1), NF_SFR_FLADDR1L);
			socle_nfc_write(socle_nand,page_addr >> 16, NF_SFR_FLADDR1H);    
			socle_nfc_write(socle_nand,NFG_CMD_BLOCK_ERASE, NF_SFR_FLCOMM);
			if (socle_nfc_read(socle_nand,NF_SFR_FLSTATE) & NF_STATE_PROT_ERR) {
				socle_nand->retcode = ERR_ERASE;
				printk("Erase Err!!\n");
			}
			socle_nfc_busy_check(socle_nand);
			break;

		case NAND_CMD_ERASE2:
			NDEBUG("_E2:c=0x%x,p=0x%x\n",column,page_addr);
			break;

		case NAND_CMD_READID:
			NDEBUG("_ID:c=0x%x,p=0x%x\n",column,page_addr);
			socle_nand->buf_start = 0;
			socle_nand->buf_count = 8;
			socle_nfc_readid(socle_nand, &id);
			break;

		case NAND_CMD_STATUS:
			NDEBUG("_STATUS:c=0x%x,p=0x%x\n",column,page_addr);
			socle_nand->buf_start = 0;
			socle_nand->buf_count = 4;
			memset(socle_nand->buf_addr, 0xff, socle_nand->buf_count);
			socle_nfc_write(socle_nand,NFG_CMD_READ_STATUS, NF_SFR_FLCOMM);
			socle_nand->reg_data = socle_nfc_read(socle_nand,NF_SFR_FLDATA);
#ifdef CONFIG_NFC_IP_PROTECT
			if (socle_nand->use_ip_protect == 1)
				socle_nand->reg_data = socle_nand->reg_data & 0x7f7f7f7f;
#endif		
			memcpy(socle_nand->buf_addr, &socle_nand->reg_data, 4);
			break;

		case NAND_CMD_RESET:
			NDEBUG("_RESET:c=0x%x, p=0x%x\n",column,page_addr);
			/* RESET without waiting for the ready line */  
			socle_nfc_write(socle_nand,NFG_CMD_RESET, NF_SFR_FLCOMM);
			break;

#ifdef CONFIG_CMD_NAND_LOCK_UNLOCK
		case NAND_CMD_LOCK:
			socle_nfc_write(socle_nand,NFS_COM_BL_LOCK, NF_SFR_FLCOMM);
			break;

		case NAND_CMD_LOCK_TIGHT:
			socle_nfc_write(socle_nand,NFS_COM_BL_LOCK_DOWN, NF_SFR_FLCOMM);
			break;

		case NAND_CMD_UNLOCK1:
			socle_nfc_write(socle_nand,page_addr << (socle_nand->nand_chip.page_shift+1), NF_SFR_FLADDR1L);
			socle_nfc_write(socle_nand,page_addr >> 16, NF_SFR_FLADDR1H);
			break;

		case NAND_CMD_UNLOCK2:
			socle_nfc_write(socle_nand,page_addr << (socle_nand->nand_chip.page_shift+1), NF_SFR_FLADDR0L);
			socle_nfc_write(socle_nand,page_addr >> 16, NF_SFR_FLADDR0H);
			socle_nfc_write(socle_nand,NFS_COM_BL_UNLOCK, NF_SFR_FLCOMM);
			break;

		case NAND_CMD_LOCK_STATUS:
			NDEBUG("_Lock_status:c=0x%x,p=0x%x\n",column,page_addr);
			socle_nand->buf_start = 0;
			socle_nand->buf_count = 4;
			memset(socle_nand->buf_addr, 0xff, socle_nand->buf_count);
			socle_nfc_write(socle_nand,NFS_COM_READ_BL_LOCK_STAT, NF_SFR_FLCOMM);
			socle_nand->reg_data = socle_nfc_read(socle_nand,NF_SFR_FLDATA);		
			memcpy(socle_nand->buf_addr, (unsigned char*) &socle_nand->reg_data , 4);
			break;
#endif
#ifdef CONFIG_NFC_IP_PROTECT
		case NAND_BLOCK_PROTECT_ON1:
			// ps: page_addr = begin_protect_block
			NDEBUG("_protect on_1:c=0x%x,p=0x%x\n",column,page_addr);	
			socle_nfc_write(socle_nand,(u32)page_addr>>(socle_nand->nand_chip.phys_erase_shift - socle_nand->nand_chip.page_shift), 
				NF_SFR_FLPB0);

			break;
		case NAND_BLOCK_PROTECT_ON2:
			// ps: page_addr= end_protect_block;
			NDEBUG("_protect on_2:c=0x%x,p=0x%x\n",column,page_addr);	
			socle_nfc_write(socle_nand,(u32) (page_addr>>(socle_nand->nand_chip.phys_erase_shift - socle_nand->nand_chip.page_shift))<< 16,
				NF_SFR_FLPB0);
			socle_nand->use_ip_protect = 1;
			NDEBUG("Protect on :NF_SFR_FLPB0 = 0x%x\n", socle_nfc_read(socle_nand,NF_SFR_FLPB0));
			break;
			
		case NAND_BLOCK_PROTECT_OFF:
			NDEBUG("_protect 0ff:c=0x%x,p=0x%x\n",column,page_addr);	
			socle_nfc_write(socle_nand,0, NF_SFR_FLPB0);
			socle_nand->use_ip_protect = 0;
			NDEBUG("Protect off :NF_SFR_FLPB0 = 0x%x\n", socle_nfc_read(socle_nand,NF_SFR_FLPB0));
			break;
#endif
		default:
			printk(KERN_ERR "non-supported command.\n");
			break;

	}

}

static uint8_t socle_nand_read_byte(struct mtd_info *mtd)
{

       struct socle_nand_info *socle_nand = mtd->priv;
        u8 byte_value = 0xff; 

	if (socle_nand->buf_start < socle_nand->buf_count)
		/* Has just send a new command? */
		byte_value = socle_nand->buf_addr[socle_nand->buf_start++];
	NDEBUG("_read one byte= %x\n",byte_value);		
	return byte_value;
}

static u16 socle_nand_read_word(struct mtd_info *mtd)
{
	struct socle_nand_info *socle_nand = mtd->priv;
        u16 word_value = 0xffff; 

	if (!(socle_nand->buf_start & 0x01) && socle_nand->buf_start < socle_nand->buf_count) {
		word_value = *((u16 *)(socle_nand->buf_addr+socle_nand->buf_start));
		socle_nand->buf_start += 2;
	}
	NDEBUG("_read word= %x\n",word_value);
	return word_value;
}


static void socle_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct socle_nand_info *socle_nand = mtd->priv;
	int real_len = min_t(size_t, len, socle_nand->buf_count - socle_nand->buf_start);
#ifdef CONFIG_NFC_DMA
	socle_nand_dma_transfer(socle_nand,buf,real_len,socle_nand->buf_start,DMA_READ_DIR);	
#else
	memcpy(buf, socle_nand->buf_addr+socle_nand->buf_start, real_len);
#endif
	socle_nand->buf_start += real_len;
}

static void socle_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct socle_nand_info *socle_nand = mtd->priv;
	int real_len = min_t(size_t, len, socle_nand->buf_count - socle_nand->buf_start);
#ifdef CONFIG_NFC_DMA
  	socle_nand_dma_transfer(socle_nand, buf,real_len,socle_nand->buf_start,DMA_WRITE_DIR);
#else
	memcpy(socle_nand->buf_addr + socle_nand->buf_start, buf, real_len);
#endif
	socle_nand->buf_start += real_len;
 }

#if 0
static int socle_nand_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	NDEBUG("_verify_buf\n");
	return 0;
}
#endif

static void socle_nand_select_chip(struct mtd_info *mtd, int chip)
{

	struct socle_nand_info *socle_nand= mtd->priv;
	switch (chip) {
		case -1:
			/* Disable the NFC clock */
			socle_nfc_write(socle_nand,0, NF_SFR_FLPSM);
			break;
		default:
			/* Enable the NFC clock */
			socle_nfc_write(socle_nand,1, NF_SFR_FLPSM);
			socle_nfc_write(socle_nand,(socle_nfc_read(socle_nand,NF_SFR_FLCTRL) & 0xFF00FFFF) | ((u32)chip << 16), NF_SFR_FLCTRL);		  
			break;  
	}
}

static int socle_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct socle_nand_info *socle_nand = mtd->priv;
	 u32 status;
	socle_nfc_busy_check(socle_nand);
	socle_nfc_write(socle_nand,NFG_CMD_READ_STATUS, NF_SFR_FLCOMM);
	status = socle_nfc_read(socle_nand,NF_SFR_FLDATA);	
	memcpy(socle_nand->buf_addr, &status, 4);
	return status; 
}

static void socle_nand_ecc_hwctl(struct mtd_info *mtd, int mode)
{
	NDEBUG("_ecc_hwctl\n");	   	
	return;
}

static int socle_nand_ecc_calculate(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	NDEBUG("_ecc_calculate\n");	   	
	return 0;
}

static int socle_nand_ecc_correct(struct mtd_info *mtd,
		uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
{

	struct socle_nand_info *socle_nand = mtd->priv;
#ifdef MOVE_ECC_REG
#else
	u32 ecc_status;
#endif

       if (socle_nand->bypass_check_ecc) {
		NDEBUG("_ecc_correct: bypass\n");	   	
		return 0;
       }
#ifdef MOVE_ECC_REG
	if (socle_nand->retcode == ERR_ECC)
		return -1;    
#else
	ecc_status = socle_nfc_read(socle_nand,NF_SFR_FLECCSTATUS);
	if (( (u8)((ecc_status >> 16) & 0xFF)) != ((u8)(ecc_status & 0xFF))) {
		printk("_ecc_correct: use_ecc=0x%x,Status =0x%x\n",socle_nand->use_hwecc ,ecc_status);
		return -1;    
	}
	NDEBUG("_ecc_correct: Ok:use_ecc =0x%x,Status =0x%x\n",socle_nand->use_hwecc ,ecc_status);
#endif	
	return 0;

}

static int socle_nfc_readid(struct socle_nand_info *socle_nand, uint32_t *id)
{ 
	memset(socle_nand->buf_addr, 0xff, socle_nand->buf_count);
	socle_nfc_write(socle_nand,NFG_CMD_READ_ID, NF_SFR_FLCOMM);
	socle_nfc_busy_check(socle_nand);
	*id = socle_nand->reg_data = socle_nfc_read(socle_nand,NF_SFR_FLDATA);
	memcpy(socle_nand->buf_addr, (unsigned char*) &socle_nand->reg_data , 4);
	socle_nand->reg_data = socle_nfc_read(socle_nand,NF_SFR_FLDATA);		
	memcpy(socle_nand->buf_addr + 4, (unsigned char*) &socle_nand->reg_data , 4);
	return 0;  
}

static int socle_nand_config_flash(struct socle_nand_info *socle_nand,
				     struct socle_nand_flash *f)
{
	//struct platform_device *pdev = socle_nand->pdev;
	//struct socle_nand_platform_data *pdata = pdev->dev.platform_data;
	
#ifdef CONFIG_NFC_IP_PROTECT
	socle_nand->use_ip_protect = 0;
#endif
	
	u32 flctr = 0; 
	socle_nfc_write(socle_nand, flctr, NF_SFR_FLCTRL); /* clear all setting */
	
	if (f->page_size != 4096 && f->page_size != 2048)
		return 1;

	if (f->flash_width != 16 && f->flash_width != 8)
		return 1;

	/* calculate flash information */
	socle_nand->oob_size = (f->page_size == 4096) ? 128: 64;

	/* calculate addressing information */

	if (f->num_blocks * f->page_per_block > 65536)
		socle_nand->row_addr_cycles = 3;
	else
		socle_nand->row_addr_cycles = 2;

	flctr |= (f->page_per_block == 128) ? NF_CTRL_BLOCK_SIZE : 0;
	flctr |= (f->page_size == 2048) ? NF_CTRL_PAGE_SIZE : 0;
	flctr |= (f->flash_width == 16) ? NF_CTRL_IO_WIDTH : 0;
	
	flctr &= ~NF_CTRL_SP_SIZE; /* disable spare by default */
	flctr &= ~NF_CTRL_COMMON_RNB_LINE;          
	if (socle_nand->row_addr_cycles == 3)
		flctr |= NF_CTRL_ADDR_CYCLE;
	else  
		flctr &= ~ NF_CTRL_ADDR_CYCLE;

	socle_nand->reg_flctr = flctr;
	socle_nfc_write(socle_nand,flctr, NF_SFR_FLCTRL);
	NDEBUG("Afterflctr =  %x \n",flctr);
	socle_nand->flash_info = f;

	return 0;
}

static int socle_nand_detect_flash(struct socle_nand_info *socle_nand)
{
	u32 id;
	int i;
	struct socle_nand_flash *f;

 	socle_nfc_configure_timing(socle_nand,NULL,0);
 	
 	socle_nfc_readid(socle_nand, &id);

	for (i = 0; i < ARRAY_SIZE(builtin_flash_types); i++) {
		f = builtin_flash_types[i];
		if (id == f->chip_id)
			break;
	}
	if( i == ARRAY_SIZE(builtin_flash_types) )
		return -1;
	socle_nand->flash_info = f;

	if( i == ARRAY_SIZE(builtin_flash_types))
		return -ENODEV;

	socle_nand_config_flash(socle_nand, f);
	
	if (f->timing->tWBns != 255){
		socle_nfc_configure_timing(socle_nand, f->timing,1);
		NDEBUG(" Check ID Again after Timing Re-set======\n"); 
		NDEBUG("NF_SFR_FLCONF= 0x%x\n",socle_nfc_read(socle_nand, NF_SFR_FLCONF));
		socle_nfc_readid(socle_nand, &id);
		if( id != f->chip_id )
			return -1;
	}else
		NDEBUG("NF_SFR_FLCONF= 0x%x\n",socle_nfc_read(socle_nand, NF_SFR_FLCONF));
	return 0;
}

/* the maximum possible buffer size for large page with OOB data
 * is: 2048 + 64 = 2112 bytes, allocate a page here for both the
 * data buffer and the DMA descriptor
 */
#define MAX_BUFF_SIZE	PAGE_SIZE

static uint8_t scan_ff_pattern[] = { 0xff, 0xff };
// implement for st  ?? 
static struct nand_bbt_descr largepage_flashbased = {
	.options = NAND_BBT_SCAN2NDPAGE,
	.offs = 0,
	.len = 2,
	.pattern = scan_ff_pattern
};

static u8 bbt_pattern[] = {'B', 'b', 't', '0' };
static u8 mirror_pattern[] = {'1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION | BBT_AUTO_REFRESH,
	.offs =	8,
	.len = 4,
	.veroffs = 12,
	.maxblocks = 4,
	.pattern = bbt_pattern,
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION | BBT_AUTO_REFRESH,
	.offs =	8,
	.len = 4,
	.veroffs = 12,
	.maxblocks = 4,
	.pattern = mirror_pattern,
};

static struct nand_ecclayout hw_largepage_oob64 = {
	.eccbytes = 32,
	.eccpos = {
		  0,   1,  2,   3,   4,   5,   6,   7,
		16, 17, 18, 19, 20, 21,  22, 23,		   
		32, 33, 34, 35, 36, 37, 38, 39,		   
		48, 49, 50, 51, 52, 53, 54, 55 },
		.oobfree = {
					{ .offset =8,	.length = 8},
					{ .offset =24,	.length = 8},
					{ .offset =40,	.length = 8},
					{ .offset =56,	.length = 8},
		  		}
};
static struct nand_ecclayout hw_largepage_oob128 = {
	.eccbytes = 64,	
	.eccpos = {		   
		  0,   1,  2,   3,   4,   5,   6,   7,
		16, 17, 18, 19, 20, 21,  22, 23,		   
		32, 33, 34, 35, 36, 37, 38, 39,		   
		48, 49, 50, 51, 52, 53, 54, 55,		   
		64, 65, 66, 67, 68, 69 ,70, 71,		   
		80, 81, 82, 83, 84, 85, 86, 87,
              96, 97, 98, 99, 100, 101,102, 103,
              112,113,114,115,116,117,118,119 },	
		.oobfree = {
					{ .offset =8,	.length = 8},
					{ .offset =24,	.length = 8},
					{ .offset =40,	.length = 8},
					{ .offset =56,	.length = 8},
					{ .offset =72,	.length = 8},					
					{ .offset =88,	.length = 8},
					{ .offset =104,	.length = 8},					
					{ .offset =120,	.length = 8},
				}
};

static void socle_nand_init_mtd( struct socle_nand_info *socle_nand)
{
	struct socle_nand_flash *f = socle_nand->flash_info;
	struct nand_chip *this = &socle_nand->nand_chip;

	this->options = (f->flash_width == 16) ? NAND_BUSWIDTH_16: 0;

	this->waitfunc	= socle_nand_waitfunc;
	this->select_chip	= socle_nand_select_chip;
	this->dev_ready	= socle_nand_dev_ready;
	this->cmdfunc	= socle_nand_cmdfunc;
	this->read_word	= socle_nand_read_word;
	this->read_byte	= socle_nand_read_byte;
	this->read_buf	= socle_nand_read_buf;
	this->write_buf	= socle_nand_write_buf;
	//this->verify_buf	= socle_nand_verify_buf;

	socle_nand->use_hwecc = 0;
	if (ECC_TYPE == ECC_HARDWARE) { 
		socle_nand->use_hwecc = 1;
		this->ecc.mode	= NAND_ECC_HW;
		if (f->page_size == 4096) {
			NDEBUG("hw_largepage_oob128\n");
			this->ecc.layout = &hw_largepage_oob128;
		}
		else if (f->page_size == 2048) {
			NDEBUG("hw_largepage_oob64\n");
			this->ecc.layout = &hw_largepage_oob64;
		}
		else
			NDEBUG("Driver don't support this kind of page size\n");
		this->ecc.size	= f->page_size;
	}
	else if (ECC_TYPE == ECC_SOFTWARE) {
		this->ecc.mode	= NAND_ECC_SOFT;
		/*
		 * In SW ECC and Kernel Version 2.6.32,
		 * this ecc.size is 256 or 512 (reference file nand_ecc.c).
		 * But u-boot only support 256B.
		 * */
		this->ecc.size	= 256;
	}
	else if (ECC_TYPE == ECC_NONE) {
		this->ecc.mode	= NAND_ECC_NONE;
		this->ecc.size	= f->page_size;
	}

	socle_nand_ecc_ctrl(socle_nand, socle_nand->use_hwecc);

	printk("ECC_TYPE = %s\n", this->ecc.mode == 0 ? "NONE ECC" : this->ecc.mode == 1 ? "SOFTWARE ECC" : "HARDWARE ECC");
	
	this->ecc.hwctl	= socle_nand_ecc_hwctl;
	this->ecc.calculate	= socle_nand_ecc_calculate;
	this->ecc.correct	= socle_nand_ecc_correct;



#if 1//def CONFIG_MTD_SOCLE_NAND_USE_FLASH_BBT
	NDEBUG("Use NAND_USE_FLASH_BBT\n");

	this->bbt_td = &bbt_main_descr;
	this->bbt_md = &bbt_mirror_descr;

	/* set up nand options */
	this->options |= (NAND_NO_READRDY | NAND_NO_AUTOINCR |
			NAND_USE_FLASH_BBT);
	//NDEBUG(" nand :this->options = %x\n",this->options );
#endif    
	this->badblock_pattern = &largepage_flashbased;
#ifdef CONFIG_NFC_DMA
	socle_nand_dma_configure(socle_nand,NF_DMA_SET_SIZE_32);
	socle_nand_dma_configure(socle_nand,NF_DMA_SET_BURST_16_ADDR_INC);
#endif
	this->chip_delay = 0;

}

static int socle_nand_probe(struct platform_device *pdev)
{
 
	struct flash_platform_data *pdata;
	struct socle_nand_info *socle_nand;
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct resource *res = NULL;
	int ret = 0;

	NDEBUG("\n\n ======socle_nand_probe ==========\n");
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -ENODEV;
	}

	mtd = kzalloc(sizeof(struct mtd_info) + sizeof(struct socle_nand_info), GFP_KERNEL);
	if (!mtd) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	socle_nand = (struct socle_nand_info *)(&mtd[1]);
	socle_nand->pdev = pdev;

	this = &socle_nand->nand_chip;
	mtd->priv = socle_nand;
	mtd->owner = THIS_MODULE;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "register resource unusable\n");
		ret = -ENXIO;
		goto free_dev;
	}
#ifdef CONFIG_NFC_INT
	socle_nand->irq = platform_get_irq(pdev, 0);
	if (socle_nand->irq < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = -ENXIO;
		goto free_dev;
	}
#endif

	if (!request_mem_region(res->start, (res->end - res->start) + 1,
                                pdev->name)) {
		ret = -EBUSY;
		goto fail_mem_region;
	}

	socle_nand->regs = ioremap(res->start, (res->end - res->start) + 1);
	if (!socle_nand->regs) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENOMEM;
		goto release_mem;
	}

	socle_nand->buf_addr = socle_nand->regs;
	platform_set_drvdata(pdev, mtd);
	
#ifdef CONFIG_NFC_INT
	ret = request_irq(socle_nand->irq, socle_nand_irq, IRQF_DISABLED,
				pdev->name, socle_nand);
	if (ret < 0) {
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		goto reset_drvdata;
	}
#endif

	ret = socle_nand_detect_flash(socle_nand);
	if (ret) {
		dev_err(&pdev->dev, "failed to detect flash\n");
		ret = -ENODEV;
		goto unregister_irq;
	}

	socle_nand_init_mtd( socle_nand);

	if (nand_scan(mtd, 1)) {
		dev_err(&pdev->dev, "failed to scan nand\n");
		ret = -ENXIO;
		goto unregister_irq;
	}

       add_mtd_partitions(mtd, pdata->parts, pdata->nr_parts);

	return ret;

unregister_irq:
#ifdef CONFIG_NFC_INT
	free_irq(socle_nand->irq, socle_nand);
reset_drvdata:
#endif
	platform_set_drvdata(pdev, NULL);
	iounmap(socle_nand->regs);
release_mem:
	release_mem_region(res->start, (res->end - res->start) + 1);
fail_mem_region:
	
free_dev:
	kfree(mtd);
	return ret;
}

static int socle_nand_remove(struct platform_device *pdev)
{

	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct socle_nand_info *socle_nand = mtd->priv;
	struct resource *res;

	del_mtd_partitions(mtd);
	del_mtd_device(mtd);
	
#ifdef CONFIG_NFC_INT
	free_irq(socle_nand->irq, socle_nand);
#endif
	platform_set_drvdata(pdev, NULL);
	iounmap(socle_nand->regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(mtd);
	
	return 0;
}

#ifdef CONFIG_PM
static int socle_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
	struct mtd_info *mtd = (struct mtd_info *)platform_get_drvdata(pdev);
	struct socle_nand_info *socle_nand = mtd->priv;

	if (socle_nand->state != STATE_READY) {
		dev_err(&pdev->dev, "driver busy, state = %d\n", socle_nand->state);
		return -EAGAIN;
	}
#else
	return 0;
#endif
}

static int socle_nand_resume(struct platform_device *pdev)
{

	struct mtd_info *mtd = (struct mtd_info *)platform_get_drvdata(pdev);
	struct socle_nand_info *socle_nand = mtd->priv;

	return socle_nand_config_flash(socle_nand, socle_nand->flash_info);
}
#else
#define socle_nand_suspend	NULL
#define socle_nand_resume	NULL
#endif

static struct platform_driver socle_nand_driver = {
	.driver = {
		.name	= "socle-nfc",
	},
	.probe		= socle_nand_probe,
	.remove		= socle_nand_remove,
	.suspend		= socle_nand_suspend,
	.resume		= socle_nand_resume,
};

static int __init socle_nand_init(void)
{
	return platform_driver_register(&socle_nand_driver);
}
module_init(socle_nand_init);

static void __exit socle_nand_exit(void)
{
	platform_driver_unregister(&socle_nand_driver);
}
module_exit(socle_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SOCLE NAND controller driver");
