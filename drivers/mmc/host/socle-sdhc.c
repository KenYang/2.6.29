/********************************************************************************
* File Name     : linux/drivers/mmc/host/socle-sdhc.c 
* Author         : Ryan Chen
* Description   : Socle SDHC driver
* 
* Copyright (C) Socle Tech. Corp.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
*without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
    
*   Version      : x,y,a,b
*   History      : 
*      1. 2009/03/08 Ryan Chen create this file 
*    
********************************************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/pci.h>

#include <linux/clk.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/mmc.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/socle-sdhc-regs.h>
#include <mach/socle-scu.h>

//#define CONFIG_SDHC_DEBUG
#ifdef CONFIG_SDHC_DEBUG
	#define DBG(fmt, args...) printk("SDHC: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

#define DRIVER_NAME "socle_sdhc"

#define MAX_SLOT		2

static char __initdata banner[] = "SOCLE SDHC, (c) 2009 SOCLE Corp.\n";

struct socle_sdhc;
/*
 *  Low level type for this driver
 *  */
struct socle_sdhc_host {
	int			id;
	struct mmc_host		*mmc;		/* MMC structure */
	struct mmc_request	*mrq;		/* Current request */
	struct mmc_command	*cmd;		/* Current command */

	unsigned int		max_clk;	/* Max possible freq (MHz) */
	unsigned int		timeout_clk;	/* Timeout freq (KHz) */
	unsigned int		clock;		/* Current clock (MHz) */
	unsigned short		power;		/* Current voltage */

	int			offset;		/* Offset into current sg */
	u8			*adma_desc;	/* ADMA descriptor table */

	dma_addr_t		adma_addr;	/* Mapped ADMA descr. table */
	int			sg_count;	/* Mapped sg entries */	
////////////////////////////////////////////////////////////
	u8	dma_mode;

#define SDMA_MODE       0x00
#define ADMA1_MODE      0x01
#define ADMA2_MODE      0x02
#define NONEDMA_MODE    0xFF
	struct socle_sdhc *sdhc;
};


struct socle_sdhc {
	void __iomem *		ioaddr;		/* Mapped address */
	spinlock_t		lock;		/* Mutex */	
	int			irq;		/* Device IRQ */	
	int 			nr_slot;
	u8 			AutoCMD12Enable;
	struct socle_sdhc_host *slot[MAX_SLOT];
};


static inline void
socle_sdhc_write(struct socle_sdhc *sdhc, u32 val, u32 reg)
{
	//DBG("reg=0x%04x, val=0x%08x, base=0x%08x\n", reg, val, base);
	iowrite32(val, (u32) sdhc->ioaddr + reg);
}

static inline u32
socle_sdhc_read(struct socle_sdhc *sdhc, u32 reg)
{
	u32 val;
	val = ioread32((u32)sdhc->ioaddr + reg);
	//DBG("reg=0x%04x, val=0x%08x, base=0x%08x\n", reg, val, base);
	return val;
}

static void socle_sdhc_reset(struct socle_sdhc_host *host, u32 mask);
static void socle_sdhc_send_command(struct socle_sdhc_host *host, struct mmc_command *cmd);
static u8 socle_sdhc_host_en_irq(struct socle_sdhc_host *host);

static int socle_sdhc_adma_table_pre(struct socle_sdhc_host *host,
	struct mmc_data *data)
{
	int direction,len;
	u8 *desc, offset;
	dma_addr_t addr;
	struct scatterlist *sg;
	int i;
	
	/*
	 * The spec does not specify endianness of descriptor table.
	 * We currently guess that it is LE.
	 */

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;
	/*
	 * The ADMA descriptor table is mapped further down as we
	 * need to fill it with data first.
	 */

	host->sg_count = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len, direction);
	
	DBG("host->sg_count = %d\n", host->sg_count);
	
	desc = host->adma_desc;
	
	for_each_sg(data->sg, sg, host->sg_count, i) {
		DBG("i = %d\n", i);
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);
		DBG("addr = 0x%x ; len = %d\n", addr, len);
		
		/*
		 * The SDHCI specification states that ADMA
		 * addresses must be 32-bit aligned. If they
		 * aren't, then we use a bounce buffer for
		 * the (up to three) bytes that screw up the
		 * alignment.
		 */
		 
		offset = (4 - (addr & 0x3)) & 0x3;
		
		if(len > 65535)
			printk("len > 65535 ====\n");
		
		if (offset) {
			printk("offset ERROR !! \n");
		}

		desc[7] = (addr >> 24) & 0xff;
		desc[6] = (addr >> 16) & 0xff;
		desc[5] = (addr >> 8) & 0xff;
		desc[4] = (addr >> 0) & 0xff;

		BUG_ON(len > 65536);

		desc[3] = (len >> 8) & 0xff;
		desc[2] = (len >> 0) & 0xff;

		desc[1] = 0x00;
		desc[0] = 0x21; /* tran, valid */

		DBG("desc = 0x%llx\n", *(u64 *)desc);
		
		desc += 8;

		/*
		 * If this triggers then we have a calculation bug
		 * somewhere. :/
		 */
		WARN_ON((desc - host->adma_desc) > (64) * 8);

	}

	/*
	 * Add a terminating entry.
	 */
	desc[7] = 0;
	desc[6] = 0;
	desc[5] = 0;
	desc[4] = 0;

	desc[3] = 0;
	desc[2] = 0;

	desc[1] = 0x00;
	desc[0] = 0x03; /* nop, end, valid */

	DBG("host->adma_addr = 0x%x\n", host->adma_addr);
	
#if 0
	host->adma_addr = dma_map_single(mmc_dev(host->mmc),
		host->adma_desc, (64) * 8, DMA_TO_DEVICE);
	DBG("host->adma_addr = 0x%x\n", host->adma_addr);
#endif

#if 0
	if (dma_mapping_error(mmc_dev(host->mmc), host->adma_addr))
		goto unmap_entries;
	BUG_ON(host->adma_addr & 0x3);
	
	return 0;

unmap_entries:
	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
		data->sg_len, direction);
	return -EINVAL;
#endif
	return 0;

}

static void socle_sdhc_adma_table_post(struct socle_sdhc_host *host,
	struct mmc_data *data)
{
	int direction;

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;
#if 0
	dma_unmap_single(mmc_dev(host->mmc), host->adma_addr,
		(64) * 8, DMA_TO_DEVICE);
#endif

	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
		data->sg_len, direction);



}

static void socle_sdhc_finish_data(struct socle_sdhc_host *host)
{
	struct mmc_data *data;
	struct mmc_request *mrq;
	
	BUG_ON(!host->cmd->data);

	data = host->cmd->data;

	socle_sdhc_adma_table_post(host, data);	
	
	/*
	 * The specification states that the block count register must
	 * be updated, but it does not specify at what point in the
	 * data flow. That makes the register entirely useless to read
	 * back so we have to assume that nothing made it to the card
	 * in the event of an error.
	 */
	if (data->error) {
		printk("data error \n ");
		data->bytes_xfered = 0;
		socle_sdhc_reset(host, SFR11_SOFT_RESET_CMD_LINE);
		socle_sdhc_reset(host, SFR11_SOFT_RESET_DAT_LINE);
		
	} else {
		data->bytes_xfered = data->blksz * data->blocks;
	}

	mrq = host->mrq;

	host->mrq = NULL;
	host->cmd = NULL;

	mmc_request_done(host->mmc, mrq);

}

/*
 * Send a command
 */
static void socle_sdhc_send_command(struct socle_sdhc_host *host, struct mmc_command *cmd)
{

	struct mmc_data *data = cmd->data;
	u32 	command_information,ret;
	u8 	Status = Status;
	u32 	timeout_cont = 0;

	DBG("cmd->opcode = %d\n", cmd->opcode);
		
	host->cmd = cmd;
	cmd->error = 0;
	
	// check response type
	if (!(cmd->flags & MMC_RSP_PRESENT)) {
		command_information = SFR3_NO_RESPONSE;
 	} else if (cmd->flags & MMC_RSP_136) {							//R2
		command_information = SFR3_RESP_LENGTH_136 | SFR3_CRC_CHECK_EN;
	} else if (cmd->flags & MMC_RSP_BUSY) {						//R1B , R5B
		command_information = SFR3_RESP_LENGTH_48B | SFR3_CRC_CHECK_EN| SFR3_INDEX_CHECK_EN;
	} else if (cmd->flags & (MMC_RSP_CRC | MMC_RSP_OPCODE)) {		//R1,5,6,7
		command_information = SFR3_RESP_LENGTH_48 | SFR3_CRC_CHECK_EN | SFR3_INDEX_CHECK_EN;
	} else { 														//R3, R4
		command_information = SFR3_RESP_LENGTH_48;
	}
	
	// check if command/DAT line is not busy
	timeout_cont = COMMANDS_TIMEOUT;
        while((socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS9(host->id)) & 0x307) && (!((socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS9(host->id))>>20) & 0x1))) {
                timeout_cont--;
                if(timeout_cont == 0) {
                        printk("busy can't execute command !! \n ");
                        return ;
                }

        }

	if(data) {

		DBG("##data transfer command\n");
		DBG("data->blocks = %d ; data->blksz = %d\n", data->blocks, data->blksz);
		DBG("max data request size = %d\n", data->blocks * data->blksz);
		
		ret = socle_sdhc_adma_table_pre(host, data);

		if (ret) {
			WARN_ON(1);
		}else {
			DBG("host->adma_addr = 0x%x\n", host->adma_addr);
			socle_sdhc_write(host->sdhc, host->adma_addr,SOCLE_SDHC_SRS22(host->id));
			socle_sdhc_write(host->sdhc, (socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS10(host->id)) & (~SFR10_DMA_SELECT_MASK))| SFR10_DMA_SELECT_ADMA2
                                                        ,SOCLE_SDHC_SRS10(host->id));
		}

		command_information |= SFR3_DMA_ENABLE;

		// set block size and block
		
		socle_sdhc_write(host->sdhc, (data->blocks <<16) |data->blksz | SFR1_DMA_BUFF_SIZE_512KB,
					SOCLE_SDHC_SRS1(host->id));
		
		// set data preset bit
		command_information |= SFR3_DATA_PRESENT;

		if ( data->blocks > 1 ) {
			command_information |= SFR3_MULTI_BLOCK_SEL | SFR3_BLOCK_COUNT_ENABLE;
				if (data->stop) {
					command_information |= SFR3_AUTOCMD12_ENABLE;   
				}
		}

		if (data->flags & MMC_DATA_READ) {
			command_information |= SFR3_TRANS_DIRECT_READ;
		}
	}

	//write argument
	socle_sdhc_write(host->sdhc, cmd->arg, SOCLE_SDHC_SRS2(host->id));

	if(cmd->opcode == MMC_STOP_TRANSMISSION) {
		command_information |= SDIOHOST_CMD_TYPE_ABORT << 22 ;
	} else {
		command_information |= SDIOHOST_CMD_TYPE_OTHER << 22 ;
	}

	command_information |= cmd->opcode <<24 ;
	
	socle_sdhc_write(host->sdhc, command_information, SOCLE_SDHC_SRS3(host->id));
	
}

/*
 * Handle a command that has been completed
 */
static void socle_sdhc_completed_command(struct socle_sdhc_host *host)
{
	u32 tmp0,tmp1,tmp2,tmp3;
	struct mmc_request *mrq;
	
	BUG_ON(host->cmd == NULL);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			
			tmp3 = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS4(host->id));
			tmp2 = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS5(host->id));
			tmp1 = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS6(host->id));
			tmp0 = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS7(host->id));
			
			host->cmd->resp[3] = tmp3 <<8 | ((tmp0 & 0xff000000) >> 24);
			host->cmd->resp[2] = tmp2 <<8 | ((tmp3 & 0xff000000) >> 24);
			host->cmd->resp[1] = tmp1 <<8 | ((tmp2 & 0xff000000) >> 24);
			host->cmd->resp[0] = tmp0 <<8 | ((tmp1 & 0xff000000) >> 24);

		} else {
			host->cmd->resp[0] = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS4(host->id));
		}
	}

	if (!host->cmd->data) {
		
		mrq = host->mrq;
		
		/*
		 * The controller needs a reset of internal state machines
		 * upon error conditions.
		 */
		if (mrq->cmd->error) {
			
			/* Some controllers need this kick or reset won't work here */

			/* Spec says we should do both at the same time, but Ricoh
			   controllers do not like that. */
			socle_sdhc_reset(host, SFR11_SOFT_RESET_CMD_LINE);
			socle_sdhc_reset(host, SFR11_SOFT_RESET_DAT_LINE);
		}

		host->mrq = NULL;
		host->cmd = NULL;

		mmc_request_done(host->mmc, mrq);

	}
}

/*
 * Handle an MMC request
 */
static void socle_sdhc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct socle_sdhc_host *host = mmc_priv(mmc);
	host->mrq = mrq;

	if(socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS9(host->id)) & SFR9_CARD_INSERTED) {
		socle_sdhc_send_command(host, mrq->cmd);
	} else {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		
	}
}

//------------------------------------------------------------------------------------------
static void socle_sdhc_set_power(struct socle_sdhc_host *host, unsigned short power)
{
	u32 pwr;
	

//	printk("socle_sdhc_set_power \n");
	
	if (host->power == power)
		return;

	if (power == (unsigned short)-1) {
//		printk("set power = 0 \n");
		socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS10(host->id)) & ~SFR10_SD_BUS_POWER, 	SOCLE_SDHC_SRS10(host->id));
		
		goto out;
	}

	pwr = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS10(host->id)) & ~SFR10_BUS_VOLTAGE_MASK;


	switch (1 << power) {
	case MMC_VDD_165_195:
		pwr |= SFR10_SET_1_8V_BUS_VOLTAGE | SFR10_SD_BUS_POWER;
		break;
	case MMC_VDD_29_30:
	case MMC_VDD_30_31:
		pwr |= SFR10_SET_3_0V_BUS_VOLTAGE | SFR10_SD_BUS_POWER;
		break;
	case MMC_VDD_32_33:
	case MMC_VDD_33_34:
		pwr |= SFR10_SET_3_3V_BUS_VOLTAGE | SFR10_SD_BUS_POWER;
		break;
	default:
		BUG();
	}

	socle_sdhc_write(host->sdhc, pwr, SOCLE_SDHC_SRS10(host->id));
out:
	host->power = power;

}
//------------------------------------------------------------------------------------------
static u8 
socle_sdhc_set_clock(struct socle_sdhc_host *host,u32 clock )
{
	u32 i,sdclk;
	u32 Temp;
	

//	printk("set clock : %d \n", clock);

	if (clock == host->clock)
		return 0;

	// if requested frequency is more than 25MHz then return error
	if ( clock > 50000000 ) {
		printk("clock > 50000000 ERROR \n ");
		return -1;
	}

	// set SD clock off
	socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS11(host->id)) & ~SFR11_SD_CLOCK_ENABLE, SOCLE_SDHC_SRS11(host->id));

	if (clock == 50000000) {
		i= 0;
		goto set_clk;
	}
	if (clock == 0)
		goto out;

    	//read base clock frequency for SD clock in kilo herz
#ifndef CONFIG_ARCH_MDK_3D
	sdclk = socle_scu_ahb_clock_get()/2; //FIXME
#else
	sdclk = 50000000; // xDK3 fixed at 50MHz
#endif


	for ( i = 1; i < 512; i = 2 * i ){
//		printk("i = %d \n", i);
		if ( ( sdclk / i ) <= clock ){
			break; 
		}
	}

	if((i == 1) && (sdclk <= clock)){
//		printk("1111 \n");
		i = 0;
	} else {
//		printk("i = %x \n", i);
		i = i>>1;
	}

set_clk:
	// read current value of SFR11 register    
	Temp = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS11(host->id));

	// clear old frequency base settings 
	Temp &= ~SFR11_SEL_FREQ_BASE_MASK;

	// Set SDCLK Frequency Select and Internal Clock Enable 
//printk("CLK FIXME \n");
//	printk(" Set SDCLK i = %x \n",i);

	Temp |= ( i << 8 ) | SFR11_INT_CLOCK_ENABLE;

	socle_sdhc_write(host->sdhc, Temp, SOCLE_SDHC_SRS11(host->id));

	// wait for clock stable is 1
	while(!(socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS11(host->id)) &  SFR11_INT_CLOCK_STABLE));

	// set SD clock on
	socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS11(host->id)) |SFR11_SD_CLOCK_ENABLE , SOCLE_SDHC_SRS11(host->id));

out:
	host->clock = clock;

	return 0;

}

//------------------------------------------------------------------------------------------

/*
 * Set the IOS
 */
static void socle_sdhc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	unsigned long flags;	
	struct socle_sdhc_host *host = mmc_priv(mmc);
	struct socle_sdhc *sdhc = host->sdhc;

	spin_lock_irqsave(&sdhc->lock, flags);

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
	if (ios->power_mode == MMC_POWER_OFF) {
		socle_sdhc_write(host->sdhc, 0 ,SOCLE_SDHC_SRS14(host->id));
		socle_sdhc_host_en_irq(host);
		host->sdhc->AutoCMD12Enable = 1;
	}

	socle_sdhc_set_clock(host, ios->clock);

	if (ios->power_mode == MMC_POWER_OFF)
		socle_sdhc_set_power(host, -1);
	else
		socle_sdhc_set_power(host, ios->vdd);

	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS10(host->id)) | SFR10_DATA_WIDTH_4BIT, SOCLE_SDHC_SRS10(host->id));
		// dissable mmc8 mode
	    	socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_HRS0) & ~HSFR0_MMC8_MASK , SOCLE_SDHC_HRS0);
	} else if (ios->bus_width == MMC_BUS_WIDTH_1){
		socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS10(host->id)) & ~SFR10_DATA_WIDTH_4BIT, SOCLE_SDHC_SRS10(host->id));
		// dissable mmc8 mode
	    	socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_HRS0) & ~HSFR0_MMC8_MASK , SOCLE_SDHC_HRS0);
	} else if (ios->bus_width == MMC_BUS_WIDTH_8){
		// dissable mmc8 mode
	    	socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_HRS0) | HSFR0_MMC8_MASK , SOCLE_SDHC_HRS0);

	} else {

		printk("ERROR socle_sdhc_set_ios !! \n ");
	}

	spin_unlock_irqrestore(&sdhc->lock, flags);


}

static u8 socle_sdhc_check_err(struct socle_sdhc_host *host)
{
    u32 SRS12, SRS15;
    u8 Error =0;
    // number of interrupt to clear
    volatile u32 IntToClear = 0;

	if(!host->cmd)
		return 0;
	WARN_ON(!host->cmd);

	SRS12 = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS12(host->id));
	SRS15 = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS15(host->id));

	DBG("SRS12 = 0x%x\n" , SRS12);

	if(SRS12 & SFR12_AUTO_CMD12_ERROR) {
		printk("SFR12_AUTO_CMD12_ERROR \n");
		if ( SRS15 & SFR15_CMD_NOT_ISSUED_ERR )
			printk("SFR15_CMD_NOT_ISSUED_ERR \n");
		if ( SRS15 & SFR15_AUTO_CMD12_INDEX_ERR )
			printk("SFR15_AUTO_CMD12_INDEX_ERR \n");
		if ( SRS15 & SFR15_AUTO_CMD12_END_BIT_ERR )
			printk("SFR15_AUTO_CMD12_END_BIT_ERR \n");
		if ( SRS15 & SFR15_AUTO_CMD12_CRC_ERR ) {
			printk("SFR15_AUTO_CMD12_CRC_ERR \n");
//			if(SRS15 & SFR15_AUTO_CMD12_TIMEOUT_ERR)
//				printk("SFR15_AUTO_CMD12_TIMEOUT_ERR \n");
//			else
//				printk("!!! SFR15_AUTO_CMD12_TIMEOUT_ERR \n");
		}
		if ( SRS15 & SFR15_AUTO_CMD12_TIMEOUT_ERR ) 
			printk("SFR15_AUTO_CMD12_TIMEOUT_ERR \n");
		if ( SRS15 & SFR15_AUTO_CMD12_NOT_EXECUTED )
			printk("SFR15_AUTO_CMD12_NOT_EXECUTED \n");

		IntToClear |= SFR12_AUTO_CMD12_ERROR;
	}

	if(SRS12 & SFR12_CURRENT_LIMIT_ERROR ) {
		printk("SFR12_CURRENT_LIMIT_ERROR \n");
		IntToClear |= SFR12_CURRENT_LIMIT_ERROR;
	}
	if ( SRS12 & SFR12_DATA_END_BIT_ERROR ){
		printk("SFR12_DATA_END_BIT_ERROR \n");
		IntToClear |= SFR12_DATA_END_BIT_ERROR;
		Error = 1;
		host->cmd->data->error = -EILSEQ;		
	}	
	if ( SRS12 & SFR12_DATA_CRC_ERROR ){
		printk("SFR12_DATA_CRC_ERROR \n");
		IntToClear |= SFR12_DATA_CRC_ERROR;
		Error = 1;
		host->cmd->data->error = -EILSEQ;		
	}
	if ( SRS12 & SFR12_DATA_TIMEOUT_ERROR ){
		printk("SFR12_DATA_TIMEOUT_ERROR : CMD : %x \n", host->cmd->opcode);
		IntToClear |= SFR12_DATA_TIMEOUT_ERROR;
		Error = 1;
		host->cmd->data->error = -ETIMEDOUT;
	}
	if ( SRS12 & SFR12_COMMAND_INDEX_ERROR ){
		printk("SFR12_COMMAND_INDEX_ERROR \n");
		host->cmd->error = -EILSEQ;
		Error = 1;
		IntToClear |= SFR12_COMMAND_INDEX_ERROR;
	}
	if ( SRS12 & SFR12_COMMAND_END_BIT_ERROR ){
		printk("SFR12_COMMAND_END_BIT_ERROR \n");
		Error = 1;
		host->cmd->error = -EILSEQ;
		IntToClear |= SFR12_COMMAND_END_BIT_ERROR;
	}
	if ( SRS12 & SFR12_COMMAND_CRC_ERROR ){
		printk("SFR12_COMMAND_CRC_ERROR \n");
		Error =1 ;
		host->cmd->error = -EILSEQ;
		if ( SRS12 & SFR12_COMMAND_TIMEOUT_ERROR ){
			IntToClear |= SFR12_COMMAND_CRC_ERROR | SFR12_COMMAND_TIMEOUT_ERROR ;
		}
		else{
			IntToClear |= SFR12_COMMAND_CRC_ERROR;
		}
	}
	if ( SRS12 & SFR12_COMMAND_TIMEOUT_ERROR ){
		//printk("SFR12_COMMAND_TIMEOUT_ERROR \n");
		Error = 1;
		IntToClear |= SFR12_COMMAND_TIMEOUT_ERROR;
		host->cmd->error = -ETIMEDOUT;
	}

	if ( SRS12 & SFR12_ADMA_ERROR ){
		Error =1;
		printk("SFR12_ADMA_ERROR!! \n");
		IntToClear |= SFR12_ADMA_ERROR;
	}

	// clear interrupt
	socle_sdhc_write(host->sdhc, IntToClear, SOCLE_SDHC_SRS12(host->id));
	
	return Error;
}

/*
 * Handle an interrupt
 */
static irqreturn_t socle_sdhc_irq(int irq, void *devid)
{
	struct socle_sdhc *sdhc = devid;
	struct socle_sdhc_host *host;
	unsigned long flags;
	u32	status;
	u8	slot;

	DBG("-----start\n");
		
	spin_lock_irqsave(&sdhc->lock, flags);

	slot = (socle_sdhc_read(sdhc, SOCLE_SDHC_HRS63)&0xf) -1;

	if(slot >= sdhc->nr_slot) {
		slot = 0;
	}

	host = sdhc->slot[slot];
	
	status = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS12(host->id));
	DBG("status = 0x%x\n", status);

	if (status & (SFR12_DMA_INTERRUPT | SFR12_CARD_INTERRUPT | SFR12_CARD_REMOVAL | SFR12_CARD_INSERTION | SFR12_ERROR_INTERRUPT)) {
		
		if ( status & SFR12_DMA_INTERRUPT ){
			DBG("SFR12_DMA_INTERRUPT\n");
			socle_sdhc_write(host->sdhc, SFR12_DMA_INTERRUPT, SOCLE_SDHC_SRS12(host->id));
		}

		if ( status & SFR12_CARD_INTERRUPT ){      
			DBG("SFR12_CARD_INTERRUPT\n");
			// clear card interrupt interrupt
			socle_sdhc_write(host->sdhc, SFR12_CARD_INTERRUPT, SOCLE_SDHC_SRS12(host->id));
			mmc_signal_sdio_irq(host->mmc);
			spin_unlock_irqrestore(&sdhc->lock, flags);
			return IRQ_HANDLED;	
			
		}

		if (status & SFR12_CARD_REMOVAL) {
			DBG("SFR12_CARD_REMOVAL\n");
			socle_sdhc_write(host->sdhc, SFR12_CARD_REMOVAL, SOCLE_SDHC_SRS12(host->id));
			while(socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS12(host->id)) & SFR12_CARD_REMOVAL); 
			mmc_detect_change(host->mmc, 0);
		}

		if(status & SFR12_CARD_INSERTION) {
			DBG("SFR12_CARD_INSERTION\n");
			socle_sdhc_write(host->sdhc, SFR12_CARD_INSERTION,SOCLE_SDHC_SRS12(host->id));
			while(socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS12(host->id)) & SFR12_CARD_INSERTION);
			mmc_detect_change(host->mmc, 0);
		}

		// check the source of the interrupt
		if ( status & SFR12_ERROR_INTERRUPT ){
			DBG("SFR12_ERROR_INTERRUPT \n");
			socle_sdhc_write(host->sdhc, SFR12_ERROR_INTERRUPT,SOCLE_SDHC_SRS12(host->id));
			if(socle_sdhc_check_err(host)) {
				if(host->cmd->data)
					socle_sdhc_finish_data(host);
			} else {
				printk("EEEEEERRRRRRR !!\n");
				if(!host->cmd) {
					goto out;
				}
			}
		}
	}

	if(status & (SFR12_TRANSFER_COMPLETE | SFR12_COMMAND_COMPLETE)) {
		
		if (status & SFR12_COMMAND_COMPLETE ) {      
			DBG("SFR12_COMMAND_COMPLETE \n");
			// clear command complete status
			socle_sdhc_write(host->sdhc, SFR12_COMMAND_COMPLETE, SOCLE_SDHC_SRS12(host->id));
			socle_sdhc_completed_command(host);
		}
		
		if (status & SFR12_TRANSFER_COMPLETE) {
			DBG("SFR12_TRANSFER_COMPLETE\n");
			// clear transfer complete status
			socle_sdhc_write(host->sdhc, SFR12_TRANSFER_COMPLETE, SOCLE_SDHC_SRS12(host->id));

			if(!host->cmd) {
				printk("No cmd transfer ==============\n");
				goto out;
			}

			if(!host->cmd->data) {
				printk("No data transfer ==============\n");
				goto out;
			}
			
			if(host->cmd->data->error) {
				printk("EEEEEERRRRRROOOOORRR ===== \n");
			}

			socle_sdhc_finish_data(host);

			if ( status & SFR12_BLOCK_GAP_EVENT ) {
				printk("SFR12_BLOCK_GAP_EVENT ???? \n");
				// continue request
				socle_sdhc_write(host->sdhc, socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS10(host->id)) |SFR10_CONTINUE_REQUEST, SOCLE_SDHC_SRS10(host->id));
			}
		}
		
	}
	
	DBG("-----end\n");
	
out:	

	spin_unlock_irqrestore(&sdhc->lock, flags);
	return IRQ_HANDLED;	
}

static int socle_sdhc_get_ro(struct mmc_host *mmc)
{
	struct socle_sdhc_host *host;
	struct socle_sdhc *sdhc;
	u32 present;

	host = mmc_priv(mmc);
	sdhc = host->sdhc;

	present = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS9(host->id)) & SFR9_WP_SWITCH_LEVEL;

//	printk("id : %d , present : %d\n", host->id,present);

	return present;


}

//------------------------------------------------------------------------------------------
static u8 SDIOHost_SetTimeout (struct socle_sdhc_host *host, u32 TimeoutVal )
{
	if( !(TimeoutVal & SFR11_TIMEOUT_MASK) )
		return -1;

	socle_sdhc_write(host->sdhc, (socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS11(host->id)) & SFR11_TIMEOUT_MASK) | TimeoutVal, SOCLE_SDHC_SRS11(host->id));

	return 0;
}
//------------------------------------------------------------------------------------------
static u8 
socle_sdhc_host_init(struct socle_sdhc *sdhc)
{
	u32 DP;

   	//printk("SDIOHost_HostInitialize : Start host initializing...  \n" );

	// reset controller for sure
	socle_sdhc_write(sdhc, socle_sdhc_read(sdhc, SOCLE_SDHC_HRS0) | HSFR0_SOFTWARE_RESET, SOCLE_SDHC_HRS0);

	// wait for clear 
	while(socle_sdhc_read(sdhc, SOCLE_SDHC_HRS0) & HSFR0_SOFTWARE_RESET);

	//DP * Tclk = 20ms
	DP = socle_scu_apb_clock_get() * 20 /1000; 
	//printk("DP = %x \n", DP);
	socle_sdhc_write(sdhc, DEBOUNCING_TIME, SOCLE_SDHC_HRS1);

	// NumberOfSlots can't be 0
	return sdhc->nr_slot;
	

}

static u8 
socle_sdhc_host_en_irq(struct socle_sdhc_host *host)
{
// IRQ ENABLE
	socle_sdhc_write(host->sdhc, SFR13_AUTO_CMD12_ERR_STAT_EN
            | SFR13_CURRENT_LIMIT_ERR_STAT_EN   
            | SFR13_DATA_END_BIT_ERR_STAT_EN      
            | SFR13_DATA_CRC_ERR_STAT_EN          
            | SFR13_DATA_TIMEOUT_ERR_STAT_EN      
            | SFR13_COMMAND_INDEX_ERR_STAT_EN     
            | SFR13_COMMAND_END_BIT_ERR_STAT_EN   
            | SFR13_COMMAND_CRC_ERR_STAT_EN       
            | SFR13_COMMAND_TIMEOUT_ERR_STAT_EN   
            | SFR13_CARD_REMOVAL_STAT_EN          
            | SFR13_CARD_INERTION_STAT_EN         
//            | SFR13_BUFFER_READ_READY_STAT_EN     
//            | SFR13_BUFFER_WRITE_READY_STAT_EN    
            | SFR13_DMA_INTERRUPT_STAT_EN         
            | SFR13_BLOCK_GAP_EVENT_STAT_EN       
            | SFR13_TRANSFER_COMPLETE_STAT_EN     
            | SFR13_COMMAND_COMPLETE_STAT_EN
		,SOCLE_SDHC_SRS13(host->id));

	socle_sdhc_write(host->sdhc , SFR14_AUTO_CMD12_ERR_SIG_EN
            | SFR14_CURRENT_LIMIT_ERR_SIG_EN
            | SFR14_DATA_END_BIT_ERR_SIG_EN   
            | SFR14_DATA_CRC_ERR_SIG_EN       
            | SFR14_DATA_TIMEOUT_ERR_SIG_EN   
            | SFR14_COMMAND_INDEX_ERR_SIG_EN  
            | SFR14_COMMAND_END_BIT_ERR_SIG_EN
            | SFR14_COMMAND_CRC_ERR_SIG_EN    
            | SFR14_COMMAND_TIMEOUT_ERR_SIG_EN
            | SFR14_CARD_REMOVAL_SIG_EN       
            | SFR14_CARD_INERTION_SIG_EN      
 //           | SFR14_BUFFER_READ_READY_SIG_EN  
 //           | SFR14_BUFFER_WRITE_READY_SIG_EN 
            | SFR14_DMA_INTERRUPT_SIG_EN      
            | SFR14_BLOCK_GAP_EVENT_SIG_EN    
            | SFR14_TRANSFER_COMPLETE_SIG_EN  
            | SFR14_COMMAND_COMPLETE_SIG_EN
		,SOCLE_SDHC_SRS14(host->id));

	
//FIXME
//	SDIOHost_SetTimeout(host,  SFR11_TIMEOUT_TMCLK_X_2_POWER_13);
	SDIOHost_SetTimeout(host,  SFR11_TIMEOUT_TMCLK_X_2_POWER_27);

	
	return 0;

}

static void socle_sdhc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct socle_sdhc_host *host;
	struct socle_sdhc *sdhc;
	unsigned long flags;
	u32 ier,iser;

	host = mmc_priv(mmc);
	sdhc = host->sdhc;
//	printk("socle_sdhc_enable_sdio_irq \n");

	spin_lock_irqsave(&sdhc->lock, flags);

	iser = socle_sdhc_read(sdhc, SOCLE_SDHC_SRS13(host->id)); 
	ier = socle_sdhc_read(sdhc, SOCLE_SDHC_SRS14(host->id));
	if(ier != iser)
		printk("IER ERROR !!\n");

	ier &= ~SFR13_CARD_INTERRUPT_STAT_EN;
	if (enable) {
//		printk("enable SFR13_CARD_INTERRUPT_STAT_EN \n");
		ier |= SFR13_CARD_INTERRUPT_STAT_EN;
		sdhc->AutoCMD12Enable = 0;
	}

//	mdelay(100); // if panic
	socle_sdhc_write(sdhc, ier, SOCLE_SDHC_SRS13(host->id));
	socle_sdhc_write(sdhc, ier, SOCLE_SDHC_SRS14(host->id));
	
	spin_unlock_irqrestore(&sdhc->lock, flags);
	
}

static const struct mmc_host_ops socle_sdhc_ops = {
	.request	= socle_sdhc_request,
	.set_ios	= socle_sdhc_set_ios,
	.get_ro		= socle_sdhc_get_ro,
	.enable_sdio_irq = socle_sdhc_enable_sdio_irq,	
};

/*****************************************************************************\
 *                                                                           *
 * Low level functions                                                       *
 *                                                                           *
\*****************************************************************************/

static void socle_sdhc_reset(struct socle_sdhc_host *host, u32 mask)
{
	socle_sdhc_write(host->sdhc, mask | (socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS11(host->id)) & ~(SFR11_SOFT_RESET_CMD_LINE | SFR11_SOFT_RESET_DAT_LINE)), SOCLE_SDHC_SRS11(host->id));
	
	if (mask & SFR11_SOFT_RESET_ALL)
		host->clock = 0;

	/* hw clears the bit when it's done */
	while(socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS11(host->id)) & (SFR11_SOFT_RESET_DAT_LINE | SFR11_SOFT_RESET_CMD_LINE) );

}

/*****************************************************************************\
 *                                                                           *
 * Device allocation/registration                                            *
 *                                                                           *
\*****************************************************************************/

struct socle_sdhc_host *socle_sdhc_alloc_host(struct device *dev)
{
	struct mmc_host *mmc;
	struct socle_sdhc_host *host;

	WARN_ON(dev == NULL);

	mmc = mmc_alloc_host(sizeof(struct socle_sdhc_host), dev);
	if (!mmc)
		return ERR_PTR(-ENOMEM);

	host = mmc_priv(mmc);
	host->mmc = mmc;

	return host;
}

void socle_sdhc_free_host(struct socle_sdhc_host *host)
{
	mmc_free_host(host->mmc);
}

/*
 * Probe for the device
 */
static int __init socle_sdhc_probe(struct platform_device *pdev)
{
	struct socle_sdhc *sdhc =0;
	struct socle_sdhc_host *host =0;
	struct mmc_host *mmc;	
	struct resource *res;
	u32 caps;
	int ret =0;
	int i,j;
	u32 temp;

	printk(banner);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	
	if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
		return -EBUSY;

	sdhc = kzalloc(sizeof(struct socle_sdhc), GFP_KERNEL);
	
	sdhc->ioaddr = (void __iomem *) IO_ADDRESS(res->start);

	sdhc->nr_slot = 0;
	
	for (i=0; i<4; i++) {
		temp = (socle_sdhc_read(sdhc, SOCLE_SDHC_HRS0) & HSFR0_AVAILABLE_SLOT) >> 16;
		if(((temp >> i) & 0x1))
			sdhc->nr_slot++;
	}
	
	sdhc->AutoCMD12Enable = 1;
	/*
	 * Reset hardware
	 */
	if(!socle_sdhc_host_init(sdhc))
		return -ENXIO;

	for(i=0;i<sdhc->nr_slot;i++) {
		mmc = mmc_alloc_host(sizeof(struct socle_sdhc_host), &pdev->dev);
		if (!mmc)
			return -ENOMEM;

		host = mmc_priv(mmc);
		host->mmc = mmc;
		host->sdhc = sdhc;
		host->id = i;

		sdhc->slot[host->id] = host;

		if (IS_ERR(host)) {
			ret = PTR_ERR(host);
			goto fail6;
		}

		caps = socle_sdhc_read(host->sdhc, SOCLE_SDHC_SRS16(host->id));

#if defined(CONFIG_ARCH_MDK_3D) || defined(CONFIG_ARCH_PDK_PC9223)
		host->max_clk = 50000000; // xDK3 fixed at 50MHz
#else
		host->max_clk = socle_scu_ahb_clock_get(); //FIXME
#endif

		if (host->max_clk == 0) {
			printk(KERN_ERR "%s: Hardware doesn't specify base clock "
				"frequency.\n", mmc_hostname(host->mmc));
			return -ENODEV;
		}

		host->timeout_clk = SFR16_GET_TIMEOUT_CLK_FREQ(caps);

		if (host->timeout_clk == 0) {
			printk(KERN_ERR "%s: Hardware doesn't specify timeout clock "
				"frequency.\n", mmc_hostname(host->mmc));
			return -ENODEV;
		}

		if (caps & SFR16_TIMEOUT_CLOCK_UNIT_MHZ)
			host->timeout_clk *= 1000000;
		else
			host->timeout_clk *= 1000;

//		printk("host->timeout_clk: %d , host->max_clk = %d \n",host->timeout_clk , host->max_clk);
		socle_sdhc_host_en_irq(host);
		
		host->mmc->parent = &pdev->dev;
		host->mmc->ops = &socle_sdhc_ops;

		for(j=7;j>0;j--) {
//			printk("min = %d \n",host->max_clk / (1<<j));
			if((host->max_clk / (1<<j)) > 300000)
				break;
		}

		
//		printk("divider = %x \n",1<<j);
		host->mmc->f_min = host->max_clk / (1<<j);
		host->mmc->f_max = host->max_clk;

//		printk("host->mmc->f_max = %d ",host->mmc->f_max);

		host->mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_29_30|MMC_VDD_30_31 | MMC_VDD_165_195;

		if (host->mmc->ocr_avail == 0) {
			printk(KERN_ERR "Hardware doesn't report any "
				"support voltages.\n");
			ret = -ENODEV;
			goto fail5;
		}

		host->dma_mode = ADMA2_MODE; // NONEDMA_MODE;SDMA_MODE,ADMA2_MODE
#if 1
		//mmc->max_seg_size = 2048*512 ;
		mmc->max_hw_segs = 64;
		mmc->max_phys_segs = 64;
		mmc->max_seg_size = 65535;		/* see blk_queue_max_segment_size */
		mmc->max_req_size = 1024*512;	/* maximum number of bytes in one req */
		mmc->max_blk_size = 2048;
		mmc->max_blk_count = 65535;
		
#else
		mmc->max_hw_segs = 32;
		mmc->max_phys_segs = 32;
		mmc->max_seg_size = 65535;
		mmc->max_req_size = 65535;
		mmc->max_blk_size = 65535;
		mmc->max_blk_count = 65536;
#endif
		
		host->mmc->caps = MMC_CAP_4_BIT_DATA |MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ | MMC_CAP_MMC_HIGHSPEED;

		if (host->dma_mode == ADMA2_MODE) {
			/*
			 * We need to allocate descriptors for all sg entries
			 * (128) and potentially one alignment transfer for
			 * each of those entries.
			 */
			 
			host->adma_desc = dma_alloc_coherent(NULL, (64) * 8, &host->adma_addr, GFP_KERNEL);
			//host->adma_desc = kmalloc((64) * 8, GFP_KERNEL);
#if 0
			host->adma_addr = dma_map_single(mmc_dev(host->mmc), host->adma_desc, (64) * 8, DMA_TO_DEVICE);
			DBG("host->adma_addr = 0x%x\n", host->adma_addr);
#endif

			if (!host->adma_desc) {
				kfree(host->adma_desc);
				printk(KERN_WARNING "%s: Unable to allocate ADMA "
					"buffers. Falling back to standard DMA.\n",
					mmc_hostname(host->mmc));
				host->dma_mode = SDMA_MODE;
				printk("FORCE SDMA_MODE ********************\n");
			}
		}


	}

	
	spin_lock_init(&sdhc->lock);

	/*
	 * Allocate the MCI interrupt
	 */
	sdhc->irq = platform_get_irq(pdev, 0);

	if (sdhc->irq < 0) {
		dev_err(&pdev->dev, "\nSocle SD/MMC host: no irq specified\n");
//		err = -ENOENT;
		goto fail5;
	}

	//printk("sdhc->irq = %d \n",sdhc->irq);

	/* Allocate the interrupt */
	ret = request_irq(sdhc->irq, socle_sdhc_irq, IRQF_DISABLED, pdev->name, sdhc);
	if (ret) {
		dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
//		goto fail0;
	}

	platform_set_drvdata(pdev, host->mmc);

	for(i=0;i<sdhc->nr_slot;i++) {
		host = sdhc->slot[i];
		mmc_add_host(host->mmc);
	}

	return 0;

fail5:
	mmc_free_host(host->mmc);
fail6:
	release_mem_region(res->start, res->end - res->start + 1);
	dev_err(&pdev->dev, "probe failed, err %d\n", ret);

	return ret;

}

/*
 * Remove a device
 */
static int __exit socle_sdhc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct socle_sdhc_host *host;
	struct socle_sdhc *sdhc;
	unsigned long flags;
	int i;

	if (!mmc) {
		printk("socle_sdhc_remove ERROR !! \n");
		return -1;
	}

	host = mmc_priv(mmc);
	sdhc = host->sdhc;
	spin_lock_irqsave(&sdhc->lock, flags);

	if (host->mrq) {
		printk(KERN_ERR "%s: Controller removed during "
			" transfer!\n", mmc_hostname(host->mmc));

		host->mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, host->mrq);
	}

	spin_unlock_irqrestore(&sdhc->lock, flags);

	for(i=0;i<sdhc->nr_slot;i++) {
		host = sdhc->slot[i];
		mmc_remove_host(host->mmc);
	}

	free_irq(sdhc->irq, host);

	kfree(host->adma_desc);

	host->adma_desc = NULL;

	for(i=0;i<sdhc->nr_slot;i++) {
		host = sdhc->slot[i];
		socle_sdhc_free_host(host);
		kfree(host);
	}

	kfree(sdhc);
	return 0;
}

#ifdef CONFIG_PM
//static int socle_sdhc_suspend(struct platform_device *pdev, pm_message_t state)
static int socle_sdhc_suspend(struct device *dev)
{
	u32 i;
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct socle_sdhc_host *host = mmc_priv(mmc);
	struct socle_sdhc *sdhc = host->sdhc;
	int ret = 0;
//printk("##--%s--start\n", __func__);
	//if (device_may_wakeup(&pdev->dev))
	//	enable_irq_wake(host->board->det_pin);

	for(i=0; i<sdhc->nr_slot; i++) {
		host = sdhc->slot[i];
		if (host->mmc)
			//ret = mmc_suspend_host(host->mmc, state);
			ret = mmc_suspend_host(host->mmc, PMSG_SUSPEND);
	}

	return ret;

}

//static int socle_sdhc_resume(struct platform_device *pdev)
//static void socle_sdhc_resume(struct device *dev)
static int socle_sdhc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct socle_sdhc_host *host = mmc_priv(mmc);
	struct socle_sdhc *sdhc = host->sdhc;
	int ret = 0;
	u32 i;
	
//printk("##--%s--start\n", __func__);
	//if (device_may_wakeup(&pdev->dev))
	//	disable_irq_wake(host->board->det_pin);

	if(!socle_sdhc_host_init(sdhc))
		return -ENXIO;

	for(i=0; i<sdhc->nr_slot; i++) {
		host->id = i;
		socle_sdhc_host_en_irq(host);
	}

	usermodehelper_enable();

	for(i=0; i<sdhc->nr_slot; i++) {
		host = sdhc->slot[i];
		if (host->mmc)
			ret = mmc_resume_host(host->mmc);
	}
	
	return ret;
;
}
#else
#define socle_sdhc_suspend	NULL
#define socle_sdhc_resume		NULL
#endif

#if 0
static const struct dev_pm_ops socle_sdhc_pm_ops = {
	.suspend = socle_sdhc_suspend,
	.resume  = socle_sdhc_resume,
	//.complete  = socle_sdhc_resume,
};
#endif	

static struct platform_driver socle_sdhc_driver = {
	.remove		= __exit_p(socle_sdhc_remove),
	.suspend	= socle_sdhc_suspend,
	.resume		= socle_sdhc_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		//.pm		= &socle_sdhc_pm_ops,
	},
};

static int __init socle_sdhc_drv_init(void)
{
	return platform_driver_probe(&socle_sdhc_driver, socle_sdhc_probe);
}

static void __exit socle_sdhc_drv_exit(void)
{
	platform_driver_unregister(&socle_sdhc_driver);
}

module_init(socle_sdhc_drv_init);
module_exit(socle_sdhc_drv_exit);

MODULE_DESCRIPTION("Socle SD Host driver");
MODULE_AUTHOR("Ryan Chen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:socle_sdhc");
