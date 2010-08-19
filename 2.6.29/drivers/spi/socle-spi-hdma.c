#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/spi-regs.h>
#include <mach/socle-scu.h>
#include <mach/dma.h>
#include <linux/dma-mapping.h>
#include <mach/pl080_dma.h>

//#define CONFIG_SPI_LOOPBACK

//#define CONFIG_SPI_DEBUG
#ifdef CONFIG_SPI_DEBUG
	#define DBG(fmt, args...) printk("SPI : %s(): " fmt, __FUNCTION__, ## args)
#else
	#define DBG(fmt, args...)
#endif

// cyli fix for different tx/rx
#define SET_TX_RX_LEN(tx, rx)	(((tx) << 16) | (rx))
#define GET_TX_LEN(len)		((len) >> 16)
#define GET_RX_LEN(len)		((len) & 0xffff)
#define INVALID_DMA_ADDRESS	0xffffffff

#define SUSPND    (1<<0)
#define SPIBUSY   (1<<1)

struct socle_spi_host {
	/* bitbang has to be first */
	struct spi_bitbang bitbang;
	u32 base;		//leonid+ 20090420 for multi-spi implict
	wait_queue_head_t wq;
	struct resource *io_area;
	int irq;
	u32 len;
	u8 bpw;
	u32 tx_xfer_cnt;
	u32 rx_xfer_cnt;
	u32 tx_total_cnt;
	u32 rx_total_cnt;
	u32 xfer_stat;
	u8 master_setup;
	u8 state;
#define TX_XFER_DONE 0x1
#define RX_XFER_DONE 0x2
	struct spi_master *master;
	struct spi_device *spi_dev;
	struct device *dev;
	const void *tx_buf;
	void *rx_buf;
	dma_addr_t	tx_dma;
	dma_addr_t	rx_dma;
	u32 dma_tx_count;
	u32 dma_rx_count;
	u8 dma_tx_avail;
	u8 dma_rx_avail;
	u32 dma_tx_buffer_size;
	u32 dma_rx_buffer_size;
	u32 spi_fifo_addr;
	void (*get_rx)(struct socle_spi_host *host);
	void (*set_tx)(struct socle_spi_host *host);
	void (*dma_get_last_rx)(struct socle_spi_host *host);
	void (*dma_set_last_tx)(struct socle_spi_host *host);
	struct socle_dma_notifier socle_spi_tx_dma_notifier;
	struct socle_dma_notifier socle_spi_rx_dma_notifier;
	struct socle_spi_dma_info *spi_dma_info;
};

struct socle_dma_client {
	char *name;
};

struct socle_spi_dma_info {
	u8 tx_dma_ch;
	u8 rx_dma_ch;
	u8 tx_dma_hdreq;
	u8 rx_dma_hdreq;
	u8 fifo_depth;
	u8 burst_type;
	struct pl080_dma_client *rx_client;
	struct pl080_dma_client *tx_client;
};

#ifdef CONFIG_SPI_SOCLE_PL080_HDMA

static struct pl080_dma_client socle_dma_client_spi_0_rx = {
	.name = "SPI 0 RX ADMA"
};

static struct pl080_dma_client socle_dma_client_spi_0_tx = {
	.name = "SPI 0 TX ADMA"
};

static struct pl080_dma_client socle_dma_client_spi_1_rx = {
	.name = "SPI 1 RX ADMA"
};

static struct pl080_dma_client socle_dma_client_spi_1_tx = {
	.name = "SPI 1 TX ADMA"
};
#endif

static struct socle_spi_dma_info spi_dma_info_0 = {
#ifdef CONFIG_SPI_SOCLE_PL080_HDMA
	.tx_dma_ch = 5, 
	.rx_dma_ch = 4,
	.burst_type = 4,
	.rx_client		= &socle_dma_client_spi_0_rx,
	.rx_client		= &socle_dma_client_spi_0_tx,
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
        .tx_dma_ch = 1,
        .rx_dma_ch = 0,
        .tx_dma_hdreq = 5,
        .rx_dma_hdreq = 4,
        .fifo_depth = 8,
        .burst_type = SOCLE_DMA_BURST_INCR4,
#endif
};

static struct socle_spi_dma_info spi_dma_info_1 = {
#ifdef CONFIG_SPI_SOCLE_PL080_HDMA
	.tx_dma_ch = 7, 
	.rx_dma_ch = 6,
	.burst_type = 4,
	.rx_client		= &socle_dma_client_spi_1_rx,
	.rx_client		= &socle_dma_client_spi_1_tx,
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
        .tx_dma_ch = 1,
        .rx_dma_ch = 0,
        .tx_dma_hdreq = 5,
        .rx_dma_hdreq = 4,
        .fifo_depth = 8,
        .burst_type = SOCLE_DMA_BURST_INCR4,
#endif
};

/*
 *  Read register
 *  */
static u32 inline
socle_spi_read(u32 reg, u32 base)		//leonid fix 20080420 for dynamic spi base-register read
{
	return ioread32(base+reg);
}

/*
 *  Write register
 *  */
static void inline
socle_spi_write(u32 reg, u32 value, u32 base)		//leonid fix 20080420 for dynamic spi base-register read
{
	//DBG("reg=0x%04x, val=0x%08x, base=0x%08x\n", reg, value, base);
	iowrite32(value, base+reg);
}


static void
socle_spi_get_rx_8(struct socle_spi_host *host)
{
	dev_dbg(host->dev, "socle_spi_get_rx_8()\n");
	*((u8 *)host->rx_buf + host->rx_xfer_cnt++) = socle_spi_read(SOCLE_SPI_RXR, host->base);
}

static void
socle_spi_set_tx_8(struct socle_spi_host *host)
{
	dev_dbg(host->dev, "socle_spi_set_tx_8()\n");
	socle_spi_write(SOCLE_SPI_TXR, *((u8 *)host->tx_buf + host->tx_xfer_cnt++), host->base);
}

static void
socle_spi_get_rx_16(struct socle_spi_host *host)
{	
	u16 *rx_temp;
	rx_temp = (u16 *)host->rx_buf + host->rx_xfer_cnt++;
	dev_dbg(host->dev, "socle_spi_set_rx_16()\n");
	*rx_temp = socle_spi_read(SOCLE_SPI_RXR, host->base);
}

static void
socle_spi_set_tx_16(struct socle_spi_host *host)
{
	u16 *tx_temp;
	tx_temp = (u16 *)host->tx_buf + host->tx_xfer_cnt++;
	dev_dbg(host->dev, "socle_spi_get_tx_16()\n");
	socle_spi_write(SOCLE_SPI_TXR, *tx_temp, host->base);
}


static void
socle_spi_dma_get_last_rx_8(struct socle_spi_host *host)
{
	u8 *rx_buf_temp;
	u8 rx_last_count=0;;

	rx_buf_temp = (u8 *)host->rx_buf;
	rx_buf_temp += host->dma_rx_count;

	if (host->dma_rx_avail !=0) {
		while(1) {
			while (SOCLE_SPI_RXFIFO_DATA_AVAIL == (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_RXFIFO_DATA_AVAIL)) {
				*rx_buf_temp = socle_spi_read(SOCLE_SPI_RXR, host->base);
//printk("8 : Last RX FIFO read : 0x%x = 0x%x\n", (u16)rx_buf_temp, *rx_buf_temp);
				rx_buf_temp++;
				rx_last_count++;
				if(rx_last_count == host->dma_rx_avail)
					return;
			}
		}
	}
}

static void
socle_spi_dma_set_last_tx_8(struct socle_spi_host *host)
{
	u8 *tx_buf_temp;
	u8 tx_last_count = 0;

	tx_buf_temp = (u8 *)host->tx_buf;
	tx_buf_temp += host->dma_tx_count;

	if(host->dma_tx_avail != 0) {
		while(1) {
			while (SOCLE_SPI_TXFIFO_FULL != (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_TXFIFO_FULL)) {
//printk("8 : Last TX FIFO write : 0x%x = 0x%x\n", (u16)tx_buf_temp, *tx_buf_temp);
				socle_spi_write(SOCLE_SPI_TXR, *tx_buf_temp, host->base);
				tx_buf_temp++;
				tx_last_count++;
				if(tx_last_count == host->dma_tx_avail)
					return;
			}
		}
	}
}

static void
socle_spi_dma_get_last_rx_16(struct socle_spi_host *host)
{
	u16 *rx_buf_temp;
	u8 rx_last_count=0;;

	rx_buf_temp = (u16 *)host->rx_buf;
	rx_buf_temp += host->dma_rx_count;

	if (host->dma_rx_avail !=0) {
		while(1) {
			while (SOCLE_SPI_RXFIFO_DATA_AVAIL == (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_RXFIFO_DATA_AVAIL)) {
				*rx_buf_temp = socle_spi_read(SOCLE_SPI_RXR, host->base);
//printk("16 : Last RX FIFO read : 0x%x = 0x%x\n", (u16)rx_buf_temp, *rx_buf_temp);
				rx_buf_temp++;
				rx_last_count++;
				if(rx_last_count == host->dma_rx_avail)
					return;
			}
		}
	}
}

static void
socle_spi_dma_set_last_tx_16(struct socle_spi_host *host)
{
	u16 *tx_buf_temp;
	u8 tx_last_count = 0;

	tx_buf_temp = (u16 *)host->tx_buf;
	tx_buf_temp += host->dma_tx_count;

	if(host->dma_tx_avail != 0) {
		while(1) {
			while (SOCLE_SPI_TXFIFO_FULL != (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_TXFIFO_FULL)) {
//printk("16 : Last TX FIFO write : 0x%x = 0x%x\n", (u16)tx_buf_temp, *tx_buf_temp);
				socle_spi_write(SOCLE_SPI_TXR, *tx_buf_temp, host->base);
				tx_buf_temp++;
				tx_last_count++;
				if(tx_last_count == host->dma_tx_avail)
					return;
			}
		}
	}
}


#ifdef CONFIG_SPI_SOCLE_PL080_HDMA

static void 
socle_spi_tx_dma_buffer_done(struct pl080_dma_chan *channel,
				void *dev_id, int size,
				enum pl080_dma_buffresult result)
{
	struct socle_spi_host *host = dev_id;
	u32 ch = host->spi_dma_info->tx_dma_ch;
	
	host->xfer_stat |= TX_XFER_DONE;
	
	pl080_dma_ctrl(ch, PL080_DMAOP_STOP);

	if (host->tx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(host->master->dev.parent, host->tx_dma,
				 host->dma_tx_buffer_size, DMA_TO_DEVICE);

	host->dma_set_last_tx(host);

	wake_up_interruptible(&host->wq);
	return;
}

static void 
socle_spi_rx_dma_buffer_done(struct pl080_dma_chan *channel,
				void *dev_id, int size,
				enum pl080_dma_buffresult result)
{
	struct socle_spi_host *host = dev_id;
	u32 ch = host->spi_dma_info->rx_dma_ch;
	
	host->xfer_stat |= RX_XFER_DONE;

	pl080_dma_ctrl(ch, PL080_DMAOP_STOP);

	if (host->rx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(host->master->dev.parent, host->rx_dma,
				 host->dma_rx_buffer_size, DMA_FROM_DEVICE);

	host->dma_get_last_rx(host);

	wake_up_interruptible(&host->wq);
	return;
}

#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA

static void
socle_spi_tx_dma_notifier_complete(void *data)
{
	struct socle_spi_host *host = data;
	u32 flags, ch = host->spi_dma_info->tx_dma_ch;
	
	host->xfer_stat |= TX_XFER_DONE;

	/* Disable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_disable_dma(ch);
	socle_release_dma_lock(flags);

	if (host->tx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(host->master->dev.parent, host->tx_dma,
				 host->dma_tx_buffer_size, DMA_TO_DEVICE);

	// transfer the last tx fifo data
	host->dma_set_last_tx(host);
	
//printk("socle_spi_tx_dma_notifier_complete - wake_up_interruptible\n");
	wake_up_interruptible(&host->wq);
	return;
}

static void
socle_spi_rx_dma_notifier_complete(void *data)
{
	struct socle_spi_host *host = data;
	u32 flags, ch = host->spi_dma_info->rx_dma_ch;
	
	host->xfer_stat |= RX_XFER_DONE;

	/* Disable the hardware dma */
	flags = socle_claim_dma_lock();
	socle_disable_dma(ch);
	socle_release_dma_lock(flags);

	if (host->rx_dma != INVALID_DMA_ADDRESS)
		dma_unmap_single(host->master->dev.parent, host->rx_dma,
				 host->dma_rx_buffer_size, DMA_FROM_DEVICE);

	// transfer the last rx fifo data
	host->dma_get_last_rx(host);

//printk("socle_spi_rx_dma_notifier_complete - wake_up_interruptible\n");
	wake_up_interruptible(&host->wq);
	return;
}

#endif

static u32
socle_spi_power(u32 base, u32 exp)
{
	u32 i;
	u32 value=1;
	
	if (exp == 0)
		return 1;
	for (i = 0; i < exp; i++)
		value *= base;
	return value;
}

static u32 
socle_spi_calculate_divisor(struct spi_device *spi, u32 clk)
{
	struct socle_spi_host *host = (struct socle_spi_host *)spi_master_get_devdata(spi->master);
	u8 div_high_3 = 0, div_low_3 = 0, spi_cdvr = 0;
	u32 sclk_divisor, sclk, pclk, power;

	dev_dbg(host->dev, "socle_spi_calculate_divisor()\n");

	/*
	 *  SCLK Divisor = (SPICDVR[5:3] + 1) * 2 power(SPICDVR[2:0] + 1)
	 *  SCLK = PCLK / SCLK Divisor
	 *  */
	pclk = socle_scu_apb_clock_get();
	while(1){
		power = socle_spi_power(2, div_low_3+1);
    for (div_high_3 = 0; div_high_3 < 8; div_high_3++) {
    	sclk_divisor = (div_high_3 + 1) * power;
      sclk = pclk / sclk_divisor;
      if (sclk < clk)
      	goto out;
    }
		div_low_3++;
  }

out:
	spi_cdvr = (div_high_3 << 3) | div_low_3;
	dev_dbg(&spi->dev, "sclk is %d, divisor is 0x%08x\n", sclk, spi_cdvr);
		return spi_cdvr;
}

volatile int act_flag=0;		//leonid+ 20090420 for chipselect lock
static void
socle_spi_chipselect(struct spi_device *spi, int value)
{
	struct socle_spi_host *host = (struct socle_spi_host *)spi_master_get_devdata(spi->master);

	dev_dbg(host->dev, "socle_spi_chipselect()\n");
	
	while(act_flag);		//leonid+ 20090420 for chipselect lock
	act_flag=1;

	if (BITBANG_CS_ACTIVE == value) {
		dev_dbg(&spi->dev, "BITBANG_CS_ACTIVE\n");
		act_flag=1;
	} else {		/* BITBANG_CS_INACTIVE */
		dev_dbg(&spi->dev, "BITBANG_CS_INACTIVE\n");
		act_flag=0;
	}
}

static int
socle_spi_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct socle_spi_host *host = (struct socle_spi_host *)spi_master_get_devdata(spi->master);
	u32 hz, cpol, cpha, lsb_first, char_len;
	u8 divisor;

	dev_dbg(host->dev, "socle_spi_setup_transfer()\n");

	host->spi_dev = spi;
	host->master_setup = 1;

	host->bpw = t ? t->bits_per_word : spi->bits_per_word;
	if(!host->bpw)		//jsho+ 20100506
		host->bpw=spi->bits_per_word;
	hz = t ? t->speed_hz : spi->max_speed_hz;
	if(!hz)		//jsho+ 20100506
		hz=spi->max_speed_hz;

//printk("@@--hz = %d\n", hz);


	if (spi->chip_select > (spi->master->num_chipselect - 1)) {
		dev_err(&spi->dev, "chipselect %d exceed the number of chipselect master supoort\n", spi->chip_select);
		return -EINVAL;
	}

	switch (host->bpw) {
	case 4:
		char_len = SOCLE_SPI_CHAR_LEN_4;
		break;
	case 5:
		char_len = SOCLE_SPI_CHAR_LEN_5;
		break;
	case 6:
		char_len = SOCLE_SPI_CHAR_LEN_6;
		break;
	case 7:
		char_len = SOCLE_SPI_CHAR_LEN_7;
		break;
	case 0:
	case 8:
		char_len = SOCLE_SPI_CHAR_LEN_8;
		break;
	case 9:
		char_len = SOCLE_SPI_CHAR_LEN_9;
		break;
	case 10:
		char_len = SOCLE_SPI_CHAR_LEN_10;
		break;
	case 11:
		char_len = SOCLE_SPI_CHAR_LEN_11;
		break;
	case 12:
		char_len = SOCLE_SPI_CHAR_LEN_12;
		break;
	case 13:
		char_len = SOCLE_SPI_CHAR_LEN_13;
		break;
	case 14:
		char_len = SOCLE_SPI_CHAR_LEN_14;
		break;
	case 15:
		char_len = SOCLE_SPI_CHAR_LEN_15;
		break;
	case 16:
		char_len = SOCLE_SPI_CHAR_LEN_16;
		break;
	default:
		dev_err(&spi->dev, "Un-support bits per word: %d\n", host->bpw);
		host->bpw = 8;
		return -EINVAL;
	}
	if (SPI_CPHA & spi->mode)
		cpha = SOCLE_SPI_CPHA_1;
	else
		cpha = SOCLE_SPI_CPHA_0;
	if (SPI_CPOL & spi->mode)
		cpol = SOCLE_SPI_CPOL_1;
	else
		cpol = SOCLE_SPI_CPOL_0;

	if (SPI_LSB_FIRST & spi->mode)
		lsb_first = SOCLE_SPI_TX_LSB_FIRST;
	else
		lsb_first = SOCLE_SPI_TX_MSB_FIRST;

	/* Configure SPI controller */
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_MDK_3D) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_spi_write(SOCLE_SPI_FWCR,
			SOCLE_SPI_MASTER_SIGNAL_CTL_HW |
			SOCLE_SPI_MASTER_SIGNAL_ACT_NO |
			SOCLE_SPI_MODE_MASTER |
			SOCLE_SPI_MASTER_SOFT_N_RST |
			SOCLE_SPI_MASTER_EN |
			SOCLE_SPI_TXRX_N_RUN |
			SOCLE_SPI_CLK_IDLE_AST |
			SOCLE_SPI_TXRX_SIMULT_DIS |
			cpol |
			cpha |
			lsb_first |
			SOCLE_SPI_MODE_UNI_DIR |
			SOCLE_SPI_DMA_REQ |	
#ifdef CONFIG_SPI_LOOPBACK
			SOCLE_SPI_OP_LOOPBACK
#else
			SOCLE_SPI_OP_NORMAL
#endif
			, host->base);

#else
	socle_spi_write(SOCLE_SPI_FWCR,
			SOCLE_SPI_MASTER_SOFT_N_RST |
			SOCLE_SPI_MASTER_EN |
			SOCLE_SPI_TXRX_N_RUN |
			SOCLE_SPI_CLK_IDLE_AST |
			SOCLE_SPI_TXRX_SIMULT_DIS |
			cpol |
			cpha |
			lsb_first |
			SOCLE_SPI_MODE_UNI_DIR |
			SOCLE_SPI_DMA_REQ |
#ifdef CONFIG_SPI_LOOPBACK
			SOCLE_SPI_OP_LOOPBACK
#else
			SOCLE_SPI_OP_NORMAL
#endif
			, host->base);
#endif

	/* Configure FIFO and clear Tx & RX FIFO */
	socle_spi_write(SOCLE_SPI_FCR,
		SOCLE_SPI_RXFIFO_INT_TRIGGER_LEVEL_4 |
		SOCLE_SPI_TXFIFO_INT_TRIGGER_LEVEL_4 |
		SOCLE_SPI_RXFIFO_CLR |
		SOCLE_SPI_TXFIFO_CLR
     		, host->base);

	/* Set the SPI slaves select and characteristic control register */
	divisor = socle_spi_calculate_divisor(spi, hz);

	socle_spi_write(SOCLE_SPI_SSCR,
			char_len |
			SOCLE_SPI_SLAVE_SEL(spi->chip_select) |
			SOCLE_SPI_CLK_DIV(divisor), host->base);


	/* Config SPI clock delay */
	socle_spi_write(SOCLE_SPI_DLYCR,
#if defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_SCDK)
                        SOCLE_SPI_PBTXRX_DELAY_NONE |           //20100629 jerry fix
#else
			SOCLE_SPI_PBTXRX_DELAY_32 |		//20100629 jerry fix 
#endif
			SOCLE_SPI_PBCT_DELAY_NONE |
			SOCLE_SPI_PBCA_DELAY_16			//to avoid SPI with HDMA transfer data wrong bug ; jerry hsieh 2010.05.27
			, host->base);

	return 0;
}

static int
socle_spi_setup(struct spi_device *spi)
{
	struct socle_spi_host *host = (struct socle_spi_host *)spi_master_get_devdata(spi->master);
	int err = 0;

	dev_dbg(host->dev, "socle_spi_setup()\n");
	
	err = socle_spi_setup_transfer(spi, NULL);
	if (err)
		dev_err(&spi->dev, "setup_transfer returned %d\n", err);
	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __FUNCTION__, spi->mode, spi->bits_per_word, spi->max_speed_hz);
	return err;
}

static int
socle_spi_txrx_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct socle_spi_host *host = (struct socle_spi_host *)spi_master_get_devdata(spi->master);
	struct device *dev = host->dev;
	int ch_tx, ch_rx;
	u8 xfer_done = 0;
	u32 tmp;
	u32 ret;
	u32 flags;
	u32 dma_data_width;
	
	dev_dbg(host->dev, "socle_spi_txrx_bufs()\n");
	
	host->state |= SPIBUSY;
	
//printk("TX_count =  %d ; RX_count = %d\n", GET_TX_LEN(t->len), GET_RX_LEN(t->len));

	host->tx_buf = t->tx_buf;
	host->rx_buf = t->rx_buf;
	host->len = GET_TX_LEN(t->len);
	host->tx_xfer_cnt = 0;
	host->rx_xfer_cnt = 0;
	host->xfer_stat = 0;
	host->tx_total_cnt = GET_TX_LEN(t->len);
	host->rx_total_cnt = GET_RX_LEN(t->len);
	
	if (host->bpw > 8) {
		host->get_rx = socle_spi_get_rx_16;
		host->set_tx = socle_spi_set_tx_16;
		host->dma_get_last_rx = socle_spi_dma_get_last_rx_16;
		host->dma_set_last_tx = socle_spi_dma_set_last_tx_16;
	} else {
		host->get_rx = socle_spi_get_rx_8;
		host->set_tx = socle_spi_set_tx_8;
		host->dma_get_last_rx = socle_spi_dma_get_last_rx_8;
		host->dma_set_last_tx = socle_spi_dma_set_last_tx_8;
	}


	/* Set transfer & receive data count */
	if (host->tx_buf) {
		socle_spi_write(SOCLE_SPI_TXCR, host->tx_total_cnt, host->base);
		xfer_done |= TX_XFER_DONE;
	} else
		socle_spi_write(SOCLE_SPI_TXCR, 0, host->base);
	if (host->rx_buf) {
		socle_spi_write(SOCLE_SPI_RXCR, host->rx_total_cnt, host->base);
		xfer_done |= RX_XFER_DONE;
	} else
		socle_spi_write(SOCLE_SPI_RXCR, 0, host->base);


	if (host->tx_buf) {
		if (host->tx_total_cnt > 8) {
		//if (0) {
//printk("TX transfetr with HDMA\n");

			host->dma_tx_avail = host->tx_total_cnt %4;
			
			if (host->dma_tx_avail != 0)
				host->dma_tx_count = host->tx_total_cnt - host->dma_tx_avail;
			else
				host->dma_tx_count = host->tx_total_cnt;
			
			DBG("host->dma_tx_avail = %d\n", host->dma_tx_avail);
			DBG("host->dma_tx_count = %d\n", host->dma_tx_count);

			if (host->bpw > 8) {
#ifdef CONFIG_SPI_SOCLE_PL080_HDMA
				dma_data_width = 4;
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
				dma_data_width = SOCLE_DMA_DATA_HALFWORD;
#endif
				host->dma_tx_buffer_size = host->dma_tx_count << 1;
			} else {
#ifdef CONFIG_SPI_SOCLE_PL080_HDMA
				dma_data_width = 2;
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
				dma_data_width = SOCLE_DMA_DATA_BYTE;
#endif
				host->dma_tx_buffer_size = host->dma_tx_count;
			}

			host->tx_dma = INVALID_DMA_ADDRESS;
			if (host->tx_buf) {
				host->tx_dma = dma_map_single(dev,
						(void *) host->tx_buf, host->dma_tx_buffer_size,
						DMA_TO_DEVICE);
				if (dma_mapping_error(dev, host->tx_dma))
					return -ENOMEM;
			}

			ch_tx = host->spi_dma_info->tx_dma_ch;

#ifdef CONFIG_SPI_SOCLE_PL080_HDMA
			flags = 0;
			pl080_dma_devconfig(ch_tx, MEM_TO_PERIPHERAL, host->spi_fifo_addr);
			pl080_dma_config(ch_tx, dma_data_width, host->spi_dma_info->burst_type);
			pl080_dma_ctrl(ch_tx, PL080_DMAOP_FLUSH);
			pl080_dma_enqueue(ch_tx, host, host->tx_dma, host->dma_tx_buffer_size);
			pl080_dma_ctrl(ch_tx, PL080_DMAOP_START);
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
			flags = socle_claim_dma_lock();
			socle_disable_dma(ch_tx);
			socle_set_dma_mode(ch_tx, SOCLE_DMA_MODE_SLICE);
			socle_set_dma_ext_hdreq_number(ch_tx, host->spi_dma_info->tx_dma_hdreq);
			socle_set_dma_burst_type(ch_tx, host->spi_dma_info->burst_type);
			socle_set_dma_source_address(ch_tx, host->tx_dma);
			socle_set_dma_destination_address(ch_tx, host->spi_fifo_addr);
			socle_set_dma_source_direction(ch_tx, SOCLE_DMA_DIR_INCR);
			socle_set_dma_destination_direction(ch_tx, SOCLE_DMA_DIR_FIXED);
			socle_set_dma_data_size(ch_tx, dma_data_width);
			socle_set_dma_transfer_count(ch_tx, host->dma_tx_buffer_size);
			socle_set_dma_slice_count(ch_tx, host->spi_dma_info->fifo_depth>>1);
			socle_set_dma_page_number(ch_tx, 1);
			socle_set_dma_buffer_size(ch_tx, host->dma_tx_buffer_size);
			socle_enable_dma(ch_tx);
			socle_release_dma_lock(flags);
#endif			
		} else {
//printk("TX transfetr with interrupt\n");
			
			while (SOCLE_SPI_TXFIFO_FULL != (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_TXFIFO_FULL)) {
				host->set_tx(host);
				if (host->tx_xfer_cnt == host->len) {
					host->xfer_stat |= TX_XFER_DONE;
					break;
				}
	
			}
			
			if(host->xfer_stat != TX_XFER_DONE)
				socle_spi_write(SOCLE_SPI_IER, socle_spi_read(SOCLE_SPI_IER, host->base) | SOCLE_SPI_IER_TXFIFO_INT_EN, host->base);
	
		}
	}

	if (host->rx_buf) {
		if(host->rx_total_cnt > 3) {
		//if(0) {
			
//printk("RX transfetr with HDMA\n");

			host->dma_rx_avail = host->rx_total_cnt %4;
				
			if (host->dma_rx_avail != 0)
				host->dma_rx_count = host->rx_total_cnt - host->dma_rx_avail;
			else
				host->dma_rx_count = host->rx_total_cnt;
				
			DBG("host->dma_rx_avail = %d\n", host->dma_rx_avail);
			DBG("host->dma_rx_count = %d\n", host->dma_rx_count);

			if (host->bpw > 8) {
				dma_data_width = SOCLE_DMA_DATA_HALFWORD;
				host->dma_rx_buffer_size = host->dma_rx_count << 1;
			} else {
				dma_data_width = SOCLE_DMA_DATA_BYTE;
				host->dma_rx_buffer_size = host->dma_rx_count;
			}

			host->rx_dma = INVALID_DMA_ADDRESS;
			host->rx_dma = dma_map_single(dev,
					host->rx_buf, host->dma_rx_buffer_size,
					DMA_FROM_DEVICE);
			if (dma_mapping_error(dev, host->rx_dma)) {
				if (host->tx_buf)
					dma_unmap_single(dev,
							host->tx_dma, host->dma_tx_buffer_size,
							DMA_TO_DEVICE);
				return -ENOMEM;
			}

			ch_rx = host->spi_dma_info->rx_dma_ch;
			
#ifdef CONFIG_SPI_SOCLE_PL080_HDMA
			flags = 0;
			pl080_dma_devconfig(ch_rx, PERIPHERAL_TO_MEM, host->spi_fifo_addr);
			pl080_dma_config(ch_rx, dma_data_width, host->spi_dma_info->burst_type);
			pl080_dma_ctrl(ch_rx, PL080_DMAOP_FLUSH);
			pl080_dma_enqueue(ch_rx, host, host->rx_dma, host->dma_rx_buffer_size);
			pl080_dma_ctrl(ch_rx, PL080_DMAOP_START);
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
			flags = socle_claim_dma_lock();
			socle_disable_dma(ch_rx);
			socle_set_dma_mode(ch_rx, SOCLE_DMA_MODE_SLICE);
			socle_set_dma_ext_hdreq_number(ch_rx, host->spi_dma_info->rx_dma_hdreq);
			socle_set_dma_burst_type(ch_rx, host->spi_dma_info->burst_type);
			socle_set_dma_source_address(ch_rx, host->spi_fifo_addr);
			socle_set_dma_destination_address(ch_rx, host->rx_dma);
			socle_set_dma_source_direction(ch_rx, SOCLE_DMA_DIR_FIXED);
			socle_set_dma_destination_direction(ch_rx, SOCLE_DMA_DIR_INCR);
			socle_set_dma_data_size(ch_rx, dma_data_width);
			socle_set_dma_transfer_count(ch_rx, host->dma_rx_buffer_size);
			socle_set_dma_slice_count(ch_rx, host->spi_dma_info->fifo_depth>>1);
			socle_set_dma_page_number(ch_rx, 1);
			socle_set_dma_buffer_size(ch_rx, host->dma_rx_buffer_size);
			socle_enable_dma(ch_rx);
			socle_release_dma_lock(flags);
#endif

		}else {
//printk("RX transfetr with interrupt\n");

				/* Enable SPI interrupt */
			socle_spi_write(SOCLE_SPI_IER,
					socle_spi_read(SOCLE_SPI_IER, host->base) |
					//SOCLE_SPI_IER_TXFIFO_INT_EN |
					SOCLE_SPI_IER_RXFIFO_INT_EN |
					//SOCLE_SPI_IER_RXFIFO_OVR_INT_EN |
					SOCLE_SPI_IER_RX_COMPLETE_INT_EN, host->base);
		}
	}

	/* Start SPI transfer */
	socle_spi_write(SOCLE_SPI_FWCR,
			socle_spi_read(SOCLE_SPI_FWCR, host->base) |
			SOCLE_SPI_TXRX_RUN, host->base);
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	udelay(100);		//leonid+ 20090420 for multi-spi sw work around
#endif
	wait_event_interruptible(host->wq, host->xfer_stat == xfer_done);

	/* Wait for the transaction to be complete */
	do {
		tmp = socle_spi_read(SOCLE_SPI_FWCR, host->base);
	} while ((tmp & SOCLE_SPI_TXRX_RUN) == SOCLE_SPI_TXRX_RUN);

	if (host->tx_buf && host->rx_buf)
	{
		//t->len= host->rx_xfer_cnt; //2009-04-08: Peter+ for returning correct length
		//ret = host->rx_xfer_cnt;
		t->len= host->rx_total_cnt; //2009-04-08: Peter+ for returning correct length
		ret = host->rx_total_cnt;
	}
	else if (host->tx_buf)
	{
		//t->len= host->tx_xfer_cnt; //2009-04-08: Peter+ for returning correct length
		//ret = host->tx_xfer_cnt;
		t->len= host->tx_total_cnt; //2009-04-08: Peter+ for returning correct length
		ret = host->tx_total_cnt;
	}
	else
	{
		//t->len= host->rx_xfer_cnt; //2009-04-08: Peter+ for returning correct length
		//ret = host->rx_xfer_cnt;
		t->len= host->rx_total_cnt; //2009-04-08: Peter+ for returning correct length
		ret = host->rx_total_cnt;
	}

	host->state &= ~SPIBUSY;
	
	act_flag=0;

	return ret;
}


static irqreturn_t
socle_spi_isr(int irq, void *_host)
{
	u32 tmp;
	struct socle_spi_host *host = (struct socle_spi_host *)_host;

	dev_dbg(host->dev, "socle_spi_isr()\n");

	/* Read & clear interrupt status */
	tmp = socle_spi_read(SOCLE_SPI_ISR, host->base);

	dev_dbg(host->dev, "int_stat: 0x%08x\n", tmp);

	/* Check if the receive data is overrun */
	if (SOCLE_SPI_RXFIFO_OVR_INT == (tmp & SOCLE_SPI_RXFIFO_OVR_INT)) {
		dev_dbg(host->dev, "receive data is overrun\n");
		dev_err(host->dev, "receive FIFO is full and another character has been received in the receiver shift register\n");
		if (host->tx_buf)
			host->xfer_stat |= TX_XFER_DONE;
		if (host->rx_buf)
			host->xfer_stat |= RX_XFER_DONE;
		wake_up_interruptible(&host->wq);
		goto out;
	}

	/* Check is receive complete */
	if (SOCLE_SPI_RX_COMPLETE_INT == (tmp &SOCLE_SPI_RX_COMPLETE_INT)) {
		DBG("@-SOCLE_SPI_RX_COMPLETE_INT\n");
		dev_dbg(host->dev, "receive is complete\n");
		while (SOCLE_SPI_RXFIFO_DATA_AVAIL == (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_RXFIFO_DATA_AVAIL)) {
			udelay(100);
			host->get_rx(host);
		}

		/* Disable the rx interrupt */
				socle_spi_write(SOCLE_SPI_IER,
						socle_spi_read(SOCLE_SPI_IER, host->base)
							&~(SOCLE_SPI_IER_RXFIFO_INT_EN | SOCLE_SPI_IER_RX_COMPLETE_INT_EN)
							, host->base
					);
		
		host->xfer_stat |= RX_XFER_DONE;
		wake_up_interruptible(&host->wq);
		goto out;
	}

	/* Check if any receive data is available */
	if (SOCLE_SPI_RXFIFO_INT == (tmp & SOCLE_SPI_RXFIFO_INT)) {
		DBG("@-SOCLE_SPI_RXFIFO_INT\n");
		dev_dbg(host->dev, "receive data is available\n");
		while (SOCLE_SPI_RXFIFO_DATA_AVAIL == (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_RXFIFO_DATA_AVAIL)) {
			host->get_rx(host);
		}
		goto out;
	}

	/* Check if the transmit FIFO is available */
	if (SOCLE_SPI_TXFIFO_INT == (tmp & SOCLE_SPI_TXFIFO_INT)) {
		DBG("@-SOCLE_SPI_TXFIFO_INT\n");
		dev_dbg(host->dev, "transmit FIFO is available\n");
		while (SOCLE_SPI_TXFIFO_FULL != (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_TXFIFO_FULL)) {
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
			udelay(10);		//leonid+ 20090420 for multi-spi sw work around
#endif
			host->set_tx(host);
			if (host->tx_xfer_cnt == host->len) {
				/* Disable the tx interrupt */
				socle_spi_write(SOCLE_SPI_IER,
						socle_spi_read(SOCLE_SPI_IER, host->base) & ~SOCLE_SPI_IER_TXFIFO_INT_EN, host->base
					);

				host->xfer_stat |= TX_XFER_DONE;
				wake_up_interruptible(&host->wq);
				goto out;
			}
		}
	}
out:
	return IRQ_HANDLED;
}

static int
socle_spi_probe(struct platform_device *pdev)
{
	struct socle_spi_host *host;
	struct spi_master *master;
	struct resource *res;
	int err;

	//DBG("##--start\n");
	dev_dbg(&pdev->dev, "socle_spi_probe()\n\n\n");
	master = spi_alloc_master(&pdev->dev, sizeof(struct socle_spi_host));
	if (NULL == master) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	host = spi_master_get_devdata(master);
	memset(host, 0, sizeof(struct socle_spi_host));
	host->master = spi_master_get(master);
	host->master->bus_num = pdev->id;
	host->master->num_chipselect = 8;
	host->dev = &pdev->dev;
	init_waitqueue_head(&host->wq);

	/* initial the spi master setup flag */
	host->master_setup = 0;

	/* Setup the state for bitbang driver */
	host->bitbang.master = host->master;
	host->bitbang.setup_transfer = socle_spi_setup_transfer;
	host->bitbang.chipselect = socle_spi_chipselect;
	host->bitbang.txrx_bufs = socle_spi_txrx_bufs;
	host->bitbang.master->setup = socle_spi_setup;

	dev_dbg(&pdev->dev, "bitbang at %p\n", &host->bitbang);
	
	/* Find and claim our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_io_res;
	}
	host->io_area = request_mem_region(res->start, (res->end - res->start), pdev->name);
	if (NULL == host->io_area) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		err = -ENXIO;
		goto err_no_io_res;
	}
	//socle_spi_base = IO_ADDRESS(host->io_area->start);		//leonid 20090420 del for no use socle_spi_base
	host->base = IO_ADDRESS(host->io_area->start);		//leonid+ 20090420


#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_MDK_3D) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_spi_write(SOCLE_SPI_FWCR, 	socle_spi_read(SOCLE_SPI_FWCR, host->base) |SOCLE_SPI_MODE_MASTER, host->base);
#endif	

		/* Reset SPI controller */
  socle_spi_write(SOCLE_SPI_FWCR,
  	socle_spi_read(SOCLE_SPI_FWCR, host->base) |
		SOCLE_SPI_MASTER_SOFT_RST, host->base);
	

			
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	/* Allocate the interrupt */
	err = request_irq(host->irq, socle_spi_isr, IRQF_DISABLED, pdev->name, host);
	if (err) {
		dev_err(&pdev->dev, "cannot claim IRQ\n");
		goto err_no_irq;
	}

	platform_set_drvdata(pdev, host);
	
	/* Register our spi controller */
	err = spi_bitbang_start(&host->bitbang);
	if (err) {
		dev_err(&pdev->dev, "failed to register SPI master\n");
		goto err_register;
	}

	if(master->bus_num == 0) {
		host->spi_dma_info = &spi_dma_info_0;
		host->spi_fifo_addr = SOCLE_SPI0;
	} else {
		host->spi_dma_info = &spi_dma_info_1;
		host->spi_fifo_addr = SOCLE_SPI1;
	}
	
#ifdef CONFIG_SPI_SOCLE_PL080_HDMA

	err = pl080_dma_request(host->spi_dma_info->tx_dma_ch, host->spi_dma_info->tx_client, NULL);
	if(err) {
		printk("Error : requset TX ARMdma fail\n");
		goto err_no_tx_dma;
	}

	err = pl080_dma_request(host->spi_dma_info->rx_dma_ch, host->spi_dma_info->rx_client, NULL);
	if(err) {
		printk("Error : requset RX ARM dma fail\n");
		goto err_no_rx_dma;
	}

	pl080_dma_set_buffdone_fn(host->spi_dma_info->tx_dma_ch, socle_spi_tx_dma_buffer_done);
	pl080_dma_set_buffdone_fn(host->spi_dma_info->rx_dma_ch, socle_spi_rx_dma_buffer_done);
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
	host->socle_spi_tx_dma_notifier.data = host;
	host->socle_spi_rx_dma_notifier.data = host;
	host->socle_spi_tx_dma_notifier.complete = socle_spi_tx_dma_notifier_complete;
	host->socle_spi_rx_dma_notifier.complete = socle_spi_rx_dma_notifier_complete;
		
	if(master->bus_num == 0) {
		err = socle_request_dma(host->spi_dma_info->tx_dma_ch, "socle spi tx hdma", &host->socle_spi_tx_dma_notifier);
		if(err) {
			printk("Error : requset TX dma fail\n");
			goto err_no_tx_dma;
		}
			
		err = socle_request_dma(host->spi_dma_info->rx_dma_ch, "socle spi rx hdma", &host->socle_spi_rx_dma_notifier);
		if(err) {
			printk("Error : requset RX dma fail\n");
			goto err_no_rx_dma;
		}
	}
#endif

	return 0;
//	dev_dbg(&pdev->dev, "shutdown=%d\n", host->bitbang.shutdown);
#ifdef CONFIG_SPI_SOCLE_PL080_HDMA

err_no_rx_dma:
	 pl080_dma_free(host->spi_dma_info->tx_dma_ch, host->spi_dma_info->rx_client);
err_no_tx_dma:
	
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA

err_no_rx_dma:
	 socle_free_dma(host->spi_dma_info->tx_dma_ch);
err_no_tx_dma:
	
#endif

err_register:
err_no_irq:
	release_resource(host->io_area);
err_no_io_res:
	spi_master_put(host->master);
err_nomem:
	return err;
	
}

static int 
socle_spi_remove(struct platform_device *pdev)
{
	struct socle_spi_host *host = platform_get_drvdata(pdev);

	dev_dbg(host->dev, "socle_spi_remove()\n");
	if (!host)
		return -1;

#ifdef CONFIG_SPI_SOCLE_PL080_HDMA
	pl080_dma_free(host->spi_dma_info->tx_dma_ch, host->spi_dma_info->rx_client);
	pl080_dma_free(host->spi_dma_info->rx_dma_ch, host->spi_dma_info->tx_client);
#elif CONFIG_SPI_SOCLE_PANTHER7_HDMA
	socle_free_dma(host->spi_dma_info->tx_dma_ch);
	socle_free_dma(host->spi_dma_info->rx_dma_ch);
#endif
	platform_set_drvdata(pdev, NULL);
	spi_unregister_master(host->master);
	free_irq(host->irq, host);
	release_resource(host->io_area);
	spi_master_put(host->master);
	return 0;
}

#ifdef CONFIG_PM
static int 
socle_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct socle_spi_host *host = platform_get_drvdata(pdev);
//printk("#-%s-start\n", __func__);	
	while (host->state & SPIBUSY)
		msleep(10);

	/* disable the spi master */
	socle_spi_write(SOCLE_SPI_FWCR, 
		socle_spi_read(SOCLE_SPI_FWCR, host->base) & ~SOCLE_SPI_MASTER_EN
		, host->base);
	
	return 0;
}

static int
socle_spi_resume(struct platform_device *pdev)
{
	struct socle_spi_host *host = platform_get_drvdata(pdev);
	struct spi_device *spi = host->spi_dev;
	int err;
//printk("#-%s-start\n", __func__);
		
	/* Reset SPI controller */
  socle_spi_write(SOCLE_SPI_FWCR,
  	socle_spi_read(SOCLE_SPI_FWCR, host->base) |
		SOCLE_SPI_MASTER_SOFT_RST, host->base);

	/* if this spi master haved setuped, setup again at resume */ 
	if (host->master_setup) {
		err = socle_spi_setup_transfer(spi, NULL);
		if (err)
			dev_err(&spi->dev, "setup_transfer returned %d\n", err);
	}
	
	return 0;
}
#else
#define socle_spi_suspend NULL
#define socle_spi_resume NULL
#endif

static struct platform_driver socle_spi_driver = {
	.probe = socle_spi_probe,
	.remove = socle_spi_remove,
	.suspend = socle_spi_suspend,
	.resume = socle_spi_resume,
	.driver = {
		.name = "socle_spi",
		.owner = THIS_MODULE,
	},
};

static int __init
socle_spi_init(void)
{
	return  platform_driver_register(&socle_spi_driver);
}

static void __exit
socle_spi_exit(void)
{
	platform_driver_unregister(&socle_spi_driver);
}

subsys_initcall(socle_spi_init);
//module_init(socle_spi_init);
module_exit(socle_spi_exit);

MODULE_DESCRIPTION("Socle SPI Driver");
MODULE_AUTHOR("Obi Hsieh");
MODULE_LICENSE("GPL");
