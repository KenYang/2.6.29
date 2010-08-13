

//#define SPI_DEBUG
#ifdef SPI_DEBUG
	#define DEBUG(fmt, args...) printk("\n[sq_spi.c]: " fmt, ## args)
#else
	#define DEBUG(fmt, args...)
#endif



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

// cyli fix for different tx/rx
#define SET_TX_RX_LEN(tx, rx)	(((tx) << 16) | (rx))
#define GET_TX_LEN(len)		((len) >> 16)
#define GET_RX_LEN(len)		((len) & 0xffff)

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
	void (*get_rx)(struct socle_spi_host *host);
	void (*set_tx)(struct socle_spi_host *host);
};

//static u32 socle_spi_base;		//leonid 20090420 del for no use socle_spi_base

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
	dev_dbg(host->dev, "socle_spi_set_rx_16()\n");
	*((u16 *)host->rx_buf + host->rx_xfer_cnt++) = socle_spi_read(SOCLE_SPI_RXR, host->base);
}

static void
socle_spi_set_tx_16(struct socle_spi_host *host)
{
	dev_dbg(host->dev, "socle_spi_get_tx_16()\n");
	socle_spi_write(SOCLE_SPI_TXR, *((u16 *)host->tx_buf + host->tx_xfer_cnt++), host->base);
}

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

	//socle_spi_base = IO_ADDRESS(host->io_area->start);	//leonid 20090420 del for no use socle_spi_base
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

	//socle_spi_base = IO_ADDRESS(host->io_area->start);		//leonid 20090420 del for no use socle_spi_base
	
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

//printk("#-%s-start\n", __func__);
	dev_dbg(host->dev, "socle_spi_setup_transfer()\n");
	DEBUG("sq_spi_setup_transfer() spi=%x t=%x\n",spi,t);	
	//socle_spi_base = IO_ADDRESS(host->io_area->start);		//leonid 20090420 del for no use socle_spi_base

	host->spi_dev = spi;
	host->master_setup = 1;

	host->bpw = t ? t->bits_per_word : spi->bits_per_word;
	if(!host->bpw)		//jsho+ 20100506
		host->bpw=spi->bits_per_word;
	hz = t ? t->speed_hz : spi->max_speed_hz;
	if(!hz)		//jsho+ 20100506
		hz=spi->max_speed_hz;


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

#if  0
	/* Reset SPI controller */
	socle_spi_write(SOCLE_SPI_FWCR,
			socle_spi_read(SOCLE_SPI_FWCR, IO_ADDRESS(SOCLE_SPI0)) |
			SOCLE_SPI_MASTER_SOFT_RST, IO_ADDRESS(SOCLE_SPI0));
	socle_spi_write(SOCLE_SPI_FWCR,
			socle_spi_read(SOCLE_SPI_FWCR, IO_ADDRESS(SOCLE_SPI1)) |
			SOCLE_SPI_MASTER_SOFT_RST, IO_ADDRESS(SOCLE_SPI1));
//	socle_spi_write(SOCLE_SPI_FWCR,
//			socle_spi_read(SOCLE_SPI_FWCR, host->base) |
//			SOCLE_SPI_MASTER_SOFT_RST, host->base);
#endif

	/* Enable SPI interrupt */
	socle_spi_write(SOCLE_SPI_IER,
//			SOCLE_SPI_IER_TXFIFO_INT_EN |
			SOCLE_SPI_IER_RXFIFO_INT_EN |
//			SOCLE_SPI_IER_RXFIFO_OVR_INT_EN |
			SOCLE_SPI_IER_RX_COMPLETE_INT_EN, host->base);

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
			SOCLE_SPI_OP_NORMAL, host->base);

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
			SOCLE_SPI_OP_NORMAL, host->base);
#endif

	/* Configure FIFO and clear Tx & RX FIFO */
	socle_spi_write(SOCLE_SPI_FCR,
			SOCLE_SPI_RXFIFO_INT_TRIGGER_LEVEL_4 |
			SOCLE_SPI_TXFIFO_INT_TRIGGER_LEVEL_4 
#if  0
		|	SOCLE_SPI_RXFIFO_CLR 
		|	SOCLE_SPI_TXFIFO_CLR
#endif
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
			SOCLE_SPI_PBTXRX_DELAY_NONE |		//20100629 jerry fix
#else 
			SOCLE_SPI_PBTXRX_DELAY_32 |		//20100629 jerry fix 
#endif
			SOCLE_SPI_PBCT_DELAY_NONE |
			SOCLE_SPI_PBCA_DELAY_1_2, host->base);

	return 0;
}

static int
socle_spi_setup(struct spi_device *spi)
{
	struct socle_spi_host *host = (struct socle_spi_host *)spi_master_get_devdata(spi->master);
	int err = 0;

	dev_dbg(host->dev, "socle_spi_setup()\n");
	DEBUG("sq_spi_setup() bits_per_word=%d\n", spi->bits_per_word); //channignlan
	
	//socle_spi_base = IO_ADDRESS(host->io_area->start);		//leonid 20090420 del for no use socle_spi_base
	
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
	u8 xfer_done = 0;
	u32 tmp;
	u32 ret;
#ifdef SPI_DEBUG
	char *p; //channinglan
#endif
	dev_dbg(host->dev, "socle_spi_txrx_bufs()\n");
	
	//socle_spi_base = IO_ADDRESS(host->io_area->start);		//leonid 20090420 del for no use socle_spi_base

	host->state |= SPIBUSY;
	
	host->tx_buf = t->tx_buf;
#ifdef SPI_DEBUG
	p = t->tx_buf;//channinglan
#endif
	host->rx_buf = t->rx_buf;
	host->len = GET_TX_LEN(t->len);
//	host->len = t->len;
	host->tx_xfer_cnt = 0;
	host->rx_xfer_cnt = 0;
	host->xfer_stat = 0;
	if (host->bpw > 8) {
		host->get_rx = socle_spi_get_rx_16;
		host->set_tx = socle_spi_set_tx_16;
	} else {
		host->get_rx = socle_spi_get_rx_8;
		host->set_tx = socle_spi_set_tx_8;
	}

	/* Set transfer & receive data count */
	if (host->tx_buf) {
		socle_spi_write(SOCLE_SPI_TXCR, GET_TX_LEN(t->len), host->base);
//		socle_spi_write(SOCLE_SPI_TXCR, (t->len), host->base);
		xfer_done |= TX_XFER_DONE;
	} else
		socle_spi_write(SOCLE_SPI_TXCR, 0, host->base);
	if (host->rx_buf) {
		socle_spi_write(SOCLE_SPI_RXCR, GET_RX_LEN(t->len), host->base);
//		socle_spi_write(SOCLE_SPI_RXCR, (t->len), host->base);
		xfer_done |= RX_XFER_DONE;
	} else
		socle_spi_write(SOCLE_SPI_RXCR, 0, host->base);

	/* Write the data into tx fifo first */
	//FIX send out with out complete done flag  JS 20071220
	if (host->tx_buf) {
		DEBUG("\n tx %x %x",*(p),*(p+1)); //channinglan
		while (SOCLE_SPI_TXFIFO_FULL != (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_TXFIFO_FULL)) {
			host->set_tx(host);
			if (host->tx_xfer_cnt == host->len) {
				host->xfer_stat |= TX_XFER_DONE;
				break;
			}
		}		
	}
	if((host->xfer_stat != TX_XFER_DONE)&& host->tx_buf)
		socle_spi_write(SOCLE_SPI_IER, socle_spi_read(SOCLE_SPI_IER, host->base) | SOCLE_SPI_IER_TXFIFO_INT_EN, host->base);
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
		t->len= host->rx_xfer_cnt; //2009-04-08: Peter+ for returning correct length
		ret = host->rx_xfer_cnt;
	}
	else if (host->tx_buf)
	{
		t->len= host->tx_xfer_cnt; //2009-04-08: Peter+ for returning correct length
		ret = host->tx_xfer_cnt;
	}
	else
	{
		t->len= host->rx_xfer_cnt; //2009-04-08: Peter+ for returning correct length
		ret = host->rx_xfer_cnt;
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
		dev_dbg(host->dev, "receive is complete\n");
		while (SOCLE_SPI_RXFIFO_DATA_AVAIL == (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_RXFIFO_DATA_AVAIL)) {
			udelay(100);
			host->get_rx(host);
		}
		host->xfer_stat |= RX_XFER_DONE;
		wake_up_interruptible(&host->wq);
		goto out;
	}

	/* Check if any receive data is available */
	if (SOCLE_SPI_RXFIFO_INT == (tmp & SOCLE_SPI_RXFIFO_INT)) {
		dev_dbg(host->dev, "receive data is available\n");
		while (SOCLE_SPI_RXFIFO_DATA_AVAIL == (socle_spi_read(SOCLE_SPI_FCR, host->base) & SOCLE_SPI_RXFIFO_DATA_AVAIL)) {
			host->get_rx(host);
		}
		goto out;
	}

	/* Check if the transmit FIFO is available */
	if (SOCLE_SPI_TXFIFO_INT == (tmp & SOCLE_SPI_TXFIFO_INT)) {
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

	dev_dbg(&pdev->dev, "socle_spi_probe()\n\n\n");
	DEBUG("sq_spi_probe()\n");

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

//	dev_dbg(&pdev->dev, "shutdown=%d\n", host->bitbang.shutdown);
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
