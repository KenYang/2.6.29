#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/hardware.h>
#include <mach/i2c-regs.h>

#include <mach/socle-scu.h>


/*
 *  Read register
 *  */
static u32 inline
socle_reg_read(u32 reg, u32 base)
{
	return ioread32(base+reg);
}

/*
 *  Write register
 *  */
static void inline
socle_reg_write(u32 reg, u32 val, u32 base)
{
	iowrite32(val, base+reg);
}

struct socle_i2c_host {
	wait_queue_head_t wq;
	struct resource *io_area;
	u32 va_base;
	int irq;
	u8 arbit_lose : 1;
	u8 ack_period : 1;
	u8 recv_ack : 1;
	struct device *dev;
	struct i2c_adapter adap;
};

static int socle_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);
static u32 socle_i2c_functionality(struct i2c_adapter *adap);
static int socle_i2c_do_transfer(struct socle_i2c_host *host, struct i2c_msg *pmsgs, int num);
static int socle_i2c_do_address(struct i2c_msg *msg, struct socle_i2c_host *host);
static int socle_i2c_read_bytes(struct i2c_msg *msgs, struct socle_i2c_host *host);
static int socle_i2c_send_bytes(struct i2c_msg *msgs, struct socle_i2c_host *host);
static void socle_i2c_master_write_byte(u8 data, struct socle_i2c_host *host);
static int socle_i2c_master_read_byte(struct socle_i2c_host *host);
static int socle_i2c_master_check_ack(struct socle_i2c_host *host);
static void socle_i2c_ack(struct socle_i2c_host *host);
static void socle_i2c_nak(struct socle_i2c_host *host);
static void socle_i2c_start(struct socle_i2c_host *host);
static void socle_i2c_stop(struct socle_i2c_host *host);
static void socle_i2c_resume(struct socle_i2c_host *host);
static irqreturn_t socle_i2c_isr(int irq_no, void *_host);
static u32 socle_i2c_power(u32 base, u32 exp);
static u32 socle_i2c_calculate_divisor(u32 clk, struct socle_i2c_host *host);
static void socle_i2c_master_initialize(struct socle_i2c_host *host);

/* I2C bus registration info */
static const struct i2c_algorithm socle_i2c_algorithm = {
	.master_xfer = socle_i2c_transfer,
	.functionality = socle_i2c_functionality,
};

static int 
socle_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct socle_i2c_host *host = (struct socle_i2c_host *)adap->algo_data;
	int i, ret;

	for (i = 0; i < adap->retries; i++) {
		ret = socle_i2c_do_transfer(host, msgs, num);
		if (ret != -EAGAIN)
			return ret;
		dev_dbg(host->dev, "Retrying transmission (%d)\n", i);
		udelay(100);
	}
	return -EREMOTEIO;
}

static int
socle_i2c_do_transfer(struct socle_i2c_host *host, struct i2c_msg *msgs, int num)
{
	int ret = -EAGAIN, i, nak_ok;
	struct i2c_msg *pmsgs;
	u32 bus_stat;

	for (i = 0; i < num; i++) {
		pmsgs = &msgs[i];
		nak_ok = pmsgs->flags & I2C_M_IGNORE_NAK;
		if (!((!pmsgs->flags) & I2C_M_NOSTART)) {
			if (0 == i) {
				bus_stat = socle_reg_read(SOCLE_I2C_LSR, host->va_base);
				if (SOCLE_I2C_AFTER_STR_COND_DET == (bus_stat & SOCLE_I2C_AFTER_STR_COND_DET)) {
					dev_dbg(host->dev, "bus is busy\n");
					ret = -EAGAIN;
					goto out;
				}
			}
			ret = socle_i2c_do_address(pmsgs, host);
			if ((ret != 0 ) && !nak_ok) {
				dev_dbg(host->dev, "NAK from device address %2.2x msg #%d\n", msgs[i].addr, i);
				ret = -EAGAIN;
				goto out;
			}
		}
		if (pmsgs->flags & I2C_M_RD) {
			/* Change the master to be as the receiver */
			socle_reg_write(SOCLE_I2C_CONR,
					socle_reg_read(SOCLE_I2C_CONR, host->va_base) &
					~SOCLE_I2C_MASTER_TRAN_SEL,
					host->va_base);

			/* Read bytes into buffer */
			ret = socle_i2c_read_bytes(pmsgs, host);
			if (ret != pmsgs->len) {
				dev_dbg(host->dev, "incomplete read transfer (%d)\n", ret);
				ret = -EAGAIN;
				goto out;
			}
		} else {
			/* Set the master to be the transmitter */
			socle_reg_write(SOCLE_I2C_CONR,
					socle_reg_read(SOCLE_I2C_CONR, host->va_base) |
					SOCLE_I2C_MASTER_TRAN_SEL,
					host->va_base);

			/* Write bytes from buffer */
			ret = socle_i2c_send_bytes(pmsgs, host);
			if (ret != pmsgs->len) {
				dev_dbg(host->dev, "incomplete write transfer (%d)\n", ret);
				ret = -EAGAIN;
				goto out;
			}
		}
	}
	//20080903 Leonid add fix transfer number
	ret=num;
out:
	socle_i2c_stop(host);
	return ret;
}

static u32
socle_i2c_functionality(struct i2c_adapter *adap)
{
	return (I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING);
}

static int
socle_i2c_do_address(struct i2c_msg *msgs, struct socle_i2c_host *host)
{
	u16 flags = msgs->flags;
	u16 nak_ok = msgs->flags & I2C_M_IGNORE_NAK;
	u8 addr;
	int ret;

	/* Set the master to be the transmitter */
	socle_reg_write(SOCLE_I2C_CONR,
			socle_reg_read(SOCLE_I2C_CONR, host->va_base) |
			SOCLE_I2C_MASTER_TRAN_SEL,
			host->va_base);

	if (flags & I2C_M_TEN) { /* A ten bits address */
		addr = 0xf0 | ((msgs->addr >> 7) & 0x06);

		/* Try extended address code... */
		socle_i2c_master_write_byte(addr, host);
		socle_i2c_start(host);
		ret = socle_i2c_master_check_ack(host);
		if ((ret != 0) && !nak_ok)
			goto out;

		/* The remaining 8 bit address */
		socle_i2c_master_write_byte(msgs->addr & 0x7f, host);
		socle_i2c_resume(host);
		ret = socle_i2c_master_check_ack(host);
		if ((ret != 0) && !nak_ok)
			goto out;

		if (flags & I2C_M_RD) {
			/* Okay, now switch into reading mode */
			addr |= 0x01;
			socle_i2c_master_write_byte(addr, host);
			socle_i2c_start(host);
			ret = socle_i2c_master_check_ack(host);
			if ((ret != 0) && !nak_ok)
				goto out;
		}
	} else {		/* normal 7 bits address */
		addr = msgs->addr << 1;
		if (flags & I2C_M_RD)
			addr |= 1;
		if (flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;
		socle_i2c_master_write_byte(addr, host);
		socle_i2c_start(host);
		ret = socle_i2c_master_check_ack(host);
		if ((ret != 0) && !nak_ok)
			goto out;
	}
	return 0;
out:
	return -1;
}

static int
socle_i2c_read_bytes(struct i2c_msg *msgs, struct socle_i2c_host *host)
{
	int inval;
	u16 flags = msgs->flags;
	int rd_cnt = 0;
	u8 *tmp = msgs->buf;
	int cnt = msgs->len;

	while (cnt > 0) {
		socle_i2c_resume(host);
		inval = socle_i2c_master_read_byte(host);
		if (-1 == inval)
			goto out;
		dev_dbg(host->dev, "reading 0x%02x\n", inval&0xff);
		*tmp = inval;
		rd_cnt++;
		tmp++;
		cnt--;
		if (flags & I2C_M_NO_RD_ACK)
			continue;
		if (cnt > 0)
			socle_i2c_ack(host);
		else
			/* Neg. ack on last byte */
			socle_i2c_nak(host);
	}
out:
	return rd_cnt;
}

static int
socle_i2c_send_bytes(struct i2c_msg *msgs, struct socle_i2c_host *host)
{
	char c;
	const char *tmp = msgs->buf;
	int cnt = msgs->len;
	u16 nak_ok = msgs->flags & I2C_M_IGNORE_NAK;
	int ret;
	int wr_cnt = 0;

	while (cnt > 0) {
		c = *tmp;
		dev_dbg(host->dev, "sending 0x%02x\n", c&0xff);
		socle_i2c_master_write_byte(c, host);
		socle_i2c_resume(host);
		ret = socle_i2c_master_check_ack(host);
		if ((ret != 0) && !nak_ok) {
			dev_dbg(host->dev, "error - bailout\n");
			break;
		} else {
			cnt--;
			tmp++;
			wr_cnt++;
		}
	}
	return wr_cnt;
}

static void
socle_i2c_master_write_byte(u8 data, struct socle_i2c_host *host)
{
	socle_reg_write(SOCLE_I2C_MTXR, data, host->va_base);
}

static int
socle_i2c_master_read_byte(struct socle_i2c_host *host)
{
	int data;
	unsigned long timeout;

	/* Wait for the master ack period interrupt occuring */
	timeout = wait_event_interruptible_timeout(host->wq, (host->ack_period == 1) || (host->arbit_lose == 1), host->adap.timeout*HZ);
	if (host->arbit_lose) {
		dev_dbg(host->dev, "arbitraton losting on waiting for the master ack period interrupt occuring\n");
		host->arbit_lose = 0;
		data = -1;
		goto out;
	}
	if (0 == timeout) {
		dev_dbg(host->dev, "timeout on waiting for the master ack period interrupt occuring\n");
		data = -1;
		goto out;
	}
	data = socle_reg_read(SOCLE_I2C_MRXR, host->va_base);
out:
	host->ack_period = 0;
	return data;
}

static int
socle_i2c_master_check_ack(struct socle_i2c_host *host)
{
	u32 ack_stat;
	unsigned long timeout;

	/* Wait for the master receive ack interrupt occuring */
	timeout = wait_event_interruptible_timeout(host->wq, (host->recv_ack == 1) || (host->arbit_lose == 1), host->adap.timeout*HZ);
	if (host->arbit_lose) {
		dev_dbg(host->dev, "arbitraton losting on waiting for the master receive ack interrupt occuring\n");
		host->arbit_lose = 0;
		ack_stat = -1;
		goto out;
	}
	if (0 == timeout) {
		dev_dbg(host->dev, "timeout on waiting for the master receive ack interrupt occuring\n");
		ack_stat = -1;
		goto out;
	}

	/* Check the ack status */
	ack_stat = socle_reg_read(SOCLE_I2C_LSR, host->va_base);
	if (SOCLE_I2C_RECV_STAT_NAK == (ack_stat & SOCLE_I2C_RECV_STAT_NAK))
		ack_stat = -1;
	else
		ack_stat = 0;

out:
	host->recv_ack = 0;
	return ack_stat;
}

static void
socle_i2c_ack(struct socle_i2c_host *host)
{
	u32 tmp;

	tmp = socle_reg_read(SOCLE_I2C_CONR, host->va_base);
	tmp &= ~SOCLE_I2C_BUS_ACK_DIS;
	socle_reg_write(SOCLE_I2C_CONR, tmp, host->va_base);
}

static void
socle_i2c_nak(struct socle_i2c_host *host)
{
	u32 tmp;
	
	tmp = socle_reg_read(SOCLE_I2C_CONR, host->va_base);
	tmp |= SOCLE_I2C_BUS_ACK_DIS;
	socle_reg_write(SOCLE_I2C_CONR, tmp, host->va_base);
}



static void
socle_i2c_start(struct socle_i2c_host *host)
{
	socle_reg_write(SOCLE_I2C_LCMR,
			SOCLE_I2C_RESUME_COND_GEN_EN |
			SOCLE_I2C_STP_COND_GEN_DIS |
			SOCLE_I2C_STR_COND_GEN_EN,
			host->va_base);
}

static void
socle_i2c_stop(struct socle_i2c_host *host)
{
	socle_reg_write(SOCLE_I2C_LCMR,
			SOCLE_I2C_RESUME_COND_GEN_EN |
			SOCLE_I2C_STP_COND_GEN_EN |
			SOCLE_I2C_STR_COND_GEN_DIS,
			host->va_base);
}

static void
socle_i2c_resume(struct socle_i2c_host *host)
{
	socle_reg_write(SOCLE_I2C_LCMR,
			SOCLE_I2C_RESUME_COND_GEN_EN |
			SOCLE_I2C_STP_COND_GEN_DIS |
			SOCLE_I2C_STR_COND_GEN_DIS,
			host->va_base);
}

static irqreturn_t
socle_i2c_isr(int irq_no, void *_host)
{
	struct socle_i2c_host *host = (struct socle_i2c_host *)_host;
	u32 tmp;

	tmp = socle_reg_read(SOCLE_I2C_ISR, host->va_base);
	if (SOCLE_I2C_ARBIT_LOSE_INT == (SOCLE_I2C_ARBIT_LOSE_INT & tmp)) {
		dev_dbg(host->dev, "arbitration lose occurs\n");

		/* Clear interrupt */
		socle_reg_write(SOCLE_I2C_ISR, 
				tmp & (~SOCLE_I2C_ARBIT_LOSE_INT),
				host->va_base);

		host->arbit_lose = 1;
		wake_up_interruptible(&host->wq);
		
	}
	if (SOCLE_I2C_MASTER_ACK_PERIOD_INT == (SOCLE_I2C_MASTER_ACK_PERIOD_INT & tmp)) {
		dev_dbg(host->dev, "ACK period interrupt generation\n");

		/* Clear interrupt */
		socle_reg_write(SOCLE_I2C_ISR,
				tmp & (~SOCLE_I2C_MASTER_ACK_PERIOD_INT),
				host->va_base);
		
		host->ack_period = 1;
		wake_up_interruptible(&host->wq);
	}
	if (SOCLE_I2C_MASTER_RECV_ACK_INT == (SOCLE_I2C_MASTER_RECV_ACK_INT & tmp)) {
		dev_dbg(host->dev, "receive ACK interrupt generation\n");

		/* Clear interrupt */
		socle_reg_write(SOCLE_I2C_ISR,
				tmp & (~SOCLE_I2C_MASTER_RECV_ACK_INT),
				host->va_base);
		host->recv_ack = 1;
		wake_up_interruptible(&host->wq);
	}
	return IRQ_HANDLED;
}

static u32
socle_i2c_power(u32 base, u32 exp)
{
	u32 i, val = 1;

	if (0 == exp)
		return 1;
	else {
		for (i = 0; i < exp; i++)
			val *= base;
		return val;
	}
}

static u32
socle_i2c_calculate_divisor(u32 clk, struct socle_i2c_host *host)
{
	u8 div_high_3 = 0, div_low_3 = 0, i2c_cdvr = 0;
	u32 sclk_divisor, sclk, pclk, power;

	/*
	 *  SCL Divisor = (I2CCDVR[5:3] + 1) * 2 power (I2CCDVR[2:0} + 1
	 *  SCL = PCLK / 5 * SCLK Divisor
	 *  */
	//20080903 Leonid Fix
	//pclk = 50000000;
	pclk = socle_scu_apb_clock_get();
	//printk("pclk = %d\n", pclk);	
	while (1) {
		power = socle_i2c_power(2, div_low_3+1);
		for (div_high_3 = 0; div_high_3 < 8; div_high_3++) {
			sclk_divisor = (div_high_3 + 1) * power;
			sclk = pclk / (5 * sclk_divisor);
			if (sclk < clk)
				goto out;
		}
		div_low_3++;
	}
out:
	i2c_cdvr = (div_high_3 << 3) | div_low_3;
	dev_dbg(host->dev, "pclk:%d, sclk:%d, I2CCDVR:0x%02x\n", pclk, sclk, i2c_cdvr);
	return i2c_cdvr;
}

static void
socle_i2c_master_initialize(struct socle_i2c_host *host)
{
	u32 i2c_cdvr = 0;

       /* Enable the I2C controller */
       socle_reg_write(SOCLE_I2C_OPR,
		       socle_reg_read(SOCLE_I2C_OPR, host->va_base) |
		       SOCLE_I2C_CORE_EN,
		       host->va_base);

       /* Enable the master port */
       socle_reg_write(SOCLE_I2C_CONR,
		       socle_reg_read(SOCLE_I2C_CONR, host->va_base) |
		       SOCLE_I2C_MASTER_PORT_EN,
		       host->va_base);

       /* Reset I2C state machine of both master and slave */
       socle_reg_write(SOCLE_I2C_OPR,
		       socle_reg_read(SOCLE_I2C_OPR, host->va_base) |
		       SOCLE_I2C_RST,
		       host->va_base);

       /* Restore the reset bit to 0 */
       socle_reg_write(SOCLE_I2C_OPR,
		       socle_reg_read(SOCLE_I2C_OPR, host->va_base) &
		       ~SOCLE_I2C_RST,
		       host->va_base);

       i2c_cdvr = socle_i2c_calculate_divisor(400000, host);

       /* Set the divisor */
       socle_reg_write(SOCLE_I2C_OPR,
		       (socle_reg_read(SOCLE_I2C_OPR, host->va_base) &
			~SOCLE_I2C_CLK_DIVISOR(0x3f)) |
		       SOCLE_I2C_CLK_DIVISOR(i2c_cdvr),
		       host->va_base);

       /* Set interrupt generation of I2C controller */
       socle_reg_write(SOCLE_I2C_IER,
		       SOCLE_I2C_ARBIT_LOSE_INT_EN |
		       SOCLE_I2C_ABNORMAL_STP_INT_DIS |
		       SOCLE_I2C_BROADCAST_ADDR_INT_DIS |
		       SOCLE_I2C_SLAVE_ADDR_INT_DIS |
		       SOCLE_I2C_SLAVE_ACK_PERIOD_INT_DIS |
		       SOCLE_I2C_SLAVE_RECV_ACK_INT_DIS |
		       SOCLE_I2C_MASTER_ACK_PERIOD_INT_EN |
		       SOCLE_I2C_MASTER_RECV_ACK_INT_EN,
		       host->va_base);
}

static int
socle_i2c_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct socle_i2c_host *host;
	int err;

	dev_dbg(&pdev->dev, "socle_i2c_probe()\n");
	host = kzalloc(sizeof(struct socle_i2c_host), GFP_KERNEL);
	if (NULL == host) {
		dev_err(&pdev->dev, "cannot allocate memory to host\n");
		err = -ENOMEM;
		goto err_no_mem;
	}

	/* Find and claim our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_io_res;
	}

	host->io_area = request_mem_region(res->start, (res->end-res->start), pdev->name);
	if (NULL == host->io_area) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		err = -ENXIO;
		goto err_no_io_res;
	}
//	host->va_base = IO_ADDRESS(host->io_area->start);
        host->va_base = (u32)ioremap(res->start, resource_size(res));
        if (!host->va_base) {
                dev_err(&pdev->dev, "cannot map I2C registers\n");
		err = -ENOMEM;
                goto release_mem;
        }
	host->irq = platform_get_irq(pdev, 0);
       if (host->irq < 0) {
	       dev_err(&pdev->dev, "no irq specified\n");
	       err = -ENOENT;
	       goto err_no_irq;
       }

       /* Allocate the interrupt */
       err = request_irq(host->irq, socle_i2c_isr, IRQF_DISABLED, pdev->name, host);
       if (err) {
	       dev_err(&pdev->dev, "cannot claim IRQ\n");
	       goto err_no_irq;

       }

	host->dev = &pdev->dev;
	init_waitqueue_head(&host->wq);
	host->arbit_lose = 0;
	host->ack_period = 0;
	host->recv_ack = 0;

	/* Initialize the I2C adapter */
	snprintf(host->adap.name, I2C_NAME_SIZE, "%s.%u",
		 pdev->name, pdev->id);
	host->adap.owner = THIS_MODULE;
	host->adap.algo = &socle_i2c_algorithm;
	host->adap.retries = 5;
	host->adap.timeout = 5;
	host->adap.class = I2C_CLASS_HWMON;
	host->adap.algo_data = host;
	host->adap.dev.parent = &pdev->dev;

       /* Add bus to I2C core */
       err = i2c_add_adapter(&host->adap);
       if (err < 0) {
	       dev_err(&pdev->dev, "failed to add bus to i2c core\n");
	       goto err_add_adapter;
       }

       platform_set_drvdata(pdev, host);
       socle_i2c_master_initialize(host);

       return 0;
err_add_adapter:
       free_irq(host->irq, host);
err_no_irq:
//       release_resource(host->io_area);
release_mem:
	release_mem_region(res->start, resource_size(res));
err_no_io_res:
       kfree(host);
err_no_mem:
       return err;
}

static int
socle_i2c_remove(struct platform_device *pdev)
{
	struct socle_i2c_host *host = (struct socle_i2c_host *)platform_get_drvdata(pdev);
        struct resource *res = NULL;

	dev_dbg(&pdev->dev, "socle_i2c_remove()\n");
	if (!host)
		return -1;
	platform_set_drvdata(pdev, NULL);
        iounmap((volatile void __iomem *)host->va_base);
	free_irq(host->irq, host);
//	release_resource(host->io_area);
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        release_mem_region(res->start, resource_size(res));

	i2c_del_adapter(&host->adap);
	kfree(host);
	return 0;
}

#ifdef CONFIG_PM
static int
socle_i2c_suspend(struct platform_device *pdev, pm_message_t msg)
{
	pr_debug("socle_i2c_suspend\n");

        return 0;
}

static int 
socle_i2c_bus_resume(struct platform_device *pdev)
{
	struct socle_i2c_host *host = (struct socle_i2c_host *)platform_get_drvdata(pdev);

	pr_debug("socle_i2c_bus_resume\n");
	
        if (host != NULL)
                socle_i2c_master_initialize(host);

	return 0;
}
#else
#define socle_i2c_suspend NULL
#define socle_i2c_bus_resume NULL
#endif

static struct platform_driver socle_i2c_driver =
{
	.probe = socle_i2c_probe,
	.remove = socle_i2c_remove,
	.suspend = socle_i2c_suspend,
	.resume = socle_i2c_bus_resume,
	.driver = {
		.name = "socle_i2c",
		.owner = THIS_MODULE,
	},
};

static int __init
socle_i2c_init(void)
{
	return platform_driver_register(&socle_i2c_driver);
}

static void __exit
socle_i2c_exit(void)
{
	platform_driver_unregister(&socle_i2c_driver);
}

//module_init(socle_i2c_init);
subsys_initcall(socle_i2c_init);
module_exit(socle_i2c_exit);

MODULE_DESCRIPTION("Socle I2C Adaptor/Algorithm Driver");
MODULE_AUTHOR("Obi Hsieh");
MODULE_LICENSE("GPL");
