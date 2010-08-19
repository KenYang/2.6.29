/*
 *  linux/drivers/char/8250_btuart.c
 *
 *  Driver for 8250/16550-type Bluetooth-serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.
 *  membase is an 'ioremapped' cookie.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/irq.h>

#include "8250.h"

void btuart_unregister_port(int line);
int btuart_register_port(struct uart_port *port);

/*
 * Configuration:
 *   share_irqs - whether we pass IRQF_SHARED to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
static unsigned int share_irqs = SERIAL8250_SHARE_IRQS;

static unsigned int nr_uarts = CONFIG_SERIAL_8250_RUNTIME_UARTS;

/*
 * Debugging.
 */
#if 0
#define DEBUG_AUTOCONF(fmt...)	printk(fmt)
#else
#define DEBUG_AUTOCONF(fmt...)	do { } while (0)
#endif

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#define PASS_LIMIT	256


#include <asm/serial.h>


#define UART_NR	1


struct btuart_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	unsigned short		capabilities;	/* port capabilities */
	unsigned short		bugs;		/* port bugs */
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */

	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;

	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

struct irq_info {
	spinlock_t		lock;
	struct btuart_port *up;
};

static struct irq_info btuart_irq[1];

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
static const struct serial8250_config uart_config[] = {
	[PORT_UNKNOWN] = {
		.name		= "unknown",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_8250] = {
		.name		= "8250",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16450] = {
		.name		= "16450",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550] = {
		.name		= "16550",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550A] = {
		.name		= "16550A",
		.fifo_size	= 16,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO,
	},
};


/* sane hardware needs no mapping */
#define map_8250_in_reg(up, offset) (offset)
#define map_8250_out_reg(up, offset) (offset)


static unsigned int serial_in(struct btuart_port *up, int offset)
{
	offset = map_8250_in_reg(up, offset) << up->port.regshift;

		return readb(up->port.membase + offset);
}

static void
serial_out(struct btuart_port *up, int offset, int value)
{
	/* Save the offset before it's remapped */
	offset = map_8250_out_reg(up, offset) << up->port.regshift;

		writeb(value, up->port.membase + offset);
}


/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */
#define serial_inp(up, offset)		serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)

/* Uart divisor latch read */
static inline int _serial_dl_read(struct btuart_port *up)
{
	return serial_inp(up, UART_DLL) | serial_inp(up, UART_DLM) << 8;
}

/* Uart divisor latch write */
static inline void _serial_dl_write(struct btuart_port *up, int value)
{
	serial_outp(up, UART_DLL, value & 0xff);
	serial_outp(up, UART_DLM, value >> 8 & 0xff);
}

#define serial_dl_read(up) _serial_dl_read(up)
#define serial_dl_write(up, value) _serial_dl_write(up, value)


/*
 * FIFO support.
 */
static inline void serial8250_clear_fifos(struct btuart_port *p)
{
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_outp(p, UART_FCR, 0);
}


/*
 * This routine is called by rs_init() to initialize a specific serial
 * port.
 */
static void autoconfig(struct btuart_port *up, unsigned int probeflags)
{
	unsigned long flags;

	if (!up->port.iobase && !up->port.mapbase && !up->port.membase)
		return;

	DEBUG_AUTOCONF("ttyBT%d: autoconf (0x%04x, 0x%p): ",
			up->port.line, up->port.iobase, up->port.membase);

	spin_lock_irqsave(&up->port.lock, flags);

	up->capabilities = 0;
	up->bugs = 0;

	up->port.type = PORT_16550A;
	up->capabilities |= UART_CAP_FIFO;

	up->port.fifosize = uart_config[up->port.type].fifo_size;
	up->capabilities = uart_config[up->port.type].flags;
	up->tx_loadsz = uart_config[up->port.type].tx_loadsz;

	if (up->port.type == PORT_UNKNOWN)
		goto out;

	/*
	 * Reset the UART.
	 */
	serial8250_clear_fifos(up);
	serial_in(up, UART_RX);
	serial_outp(up, UART_IER, 0);

 out:
	spin_unlock_irqrestore(&up->port.lock, flags);
	DEBUG_AUTOCONF("type=%s\n", uart_config[up->port.type].name);
}


static inline void __stop_tx(struct btuart_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}
}

static void serial8250_stop_tx(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;

	__stop_tx(up);

}

static void transmit_chars(struct btuart_port *up);

static void serial8250_start_tx(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;

	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);

	}

}

static void serial8250_stop_rx(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

static void serial8250_enable_ms(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void
receive_chars(struct btuart_port *up, unsigned int *status)
{
	struct tty_struct *tty = up->port.info->port.tty;
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;

	do {
		if (likely(lsr & UART_LSR_DR))
			ch = serial_inp(up, UART_RX);
		else
			/*
			 * Intel 82571 has a Serial Over Lan device that will
			 * set UART_LSR_BI without setting UART_LSR_DR when
			 * it receives a break. To avoid reading from the
			 * receive buffer without UART_LSR_DR bit set, we
			 * just force the read character to be 0
			 */
			ch = 0;

		flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				up->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);

ignore_char:
		lsr = serial_inp(up, UART_LSR);
	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (max_count-- > 0));
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
	*status = lsr;
}

static void transmit_chars(struct btuart_port *up)
{
	struct circ_buf *xmit = &up->port.info->xmit;
	int count;

	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		serial8250_stop_tx(&up->port);
		return;
	}
	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

//	printk("uart_circ_chars_pending=%d\n",uart_circ_chars_pending(xmit));

	count = up->tx_loadsz;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static unsigned int check_modem_status(struct btuart_port *up)
{
	unsigned int status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.info != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

		wake_up_interruptible(&up->port.info->delta_msr_wait);
	}

	return status;
}

/*
 * This handles the interrupt from one port.
 */
static inline void
serial8250_handle_port(struct btuart_port *up)
{
	unsigned int status;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);

	status = serial_inp(up, UART_LSR);

	DEBUG_INTR("status = %x...", status);

	if (status & (UART_LSR_DR | UART_LSR_BI))
		receive_chars(up, &status);
	check_modem_status(up);
	if (status & UART_LSR_THRE)
		transmit_chars(up);

	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * This is the serial driver's interrupt routine.
 */
static irqreturn_t btuart_interrupt(int irq, void *dev_id)
{
	struct irq_info *i = dev_id;
	int pass_counter = 0, handled = 0, end = 0;

	DEBUG_INTR("btuart_interrupt(%d)...", irq);

	spin_lock(&i->lock);

	do {
		struct btuart_port *up;
		unsigned int iir;

		up = (struct btuart_port *)(i->up);

		iir = serial_in(up, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			serial8250_handle_port(up);
			handled = 1;

		}
		else
			end = 1;

		if (pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk(KERN_ERR "btuart: too much work for "
				"irq%d\n", irq);
			break;
		}
	} while (end);

	spin_unlock(&i->lock);

	DEBUG_INTR("end.\n");

	return IRQ_RETVAL(handled);
}

static unsigned int serial8250_tx_empty(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;
	unsigned long flags;
	unsigned int lsr;

	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return lsr & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
}

static unsigned int serial8250_get_mctrl(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;
	unsigned int status;
	unsigned int ret;

	status = check_modem_status(up);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial8250_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct btuart_port *up = (struct btuart_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_out(up, UART_MCR, mcr);
}

static void serial8250_break_ctl(struct uart_port *port, int break_state)
{
	struct btuart_port *up = (struct btuart_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int serial8250_startup(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;
	unsigned long flags;
	unsigned char lsr, iir;
	int retval;
	int irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;

	up->capabilities = uart_config[up->port.type].flags;
	up->mcr = 0;


	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial8250_clear_fifos(up);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_inp(up, UART_LSR);
	(void) serial_inp(up, UART_RX);
	(void) serial_inp(up, UART_IIR);
	(void) serial_inp(up, UART_MSR);

	btuart_irq[0].up = up;
		retval = request_irq(up->port.irq, btuart_interrupt,
				  irq_flags, "btuart", btuart_irq);
		if (retval)
			return retval;

	/*
	 * Now, initialize the UART
	 */
	serial_outp(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl |= TIOCM_OUT2;

	serial8250_set_mctrl(&up->port, up->port.mctrl);

	/*
	 * Do a quick test to see if we receive an
	 * interrupt when we enable the TX irq.
	 */
	serial_outp(up, UART_IER, UART_IER_THRI);
	lsr = serial_in(up, UART_LSR);
	iir = serial_in(up, UART_IIR);
	serial_outp(up, UART_IER, 0);

	if (lsr & UART_LSR_TEMT && iir & UART_IIR_NO_INT) {
		if (!(up->bugs & UART_BUG_TXEN)) {
			up->bugs |= UART_BUG_TXEN;
			printk("ttyBT%d - enabling bad tx status workarounds\n",
				 port->line);
		}
	} else {
		up->bugs &= ~UART_BUG_TXEN;
	}

	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_inp(up, UART_LSR);
	serial_inp(up, UART_RX);
	serial_inp(up, UART_IIR);
	serial_inp(up, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_outp(up, UART_IER, up->ier);

	return 0;
}

static void serial8250_shutdown(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;
	unsigned long flags;

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;

	serial8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	serial8250_clear_fifos(up);


	(void) serial_in(up, UART_RX);

		free_irq(up->port.irq, btuart_irq);
}

static unsigned int serial8250_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

		quot = uart_get_divisor(port, baud);

	return quot;
}

static void
serial8250_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct btuart_port *up = (struct btuart_port *)port;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = serial8250_get_divisor(port, baud);

	if (up->capabilities & UART_CAP_FIFO && up->port.fifosize > 1) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = uart_config[up->port.type].fcr;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;

	serial_out(up, UART_IER, up->ier);


	serial_outp(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */

	serial_dl_write(up, quot);

	/*
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */

	serial_outp(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		}
		serial_outp(up, UART_FCR, fcr);		/* set fcr */
	serial8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}

static void
serial8250_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct btuart_port *p = (struct btuart_port *)port;

	if (p->pm)
		p->pm(port, state, oldstate);
}

/*
 * Resource handling.
 */
static int serial8250_request_std_resource(struct btuart_port *up)
{
	unsigned int size = 8 << up->port.regshift;
	int ret = 0;

		if (!up->port.mapbase)
			return ret;

		if (!request_mem_region(up->port.mapbase, size, "btuart")) {
			ret = -EBUSY;
			return ret;
		}

		if (up->port.flags & UPF_IOREMAP) {
			up->port.membase = ioremap_nocache(up->port.mapbase,
									size);
			if (!up->port.membase) {
				release_mem_region(up->port.mapbase, size);
				ret = -ENOMEM;
				return ret;
			}
		}
	return ret;
}

static void serial8250_release_std_resource(struct btuart_port *up)
{
	unsigned int size = 8 << up->port.regshift;

		if (!up->port.mapbase)
			return;

		if (up->port.flags & UPF_IOREMAP) {
			iounmap(up->port.membase);
			up->port.membase = NULL;
		}

		release_mem_region(up->port.mapbase, size);
}


static void serial8250_release_port(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;

	serial8250_release_std_resource(up);
}

static int serial8250_request_port(struct uart_port *port)
{
	struct btuart_port *up = (struct btuart_port *)port;
	int ret = 0;

	ret = serial8250_request_std_resource(up);
	if (ret == 0 )
			serial8250_release_std_resource(up);

	return ret;
}

static void serial8250_config_port(struct uart_port *port, int flags)
{
	struct btuart_port *up = (struct btuart_port *)port;
	int probeflags = PROBE_ANY;
	int ret;

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = serial8250_request_std_resource(up);
	if (ret < 0)
		return;

	if (flags & UART_CONFIG_TYPE)
		autoconfig(up, probeflags);

	if (up->port.type == PORT_UNKNOWN)
		serial8250_release_std_resource(up);
}

static int
serial8250_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

static const char *
serial8250_type(struct uart_port *port)
{
	int type = port->type;

	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

static struct uart_ops serial8250_pops = {
	.tx_empty	= serial8250_tx_empty,
	.set_mctrl	= serial8250_set_mctrl,
	.get_mctrl	= serial8250_get_mctrl,
	.stop_tx	= serial8250_stop_tx,
	.start_tx	= serial8250_start_tx,
	.stop_rx	= serial8250_stop_rx,
	.enable_ms	= serial8250_enable_ms,
	.break_ctl	= serial8250_break_ctl,
	.startup	= serial8250_startup,
	.shutdown	= serial8250_shutdown,
	.set_termios	= serial8250_set_termios,
	.pm		= serial8250_pm,
	.type		= serial8250_type,
	.release_port	= serial8250_release_port,
	.request_port	= serial8250_request_port,
	.config_port	= serial8250_config_port,
	.verify_port	= serial8250_verify_port,
};

static struct btuart_port btuart_ports[UART_NR];

static void __init serial8250_isa_init_ports(void)
{
	static int first = 1;
	int i;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < nr_uarts; i++) {
		struct btuart_port *up = &btuart_ports[i];

		up->port.line = i;
		spin_lock_init(&up->port.lock);

		/*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		up->mcr_mask = ~ALPHA_KLUDGE_MCR;
		up->mcr_force = ALPHA_KLUDGE_MCR;

		up->port.ops = &serial8250_pops;
	}

}

static void __init
serial8250_register_ports(struct uart_driver *drv, struct device *dev)
{
	int i;

	serial8250_isa_init_ports();

	for (i = 0; i < nr_uarts; i++) {
		struct btuart_port *up = &btuart_ports[i];

		up->port.dev = dev;
		uart_add_one_port(drv, &up->port);
	}
}

#define SERIAL8250_CONSOLE	NULL

static struct uart_driver serial8250_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "btuart",
	.dev_name		= "ttyBT",
#if 0
	.major			= TTY_MAJOR,
	.minor			= 64,
#else
	.major			= 204, // like atmel_serial
	.minor			= 154,
#endif
	.nr			= UART_NR,
	.cons			= SERIAL8250_CONSOLE,
};


#if 0
/**
 *	serial8250_suspend_port - suspend one serial port
 *	@line:  serial line number
 *
 *	Suspend one serial port.
 */
void serial8250_suspend_port(int line)
{
	uart_suspend_port(&serial8250_reg, &btuart_ports[line].port);
}

/**
 *	serial8250_resume_port - resume one serial port
 *	@line:  serial line number
 *
 *	Resume one serial port.
 */
void serial8250_resume_port(int line)
{
	struct btuart_port *up = &btuart_ports[line];

	uart_resume_port(&serial8250_reg, &up->port);
}
#endif

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int __devinit serial8250_probe(struct platform_device *dev)
{
	struct plat_serial8250_port *p = dev->dev.platform_data;
	struct uart_port port;
	int ret, i;

	memset(&port, 0, sizeof(struct uart_port));

	for (i = 0; p && p->flags != 0; p++, i++) {
		port.iobase		= p->iobase;
		port.membase		= p->membase;
		port.irq		= p->irq;
		port.uartclk		= p->uartclk;
		port.regshift		= p->regshift;
		port.iotype		= p->iotype;
		port.flags		= p->flags;
		port.mapbase		= p->mapbase;
		port.hub6		= p->hub6;
		port.private_data	= p->private_data;
		port.dev		= &dev->dev;
		if (share_irqs)
			port.flags |= UPF_SHARE_IRQ;
		ret = btuart_register_port(&port);
		if (ret < 0) {
			dev_err(&dev->dev, "unable to register port at index %d "
				"(IO%lx MEM%llx IRQ%d): %d\n", i,
				p->iobase, (unsigned long long)p->mapbase,
				p->irq, ret);
		}
	}
	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int __devexit serial8250_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct btuart_port *up = &btuart_ports[i];

		if (up->port.dev == &dev->dev)
			btuart_unregister_port(i);
	}
	return 0;
}

static int serial8250_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct btuart_port *up = &btuart_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			uart_suspend_port(&serial8250_reg, &up->port);
	}

	return 0;
}

static int serial8250_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct btuart_port *up = &btuart_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			serial8250_resume_port(i);
	}

	return 0;
}

static struct platform_driver serial8250_bluetooth_driver = {
	.probe		= serial8250_probe,
	.remove		= __devexit_p(serial8250_remove),
	.suspend	= serial8250_suspend,
	.resume		= serial8250_resume,
	.driver		= {
		.name	= "btuart",
		.owner	= THIS_MODULE,
	},
};

/*
 * This "device" covers _all_ ISA 8250-compatible serial devices listed
 * in the table in include/asm/serial.h
 */
static struct platform_device *serial8250_isa_devs;

/*
 * serial8250_register_port and serial8250_unregister_port allows for
 * 16x50 serial ports to be configured at run-time, to support PCMCIA
 * modems and PCI multiport cards.
 */
static DEFINE_MUTEX(btuart_mutex);

static struct btuart_port *serial8250_find_match_or_unused(struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (uart_match_port(&btuart_ports[i].port, port))
			return &btuart_ports[i];

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < nr_uarts; i++)
		if (btuart_ports[i].port.type == PORT_UNKNOWN &&
		    btuart_ports[i].port.iobase == 0)
			return &btuart_ports[i];

	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (btuart_ports[i].port.type == PORT_UNKNOWN)
			return &btuart_ports[i];

	return NULL;
}

/**
 *	serial8250_register_port - register a serial port
 *	@port: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int btuart_register_port(struct uart_port *port)
{
	struct btuart_port *uart;
	int ret = -ENOSPC;

	if (port->uartclk == 0)
		return -EINVAL;

	mutex_lock(&btuart_mutex);

	uart = serial8250_find_match_or_unused(port);
	if (uart) {
		uart_remove_one_port(&serial8250_reg, &uart->port);

		uart->port.iobase       = port->iobase;
		uart->port.membase      = port->membase;
		uart->port.irq          = port->irq;
		uart->port.uartclk      = port->uartclk;
		uart->port.fifosize     = port->fifosize;
		uart->port.regshift     = port->regshift;
		uart->port.iotype       = port->iotype;
		uart->port.flags        = port->flags | UPF_BOOT_AUTOCONF;
		uart->port.mapbase      = port->mapbase;
		uart->port.private_data = port->private_data;
		if (port->dev)
			uart->port.dev = port->dev;

		ret = uart_add_one_port(&serial8250_reg, &uart->port);
		if (ret == 0)
			ret = uart->port.line;
	}
	mutex_unlock(&btuart_mutex);

	return ret;
}
EXPORT_SYMBOL(btuart_register_port);

/**
 *	serial8250_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void btuart_unregister_port(int line)
{
	struct btuart_port *uart = &btuart_ports[line];

	mutex_lock(&btuart_mutex);
	uart_remove_one_port(&serial8250_reg, &uart->port);
	if (serial8250_isa_devs) {
		uart->port.flags &= ~UPF_BOOT_AUTOCONF;
		uart->port.type = PORT_UNKNOWN;
		uart->port.dev = &serial8250_isa_devs->dev;
		uart_add_one_port(&serial8250_reg, &uart->port);
	} else {
		uart->port.dev = NULL;
	}
	mutex_unlock(&btuart_mutex);
}
EXPORT_SYMBOL(btuart_unregister_port);

static int __init serial8250_init(void)
{
	int ret;

	if (nr_uarts > UART_NR)
		nr_uarts = UART_NR;

	printk(KERN_INFO "btuart: bluetooth UART driver "
		"%d ports, IRQ sharing %sabled\n", nr_uarts,
		share_irqs ? "en" : "dis");

		spin_lock_init(&btuart_irq[0].lock);

	ret = uart_register_driver(&serial8250_reg);
	if (ret)
		goto out;

	serial8250_isa_devs = platform_device_alloc("btuart",
						    PLAT8250_DEV_LEGACY);
	if (!serial8250_isa_devs) {
		ret = -ENOMEM;
		goto unreg_uart_drv;
	}

	ret = platform_device_add(serial8250_isa_devs);
	if (ret)
		goto put_dev;

	serial8250_register_ports(&serial8250_reg, &serial8250_isa_devs->dev);

	ret = platform_driver_register(&serial8250_bluetooth_driver);
	if (ret == 0)
		goto out;

	platform_device_del(serial8250_isa_devs);
 put_dev:
	platform_device_put(serial8250_isa_devs);
 unreg_uart_drv:
	uart_unregister_driver(&serial8250_reg);
 out:
	return ret;
}

static void __exit serial8250_exit(void)
{
	struct platform_device *isa_dev = serial8250_isa_devs;

	/*
	 * This tells serial8250_unregister_port() not to re-register
	 * the ports (thereby making serial8250_bluetooth_driver permanently
	 * in use.)
	 */
	serial8250_isa_devs = NULL;

	platform_driver_unregister(&serial8250_bluetooth_driver);
	platform_device_unregister(isa_dev);

	uart_unregister_driver(&serial8250_reg);
}

module_init(serial8250_init);
module_exit(serial8250_exit);

#if 0
EXPORT_SYMBOL(serial8250_suspend_port);
EXPORT_SYMBOL(serial8250_resume_port);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Bluetooth serial driver");


MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
