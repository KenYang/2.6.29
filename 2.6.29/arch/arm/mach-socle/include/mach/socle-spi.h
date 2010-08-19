/* linux/include/asm/arch-socle/socle-spi.h
 *
 * Copyright (c) 2006 Socle-tech Corp
 *		      http://www.socle-tech.com.tw/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * LDK SPI configuration
*/

#ifndef __LDK_SPIREG_H
#define __LDK_SPIREG_H                     1

#include <mach/platform.h>


#define LDK_VA_SPI	IO_ADDRESS(SOCLE_SPI0)

/* Character Len */

#define	CHAR_4BIT		(3)
#define	CHAR_5BIT		(4)
#define	CHAR_6BIT		(5)
#define	CHAR_7BIT		(6)
#define	CHAR_8BIT		(7)
#define	CHAR_9BIT		(8)
#define	CHAR_10BIT		(9)
#define	CHAR_11BIT		(10)
#define	CHAR_12BIT		(11)
#define	CHAR_13BIT		(12)
#define	CHAR_14BIT		(13)
#define	CHAR_15BIT		(14)
#define	CHAR_16BIT		(15)

/* PBCA parameters */

#define	PBCA_HALF		(0)
#define	PBCA_4			(1)
#define	PBCA_8			(2)
#define	PBCA_16			(3)
#define	PBCA_32			(4)
#define	PBCA_64			(5)
#define	PBCA_128		(6)
#define	PBCA_256		(7)

/* PBCT parameters */

#define	PBCT_0			(0)
#define	PBCT_4			(1)
#define	PBCT_8			(2)
#define	PBCT_16			(3)
#define	PBCT_32			(4)
#define	PBCT_64			(5)
#define	PBCT_128		(6)
#define	PBCT_256		(7)

/* PBTXRX parameters */

#define	PBTXRX_0		(0)
#define	PBTXRX_4		(1)
#define	PBTXRX_8		(2)
#define	PBTXRX_16		(3)
#define	PBTXRX_32		(4)
#define	PBTXRX_64		(5)
#define	PBTXRX_128		(6)
#define	PBTXRX_256		(7)


/*
* ioctl commands
*/
/* soft reset, master enable, and run bit is ignored, driver handles it itself. */
#define	SPI_IOC_FWCR			_IOWR(MAJOR_SPI, 0, unsigned short)
#define	SPI_IOC_SOFT_RESET		_IOW(MAJOR_SPI, 1, unsigned int)
#define	SPI_IOC_DLYCR			_IOWR(MAJOR_SPI, 2, unsigned short)
#define	SPI_IOC_SSCR			_IOWR(MAJOR_SPI, 3, unsigned short)
#define	SPI_FIFO_RX_TRIG		_IOWR(MAJOR_SPI, 4, unsigned int)
/* legal value: 0, 1, 2 */
#define	SPI_FIFO_TX_TRIG		_IOWR(MAJOR_SPI, 5, unsigned int)
#define	SPI_FIFO_RESET			_IOW(MAJOR_SPI, 6, unsigned int)
#define	SPI_IOC_INT_ENABLE		_IOWR(MAJOR_SPI, 7, unsigned int)

#define	SPI_IOC_LOOPBACK		_IOW(MAJOR_SPI, 10, unsigned int)
#define	SPI_IOC_BIDRECTION		_IOW(MAJOR_SPI, 11, unsigned int)
#define	SPI_IOC_LSB_FIRST		_IOW(MAJOR_SPI, 12, unsigned int)
#define	SPI_IOC_CLK_PHASE_2		_IOW(MAJOR_SPI, 13, unsigned int)
#define	SPI_IOC_CLK_INACTIVE_HI		_IOW(MAJOR_SPI, 14, unsigned int)
#define	SPI_IOC_FULL_DUPLEX			_IOW(MAJOR_SPI, 15, unsigned int)
#define	SPI_IOC_CLK_IDLE_ENABLE		_IOW(MAJOR_SPI, 16, unsigned int)

#define	SPI_IOC_CLK_ACTIVE_DELAY	_IOW(MAJOR_SPI, 20, unsigned int)
#define	SPI_IOC_CLK_TRANS_DELAY		_IOW(MAJOR_SPI, 21, unsigned int)
#define	SPI_IOC_CLK_TXRX_DELAY		_IOW(MAJOR_SPI, 22, unsigned int)

/* legal value: 0 ~ 63 */
#define	SPI_IOC_CLK_RATE		_IOWR(MAJOR_SPI, 30, unsigned int)
/* legal value: 0, 1, 2 */
#define	SPI_IOC_CHAR_LEN		_IOWR(MAJOR_SPI, 31, unsigned int)

/*For ioctl define*/
// Transcation flow control
#define	LOOPBACK_ENABLE			0x0001		// Enable internal loopback mode
#define	BIDIRECTION_ENABLE		0x0002		// Enable bidirection mode
#define	LSB_FIRST_EN			0x0004		// Enable LSB first mode
#define	CLOCK_PHASE				0x0008		// Select clock phase
#define	CLOCK_POLARITY			0x0010		// Select clock polarity
#define	CONCURRENT_ENABLE		0x0020		// Tx & Rx concurrent mode enable
#define	CLOCK_IDLE_ASSERT		0x0100		// SPI clock idle enable control
#define	SPI_RUN					0x0200		// SPI transmit & receive control, set this bit will start SPI controller
#define	MASETR_ENABLE			0x0400		// Enable master mode
#define	RESET_CONTROL			0x0800		// Reset SPI controller

#define SPI_LOOPBACK_EN()       (outw(inw(O_FWCR) | LOOPBACK_ENABLE ,    O_FWCR))
#define SPI_LOOPBACK_DIS()      (outw(inw(O_FWCR) & ~LOOPBACK_ENABLE,    O_FWCR))
#define SPI_BIDIRECTION_EN()    (outw(inw(O_FWCR) | BIDIRECTION_ENABLE , O_FWCR))
#define SPI_BIDIRECTION_DIS()   (outw(inw(O_FWCR) & ~BIDIRECTION_ENABLE, O_FWCR))
#define SPI_LSBFIRST_EN()       (outw(inw(O_FWCR) | LSB_FIRST_EN ,       O_FWCR))
#define SPI_MSBFIRST_EN()       (outw(inw(O_FWCR) & ~LSB_FIRST_EN,       O_FWCR))
#define SPI_SECOND_PHASE()      (outw(inw(O_FWCR) | CLOCK_PHASE ,        O_FWCR))
#define SPI_FIRST_PHASE()       (outw(inw(O_FWCR) & ~CLOCK_PHASE,        O_FWCR))
#define SPI_LOW_IDLE()          (outw(inw(O_FWCR) | CLOCK_POLARITY ,     O_FWCR))
#define SPI_HIGH_IDLE()         (outw(inw(O_FWCR) & ~CLOCK_POLARITY,     O_FWCR))
#define SPI_CONCURRENT_EN()     (outw(inw(O_FWCR) | CONCURRENT_ENABLE ,  O_FWCR))
#define SPI_CONCURRENT_DIS()    (outw(inw(O_FWCR) & ~CONCURRENT_ENABLE,  O_FWCR))
#define SPI_CLOCK_IDLE_EN()     (outw(inw(O_FWCR) | CLOCK_IDLE_ASSERT ,  O_FWCR))
#define SPI_CLOCK_IDLE_DIS()    (outw(inw(O_FWCR) & ~CLOCK_IDLE_ASSERT,  O_FWCR))
#define SPI_START_TRANSFER()    (outw(inw(O_FWCR) | SPI_RUN ,            O_FWCR))
#define SPI_MASETR_EN()         (outw(inw(O_FWCR) | MASETR_ENABLE ,      O_FWCR))
#define SPI_MASETR_DIS()        (outw(inw(O_FWCR) & ~MASETR_ENABLE,      O_FWCR))
#define SPI_RESET()             (outw(inw(O_FWCR) | RESET_CONTROL,       O_FWCR))
#define IS_SPI_RUNNING()        (inw(O_FWCR) & SPI_RUN)

// FIFO control
#define SPI_FIFO_SIZE           8

#define SPI_TX_THRESH_2         (0<<8)
#define SPI_TX_THRESH_4         (1<<8)
#define SPI_TX_THRESH_6         (2<<8)
#define SPI_RX_THRESH_2         (0<<11)
#define SPI_RX_THRESH_4         (1<<11)
#define SPI_RX_THRESH_6         (2<<11)

#define	RECEIVE_DATA_AVAILABLE  0x0001		// when this bit is set, there at last one data in receive FIFO
#define	TRANSMIT_FIFO_FULL      0x0002		// when this bit is set, there transmit FIFO is full
#define	CLEAR_TRANSMIT          0x0004		// clear transmit FIFO
#define	CLEAR_RECEIVE           0x0008		// clear receive FIFO
#define	TRANSMIT_LEVEL_MASK     0x0700//0x0070		// transimt FIFO level mask
#define	RECEIVE_LEVEL_MASK      0x3800//0x0380		// receive FIFO level mask

#define IS_RX_DATA_AVAILABLE()  (inw(O_FCR) & RECEIVE_DATA_AVAILABLE)
#define IS_TX_FIFO_FULL()       (inw(O_FCR) & TRANSMIT_FIFO_FULL)
#define SPI_CLEAR_TX_FIFO()     (outw(inw(O_FCR) | CLEAR_TRANSMIT, O_FCR))
#define SPI_CLEAR_RX_FIFO()     (outw(inw(O_FCR) | CLEAR_RECEIVE,  O_FCR))
#define SPI_TX_FIFO_THRESH_2()  (outw((inw(O_FCR) & ~TRANSMIT_LEVEL_MASK) | SPI_TX_THRESH_2, O_FCR))
#define SPI_TX_FIFO_THRESH_4()  (outw((inw(O_FCR) & ~TRANSMIT_LEVEL_MASK) | SPI_TX_THRESH_4, O_FCR))
#define SPI_TX_FIFO_THRESH_6()  (outw((inw(O_FCR) & ~TRANSMIT_LEVEL_MASK) | SPI_TX_THRESH_6, O_FCR))
#define SPI_RX_FIFO_THRESH_2()  (outw((inw(O_FCR) & ~RECEIVE_LEVEL_MASK)  | SPI_RX_THRESH_2, O_FCR))
#define SPI_RX_FIFO_THRESH_4()  (outw((inw(O_FCR) & ~RECEIVE_LEVEL_MASK)  | SPI_RX_THRESH_4, O_FCR))
#define SPI_RX_FIFO_THRESH_6()  (outw((inw(O_FCR) & ~RECEIVE_LEVEL_MASK)  | SPI_RX_THRESH_6, O_FCR))

// Interrupt enable control
#define	TRANSFER_COMPLETE       0x0001		// transfer complete interrupt control
#define	RECEIVE_FIFO_OVERRUN    0x0002		// receive FIFO overrun interrupt control
#define	RECEIVE_DATA            0x0004		// receive FIFO level reach interrupt control
#define	TRANSMIT_DATA           0x0008		// transmit FIFO level reach interrupt control

#define	IE_RXCP                 0x0001		// transfer complete interrupt control
#define	IE_OR                   0x0002		// receive FIFO overrun interrupt control
#define	IE_RX                   0x0004		// receive FIFO level reach interrupt control
#define	IE_TX                   0x0008		// transmit FIFO level reach interrupt control

#define SPI_COMPLETE_INT_EN()   (outw(inw(O_IER) | TRANSFER_COMPLETE ,    O_IER))
#define SPI_COMPLETE_INT_DIS()  (outw(inw(O_IER) & ~TRANSFER_COMPLETE,    O_IER))
#define SPI_OVERRUN_INT_EN()    (outw(inw(O_IER) | RECEIVE_FIFO_OVERRUN , O_IER))
#define SPI_OVERRUN_INT_DIS()   (outw(inw(O_IER) & ~RECEIVE_FIFO_OVERRUN, O_IER))
#define SPI_RECEIVE_INT_EN()    (outw(inw(O_IER) | RECEIVE_DATA ,         O_IER))
#define SPI_RECEIVE_INT_DIS()   (outw(inw(O_IER) & ~RECEIVE_DATA,         O_IER))
#define SPI_TRANSMIT_INT_EN()   (outw(inw(O_IER) | TRANSMIT_DATA ,        O_IER))
#define SPI_TRANSMIT_INT_DIS()  (outw(inw(O_IER) & ~TRANSMIT_DATA,        O_IER))
#define SPI_INT_ENABLE(x)       (outw(inw(O_IER) | (x) , O_IER))
#define SPI_INT_DISABLE(x)      (outw(inw(O_IER) & ~(x), O_IER))

// Clock delay control
#define	ACTIVE_DELAY_MASK       0x0007      // Period before SPI clock active (PBCA).
#define	TRANSFER_DELAY_MASK     0x0031      // Period between two consecutive transfer (PBCT).
#define	TxRx_DELAY_MASK         0x0700      // Period between Tx and Rx transfer (PBTxRx).

#define	CLOCK_HALF              0x00        // 1/2 clock
#define	CLOCK_ZERO              0x00        // zero clock
#define	CLOCK_4                 0x01        // 4 clock
#define	CLOCK_8                 0x02        // 8 clock
#define	CLOCK_16                0x03        // 16 clock
#define	CLOCK_32                0x04        // 32 clock
#define	CLOCK_64                0x05        // 64 clock
#define	CLOCK_128               0x06        // 128 clock
#define	CLOCK_256               0x07        // 256 clock

#define ACTIVE_DELAY_HALF()     (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_HALF, O_DLYCR))
#define ACTIVE_DELAY_4()        (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_4,    O_DLYCR))
#define ACTIVE_DELAY_8()        (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_8,    O_DLYCR))
#define ACTIVE_DELAY_16()       (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_16,   O_DLYCR))
#define ACTIVE_DELAY_32()       (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_32,   O_DLYCR))
#define ACTIVE_DELAY_64()       (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_64,   O_DLYCR))
#define ACTIVE_DELAY_128()      (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_128,  O_DLYCR))
#define ACTIVE_DELAY_256()      (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | CLOCK_256,  O_DLYCR))
#define SET_ACTIVE_DELAY(x)     (outw((inw(O_DLYCR) & ~ACTIVE_DELAY_MASK) | (x),        O_DLYCR))

#define TRANSFER_DELAY_ZERO()   (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_ZERO, O_DLYCR))
#define TRANSFER_DELAY_4()      (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_4,    O_DLYCR))
#define TRANSFER_DELAY_8()      (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_8,    O_DLYCR))
#define TRANSFER_DELAY_16()     (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_16,   O_DLYCR))
#define TRANSFER_DELAY_32()     (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_32,   O_DLYCR))
#define TRANSFER_DELAY_64()     (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_64,   O_DLYCR))
#define TRANSFER_DELAY_128()    (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_128,  O_DLYCR))
#define TRANSFER_DELAY_256()    (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | CLOCK_256,  O_DLYCR))
#define SET_TRANSFER_DELAY(x)   (outw((inw(O_DLYCR) & ~TRANSFER_DELAY_MASK) | (x),        O_DLYCR))

#define TxRx_DELAY_ZERO()       (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_ZERO, O_DLYCR))
#define TxRx_DELAY_4()          (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_4,    O_DLYCR))
#define TxRx_DELAY_8()          (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_8,    O_DLYCR))
#define TxRx_DELAY_16()         (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_16,   O_DLYCR))
#define TxRx_DELAY_32()         (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_32,   O_DLYCR))
#define TxRx_DELAY_64()         (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_64,   O_DLYCR))
#define TxRx_DELAY_128()        (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_128,  O_DLYCR))
#define TxRx_DELAY_256()        (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | CLOCK_256,  O_DLYCR))
#define SET_TxRx_DELAY(x)       (outw((inw(O_DLYCR) & ~TxRx_DELAY_MASK) | (x),        O_DLYCR))

// slave select & characteristic control
#define CLOCK_DIVISOR_MASK      0x001f
#define SLAVE_SELECT_MASK       0x0700
#define CHARACTER_LENGTH_MASK   0x7800

#define	LENGTH_4BITS            0x03
#define	LENGTH_5BITS            0x04
#define	LENGTH_6BITS            0x05
#define	LENGTH_7BITS            0x06
#define	LENGTH_8BITS            0x07
#define	LENGTH_9BITS            0x08
#define	LENGTH_10BITS           0x09
#define	LENGTH_11BITS           0x0a
#define	LENGTH_12BITS           0x0b
#define	LENGTH_13BITS           0x0c
#define	LENGTH_14BITS           0x0d
#define	LENGTH_15BITS           0x0e
#define	LENGTH_16BITS           0x0f

#define SET_CLOCK_DIVISOR(x)    (outw((inw(O_SSCR) & ~CLOCK_DIVISOR_MASK) | x,         O_SSCR))
#define SPI_SELECT_SLAVE(x)     (outw((inw(O_SSCR) & ~SLAVE_SELECT_MASK)  | (x << 8),  O_SSCR))
#define SET_BIT_LENGTH(x)       (outw((inw(O_SSCR) & ~SLAVE_SELECT_MASK)  | (x << 11), O_SSCR))

// interrupt status
#define TRANSFER_COMPLETE_INT	0x01    // transfer complete interrupt
#define OVERRUN_INT             0x02    // overrun interrupt
#define RECEIVE_FIFO_INT        0x04    // receive FIFO level reach interrupt
#define TRANSMIT_FIFO_INT       0x08    // transmit FIFO level reach interrupt

#define IS_TRANSFER_COMPLETE()  (inw(O_ISR) & TRANSFER_COMPLETE_INT)
#define IS_OVERRUN()            (inw(O_ISR) & OVERRUN_INT)
#define IS_RECEVIE_FIFO()       (inw(O_ISR) & RECEIVE_FIFO_INT)
#define IS_TRANSMIT_FIFO()      (inw(O_ISR) & TRANSMIT_FIFO_INT)
#define CHECK_RECEVIE_COMPLETE(x)   ((x) & TRANSFER_COMPLETE_INT)
#define CHECK_OVERRUN(x)            ((x) & OVERRUN_INT)
#define CHECK_RECEVIE_FIFO(x)       ((x) & RECEIVE_FIFO_INT)
#define CHECK_TRANSMIT_FIFO(x)      ((x) & TRANSMIT_FIFO_INT)

// data read & write control
#define SPI_WRITE_DATA(x)       (outw((x), O_TXR))
#define SPI_READ_DATA()         (inw(O_RXR))

// Transmit & Receive data count setting
#define SET_TRANSMIT_COUNT(x)   (outw((x), O_TXCR))
#define SET_RECEIVE_COUNT(x)    (outw((x), O_RXCR))
#define GET_TRANSMIT_COUNT(x)   (inw(O_TXCR))
#define GET_RECEIVE_COUNT(x)    (inw(O_RXCR))

// define
#define	SPI_MSB_FIRST				0
#define	SPI_LSB_FIRST				1

#define	SPI_SLAVE				0
#define	SPI_MASTER				1

#define	UNIDIRECTION			0
#define	BIDIRECTION				1

#define	FIRST_SCLK_EDGE			0
#define	SECOND_SCLK_EDGE		1

#define	LOW_IDLE				0
#define	HIGH_IDLE				1

#define	CLOCK_NOT_ASSERT		0
#define	CLOCK_ASSERT			1

#define	TX_RX_NO_CONCURRENT		0
#define	TX_RX_CONCURRENT		1


// AMTEL AT250x0 SPI EEPROM define

#define	MAX_PACKAGE_LENGTH	8
#define	AT25040_MAX_BYTE	256

// status mask
#define AT250x0_READY_MASK	0x0001	// ready mask
#define AT250x0_WREN_MASK	0x0002	// write enable mask
#define AT250x0_PROT_MASK	0x000c	// protect level mask
//#define AT250x0_ADDR8_MASK	0x0100	// address bit 8 mask
#define AT250x0_ADDR8_MASK	0x00	// address bit 8 mask

// instruction define
#define AT250x0_WREN	0x06		// set write enable latch
#define AT250x0_WRDI	0x04		// reset write disable latch
#define AT250x0_RDSR	0x05		// read status regisgter
#define AT250x0_WRSR	0x01		// write status register
#define AT250x0_READ	0x03		// read data from memory array
#define AT250x0_WRITE	0x02		// write data to memory array

#define AT250x0_IS_MASK	0x07		// instruction bits mask
#endif

