/* linux/include/asm/arch-socle/socle-i2c.h
 *
 * Copyright (c) 2006 Socle-tech Corp
 *		      http://www.socle-tech.com.tw/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * LDK SCU configuration
*/

#ifndef __LDK_I2CREG_H
#define __LDK_I2CREG_H                     1

#include <mach/platform.h>

#ifdef LDK_I2C_DEBUG
#define I2CDEBUG(fmt, args...) printk( KERN_DEBUG "LDK_I2C : " fmt, ## args)
#else
#define I2CDEBUG(fmt, args...) \
        do { } while (0)
#endif

struct socle_i2c_reg {
	unsigned long i2c_ier;
	unsigned long i2c_opr;
};
struct i2c_algo_socle_data
{
        void (*write_byte) (u8 value);
        u8   (*read_byte) (void);
        void (*start) (void);
        void (*repeat_start) (void);
        void (*stop) (void);
        void (*abort) (void);
        int  (*wait_bus_not_busy) (void);
        int  (*wait_for_interrupt) (int wait_type);
        void (*transfer) (int lastbyte, int receive, int midbyte);
        void (*reset) (void);

	int udelay;
	int timeout;
};

#define LDK_VA_I2C	IO_ADDRESS(SOCLE_I2C0)

//#define DEF_TIMEOUT             (5)
#define DEF_TIMEOUT             (12)	//for low speed dk
#define BUS_ERROR               (-EREMOTEIO)
#define ACK_DELAY               0       /* time to delay before checking bus error */
#define MAX_MESSAGES            65536   /* maximum number of messages to send */

//#define I2C_SLEEP_TIMEOUT       (12)       /* time to sleep for on i2c transactions */
//#define I2C_SLEEP_TIMEOUT       (2)       /* time to sleep for on i2c transactions */
#define I2C_SLEEP_TIMEOUT       (8)       /* time to sleep for on i2c transactions */
#define I2C_RETRY               (-2000) /* an error has occurred retry transmit */
#define I2C_TRANSMIT		1
#define I2C_RECEIVE		0
#define I2C_MTX_GET_NAK             0xf0
#define I2C_MTX_GET_ACK             0xf1
//#define I2C_SOCLE_SLAVE_ADDR      0x1    /* slave socle unit address */

#define I2C_ICR_INIT            (ICR_BEIE | ICR_IRFIE | ICR_ITEIE | ICR_GCD | ICR_SCLE) /* ICR initialization value */
/* ICR initialize bit values 
*                       
*  15. FM       0 (100 Khz operation)
*  14. UR       0 (No unit reset)
*  13. SADIE    0 (Disables the unit from interrupting on slave addresses 
*                                       matching its slave address)
*  12. ALDIE    0 (Disables the unit from interrupt when it loses arbitration 
*                                       in master mode)
*  11. SSDIE    0 (Disables interrupts from a slave stop detected, in slave mode)  
*  10. BEIE     1 (Enable interrupts from detected bus errors, no ACK sent)
*  9.  IRFIE    1 (Enable interrupts from full buffer received)
*  8.  ITEIE    1 (Enables the I2C unit to interrupt when transmit buffer empty)
*  7.  GCD      1 (Disables i2c unit response to general call messages as a slave) 
*  6.  IUE      0 (Disable unit until we change settings)
*  5.  SCLE     1 (Enables the i2c clock output for master mode (drives SCL)   
*  4.  MA       0 (Only send stop with the ICR stop bit)
*  3.  TB       0 (We are not transmitting a byte initially)
*  2.  ACKNAK   0 (Send an ACK after the unit receives a byte)
*  1.  STOP     0 (Do not send a STOP)
*  0.  START    0 (Do not send a START)
*
*/

#define I2C_ISR_INIT            0x7FF  /* status register init */
/* I2C status register init values 
 *
 * 10. BED      1 (Clear bus error detected)
 * 9.  SAD      1 (Clear slave address detected)
 * 7.  IRF      1 (Clear IDBR Receive Full)
 * 6.  ITE      1 (Clear IDBR Transmit Empty)
 * 5.  ALD      1 (Clear Arbitration Loss Detected)
 * 4.  SSD      1 (Clear Slave Stop Detected)
 */
#define I2C_MTXR  (LDK_VA_I2C+0x00)
#define I2C_MRXR  (LDK_VA_I2C+0x04)
#define I2C_STXR  (LDK_VA_I2C+0x08)
#define I2C_SRXR  (LDK_VA_I2C+0x0c)
#define I2C_SADDR (LDK_VA_I2C+0x10)
#define I2C_IER   (LDK_VA_I2C+0x14)
#define I2C_ISR   (LDK_VA_I2C+0x18)
#define I2C_LCMR  (LDK_VA_I2C+0x1c)
#define I2C_LSR   (LDK_VA_I2C+0x20)
#define I2C_CONR  (LDK_VA_I2C+0x24)
#define I2C_OPR   (LDK_VA_I2C+0x28)

#define IER_MTX_ACKNAK (1<<0)
#define IER_MRX_ACKNAK (1<<1)
#define ISR_MTX_ACK 0x1
#define ISR_MRX_ACK 0x2

#define LCMR_START	0x1
#define LCMR_STOP	0x2
#define LCMR_RESUME	0x4

#define LSR_BUSY   0x1
#define LSR_RCV_NAK   0x2

#define CONR_NAK   (1<<4)

#define CONR_SLAVE_EN   (1<<0)
#define CONR_STX	(1<<1)
#define CONR_MASTER_EN	(1<<2)
#define CONR_MTX	(1<<3)


//#define OPR_DIVISOR_INIT	(7<<0)
//#define OPR_DIVISOR_INIT	(63<<0)
//#define OPR_DIVISOR_INIT	0x3c  //16kHz
#define OPR_DIVISOR_INIT	0x3f  //16kHz for TI decoder initial 
#define OPR_ENABLE_I2C		(1<<6)
#define OPR_SM_RESET		(1<<7)
#define OPR_10bit		(1<<8)
// ian add 2005/9/23 for control registersm
//I2C SADDR
#define I2C_SLAVE_ADDRESS		0xe0
//I2C IER
#define I2C_SET_IER				0xe1
//I2C LCMR
#define I2C_START_GEN			0xe2
#define I2C_STOP_GEN			0xe3
#define I2C_RESUME_GEN			0xe4
//I2C CONR
#define I2C_SLAVE_ENABLE		0xe5
#define I2C_SLAVE_RXTX_SEL		0xe6
#define I2C_MASTER_ENABLE		0xe7
#define I2C_MASTER_RXTX_SEL		0xe8
#define I2C_ACK_ENABLE			0xe9
//I2C OPR
#define I2C_SLAVE_ADDRESS_MOD	0xea
#define I2C_STATE_MACHINE_RESET 0xeb
#define I2C_CORE_ENABLE			0xec
#define I2C_CLOCK_DIV			0xed


#endif

