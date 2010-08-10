#ifndef __I2S_REG_H_INCLUDED
#define __I2S_REG_H_INCLUDED

#if defined(CONFIG_ARCH_P7DK) ||defined(CONFIG_ARCH_PDK_PC7210) || defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
#define FIFO_DEPTH 8
#define PCM_BURST_TYPE SOCLE_DMA_BURST_INCR4
#elif defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_PDK_PC9002) || defined(CONFIG_ARCH_SCDK) || defined(CONFIG_ARCH_MSMV)
#define FIFO_DEPTH 32
#define PCM_BURST_TYPE SOCLE_DMA_BURST_INCR16
#endif

/*
 *  Registers
 *  */
#define SOCLE_I2S_OPR 				0x00 	/* I2S version control and operation start register */
#define SOCLE_I2S_TXR 				0x04	/* I2S transmitter FIFO input */
#define SOCLE_I2S_RXR 				0x08	/* I2S receiver FIFO output */
#define SOCLE_I2S_TXCTL 			0x0C	/* I2S transmitter control register */
#define SOCLE_I2S_RXCTL 			0x10	/* I2S receiver control register */
#define SOCLE_I2S_FIFOSTS 			0x14 /* I2S transmit and receive FIFO control register */
#define SOCLE_I2S_IER 				0x18	/* I2S interrupt enable/mask register */
#define SOCLE_I2S_ISR 				0x1C	/* I2S interrupt status register */

/*
 *  SOCLE_I2S_ORP
 *  */

#define SOCLE_I2S_VER_MASK 			(0xFF<<24)
#define SOCLE_I2S_VER_SH 				(24)
#define SOCLE_I2S_VER_MAJOR_MASK 		(0xF<<OPR_I2S_VER_MAJOR_SH)
#define SOCLE_I2S_VER_MINOR_MASK 		(0xF<<OPR_I2S_VER_MINOR_SH)
#define SOCLE_I2S_VER_MAJOR_SH 		(28)
#define SOCLE_I2S_VER_MINOR_SH 		(24)

/* Reset Tx logic
 * Writing to this bit will reset Tx logic and its FSM*/
#define SOCLE_I2S_TX_N_RST (0x0) /* don't reset Tx logic*/
#define SOCLE_I2S_TX_RST (0x1 << 17) /* Reset tx logic */

/* Reset Rx logic
 * Writing to this bit will reset Rx logic and its FSM*/
#define SOCLE_I2S_RX_N_RST (0x0) /* don't reset Rx logic */
#define SOCLE_I2S_RX_RST (0x1 << 16) /* reset rx logic */

/* HDMA REQ1 Disable */
#define SOCLE_I2S_HDMA_REQ_1_DIS (0x1 << 6) /* disable HDMA req1 */
#define SOCLE_I2S_HDMA_REQ_1_EN (0x0) /* enable HDMA req1 */

/* HDMA REQ2 Disable */
#define SOCLE_I2S_HDMA_REQ_2_DIS (0x1 << 5) /* disable HDMA req2 */
#define SOCLE_I2S_HDMA_REQ_2_EN (0x0) /* enable HDMA req2 */

/* HDMA REQ1 CH
 * This bit is to indicate the hardware DMA IF1 is used for which FIFO*/
#define SOCLE_I2S_HDMA_IF_1_TX	(0x0) /* used for tx FIFO*/
#define SOCLE_I2S_HDMA_IF_1_RX	(0x1 << 4) /* used for rx FIFO */

/* HDMA REQ2 CH
 * This bit is to indicate the hardware DMA IF2 is used for which FIFO*/
#define SOCLE_I2S_HDMA_IF_2_TX	(0x0) /* used for tx FIFO */
#define SOCLE_I2S_HDMA_IF_2_RX	(0x1 << 3) /* used for rx FIFO */

/* I2S loop-back mode
 * This bit is to indicate the operation mode of the I2S controller is in a
 * normal operation mode or in a loop-back mode*/
#define SOCLE_I2S_OP_NORMAL (0x0) /* normal operation mdoe */
#define SOCLE_I2S_OP_LOOPBACK (0x1 << 2) /* loop-back mode */

/* I2S transmit-operation start
 * The transmitter starts to send SLCK and LRCK signals and transmit data
 * stored in the Tx FIFO to receiver after this bit is set to 1 (Transmitter acts as a master*/
#define SOCLE_I2S_TX_OP_STP (0x0) /* transmitter stop */
#define SOCLE_I2S_TX_OP_STR (0x1 << 1) /* transmitter start */

/* I2S receive-operation start
 * The receiver starts to send SCLK and LRCK signals and receive data
 * from transmitter after this bit is set to 1 (Receiver acts as a master)*/
#define SOCLE_I2S_RX_OP_STP (0x0) /* receiver stop */
#define SOCLE_I2S_RX_OP_STR 0x1 /* transmitter start */

/*
 *  SOCLE_TXCTL
 *  */
/* Transmitter devices select
 * This value is used to select which device of the
 * transmitter is active now (including loop-back mode)*/
#define SOCLE_I2S_TX_DEV_SEL_0 (0x0) /* device 0 */
#define SOCLE_I2S_TX_DEV_SEL_1 (0x1 << 18) /* device 1 */
#define SOCLE_I2S_TX_DEV_SEL_2 (0x2 << 18) /* device 2 */
#define SOCLE_I2S_TX_DEV_SEL_3 (0x3 << 18) /* device 3 */
#define SOCLE_I2S_TX_DEV_SEL_MASK (0x3 << 18)

/* Socle IP not support RATIO set at odd mode ryan add 20080411
UDA1342 (33.8688MHz) (44.1KHz) 
33868800 / 44100 = 768fs
768fs= (oversampling rate) 128fs * (Ratio bit) 6
MOSA (16.9344MHz) (44.1KHz)
16934400 / 44100 = 384fs
384fs= (oversampling rate) 128fs * (Ratio bit) 3 (X)
384fs= (oversampling rate) 64fs * (Ratio bit) 6 (V)
*/

#define SOCLE_I2S_768FS			(SOCLE_I2S_RATIO(6) | SOCLE_I2S_OVERSAMPLING_RATE_128)
#define SOCLE_I2S_384FS			(SOCLE_I2S_RATIO(6) | SOCLE_I2S_OVERSAMPLING_RATE_64)


/* Oversampling rate select bits */
#define SOCLE_I2S_OVERSAMPLING_RATE_32 (0x0) /* 32fs */
#define SOCLE_I2S_OVERSAMPLING_RATE_64 (0x1 << 16) /* 64fs */
#define SOCLE_I2S_OVERSAMPLING_RATE_128 (0x2 << 16) /* 128fs */
#define SOCLE_I2S_OVERSAMPLING_RATE_MASK (0x3 << 16)

/* Ratio bits
 * (MCLK / Ratio) = oversampling rate = SCLK frequency*/
#define SOCLE_I2S_RATIO(x) (((x) & 0xFF) << 8)
#define SOCLE_I2S_RATIO_MASK (0xFF << 8)

/* Sampling data resolution
 * Number of bits that are transmitted from each audio word (20 and 24 bits is not available now)*/
#define SOCLE_I2S_SAMPLE_RES_8 (0x0) /* 8 bits */
#define SOCLE_I2S_SAMPLE_RES_16 (0x1 << 4) /* 16 bits */
#define SOCLE_I2S_SAMPLE_RES_20 (0x2 << 4) /* 20 bits */
#define SOCLE_I2S_SAMPLE_RES_24 (0x3 << 4) /* 24 bits */
#define SOCLE_I2S_SAMPLE_RES_MASK (0x3 << 4)

/* Mono/Stereo mode
 * When the bit is set to 1, transmitter is at Mono mode, and data output from left channel,
 * Default is stereo mode*/
#define SOCLE_I2S_STEREO (0x0)	/* stereo */
#define SOCLE_I2S_MONO (0x1 << 3) /* mono */

/* Bus interface mode
 * Choose the type of the bus interface*/
#define SOCLE_I2S_BUS_IF_I2S (0x0) /* I2S */
#define SOCLE_I2S_BUS_IF_LJ (0x1 << 1) /* left-justified (LSB first) */
#define SOCLE_I2S_BUS_IF_RJ (0x2 << 1) /* right-justified (MSB first) */
#define SOCLE_I2S_BUS_IF_MASK (0x3 << 1)

/*Master/Slave mode select
 * This bit decides the transmitter acts as a master or slave*/
#define SOCLE_I2S_MASTER 0x1	/* master mode */
#define SOCLE_I2S_SLAVE 0x0	/* slave mode */

/*
 *  SOCLE_I2S RXCTL
 *  */
/* Clear Rx FIFO bit
 * Writing a '1' to this bit clears the receiver FIFO
 * and resets its logic. But it doesn't clear the shift
 * register, i.e. receiving of the current character continues*/
#define SOCLE_I2S_RX_FIFO_N_CLR	0x0 /* don't clear */
#define SOCLE_I2S_RX_FIFO_CLR (0x1 << 24) /* clear */

/*
 *  SOCLE_I2S_FIFOSTS
 *  */
/* Tx interrupt trigger level */
#define SOCLE_I2S_TX_INT_TRIG_LEV_ALMOST_EMPTY (0x0) /* almost empty */
#define SOCLE_I2S_TX_INT_TRIG_LEV_HALF_FULL (0x1 << 18)	/* half full */
#define SOCLE_I2S_TX_INT_TRIG_LEV_ALMOST_FULL (0x2 << 18) /* almost full */
#define SOCLE_I2S_TX_INT_TRIG_LEV_MASK (0x3 << 18)

/* Rx interrupt trigger level */
#define SOCLE_I2S_RX_INT_TRIG_LEV_ALMOST_EMPTY (0x0) /* almost empty */
#define SOCLE_I2S_RX_INT_TRIG_LEV_HALF_FULL (0x1 << 16)	/* half full */
#define SOCLE_I2S_RX_INT_TRIG_LEV_ALMOST_FULL (0x2 << 16) /* almost full */
#define SOCLE_I2S_RX_INT_TRIG_DEV_MASK (0x3 << 16)

/* Tx FIFO half full flag */
#define SOCLE_I2S_TX_FIFO_HALF_FULL (0x1 << 9) /* this bit is set whenever Tx FIFO is half full */

/* Rx FIFO half full flag */
#define SOCLE_I2S_RX_FIFO_HALF_FULL (0x1 << 8) /* this bit is set whenever Rx FIFO is half full */

/* Tx FIFO almost full flag */
#define SOCLE_I2S_TX_FIFO_ALMOST_FULL (0x1 << 7) /* this bit is set whenever Tx FIFO is almost full */

/* Tx FIFO almost empty flag */
#define SOCLE_I2S_TX_FIFO_ALMOST_EMPTY (0x1 << 6) /* this bit is set whenever Tx FIFO is almost empty */

/* Rx FIFO almost full flag */
#define SOCLE_I2S_RX_FIFO_ALMOST_FULL (0x1 << 5) /* this bit is set whenever Rx FIFO is almost full */

/* Rx FIFO almost empty flag */
#define SOCLE_I2S_RX_FIFO_ALMOST_EMPTY (0x1 << 4) /* this bit is set whenever Rx FIFO is almost empty */

/* Tx FIFO full flag */
#define SOCLE_I2S_TX_FIFO_FULL (0x1 << 3) /* this bit is set whenever TX FIFO is full */

/* Tx FIFO empty flag */
#define SOCLE_I2S_TX_FIFO_EMPTY (0x1 << 2) /* this bit is set whenever TX FIFO is empty */

/* RX FIFO full flag */
#define SOCLE_I2S_RX_FIFO_FULL (0x1 << 1) /* this bit is set whenver RX FIFO is full */

/* RX FIFO empty flag */
#define SOCLE_I2S_RX_FIFO_EMPTY 0x1 /* this bit is set whenever RX FIFO is empty */

/*
 *  SOCLE_I2S_IER
 *  */
/* Tx FIFO data trigger interrupt enable bit
 * This bit enables the interrupt when Tx FIFO's trigger level is reached*/
#define SOCLE_I2S_TX_FIFO_TRIG_INT_DIS (0x0) /* disable */
#define SOCLE_I2S_TX_FIFO_TRIG_INT_EN (0x1 << 2) /* enable */

/* Rx FIFO data trigger interrupt enable bit
 * This bit enable the interrupt when Rx FIFO's trigger level is reached*/
#define SOCLE_I2S_RX_FIFO_TRIG_INT_DIS (0x0) /* disable */
#define SOCLE_I2S_RX_FIFO_TRIG_INT_EN (0x1 << 1) /* enable */

/* Rx FIFO overrun interrupt enable bit
 * This bit enables the interrupt when Rx FIFO overrun condition is occurred*/
#define SOCLE_I2S_RX_FIFO_OVR_INT_DIS (0x0) /* disable */
#define SOCLE_I2S_RX_FIFO_OVR_INT_EN 0x1 /* enable */

/*
 *  SOCLE_I2S_ISR
 *  */
/* Tx FIFO data trigger interrupt */
#define SOCLE_I2S_TX_FIFO_TRIG_INT (0x1 << 2) /* this bit set when Tx FIFO's trigger level is reached
					       * and cpu wishes to keep transmitting data to the device.
					       * this bit is clear when data in Tx FIFO is above trigger level*/

/* Rx FIFO data trigger interrupt */
#define SOCLE_I2S_RX_FIFO_TRIG_INT (0x1 << 1) /* this bit is set when Rx FIFO's trigger level is reached
					       * this bit is clear when data in Rx FIFO is below trigger level*/

/* Rx FIFO overrun interrupt */
#define SOCLE_I2S_RX_FIFO_OVR_INT 0x1 /* this bit is set when Rx FIFO is full and another character has been
				       * received in the receiver shift register.
				       * if another character is starting to arrive, it will overwrite the data
				       * in the shift register but the FIFO will remain inact.
				       * this bit is cleared after Rx FIFO is cleared by software simultaneously*/

#endif
