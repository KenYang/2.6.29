#ifndef __PANTHER7_HDMA_REGS_H_INCLUDED
#define __PANTHER7_HDMA_REGS_H_INCLUDED
/*
 *  Register for HDMA
 *  */
#define PANTHER7_HDMA_CON0 0x0000 /* HDMA channel 0 control register */
#define PANTHER7_HDMA_CON1 0x0004 /* HDMA channel 1 control register */
#define PANTHER7_HDMA_ISRC0 0x0008 /* HDMA channel 0 initial source address register */
#define PANTHER7_HDMA_IDST0 0x000C /* HDMA channel 0 initial destination address register */
#define PANTHER7_HDMA_ICNT0 0x0010 /* HDMA channel 0 initial terminate count register */
#define PANTHER7_HDMA_ISRC1 0x0014 /* HDMA channel 1 initial source address register */
#define PANTHER7_HDMA_IDST1 0x0018 /* HDMA channel 1 initial destination address register */
#define PANTHER7_HDMA_ICNT1 0x001C /* HDMA channel 1 initial terminate count register */
#define PANTHER7_HDMA_CSRC0 0x0020 /* HDMA channel 0 current source address register */
#define PANTHER7_HDMA_CDST0 0x0024 /* HDMA channel 0 current destination address register */
#define PANTHER7_HDMA_CCNT0 0x0028 /* HDMA channel 0 current count register */
#define PANTHER7_HDMA_CSRC1 0x002C /* HDMA channel 1 current source address register */
#define PANTHER7_HDMA_CDST1 0x0030 /* HDMA channel 1 current destination address register */
#define PANTHER7_HDMA_CCNT1 0x0034 /* HDMA channel 1current count register */
#define PANTHER7_HDMA_ISR 0x0038 /* HDMA DMA interrupt ch0/1 status register */
#define PANTHER7_HDMA_DSR 0x003C /* HDMA DMA ch0/1 status register */
#define PANTHER7_HDMA_ISCNT0 0x0040 /* HDMA channel 0 initial slice count register */
#define PANTHER7_HDMA_IPNCNT0 0x0044 /* HDMA channel 0 initial page number count down */
#define PANTHER7_HDMA_IADDR_BS0 0x0048 /* HDMA channel 0 initial address buffer size register */
#define PANTHER7_HDMA_ISCNT1 0x004C /* HDMA channel 1 initial slice count register */
#define PANTHER7_HDMA_IPNCNT1 0x0050 /* HDMA channel 1 initial page number count down */
#define PANTHER7_HDMA_IADDR_BS1 0x0054 /* HDMA channel 1 initial address buffer size register */
#define PANTHER7_HDMA_CSCNT0 0x0058 /* HDMA channel 0 current slice count register */
#define PANTHER7_HDMA_CPNCNTD0 0x005C /* HDMA channel 0 current page number count down register */
#define PANTHER7_HDMA_CADDR_BS0 0x0060 /* HDMA channel 0 current address buffer size register */
#define PANTHER7_HDMA_CSCNT1 0x0064 /* HDMA channel 1 current slice count register */
#define PANTHER7_HDMA_CPNCNTD1 0x0068 /* HDMA channel 1 current page number count down register */
#define PANTHER7_HDMA_CADDR_BS1 0x006C /* HDMA channel 1 current address buffer size register */
#define PANTHER7_HDMA_PACNT0 0x0070 /* HDMA channel 0 page accumulation count register */
#define PANTHER7_HDMA_PACNT1 0x0074 /* HDMA channel 1 page accumulation count register */
#define PANTHER7_HDMA_PRIO 0x0078 /* HDMA arbitration priority mode */

/*
 *  PANTHER7_HDMA_CONx
 *  */
/* Clear CPNCNTDx register */
#define PANTHER7_HDMA_CPNCNTD_NCLR 0x0 /* not clear */
#define PANTHER7_HDMA_CPNCNTD_CLR (0x1 << 23) /* clear */

/* Hardware HDMA turnaround enable/disable */
#define PANTHER7_HDMA_SLICE_MODE_DIS 0x0 /* disable */
#define PANTHER7_HDMA_SLICE_MODE_EN (0x1 << 22)	/* enable */

/* HDMA channel enable/disable */
#define PANTHER7_HDMA_CH_DIS 0x0 /* disable */
#define PANTHER7_HDMA_CH_EN (0x1 << 21) /* enable */

/* Interrupt Mode Set */
#define PANTHER7_HDMA_INT_MODE_POLLING 0x0 /* polling mode */
#define PANTHER7_HDMA_INT_MODE_INTERRUPT (0x1 << 18) /* interrupt mode */

/* On the fly mode */
#define PANTHER7_HDMA_FLY_DIS 0x0 /* disable on the fly */
#define PANTHER7_HDMA_FLY_READ (0x1 << 16) /* Read on the fly(read from OFM bus) */
#define PANTHER7_HDMA_FLY_WRITE (0x2 << 16) /* Write on the fly(write to OFM bus) */

/* Transfer Mode */
#define PANTHER7_HDMA_BURST_SINGLE 0x0 /* Single */
#define PANTHER7_HDMA_BURST_INCR4 (0x3 << 13) /* INCR4 */
#define PANTHER7_HDMA_BURST_INCR8 (0x5 << 13) /* INCR8 */
#define PANTHER7_HDMA_BURST_INCR16 (0x7 << 13) /* INCR16 */

/* External HDREQ source selection */
#define PANTHER7_HDMA_EXT_HDREQ_SEL(x) (((x) & 0xf) << 9)

/* Direction of source address */
#define PANTHER7_HDMA_DIR_SRC_INC 0x0 /* increment */
#define PANTHER7_HDMA_DIR_SRC_FIXED (0x1 << 7) /* fixed */

/* Direction of destination address */
#define PANTHER7_HDMA_DIR_DST_INC 0x0 /* increment */
#define PANTHER7_HDMA_DIR_DST_FIXED (0x1 << 5) /* fixed */

/* Data size for transfer */
#define PANTHER7_HDMA_DATA_BYTE 0x0 /* byte */
#define PANTHER7_HDMA_DATA_HALFWORD (0x1 << 3) /* halfword */
#define PANTHER7_HDMA_DATA_WORD (0x2 << 3) /* word */

/* Command of Software DMA operation */
#define PANTHER7_HDMA_SWDMA_OP_NO 0x0	/* no command */
#define PANTHER7_HDMA_SWDMA_OP_START (0x1 << 1) /* start software DMA operation */
#define PANTHER7_HDMA_SWDMA_OP_PAUSE (0x2 << 1) /* pause software DMA operation */
#define PANTHER7_HDMA_SWDMA_OP_CANCEL (0x3 << 1) /* cancel software DMA operation */

/* Disable / Enable hardware trigger DMA mode */
#define PANTHER7_HDMA_HWDMA_TRIGGER_DIS 0x0 /* disable */
#define PANTHER7_HDMA_HWDMA_TRIGGER_EN 0x1 /* enable */

/*
 *  PANTHER7_HDMA_ISR
 *  */
/* Mask channel 1 Page accumulation overflow Interrupt */
#define PANTHER7_HDMA_CH1_PAGE_ACCUM_OVF_INT_NMASK 0x0 /* not mask */
#define PANTHER7_HDMA_CH1_PAGE_ACCUM_OVF_INT_MASK (0x1 << 13) /* mask */

/* Mask channel 0 Page accumulation overflow Interrupt */
#define PANTHER7_HDMA_CH0_PAGE_ACCUM_OVF_INT_NMASK 0x0 /* not mask */
#define PANTHER7_HDMA_CH0_PAGE_ACCUM_OVF_INT_MASK (0x1 << 12) /* mask */

/* Mask channel 1 Page Interrupt */
#define PANTHER7_HDMA_CH1_PAGE_INT_NMASK 0x0 /* not mask */
#define PANTHER7_HDMA_CH1_PAGE_INT_MASK (0x1 << 11) /* mask */

/* Mask channel 0 Page Interrupt */
#define PANTHER7_HDMA_CH0_PAGE_INT_NMASK 0x0 /* not mask */
#define PANTHER7_HDMA_CH0_PAGE_INT_MASK (0x1 << 10) /* mask */

/* Mask channel 1 Interrupt */
#define PANTHER7_HDMA_CH1_INT_NMASK 0x0 /* not mask */
#define PANTHER7_HDMA_CH1_INT_MASK (0x1 << 9) /* mask */

/* Mask channel 0 Interrupt */
#define PANTHER7_HDMA_CH0_INT_NMASK 0x0 /* not mask */
#define PANTHER7_HDMA_CH0_INT_MASK (0x1 << 8) /* mask */

/* Channel 1 Page acuumulation counter overflow Interrupt active,
 * write "0" to clear interrupt and write "1" is no affect*/
#define PANTHER7_HDMA_CH1_PAGE_ACCUM_OVF_INT_NACT 0x0	/* not active */
#define PANTHER7_HDMA_CH1_PAGE_ACCUM_OVF_INT_ACT (0x1 << 5) /* active */

/* Channel 0 Page acuumulation counter overflow Interrupt active,
 * write "0" to clear interrupt and write "1" is no affect*/
#define PANTHER7_HDMA_CH0_PAGE_ACCUM_OVF_INT_NACT 0x0	/* not active */
#define PANTHER7_HDMA_CH0_PAGE_ACCUM_OVF_INT_ACT (0x1 << 4) /* active */

/* Channel 1 Page Interrupt active, write "0" to clear interrupt,
 * and write "1" is no affect*/
#define PANTHER7_HDMA_CH1_PAGE_INT_NACT 0x0	/* not active */
#define PANTHER7_HDMA_CH1_PAGE_INT_ACT (0x1 << 3) /* active */

/* Channel 0 Page Interrupt active, write "0" to clear interrupt,
 * and write "1" is no affect*/
#define PANTHER7_HDMA_CH0_PAGE_INT_NACT 0x0	/* not active */
#define PANTHER7_HDMA_CH0_PAGE_INT_ACT (0x1 << 2)	/* active */

/* Channel 1 Interrupt active, write "0" to clear interrupt,
 * and write "1" is no affect*/
#define PANTHER7_HDMA_CH1_INT_NCT 0x0	/* not active */
#define PANTHER7_HDMA_CH1_INT_ACT (0x1 << 1) /* active */

/* Channel 0 Page Interrupt active, write "0" to clear interrupt,
 * and write "1" is no affect*/
#define PANTHER7_HDMA_CH0_INT_NACT 0x0	/* not active */
#define PANTHER7_HDMA_CH0_INT_ACT 0x1	/* active */

/*
 *  PANTHER7_HDMA_DSR
 *  */
/* Channel 1 status */
#define PANTHER7_HDMA_CH1_STAT_RDY		0x0			/* channel1 is ready */
#define PANTHER7_HDMA_CH1_STAT_PERF_DMA	(0x1 << 1)	/* channel1 is performing DMA */

/* Channel 0 status */
#define PANTHER7_HDMA_CH0_STAT_RDY		0x0			/* channel0 is ready */
#define PANTHER7_HDMA_CH0_STAT_PERF_DMA	0x1			/* channel0 is performing DMA */

/*
 *  PANTHER7_HDMA_PRIO
 *  */
/* HDMA arbiter priority mode */
#define PANTHER7_HDMA_PRIOR_CH0_HIGH 0x0 /* channel 0 > channel 1 */
#define PANTHER7_HDMA_PRIOR_CH1_HIGH (0x1 < 1) /* channel 1 > channel 0 */

/* HDMA arbiter priority mode */
#define PANTHER7_HDMA_PRIOR_MODE_ROUND_ROBIN 0x0 /* round-robin */
#define PANTHER7_HDMA_PRIOR_MODE_FIXED_ARBIT /* fixed arbitration */

#endif
