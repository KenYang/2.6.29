#ifndef __SOCLE_DMA_H_INCLUDED
#define __SOCLE_DMA_H_INCLUDED

#include <linux/spinlock.h>
#include <linux/ioport.h>

#define SOCLE_MAX_DMA_CHANNELS 16

struct socle_dma_notifier {
	void (*complete)(void *data);
	void (*error)(void *data);
	void (*page_interrupt)(void *data);
	void (*page_accumulation_overflow)(void *data);
	void *data;
};

struct socle_dma;
struct socle_dma_ops {
	int (*request)(u32 ch, struct socle_dma *dma);
	void (*free)(u32 ch, struct socle_dma *dma);
	void (*enable)(u32 ch, struct socle_dma *dma);
	void (*disable)(u32 ch, struct socle_dma *dma);
	void (*set_page_number)(u32 ch, struct socle_dma *dma, u32 num);
};

struct socle_dma{
	u32 src_addr;		/* source address */
	u32 dst_addr;		/* destination address */
	u8 src_dir;		/* source direction */
	u8 dst_dir;		/* destination direction */
	u32 tx_cnt;		/* transfer count */
	u8 burst_type;		/* burst type */
	u8 data_size;		/* data size */
	u8 mode;		/* transfer mode */
	u32 slice_cnt;		/* slice count */
	u32 buf_size;		/* buffer size */
	u32 fly_op;		/* on the fly mode */
	u32 ext_hdreq;		/* external hdreq source selection */
	const char *dma_name;	/* dma name */
	const char *dev_id;	/* device name */
	struct resource *res;	/* io memory resource */
	u8 active;		/* transfer active */
	u32 lock;		/* device is allocated */
	u32 irq;		/* controller IRQ */
	void *private_data;	/* private data */
	struct socle_dma_ops *ops;
	struct socle_dma_notifier *notifier;
	//leonid+ 
	u32	base;	
};

/* DMA mode */
enum SOCLE_DMA_MODE {
	SOCLE_DMA_MODE_SW,
	SOCLE_DMA_MODE_HW,
	SOCLE_DMA_MODE_SLICE,
};

/* DMA burst type */
enum SOCLE_DMA_BURST {
	SOCLE_DMA_BURST_SINGLE,
	SOCLE_DMA_BURST_INCR4,
	SOCLE_DMA_BURST_INCR8,
	SOCLE_DMA_BURST_INCR16,
};

/* DMA source/destination direction */
enum SOCLE_DMA_DIRECTION {
	SOCLE_DMA_DIR_INCR,
	SOCLE_DMA_DIR_FIXED,
};

/* DMA data size */
enum SOCLE_DMA_DATA_SIZE {
	SOCLE_DMA_DATA_BYTE,
	SOCLE_DMA_DATA_HALFWORD,
	SOCLE_DMA_DATA_WORD,
};

/* DMA on the fly mode enable/disable */
enum SOCLE_DMA_FLY {
	SOCLE_DMA_FLY_DIS,
	SOCLE_DMA_FLY_READ,
	SOCLE_DMA_FLY_WRITE,
};

extern spinlock_t socle_dma_spin_lock;

static inline unsigned long
socle_claim_dma_lock(void)
{
	unsigned long flags;

	spin_lock_irqsave(&socle_dma_spin_lock, flags);
	return flags;
}

static inline void
socle_release_dma_lock(unsigned long flags)
{
	spin_unlock_irqrestore(&socle_dma_spin_lock, flags);
}

/* Request a DMA channel */
extern int socle_request_dma(u32 ch, const char *dev_id, struct socle_dma_notifier *notifier);

/* Free a DMA channel */
extern void socle_free_dma(u32 ch);

/* Enable DMA channel */
extern void socle_enable_dma(u32 ch);

/* Disable DMA for this channel */
extern void socle_disable_dma(u32 ch);

/* Test whether the specified channel has an active DMA transfer */
extern int socle_dma_channel_active(u32 ch);

/* Set DMA transfer count */
extern void socle_set_dma_transfer_count(u32 ch, u32 cnt);

/* Set DMA source address */
extern void socle_set_dma_source_address(u32 ch, u32 addr);

/* Set DMA source address */
extern void socle_set_dma_destination_address(u32 ch, u32 addr);

/* Set DMA source direction */
extern void socle_set_dma_source_direction(u32 ch, int dir);

/* Set DMA destination direction */
extern void socle_set_dma_destination_direction(u32 ch, int dir);

/* Set DMA burst type */
extern void socle_set_dma_burst_type(u32 ch, int burst);

/* Set DMA data size */
extern void socle_set_dma_data_size(u32 ch, int size);

/* Set DMA mode */
extern void socle_set_dma_mode(u32 ch, int mode);

/* Set DMA external hdreq source selection */
extern void socle_set_dma_ext_hdreq_number(u32 ch, u32 num);

/* Set DMA on the fly mode enable/disable */
extern void socle_set_dma_fly_operation(u32 ch, u32 fly_op);

/* Set DMA slice count */
extern void socle_set_dma_slice_count(u32 ch, u32 cnt);

/* Set DMA page number */
extern void socle_set_dma_page_number(u32 ch, u32 num);

/* Set DMA buffer size */
extern void socle_set_dma_buffer_size(u32 ch, u32 size);

extern void socle_dump_dma_register(u32 ch, u32 size);

extern void socle_platform_dma_init(struct socle_dma **dma);
#endif
