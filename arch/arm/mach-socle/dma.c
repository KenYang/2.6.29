#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <asm/system.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mach/dma.h>
#include <asm/io.h>

DEFINE_SPINLOCK(socle_dma_spin_lock);
EXPORT_SYMBOL(socle_dma_spin_lock);

static struct socle_dma *socle_dma_channel[SOCLE_MAX_DMA_CHANNELS] = {NULL};

static void*
socle_dma_seq_start(struct seq_file *s, loff_t *pos)
{
	return socle_dma_channel[*pos];
}

static void*
socle_dma_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return NULL;
}

static void 
socle_dma_seq_stop(struct seq_file *s, void *v)
{
	/* Actually, there's nothing to do here */
}

static int
socle_dma_seq_show(struct seq_file *s, void *v)
{
	struct socle_dma *ch = (struct socle_dma *)v;

	seq_printf(s, "Channel%2d:\n", (u32)s->index);
	seq_printf(s, "Name     : %s\n", ch->dma_name);
	seq_printf(s, "IRQ      : %d\n", ch->irq);
 	seq_printf(s, "IO memory: %08x-%08x\n", ch->res->start, ch->res->end);
	seq_printf(s, "Lock     : %s %s\n", ch->lock ? "locked by" : "unlocked", ch->dev_id ? ch->dev_id : "");
	seq_printf(s, "Active   : %s\n", ch->active ? "active" : "idle");
	seq_printf(s, "\n");
	return 0;
}

/*
 *  Tie the sequence operators up.
 *  */
static struct seq_operations socle_dma_seq_ops = {
	.start = socle_dma_seq_start,
	.next = socle_dma_seq_next,
	.stop = socle_dma_seq_stop,
	.show = socle_dma_seq_show,
};

/*
 *  Now to implement the /proc file we need only make an open
 *  method which sets up the sequence operators.
 *  */
static int socle_dma_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &socle_dma_seq_ops);
}

/*
 *  Create a set of file operations for our proc file.
 *  */
static struct file_operations socle_dma_proc_ops = {
	.owner = THIS_MODULE,
	.open = socle_dma_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release, 
};

static struct proc_dir_entry *socle_dma_proc_entry;



/*
 *  Request DMA channel
 *  We have to allocate an interrupt as well...
 *  */
extern int
socle_request_dma(u32 ch, const char *dev_id, struct socle_dma_notifier *notifier)
{
	struct socle_dma *dma = socle_dma_channel[ch];
	int ret;

	if ((ch >= SOCLE_MAX_DMA_CHANNELS) || !dma->ops)
		goto bad_dma;
	if (xchg(&dma->lock, 1) != 0)
		goto busy;
	dma->dev_id = dev_id;
	dma->active = 0;
	ret = 0;
	dma->notifier = notifier;
	if (dma->ops->request)
		ret = dma->ops->request(ch, dma);
	if (ret)
		xchg(&dma->lock, 0);
	return ret;
bad_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to allocate DMA\n", ch, dma->dma_name);
	return -EINVAL;
busy:
	return -EBUSY;
}
EXPORT_SYMBOL(socle_request_dma);

/*
 *  Free DMA channel
 *  We have to free interrupt as well
 *  */
extern void
socle_free_dma(u32 ch)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (ch >= SOCLE_MAX_DMA_CHANNELS || !dma->ops)
		goto bad_dma;
	if (dma->active) {
		printk(KERN_ERR "Socle DMA Channel%d.%s: freeing active DMA\n", ch, dma->dma_name);
		dma->ops->disable(ch, dma);
		dma->active = 0;
	}
	if (xchg(&dma->lock, 0) != 0) {
		dma->notifier = NULL;
		dma->dev_id = NULL;
		if (dma->ops->free)
			dma->ops->free(ch, dma);
		return;
	}
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to free free DMA\n", ch, dma->dma_name);
	return;
bad_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to free DMA\n", ch, dma->dma_name);
}
EXPORT_SYMBOL(socle_free_dma);

/*
 *  Enable DMA channel
 *  */
extern void 
socle_enable_dma(u32 ch)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (0 == dma->active) {
		dma->active = 1;
		dma->ops->enable(ch, dma);
	}
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to enable free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_enable_dma);

/*
 *  Disable DMA channel
 *  */
extern void
socle_disable_dma(u32 ch)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (1 == dma->active) {
		dma->active = 0;
		dma->ops->disable(ch, dma);
	}
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to disable free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_disable_dma);

/*
 *  Is the specified DAM channel active
 *  */
extern int
socle_dma_channel_active(u32 ch)
{
	return socle_dma_channel[ch]->active;
}
EXPORT_SYMBOL(socle_dma_channel_active);

extern void
socle_set_dma_transfer_count(u32 ch, u32 cnt)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering transfer count "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->tx_cnt = cnt;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_transfer_count);

extern void
socle_set_dma_source_address(u32 ch, u32 addr)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering source address "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->src_addr = addr;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_source_address);

extern void
socle_set_dma_destination_address(u32 ch, u32 addr)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering destination address "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->dst_addr = addr;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_destination_address);

extern void
socle_set_dma_source_direction(u32 ch, int dir)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering source direction "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->src_dir = dir;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_source_direction);

extern void
socle_set_dma_destination_direction(u32 ch, int dir)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering destination direction "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->dst_dir = dir;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_destination_direction);

extern void
socle_set_dma_burst_type(u32 ch, int burst)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering burst type "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->burst_type = burst;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_burst_type);

extern void
socle_set_dma_data_size(u32 ch, int size)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering data size "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->data_size = size;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_data_size);

extern void
socle_set_dma_mode(u32 ch, int mode)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering dma mode "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->mode = mode;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_mode);

extern void
socle_set_dma_ext_hdreq_number(u32 ch, u32 num)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering external hardware request number "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->ext_hdreq = num;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_ext_hdreq_number);

extern void
socle_set_dma_fly_operation(u32 ch, u32 fly_op)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering fly operation "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->fly_op = fly_op;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_fly_operation);

extern void
socle_set_dma_slice_count(u32 ch, u32 cnt)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering slice count "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->slice_cnt = cnt;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_slice_count);

extern void
socle_set_dma_page_number(u32 ch, u32 num)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	dma->ops->set_page_number(ch, dma, num);
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_page_number);

extern void
socle_set_dma_buffer_size(u32 ch, u32 size)
{
	struct socle_dma *dma = socle_dma_channel[ch];

	if (!dma->lock)
		goto free_dma;
	if (socle_dma_channel[ch]->active) 
		printk(KERN_ERR "Socle DMA Channel%d.%s: altering buffer size "
		       "while channel active\n", ch, dma->dma_name);
	socle_dma_channel[ch]->buf_size = size;
	return;
free_dma:
	printk(KERN_ERR "Socle DMA Channel%d.%s: trying to access free DMA\n", ch, dma->dma_name);
	BUG();
}
EXPORT_SYMBOL(socle_set_dma_buffer_size);

extern void 
socle_dump_dma_register(u32 ch, u32 size)
{
	int i;

	printk(KERN_INFO "%s:\n", socle_dma_channel[ch]->dma_name);
	for (i = 0; i <= size; i += 4)
		printk(KERN_INFO "0x%08x/0x%08x:0x%08x\n", socle_dma_channel[ch]->res->start+i, IO_ADDRESS(socle_dma_channel[ch]->res->start+i), ioread32(IO_ADDRESS(socle_dma_channel[ch]->res->start+i)));
}

EXPORT_SYMBOL(socle_dump_dma_register);

static int __init
socle_init_dma(void)
{
	socle_platform_dma_init(socle_dma_channel);

	/* Install the proc_fs entry */
//	socle_dma_proc_entry = create_proc_entry("socle_dma", S_IRUGO | S_IFREG, &proc_root);
	socle_dma_proc_entry = create_proc_entry("socle_dma", S_IRUGO | S_IFREG, NULL);
	if (socle_dma_proc_entry) {
		socle_dma_proc_entry->proc_fops = &socle_dma_proc_ops;
		socle_dma_proc_entry->data = NULL;
	} else
		return -ENOMEM;
	return 0;
}

core_initcall(socle_init_dma);

