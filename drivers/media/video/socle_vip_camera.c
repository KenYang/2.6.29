/*
 * V4L2 Driver for SuperH Mobile CEU interface
 *
 * Copyright (C) 2008 Magnus Damm
 *
 * Based on V4L2 Driver for PXA camera host - "pxa_camera.c",
 *
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
//#include <linux/mutex.h>
#include <linux/videodev2.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>
#include <media/socle_vip_camera.h>
#include <media/videobuf-dma-contig.h>
#include <mach/regs-vip.h>



//static DEFINE_MUTEX(camera_lock);

/* per video frame buffer */
struct socle_camera_buffer {
	struct videobuf_buffer vb; /* v4l buffer must be first */
	const struct soc_camera_data_format *fmt;
};

struct socle_camera_dev {
	struct device *dev;
	struct soc_camera_host ici;
	struct soc_camera_device *icd;

	unsigned int irq;
	void __iomem *base;
	unsigned int video_limit;

	/* lock used to protect videobuf */
	spinlock_t lock;
	struct list_head capture;
	struct videobuf_buffer *active;

	struct socle_vip_camera_info *pdata;
	unsigned int cb_off;
	unsigned int cr_off;
};

static void vip_write(struct socle_camera_dev *priv,
		      unsigned int reg_offs, unsigned int data)
{
	iowrite32(data, priv->base + reg_offs);
}

static unsigned long vip_read(struct socle_camera_dev *priv,
			      unsigned int reg_offs)
{
	return ioread32(priv->base + reg_offs);
}

/*
 *  Videobuf operations
 */
static int socle_camera_videobuf_setup(struct videobuf_queue *vq,
					unsigned int *count,
					unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct socle_camera_dev *pcdev = ici->priv;
	int bytes_per_pixel = (icd->current_fmt->depth + 7) >> 3;

	*size = PAGE_ALIGN(icd->width * icd->height * bytes_per_pixel);

	if (0 == *count)
		*count = 2;

	if (pcdev->video_limit) {
// 20100607 cyli fix because dma_alloc_coherent() in videobuf-dma-contig.c use order aligned
//		while (*size * *count > pcdev->video_limit)
		while ((1 << (get_order(*size) + PAGE_SHIFT)) * *count > pcdev->video_limit)
			(*count)--;
	}
// 20100402 cyli add
	pcdev->active = NULL;
	//printk( "socle_camera_videobuf_setup: count=%d, size=%d\n", *count, *size);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq,
			struct socle_camera_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		&buf->vb, buf->vb.baddr, buf->vb.bsize);

	if (in_interrupt())
		BUG();

	videobuf_dma_contig_free(vq, &buf->vb);
	dev_dbg(&icd->dev, "%s freed\n", __func__);
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static void socle_camera_capture(struct socle_camera_dev *pcdev)
{
	if (pcdev->active) {
			vip_write(pcdev, SOCLE_VIP_CAPTURE_F1SA_Y, (u32)videobuf_to_dma_contig(pcdev->active));
			vip_write(pcdev, SOCLE_VIP_CAPTURE_F1SA_Cb, (u32)videobuf_to_dma_contig(pcdev->active)+pcdev->cb_off);
			vip_write(pcdev, SOCLE_VIP_CAPTURE_F1SA_Cr, (u32)videobuf_to_dma_contig(pcdev->active)+pcdev->cr_off);
			vip_write(pcdev, SOCLE_VIP_FB_SR, vip_read(pcdev, SOCLE_VIP_FB_SR) & ~0x1); 
	}
	
}

static int socle_camera_videobuf_prepare(struct videobuf_queue *vq,
					  struct videobuf_buffer *vb,
					  enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct socle_camera_buffer *buf;
	int ret;

	buf = container_of(vb, struct socle_camera_buffer, vb);

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		vb, vb->baddr, vb->bsize);

	/* Added list head initialization on alloc */
	WARN_ON(!list_empty(&vb->queue));

#ifdef DEBUG
	/* This can be useful if you want to see if we actually fill
	 * the buffer with something */
	memset((void *)vb->baddr, 0xaa, vb->bsize);
#endif

	BUG_ON(NULL == icd->current_fmt);

	if (buf->fmt	!= icd->current_fmt ||
	    vb->width	!= icd->width ||
	    vb->height	!= icd->height ||
	    vb->field	!= field) {
		buf->fmt	= icd->current_fmt;
		vb->width	= icd->width;
		vb->height	= icd->height;
		vb->field	= field;
		vb->state	= VIDEOBUF_NEEDS_INIT;
	}

	vb->size = vb->width * vb->height * ((buf->fmt->depth + 7) >> 3);
	if (0 != vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;
		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;
fail:
	free_buffer(vq, buf);
out:
	return ret;
}

static void socle_camera_videobuf_queue(struct videobuf_queue *vq,
					 struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct socle_camera_dev *pcdev = ici->priv;
	unsigned long flags;
	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		vb, vb->baddr, vb->bsize);

	vb->state = VIDEOBUF_ACTIVE;
	spin_lock_irqsave(&pcdev->lock, flags);
	list_add_tail(&vb->queue, &pcdev->capture);

	if (!pcdev->active) {
		pcdev->active = vb;
		socle_camera_capture(pcdev);
		vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev, SOCLE_VIP_CTRL)  | VIP_CTRL_CAPTURE_EN);
	}
	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void socle_camera_videobuf_release(struct videobuf_queue *vq,
					   struct videobuf_buffer *vb)
{
	free_buffer(vq, container_of(vb, struct socle_camera_buffer, vb));
}

static struct videobuf_queue_ops socle_camera_videobuf_ops = {
	.buf_setup      = socle_camera_videobuf_setup,
	.buf_prepare    = socle_camera_videobuf_prepare,
	.buf_queue      = socle_camera_videobuf_queue,
	.buf_release    = socle_camera_videobuf_release,
};

static irqreturn_t socle_camera_irq(int irq, void *data)
{
	struct socle_camera_dev *pcdev = data;
	struct videobuf_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	vb = pcdev->active;
	list_del_init(&vb->queue);

	if (!list_empty(&pcdev->capture))
		pcdev->active = list_entry(pcdev->capture.next,
					   struct videobuf_buffer, queue);
	else
		pcdev->active = NULL;

	//printk("SOCLE_VIP_FB_SR= %x\n",vip_read(pcdev, SOCLE_VIP_FB_SR));
	socle_camera_capture(pcdev);
	vip_read(pcdev,SOCLE_VIP_INT_STS);
	//if(vip_read(pcdev,SOCLE_VIP_STS)&0x1)
	//	printk("vip overflow!!\n");
	vb->state = VIDEOBUF_DONE;
	do_gettimeofday(&vb->ts);
	vb->field_count++;
	wake_up(&vb->done);
	spin_unlock_irqrestore(&pcdev->lock, flags);

	return IRQ_HANDLED;
}

static int socle_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct socle_camera_dev *pcdev = ici->priv;
	int ret = -EBUSY;

	if (pcdev->icd)
		goto err;

	dev_info(&icd->dev,
		 "Socle VIP camera driver attached to camera %d\n",
		 icd->devnum);

	if (pcdev->pdata->enable_camera)
		pcdev->pdata->enable_camera();

	ret = icd->ops->init(icd);
	if (ret)
		goto err;

	
	
	

	pcdev->icd = icd;
err:

	return ret;
}

static void socle_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct socle_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	/* disable capture, disable interrupts */
	vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev, SOCLE_VIP_CTRL) & ~VIP_CTRL_CAPTURE_EN);
	//vip_write(pcdev, SOCLE_VIP_INT_MASK, VIP_INT_MASK_DISABLE);

	icd->ops->release(icd);
	if (pcdev->pdata->disable_camera)
		pcdev->pdata->disable_camera();

	dev_info(&icd->dev,
		 "Socle VIP camera driver detached from camera %d\n",
		 icd->devnum);

	pcdev->icd = NULL;
}

static int socle_camera_set_bus_param(struct soc_camera_device *icd,
				       __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct socle_camera_dev *pcdev = ici->priv;
	int ret;
	unsigned long camera_flags, common_flags;

	camera_flags = icd->ops->query_bus_param(icd);
	common_flags = soc_camera_bus_param_compatible(camera_flags,
						       pcdev->pdata->flags);
	if (!common_flags)
		return -EINVAL;

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

	if(!(common_flags & SOCAM_DATAWIDTH_8))
		return -EINVAL;
	
	//set output format
	switch (pixfmt) {
		case V4L2_PIX_FMT_YUV420:
			vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev,SOCLE_VIP_CTRL) & ~VIP_CTRL_422_OUTPUT);
			pcdev->cb_off=icd->width*icd->height;
			pcdev->cr_off=pcdev->cb_off+(icd->width*icd->height/4);
			break;
		case V4L2_PIX_FMT_YUV422P:
			vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev,SOCLE_VIP_CTRL) | VIP_CTRL_422_OUTPUT);
			pcdev->cb_off=icd->width*icd->height;
			pcdev->cr_off=pcdev->cb_off+(icd->width*icd->height/2);
			break;
		default:
			return -EINVAL;
	}

	if(common_flags & SOCAM_PCLK_SAMPLE_RISING)
		vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev,SOCLE_VIP_CTRL) & ~VIP_CTRL_NEGATIVE_EDGE );
	if(common_flags & SOCAM_PCLK_SAMPLE_FALLING)
		vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev,SOCLE_VIP_CTRL) | VIP_CTRL_NEGATIVE_EDGE );

	//set frame size
	vip_write(pcdev, SOCLE_VIP_FS, ((icd->width)<<VIP_FS_WIDTH_SHIFT) | (icd->height)<<VIP_FS_HEIGHT_SHIFT);
	//printk("set: SOCLE_VIP_CTRL=%x\n",vip_read(pcdev, SOCLE_VIP_CTRL));
	return 0;
}

#if 0
static int socle_camera_try_bus_param(struct soc_camera_device *icd,
				       __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct socle_camera_dev *pcdev = ici->priv;
	unsigned long camera_flags, common_flags;

	camera_flags = icd->ops->query_bus_param(icd);
	common_flags = soc_camera_bus_param_compatible(camera_flags,
						       pcdev->pdata->flags);
	//printk("camera_flags=%lx, common_flags=%lx\n",camera_flags,common_flags);
	if (!common_flags)
		return -EINVAL;

	return 0;
}
#endif

static int socle_camera_set_fmt_cap(struct soc_camera_device *icd,
				     __u32 pixfmt, struct v4l2_rect *rect)
{
	return icd->ops->set_fmt(icd, pixfmt, rect);
}

static int socle_camera_try_fmt_cap(struct soc_camera_device *icd,
				     struct v4l2_format *f)
{
	int ret;
	/* FIXME: calculate using depth and bus width */

	if (f->fmt.pix.height < 240)
		f->fmt.pix.height = 240;
	if (f->fmt.pix.height > 480)
		f->fmt.pix.height = 480;
	if (f->fmt.pix.width < 320)
		f->fmt.pix.width = 320;
	if (f->fmt.pix.width >720 )
		f->fmt.pix.width = 720;
	f->fmt.pix.width &= ~0x03;
	f->fmt.pix.height &= ~0x03;

	/* limit to sensor capabilities */
//	return icd->ops->try_fmt(icd, f);
	ret = icd->ops->try_fmt(icd, f);
	if (ret < 0)
		return ret;

	switch (f->fmt.pix.field) {
	case V4L2_FIELD_INTERLACED:
		break;
	case V4L2_FIELD_ANY:
		f->fmt.pix.field = V4L2_FIELD_NONE;
		/* fall-through */
	case V4L2_FIELD_NONE:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int socle_camera_reqbufs(struct soc_camera_file *icf,
				 struct v4l2_requestbuffers *p)
{
	int i;

	/* This is for locking debugging only. I removed spinlocks and now I
	 * check whether .prepare is ever called on a linked buffer, or whether
	 * a dma IRQ can occur for an in-work or unlinked buffer. Until now
	 * it hadn't triggered */
	for (i = 0; i < p->count; i++) {
		struct socle_camera_buffer *buf;

		buf = container_of(icf->vb_vidq.bufs[i],
				   struct socle_camera_buffer, vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

static unsigned int socle_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct socle_camera_buffer *buf;

	buf = list_entry(icf->vb_vidq.stream.next,
			 struct socle_camera_buffer, vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE ||
	    buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN|POLLRDNORM;

	return 0;
}

static int socle_camera_querycap(struct soc_camera_host *ici,
				  struct v4l2_capability *cap)
{
	strlcpy(cap->card, "Socle_VIP_Camera", sizeof(cap->card));
	cap->version = KERNEL_VERSION(0, 0, 5);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static void socle_camera_init_videobuf(struct videobuf_queue *q,
					struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct socle_camera_dev *pcdev = ici->priv;

	videobuf_queue_dma_contig_init(q,
				       &socle_camera_videobuf_ops,
				       &ici->dev, &pcdev->lock,
				       V4L2_BUF_TYPE_VIDEO_CAPTURE,
				       V4L2_FIELD_NONE,
				       sizeof(struct socle_camera_buffer),
				       icd);
}

static struct soc_camera_host_ops socle_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= socle_camera_add_device,
	.remove		= socle_camera_remove_device,
	.set_fmt	= socle_camera_set_fmt_cap,
	.try_fmt	= socle_camera_try_fmt_cap,
	.reqbufs	= socle_camera_reqbufs,
	.poll		= socle_camera_poll,
	.querycap	= socle_camera_querycap,
//	.try_bus_param	= socle_camera_try_bus_param,
	.set_bus_param	= socle_camera_set_bus_param,
	.init_videobuf	= socle_camera_init_videobuf,
};

static int socle_camera_probe(struct platform_device *pdev)
{
	struct socle_camera_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	unsigned int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);

	if (!res || !irq) {
		dev_err(&pdev->dev, "Not enough VIP platform resources.\n");
		err = -ENODEV;
		goto exit;
	}

	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	platform_set_drvdata(pdev, pcdev);
	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);

	pcdev->pdata = pdev->dev.platform_data;
	if (!pcdev->pdata) {
		err = -EINVAL;
		dev_err(&pdev->dev, "VIP platform data not set.\n");
		goto exit_kfree;
	}

	if (!request_mem_region(res->start, res->end - res->start + 1, pdev->name)) {
		dev_err(&pdev->dev, "%s(): Failed to request io memory!\n", __func__);
		err = -EBUSY;
		goto exit_kfree;
	}

	base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (!base) {
		err = -ENXIO;
		dev_err(&pdev->dev, "Unable to ioremap VIP registers.\n");
		goto exit_req_mem;
	}

	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		err = dma_declare_coherent_memory(&pdev->dev, res->start,
						  res->start,
						  (res->end - res->start) + 1,
						  DMA_MEMORY_MAP |
						  DMA_MEMORY_EXCLUSIVE);
		if (!err) {
			dev_err(&pdev->dev, "Unable to declare VIP memory.\n");
			err = -ENXIO;
			goto exit_iounmap;
		}

		pcdev->video_limit = (res->end - res->start) + 1;
	}

	//reset vip
	vip_write(pcdev, SOCLE_VIP_CTRL, 0);
	//set incr16
	vip_write(pcdev, SOCLE_VIP_AHBR_CTRL, VIP_AHBR_CTRL_INCR16);
	//set ping pong mode
	vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev, SOCLE_VIP_CTRL) & ~VIP_CTRL_ONE_FRAME_STOP);
	vip_write(pcdev, SOCLE_VIP_CTRL, vip_read(pcdev, SOCLE_VIP_CTRL) | VIP_CTRL_PING_PONG_MODE);
	/* request irq */
	vip_write(pcdev, SOCLE_VIP_INT_MASK, VIP_INT_MASK_CAPTURE_COMPLETE);
	err = request_irq(pcdev->irq, socle_camera_irq, IRQF_DISABLED,
			  pdev->dev.bus_id, pcdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to register VIP interrupt.\n");
		goto exit_release_mem;
	}

	pcdev->ici.priv = pcdev;
	pcdev->ici.dev.parent = &pdev->dev;
	pcdev->ici.nr = pdev->id;
	pcdev->ici.drv_name = dev_name(&pdev->dev);
	pcdev->ici.ops = &socle_camera_host_ops;
	pcdev->ici.dev.dma_mem=pdev->dev.dma_mem;
	//pcdev->ici.dev.coherent_dma_mask=pdev->dev.coherent_dma_mask;
	err = soc_camera_host_register(&pcdev->ici);
	if (err)
		goto exit_free_irq;

	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
exit_release_mem:
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
exit_iounmap:
	iounmap(base);
exit_req_mem:
	release_mem_region(res->start, res->end - res->start + 1);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int socle_camera_remove(struct platform_device *pdev)
{
	struct socle_camera_dev *pcdev = platform_get_drvdata(pdev);

	soc_camera_host_unregister(&pcdev->ici);
	free_irq(pcdev->irq, pcdev);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
	iounmap(pcdev->base);
	kfree(pcdev);
	return 0;
}

#if 0
static int socle_vip_camera_runtime_nop(struct device *dev)
{
	/* Runtime PM callback shared between ->runtime_suspend()
	 * and ->runtime_resume(). Simply returns success.
	 *
	 * This driver re-initializes all registers after
	 * pm_runtime_get_sync() anyway so there is no need
	 * to save and restore registers here.
	 */
	return 0;
}
#endif

#ifdef CONFIG_PM

// 2010/06/08 cyli add power management

static u32 socle_vip_regs[SOCLE_VIP_L_SFT / 4 + 1];

static int socle_vip_camera_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct socle_camera_dev *pcdev = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < (SOCLE_VIP_L_SFT / 4 + 1); i++)
		socle_vip_regs[i] = vip_read(pcdev, i * 4);
		//socle_vip_regs[i] = vip_read(pcdev, (unsigned long)(i * 4));

	return 0;
}

static int socle_vip_camera_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct socle_camera_dev *pcdev = platform_get_drvdata(pdev);
	int i, offset;

	for (i = 0; i < (SOCLE_VIP_L_SFT / 4 + 1); i++) {
		offset = i * 4;
		if ((SOCLE_VIP_CTRL == offset) || (SOCLE_VIP_RESET == offset))
			continue;
		vip_write(pcdev, offset, socle_vip_regs[i]);
	}
	vip_write(pcdev, SOCLE_VIP_CTRL, socle_vip_regs[SOCLE_VIP_CTRL / 4]);

	return 0;
}

#else
#define socle_vip_camera_suspend	NULL
#define socle_vip_camera_resume	NULL
#endif

static struct dev_pm_ops socle_vip_camera_dev_pm_ops = {
	.suspend = socle_vip_camera_suspend,
	.resume = socle_vip_camera_resume,
	//.runtime_suspend = socle_vip_camera_runtime_nop,
	//.runtime_resume = socle_vip_camera_runtime_nop,
};

static struct platform_driver socle_camera_driver = {
	.driver 	= {
		.name	= "socle_camera",
		.pm	= &socle_vip_camera_dev_pm_ops,
	},
	.probe		= socle_camera_probe,
	.remove		= socle_camera_remove,
};

static int __init socle_camera_init(void)
{
	return platform_driver_register(&socle_camera_driver);
}

static void __exit socle_camera_exit(void)
{
	platform_driver_unregister(&socle_camera_driver);
}

module_init(socle_camera_init);
module_exit(socle_camera_exit);

MODULE_DESCRIPTION("Socle VIP camera driver");
MODULE_AUTHOR("JS Ho");
MODULE_LICENSE("GPL");
