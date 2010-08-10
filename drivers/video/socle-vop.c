/********************************************************************************
* File Name     : drivers/char/socle-vop.c 
* Author        : jsho
* Description   : Socle VOP Driver (VOP)
* 
* Copyright (C) Socle Tech. Corp.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
    
*   Version      : 0,0,0,1
*   History      : 
*      1. 2008/06/18 jsho create this file
*    
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
//#include <asm/uaccess.h>		// for ioctl
#include <mach/regs-vop.h>
#include <mach/socle-vop.h>

//#define CONFIG_VOP_DEBUG
#ifdef CONFIG_VOP_DEBUG
	#define VOP_DBG(fmt, args...) printk("VOP: " fmt, ## args)
#else
	#define VOP_DBG(fmt, args...)
#endif

static char __initdata banner[] = "SOCLE VOP, (c) 2008 SOCLE Corp.\n";
static int vop_mmap(struct file *flip, struct vm_area_struct *vma);
static char *frame_va;
static dma_addr_t frame_pa;
static unsigned int frame_len;
static unsigned int addr_y, addr_cb, addr_cr;
static void vopReset(void);
static void vopStart(void);
static void vopStop(void);
static int vopSetOutFormat(int format);
static int vopSetFrameSize(u32 framesize);

static int vopSetFrameDisplayAddr(int frameNum, u32 Y_pt, u32 Cb_pt, u32 Cr_pt);

//#define USE_INT
#ifdef USE_INT
static int vopSetFrameMode(int frame_mode);
static irqreturn_t vopOneFrameIsr (int irq, void *pparm);
static irqreturn_t vopTwoFrameIsr (int irq, void *pparm);
#endif

static inline void
socle_vop_write(u32 val, u32 reg)
{
	iowrite32(val, SOCLE_VOP_BASE+reg);
}

static inline u32
socle_vop_read(u32 reg)
{
	return ioread32(SOCLE_VOP_BASE+reg);
}

static void
vopReset()
{
	socle_vop_write(VOP_CTRL_RESET | VOP_CTRL_DISPLAY_DIS, SOCLE_VOP_CTRL);
	msleep(260);
	socle_vop_write(VOP_INT_DISABLE, SOCLE_VOP_INTE);
	socle_vop_write(VOP_CTRL_NO_RESET | VOP_CTRL_DISPLAY_DIS, SOCLE_VOP_CTRL);
}

static void
vopStart()
{
	socle_vop_write(VOP_AHBR_CTRL_INCR16, SOCLE_VOP_AHBR_CTRL);
#ifdef USE_INT
	socle_vop_write(VOP_INTE_DISPLAY_COMPLETE, SOCLE_VOP_INTE);
#endif
	socle_vop_write(socle_vop_read(SOCLE_VOP_CTRL) | VOP_CTRL_DISPLAY_EN, SOCLE_VOP_CTRL);
}

static void
vopStop ()
{  
  socle_vop_write(VOP_INT_DISABLE, SOCLE_VOP_INTE);
  socle_vop_write(VOP_CTRL_DISPLAY_DIS, SOCLE_VOP_CTRL);
}

static int
vopSetOutFormat(int format)
{
	if(format==FORMAT_NTSC)
		socle_vop_write(socle_vop_read(SOCLE_VOP_CTRL) & ~VOP_CTRL_FORMAT_PAL, SOCLE_VOP_CTRL);
	else if(format==FORMAT_PAL)
		socle_vop_write(socle_vop_read(SOCLE_VOP_CTRL) | VOP_CTRL_FORMAT_PAL, SOCLE_VOP_CTRL);
	else {
		printk("VOP : Error Output Format!!\n");
		return -1;
	}
	return 0;	
}

static int
vopSetFrameSize(u32 framesize)
{	
	int ret=0;
	int width=(framesize >> 10);
	int height=(framesize & 0x3ff);
	if(width>720 || height>480) {
		printk("VOP: display size error\n");
		return -1;
	}		
	addr_y = frame_pa;
  addr_cb = (addr_y + width*height);
  addr_cr = (addr_cb + width*height/4);
  
	socle_vop_write(framesize, SOCLE_VOP_FSS);
	ret |= vopSetFrameDisplayAddr(FRAME1, addr_y, addr_cb, addr_cr);
	return ret;
}

static int 
vopSetFrameDisplayAddr(int frameNum, u32 Y_pt, u32 Cb_pt, u32 Cr_pt)
{
	if(frameNum==FRAME1) {
		socle_vop_write(Y_pt, SOCLE_VOP_DRF1SAY);
		socle_vop_write(Cb_pt, SOCLE_VOP_DRF1SACB);
		socle_vop_write(Cr_pt, SOCLE_VOP_DRF1SACR);
		socle_vop_write(VOP_FBS_FRAME1_USE_BY_HW, SOCLE_VOP_FBS);
	}
	else if(frameNum==FRAME2) {
		socle_vop_write(Y_pt, SOCLE_VOP_DRF2SAY);
		socle_vop_write(Cb_pt, SOCLE_VOP_DRF2SACB);
		socle_vop_write(Cr_pt, SOCLE_VOP_DRF2SACR);
		socle_vop_write(VOP_FBS_FRAME2_USE_BY_HW, SOCLE_VOP_FBS);
	}
	else {
		printk("VOP: Error Frame Number!!\n");
    return -1;
	}
	return 0;
}

#ifdef USE_INT 
static int
vopSetFrameMode(int frame_mode)
{
	if(frame_mode==ONE_FRAME) {
		if (request_irq(SOCLE_INTC_VOP, (irq_handler_t) vopOneFrameIsr, IRQF_DISABLED, "vop", NULL) < 0) {
			printk("VOP: Can't allocate irq\n");
	  	return -EBUSY;
	   ;
		}
	}
	else if(frame_mode==TWO_FRAME) {
		if (request_irq (SOCLE_INTC_VOP, (irq_handler_t) vopTwoFrameIsr, IRQF_DISABLED, "vop", NULL) < 0) {
			printk("VOP: Can't allocate irq\n");
	  	return -EBUSY;
	  }
	}
	else {
		printk("VOP: Error Frame Mode!!\n");
    return -1;
	}
	return 0;
}


static irqreturn_t vopOneFrameIsr (int irq, void *dev)
{
  u32 int_sts = socle_vop_read(SOCLE_VOP_INT_STS);
  u32 frame_sts = socle_vop_read(SOCLE_VOP_FBS);
	if (int_sts & VOP_INT_STS_DISPLAY_COMPLETE) {    
  	if ((frame_sts & VOP_FBS_FRAME1_USE_BY_HW) == 0) 
	  	socle_vop_write(VOP_FBS_FRAME1_USE_BY_HW, SOCLE_VOP_FBS);
  }
 
  if (int_sts & VOP_INT_STS_BUFFER_UNDERUN) {
  	int line_num;
    line_num = int_sts >> VOP_INT_STS_LINE_NUM_OF_UNDERUN_SHIFT;
		printk("VOP : Buffer Underflow at line 0x%x\n",line_num);
	}
	return IRQ_HANDLED;
}

static irqreturn_t vopTwoFrameIsr (int irq, void *dev)
{
  u32 int_sts = socle_vop_read(SOCLE_VOP_INT_STS);
  u32 frame_sts = socle_vop_read(SOCLE_VOP_FBS);
  
	if (int_sts & VOP_INT_STS_DISPLAY_COMPLETE) {    
  	if ((frame_sts & VOP_FBS_FRAME1_USE_BY_HW) == 0)
	  	socle_vop_write(VOP_FBS_FRAME1_USE_BY_HW, SOCLE_VOP_FBS);
  	if ((frame_sts & VOP_FBS_FRAME2_USE_BY_HW) == 0)
	  	socle_vop_write(VOP_FBS_FRAME2_USE_BY_HW, SOCLE_VOP_FBS);
  }
  if (int_sts & VOP_INT_STS_BUFFER_UNDERUN) {
  	int line_num;
    line_num = int_sts >> VOP_INT_STS_LINE_NUM_OF_UNDERUN_SHIFT;
		printk("VOP : Buffer Underflow at line 0x%x\n",line_num);
	}

	return IRQ_HANDLED;
}
#endif

static int
vop_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	VOP_DBG("vop_ioctl cmd=0x%08x arg=0x%08lx\n", cmd, arg);
	switch (cmd) {
	case VOP_CTRL:
		if (arg == VOP_START) {
			vopStart();
		  return 0;
		} 
		else if(arg == VOP_STOP) {
			vopStop();
		  return 0;
		}
		else if(arg == VOP_RESET) {
			vopReset();
			return 0;
		}
		else {
			VOP_DBG("unknown arg\n");
			return 0;
		}
	case VOP_SIZE:
		if(vopSetFrameSize(arg))
			return -1;
		return 0;		
	default:
		printk("VOP: unknown command\n");
		return -EINVAL;
	}
}


static int
vop_open(struct inode *inode, struct file *file)
{
	VOP_DBG("vop open\n");
	return 0;
}

#if 0
static ssize_t 
vop_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos) 
{
	VOP_DBG("vop write\n");
	return count;
}
#endif

static int
vop_release(struct inode *inode, struct file *file)
{
	VOP_DBG("vop release\n");
	return 0;
}

static const struct file_operations vop_fops = {
        .owner =        THIS_MODULE,
        .ioctl =        vop_ioctl,
//        .write =        vop_write,
        .mmap =         vop_mmap,
        .open =         vop_open,
        .release =      vop_release,
};

struct miscdevice misc_vop = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "socle-vop",
	.fops = &vop_fops,
};

static int
vop_remove(struct platform_device *dev)
{
	VOP_DBG("vop remove\n");
#ifdef USE_INT
  free_irq(SOCLE_INTC_VOP, NULL);
#endif
	misc_deregister(&misc_vop);
	return 0;
}

static int
vop_probe(struct platform_device *pdev)
{
	int ret = 0;
	dma_addr_t dma;
  frame_va = (char *)dma_alloc_writecombine(NULL, SZ_512K, &dma, GFP_KERNEL);
	if (!frame_va) {
		printk("VOP: unable to map framebuffer\n");
		return -ENOMEM;
	}
	frame_pa = dma;
	frame_len = SZ_512K;
  ret = misc_register(&misc_vop);
	if(ret!=0)
		printk("VOP: probe fail\n");
		
	vopReset();
	ret |= vopSetOutFormat(FORMAT_NTSC);
#ifdef USE_INT  
	ret |= vopSetFrameMode(ONE_FRAME);
#endif
	if(ret!=0)
		printk("VOP: init fail\n");
	return ret;
}

static int 
vop_mmap(struct file *flip, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(NULL, vma, (char *)frame_va, frame_pa, frame_len);
}

static struct platform_driver vop_drv = {
	.probe		= vop_probe,
	.remove		= vop_remove,
	.driver		= {
		.name	= "socle-vop",
		.owner	= THIS_MODULE,
	},
};


static int __init
vop_init(void)
{
	printk(banner);
	return platform_driver_register(&vop_drv);
}

static void __exit
vop_exit(void)
{
	platform_driver_unregister(&vop_drv);
}

module_init(vop_init);
module_exit(vop_exit);

MODULE_DESCRIPTION("SOCLE VOP Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("JS Ho <jsho@socle-tech.com.tw>");
