/********************************************************************************
* File Name     : drivers/usb/gadget/socle_otg_udc.c
* Author         : Leonid Cheng
* Description   : driver for socle-series USB OTG peripheral controller (OTG UDC Driver)
* 
* Copyright (C) Socle Tech. Corp.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
*without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
    
*   Version      : 1,0,0,0
*   History      : 
*      1. 2008/12/12 leonid cheng create this file 
*    
********************************************************************************/

#undef VERBOSE

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <mach/otg_devices.h>		//20090527 leonid fix for 2.6.27 include path
#include <linux/dmapool.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>

#include "socle_otg_udc.h"

#ifdef CONFIG_SOCLE_USB_OTG
static char probe_a=0;		//1:a has probe		0:a not probe or has  remove
static char probe_b=0;		//1:b has probe		0:b not probe or has  remove
static char b_init=0;	
static char b_exit=0;	
static char a_init=0;	
static char a_exit=0;

struct 	timer_list 			otg_timer;

static void socle_otg_switch_detection(struct work_struct *work);
static DECLARE_DELAYED_WORK (otg_dwork, socle_otg_switch_detection);

#define	DRIVER_DESC_OTG	"SOCLE High-Speed USB OTG Controller driver"
//leonid+ for OTG
static const char driver_name_otg[] = "socle_otg";
static const char driver_desc_otg[] = DRIVER_DESC_OTG;

#endif

#define	DRIVER_DESC	"SOCLE High-Speed USB SOC Device Controller driver"
#define	DRIVER_AUTHOR	"Leonid"
#define	DRIVER_VERSION	"DEC 12, 2008"

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

static const char driver_name[] = "socle_otg_udc";
static const char driver_desc[] = DRIVER_DESC;

volatile static struct usb_dr_device *dr_regs = NULL;
volatile static struct usb_sys_interface *usb_sys_regs = NULL;

/* it is initialized in probe()  */
static struct socle_otg_udc *udc_controller = NULL;

static const struct usb_endpoint_descriptor
socle_otg_ep0_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	0,
	.bmAttributes =		USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize =	USB_MAX_CTRL_PAYLOAD,
};

static int socle_otg_udc_suspend(struct platform_device *pdev, pm_message_t state);
static int socle_otg_udc_resume(struct platform_device *pdev);
static void socle_otg_ep_fifo_flush(struct usb_ep *_ep);

#ifdef CONFIG_PPC32
#define socle_otg_readl(addr)		in_le32(addr)
#define socle_otg_writel(addr, val32) out_le32(val32, addr)
#else
#define socle_otg_readl(addr)		readl(addr)
#define socle_otg_writel(addr, val32) writel(addr, val32)
#endif

/********************************************************************
 *	Internal Used Function
********************************************************************/
/*-----------------------------------------------------------------
 * done() - retire a request; caller blocked irqs
 * @status : request status to be set, only works when
 *	request is still in progress.
 *--------------------------------------------------------------*/
static void done(struct socle_otg_ep *ep, struct socle_otg_req *req, int status)
{
	struct socle_otg_udc *udc = NULL;
	unsigned char stopped = ep->stopped;
	struct ep_td_struct *curr_td, *next_td;
	int j;

	udc = (struct socle_otg_udc *)ep->udc;
	/* Removed the req from socle_otg_ep->queue */
	list_del_init(&req->queue);

	/* req.status should be set as -EINPROGRESS in ep_queue() */
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* Free dtd for the request */
	next_td = req->head;
	for (j = 0; j < req->dtd_count; j++) {
		curr_td = next_td;
		if (j != req->dtd_count - 1) {
			next_td = curr_td->next_td_virt;
		}
		dma_pool_free(udc->td_pool, curr_td, curr_td->td_dma);
	}

	if (req->mapped) {
		dma_unmap_single(ep->udc->gadget.dev.parent,
			req->req.dma, req->req.length,
			ep_is_in(ep)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	} else
		dma_sync_single_for_cpu(ep->udc->gadget.dev.parent,
			req->req.dma, req->req.length,
			ep_is_in(ep)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);

	if (status && (status != -ESHUTDOWN))
		VDBG("complete %s req %p stat %d len %u/%u",
			ep->ep.name, &req->req, status,
			req->req.actual, req->req.length);

	ep->stopped = 1;

	spin_unlock(&ep->udc->lock);
	/* complete() is from gadget layer,
	 * eg fsg->bulk_in_complete() */
	if (req->req.complete)
		req->req.complete(&ep->ep, &req->req);

	spin_lock(&ep->udc->lock);
	ep->stopped = stopped;
}

/*-----------------------------------------------------------------
 * nuke(): delete all requests related to this ep
 * called with spinlock held
 *--------------------------------------------------------------*/
static void nuke(struct socle_otg_ep *ep, int status)
{
	ep->stopped = 1;

	/* Flush fifo */
	socle_otg_ep_fifo_flush(&ep->ep);

	/* Whether this eq has request linked */
	while (!list_empty(&ep->queue)) {
		struct socle_otg_req *req = NULL;

		req = list_entry(ep->queue.next, struct socle_otg_req, queue);
		done(ep, req, status);
	}
}

/*------------------------------------------------------------------
	Internal Hardware related function
 ------------------------------------------------------------------*/

static int dr_controller_setup(struct socle_otg_udc *udc)
{
	unsigned int tmp = 0, portctrl = 0, ctrl = 0;
	unsigned long timeout;
#define SOCLE_OTG_UDC_RESET_TIMEOUT 1000

	/* before here, make sure dr_regs has been initialized */
	if (!udc)
		return -EINVAL;

	/* Stop and reset the usb controller */
	tmp = socle_otg_readl(&dr_regs->usbcmd);
	tmp &= ~USB_CMD_RUN_STOP;
	socle_otg_writel(tmp, &dr_regs->usbcmd);

	tmp = socle_otg_readl(&dr_regs->usbcmd);
	tmp |= USB_CMD_CTRL_RESET;
	socle_otg_writel(tmp, &dr_regs->usbcmd);

	/* Wait for reset to complete */
	timeout = jiffies + SOCLE_OTG_UDC_RESET_TIMEOUT;
	while (socle_otg_readl(&dr_regs->usbcmd) & USB_CMD_CTRL_RESET) {
		if (time_after(jiffies, timeout)) {
			ERR("udc reset timeout! \n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	//socle_otg_writel(0x3, &dr_regs->res1[144]);
	//printk("\n&dr_regs->res1[144] = 0x%08x\n", &dr_regs->res1[144]);
	//printk("\n&dr_regs->usbmode = 0x%08x\n", &dr_regs->usbmode);

	/* Set the controller as device mode */
	tmp = socle_otg_readl(&dr_regs->usbmode);
	tmp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	tmp |= USB_MODE_SETUP_LOCK_OFF;
	socle_otg_writel(tmp, &dr_regs->usbmode);

	//printk("\n\nINCR = 0x%08x : dr_controller_setup", socle_otg_readl(&dr_regs->res1[144]));

	/* Clear the setup status */
	socle_otg_writel(0, &dr_regs->usbsts);

	tmp = udc->ep_qh_dma;
	tmp &= USB_EP_LIST_ADDRESS_MASK;
	socle_otg_writel(tmp, &dr_regs->endpointlistaddr);

	VDBG("vir[qh_base] is %p phy[qh_base] is 0x%8x reg is 0x%8x",
		(int)udc->ep_qh, (int)tmp,
		socle_otg_readl(&dr_regs->endpointlistaddr));

	/* Config PHY interface */
	portctrl = socle_otg_readl(&dr_regs->portsc1);
	portctrl &= ~(PORTSCX_PHY_TYPE_SEL & PORTSCX_PORT_WIDTH);
	switch (udc->phy_mode) {
	case SOCLE_USB2_PHY_ULPI:
		portctrl |= PORTSCX_PTS_ULPI;
		break;
	case SOCLE_USB2_PHY_UTMI_WIDE:
		portctrl |= PORTSCX_PTW_16BIT;
		/* fall through */
	case SOCLE_USB2_PHY_UTMI:
		portctrl |= PORTSCX_PTS_UTMI;
		break;
	case SOCLE_USB2_PHY_SERIAL:
		portctrl |= PORTSCX_PTS_FSLS;
		break;
	default:
		return -EINVAL;
	}

#ifdef SOCLE_OTG_FORCE_FULL
	portctrl = portctrl | PORTSCX_PORT_FORCE_FULL_SPEED;
#endif
	socle_otg_writel(portctrl, &dr_regs->portsc1);

	/* Config control enable i/o output, cpu endian register */
	ctrl = __raw_readl(&usb_sys_regs->control);
	ctrl |= USB_CTRL_IOENB;
	__raw_writel(ctrl, &usb_sys_regs->control);

#if defined(CONFIG_PPC32) && !defined(CONFIG_NOT_COHERENT_CACHE)
	/* Turn on cache snooping hardware, since some PowerPC platforms
	 * wholly rely on hardware to deal with cache coherent. */

	/* Setup Snooping for all the 4GB space */
	tmp = SNOOP_SIZE_2GB;	/* starts from 0x0, size 2G */
	__raw_writel(tmp, &usb_sys_regs->snoop1);
	tmp |= 0x80000000;	/* starts from 0x8000000, size 2G */
	__raw_writel(tmp, &usb_sys_regs->snoop2);
#endif

	return 0;
}

/* Enable DR irq and set controller to run state */
static void dr_controller_run(struct socle_otg_udc *udc)
{
	u32 temp;

	/* Enable DR irq reg */
	temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
		| USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
		| USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

	socle_otg_writel(temp, &dr_regs->usbintr);

	/* Clear stopped bit */
	udc->stopped = 0;

	/* Set the controller as device mode */
	temp = socle_otg_readl(&dr_regs->usbmode);
	temp |= USB_MODE_CTRL_MODE_DEVICE;
	socle_otg_writel(temp, &dr_regs->usbmode);
	
	//printk("\n\nINCR = 0x%08x  :  dr_controller_run\n", socle_otg_readl(&dr_regs->res1[144]));

	/* Set controller to Run */
	temp = socle_otg_readl(&dr_regs->usbcmd);
	temp |= USB_CMD_RUN_STOP;
	socle_otg_writel(temp, &dr_regs->usbcmd);

	return;
}

static void dr_controller_stop(struct socle_otg_udc *udc)
{
	unsigned int tmp;

	/* disable all INTR */
	socle_otg_writel(0, &dr_regs->usbintr);

	/* Set stopped bit for isr */
	udc->stopped = 1;

	/* disable IO output */
/*	usb_sys_regs->control = 0; */

	/* set controller to Stop */
	tmp = socle_otg_readl(&dr_regs->usbcmd);
	tmp &= ~USB_CMD_RUN_STOP;
	socle_otg_writel(tmp, &dr_regs->usbcmd);

	return;
}

void dr_ep_setup(unsigned char ep_num, unsigned char dir, unsigned char ep_type)
{
	unsigned int tmp_epctrl = 0;

	//printk("ep_num = %d, dir = %d, ep_type = %d\n", ep_num, dir, ep_type);

	tmp_epctrl = socle_otg_readl(&dr_regs->endptctrl[ep_num]);
	if (dir) {
		if (ep_num)
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_TX_ENABLE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_TX_EP_TYPE_SHIFT);
	} else {
		if (ep_num)
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_RX_ENABLE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_RX_EP_TYPE_SHIFT);
	}

	socle_otg_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

static void
dr_ep_change_stall(unsigned char ep_num, unsigned char dir, int value)
{
	u32 tmp_epctrl = 0;

	tmp_epctrl = socle_otg_readl(&dr_regs->endptctrl[ep_num]);

	if (value) {
		/* set the stall bit */
		if (dir)
			tmp_epctrl |= EPCTRL_TX_EP_STALL;
		else
			tmp_epctrl |= EPCTRL_RX_EP_STALL;
	} else {
		/* clear the stall bit and reset data toggle */
		if (dir) {
			tmp_epctrl &= ~EPCTRL_TX_EP_STALL;
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		} else {
			tmp_epctrl &= ~EPCTRL_RX_EP_STALL;
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		}
	}
	socle_otg_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

/* Get stall status of a specific ep
   Return: 0: not stalled; 1:stalled */
static int dr_ep_get_stall(unsigned char ep_num, unsigned char dir)
{
	u32 epctrl;

	epctrl = socle_otg_readl(&dr_regs->endptctrl[ep_num]);
	if (dir)
		return (epctrl & EPCTRL_TX_EP_STALL) ? 1 : 0;
	else
		return (epctrl & EPCTRL_RX_EP_STALL) ? 1 : 0;
}

/********************************************************************
	Internal Structure Build up functions
********************************************************************/

/*------------------------------------------------------------------
* struct_ep_qh_setup(): set the Endpoint Capabilites field of QH
 * @zlt: Zero Length Termination Select (1: disable; 0: enable)
 * @mult: Mult field
 ------------------------------------------------------------------*/
static void struct_ep_qh_setup(struct socle_otg_udc *udc, unsigned char ep_num,
		unsigned char dir, unsigned char ep_type,
		unsigned int max_pkt_len,
		unsigned int zlt, unsigned char mult)
{
	struct ep_queue_head *p_QH = &udc->ep_qh[2 * ep_num + dir];
	unsigned int tmp = 0;

	/* set the Endpoint Capabilites in QH */
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		/* Interrupt On Setup (IOS). for control ep  */
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| EP_QUEUE_HEAD_IOS;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| (mult << EP_QUEUE_HEAD_MULT_POS);
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		tmp = max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS;
		break;
	default:
		VDBG("error ep type is %d", ep_type);
		return;
	}
	if (zlt)
		tmp |= EP_QUEUE_HEAD_ZLT_SEL;
	p_QH->max_pkt_length = cpu_to_le32(tmp);

	return;
}

/* Setup qh structure and ep register for ep0. */
static void ep0_setup(struct socle_otg_udc *udc)
{
	/* the intialization of an ep includes: fields in QH, Regs,
	 * socle_otg_ep struct */
	struct_ep_qh_setup(udc, 0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 0, 0);
	struct_ep_qh_setup(udc, 0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 0, 0);
	dr_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
	dr_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);

	return;

}

/***********************************************************************
		Endpoint Management Functions
***********************************************************************/

/*-------------------------------------------------------------------------
 * when configurations are set, or when interface settings change
 * for example the do_set_interface() in gadget layer,
 * the driver will enable or disable the relevant endpoints
 * ep0 doesn't use this routine. It is always enabled.
-------------------------------------------------------------------------*/
static int socle_otg_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct socle_otg_udc *udc = NULL;
	struct socle_otg_ep *ep = NULL;
	unsigned short max = 0;
	unsigned char mult = 0, zlt;
	int retval = -EINVAL;
	unsigned long flags = 0;

	ep = container_of(_ep, struct socle_otg_ep, ep);

	/* catch various bogus parameters */
	if (!_ep || !desc || ep->desc
			|| (desc->bDescriptorType != USB_DT_ENDPOINT))
		return -EINVAL;

	udc = ep->udc;

	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize);

	/* Disable automatic zlp generation.  Driver is reponsible to indicate
	 * explicitly through req->req.zero.  This is needed to enable multi-td
	 * request. */
	zlt = 1;

	/* Assume the max packet size from gadget is always correct */
	switch (desc->bmAttributes & 0x03) {
	case USB_ENDPOINT_XFER_CONTROL:
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		/* mult = 0.  Execute N Transactions as demonstrated by
		 * the USB variable length packet protocol where N is
		 * computed using the Maximum Packet Length (dQH) and
		 * the Total Bytes field (dTD) */
		mult = 0;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		/* Calculate transactions needed for high bandwidth iso */
		mult = (unsigned char)(1 + ((max >> 11) & 0x03));
		max = max & 0x8ff;	/* bit 0~10 */
		/* 3 transactions at most */
		if (mult > 3)
			goto en_done;
		break;
	default:
		goto en_done;
	}

	spin_lock_irqsave(&udc->lock, flags);
	ep->ep.maxpacket = max;
	ep->desc = desc;
	ep->stopped = 0;

	/* Controller related setup */
	/* Init EPx Queue Head (Ep Capabilites field in QH
	 * according to max, zlt, mult) */
	struct_ep_qh_setup(udc, (unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					?  USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK),
			max, zlt, mult);

	/* Init endpoint ctrl register */
	dr_ep_setup((unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					? USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK));

	spin_unlock_irqrestore(&udc->lock, flags);
	retval = 0;

	VDBG("enabled %s (ep%d%s) maxpacket %d",ep->ep.name,
			ep->desc->bEndpointAddress & 0x0f,
			(desc->bEndpointAddress & USB_DIR_IN)
				? "in" : "out", max);
en_done:
	return retval;
}

/*---------------------------------------------------------------------
 * @ep : the ep being unconfigured. May not be ep0
 * Any pending and uncomplete req will complete with status (-ESHUTDOWN)
*---------------------------------------------------------------------*/
static int socle_otg_ep_disable(struct usb_ep *_ep)
{
	struct socle_otg_udc *udc = NULL;
	struct socle_otg_ep *ep = NULL;
	unsigned long flags = 0;
	u32 epctrl;
	int ep_num;

	ep = container_of(_ep, struct socle_otg_ep, ep);
	if (!_ep || !ep->desc) {
		VDBG("%s not enabled", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	/* disable ep on controller */
	ep_num = ep_index(ep);
	epctrl = socle_otg_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl &= ~EPCTRL_TX_ENABLE;
	else
		epctrl &= ~EPCTRL_RX_ENABLE;
	socle_otg_writel(epctrl, &dr_regs->endptctrl[ep_num]);

	udc = (struct socle_otg_udc *)ep->udc;
	spin_lock_irqsave(&udc->lock, flags);

	/* nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;
	spin_unlock_irqrestore(&udc->lock, flags);

	VDBG("disabled %s OK", _ep->name);
	return 0;
}

/*---------------------------------------------------------------------
 * allocate a request object used by this endpoint
 * the main operation is to insert the req->queue to the eq->queue
 * Returns the request, or null if one could not be allocated
*---------------------------------------------------------------------*/
static struct usb_request *
socle_otg_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct socle_otg_req *req = NULL;

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void socle_otg_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct socle_otg_req *req = NULL;

	req = container_of(_req, struct socle_otg_req, req);

	if (_req)
		kfree(req);
}

/*-------------------------------------------------------------------------*/
static int socle_otg_queue_td(struct socle_otg_ep *ep, struct socle_otg_req *req)
{
	int i = ep_index(ep) * 2 + ep_is_in(ep);
	u32 temp, bitmask, tmp_stat;
	struct ep_queue_head *dQH = &ep->udc->ep_qh[i];

	/* VDBG("QH addr Register 0x%8x", dr_regs->endpointlistaddr);
	VDBG("ep_qh[%d] addr is 0x%8x", i, (u32)&(ep->udc->ep_qh[i])); */

	bitmask = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));

	/* check if the pipe is empty */
	if (!(list_empty(&ep->queue))) {
		/* Add td to the end */
		struct socle_otg_req *lastreq;
		lastreq = list_entry(ep->queue.prev, struct socle_otg_req, queue);
		lastreq->tail->next_td_ptr =
			cpu_to_le32(req->head->td_dma & DTD_ADDR_MASK);
		/* Read prime bit, if 1 goto done */
		if (socle_otg_readl(&dr_regs->endpointprime) & bitmask)
			goto out;

		do {
			/* Set ATDTW bit in USBCMD */
			temp = socle_otg_readl(&dr_regs->usbcmd);
			socle_otg_writel(temp | USB_CMD_ATDTW, &dr_regs->usbcmd);

			/* Read correct status bit */
			tmp_stat = socle_otg_readl(&dr_regs->endptstatus) & bitmask;

		} while (!(socle_otg_readl(&dr_regs->usbcmd) & USB_CMD_ATDTW));

		/* Write ATDTW bit to 0 */
		temp = socle_otg_readl(&dr_regs->usbcmd);
		socle_otg_writel(temp & ~USB_CMD_ATDTW, &dr_regs->usbcmd);

		if (tmp_stat)
			goto out;
	}

	/* Write dQH next pointer and terminate bit to 0 */
	temp = req->head->td_dma & EP_QUEUE_HEAD_NEXT_POINTER_MASK;
	dQH->next_dtd_ptr = cpu_to_le32(temp);

	/* Clear active and halt bit */
	temp = cpu_to_le32(~(EP_QUEUE_HEAD_STATUS_ACTIVE
			| EP_QUEUE_HEAD_STATUS_HALT));
	dQH->size_ioc_int_sts &= temp;

	/* Prime endpoint by writing 1 to ENDPTPRIME */
	temp = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));
	socle_otg_writel(temp, &dr_regs->endpointprime);
out:
	return 0;
}

/* Fill in the dTD structure
 * @req: request that the transfer belongs to
 * @length: return actually data length of the dTD
 * @dma: return dma address of the dTD
 * @is_last: return flag if it is the last dTD of the request
 * return: pointer to the built dTD */
static struct ep_td_struct *socle_otg_build_dtd(struct socle_otg_req *req, unsigned *length,
		dma_addr_t *dma, int *is_last)
{
	u32 swap_temp;
	struct ep_td_struct *dtd;

	/* how big will this transfer be? */
	*length = min(req->req.length - req->req.actual,
			(unsigned)EP_MAX_LENGTH_TRANSFER);

	dtd = dma_pool_alloc(udc_controller->td_pool, GFP_KERNEL, dma);
	if (dtd == NULL)
		return dtd;

	dtd->td_dma = *dma;
	/* Clear reserved field */
	swap_temp = cpu_to_le32(dtd->size_ioc_sts);
	swap_temp &= ~DTD_RESERVED_FIELDS;
	dtd->size_ioc_sts = cpu_to_le32(swap_temp);

	/* Init all of buffer page pointers */
	swap_temp = (u32) (req->req.dma + req->req.actual);
	dtd->buff_ptr0 = cpu_to_le32(swap_temp);
	dtd->buff_ptr1 = cpu_to_le32(swap_temp + 0x1000);
	dtd->buff_ptr2 = cpu_to_le32(swap_temp + 0x2000);
	dtd->buff_ptr3 = cpu_to_le32(swap_temp + 0x3000);
	dtd->buff_ptr4 = cpu_to_le32(swap_temp + 0x4000);

	req->req.actual += *length;

	/* zlp is needed if req->req.zero is set */
	if (req->req.zero) {
		if (*length == 0 || (*length % req->ep->ep.maxpacket) != 0)
			*is_last = 1;
		else
			*is_last = 0;
	} else if (req->req.length == req->req.actual)
		*is_last = 1;
	else
		*is_last = 0;

	if ((*is_last) == 0)
		VDBG("multi-dtd request!\n");
	/* Fill in the transfer size; set active bit */
	swap_temp = ((*length << DTD_LENGTH_BIT_POS) | DTD_STATUS_ACTIVE);

	/* Enable interrupt for the last dtd of a request */
	if (*is_last && !req->req.no_interrupt)
		swap_temp |= DTD_IOC;

	dtd->size_ioc_sts = cpu_to_le32(swap_temp);

	mb();

	VDBG("length = %d address= 0x%x", *length, (int)*dma);

	return dtd;
}

/* Generate dtd chain for a request */
static int socle_otg_req_to_dtd(struct socle_otg_req *req)
{
	unsigned	count;
	int		is_last;
	int		is_first =1;
	struct ep_td_struct	*last_dtd = NULL, *dtd;
	dma_addr_t dma;

	do {
		dtd = socle_otg_build_dtd(req, &count, &dma, &is_last);
		if (dtd == NULL)
			return -ENOMEM;

		if (is_first) {
			is_first = 0;
			req->head = dtd;
		} else {
			last_dtd->next_td_ptr = cpu_to_le32(dma);
			last_dtd->next_td_virt = dtd;
		}
		last_dtd = dtd;

		req->dtd_count++;
	} while (!is_last);

	dtd->next_td_ptr = cpu_to_le32(DTD_NEXT_TERMINATE);

	req->tail = dtd;

	return 0;
}

/* queues (submits) an I/O request to an endpoint */
static int
socle_otg_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct socle_otg_ep *ep = container_of(_ep, struct socle_otg_ep, ep);
	struct socle_otg_req *req = container_of(_req, struct socle_otg_req, req);
	struct socle_otg_udc *udc;
	unsigned long flags;
	int is_iso = 0;

	/* catch various bogus parameters */
	if (!_req || !req->req.complete || !req->req.buf
			|| !list_empty(&req->queue)) {
		VDBG("%s, bad params\n", __FUNCTION__);
		return -EINVAL;
	}
	if (!_ep || (!ep->desc && ep_index(ep))) {
		VDBG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}
	if (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		if (req->req.length > ep->ep.maxpacket)
			return -EMSGSIZE;
		is_iso = 1;
	}

	udc = ep->udc;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	req->ep = ep;

	/* map virtual address to hardware */
	if (req->req.dma == DMA_ADDR_INVALID) {
		req->req.dma = dma_map_single(ep->udc->gadget.dev.parent,
					req->req.buf,
					req->req.length, ep_is_in(ep)
						? DMA_TO_DEVICE
						: DMA_FROM_DEVICE);
		req->mapped = 1;
	} else {
		dma_sync_single_for_device(ep->udc->gadget.dev.parent,
					req->req.dma, req->req.length,
					ep_is_in(ep)
						? DMA_TO_DEVICE
						: DMA_FROM_DEVICE);
		req->mapped = 0;
	}

	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->dtd_count = 0;

	spin_lock_irqsave(&udc->lock, flags);

	/* build dtds and push them to device queue */
	if (!socle_otg_req_to_dtd(req)) {
		socle_otg_queue_td(ep, req);
	} else {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -ENOMEM;
	}

	/* Update ep0 state */
	if ((ep_index(ep) == 0))
		udc->ep0_state = DATA_STATE_XMIT;

	/* irq handler advances the queue */
	if (req != NULL)
		list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

/* dequeues (cancels, unlinks) an I/O request from an endpoint */
static int socle_otg_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct socle_otg_ep *ep = container_of(_ep, struct socle_otg_ep, ep);
	struct socle_otg_req *req;
	unsigned long flags;
	int ep_num, stopped, ret = 0;
	u32 epctrl;

	if (!_ep || !_req)
		return -EINVAL;

	spin_lock_irqsave(&ep->udc->lock, flags);
	stopped = ep->stopped;

	/* Stop the ep before we deal with the queue */
	ep->stopped = 1;
	ep_num = ep_index(ep);
	epctrl = socle_otg_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl &= ~EPCTRL_TX_ENABLE;
	else
		epctrl &= ~EPCTRL_RX_ENABLE;
	socle_otg_writel(epctrl, &dr_regs->endptctrl[ep_num]);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		ret = -EINVAL;
		goto out;
	}

	/* The request is in progress, or completed but not dequeued */
	if (ep->queue.next == &req->queue) {
		_req->status = -ECONNRESET;
		socle_otg_ep_fifo_flush(_ep);	/* flush current transfer */

		/* The request isn't the last request in this ep queue */
		if (req->queue.next != &ep->queue) {
			struct ep_queue_head *qh;
			struct socle_otg_req *next_req;

			qh = ep->qh;
			next_req = list_entry(req->queue.next, struct socle_otg_req,
					queue);

			/* Point the QH to the first TD of next request */
			socle_otg_writel((u32) next_req->head, &qh->curr_dtd_ptr);
		}

		/* The request hasn't been processed, patch up the TD chain */
	} else {
		struct socle_otg_req *prev_req;

		prev_req = list_entry(req->queue.prev, struct socle_otg_req, queue);
		socle_otg_writel(socle_otg_readl(&req->tail->next_td_ptr),
				&prev_req->tail->next_td_ptr);

	}

	done(ep, req, -ECONNRESET);

	/* Enable EP */
out:	epctrl = socle_otg_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl |= EPCTRL_TX_ENABLE;
	else
		epctrl |= EPCTRL_RX_ENABLE;
	socle_otg_writel(epctrl, &dr_regs->endptctrl[ep_num]);
	ep->stopped = stopped;

	spin_unlock_irqrestore(&ep->udc->lock, flags);
	return ret;
}

/*-------------------------------------------------------------------------*/

/*-----------------------------------------------------------------
 * modify the endpoint halt feature
 * @ep: the non-isochronous endpoint being stalled
 * @value: 1--set halt  0--clear halt
 * Returns zero, or a negative error code.
*----------------------------------------------------------------*/
static int socle_otg_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct socle_otg_ep *ep = NULL;
	unsigned long flags = 0;
	int status = -EOPNOTSUPP;	/* operation not supported */
	unsigned char ep_dir = 0, ep_num = 0;
	struct socle_otg_udc *udc = NULL;

	ep = container_of(_ep, struct socle_otg_ep, ep);
	udc = ep->udc;
	if (!_ep || !ep->desc) {
		status = -EINVAL;
		goto out;
	}

	if (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		status = -EOPNOTSUPP;
		goto out;
	}

	/* Attempt to halt IN ep will fail if any transfer requests
	 * are still queue */
	if (value && ep_is_in(ep) && !list_empty(&ep->queue)) {
		status = -EAGAIN;
		goto out;
	}

	status = 0;
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;
	ep_num = (unsigned char)(ep_index(ep));
	spin_lock_irqsave(&ep->udc->lock, flags);
	dr_ep_change_stall(ep_num, ep_dir, value);
	spin_unlock_irqrestore(&ep->udc->lock, flags);

	if (ep_index(ep) == 0) {
		udc->ep0_state = WAIT_FOR_SETUP;
		udc->ep0_dir = 0;
	}
out:
	VDBG(" %s %s halt stat %d", ep->ep.name,
			value ?  "set" : "clear", status);

	return status;
}

static void socle_otg_ep_fifo_flush(struct usb_ep *_ep)
{
	struct socle_otg_ep *ep;
	int ep_num, ep_dir;
	u32 bits;
	unsigned long timeout;
#define SOCLE_OTG_UDC_FLUSH_TIMEOUT 1000

	if (!_ep) {
		return;
	} else {
		ep = container_of(_ep, struct socle_otg_ep, ep);
		if (!ep->desc)
			return;
	}
	ep_num = ep_index(ep);
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;

	if (ep_num == 0)
		bits = (1 << 16) | 1;
	else if (ep_dir == USB_SEND)
		bits = 1 << (16 + ep_num);
	else
		bits = 1 << ep_num;

	timeout = jiffies + SOCLE_OTG_UDC_FLUSH_TIMEOUT;
	do {
		socle_otg_writel(bits, &dr_regs->endptflush);

		/* Wait until flush complete */
		while (socle_otg_readl(&dr_regs->endptflush)) {
			if (time_after(jiffies, timeout)) {
				ERR("ep flush timeout\n");
				return;
			}
			cpu_relax();
		}
		/* See if we need to flush again */
	} while (socle_otg_readl(&dr_regs->endptstatus) & bits);
}

static struct usb_ep_ops socle_otg_ep_ops = {
	.enable = socle_otg_ep_enable,
	.disable = socle_otg_ep_disable,

	.alloc_request = socle_otg_alloc_request,
	.free_request = socle_otg_free_request,

	.queue = socle_otg_ep_queue,
	.dequeue = socle_otg_ep_dequeue,

	.set_halt = socle_otg_ep_set_halt,
	.fifo_flush = socle_otg_ep_fifo_flush,	/* flush fifo */
};

/*-------------------------------------------------------------------------
		Gadget Driver Layer Operations
-------------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 * Get the current frame number (from DR frame_index Reg )
 *----------------------------------------------------------------------*/
static int socle_otg_get_frame(struct usb_gadget *gadget)
{
	return (int)(socle_otg_readl(&dr_regs->frindex) & USB_FRINDEX_MASKS);
}

/*-----------------------------------------------------------------------
 * Tries to wake up the host connected to this gadget
 -----------------------------------------------------------------------*/
static int socle_otg_wakeup(struct usb_gadget *gadget)
{
	struct socle_otg_udc *udc = container_of(gadget, struct socle_otg_udc, gadget);
	u32 portsc;

	/* Remote wakeup feature not enabled by host */
	if (!udc->remote_wakeup)
		return -ENOTSUPP;

	portsc = socle_otg_readl(&dr_regs->portsc1);
	/* not suspended? */
	if (!(portsc & PORTSCX_PORT_SUSPEND))
		return 0;
	/* trigger force resume */
	portsc |= PORTSCX_PORT_FORCE_RESUME;
#ifdef SOCLE_OTG_FORCE_FULL
	portsc = portsc | PORTSCX_PORT_FORCE_FULL_SPEED;
#endif
	socle_otg_writel(portsc, &dr_regs->portsc1);
	return 0;
}

static int can_pullup(struct socle_otg_udc *udc)
{
//	return (udc->driver && udc->softconnect && udc->vbus_active) ;
	return (udc->driver && udc->softconnect) ;		//20090527 leonid+ for connect/disconnect
}

/* Notify controller that VBUS is powered, Called by whatever
   detects VBUS sessions */
static int socle_otg_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct socle_otg_udc	*udc;
	unsigned long	flags;

	udc = container_of(gadget, struct socle_otg_udc, gadget);
	spin_lock_irqsave(&udc->lock, flags);
	VDBG("VBUS %s\n", is_active ? "on" : "off");
	udc->vbus_active = (is_active != 0);
	if (can_pullup(udc))
		socle_otg_writel((socle_otg_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	else
		socle_otg_writel((socle_otg_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

/* constrain controller's VBUS power usage
 * This call is used by gadget drivers during SET_CONFIGURATION calls,
 * reporting how much power the device may consume.  For example, this
 * could affect how quickly batteries are recharged.
 *
 * Returns zero on success, else negative errno.
 */
static int socle_otg_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
#ifdef CONFIG_USB_OTG
	struct socle_otg_udc *udc;

	//printk("socle_otg_vbus_draw -- CONFIG_USB_OTG\n");	

	udc = container_of(gadget, struct socle_otg_udc, gadget);

	if (udc->transceiver)
		return otg_set_power(udc->transceiver, mA);
#endif
	return -ENOTSUPP;
}

/* Change Data+ pullup status
 * this func is used by usb_gadget_connect/disconnet
 */
static int socle_otg_pullup(struct usb_gadget *gadget, int is_on)
{
	struct socle_otg_udc *udc;

	udc = container_of(gadget, struct socle_otg_udc, gadget);
	udc->softconnect = (is_on != 0);
	if (can_pullup(udc))
		socle_otg_writel((socle_otg_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	else
		socle_otg_writel((socle_otg_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);

	return 0;
}

/* defined in usb_gadget.h */
static struct usb_gadget_ops socle_otg_gadget_ops = {
	.get_frame = socle_otg_get_frame,
	.wakeup = socle_otg_wakeup,
/*	.set_selfpowered = socle_otg_set_selfpowered,	*/ /* Always selfpowered */
	.vbus_session = socle_otg_vbus_session,
	.vbus_draw = socle_otg_vbus_draw,
	.pullup = socle_otg_pullup,
};

/* Set protocol stall on ep0, protocol stall will automatically be cleared
   on new transaction */
static void ep0stall(struct socle_otg_udc *udc)
{
	u32 tmp;

	/* must set tx and rx to stall at the same time */
	tmp = socle_otg_readl(&dr_regs->endptctrl[0]);
	tmp |= EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL;
	socle_otg_writel(tmp, &dr_regs->endptctrl[0]);
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;
}

/* Prime a status phase for ep0 */
static int ep0_prime_status(struct socle_otg_udc *udc, int direction)
{
	struct socle_otg_req *req = udc->status_req;
	struct socle_otg_ep *ep;
	int status = 0;

	if (direction == EP_DIR_IN)
		udc->ep0_dir = USB_DIR_IN;
	else
		udc->ep0_dir = USB_DIR_OUT;

	ep = &udc->eps[0];
	udc->ep0_state = WAIT_FOR_OUT_STATUS;

	req->ep = ep;
	req->req.length = 0;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	if (socle_otg_req_to_dtd(req) == 0)
		status = socle_otg_queue_td(ep, req);
	else
		return -ENOMEM;

	if (status)
		ERR("Can't queue ep0 status request \n");
	list_add_tail(&req->queue, &ep->queue);

	return status;
}

static inline int udc_reset_ep_queue(struct socle_otg_udc *udc, u8 pipe)
{
	struct socle_otg_ep *ep = get_ep_by_pipe(udc, pipe);

	if (!ep->name)
		return 0;

	nuke(ep, -ESHUTDOWN);

	return 0;
}

/*
 * ch9 Set address
 */
static void ch9setaddress(struct socle_otg_udc *udc, u16 value, u16 index, u16 length)
{
	/* Save the new address to device struct */
	udc->device_address = (u8) value;
	/* Update usb state */
	udc->usb_state = USB_STATE_ADDRESS;
	/* Status phase */
	if (ep0_prime_status(udc, EP_DIR_IN))
		ep0stall(udc);
}

/*
 * ch9 Get status
 */
static void ch9getstatus(struct socle_otg_udc *udc, u8 request_type, u16 value,
		u16 index, u16 length)
{
	u16 tmp = 0;		/* Status, cpu endian */

	struct socle_otg_req *req;
	struct socle_otg_ep *ep;
	int status = 0;

	ep = &udc->eps[0];

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* Get device status */
		tmp = 1 << USB_DEVICE_SELF_POWERED;
		tmp |= udc->remote_wakeup << USB_DEVICE_REMOTE_WAKEUP;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		/* Get interface status */
		/* We don't have interface information in udc driver */
		tmp = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		/* Get endpoint status */
		struct socle_otg_ep *target_ep;

		target_ep = get_ep_by_pipe(udc, get_pipe_by_windex(index));

		/* stall if endpoint doesn't exist */
		if (!target_ep->desc)
			goto stall;
		tmp = dr_ep_get_stall(ep_index(target_ep), ep_is_in(target_ep))
				<< USB_ENDPOINT_HALT;
	}

	udc->ep0_dir = USB_DIR_IN;
	/* Borrow the per device status_req */
	req = udc->status_req;
	/* Fill in the reqest structure */
	*((u16 *) req->req.buf) = cpu_to_le16(tmp);
	req->ep = ep;
	req->req.length = 2;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	/* prime the data phase */
	if ((socle_otg_req_to_dtd(req) == 0))
		status = socle_otg_queue_td(ep, req);
	else			/* no mem */
		goto stall;

	if (status) {
		ERR("Can't respond to getstatus request \n");
		goto stall;
	}
	list_add_tail(&req->queue, &ep->queue);
	udc->ep0_state = DATA_STATE_XMIT;
	return;
stall:
	ep0stall(udc);
}

static void setup_received_irq(struct socle_otg_udc *udc,
		struct usb_ctrlrequest *setup)
{
	u16 wValue = le16_to_cpu(setup->wValue);
	u16 wIndex = le16_to_cpu(setup->wIndex);
	u16 wLength = le16_to_cpu(setup->wLength);

	udc_reset_ep_queue(udc, 0);

	switch (setup->bRequest) {
		/* Request that need Data+Status phase from udc */
	case USB_REQ_GET_STATUS:
		if ((setup->bRequestType & (USB_DIR_IN | USB_TYPE_STANDARD))
					!= (USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		ch9getstatus(udc, setup->bRequestType, wValue, wIndex, wLength);
		break;

		/* Requests that need Status phase from udc */
	case USB_REQ_SET_ADDRESS:
		if (setup->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD
						| USB_RECIP_DEVICE))
			break;
		ch9setaddress(udc, wValue, wIndex, wLength);
		break;

		/* Handled by udc, no data, status by udc */
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
	{	/* status transaction */
		int rc = -EOPNOTSUPP;

		if ((setup->bRequestType & USB_RECIP_MASK)
				== USB_RECIP_ENDPOINT) {
			int pipe = get_pipe_by_windex(wIndex);
			struct socle_otg_ep *ep;

			if (wValue != 0 || wLength != 0 || pipe > udc->max_ep)
				break;
			ep = get_ep_by_pipe(udc, pipe);

			spin_unlock(&udc->lock);
			rc = socle_otg_ep_set_halt(&ep->ep,
					(setup->bRequest == USB_REQ_SET_FEATURE)
						? 1 : 0);
			spin_lock(&udc->lock);

		} else if ((setup->bRequestType & USB_RECIP_MASK)
				== USB_RECIP_DEVICE) {
			/* Note: The driver has not include OTG support yet.
			 * This will be set when OTG support is added */
			if (!udc->gadget.is_otg)
				break;
			else if (setup->bRequest == USB_DEVICE_B_HNP_ENABLE)
				udc->gadget.b_hnp_enable = 1;
			else if (setup->bRequest == USB_DEVICE_A_HNP_SUPPORT)
				udc->gadget.a_hnp_support = 1;
			else if (setup->bRequest ==
					USB_DEVICE_A_ALT_HNP_SUPPORT)
				udc->gadget.a_alt_hnp_support = 1;
			rc = 0;
		}
		if (rc == 0) {
			if (ep0_prime_status(udc, EP_DIR_IN))
				ep0stall(udc);
		}
		break;
	}
		/* Requests handled by gadget */
	default:
		if (wLength) {
			/* Data phase from gadget, status phase from udc */
			udc->ep0_dir = (setup->bRequestType & USB_DIR_IN)
					?  USB_DIR_IN : USB_DIR_OUT;
			spin_unlock(&udc->lock);
			if (udc->driver->setup(&udc->gadget,
					&udc->local_setup_buff) < 0)
				ep0stall(udc);
			spin_lock(&udc->lock);
			udc->ep0_state = (setup->bRequestType & USB_DIR_IN)
					?  DATA_STATE_XMIT : DATA_STATE_RECV;

		} else {
			/* No data phase, IN status from gadget */
			udc->ep0_dir = USB_DIR_IN;
			spin_unlock(&udc->lock);
			if (udc->driver->setup(&udc->gadget,
					&udc->local_setup_buff) < 0)
				ep0stall(udc);
			spin_lock(&udc->lock);
			udc->ep0_state = WAIT_FOR_OUT_STATUS;
		}
		break;
	}
}

/* Process request for Data or Status phase of ep0
 * prime status phase if needed */
static void ep0_req_complete(struct socle_otg_udc *udc, struct socle_otg_ep *ep0,
		struct socle_otg_req *req)
{
	if (udc->usb_state == USB_STATE_ADDRESS) {
		/* Set the new address */
		u32 new_address = (u32) udc->device_address;
		socle_otg_writel(new_address << USB_DEVICE_ADDRESS_BIT_POS,
				&dr_regs->deviceaddr);
	}

	done(ep0, req, 0);

	switch (udc->ep0_state) {
	case DATA_STATE_XMIT:
		/* receive status phase */
		if (ep0_prime_status(udc, EP_DIR_OUT))
			ep0stall(udc);
		break;
	case DATA_STATE_RECV:
		/* send status phase */
		if (ep0_prime_status(udc, EP_DIR_IN))
			ep0stall(udc);
		break;
	case WAIT_FOR_OUT_STATUS:
		udc->ep0_state = WAIT_FOR_SETUP;
		break;
	case WAIT_FOR_SETUP:
		ERR("Unexpect ep0 packets \n");
		break;
	default:
		ep0stall(udc);
		break;
	}
}

/* Tripwire mechanism to ensure a setup packet payload is extracted without
 * being corrupted by another incoming setup packet */
static void tripwire_handler(struct socle_otg_udc *udc, u8 ep_num, u8 *buffer_ptr)
{
	u32 temp;
	struct ep_queue_head *qh;

	qh = &udc->ep_qh[ep_num * 2 + EP_DIR_OUT];

	/* Clear bit in ENDPTSETUPSTAT */
	temp = socle_otg_readl(&dr_regs->endptsetupstat);
	socle_otg_writel(temp | (1 << ep_num), &dr_regs->endptsetupstat);

	/* while a hazard exists when setup package arrives */
	do {
		/* Set Setup Tripwire */
		temp = socle_otg_readl(&dr_regs->usbcmd);
		socle_otg_writel(temp | USB_CMD_SUTW, &dr_regs->usbcmd);

		/* Copy the setup packet to local buffer */
		memcpy(buffer_ptr, (u8 *) qh->setup_buffer, 8);
	} while (!(socle_otg_readl(&dr_regs->usbcmd) & USB_CMD_SUTW));

	/* Clear Setup Tripwire */
	temp = socle_otg_readl(&dr_regs->usbcmd);
	socle_otg_writel(temp & ~USB_CMD_SUTW, &dr_regs->usbcmd);
}

/* process-ep_req(): free the completed Tds for this req */
static int process_ep_req(struct socle_otg_udc *udc, int pipe,
		struct socle_otg_req *curr_req)
{
	struct ep_td_struct *curr_td;
	int	td_complete, actual, remaining_length, j, tmp;
	int	status = 0;
	int	errors = 0;
	struct  ep_queue_head *curr_qh = &udc->ep_qh[pipe];
	int direction = pipe % 2;

	curr_td = curr_req->head;
	td_complete = 0;
	actual = curr_req->req.length;

	for (j = 0; j < curr_req->dtd_count; j++) {
		remaining_length = (le32_to_cpu(curr_td->size_ioc_sts)
					& DTD_PACKET_SIZE)
				>> DTD_LENGTH_BIT_POS;
		actual -= remaining_length;

		if ((errors = le32_to_cpu(curr_td->size_ioc_sts) &
						DTD_ERROR_MASK)) {
			if (errors & DTD_STATUS_HALTED) {
				ERR("dTD error %08x QH=%d\n", errors, pipe);
				/* Clear the errors and Halt condition */
				tmp = le32_to_cpu(curr_qh->size_ioc_int_sts);
				tmp &= ~errors;
				curr_qh->size_ioc_int_sts = cpu_to_le32(tmp);
				status = -EPIPE;
				/* FIXME: continue with next queued TD? */

				break;
			}
			if (errors & DTD_STATUS_DATA_BUFF_ERR) {
				VDBG("Transfer overflow");
				status = -EPROTO;
				break;
			} else if (errors & DTD_STATUS_TRANSACTION_ERR) {
				VDBG("ISO error");
				status = -EILSEQ;
				break;
			} else
				ERR("Unknown error has occured (0x%x)!\r\n",
					errors);

		} else if (le32_to_cpu(curr_td->size_ioc_sts)
				& DTD_STATUS_ACTIVE) {
			VDBG("Request not complete");
			status = REQ_UNCOMPLETE;
			return status;
		} else if (remaining_length) {
			if (direction) {
				VDBG("Transmit dTD remaining length not zero");
				status = -EPROTO;
				break;
			} else {
				td_complete++;
				break;
			}
		} else {
			td_complete++;
			VDBG("dTD transmitted successful ");
		}

		if (j != curr_req->dtd_count - 1)
			curr_td = (struct ep_td_struct *)curr_td->next_td_virt;
	}

	if (status)
		return status;

	curr_req->req.actual = actual;

	return 0;
}

/* Process a DTD completion interrupt */
static void dtd_complete_irq(struct socle_otg_udc *udc)
{
	u32 bit_pos;
	int i, ep_num, direction, bit_mask, status;
	struct socle_otg_ep *curr_ep;
	struct socle_otg_req *curr_req, *temp_req;

	/* Clear the bits in the register */
	bit_pos = socle_otg_readl(&dr_regs->endptcomplete);
	socle_otg_writel(bit_pos, &dr_regs->endptcomplete);

	if (!bit_pos)
		return;

	for (i = 0; i < udc->max_ep * 2; i++) {
		ep_num = i >> 1;
		direction = i % 2;

		bit_mask = 1 << (ep_num + 16 * direction);

		if (!(bit_pos & bit_mask))
			continue;

		curr_ep = get_ep_by_pipe(udc, i);

		/* If the ep is configured */
		if (curr_ep->name == NULL) {
			WARN_SOCLE("Invalid EP?");		//leonid fix for OTG warning message
			continue;
		}

		/* process the req queue until an uncomplete request */
		list_for_each_entry_safe(curr_req, temp_req, &curr_ep->queue,
				queue) {
			status = process_ep_req(udc, i, curr_req);

			VDBG("status of process_ep_req= %d, ep = %d",
					status, ep_num);
			if (status == REQ_UNCOMPLETE)
				break;
			/* write back status to req */
			curr_req->req.status = status;

			if (ep_num == 0) {
				ep0_req_complete(udc, curr_ep, curr_req);
				break;
			} else
				done(curr_ep, curr_req, status);
		}
	}
}

/* Process a port change interrupt */
static void port_change_irq(struct socle_otg_udc *udc)
{
	u32 speed;

	if (udc->bus_reset)
		udc->bus_reset = 0;

	/* Bus resetting is finished */
	if (!(socle_otg_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET)) {
		/* Get the speed */
		speed = (socle_otg_readl(&dr_regs->portsc1)
				& PORTSCX_PORT_SPEED_MASK);
		switch (speed) {
		case PORTSCX_PORT_SPEED_HIGH:
			udc->gadget.speed = USB_SPEED_HIGH;
			break;
		case PORTSCX_PORT_SPEED_FULL:
			udc->gadget.speed = USB_SPEED_FULL;
			break;
		case PORTSCX_PORT_SPEED_LOW:
			udc->gadget.speed = USB_SPEED_LOW;
			break;
		default:
			udc->gadget.speed = USB_SPEED_UNKNOWN;
			break;
		}
	}

	/* Update USB state */
	if (!udc->resume_state)
		udc->usb_state = USB_STATE_DEFAULT;
}

/* Process suspend interrupt */
static void suspend_irq(struct socle_otg_udc *udc)
{
	udc->resume_state = udc->usb_state;
	udc->usb_state = USB_STATE_SUSPENDED;

	/* report suspend to the driver, serial.c does not support this */
	if (udc->driver->suspend)
		udc->driver->suspend(&udc->gadget);
}

static void bus_resume(struct socle_otg_udc *udc)
{
	udc->usb_state = udc->resume_state;
	udc->resume_state = 0;

	/* report resume to the driver, serial.c does not support this */
	if (udc->driver->resume)
		udc->driver->resume(&udc->gadget);
}

/* Clear up all ep queues */
static int reset_queues(struct socle_otg_udc *udc)
{
	u8 pipe;

	for (pipe = 0; pipe < udc->max_pipes; pipe++)
		udc_reset_ep_queue(udc, pipe);

	/* report disconnect; the driver is already quiesced */
	udc->driver->disconnect(&udc->gadget);

	return 0;
}

/* Process reset interrupt */
static void reset_irq(struct socle_otg_udc *udc)
{
	u32 temp;
	unsigned long timeout;

	//printk("reset_irq\n");

	/* Clear the device address */
	temp = socle_otg_readl(&dr_regs->deviceaddr);
	socle_otg_writel(temp & ~USB_DEVICE_ADDRESS_MASK, &dr_regs->deviceaddr);

	udc->device_address = 0;

	/* Clear usb state */
	udc->resume_state = 0;
	udc->ep0_dir = 0;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->remote_wakeup = 0;	/* default to 0 on reset */
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

	/* Clear all the setup token semaphores */
	temp = socle_otg_readl(&dr_regs->endptsetupstat);
	socle_otg_writel(temp, &dr_regs->endptsetupstat);

	/* Clear all the endpoint complete status bits */
	temp = socle_otg_readl(&dr_regs->endptcomplete);
	socle_otg_writel(temp, &dr_regs->endptcomplete);

	timeout = jiffies + 100;
	while (socle_otg_readl(&dr_regs->endpointprime)) {
		/* Wait until all endptprime bits cleared */
		if (time_after(jiffies, timeout)) {
			ERR("Timeout for reset\n");
			break;
		}
		cpu_relax();
	}

	/* Write 1s to the flush register */
	socle_otg_writel(0xffffffff, &dr_regs->endptflush);

	if (socle_otg_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET) {
		VDBG("Bus reset");
		/* Bus is reseting */
		udc->bus_reset = 1;
		/* Reset all the queues, include XD, dTD, EP queue
		 * head and TR Queue */
		reset_queues(udc);
		udc->usb_state = USB_STATE_DEFAULT;
	} else {
		VDBG("Controller reset");
		/* initialize usb hw reg except for regs for EP, not
		 * touch usbintr reg */
		dr_controller_setup(udc);

		/* Reset all internal used Queues */
		reset_queues(udc);

		ep0_setup(udc);

		/* Enable DR IRQ reg, Set Run bit, change udc state */
		dr_controller_run(udc);
		udc->usb_state = USB_STATE_ATTACHED;
	}
}

/*
 * USB device controller interrupt handler
 */
static irqreturn_t socle_otg_udc_irq(int irq, void *_udc)
{
	struct socle_otg_udc *udc = _udc;
	u32 irq_src;
	irqreturn_t status = IRQ_NONE;
	unsigned long flags;

	//printk("socle_otg_udc_irq : 0x%08x\n", ioread32(IO_ADDRESS(SOCLE_OTG_UDC0) + 0x144));
	//printk("socle_otg_udc_irq\n");
	
	/* Disable ISR for OTG host mode */
	if (udc->stopped)
		return IRQ_NONE;
	spin_lock_irqsave(&udc->lock, flags);
	irq_src = socle_otg_readl(&dr_regs->usbsts) & socle_otg_readl(&dr_regs->usbintr);
	/* Clear notification bits */
	socle_otg_writel(irq_src, &dr_regs->usbsts);

	/* VDBG("irq_src [0x%8x]", irq_src); */

	/* Need to resume? */
	if (udc->usb_state == USB_STATE_SUSPENDED)
		if ((socle_otg_readl(&dr_regs->portsc1) & PORTSCX_PORT_SUSPEND) == 0)
			bus_resume(udc);

	/* USB Interrupt */
	if (irq_src & USB_STS_INT) {
		VDBG("Packet int");
		/* Setup package, we only support ep0 as control ep */
		if (socle_otg_readl(&dr_regs->endptsetupstat) & EP_SETUP_STATUS_EP0) {
			tripwire_handler(udc, 0,
					(u8 *) (&udc->local_setup_buff));
			setup_received_irq(udc, &udc->local_setup_buff);
			status = IRQ_HANDLED;
		}

		/* completion of dtd */
		if (socle_otg_readl(&dr_regs->endptcomplete)) {
			dtd_complete_irq(udc);
			status = IRQ_HANDLED;
		}
	}

	/* SOF (for ISO transfer) */
	if (irq_src & USB_STS_SOF) {
		status = IRQ_HANDLED;
	}

	/* Port Change */
	if (irq_src & USB_STS_PORT_CHANGE) {
		port_change_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Reset Received */
	if (irq_src & USB_STS_RESET) {
		reset_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Sleep Enable (Suspend) */
	if (irq_src & USB_STS_SUSPEND) {
		suspend_irq(udc);
		status = IRQ_HANDLED;
	}

	if (irq_src & (USB_STS_ERR | USB_STS_SYS_ERR)) {
		VDBG("Error IRQ %x ", irq_src);
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	return status;
}

/*----------------------------------------------------------------*
 * Hook to gadget drivers
 * Called by initialization code of gadget drivers
*----------------------------------------------------------------*/
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	int retval = -ENODEV;
	unsigned long flags = 0;
	
	if (!udc_controller){
		printk("!udc_controller\n");
		return -ENODEV;
	}
	
	if (!driver || (driver->speed != USB_SPEED_FULL
				&& driver->speed != USB_SPEED_HIGH)
			|| !driver->bind || !driver->disconnect
			|| !driver->setup){
		return -EINVAL;
		}

	if (udc_controller->driver){
		return -EBUSY;
		}
	
	/* lock is needed but whether should use this lock or another */
	spin_lock_irqsave(&udc_controller->lock, flags);

	driver->driver.bus = 0;
	/* hook up the driver */
	udc_controller->driver = driver;
	udc_controller->gadget.dev.driver = &driver->driver;
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	/* bind udc driver to gadget driver */
	retval = driver->bind(&udc_controller->gadget);
	if (retval) {
		VDBG("bind to %s --> %d", driver->driver.name, retval);
		udc_controller->gadget.dev.driver = 0;
		udc_controller->driver = 0;
		goto out;
	}

	/* Enable DR IRQ reg and Set usbcmd reg  Run bit */
	dr_controller_run(udc_controller);
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;
	printk(KERN_INFO "%s: bind to driver %s \n",
			udc_controller->gadget.name, driver->driver.name);

out:
	if (retval)
		printk("retval %d \n", retval);
	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

/* Disconnect from gadget driver */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct socle_otg_ep *loop_ep;
	unsigned long flags;
	
	if (!udc_controller)
		return -ENODEV;

	if (!driver || driver != udc_controller->driver || !driver->unbind)
		return -EINVAL;

#ifdef CONFIG_USB_OTG
	printk("usb_gadget_unregister_driver -- CONFIG_USB_OTG\n");		
	if (udc_controller->transceiver)
		(void)otg_set_peripheral(udc_controller->transceiver, 0);
#endif

	/* stop DR, disable intr */
	dr_controller_stop(udc_controller);

	/* in fact, no needed */
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;

	/* stand operation */
	spin_lock_irqsave(&udc_controller->lock, flags);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
	nuke(&udc_controller->eps[0], -ESHUTDOWN);
	list_for_each_entry(loop_ep, &udc_controller->gadget.ep_list,
			ep.ep_list)
		nuke(loop_ep, -ESHUTDOWN);
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	/* unbind gadget and unhook driver. */
	driver->unbind(&udc_controller->gadget);
	udc_controller->gadget.dev.driver = 0;
	udc_controller->driver = 0;

	printk("unregistered gadget driver '%s'\r\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------
		PROC File System Support
-------------------------------------------------------------------------*/
#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>

static const char proc_filename[] = "driver/socle_usb2_udc";

static int socle_otg_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char *buf = page;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t, i;
	u32 tmp_reg;
	struct socle_otg_ep *ep = NULL;
	struct socle_otg_req *req;

	struct socle_otg_udc *udc = udc_controller;
	if (off != 0)
		return 0;

	spin_lock_irqsave(&udc->lock, flags);

	/* ------basic driver infomation ---- */
	t = scnprintf(next, size,
			DRIVER_DESC "\n"
			"%s version: %s\n"
			"Gadget driver: %s\n\n",
			driver_name, DRIVER_VERSION,
			udc->driver ? udc->driver->driver.name : "(none)");
	size -= t;
	next += t;

	/* ------ DR Registers ----- */
	tmp_reg = socle_otg_readl(&dr_regs->usbcmd);
	t = scnprintf(next, size,
			"USBCMD reg:\n"
			"SetupTW: %d\n"
			"Run/Stop: %s\n\n",
			(tmp_reg & USB_CMD_SUTW) ? 1 : 0,
			(tmp_reg & USB_CMD_RUN_STOP) ? "Run" : "Stop");
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->usbsts);
	t = scnprintf(next, size,
			"USB Status Reg:\n"
			"Dr Suspend: %d" "Reset Received: %d" "System Error: %s"
			"USB Error Interrupt: %s\n\n",
			(tmp_reg & USB_STS_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_STS_RESET) ? 1 : 0,
			(tmp_reg & USB_STS_SYS_ERR) ? "Err" : "Normal",
			(tmp_reg & USB_STS_ERR) ? "Err detected" : "No err");
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->usbintr);
	t = scnprintf(next, size,
			"USB Intrrupt Enable Reg:\n"
			"Sleep Enable: %d" "SOF Received Enable: %d"
			"Reset Enable: %d\n"
			"System Error Enable: %d"
			"Port Change Dectected Enable: %d\n"
			"USB Error Intr Enable: %d" "USB Intr Enable: %d\n\n",
			(tmp_reg & USB_INTR_DEVICE_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_INTR_SOF_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_RESET_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_SYS_ERR_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_PTC_DETECT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_ERR_INT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_INT_EN) ? 1 : 0);
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->frindex);
	t = scnprintf(next, size,
			"USB Frame Index Reg:" "Frame Number is 0x%x\n\n",
			(tmp_reg & USB_FRINDEX_MASKS));
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->deviceaddr);
	t = scnprintf(next, size,
			"USB Device Address Reg:" "Device Addr is 0x%x\n\n",
			(tmp_reg & USB_DEVICE_ADDRESS_MASK));
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->endpointlistaddr);
	t = scnprintf(next, size,
			"USB Endpoint List Address Reg:"
			"Device Addr is 0x%x\n\n",
			(tmp_reg & USB_EP_LIST_ADDRESS_MASK));
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->portsc1);
	t = scnprintf(next, size,
		"USB Port Status&Control Reg:\n"
		"Port Transceiver Type : %s" "Port Speed: %s \n"
		"PHY Low Power Suspend: %s" "Port Reset: %s"
		"Port Suspend Mode: %s \n" "Over-current Change: %s"
		"Port Enable/Disable Change: %s\n"
		"Port Enabled/Disabled: %s"
		"Current Connect Status: %s\n\n", ( {
			char *s;
			switch (tmp_reg & PORTSCX_PTS_FSLS) {
			case PORTSCX_PTS_UTMI:
				s = "UTMI"; break;
			case PORTSCX_PTS_ULPI:
				s = "ULPI "; break;
			case PORTSCX_PTS_FSLS:
				s = "FS/LS Serial"; break;
			default:
				s = "None"; break;
			}
			s;} ), ( {
			char *s;
			switch (tmp_reg & PORTSCX_PORT_SPEED_UNDEF) {
			case PORTSCX_PORT_SPEED_FULL:
				s = "Full Speed"; break;
			case PORTSCX_PORT_SPEED_LOW:
				s = "Low Speed"; break;
			case PORTSCX_PORT_SPEED_HIGH:
				s = "High Speed"; break;
			default:
				s = "Undefined"; break;
			}
			s;
		} ),
		(tmp_reg & PORTSCX_PHY_LOW_POWER_SPD) ?
		"Normal PHY mode" : "Low power mode",
		(tmp_reg & PORTSCX_PORT_RESET) ? "In Reset" :
		"Not in Reset",
		(tmp_reg & PORTSCX_PORT_SUSPEND) ? "In " : "Not in",
		(tmp_reg & PORTSCX_OVER_CURRENT_CHG) ? "Dected" :
		"No",
		(tmp_reg & PORTSCX_PORT_EN_DIS_CHANGE) ? "Disable" :
		"Not change",
		(tmp_reg & PORTSCX_PORT_ENABLE) ? "Enable" :
		"Not correct",
		(tmp_reg & PORTSCX_CURRENT_CONNECT_STATUS) ?
		"Attached" : "Not-Att");
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->usbmode);
	t = scnprintf(next, size,
			"USB Mode Reg:" "Controller Mode is : %s\n\n", ( {
				char *s;
				switch (tmp_reg & USB_MODE_CTRL_MODE_HOST) {
				case USB_MODE_CTRL_MODE_IDLE:
					s = "Idle"; break;
				case USB_MODE_CTRL_MODE_DEVICE:
					s = "Device Controller"; break;
				case USB_MODE_CTRL_MODE_HOST:
					s = "Host Controller"; break;
				default:
					s = "None"; break;
				}
				s;
			} ));
	size -= t;
	next += t;

	tmp_reg = socle_otg_readl(&dr_regs->endptsetupstat);
	t = scnprintf(next, size,
			"Endpoint Setup Status Reg:" "SETUP on ep 0x%x\n\n",
			(tmp_reg & EP_SETUP_STATUS_MASK));
	size -= t;
	next += t;

	for (i = 0; i < udc->max_ep / 2; i++) {
		tmp_reg = socle_otg_readl(&dr_regs->endptctrl[i]);
		t = scnprintf(next, size, "EP Ctrl Reg [0x%x]: = [0x%x]\n",
				i, tmp_reg);
		size -= t;
		next += t;
	}
	tmp_reg = socle_otg_readl(&dr_regs->endpointprime);
	t = scnprintf(next, size, "EP Prime Reg = [0x%x]\n", tmp_reg);
	size -= t;
	next += t;

	tmp_reg = usb_sys_regs->snoop1;
	t = scnprintf(next, size, "\nSnoop1 Reg : = [0x%x]\n\n", tmp_reg);
	size -= t;
	next += t;

	tmp_reg = usb_sys_regs->control;
	t = scnprintf(next, size, "General Control Reg : = [0x%x]\n\n",
			tmp_reg);
	size -= t;
	next += t;

	/* ------socle_otg_udc, socle_otg_ep, socle_otg_request structure information ----- */
	ep = &udc->eps[0];
	t = scnprintf(next, size, "For %s Maxpkt is 0x%x index is 0x%x\n",
			ep->ep.name, ep_maxpacket(ep), ep_index(ep));
	size -= t;
	next += t;

	if (list_empty(&ep->queue)) {
		t = scnprintf(next, size, "its req queue is empty\n\n");
		size -= t;
		next += t;
	} else {
		list_for_each_entry(req, &ep->queue, queue) {
			t = scnprintf(next, size,
				"req %p actual 0x%x length 0x%x  buf %p\n",
				&req->req, req->req.actual,
				req->req.length, req->req.buf);
			size -= t;
			next += t;
		}
	}
	/* other gadget->eplist ep */
	list_for_each_entry(ep, &udc->gadget.ep_list, ep.ep_list) {
		if (ep->desc) {
			t = scnprintf(next, size,
					"\nFor %s Maxpkt is 0x%x "
					"index is 0x%x\n",
					ep->ep.name, ep_maxpacket(ep),
					ep_index(ep));
			size -= t;
			next += t;

			if (list_empty(&ep->queue)) {
				t = scnprintf(next, size,
						"its req queue is empty\n\n");
				size -= t;
				next += t;
			} else {
				list_for_each_entry(req, &ep->queue, queue) {
					t = scnprintf(next, size,
						"req %p actual 0x%x length"
						"0x%x  buf %p\n",
						&req->req, req->req.actual,
						req->req.length, req->req.buf);
					size -= t;
					next += t;
					}	/* end for each_entry of ep req */
				}	/* end for else */
			}	/* end for if(ep->queue) */
		}		/* end (ep->desc) */

	spin_unlock_irqrestore(&udc->lock, flags);

	*eof = 1;
	return count - size;
}

#define create_proc_file()	create_proc_read_entry(proc_filename, \
				0, NULL, socle_otg_proc_read, NULL)

#define remove_proc_file()	remove_proc_entry(proc_filename, NULL)

#else				/* !CONFIG_USB_GADGET_DEBUG_FILES */

#define create_proc_file()	do {} while (0)
#define remove_proc_file()	do {} while (0)

#endif				/* CONFIG_USB_GADGET_DEBUG_FILES */

/*-------------------------------------------------------------------------*/

/* Release udc structures */
static void socle_otg_udc_release(struct device *dev)
{
	complete(udc_controller->done);
	dma_free_coherent(dev, udc_controller->ep_qh_size,
			udc_controller->ep_qh, udc_controller->ep_qh_dma);
	kfree(udc_controller);
}

/******************************************************************
	Internal structure setup functions
*******************************************************************/
/*------------------------------------------------------------------
 * init resource for globle controller
 * Return the udc handle on success or NULL on failure
 ------------------------------------------------------------------*/
static struct socle_otg_udc * struct_udc_setup(struct platform_device *pdev)
{
	struct socle_otg_udc *udc;
	struct socle_usb2_platform_data *pdata;
	size_t size;

	udc = kzalloc(sizeof(struct socle_otg_udc), GFP_KERNEL);
	if (udc == NULL) {
		ERR("malloc udc failed\n");
		return NULL;
	}

	pdata = pdev->dev.platform_data;
	udc->phy_mode = SOCLE_USB2_PHY_UTMI;//pdata->phy_mode;
	/* max_ep_nr is bidirectional ep number, max_ep doubles the number */
	//udc->max_ep = pdata->max_ep_nr * 2;
	//FIXME ryan
	udc->max_ep = 5 * 2;

	udc->eps = kzalloc(sizeof(struct socle_otg_ep) * udc->max_ep, GFP_KERNEL);
	if (!udc->eps) {
		ERR("malloc socle_otg_ep failed\n");
		goto cleanup;
	}

	/* initialized QHs, take care of alignment */
	size = udc->max_ep * sizeof(struct ep_queue_head);
	if (size < QH_ALIGNMENT)
		size = QH_ALIGNMENT;
	else if ((size % QH_ALIGNMENT) != 0) {
		size += QH_ALIGNMENT + 1;
		size &= ~(QH_ALIGNMENT - 1);
	}
	udc->ep_qh = dma_alloc_coherent(&pdev->dev, size,
					&udc->ep_qh_dma, GFP_KERNEL);
	if (!udc->ep_qh) {
		ERR("malloc QHs for udc failed\n");
		kfree(udc->eps);
		goto cleanup;
	}

	udc->ep_qh_size = size;

	/* Initialize ep0 status request structure */
	/* FIXME: socle_otg_alloc_request() ignores ep argument */
	udc->status_req = container_of(socle_otg_alloc_request(NULL, GFP_KERNEL),
			struct socle_otg_req, req);
	/* allocate a small amount of memory to get valid address */
	udc->status_req->req.buf = kmalloc(8, GFP_KERNEL);
	udc->status_req->req.dma = virt_to_phys(udc->status_req->req.buf);

	udc->resume_state = USB_STATE_NOTATTACHED;
	udc->usb_state = USB_STATE_POWERED;
	udc->ep0_dir = 0;
	udc->remote_wakeup = 0;	/* default to 0 on reset */
	spin_lock_init(&udc->lock);

	return udc;

cleanup:
	kfree(udc);
	return NULL;
}

/*----------------------------------------------------------------
 * Setup the socle_otg_ep struct for eps
 * Link socle_otg_ep->ep to gadget->ep_list
 * ep0out is not used so do nothing here
 * ep0in should be taken care
 *--------------------------------------------------------------*/
static int struct_ep_setup(struct socle_otg_udc *udc, unsigned char index,
		char *name, int link)
{
	struct socle_otg_ep *ep = &udc->eps[index];

	ep->udc = udc;
	strcpy(ep->name, name);
	ep->ep.name = ep->name;

	ep->ep.ops = &socle_otg_ep_ops;
	ep->stopped = 0;

	/* for ep0: maxP defined in desc
	 * for other eps, maxP is set by epautoconfig() called by gadget layer
	 */
	ep->ep.maxpacket = (unsigned short) ~0;

	/* the queue lists any req for this ep */
	INIT_LIST_HEAD(&ep->queue);

	/* gagdet.ep_list used for ep_autoconfig so no ep0 */
	if (link)
		list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	ep->gadget = &udc->gadget;
	ep->qh = &udc->ep_qh[index];

	return 0;
}

/* Driver probe function
 * all intialize operations implemented here except enabling usb_intr reg
 */

static int socle_otg_udc_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	int i;

	if (strcmp(pdev->name, driver_name)) {
		VDBG("Wrong device\n");
		return -ENODEV;
	}

	/* board setup should have been done in the platform code */

	/* Initialize the udc structure including QH member and other member */
	udc_controller = struct_udc_setup(pdev);
	if (!udc_controller) {
		VDBG("udc_controller is NULL \n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res){
		dev_err(&pdev->dev, "register resources unusable\n");
                ret = -ENXIO;
                goto free_dev;
	}

	udc_controller->irq = platform_get_irq(pdev, 0);
	if (udc_controller->irq < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = -ENXIO;
		goto free_dev;
	}
//#ifndef CONFIG_SOCLE_USB_OTG
	if (!request_mem_region(res->start, res->end - res->start + 1,
				driver_name)) {
		ERR("request mem region for %s failed \n", pdev->name);
		ret = -EBUSY;
		goto free_somthing_1;
	}
//#endif

	dr_regs = ioremap(res->start, (res->end - res->start) + 1);
	if (!dr_regs) {
		dev_err(&pdev->dev, "cannot map SocleDev registers\n");
		ret = -ENOMEM;
		goto release_mem;
	}
		
	usb_sys_regs = (struct usb_sys_interface *)
			((u32)dr_regs + USB_DR_SYS_OFFSET);

	//leonid fix from SA_SHIRQ to IRQF_SHARED
	ret = request_irq(udc_controller->irq, socle_otg_udc_irq, IRQF_SHARED,
			driver_name, udc_controller);
	if (ret != 0) {
		ERR("cannot request irq %d err %d \n",
				udc_controller->irq, ret);
		goto unmap_regs;
	}

	/* initialize usb hw reg except for regs for EP,
	 * leave usbintr reg untouched */
	dr_controller_setup(udc_controller);

	/* Setup gadget structure */
	udc_controller->gadget.ops = &socle_otg_gadget_ops;
	udc_controller->gadget.is_dualspeed = 1;
	udc_controller->gadget.ep0 = &udc_controller->eps[0].ep;
	INIT_LIST_HEAD(&udc_controller->gadget.ep_list);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
	udc_controller->gadget.name = driver_name;

	/* Setup gadget.dev and register with kernel */
	strcpy(udc_controller->gadget.dev.bus_id, "gadget");
	udc_controller->gadget.dev.release = socle_otg_udc_release;
	udc_controller->gadget.dev.parent = &pdev->dev;
	ret = device_register(&udc_controller->gadget.dev);

	if (ret < 0){
		dev_err(&pdev->dev, "device register error\n");
		goto unregister_irq;
	}

	/* setup QH and epctrl for ep0 */
	ep0_setup(udc_controller);
	/* setup udc->eps[] for ep0 */
	struct_ep_setup(udc_controller, 0, "ep0", 0);
	/* for ep0: the desc defined here;
	 * for other eps, gadget layer called ep_enable with defined desc
	 */
	udc_controller->eps[0].desc = &socle_otg_ep0_desc;
	udc_controller->eps[0].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;

	/* setup the udc->eps[] for non-control endpoints and link
	 * to gadget.ep_list */
	for (i = 1; i < (int)(udc_controller->max_ep / 2); i++) {
		char name[14];

		sprintf(name, "ep%dout", i);
		struct_ep_setup(udc_controller, i * 2, name, 1);
		sprintf(name, "ep%din", i);
		struct_ep_setup(udc_controller, i * 2 + 1, name, 1);
	}

	/* use dma_pool for TD management */
	udc_controller->td_pool = dma_pool_create("udc_td", &pdev->dev,
			sizeof(struct ep_td_struct),
			DTD_ALIGNMENT, UDC_DMA_BOUNDARY);
	if (udc_controller->td_pool == NULL) {
		ret = -ENOMEM;
		goto err4;
	}
	create_proc_file();
	return 0;

err4:
	device_unregister(&udc_controller->gadget.dev);
unregister_irq:
	free_irq(udc_controller->irq, udc_controller);
unmap_regs:
	iounmap(dr_regs);
release_mem:
//#ifndef CONFIG_SOCLE_USB_OTG
	release_mem_region(res->start, res->end - res->start + 1);
//#endif
free_somthing_1:

free_dev:

	return ret;
}

/* Driver removal function
 * Free resources and finish pending transactions
 */
static int  socle_otg_udc_remove(struct platform_device *pdev)
{
//#ifndef CONFIG_SOCLE_USB_OTG		//leonid+ for OTG warning message
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
//#endif
	
	DECLARE_COMPLETION(done);

	if (!udc_controller)
		return -ENODEV;
	udc_controller->done = &done;

	/* DR has been stopped in usb_gadget_unregister_driver() */
	remove_proc_file();

	/* Free allocated memory */
	kfree(udc_controller->status_req->req.buf);
	kfree(udc_controller->status_req);
	kfree(udc_controller->eps);

	dma_pool_destroy(udc_controller->td_pool);
	free_irq(udc_controller->irq, udc_controller);

//#ifndef CONFIG_SOCLE_USB_OTG		//leonid+ for OTG warning message
	iounmap(dr_regs);
	release_mem_region(res->start, res->end - res->start + 1);
//#endif
	device_unregister(&udc_controller->gadget.dev);
	/* free udc --wait for the release() finished */
	wait_for_completion(&done);

	return 0;
}

/*-----------------------------------------------------------------
 * Modify Power management attributes
 * Used by OTG statemachine to disable gadget temporarily
 -----------------------------------------------------------------*/
static int socle_otg_udc_suspend(struct platform_device *pdev, pm_message_t state)
{
	dr_controller_stop(udc_controller);
	return 0;
}

/*-----------------------------------------------------------------
 * Invoked on USB resume. May be called in_interrupt.
 * Here we start the DR controller and enable the irq
 *-----------------------------------------------------------------*/
static int socle_otg_udc_resume(struct platform_device *pdev)
{
#if 0		//20100625 leonid remove for suspend/resume no hang
	/* Enable DR irq reg and set controller Run */
	if (udc_controller->stopped) {
		dr_controller_setup(udc_controller);
		dr_controller_run(udc_controller);
	}
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;
#endif
	return 0;
}

/*-------------------------------------------------------------------------
	Register entry point for the peripheral controller driver
--------------------------------------------------------------------------*/

static struct platform_driver udc_driver = {
	.probe		= socle_otg_udc_probe,
	//.remove  = __exit_p(socle_otg_udc_remove),
	.remove  = socle_otg_udc_remove,			//leonid fix for OTG warning message
	/* these suspend and resume are not usb suspend and resume */
	.suspend = socle_otg_udc_suspend,
	.resume  = socle_otg_udc_resume,
	.driver  = {
		.name = (char *)driver_name,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_SOCLE_USB_OTG
static int udc_init(void)
#else
static int __init udc_init(void)
#endif
{
	printk(KERN_INFO "%s (%s)\n", driver_desc, DRIVER_VERSION);
	return platform_driver_register(&udc_driver);
}

#ifdef CONFIG_SOCLE_USB_OTG
static void udc_exit(void)
#else
static void __exit udc_exit(void)
#endif
{
	platform_driver_unregister(&udc_driver);
	printk("%s unregistered \n", driver_desc);
}

#ifndef CONFIG_SOCLE_USB_OTG
module_init(udc_init);
module_exit(udc_exit);
#endif

//leonid+ add for OTG

#ifdef CONFIG_SOCLE_USB_OTG

extern int ehci_hcd_init(void);
extern void ehci_hcd_cleanup(void);

static void
//socle_otg_switch_detection(void *data)
socle_otg_switch_detection(struct work_struct *work)
{
	int ret=0;

		if(b_exit == 1){
			//printk("B_EXIT\n");
			udc_exit();
			probe_b = 0;
			b_exit = 0;
		}
#ifdef CONFIG_SOCLE_EHCI
		if(a_exit == 1){
			//printk("A_EXIT\n");
			ehci_hcd_cleanup();
			probe_a = 0;
			a_exit = 0;
		}	
		if(a_init == 1){
			//printk("A_INIT\n");
			ret = ehci_hcd_init();
			probe_a = 1;
			a_init = 0;
		}
#endif
		if(b_init == 1){
			//printk("B_INIT\n");
			ret = udc_init();
			probe_b = 1;
			b_init = 0;
		}
	
	return ;
}


#define OTG_SC_ID	0x100
#define OTG_SC_IDIS	0x00010000
#define OTG_SC_IDIE	0x01000000


unsigned long otg_sc = IO_ADDRESS(SOCLE_OTG0) + 0x1a4;

static irqreturn_t socle_otg_irq(int irq, void *_udc)
{
	struct socle_otg_udc *udc = _udc;
	unsigned long flags;
	unsigned long otg_sts;
	
	spin_lock_irqsave(&udc->lock, flags);

	otg_sts = ioread32(otg_sc);

	//printk("socle_otg_irq : otg_sts = 0x%08x\n", (unsigned int)otg_sts);
	
	if(otg_sts & OTG_SC_IDIS){		
		printk("TYPE CHANGE\n");
		if(!(otg_sts & OTG_SC_ID)){		//id=1,	not A_type
		
#ifdef CONFIG_SOCLE_EHCI
			if(probe_a){
				//printk("remove A\n");
				a_exit = 1;
			}
#endif
			if(!probe_b){
				//printk("probe B\n");
				b_init=1;
			}
		}else{
			if(probe_b){
				//printk("remove B\n");
				b_exit = 1;
			}
			
#ifdef CONFIG_SOCLE_EHCI	
			if(!probe_a){
				//printk("probe A\n");
				a_init=1;	
			}	
#endif
		}
		iowrite32((otg_sts | OTG_SC_IDIE), otg_sc);
		//printk("otg_sc = 0x%08x\n", ioread32(otg_sc));
		schedule_delayed_work (&otg_dwork, HZ / HZ);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return IRQ_HANDLED;
}

static void *otg_dev;

static int socle_otg_probe(void)
{
	int ret = 0;
	unsigned long status;

	otg_dev = kzalloc(0x4, GFP_KERNEL);
	
	status = ioread32(otg_sc);

	if(status & OTG_SC_ID){	//B type (device)
		printk("B_TYPE : DEVICE\n");
		ret = udc_init();
		probe_b = 1;
	}
	
#ifdef CONFIG_SOCLE_EHCI
	else{		
		printk("A_TYPE : HOST\n");
		ret = ehci_hcd_init();
		probe_a = 1;
	}	
#endif

	//iowrite32(id_flag, otg_sc);
	iowrite32((status | OTG_SC_IDIE | OTG_SC_IDIS), otg_sc);

	ret = request_irq(IRQ_OTG0, socle_otg_irq, IRQF_SHARED, driver_name_otg, otg_dev);
	if (ret != 0) {
                ERR("cannot request irq %d err %d \n", IRQ_OTG0, ret);
        }

	return ret;
}

/* Driver removal function
 * Free resources and finish pending transactions
 */
static int socle_otg_remove(void)
{
	free_irq(IRQ_OTG0, otg_dev);	

	if(probe_a){
		printk("socle_otg_remove : remove A\n");
		ehci_hcd_cleanup();	
		//usb_ehci_socle_remove(hcd, dev);
	}else{
		printk("socle_otg_remove : remove B\n");	
		udc_exit();
	}
	
	//del_timer_sync (&otg_timer);

	
	//cancel_delayed_work (&otg_work);
	cancel_delayed_work (&otg_dwork);
	flush_scheduled_work ();
	kfree(otg_dev);

	return 0;
}

/*-------------------------------------------------------------------------
	Register entry point for OTG
--------------------------------------------------------------------------*/

static int __init otg_init(void)
{
	socle_otg_probe();
	
	return 0;
}

module_init(otg_init);

static void __exit otg_exit(void)
{
	socle_otg_remove();
	printk("%s unregistered \n", driver_desc_otg);
}

module_exit(otg_exit);
	
#endif

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
