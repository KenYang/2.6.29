/********************************************************************************
* File Name     : arch/arm/mach-socle/gpio.c
* Author         : cyli
* Description   : Socle GPIO Service
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

*   Version      : 2,0,0,1
*   History      :
*      1. 2007/08/29 cyli create this file
*      2. 2007/11/23 cyli add for interrupt service and proc
*      3. 2010/06/04 cyli add power management and register to platform
*
********************************************************************************/

#include <asm/io.h>
//#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <mach/regs-gpio.h>
#include <mach/gpio.h>


//#define CONFIG_GPIO_DEBUG
#ifdef CONFIG_GPIO_DEBUG
	#define GPIO_DBG(fmt, args...) printk("GPIO: %s(): " fmt, __FUNCTION__, ## args)
#else
	#define GPIO_DBG(fmt, args...)
#endif


struct socle_gpio_irq_s {
	int lock;
	int active;
	int int_cnt;
	int trigger_type;
	const char *name;
	irq_handler_t sub_routine;
	void *pparam;
};

struct socle_gpio_s {
	int base;
#ifdef SOCLE_GPIO_WITH_INT
	const char *name;
	int irq;
	int irq_num;
	int irq_array[GPIO_PER_PORT_PIN_NUM * GPIO_PORT_NUM];
	struct socle_gpio_irq_s pin[GPIO_PER_PORT_PIN_NUM * GPIO_PORT_NUM];
#endif
};

#define GPIO_GET_VALUE_WITH_MASK(fun_get, port, mask)	(fun_get((port)) & (mask))
#define GPIO_SET_VALUE_WITH_MASK(fun_get, fun_set, port, value, mask)		\
	u8 data;														\
	data = (fun_get((port)) & (~mask)) | ((value) & (mask));					\
	fun_set((port), data);

#define SOCLE_GPIO_TOTAL_GP_NUM		(1 + SOCLE_GPIO_GP1 + SOCLE_GPIO_GP2 + SOCLE_GPIO_GP3)
#define SOCLE_GPIO_MAX_PIN_NUM		(GPIO_PER_PORT_PIN_NUM * GPIO_PORT_NUM * SOCLE_GPIO_TOTAL_GP_NUM)

#define GET_GPIO_PORT_FROM_PIN_NUM(pin)		((pin) / GPIO_PER_PORT_PIN_NUM)
#define GET_GPIO_GP_PIN_FROM_PIN_NUM(pin)		((pin) % (GPIO_PER_PORT_PIN_NUM * GPIO_PORT_NUM))
#define GET_GPIO_ABS_PIN_FROM_PIN_NUM(pin)		((pin) % GPIO_PER_PORT_PIN_NUM)
#define GET_GPIO_GROUP_FROM_PORT_NUM(port)		((port) / GPIO_PORT_NUM)


static struct socle_gpio_s socle_gpio[SOCLE_GPIO_TOTAL_GP_NUM] = {{0}};
static DEFINE_SPINLOCK(socle_gpio_lock);

static char __initdata banner[] = "SOCLE GPIO Service, (c) 2007 SOCLE Corp.\n";


///////////////// * GPIO Functions * /////////////////

static inline int
gpio_read(u32 offset, u32 base)
{
	return (ioread32(base + offset));
}

static inline void
gpio_write(u32 offset, u32 data, u32 base)
{
	iowrite32(data, base + offset);
}

extern int
socle_gpio_get_value(u8 port)
{
	u8 value;
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_PADR;
		break;
	case 1:
		offset = GPIO_PBDR;
		break;
	case 2:
		offset = GPIO_PCDR;
		break;
	case 3:
		offset = GPIO_PDDR;
		break;
	default:
		return -1;
	}

	value = gpio_read(offset, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);

	return value;
}



extern void
socle_gpio_set_value(u8 port, u8 value)
{
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_PADR;
		break;
	case 1:
		offset = GPIO_PBDR;
		break;
	case 2:
		offset = GPIO_PCDR;
		break;
	case 3:
		offset = GPIO_PDDR;
		break;
	default:
		return;
	}

	gpio_write(offset, value, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);
}


extern int
socle_gpio_get_direction(u8 port)
{
	u8 dir;
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_PACON;
		break;
	case 1:
		offset = GPIO_PBCON;
		break;
	case 2:
		offset = GPIO_PCCON;
		break;
	case 3:
		offset = GPIO_PDCON;
		break;
	default:
		return -1;
	}

	dir = gpio_read(offset, base);
 
	GPIO_DBG("base = 0x%08x, offset = 0x%x, dir = 0x%x\n", base, offset, dir);

	return dir;
}

extern void
socle_gpio_set_direction(u8 port, u8 dir)
{
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_PACON;
		break;
	case 1:
		offset = GPIO_PBCON;
		break;
	case 2:
		offset = GPIO_PCCON;
		break;
	case 3:
		offset = GPIO_PDCON;
		break;
	default:
		return;
	}

	gpio_write(offset, dir, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, dir = 0x%x\n", base, offset, dir);
}

extern int
socle_gpio_get_interrupt_mask(u8 port)
{
	u8 value;
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_IEA;
		break;
	case 1:
		offset = GPIO_IEB;
		break;
	case 2:
		offset = GPIO_IEC;
		break;
	case 3:
		offset = GPIO_IED;
		break;
	default:
		return -1;
	}

	value = gpio_read(offset, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);

	return value;
}

extern void
socle_gpio_set_interrupt_mask(u8 port, u8 value)
{
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_IEA;
		break;
	case 1:
		offset = GPIO_IEB;
		break;
	case 2:
		offset = GPIO_IEC;
		break;
	case 3:
		offset = GPIO_IED;
		break;
	default:
		return;
	}

	gpio_write(offset, value, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);
}

extern int
socle_gpio_get_interrupt_sense(u8 port)
{
	u8 value;
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_ISA;
		break;
	case 1:
		offset = GPIO_ISB;
		break;
	case 2:
		offset = GPIO_ISC;
		break;
	case 3:
		offset = GPIO_ISD;
		break;
	default:
		return -1;
	}

	value = gpio_read(offset, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);

	return value;
}

extern void
socle_gpio_set_interrupt_sense(u8 port, u8 value)
{
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_ISA;
		break;
	case 1:
		offset = GPIO_ISB;
		break;
	case 2:
		offset = GPIO_ISC;
		break;
	case 3:
		offset = GPIO_ISD;
		break;
	default:
		return;
	}

	gpio_write(offset, value, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);
}

extern int
socle_gpio_get_interrupt_both_edges(u8 port)
{
	u8 value;
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_IBEA;
		break;
	case 1:
		offset = GPIO_IBEB;
		break;
	case 2:
		offset = GPIO_IBEC;
		break;
	case 3:
		offset = GPIO_IBED;
		break;
	default:
		return -1;
	}

	value = gpio_read(offset, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);

	return value;
}

extern void
socle_gpio_set_interrupt_both_edges(u8 port, u8 value)
{
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_IBEA;
		break;
	case 1:
		offset = GPIO_IBEB;
		break;
	case 2:
		offset = GPIO_IBEC;
		break;
	case 3:
		offset = GPIO_IBED;
		break;
	default:
		return;
	}

	gpio_write(offset, value, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);
}

extern int
socle_gpio_get_interrupt_event(u8 port)
{
	u8 value;
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_IEVA;
		break;
	case 1:
		offset = GPIO_IEVB;
		break;
	case 2:
		offset = GPIO_IEVC;
		break;
	case 3:
		offset = GPIO_IEVD;
		break;
	default:
		return -1;
	}

	value = gpio_read(offset, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);

	return value;
}

extern void
socle_gpio_set_interrupt_event(u8 port, u8 value)
{
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_IEVA;
		break;
	case 1:
		offset = GPIO_IEVB;
		break;
	case 2:
		offset = GPIO_IEVC;
		break;
	case 3:
		offset = GPIO_IEVD;
		break;
	default:
		return;
	}

	gpio_write(offset, value, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);
}

extern void
socle_gpio_set_interrupt_clear(u8 port, u8 value)
{
	u16 offset;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = GPIO_ICA;
		break;
	case 1:
		offset = GPIO_ICB;
		break;
	case 2:
		offset = GPIO_ICC;
		break;
	case 3:
		offset = GPIO_ICD;
		break;
	default:
		return;
	}

	gpio_write(offset, value, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);
}


extern int
socle_gpio_get_interrupt_status(u8 port)
{
	u32 value;
	u16 offset = GPIO_ISR;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	value = gpio_read(offset, base);

	GPIO_DBG("base = 0x%08x, offset = 0x%x, value = 0x%x\n", base, offset, value);

	return value;
}



extern int
socle_gpio_get_value_with_mask(u8 port, u8 mask)
{
	socle_gpio_direction_input_with_mask(port, mask);
	return socle_gpio_get_value(port) & mask;
}

extern void
socle_gpio_set_value_with_mask(u8 port, u8 value, u8 mask)
{
 	u8 data;

	socle_gpio_direction_output_with_mask(port, mask);
	data = (socle_gpio_get_value(port) & ~mask) | (value & mask);
	socle_gpio_set_value(port, data);
}

extern void
socle_gpio_direction_input_with_mask(u8 port, u8 mask)
{
	u8 dir;

	dir = socle_gpio_get_direction(port) & ~mask;
	socle_gpio_set_direction(port, dir);
}

extern void
socle_gpio_direction_output_with_mask(u8 port, u8 mask)
{
	u8 dir;

	dir = socle_gpio_get_direction(port) | mask;
	socle_gpio_set_direction(port, dir);
}

extern int
socle_gpio_get_interrupt_mask_with_mask(u8 port, u8 mask)
{
	return GPIO_GET_VALUE_WITH_MASK(socle_gpio_get_interrupt_mask, port, mask);
}

extern void
socle_gpio_set_interrupt_mask_with_mask(u8 port, u8 value, u8 mask)
{
	GPIO_SET_VALUE_WITH_MASK(socle_gpio_get_interrupt_mask, socle_gpio_set_interrupt_mask, port, value, mask)
}

extern int
socle_gpio_get_interrupt_sense_with_mask(u8 port, u8 mask)
{
	return GPIO_GET_VALUE_WITH_MASK(socle_gpio_get_interrupt_sense, port, mask);
}

extern void
socle_gpio_set_interrupt_sense_with_mask(u8 port, u8 value, u8 mask)
{
	GPIO_SET_VALUE_WITH_MASK(socle_gpio_get_interrupt_sense, socle_gpio_set_interrupt_sense, port, value, mask)
}

extern int
socle_gpio_get_interrupt_both_edges_with_mask(u8 port, u8 mask)
{
	return GPIO_GET_VALUE_WITH_MASK(socle_gpio_get_interrupt_both_edges, port, mask);
}

extern void
socle_gpio_set_interrupt_both_edges_with_mask(u8 port, u8 value, u8 mask)
{
	GPIO_SET_VALUE_WITH_MASK(socle_gpio_get_interrupt_both_edges, socle_gpio_set_interrupt_both_edges, port, value, mask)
}

extern int
socle_gpio_get_interrupt_event_with_mask(u8 port, u8 mask)
{
	return GPIO_GET_VALUE_WITH_MASK(socle_gpio_get_interrupt_event, port, mask);
}

extern void
socle_gpio_set_interrupt_event_with_mask(u8 port, u8 value, u8 mask)
{
	GPIO_SET_VALUE_WITH_MASK(socle_gpio_get_interrupt_event, socle_gpio_set_interrupt_event, port, value, mask)
}

extern int
socle_gpio_get_interrupt_status_with_port(u8 port)
{
	u32 value;
	u16 offset;

	value = socle_gpio_get_interrupt_status(port);

	switch (port % GPIO_PORT_NUM) {
	case 0:
		offset = 0;
		break;
	case 1:
		offset = 8;
		break;
	case 2:
		offset = 16;
		break;
	case 3:
		offset = 24;
		break;
	default:
		return -1;
	}

	value = value >> offset;

	return (value & 0xFF);
}


extern void
socle_gpio_test_mode_en(u8 port, int en)
{
	int data;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	data = gpio_read(GPIO_TEST, base);

	if (en)
		data |= 0x1;
	else
		data &= ~0x1;

	gpio_write(GPIO_TEST, data, base);
}

extern void
socle_gpio_test_mode_ctrl(u8 port, int mode)
{
	int  test_mode, data;
	u32 base;

	base = socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].base;

	switch (mode) {
	case PB2PA:
		test_mode = PB2PA;
		break;
	case PA2PB:
		test_mode = PA2PB;
		break;
	case PD2PC:
		test_mode = PD2PC;
		break;
	case PC2PD:
		test_mode = PC2PD;
		break;
	default:
		printk("Invalid argument! mode = %d\n", mode);
		return;
	}

	data = gpio_read(GPIO_TEST, base) & ~0x6;
	gpio_write(GPIO_TEST, (test_mode << 1) | data, base);
}

extern void
socle_gpio_claim_lock(void)
{
	spin_lock(&socle_gpio_lock);
}

extern void
socle_gpio_release_lock(void)
{
	spin_unlock(&socle_gpio_lock);
}


static int __init
socle_gpio_get_mem_res(struct platform_device *pdev)
{
	int i;
	struct resource *res;

	for (i = 0; i < SOCLE_GPIO_TOTAL_GP_NUM; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			return -ENXIO;

		if (!request_mem_region(res->start, resource_size(res), res->name))
			return -EBUSY;

		socle_gpio[i].base = (u32) ioremap(res->start, resource_size(res));
		if (!socle_gpio[i].base)
			return -ENOMEM;
	}

	return 0;
}



#ifdef SOCLE_GPIO_WITH_INT

///////////////// * GPIO Interrupt Service * /////////////////

extern int
socle_gpio_get_irq(u8 port)
{
	return socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].irq;
}
EXPORT_SYMBOL(socle_gpio_get_irq);

extern int
socle_request_gpio_irq(int pin, irq_handler_t handler, int irqflags, const char *devname, void *dev_id)
{
	int port = GET_GPIO_PORT_FROM_PIN_NUM(pin), val = 1 << GET_GPIO_ABS_PIN_FROM_PIN_NUM(pin);
	struct socle_gpio_irq_s *irq = &socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].pin[GET_GPIO_GP_PIN_FROM_PIN_NUM(pin)];

	GPIO_DBG("SOCLE_GPIO_MAX_PIN_NUM=%d pin[%d] isr@0x%x\n", SOCLE_GPIO_MAX_PIN_NUM, pin, (int)handler);

	if ((pin >= SOCLE_GPIO_MAX_PIN_NUM) || !handler)
		goto bad;
	if (xchg(&irq->lock, 1) != 0)
		goto busy;

	irq->active = 1;
	irq->int_cnt = 0;
	irq->trigger_type = irqflags;
	irq->name = devname;
	irq->sub_routine = handler;
	irq->pparam = dev_id;
	socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].irq_array[socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].irq_num] = GET_GPIO_GP_PIN_FROM_PIN_NUM(pin);
	socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].irq_num++;

	// set as normal mode
	socle_gpio_test_mode_en(port, 0);

	// set as input
	socle_gpio_get_value_with_mask(port, val);

	// set interrupt trigger type
	if (GPIO_INT_SENSE_EDGE == (irqflags & GPIO_INT_SENSE)) {
		socle_gpio_set_interrupt_sense_with_mask(port,  0x0, val);

		if (GPIO_INT_SINGLE_EDGE == (irqflags & GPIO_INT_BOTH_EDGE)) {
			socle_gpio_set_interrupt_both_edges_with_mask(port, 0x0, val);

			if (GPIO_INT_EVENT_LO == (irqflags & GPIO_INT_EVENT)) {
				socle_gpio_set_interrupt_event_with_mask(port, 0x0, val);
			} else if (GPIO_INT_EVENT_HI == (irqflags & GPIO_INT_EVENT))  {
				socle_gpio_set_interrupt_event_with_mask(port, val, val);
			}

		} else if (GPIO_INT_BOTH_EDGE == (irqflags & GPIO_INT_BOTH_EDGE))  {
			socle_gpio_set_interrupt_both_edges_with_mask(port, val, val);
		}

	} else if (GPIO_INT_SENSE_LEVEL == (irqflags & GPIO_INT_SENSE))  {
		socle_gpio_set_interrupt_sense_with_mask(port,  val, val);

		if (GPIO_INT_EVENT_LO == (irqflags & GPIO_INT_EVENT)) {
			socle_gpio_set_interrupt_event_with_mask(port, 0x0, val);
		} else if (GPIO_INT_EVENT_HI == (irqflags & GPIO_INT_EVENT))  {
			socle_gpio_set_interrupt_event_with_mask(port, val, val);
		}
	}

	// enable interrupt
	socle_gpio_set_interrupt_mask_with_mask(port, val, val);

	return 0;

bad:
	printk("Socle GPIO: trying to allocate pin[%d] fail!\n", pin);
	return -EINVAL;
busy:
	return -EBUSY;
}
EXPORT_SYMBOL(socle_request_gpio_irq);

extern void
socle_free_gpio_irq(int pin, void *dev_id)
{
	int port = GET_GPIO_PORT_FROM_PIN_NUM(pin), val = 1 << GET_GPIO_ABS_PIN_FROM_PIN_NUM(pin);
	struct socle_gpio_irq_s *irq = &socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].pin[GET_GPIO_GP_PIN_FROM_PIN_NUM(pin)];

	if (pin >= SOCLE_GPIO_MAX_PIN_NUM)
		goto bad;

	if (dev_id != irq->pparam)
		goto bad;

	if (irq->active) {
		printk("Socle GPIO: freeing active pin[%d]\n", pin);
		irq->active = 0;
		socle_gpio_set_interrupt_mask_with_mask(port, 0x0, val);
	}

	if (xchg(&irq->lock, 0) != 0) {
		irq->name = NULL;
		irq->sub_routine = NULL;
		socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].irq_num--;
		return;
	}

bad:
	printk("Socle GPIO: trying to free pin[%d] fail!\n", pin);
}
EXPORT_SYMBOL(socle_free_gpio_irq);

extern void 
socle_enable_gpio_irq(int pin)
{
	int port = GET_GPIO_PORT_FROM_PIN_NUM(pin), val = 1 << GET_GPIO_ABS_PIN_FROM_PIN_NUM(pin);
	struct socle_gpio_irq_s *irq = &socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].pin[GET_GPIO_GP_PIN_FROM_PIN_NUM(pin)];

	if ((pin >= SOCLE_GPIO_MAX_PIN_NUM) || !irq->lock)
		goto bad;

	if (0 == irq->active) {
		irq->active = 1;
		socle_gpio_set_interrupt_mask_with_mask(port, val, val);
	}

	return;

bad:
	printk("Socle GPIO: trying to enable free pin[%d] fail!\n", pin);
	BUG();
}
EXPORT_SYMBOL(socle_enable_gpio_irq);

extern void
socle_disable_gpio_irq(int pin)
{
	int port = GET_GPIO_PORT_FROM_PIN_NUM(pin), val = 1 << GET_GPIO_ABS_PIN_FROM_PIN_NUM(pin);
	struct socle_gpio_irq_s *irq = &socle_gpio[GET_GPIO_GROUP_FROM_PORT_NUM(port)].pin[GET_GPIO_GP_PIN_FROM_PIN_NUM(pin)];

	if ((pin >= SOCLE_GPIO_MAX_PIN_NUM) || !irq->lock)
		goto bad;

	if (1 == irq->active) {
		irq->active = 0;
		socle_gpio_set_interrupt_mask_with_mask(port, 0x0, val);
	}

	return;

bad:
	printk("Socle GPIO: trying to disable free pin[%d] fail!\n", pin);
	BUG();
}
EXPORT_SYMBOL(socle_disable_gpio_irq);

static inline void
socle_gpio_com_isr(int group)
{
	int status, i, idx;

	if (!socle_gpio[group].irq_num)
		return;

	// read interrupt status
	status = socle_gpio_get_interrupt_status(GPIO_PORT_NUM * group);

	for (i = 0; i < socle_gpio[group].irq_num; i++) {
		idx = socle_gpio[group].irq_array[i];

		GPIO_DBG("i = %d, status = 0x%08x, idx = %d, 0x%08x\n", i, status, idx, (1 << idx));

		if ((1 << idx) & status) {
			int pin = idx + GPIO_PER_PORT_PIN_NUM * GPIO_PORT_NUM * group;
			int port = GET_GPIO_PORT_FROM_PIN_NUM(pin), val = 1 << GET_GPIO_ABS_PIN_FROM_PIN_NUM(pin);
			struct socle_gpio_irq_s *irq = &socle_gpio[group].pin[GET_GPIO_GP_PIN_FROM_PIN_NUM(pin)];

			GPIO_DBG("idx = %d, pin[%d]\n", idx, pin);

			// execute sub_routine
			irq->sub_routine(pin, irq->pparam);
			// clear interrupt
			socle_gpio_set_interrupt_clear(port, val);

			irq->int_cnt++;
		}
	}
}

static irqreturn_t
socle_gpio_isr_gp0(int irq, void *data)
{
	socle_gpio_com_isr(0);
	return IRQ_HANDLED;
}

#if SOCLE_GPIO_GP1 == 1
static irqreturn_t
socle_gpio_isr_gp1(int irq, void *data)
{
	socle_gpio_com_isr(1);
	return IRQ_HANDLED;
}
#endif

#if SOCLE_GPIO_GP2 == 1
static irqreturn_t
socle_gpio_isr_gp2(int irq, void *data)
{
	socle_gpio_com_isr(2);
	return IRQ_HANDLED;
}
#endif

#if SOCLE_GPIO_GP3 == 1
static irqreturn_t
socle_gpio_isr_gp3(int irq, void *data)
{
	socle_gpio_com_isr(3);
	return IRQ_HANDLED;
}
#endif

static irqreturn_t (*socle_gpio_isr_array[]) (int irq, void *data) = {
	socle_gpio_isr_gp0,
#if SOCLE_GPIO_GP1 == 1
	socle_gpio_isr_gp1,
#endif
#if SOCLE_GPIO_GP2 == 1
	socle_gpio_isr_gp2,
#endif
#if SOCLE_GPIO_GP3 == 1
	socle_gpio_isr_gp3,
#endif
};


///////////////// * GPIO Interrupt Proc * /////////////////

static struct proc_dir_entry *socle_gpio_irq_proc_entry;

static void*
socle_gpio_irq_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= (sizeof(socle_gpio) / sizeof(struct socle_gpio_s)))
		return NULL;
	return &socle_gpio[*pos];
}

static void*
socle_gpio_irq_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
//	(*pos)++;
	return NULL;
}

static void 
socle_gpio_irq_seq_stop(struct seq_file *s, void *v)
{
	/* Actually, there's nothing to do here */
}

static int
socle_gpio_irq_seq_show(struct seq_file *s, void *v)
{
	struct socle_gpio_s *gpio = (struct socle_gpio_s *)v;
	int i;

	seq_printf(s, "GPIO#%d ----------------------------------------------------------\n", (int)s->index);
	seq_printf(s, "Name         : %s\n", gpio->name);
	seq_printf(s, "Base         : 0x%08x\n", gpio->base);
	seq_printf(s, "IRQ          : %d\n", gpio->irq);
	seq_printf(s, "Active Number: %d\n", gpio->irq_num);
	seq_printf(s, "Status       :\n");

	for (i = 0; i < (GPIO_PER_PORT_PIN_NUM * GPIO_PORT_NUM); i++) {
		seq_printf(s, "\tPin[%02d]: ", i);
		seq_printf(s, "Lock: %s %s, ", gpio->pin[i].lock ? "locked by" : "unlocked", gpio->pin[i].name ? gpio->pin[i].name : "");
		seq_printf(s, "Active: %s\n", gpio->pin[i].active ? "active" : "idle");

		if (gpio->pin[i].lock) {
			int irqflags = gpio->pin[i].trigger_type;

			seq_printf(s, "\t\tTriggle Type: ");
			if (GPIO_INT_SENSE_EDGE == (irqflags & GPIO_INT_SENSE)) {
				if (GPIO_INT_SINGLE_EDGE == (irqflags & GPIO_INT_BOTH_EDGE)) {
					if (GPIO_INT_EVENT_LO == (irqflags & GPIO_INT_EVENT)) {
						seq_printf(s, "Falling Edge");
					} else if (GPIO_INT_EVENT_HI == (irqflags & GPIO_INT_EVENT))  {
						seq_printf(s, "Rising Edge");
					}
				} else if (GPIO_INT_BOTH_EDGE == (irqflags & GPIO_INT_BOTH_EDGE))  {
					seq_printf(s, "Both Edge");
				}
			} else if (GPIO_INT_SENSE_LEVEL == (irqflags & GPIO_INT_SENSE))  {
				if (GPIO_INT_EVENT_LO == (irqflags & GPIO_INT_EVENT)) {
					seq_printf(s, "Low Level");
				} else if (GPIO_INT_EVENT_HI == (irqflags & GPIO_INT_EVENT))  {
					seq_printf(s, "High Level");
				}
			}

			seq_printf(s, ", Interrupt Count: %d\n", gpio->pin[i].int_cnt);
		}
	}
	seq_printf(s, "\n");

	return 0;
}

static struct seq_operations socle_gpio_irq_seq_ops = {
	.start = socle_gpio_irq_seq_start,
	.next = socle_gpio_irq_seq_next,
	.stop = socle_gpio_irq_seq_stop,
	.show = socle_gpio_irq_seq_show,
};

static int socle_gpio_irq_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &socle_gpio_irq_seq_ops);
}

static struct file_operations socle_gpio_irq_proc_ops = {
	.owner = THIS_MODULE,
	.open = socle_gpio_irq_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release, 
};


static int __init
socle_gpio_get_irq_res(struct platform_device *pdev)
{
	int i, ret;
	struct resource *res;

	for (i = 0; i < SOCLE_GPIO_TOTAL_GP_NUM; i++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!res || res->start < 0)
			return -ENXIO;

		socle_gpio[i].irq = res->start;
		socle_gpio[i].name = res->name;

		ret = request_irq(socle_gpio[i].irq, socle_gpio_isr_array[i], IRQF_SHARED, socle_gpio[i].name, socle_gpio);
		if (ret) {
			printk("Error! Fail to request irq(%d)\n", socle_gpio[i].irq);
			return ret;
		}
	}

	/* Install the proc_fs entry */
//	socle_gpio_irq_proc_entry = create_proc_entry("socle_gpio_irq", S_IRUGO | S_IFREG, &proc_root);
	socle_gpio_irq_proc_entry = create_proc_entry("socle_gpio_irq", S_IRUGO | S_IFREG, NULL);
	if (socle_gpio_irq_proc_entry)
		socle_gpio_irq_proc_entry->proc_fops = &socle_gpio_irq_proc_ops;
	else
		return -ENOMEM;

	return 0;
}


#endif	//SOCLE_GPIO_WITH_INT


///////////////// * GPIO Power Management * /////////////////

#ifdef CONFIG_PM

static u32 socle_gpio_regs[SOCLE_GPIO_TOTAL_GP_NUM][GPIO_IEVD / 4 + 1];

extern void
socle_gpio_suspend(void)
{
	int i, gp;
	u32 base;

	for (gp = 0; gp < SOCLE_GPIO_TOTAL_GP_NUM; gp++) {
		base = socle_gpio[gp].base;
		for (i = 0; i < (GPIO_IEVD / 4 + 1); i++)
			socle_gpio_regs[gp][i] = gpio_read(i * 4, base);
	}
}
EXPORT_SYMBOL(socle_gpio_suspend);

extern void
socle_gpio_resume(void)
{
	int i, gp;
	u32 base;

	for (gp = 0; gp < SOCLE_GPIO_TOTAL_GP_NUM; gp++) {
		base = socle_gpio[gp].base;
		for (i = 0; i < (GPIO_IEVD / 4 + 1); i++)
			gpio_write(i * 4, socle_gpio_regs[gp][i], base);
	}
}
EXPORT_SYMBOL(socle_gpio_resume);

#endif


///////////////// * GPIO Expoort Symbol * /////////////////

EXPORT_SYMBOL(socle_gpio_get_value);
EXPORT_SYMBOL(socle_gpio_set_value);

EXPORT_SYMBOL(socle_gpio_get_direction);
EXPORT_SYMBOL(socle_gpio_set_direction);

EXPORT_SYMBOL(socle_gpio_get_interrupt_mask);
EXPORT_SYMBOL(socle_gpio_set_interrupt_mask);
EXPORT_SYMBOL(socle_gpio_get_interrupt_sense);
EXPORT_SYMBOL(socle_gpio_set_interrupt_sense);
EXPORT_SYMBOL(socle_gpio_get_interrupt_both_edges);
EXPORT_SYMBOL(socle_gpio_set_interrupt_both_edges);

EXPORT_SYMBOL(socle_gpio_get_interrupt_event);
EXPORT_SYMBOL(socle_gpio_set_interrupt_event);

EXPORT_SYMBOL(socle_gpio_set_interrupt_clear);

EXPORT_SYMBOL(socle_gpio_get_interrupt_status);


EXPORT_SYMBOL(socle_gpio_get_value_with_mask);
EXPORT_SYMBOL(socle_gpio_set_value_with_mask);
EXPORT_SYMBOL(socle_gpio_direction_input_with_mask);
EXPORT_SYMBOL(socle_gpio_direction_output_with_mask);
EXPORT_SYMBOL(socle_gpio_get_interrupt_mask_with_mask);
EXPORT_SYMBOL(socle_gpio_set_interrupt_mask_with_mask);
EXPORT_SYMBOL(socle_gpio_get_interrupt_sense_with_mask);
EXPORT_SYMBOL(socle_gpio_set_interrupt_sense_with_mask);
EXPORT_SYMBOL(socle_gpio_get_interrupt_both_edges_with_mask);
EXPORT_SYMBOL(socle_gpio_set_interrupt_both_edges_with_mask);
EXPORT_SYMBOL(socle_gpio_get_interrupt_event_with_mask);
EXPORT_SYMBOL(socle_gpio_set_interrupt_event_with_mask);
EXPORT_SYMBOL(socle_gpio_get_interrupt_status_with_port);

EXPORT_SYMBOL(socle_gpio_test_mode_en);
EXPORT_SYMBOL(socle_gpio_test_mode_ctrl);

EXPORT_SYMBOL(socle_gpio_claim_lock);
EXPORT_SYMBOL(socle_gpio_release_lock);


static int __init
socle_gpio_probe(struct platform_device *pdev)
{
	int ret;

	printk(banner);

	ret = socle_gpio_get_mem_res(pdev);
	if (ret) {
		printk("Get memory resource fail!\n");
		return ret;
	}

#ifdef SOCLE_GPIO_WITH_INT
	ret = socle_gpio_get_irq_res(pdev);
	if (ret) {
		printk("Get memory resource fail!\n");
		return ret;
	}
#endif

	return ret;
}

static struct platform_driver socle_gpio_driver = {
	.probe		= socle_gpio_probe,
//	.remove		= __exit_p(socle_gpio_remove),
	.driver		= {
		.name	= "socle-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init socle_gpio_init(void)
{
	return platform_driver_register(&socle_gpio_driver);
}

arch_initcall(socle_gpio_init);

