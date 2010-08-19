/*
 *  linux/arch/arm/mach-socle/pdk-pc9223.c
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
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/serial_8250.h>

#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/io.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>


#include <mach/platform.h>
#include <mach/hardware.h>

#include "devs.h"
#include "generic.h"

/* --------------------------------------------------------------------
 *  UART
 * -------------------------------------------------------------------- */
 /* SOCLE-PDK UART */

#define PDK_UART_CLOCK (176947200/2)

static struct plat_serial8250_port pdk_uart_data[] = {
	{
		.mapbase	= SOCLE_UART0,
		.membase	= (char*)(IO_ADDRESS(SOCLE_UART0)),
		.irq			= IRQ_UART0,
		.uartclk		= PDK_UART_CLOCK,
		.regshift		= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},{
		.mapbase	= SOCLE_UART1,
		.membase	= (char*)(IO_ADDRESS(SOCLE_UART1)),
		.irq			= IRQ_UART1,
		.uartclk		= PDK_UART_CLOCK,
		.regshift		= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
#ifndef CONFIG_SERIAL_8250_BTUART
	{
		.mapbase	= SOCLE_UART2,
		.membase	= (char*)(IO_ADDRESS(SOCLE_UART2)),
		.irq			= IRQ_UART2,
		.uartclk		= PDK_UART_CLOCK,
		.regshift		= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
#endif
	{ },
};

struct platform_device pdk_uart_device = {
	.name		= "serial8250",
	.id		= PLAT8250_DEV_PLATFORM,
	.dev.platform_data	= pdk_uart_data,
};

#ifdef CONFIG_SERIAL_8250_BTUART
static struct plat_serial8250_port pdk_uartBT_data[] = {
	{
		.mapbase	= SOCLE_UART2,
		.membase	= (char*)(IO_ADDRESS(SOCLE_UART2)),
		.irq			= IRQ_UART2,
		.uartclk		= PDK_UART_CLOCK,
		.regshift		= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{ },
};

struct platform_device pdk_uartBT_device = {
	.name		= "btuart",
	.id		= PLAT8250_DEV_PLATFORM1,
	.dev.platform_data	= pdk_uartBT_data,
};
#endif

static struct map_desc pdk_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(SOCLE_INTC0),
		.pfn		= __phys_to_pfn(SOCLE_INTC0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_SDRSTMC0),
		.pfn		= __phys_to_pfn(SOCLE_SDRSTMC0),
		.length		= SZ_128K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_NOR_FLASH0),
		.pfn		= __phys_to_pfn(SOCLE_NOR_FLASH0),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_UART0),
		.pfn		= __phys_to_pfn(SOCLE_UART0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_UART1),
		.pfn		= __phys_to_pfn(SOCLE_UART1),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_UART2),
		.pfn		= __phys_to_pfn(SOCLE_UART2),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_SPI0),
		.pfn		= __phys_to_pfn(SOCLE_SPI0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_SPI1),
		.pfn		= __phys_to_pfn(SOCLE_SPI1),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_I2C0),
		.pfn		= __phys_to_pfn(SOCLE_I2C0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_I2S0),
		.pfn		= __phys_to_pfn(SOCLE_I2S0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_SDHC0),
		.pfn		= __phys_to_pfn(SOCLE_SDHC0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_TIMER0),
		.pfn		= __phys_to_pfn(SOCLE_TIMER0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_TIMER_PWM0),
		.pfn		= __phys_to_pfn(SOCLE_TIMER_PWM0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_GPIO0),
		.pfn		= __phys_to_pfn(SOCLE_GPIO0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_GPIO1),
		.pfn		= __phys_to_pfn(SOCLE_GPIO1),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_GPIO2),
		.pfn		= __phys_to_pfn(SOCLE_GPIO2),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_GPIO3),
		.pfn		= __phys_to_pfn(SOCLE_GPIO3),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_RTC0),
		.pfn		= __phys_to_pfn(SOCLE_RTC0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_WDT0),
		.pfn		= __phys_to_pfn(SOCLE_WDT0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_ADC0),
		.pfn		= __phys_to_pfn(SOCLE_ADC0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_SCU0),
		.pfn		= __phys_to_pfn(SOCLE_SCU0),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_MAC0),
		.pfn		= __phys_to_pfn(SOCLE_MAC0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_OTG_UDC0),
		.pfn		= __phys_to_pfn(SOCLE_OTG_UDC0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_UHC0),
		.pfn		= __phys_to_pfn(SOCLE_UHC0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_HDMA0),
		.pfn		= __phys_to_pfn(SOCLE_HDMA0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_LCD0),
		.pfn		= __phys_to_pfn(SOCLE_LCD0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_VOP0),
		.pfn		= __phys_to_pfn(SOCLE_VOP0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(PANTHER7_HDMA0),
		.pfn		= __phys_to_pfn(PANTHER7_HDMA0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_NAND0),
		.pfn		= __phys_to_pfn(SOCLE_NAND0),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(SOCLE_VIP0),
		.pfn		= __phys_to_pfn(SOCLE_VIP0),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
};

#ifdef CONFIG_MMU
#ifdef CONFIG_ANDROID_SYSTEM
static void pc9223_power_off(void)
{
	while(1){};
}
#endif

static void __init pdk_map_io(void)
{
	iotable_init(pdk_io_desc, ARRAY_SIZE(pdk_io_desc));
#ifdef CONFIG_ANDROID_SYSTEM
		        pm_power_off = pc9223_power_off;
#endif
	
}
#endif

static void __init pdk_init(void)
{
	socle_add_lcd_device();
	platform_device_register(&pdk_uart_device);
#ifdef CONFIG_SERIAL_8250_BTUART
	platform_device_register(&pdk_uartBT_device);
#endif
	//add device register
	socle_add_device_gpio();		//20100622 cyli+ for gpio power management
	socle_add_device_flash();
	socle_add_device_rtc();
	socle_add_device_watchdog();
	socle_add_device_eth();
	socle_add_device_ehci();
        socle_add_device_otg_udc();
	socle_add_device_i2c();
	socle_add_device_adc();
	socle_add_device_snd_i2s();
		
#if !defined(CONFIG_SQ_GDR)		
	socle_add_device_nfc();
#else		
	socle_del_device_nfc();	
#endif	
	socle_add_device_vop();
	socle_add_device_vip();
	socle_sdhc_add_device_mci();
	socle_spi_add_device();
	pdk_add_device_kpd();
	socle_add_device_adc_battery();
	//socle_add_device_adc_ts();
	socle_add_device_led_backlight();	

}

extern struct socle_dma panther7_hdma_channel_0;
extern struct socle_dma panther7_hdma_channel_1;

extern void
socle_platform_dma_init(struct socle_dma **dma)
{
	dma[0] = &panther7_hdma_channel_0;
	dma[1] = &panther7_hdma_channel_1;
}

MACHINE_START(SOCLE, "PDK-PC9223")
	.phys_io	= 0x19000000,
#ifdef CONFIG_MMU
	.io_pg_offst	= ((0xfc000000) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x40000100,
#ifdef CONFIG_MMU
	.map_io		= pdk_map_io,
#endif
	.init_irq	= socle_init_irq,
	.timer		= &socle_timer,
	.init_machine	= pdk_init,
MACHINE_END

