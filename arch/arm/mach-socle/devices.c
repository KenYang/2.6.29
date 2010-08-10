/*
 * arch/arm/mach-socle/devices.c
 *
 *  Copyright (C) 2005 Thibaut VARENE <varenet@parisc-linux.org>
 *  Copyright (C) 2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>		//leonid+ for pdk-pc7210 lcd arbiter
 
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/platform_device.h>

#include <mach/hardware.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#include <linux/spi/spi.h>

#include <linux/spi/ads7846.h>
//#include <mach/socle-clcd.h>
#include <mach/i2s.h>
#include <mach/dma.h>
#include "devs.h"

#include <mach/gpio.h>

#if defined(CONFIG_ARCH_PDK_PC9002) || defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_SCDK)
#include <mach/regs-mp-gpio.h>
#include <mach/mp-gpio.h>
#include <mach/cheetah-scu.h>
#endif

#if defined(CONFIG_ARCH_PDK_PC9220)
#include <mach/pc9220-scu.h>
#endif
#if defined(CONFIG_ARCH_PDK_PC9223)
#include <mach/pc9223-scu.h>
#endif

#include <media/soc_camera_platform.h>
#include <media/socle_vip_camera.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/init.h>

#include <linux/spi/libertas_spi.h>		//20100629 jerry+ for socle_spi 

/* --------------------------------------------------------------------
 *  NOR Flash
 * -------------------------------------------------------------------- */
#if defined(CONFIG_MTD_SOCLE) || defined(CONFIG_MTD_SOCLE_MODULE)
#if defined(CONFIG_ARCH_MDK_3D) || defined(CONFIG_ARCH_MDK_FHD)
static struct mtd_partition nor_partitions[] = {
	/* bootloader (U-Boot, etc) */
	{
	      .name		= "u-boot",
	      .offset		= 0,
	      .size			= 0x40000,
	      .mask_flags	= MTD_WRITEABLE,
	},
	/* diag */
	{
	      .name		= "diag",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= 0x100000 - 0x40000,
	      .mask_flags	= MTD_WRITEABLE,
	},
	/* env */
	{
	      .name		= "env",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= 0x120000 - 0x100000,
	      .mask_flags	= MTD_WRITEABLE,
	},
	{
	      .name		= "kernel0-openlinux",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= 0x800000 - 0x120000,
	      .mask_flags	= MTD_WRITEABLE,
	},
	{
	      .name		= "filesystem",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= MTDPART_SIZ_FULL,
	      .mask_flags	= MTD_WRITEABLE,
	}
};

static struct flash_platform_data socle_nor_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= nor_partitions,
	.nr_parts		= ARRAY_SIZE(nor_partitions),
};

static struct resource socle_nor_resource[] = {
	[0] = {	
		.start		= SOCLE_NOR_FLASH0,
		.end		= SOCLE_NOR_FLASH0 + SOCLE_NOR_FLASH_SIZE- 1,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device socle_nor_device = {
	.name		= "socle-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &socle_nor_data,
	},
	.num_resources	= ARRAY_SIZE(socle_nor_resource),
	.resource	= socle_nor_resource,
};
#ifdef SOCLE_NOR_FLASH1
static struct mtd_partition nor_partitions_2[] = {
	/* kernel1-android */
	{
	      .name		= "kernel1-android",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= 0x400000,
	      .mask_flags	= MTD_WRITEABLE,
	},
	{
	      .name		= "android-ramdisk",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= MTDPART_SIZ_FULL,
	      .mask_flags	= MTD_WRITEABLE,
	}
};

static struct flash_platform_data socle_nor_data_2 = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= nor_partitions_2,
	.nr_parts		= ARRAY_SIZE(nor_partitions_2),
};

static struct resource socle_nor_resource_2[] = {
	[0] = {	
		.start		= SOCLE_NOR_FLASH1,
		.end		= SOCLE_NOR_FLASH1 + SOCLE_NOR_FLASH_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device socle_nor_device_2 = {
	.name		= "socle-flash",
	.id		= 1,
	.dev		= {
		.platform_data	= &socle_nor_data_2,
	},
	.num_resources	= ARRAY_SIZE(socle_nor_resource_2),
	.resource	= socle_nor_resource_2,
};
#endif
#else //#ifdef CONFIG_ARCH_MDK_3D
static struct mtd_partition nor_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
	      .name		= "u-boot",
	      .offset		= 0,
	      .size			= 0x120000,
#ifdef CONFIG_SOCLE_INR
	      .mask_flags	= 0,
#else
	      .mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
	/* kernel */
	{
	      .name		= "kernel",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= 0x1e0000,
#ifdef CONFIG_SOCLE_INR
	      .mask_flags	= 0,
#else
	      .mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#if (SOCLE_NOR_FLASH_SIZE == SZ_16M)
	/* file system */
	{
	      .name		= "ramdisk",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= 0x500000,
	      .mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
#endif
	{
	      .name		= "data",
	      .offset		= MTDPART_OFS_APPEND,
	      .size			= MTDPART_SIZ_FULL,
	      .mask_flags	= 0, /* force read-only */	      
	}
};

static struct flash_platform_data socle_nor_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= nor_partitions,
	.nr_parts		= ARRAY_SIZE(nor_partitions),
};

static struct resource socle_nor_resource[] = {
	[0] = {	
		.start		= SOCLE_NOR_FLASH0,
		.end		= SOCLE_NOR_FLASH0 + SOCLE_NOR_FLASH_SIZE- 1,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device socle_nor_device = {
	.name		= "socle-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &socle_nor_data,
	},
	.num_resources	= ARRAY_SIZE(socle_nor_resource),
	.resource	= socle_nor_resource,
};
#ifdef SOCLE_NOR_FLASH1
static struct mtd_partition nor_partitions_2[] = {
	/* file system */
	{
	      .name		= "ramdisk",
	      .offset		= 0,
	      .size			= MTDPART_SIZ_FULL,
#ifdef CONFIG_SOCLE_INR
          .mask_flags	= 0,
#else
	      .mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	}
};

static struct flash_platform_data socle_nor_data_2 = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= nor_partitions_2,
	.nr_parts		= ARRAY_SIZE(nor_partitions_2),
};

static struct resource socle_nor_resource_2[] = {
	[0] = {	
		.start		= SOCLE_NOR_FLASH1,
		.end		= SOCLE_NOR_FLASH1 + SOCLE_NOR_FLASH_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device socle_nor_device_2 = {
	.name		= "socle-flash",
	.id		= 1,
	.dev		= {
		.platform_data	= &socle_nor_data_2,
	},
	.num_resources	= ARRAY_SIZE(socle_nor_resource_2),
	.resource	= socle_nor_resource_2,
};
#endif

#endif
void __init socle_add_device_flash(void)
{
	platform_device_register(&socle_nor_device);
#ifdef SOCLE_NOR_FLASH1
	platform_device_register(&socle_nor_device_2);
#endif
}
#else
void __init socle_add_device_flash(void) {}
#endif


/* --------------------------------------------------------------------
 *  Fake Battery Detect
 * -------------------------------------------------------------------- */

// fatke battery
struct platform_device socle_fake_battery = {
	.name	= "socle-fake-battery",
        .id             = -1,
};

void __init socle_add_device_fake_battery(void)
{
        platform_device_register(&socle_fake_battery);
}

//fake rfkill
struct platform_device socle_fake_rfkill = {
	.name	= "socle_rfkill",
        .id             = -1,
};

void __init socle_add_device_fake_rfkill(void)
{
        platform_device_register(&socle_fake_rfkill);
}


// ADC Battery Detect
struct platform_device socle_adc_battery = {
	.name	= "socle-adc-battery",
        .id             = -1,
};

void __init socle_add_device_adc_battery(void)
{
        platform_device_register(&socle_adc_battery);
}

// ADC Touch Screen
struct platform_device socle_adc_ts= {
	.name	= "socle-adc-ts",
        .id             = -1,
};

void __init socle_add_device_adc_ts(void)
{
        platform_device_register(&socle_adc_ts);
}


/* --------------------------------------------------------------------
 *  Keyboard
 * -------------------------------------------------------------------- */

#if defined(CONFIG_CDK_KPD) || defined(CONFIG_CDK_KPD_MODULE)
#include <linux/input.h>
static unsigned char cdkkpd_keycode[]  = {
	[0]		= KEY_LEFTCTRL,
	[1]		= KEY_LEFTSHIFT,
	[2]		= KEY_LEFTALT,
	[3]		= KEY_LEFTMETA,
	[4]		= KEY_RIGHTCTRL,
	[5]		= KEY_RIGHTSHIFT,
	[6]		= KEY_RIGHTALT,
	[7]		= KEY_RIGHTMETA,
	//	4 X 4 KeyPad
	//		c0 c1 c2 c3
	//	r0	[8][9][10][11]
	//	r1	[12][13][14][15]
	//	r2	[16][17][18][19]
	//	r3	[20][21][22][23]
	[8]		= KEY_ESC,
	[9]		= KEY_LEFT,
	[10]		= KEY_RIGHT,
	[11]		= KEY_ENTER,
	[12]		= KEY_P,
	[13]		= KEY_U,
	[14]		= KEY_SPACE,
	[15]		= KEY_TAB,
	[16]		= KEY_BACKSPACE,
	[17]		= KEY_A,
	[18]		= KEY_B,
	[19]		= KEY_C,
	[20]		= KEY_D,
	[21]		= KEY_E,
	[22]		= KEY_F,
	[23]		= KEY_G,
};

struct platform_device cdk_device_kpd = {
	.name	= "cdk-keypad",
	.id		= -1,
	.dev	= {
		.platform_data	= &cdkkpd_keycode,
	},
};

void __init cdk_add_device_kpd(void)
{
	platform_device_register(&cdk_device_kpd);
}

#else
void __init cdk_add_device_kpd(void) {}
#endif

/* --------------------------------------------------------------------
 *  PANTHER-GPIO Keypad
 * -------------------------------------------------------------------- */

#if defined(CONFIG_KEYBOARD_P7DK) || defined(CONFIG_KEYBOARD_P7DK_MODULE) || defined(CONFIG_KEYBOARD_MDK_FHD) || defined(CONFIG_KEYBOARD_MDK_FHD_MODULE)
#include <linux/input.h>
static unsigned char pdkkpd_keycode[]  = {
        [0]             = KEY_LEFTCTRL,
        [1]             = KEY_LEFTSHIFT,
        [2]             = KEY_LEFTALT,
        [3]             = KEY_LEFTMETA,
        [4]             = KEY_RIGHTCTRL,
        [5]             = KEY_RIGHTSHIFT,
        [6]             = KEY_RIGHTALT,
        [7]             = KEY_RIGHTMETA,
        //      4 X 4 KeyPad
        //              c0 c1 c2 c3
        //      r0      [8][9][10][11]
        //      r1      [12][13][14][15]
        //      r2      [16][17][18][19]
        //      r3      [20][21][22][23]
        [8]             = KEY_ESC,
        [9]             = KEY_LEFT,
        [10]       = KEY_RIGHT,
        [11]       = KEY_ENTER,
        [12]       = KEY_P,
        [13]       = KEY_U,
	 [14]	= KEY_SPACE,
	 [15]	= KEY_TAB,
	 [16]	= KEY_BACKSPACE,
	 [17]	= KEY_A,
	 [18]	= KEY_B,
	 [19]	= KEY_C,
	 [20]	= KEY_D,
	 [21]	= KEY_E,
	 [22]	= KEY_F,
	 [23]	= KEY_G,
};

struct platform_device pdk_device_kpd = {
        .name   = "pdk-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &pdkkpd_keycode,
        },
};

void __init pdk_add_device_kpd(void)
{
        platform_device_register(&pdk_device_kpd);
}

/* --------------------------------------------------------------------
 *  PC9220 Keypad
 * -------------------------------------------------------------------- */

#elif defined(CONFIG_KEYBOARD_PC9220) || defined(CONFIG_KEYBOARD_PC9220_MODULE)
#include <linux/input.h>
static unsigned char pdkkpd_keycode[]  = {
        [0]             = KEY_LEFTCTRL,
        [1]             = KEY_LEFTSHIFT,
        [2]             = KEY_LEFTALT,
        [3]             = KEY_LEFTMETA,
        [4]             = KEY_RIGHTCTRL,
        [5]             = KEY_RIGHTSHIFT,
        [6]             = KEY_RIGHTALT,
        [7]             = KEY_RIGHTMETA,
        //      4 X 4 KeyPad
        //              c0 c1 c2 c3
        //      r0      [8][9][10][11]
        //      r1      [12][13][14][15]
        //      r2      [16][17][18][19]
        //      r3      [20][21][22][23]
#ifndef CONFIG_ANDROID_SYSTEM
        [8]             = KEY_ESC,
        [9]             = KEY_LEFT,
        [10]       = KEY_RIGHT,
        [11]       = KEY_ENTER,
        [12]       = KEY_P,
        [13]       = KEY_U,
	 [14]	= KEY_SPACE,
	 [15]	= KEY_TAB,
	 [16]	= KEY_BACKSPACE,
	 [17]	= KEY_A,
	 [18]	= KEY_B,
	 [19]	= KEY_C,
	 [20]	= KEY_D,
	 [21]	= KEY_E,
	 [22]	= KEY_F,
	 [23]	= KEY_G,
#else
#ifndef CONFIG_SOCLE_7INCH_PANEL
	 //  20 16 12  8
	 //  21 17 13  9
	 //  22 18 14 10
	 //  23 19 15 11
        [8]             = 212, //Camera
        [9]             = KEY_SEARCH, //Search
        [10]       = KEY_F2,
        [11]       = KEY_F4, //End Call
        [12]       = KEY_HOME, // Home
        [13]       = KEY_U,
	 [14]	= KEY_RIGHT, //RIGHT
	 [15]	= KEY_POWER,
	 [16]	= KEY_F1, // Menu
	 [17]	= KEY_UP, //UP
	 [18]	= KEY_DOWN, //DOWN
	 [19]	= KEY_VOLUMEUP,
	 [20]	= KEY_BACK, //Back
	 [21]	= 232, //DPAD_CENTER
	 [22]	= KEY_LEFT, //LEFT
	 [23]	= KEY_VOLUMEDOWN,
#else
	 //  11 15 19 23
	 //  10 14 18 22
	 //  9  13 17 21
	 //  8  12 16 20
        [23]             = 212, //Camera
        [22]             = KEY_SEARCH, //Search
        [21]       = KEY_F2,
        [20]       = KEY_F4, //End Call
        [19]       = KEY_HOME, // Home
//        [18]       = KEY_U,
        [18]       = KEY_HOME,
	 [17]	= KEY_RIGHT, //RIGHT
	 [16]	= KEY_POWER,
	 [15]	= KEY_F1, // Menu
	 [14]	= KEY_UP, //UP
	 [13]	= KEY_DOWN, //DOWN
	 [12]	= KEY_VOLUMEUP,
	 [11]	= KEY_BACK, //Back
	 [10]	= 232, //DPAD_CENTER
	 [9]	= KEY_LEFT, //LEFT
	 [8]	= KEY_VOLUMEDOWN,
#endif
#endif
};

struct platform_device pdk_device_kpd = {
        .name   = "pc9220-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &pdkkpd_keycode,
        },
};

void __init pdk_add_device_kpd(void)
{
        platform_device_register(&pdk_device_kpd);
}

/* --------------------------------------------------------------------
 *  PC9223 Keypad
 * -------------------------------------------------------------------- */
 
#elif defined(CONFIG_KEYBOARD_PC9223) || defined(CONFIG_KEYBOARD_PC9223_MODULE)
#include <linux/input.h>
static unsigned char pdkkpd_keycode[]  = {
        [0]             = KEY_LEFTCTRL,
        [1]             = KEY_LEFTSHIFT,
        [2]             = KEY_LEFTALT,
        [3]             = KEY_LEFTMETA,
        [4]             = KEY_RIGHTCTRL,
        [5]             = KEY_RIGHTSHIFT,
};

struct platform_device pdk_device_kpd = {
        .name   = "pc9223-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &pdkkpd_keycode,
        },
};

void __init pdk_add_device_kpd(void)
{
        platform_device_register(&pdk_device_kpd);
}

#elif defined(CONFIG_KEYBOARD_PDK) || defined(CONFIG_KEYBOARD_PDK_MODULE) \
	 || defined(CONFIG_KEYBOARD_PDK) || defined(CONFIG_KEYBOARD_PDK_MODULE)
#include <linux/input.h>
static unsigned char pdkkpd_keycode[]  = {
        [0]             = KEY_LEFTCTRL,
        [1]             = KEY_LEFTSHIFT,
        [2]             = KEY_LEFTALT,
        [3]             = KEY_LEFTMETA,
        [4]             = KEY_RIGHTCTRL,
        [5]             = KEY_RIGHTSHIFT,
        [6]             = KEY_RIGHTALT,
        [7]             = KEY_RIGHTMETA,
        //      4 X 4 KeyPad
        //              c0 c1 c2 c3
        //      r0      [8][9][10][11]
        //      r1      [12][13][14][15]
        //      r2      [16][17][18][19]
        //      r3      [20][21][22][23]
        [8]             = KEY_ESC,
        [9]             = KEY_LEFT,
        [10]    = KEY_RIGHT,
        [11]    = KEY_ENTER,
        [12]    = KEY_P,
        [13]    = KEY_U,
	 [14]	= KEY_SPACE,
	 [15]	= KEY_TAB,
	 [16]	= KEY_BACKSPACE,
	 [17]	= KEY_A,
	 [18]	= KEY_B,
	 [19]	= KEY_C,
	 [20]	= KEY_D,
	 [21]	= KEY_E,
	 [22]	= KEY_F,
	 [23]	= KEY_G,
};

struct platform_device pdk_device_kpd = {
        .name   = "pdk-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &pdkkpd_keycode,
        },
};

void __init pdk_add_device_kpd(void)
{
        platform_device_register(&pdk_device_kpd);
}

#elif defined(CONFIG_KEYBOARD_INR) || defined(CONFIG_KEYBOARD_INR_MODULE)
#include <linux/input.h>
static unsigned char pdkkpd_keycode[]  = {
        [0]             = KEY_VOLUMEDOWN,		// volume down
        [1]             = KEY_VOLUMEUP,			// volume up
        [2]             = KEY_F13,				// iPod plug
        [3]             = KEY_F14,				// iPod unplug
        [4]             = KEY_F15,				// line-in plug
        [5]             = KEY_F16,				// line-in unplug
        [6]             = KEY_F17,
        [7]             = KEY_F18,
        //      4 X 4 KeyPad
        //              c0 c1 c2 c3
        //      r0      [8][9][10][11]
        //      r1      [12][13][14][15]
        //      r2      [16][17][18][19]
        //      r3      [20][21][22][23]
        [8]             = KEY_RESERVED,
        [9]             = KEY_UP,
        [10]    = KEY_RESERVED,
        [11]    = KEY_POWER,
        [12]    = KEY_LEFT,
        [13]    = KEY_ENTER,
	 [14]	= KEY_RIGHT,
	 [15]	= KEY_RESERVED,
	 [16]	= KEY_RESERVED,
	 [17]	= KEY_DOWN,
	 [18]	= KEY_RESERVED,
	 [19]	= KEY_F5,			// favorite 5
	 [20]	= KEY_F1,			// favorite 1
	 [21]	= KEY_F2,			// favorite 2
	 [22]	= KEY_F3,			// favorite 3
	 [23]	= KEY_F4,			// favorite 4
};

struct platform_device pdk_device_kpd = {
        .name   = "inr-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &pdkkpd_keycode,
        },
};

void __init pdk_add_device_kpd(void)
{
        platform_device_register(&pdk_device_kpd);
}

#elif defined(CONFIG_KEYBOARD_PC9220) || defined(CONFIG_KEYBOARD_PC9220_MODULE)	\
	|| defined(CONFIG_KEYBOARD_PC9223) || defined(CONFIG_KEYBOARD_PC9223_MODULE)
#include <linux/input.h>
static unsigned char pc9220_kpd_keycode[]  = {
        [0]             = KEY_LEFTCTRL,
        [1]             = KEY_LEFTSHIFT,
        [2]             = KEY_LEFTALT,
        [3]             = KEY_LEFTMETA,
        [4]             = KEY_RIGHTCTRL,
        [5]             = KEY_RIGHTSHIFT,
        [6]             = KEY_RIGHTALT,
        [7]             = KEY_RIGHTMETA,
        //      4 X 4 KeyPad
        //              c0 c1 c2 c3
        //      r0      [8][9][10][11]
        //      r1      [12][13][14][15]
        //      r2      [16][17][18][19]
        //      r3      [20][21][22][23]
        [8]             = KEY_RESERVED,
        [9]             = KEY_UP,
        [10]    = KEY_RESERVED,
        [11]    = KEY_ESC,
        [12]    = KEY_LEFT,
        [13]    = KEY_ENTER,
	 [14]	= KEY_RIGHT,
	 [15]	= KEY_RESERVED,
	 [16]	= KEY_RESERVED,
	 [17]	= KEY_DOWN,
	 [18]	= KEY_RESERVED,
	 [19]	= KEY_F5,
	 [20]	= KEY_F1,
	 [21]	= KEY_F2,
	 [22]	= KEY_F3,
	 [23]	= KEY_F4,
};

struct platform_device pc9220_device_kpd = {
        .name   = "pc9220-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &pc9220_kpd_keycode,
        },
};

void __init pdk_add_device_kpd(void)
{
        platform_device_register(&pc9220_device_kpd);
}

#else
void __init pdk_add_device_kpd(void) {}
#endif

/* --------------------------------------------------------------------
 *  SOCLE-GPIO Keypad
 * -------------------------------------------------------------------- */

#if defined(CONFIG_KEYBOARD_MSMV) || defined(CONFIG_KEYBOARD_MSMV_MODULE)
#include <linux/input.h>
static unsigned char msmvkpd_keycode[]  = {
        [0]             = KEY_1,
        [1]             = KEY_2,
        [2]             = KEY_3,
        [3]             = KEY_4,
        [4]             = KEY_5,
        [5]             = KEY_6,
        [6]             = KEY_7,
        [7]             = KEY_8,

        [8]             = KEY_P,
        [9]             = KEY_SPACE,
/*
        [10]       = KEY_RIGHT,
        [11]       = KEY_ENTER,
        [12]       = KEY_P,
        [13]       = KEY_U,
	 [14]	= KEY_SPACE,
	 [15]	= KEY_TAB,
	 [16]	= KEY_BACKSPACE,
	 [17]	= KEY_A,
	 [18]	= KEY_B,
	 [19]	= KEY_C,
	 [20]	= KEY_D,
	 [21]	= KEY_E,
	 [22]	= KEY_F,
	 [23]	= KEY_G,
*/
};

struct platform_device msmv_device_kpd = {
        .name   = "msmv-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &msmvkpd_keycode,
        },
};

void __init msmv_add_device_kpd(void)
{
        platform_device_register(&msmv_device_kpd);
}

#else
void __init msmv_add_device_kpd(void) {}
#endif

/* --------------------------------------------------------------------
 *  ADC
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SOCLE_ADC) || defined(CONFIG_SOCLE_ADC_MODULE)
static struct resource socle_adc_resource[] = {
	[0] = {
		.start = SOCLE_ADC0,
		.end   = SOCLE_ADC0 + SZ_4K -1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_ADC0,
		.end   = IRQ_ADC0,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device socle_device_adc = {
	.name		  = "socle-adc",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(socle_adc_resource),
	.resource	  = socle_adc_resource,
};

void __init socle_add_device_adc(void)
{
	platform_device_register(&socle_device_adc);
}
#else
void __init socle_add_device_adc(void) {}
#endif

#if defined(CONFIG_TOUCHSCREEN_SOCLE_ADS7846) || defined(CONFIG_TOUCHSCREEN_SOCLE_ADS7846_MODULE)
static int ads7843_pendown_state(void)
{
	u32 ret;

#if defined(CONFIG_ARCH_MDK_3D)	
	ret = readl(IO_ADDRESS(SOCLE_VIC0)+0x8);
	if(ret & 0x80000000)
		return 1;
	else
		return 0;
#elif defined(CONFIG_ARCH_PDK_PC9223) 
	ret = readl(IO_ADDRESS(SOCLE_GPIO1)+0x8);
	if(!(ret & 0x20))
		return 1;
	else
		return 0;
#endif 
}

static struct ads7846_platform_data ads_info = {
        .model                  = 7846,
        .x_min                  = 150,
        .x_max                  = 3830,
        .y_min                  = 190,
        .y_max                  = 3830,
        .vref_delay_usecs       = 100,
        .x_plate_ohms           = 450,
        .y_plate_ohms           = 250,
        .pressure_max           = 15000,
        .pressure_min           = 0,			
        .debounce_max           = 1,
        .debounce_rep           = 0,
        .debounce_tol           = (~0),
        .get_pendown_state      = ads7843_pendown_state,
};
#endif

//20100629 jerry+ for socle_spi_wifi
#if defined(CONFIG_SOCLE_LIBERTAS_SPI) || defined(CONFIG_SOCLE_LIBERTAS_SPI_MODULE)
static struct libertas_spi_platform_data libertas_spi_info = {
	.use_dummy_writes	= 0,
	.setup 				= NULL,
	.teardown			= NULL,
};
#endif

/* --------------------------------------------------------------------
 *  SPI Master
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SPI_SOCLE) || defined(CONFIG_SPI_SOCLE_MODULE)
static u64 socle_spi_dma_mask = 0xffffffffUL;
static struct resource socle_spi0_resources[] = {
	[0] = {
		.start = SOCLE_SPI0,
		.end = SOCLE_SPI0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI0,
		.end = IRQ_SPI0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_spi0_device = {
	.name = "socle_spi",
	.id = 0,
	.dev = {
		.dma_mask = &socle_spi_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_spi0_resources,
	.num_resources = ARRAY_SIZE(socle_spi0_resources),
};

#ifdef SOCLE_SPI1
static struct resource socle_spi1_resources[] = {
	[0] = {
		.start = SOCLE_SPI1,
		.end = SOCLE_SPI1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI1,
		.end = IRQ_SPI1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_spi1_device = {
	.name = "socle_spi",
	.id = 1,
	.dev = {
		.dma_mask = &socle_spi_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_spi1_resources,
	.num_resources = ARRAY_SIZE(socle_spi1_resources),
};
#endif


//20100629 jerry fix for socle_spi_wifi
static struct spi_board_info socle_spi_board_info[] __initdata = {
//#if defined(CONFIG_WIFI_SOCLE) || defined(CONFIG_WIFI_SOCLE_MODULE)
#if defined(CONFIG_SOCLE_LIBERTAS_SPI) || defined(CONFIG_SOCLE_LIBERTAS_SPI_MODULE)
{ 
		//.modalias = "wifi_aw_gh320",
		//.platform_data = NULL,
		.modalias = "libertas_spi",
		.platform_data = &libertas_spi_info,
		.controller_data = NULL,
#if defined(CONFIG_ARCH_P7DK)
		.irq = SET_GPIO_PIN_NUM(PA, 5),	//PA5
#elif defined(CONFIG_ARCH_PDK_PC7210)
		.irq = SET_GPIO_PIN_NUM(PC, 1),	//PC1
#elif defined(CONFIG_ARCH_PDK_PC9002)		
		.irq = MP_GPIO_INT,
#elif defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)		
		.irq = IRQ_EXT1,
#else
		.irq = IRQ_EXT0,		
#endif	
		.max_speed_hz = 3000000, /* 3.0 MHz */
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_SOCLE) || defined(CONFIG_TOUCHSCREEN_SOCLE_MODULE)
	{
		.modalias = "ts_tsc2000",
		.platform_data = NULL,
		.controller_data = NULL,
#if defined (CONFIG_ARCH_P7DK) 
		.irq = SET_GPIO_PIN_NUM(PA, 5), 	//set PA5 as low level triggle interrupt
#elif defined(CONFIG_ARCH_PDK_PC7210)
		.irq = SET_GPIO_PIN_NUM(PF, 6),
#else 	//INCLUDE , CDK , PC9002, pc9220, pc9223
		.irq = IRQ_EXT0,
#endif
		.max_speed_hz = 3000000, /* 3.0 MHz */
#ifdef CONFIG_ARCH_PDK_PC7210
		.bus_num = 0,
#else
		.bus_num = 1,
#endif	
		.chip_select = 0,
		.mode = SPI_MODE_1,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_SOCLE_ADS7846) || defined(CONFIG_TOUCHSCREEN_SOCLE_ADS7846_MODULE) 
        {
                .modalias       = "ads7846",
#ifdef CONFIG_ARCH_MDK_3D
                .chip_select    = 1,
                .bus_num        = 0,
//                .irq            = IRQ_EXT0,
                .irq            = IRQ_EXT2,
#elif defined(CONFIG_ARCH_PDK_PC9223)
		.chip_select    = 0,
              .bus_num        = 1,
              .irq            = IRQ_EXT0,

#else
                .chip_select    = 0,
                .bus_num        = 1,
                .irq            = IRQ_EXT1,
#endif
                .max_speed_hz   = 125000 * 20,  /* (max sample rate @ 3V) * (cmd + data + overhead) */
                .platform_data  = &ads_info,
        },
#endif

#if defined(CONFIG_SENSORS_ADC_MAX1110) || defined(CONFIG_SENSORS_ADC_MAX1110_MODULE)

        {
                .modalias       = "hwmon_adc_max1110",
                .chip_select    = 0,
                .max_speed_hz   = 200000,  /* 200KHz */
                .bus_num        = 1,
		  .mode 			= SPI_MODE_0,
                .irq            = -1,
        },
#endif

#if defined(CONFIG_EEPROM_AT25040) || defined(CONFIG_EEPROM_AT25040_MODULE)
	{
		.modalias = "eeprom_at25040",
		.platform_data = NULL,
		.controller_data = NULL,
		.irq = -1,
		.max_speed_hz = 3000000, /* 3.0 MHz */
#if (defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_PDK_PC9002) || defined(CONFIG_ARCH_SCDK))
		.bus_num = 1,
#else
		.bus_num = 0,
#endif
		.chip_select = 0,
		.mode = SPI_MODE_0,
	},
#endif
#if defined(CONFIG_SOCLE_MAILBOX_MASTER) || defined(CONFIG_SOCLE_MAILBOX_MASTER_MODULE)
	{
		.modalias = "spi_mailbox_mst",
		.platform_data = NULL,
		.controller_data = NULL,
		.irq = -1,
		.max_speed_hz = 2000000, /* 2.0 MHz */
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	},
#endif
};

void __init socle_spi_add_device(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)

//20100629 jerry+ for socle_spi_wifi
//#if defined(CONFIG_WIFI_SOCLE) || defined(CONFIG_WIFI_SOCLE_MODULE)
#if defined(CONFIG_SOCLE_LIBERTAS_SPI) || defined(CONFIG_SOCLE_LIBERTAS_SPI_MODULE)
	socle_scu_dev_enable(SOCLE_DEVCON_EXT_INT1);	//for EXT1 GPIO switch
#endif

#if defined(CONFIG_TOUCHSCREEN_SOCLE) || defined(CONFIG_TOUCHSCREEN_SOCLE_MODULE) || \
	defined(CONFIG_TOUCHSCREEN_SOCLE_ADS7846) || defined(CONFIG_TOUCHSCREEN_SOCLE_ADS7846_MODULE)

	socle_scu_dev_enable(SOCLE_DEVCON_EXT_INT0);	//for EXT0 GPIO switch
#endif

	socle_scu_dev_enable(SOCLE_DEVCON_SPI0);
	socle_scu_dev_enable(SOCLE_DEVCON_SPI1);
#endif


	platform_device_register(&socle_spi0_device);
#ifdef SOCLE_SPI1
	platform_device_register(&socle_spi1_device);	
#endif
	spi_register_board_info(socle_spi_board_info, ARRAY_SIZE(socle_spi_board_info));
}
#else
void __init socle_spi_add_device(void) {}
#endif


/* --------------------------------------------------------------------
 *  Mailbox slave driver on master
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SOCLE_MAILBOX_SLAVE) || defined(CONFIG_SOCLE_MAILBOX_SLAVE_MODULE)
static struct platform_device socle_device_mailbox = {
	.name		= "spi_mailbox_slv",
	.id			= -1,
	.resource	= 0,
};

void __init socle_add_device_mailbox(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_SPI0);
#endif
	platform_device_register(&socle_device_mailbox);
}
#else
void __init socle_add_device_mailbox(void) {}
#endif

/* --------------------------------------------------------------------
 *  MMC / SD
 * -------------------------------------------------------------------- */
 
#if defined(CONFIG_MMC_SOCLE_SDMMC) || defined(CONFIG_MMC_SOCLE_SDMMC_MODULE)

static int socle_sdmmc_dma_ch = 0;

static u64 socle_sdmmc_dma_mask = 0xffffffffUL;
static struct resource socle_sdmmc_resources[] = {
	[0] = {
		.start = SOCLE_SDMMC0,
		.end = SOCLE_SDMMC0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDMMC0,
		.end = IRQ_SDMMC0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_sdmmc_device = {
	.name = "socle_sdmmc",
	.id = -1,
	.dev = {
		.dma_mask = &socle_sdmmc_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_sdmmc_resources,
	.num_resources = ARRAY_SIZE(socle_sdmmc_resources),
};

void __init socle_sdmmc_add_device_mmc(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220)
	socle_scu_dev_enable(SOCLE_DEVCON_SDMMC);
#endif

#ifdef CONFIG_ARCH_PDK_PC9002
	socle_mp_gpio_set_port_num_value(MP_PA,5,0);
#endif
	platform_device_add_data(&socle_sdmmc_device, (void *)&socle_sdmmc_dma_ch, sizeof(int));	
	platform_device_register(&socle_sdmmc_device);
}
#else
void __init socle_sdmmc_add_device_mmc(void) {}
#endif

/* --------------------------------------------------------------------
 *  SDHC 2.0
 * -------------------------------------------------------------------- */
 
#if defined(CONFIG_MMC_SOCLE_SDHC) || defined(CONFIG_MMC_SOCLE_SDHC_MODULE)

static u64 socle_sdhc_dma_mask = 0xffffffffUL;
static struct resource socle_sdhc0_resources[] = {
	[0] = {
		.start = SOCLE_SDHC0,
		.end = SOCLE_SDHC0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDHC0,
		.end = IRQ_SDHC0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_sdhc0_device = {
	.name = "socle_sdhc",
	.id = 0,
	.dev = {
		.dma_mask = &socle_sdhc_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_sdhc0_resources,
	.num_resources = ARRAY_SIZE(socle_sdhc0_resources),
};

#ifdef SOCLE_SDHC1
static struct resource socle_sdhc1_resources[] = {
	[0] = {
		.start = SOCLE_SDHC1,
		.end = SOCLE_SDHC1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDHC1,
		.end = IRQ_SDHC1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_sdhc1_device = {
	.name = "socle_sdhc",
	.id = 1,
	.dev = {
		.dma_mask = &socle_sdhc_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_sdhc1_resources,
	.num_resources = ARRAY_SIZE(socle_sdhc1_resources),
};
#endif

#ifdef SOCLE_SDHC2
static struct resource socle_sdhc2_resources[] = {
	[0] = {
		.start = SOCLE_SDHC2,
		.end = SOCLE_SDHC2 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDHC2,
		.end = IRQ_SDHC2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_sdhc2_device = {
	.name = "socle_sdhc",
	.id = 2,
	.dev = {
		.dma_mask = &socle_sdhc_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_sdhc2_resources,
	.num_resources = ARRAY_SIZE(socle_sdhc2_resources),
};
#endif

void __init socle_sdhc_add_device_mci(void)
{
#if defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_SDMMC);
#endif
	platform_device_register(&socle_sdhc0_device);
#ifdef SOCLE_SDHC1
	platform_device_register(&socle_sdhc1_device);
#endif
#ifdef SOCLE_SDHC2
	platform_device_register(&socle_sdhc2_device);
#endif
}
#else
void __init socle_sdhc_add_device_mci(void) {}
#endif

#if 0
/* --------------------------------------------------------------------
 *  PWM Timer
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SOCLE_PWMT) || defined(CONFIG_SOCLE_PWMT_MODULE)
static struct resource socle_pwmt_resource[] = {
	[0] = {
		.start = SOCLE_TIMER_PWM0,
		.end   = SOCLE_TIMER_PWM0 + SZ_4K -1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_PWM0,
		.end   = IRQ_PWM0,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_PWM1,
		.end   = IRQ_PWM1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device socle_device_pwmt = {
	.name		= "socle-pwmt",
	.id			= -1,
	.num_resources	= ARRAY_SIZE(socle_pwmt_resource),
	.resource	= socle_pwmt_resource,
};

void __init socle_add_device_pwmt(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220)
	socle_scu_dev_enable(SCU_DEVCON_PWM0_GPIO);
	socle_scu_dev_enable(SCU_DEVCON_PWM1_GPIO);
#endif
//	platform_device_register(&socle_device_pwmt);
}
#else
void __init socle_add_device_pwmt(void) {}
#endif
#endif


/* --------------------------------------------------------------------
 *  I2C
 * -------------------------------------------------------------------- */

#if defined(CONFIG_I2C_SOCLE) || defined(CONFIG_I2C_SOCLE_MODULE)
static u64 socle_i2c_0_dma_mask = 0xffffffffUL;
static struct resource socle_i2c_0_resources[] = {
	[0] = {
		.start = SOCLE_I2C0,
		.end = SOCLE_I2C0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C0,
		.end = IRQ_I2C0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_i2c_0_device = {
	.name = "socle_i2c",
	.id = 0,
	.dev = {
		.dma_mask = &socle_i2c_0_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_i2c_0_resources,
	.num_resources = ARRAY_SIZE(socle_i2c_0_resources),
};

#ifdef SOCLE_I2C1
static u64 socle_i2c_1_dma_mask = 0xffffffffUL;
static struct resource socle_i2c_1_resources[] = {
	[0] = {
		.start = SOCLE_I2C1,
		.end = SOCLE_I2C1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C1,
		.end = IRQ_I2C1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_i2c_1_device = {
	.name = "socle_i2c",
	.id = 1,
	.dev = {
		.dma_mask = &socle_i2c_1_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_i2c_1_resources,
	.num_resources = ARRAY_SIZE(socle_i2c_1_resources),
};
#endif

#ifdef SOCLE_I2C2
static u64 socle_i2c_2_dma_mask = 0xffffffffUL;
static struct resource socle_i2c_2_resources[] = {
	[0] = {
		.start = SOCLE_I2C2,
		.end = SOCLE_I2C2 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C2,
		.end = IRQ_I2C2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_i2c_2_device = {
	.name = "socle_i2c",
	.id = 2,
	.dev = {
		.dma_mask = &socle_i2c_2_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_i2c_2_resources,
	.num_resources = ARRAY_SIZE(socle_i2c_2_resources),
};
#endif
#if 0
#include <mach/ms6335.h>
static struct i2c_board_info __initdata ms6335_i2c_devices[] = {
//static struct i2c_board_info ms6335_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ms6335",MS6335_SLAVE_ADDR_DAC),
	}
};
#endif

void __init socle_add_device_i2c(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_I2C);
#endif

//	i2c_register_board_info(0, ms6335_i2c_devices, ARRAY_SIZE(ms6335_i2c_devices));
	platform_device_register(&socle_i2c_0_device);
//i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info);

#ifdef SOCLE_I2C1
	platform_device_register(&socle_i2c_1_device);
#endif
#ifdef SOCLE_I2C2
	platform_device_register(&socle_i2c_2_device);
#endif
}
#else
void __init socle_add_device_i2c(void) {}
#endif

/* --------------------------------------------------------------------
 *  I2S
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SND_SOCLE) || defined(CONFIG_SND_SOCLE_MODULE)
static struct socle_i2s_platform_data socle_i2s_data = {
#if (defined(CONFIG_ARCH_P7DK) || defined(CONFIG_ARCH_PDK_PC7210) || defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223))		//20080109 leonid+ for Panther 7
	.tx_dma_ch = 0,
	.rx_dma_ch = 1,
	.tx_dma_hdreq = 6,
	.rx_dma_hdreq = 7,
	.fifo_depth = 8,
	.burst_type = SOCLE_DMA_BURST_INCR4,
#elif (defined(CONFIG_ARCH_MSMV) )
	.tx_dma_ch = 0,
	.rx_dma_ch = 1,
	.tx_dma_hdreq = 6,
	.rx_dma_hdreq = 7,
	.fifo_depth = 32,
	.burst_type = SOCLE_DMA_BURST_INCR16,
#elif (defined(CONFIG_ARCH_MDK_3D))
	.tx_dma_ch = 0,
	.rx_dma_ch = 2,
	.tx_dma_hdreq = 5,
	.rx_dma_hdreq = 5,
	.fifo_depth = 8,
	.burst_type = SOCLE_DMA_BURST_INCR4,
#else
	.tx_dma_ch = 4,
	.rx_dma_ch = 5,
	.tx_dma_hdreq = 0,
	.rx_dma_hdreq = 1,
	.fifo_depth = 32,
	.burst_type = SOCLE_DMA_BURST_INCR16,
#endif
};

static u64 socle_i2s_0_dma_mask = 0xffffffffUL;
static struct resource socle_i2s_0_resources[] = {
	[0] = {
		.start = SOCLE_I2S0,
		.end = SOCLE_I2S0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2S0,
		.end = IRQ_I2S0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_i2s_0_device = {
	.name = "socle_snd",
	.id = 0,
	.dev = {
		.dma_mask = &socle_i2s_0_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_i2s_0_resources,
	.num_resources = ARRAY_SIZE(socle_i2s_0_resources),
};

void __init socle_add_device_snd_i2s(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_I2S_TX_RX);
#endif

	platform_device_add_data(&socle_i2s_0_device, (void *)&socle_i2s_data, sizeof(struct socle_i2s_platform_data));
	platform_device_register(&socle_i2s_0_device);
	
}
#elif defined(CONFIG_SND_SOC_SOCLE)
void __init socle_add_device_snd_i2s(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
        socle_scu_dev_enable(SOCLE_DEVCON_I2S_TX_RX);
#endif
}
#else
void __init socle_add_device_snd_i2s(void) {}
#endif

/* --------------------------------------------------------------------
 *  USB Device (Gadget)
 * -------------------------------------------------------------------- */

#if defined(CONFIG_USB_GADGET_SOCLE) || defined(CONFIG_USB_GADGET_SOCLE_MODULE)
static u64 socle_udc_dma_mask = 0xffffffffUL;
static struct resource socle_udc_resources[] = {
	[0] = {
		.start = SOCLE_UDC0,
		.end = SOCLE_UDC0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UDC0,
		.end = IRQ_UDC0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_udc_device = {
	.name = "socle_udc",
	.id = 0,
	.dev = {
		.dma_mask = &socle_udc_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_udc_resources,
	.num_resources = ARRAY_SIZE(socle_udc_resources),
};

void __init socle_add_device_udc(void)
{
#if defined (CONFIG_ARCH_PDK_PC7210) ||  defined(CONFIG_ARCH_P7DK)
	socle_scu_usb_tranceiver_upstream();
#endif
	platform_device_register(&socle_udc_device);
	
}
#else
void __init socle_add_device_udc(void) {}
#endif


/* --------------------------------------------------------------------
 *  USB OTG Device (Gadget)
 * -------------------------------------------------------------------- */

#if defined(CONFIG_USB_GADGET_SOCLE_OTG) || defined(CONFIG_USB_GADGET_SOCLE_OTG_MODULE)
static u64 socle_otg_udc_dma_mask = 0xffffffffUL;
static struct resource socle_otg_udc_0_resources[] = {
	[0] = {
		.start = SOCLE_OTG_UDC0,
		.end = SOCLE_OTG_UDC0,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_OTG_UDC0,
		.end = IRQ_OTG_UDC0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_otg_udc_0_device = {
	.name = "socle_otg_udc",
	.id = 0,
	.dev = {
		.dma_mask = &socle_otg_udc_dma_mask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = NULL,
	},
	.resource = socle_otg_udc_0_resources,
	.num_resources = ARRAY_SIZE(socle_otg_udc_0_resources),
};

void __init socle_add_device_otg_udc(void)
{
	platform_device_register(&socle_otg_udc_0_device);

}
#else
void __init socle_add_device_otg_udc(void) {}
#endif

#ifdef CONFIG_ANDROID_SYSTEM
               //20090527 leonid+ for adb
struct platform_device socle_android_adb = {
       .name   = "android_usb",
        .id             = -1,
};
void __init socle_add_device_android_adb(void)
{
        platform_device_register(&socle_android_adb);
}
#endif


/* --------------------------------------------------------------------
 *  HDMA PSEUDO 
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SOCLE_HDMA_PSEUDO) || defined(CONFIG_SOCLE_HDMA_PSEUDO_MODULE)
static struct resource socle_hdma_pseudo_resource[] = {
	[0] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_MEM,	
	},	
	[1] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_IRQ,	
	}
};

struct platform_device socle_device_hdma_pseudo = {	
	.name		  = "socle-hdma-pseudo",	
		.id		  = -1,	
		.num_resources	  = ARRAY_SIZE(socle_hdma_pseudo_resource),	
		.resource	  = socle_hdma_pseudo_resource,
};

void __init socle_add_device_hdma_pseudo(void)
{	
	platform_device_register(&socle_device_hdma_pseudo);
}
#else
void __init socle_add_device_hdma_pseudo(void) {}
#endif

//20080513 leonid+ for MSMV
/* --------------------------------------------------------------------
 *  CURRENT SENSE
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SOCLE_CURRENT_SENSE_LM3824) || defined(CONFIG_SOCLE_CURRENT_SENSE_LM3824_MODULE)
static struct resource socle_current_sense_resource[] = {
	[0] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_MEM,	
	},	
	[1] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_IRQ,	
	}
};

struct platform_device socle_device_current_sense = {	
	.name		  = "socle-current-sense",	
		.id		  = -1,	
		.num_resources	  = ARRAY_SIZE(socle_current_sense_resource),	
		.resource	  = socle_current_sense_resource,
};

void __init socle_add_device_current_sense(void)
{	
	platform_device_register(&socle_device_current_sense);
}
#else
void __init socle_add_device_current_sense(void) {}
#endif


//20080625 leonid+ for MSMV
//MSMV CHAR 
#if defined(CONFIG_SOCLE_MSMV) || defined(CONFIG_SOCLE_MSMV_MODULE)
static struct resource socle_msmv_resource[] = {
	[0] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_MEM,	
	},	
	[1] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_IRQ,	
	}
};

struct platform_device socle_device_msmv = {	
	.name		  = "socle-msmv",	
		.id		  = -1,	
		.num_resources	  = ARRAY_SIZE(socle_msmv_resource),	
		.resource	  = socle_msmv_resource,
};

void __init socle_add_device_msmv(void)
{	
	platform_device_register(&socle_device_msmv);
}
#else
void __init socle_add_device_msmv(void) {}
#endif


//20080625 leonid+ for MSMV SRAM MMAP
//MSMV CHAR  SRAM MMAP
#if defined(CONFIG_SOCLE_MSMV_SRAM) || defined(CONFIG_SOCLE_MSMV_SRAM_MODULE)
static struct resource socle_msmv_sram_resource[] = {
	[0] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_MEM,	
	},	
	[1] = {		
		.start = 0,		
		.end   = 0,		
		.flags = IORESOURCE_IRQ,	
	}
};

struct platform_device socle_device_msmv_sram = {	
	.name		  = "socle-msmv-sram",	
		.id		  = -1,	
		.num_resources	  = ARRAY_SIZE(socle_msmv_sram_resource),	
		.resource	  = socle_msmv_sram_resource,
};

void __init socle_add_device_msmv_sram(void)
{	
	platform_device_register(&socle_device_msmv_sram);
}
#else
void __init socle_add_device_msmv_sram(void) {}
#endif


//INR
#if defined(CONFIG_SOCLE_INR) || defined(CONFIG_SOCLE_INR_MODULE)
static struct resource socle_inr_resource[] = {
	[0] = {	
		.start		= SOCLE_NOR_FLASH0,
		.end		= SOCLE_NOR_FLASH0 + SOCLE_NOR_FLASH_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
};

struct platform_device socle_device_inr = {	
	.name			= "socle-inr",	
	.id				= -1,	
	.num_resources	= ARRAY_SIZE(socle_inr_resource),	
	.resource		= socle_inr_resource,
};

void __init socle_add_device_inr(void)
{	
	platform_device_register(&socle_device_inr);
}
#else
void __init socle_add_device_inr(void) {}
#endif

/* --------------------------------------------------------------------
 *  USB Host
 * -------------------------------------------------------------------- */
 /* SOCLE EHCI USB Host Controller */
#if defined(CONFIG_SOCLE_EHCI) || defined(CONFIG_SOCLE_EHCI_MODULE)
static struct resource ehci_0_hcd_socle_resource[] = {
	[0] = {
		.start = SOCLE_UHC0 + 0x100,
		.end   = SOCLE_UHC0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UHC0,
		.end   = IRQ_UHC0,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 socle_ehci_dev_usb_dmamask = 0xffffffffUL;

struct platform_device ehci_0_hcd_socle_device = {
	.name		  = "SOCLE_EHCI",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(ehci_0_hcd_socle_resource),
	.resource	  = ehci_0_hcd_socle_resource,
	.dev              = {
		.dma_mask = &socle_ehci_dev_usb_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}	
};

#ifdef SOCLE_UHC1
static struct resource ehci_1_hcd_socle_resource[] = {
	[0] = {
		.start = SOCLE_UHC1 + 0x100,
		.end   = SOCLE_UHC1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UHC1,
		.end   = IRQ_UHC1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device ehci_1_hcd_socle_device = {
	.name		  = "SOCLE_EHCI",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(ehci_1_hcd_socle_resource),
	.resource	  = ehci_1_hcd_socle_resource,
	.dev              = {
		.dma_mask = &socle_ehci_dev_usb_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}	
};
#endif
#ifdef SOCLE_UHC2
static struct resource ehci_2_hcd_socle_resource[] = {
        [0] = {
                .start = SOCLE_UHC2 + 0x100,
                .end   = SOCLE_UHC2 + SZ_4K - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_UHC2,
                .end   = IRQ_UHC2,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device ehci_2_hcd_socle_device = {
        .name             = "SOCLE_EHCI",
        .id               = 2,
        .num_resources    = ARRAY_SIZE(ehci_2_hcd_socle_resource),
        .resource         = ehci_2_hcd_socle_resource,
        .dev              = {
                .dma_mask = &socle_ehci_dev_usb_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};
#endif


void __init socle_add_device_ehci(void)
{
#ifdef CONFIG_ARCH_PDK_PC9002
        socle_mp_gpio_set_port_num_value(MP_PA,6,0);
#endif

#if defined (CONFIG_ARCH_PDK_PC7210) ||  defined(CONFIG_ARCH_P7DK)
	socle_scu_usb_tranceiver_downstream();
#endif

	platform_device_register(&ehci_0_hcd_socle_device);

#ifdef SOCLE_UHC1
	platform_device_register(&ehci_1_hcd_socle_device);
#endif
#ifdef SOCLE_UHC2
	platform_device_register(&ehci_2_hcd_socle_device);
#endif
}
#else
void __init socle_add_device_ehci(void) {}
#endif

/* SOCLE-OHCI USB Host Controller */
#if defined(CONFIG_SOCLE_OHCI) || defined(CONFIG_SOCLE_OHCI_MODULE)
static struct resource ohci_hcd_socle_resource[] = {
	[0] = {
		.start = SOCLE_UHC0,
		.end   = SOCLE_UHC0 + 0x100 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UHC0,
		.end   = IRQ_UHC0,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 socle_ohci_dev_usb_dmamask = 0xffffffffUL;

struct platform_device ohci_hcd_socle_device = {
	.name		  = "SOCLE_OHCI",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(ohci_hcd_socle_resource),
	.resource	  = ohci_hcd_socle_resource,
	.dev              = {
		.dma_mask = &socle_ohci_dev_usb_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

#ifdef CONFIG_SOCLE_DUAL_PORT

static struct resource ohci1_hcd_socle_resource[] = {
	[0] = {
		.start = SOCLE_UHC1,
		.end   = SOCLE_UHC1 + 0x100 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UHC1,
		.end   = IRQ_UHC1,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 socle_ohci1_dev_usb_dmamask = 0xffffffffUL;

struct platform_device ohci1_hcd_socle_device = {
	.name		  = "SOCLE_OHCI",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(ohci1_hcd_socle_resource),
	.resource	  = ohci1_hcd_socle_resource,
	.dev              = {
		.dma_mask = &socle_ohci1_dev_usb_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

#endif

void __init socle_add_device_ohci(void)
{
#ifdef CONFIG_ARCH_PDK_PC9002
        socle_mp_gpio_set_port_num_value(MP_PA,6,0);
#endif

#if defined (CONFIG_ARCH_PDK_PC7210) ||  defined(CONFIG_ARCH_P7DK)
	socle_scu_usb_tranceiver_downstream();
#endif

	platform_device_register(&ohci_hcd_socle_device);
}
#else
void __init socle_add_device_ohci(void) {}
#endif

/* --------------------------------------------------------------------
 *  Ethernet
 * -------------------------------------------------------------------- */

//20080512 leonid+ for ethernet probe modify
#if defined(CONFIG_SOCLE_ETHER) || defined(CONFIG_SOCLE_ETHER_MODULE)
static u64 socle_eth_dmamask = 0xffffffffUL;
static struct resource socle_mac0_resources[] = {
	[0] = {
		.start = SOCLE_MAC0,
		.end = SOCLE_MAC0 + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MAC0,
		.end = IRQ_MAC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device socle_eth0_device = {
	.name		= "socle_ether",
	.id		= 0,
	.dev		= {
				.dma_mask		= &socle_eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
	},
	.resource	= socle_mac0_resources,
	.num_resources = ARRAY_SIZE(socle_mac0_resources),
};

#ifdef SOCLE_MAC1
static struct resource socle_mac1_resources[] = {
	[0] = {
		.start = SOCLE_MAC1,
		.end = SOCLE_MAC1 + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MAC1,
		.end = IRQ_MAC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device socle_eth1_device = {
	.name		= "socle_ether",
	.id		= 1,
	.dev		= {
				.dma_mask		= &socle_eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
	},
	.resource	= socle_mac1_resources,
	.num_resources = ARRAY_SIZE(socle_mac1_resources),
};
#endif
void __init socle_add_device_eth(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_MAC);
#endif

	platform_device_register(&socle_eth0_device);
#ifdef SOCLE_MAC1
	platform_device_register(&socle_eth1_device);
#endif
}
#else
void __init socle_add_device_eth(void) {}
#endif

/* --------------------------------------------------------------------
 *  NAND	
 *  PDK-PC9002 NAND I/O pad GPIO PO(1,2,5,6,7) & PP(0,1,4,5,6,7) with the I/O pad
 *  So enable SCU NAND for nand workable
 * -------------------------------------------------------------------- */
#if defined(CONFIG_MTD_SOCLE_NAND) || defined(CONFIG_MTD_SOCLE_NAND_MODULE)

#if (defined(CONFIG_ARCH_MSMV)) 
#define DEFAULT_NUM_PARTITIONS 11
static struct mtd_partition nand_partitions[] = {
     {
	  .name = "Boot Area",
	  .offset = 0x40000,		//0x40000 - 0x80000 
	  .size = 256 * 1024,
     },
     {
	  .name = "Diag Area",
	  .offset = MTDPART_OFS_APPEND,	//0x80000 - 0x180000 
	  .size = 1 * 1024 * 1024,					
     },
     {
	  .name = "Kernel",
	  .offset = MTDPART_OFS_APPEND,	//0x180000 - 0x1480000 
	  .size = 19 * 1024 * 1024,
     },
     {
	  .name = "Root Filesystem",
	  .offset = MTDPART_OFS_APPEND,	//0x1480000 - 0x3480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data0",
	  .offset = MTDPART_OFS_APPEND,	//0x3480000 - 0x5480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data1",
	  .offset = MTDPART_OFS_APPEND,	//0x3480000 - 0x5480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data2",
	  .offset = MTDPART_OFS_APPEND,	//0x5480000 - 0x7480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data3",
	  .offset = MTDPART_OFS_APPEND,	//0x7480000 - 0x9480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data4",
	  .offset = MTDPART_OFS_APPEND,	//0x9480000 - 0xb480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data5",
	  .offset = MTDPART_OFS_APPEND,	//0xb480000 - 0xc480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data6",
	  .offset = MTDPART_OFS_APPEND,	//0xc480000 - 
	  .size = MTDPART_SIZ_FULL,
     },
};

#elif defined(CONFIG_ARCH_MDK_3D) || defined(CONFIG_ARCH_MDK_FHD)
#define NAND_RESERVED (0x02000000)
static struct mtd_partition nand_partitions[] = {
     {
	  .name = "data0",
	  .offset = NAND_RESERVED,
	  .size = 0x0a000000, // 160M					
     },
     {
	  .name = "system",
	  .offset = MTDPART_OFS_APPEND,
	  .size = 0x08000000, //128M
	  .mask_flags	= 0, //force read-only
     },
     {
	  .name = "userdata",
	  .offset = MTDPART_OFS_APPEND,
	  .size = 0x10000000, //256M
     },
     {
	  .name = "data1",
	  .offset = MTDPART_OFS_APPEND,
	  .size = MTDPART_SIZ_FULL, //512-64M
     },
};
#else

#ifndef CONFIG_ANDROID_SYSTEM
#define DEFAULT_NUM_PARTITIONS 5
#else
#define DEFAULT_NUM_PARTITIONS 4
#endif
static struct mtd_partition nand_partitions[] = {
     {
	  .name = "Boot Area",
	  .offset = 0x40000,		//0x40000 - 0x80000 
	  .size = 256 * 1024,
     },
     {
	  .name = "Diag Area",
	  .offset = MTDPART_OFS_APPEND,	//0x80000 - 0x180000 
	  .size = 1 * 1024 * 1024,					
     },
     {
	  .name = "Kernel",
	  .offset = MTDPART_OFS_APPEND,	//0x180000 - 0x1480000 
	  .size = 19 * 1024 * 1024,
     },
#ifndef CONFIG_ANDROID_SYSTEM
     {
	  .name = "Root Filesystem",
	  .offset = MTDPART_OFS_APPEND,	//0x1480000 - 0x3480000 
	  .size = 32 * 1024 * 1024,
     },
     {
	  .name = "data0",
	  .offset = MTDPART_OFS_APPEND,	//0x3480000 - 0x5480000 - 
	  .size = MTDPART_SIZ_FULL,
     },
#else
     {
	  .name = "system",
	  .offset = MTDPART_OFS_APPEND,
	  .size = 55 * 1024 * 1024,
     },
     {
	  .name = "userdata",
	  .offset = MTDPART_OFS_APPEND,
	  .size = MTDPART_SIZ_FULL,
     },
#endif
};
#endif

static struct resource socle_nand_resources[] = {
	{
		.start	= SOCLE_NAND0,
		.end	= SOCLE_NAND0 + SZ_64M - 1,
		.flags	= IORESOURCE_MEM,
	}
};

static struct flash_platform_data socle_nand_data = {
	.map_name	= "socle nand",
	.width		= 2,
	.parts		= nand_partitions,
	.nr_parts		= ARRAY_SIZE(nand_partitions),
};

static struct platform_device socle_nand_device = {
	.name		= "socle-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &socle_nand_data,
	},
	.resource	= socle_nand_resources,
	.num_resources	= ARRAY_SIZE(socle_nand_resources),
};

void __init socle_add_device_nand(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220)
	socle_scu_dev_enable(SOCLE_DEVCON_NFC);
#endif

#if (defined(CONFIG_ARCH_CDK) || defined(CONFIG_ARCH_PDK_PC9002) || defined(CONFIG_ARCH_SCDK))
	socle_scu_nand_if_set();
#endif
	platform_device_register(&socle_nand_device);

}
#else
void __init socle_add_device_nand(void) {}
#endif


#if defined(CONFIG_MTD_NAND_SOCLE) || defined(CONFIG_MTD_NAND_SOCLE_MODULE)
static struct mtd_partition nfc_partitions[] = {
#if 1//for 512 Mb, 2k page
     {
          .name = "Boot Area",
          .offset = 0x40000,            //0x40000 - 0x80000
          .size = 256 * 1024,
     },
     {
          .name = "Diag Area",
          .offset = MTDPART_OFS_APPEND, //0x80000 - 0x180000
          .size = 1 * 1024 * 1024,
     },
     {
          .name = "Kernel",
          .offset = MTDPART_OFS_APPEND, //0x180000 - 0x1480000
          .size = 19 * 1024 * 1024,
     },
#ifndef CONFIG_ANDROID_SYSTEM
     {
          .name = "Root Filesystem",
          .offset = MTDPART_OFS_APPEND, //0x1480000 - 0x3480000
          .size = 32 * 1024 * 1024,
     },
     {
          .name = "data0",
          .offset = MTDPART_OFS_APPEND, //0x3480000 - 0x5480000 
          .size = 32 * 1024 * 1024,
     },
     {
          .name = "data1",
          .offset = MTDPART_OFS_APPEND, //0x5480000 - 
          .size = MTDPART_SIZ_FULL,
     },
#else
     {
          .name = "system",
          .offset = MTDPART_OFS_APPEND,
          .size = 55 * 1024 * 1024,
     },
     {
          .name = "userdata",
          .offset = MTDPART_OFS_APPEND,
          .size = MTDPART_SIZ_FULL,
     },
#endif
#else // for smauang 2GB, 4k page 
     {
	  .name = "IPL", // 0x00000000
	  .offset = 0,		
	  .size = 128* 1024,

     },
     {
		.name = "Env",//0x00020000
		.offset = MTDPART_OFS_APPEND,
		.size = 128 * 1024,
	},
	{
		.name = "Uboot",//0x00040000
		.offset = MTDPART_OFS_APPEND,
		.size = 256* 1024,
	},
	{
		.name = "Diag",//0x00080000
		.offset = MTDPART_OFS_APPEND,
		.size = 1024* 1024,
	},
	{
		.name = "Kernel",//0x00180000, 30.5M
		.offset = MTDPART_OFS_APPEND,
		.size = (30*1024*1024 + 512*1024),
	},
	{
		.name = "File System",//0x02000000, 32M
		.offset = MTDPART_OFS_APPEND,
		.size = 32*1024*1024,
	},
	{
		.name = "data0 ",//0x04000000, 32M
		.offset = MTDPART_OFS_APPEND,
		.size = 32*1024*1024,
	},
	{
		.name = "data1 ",//0x06000000, 128M
		.offset = MTDPART_OFS_APPEND,
		.size = 128*1024*1024,
	},
	{
		.name = "android_system ",//0x0E000000, 64M
		.offset = MTDPART_OFS_APPEND,
		.size = 64*1024*1024,
	},
	{
		.name = "android_data ",//0x12000000
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	},
#endif     
};

static struct resource socle_nfc_resources[] = {
        [0] = {
                .start = SOCLE_NAND0,
                .end = SOCLE_NAND0 + SZ_128K - 1,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_NAND0,
                .end = IRQ_NAND0,
                .flags = IORESOURCE_IRQ,
        },
};

static struct flash_platform_data socle_nfc_data = {
	.map_name	= "socle nfc",
	.width		= 2,
	.parts		= nfc_partitions,
	.nr_parts		= ARRAY_SIZE(nfc_partitions),
};

static struct platform_device socle_device_nfc = {
        .name           = "socle-nfc",
        .id                     = -1,
        .dev		= {
		.platform_data	= &socle_nfc_data,
	 },
        .resource = socle_nfc_resources,
        .num_resources = ARRAY_SIZE(socle_nfc_resources),
};

void __init socle_add_device_nfc(void)
{
#if defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_NFC);
#endif
        platform_device_register(&socle_device_nfc);
}
#else
void __init socle_add_device_nfc(void) {}
#endif

/* --------------------------------------------------------------------
 *  RTC
 * -------------------------------------------------------------------- */

#if defined(CONFIG_RTC_DRV_SOCLE) || defined(CONFIG_RTC_DRV_SOCLE_MODULE)
static struct resource socle_rtc_resources[] = {
	[0] = {
		.start = SOCLE_RTC0,
		.end = SOCLE_RTC0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_RTC0,
		.end = IRQ_RTC0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_rtc_device = {
	.name		  = "socle-rtc",
	.id		  = -1,
	.resource = socle_rtc_resources,
	.num_resources = ARRAY_SIZE(socle_rtc_resources),
};

void __init socle_add_device_rtc(void)
{
	platform_device_register(&socle_rtc_device);
}
#else
void __init socle_add_device_rtc(void) {}
#endif

/* --------------------------------------------------------------------
 *  Watchdog
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SOCLE_WATCHDOG) || defined(CONFIG_SOCLE_WATCHDOG_MODULE)
static struct platform_device socle_device_wdt = {
	.name		= "socle-wdt",
	.id		= -1,
	.num_resources	= 0,
};

void __init socle_add_device_watchdog(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_wdt_reset_enable(1);		//20100629 leonid fix for watchdog reset 
#endif

	platform_device_register(&socle_device_wdt);
}
#else
void __init socle_add_device_watchdog(void) {}
#endif

/* --------------------------------------------------------------------
 *  VOP
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SOCLE_VOP) || defined(CONFIG_SOCLE_VOP_MODULE)
static struct resource socle_vop_resources[] = {
	[0] = {
		.start = SOCLE_VOP0,
		.end = SOCLE_VOP0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VOP0,
		.end = IRQ_VOP0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device socle_device_vop = {
	.name		= "socle-vop",
	.id			= -1,
	.resource = socle_vop_resources,
	.num_resources = ARRAY_SIZE(socle_vop_resources),
};

void __init socle_add_device_vop(void)
{
#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_LCDC_VOP);
#endif

	platform_device_register(&socle_device_vop);
}
#else
void __init socle_add_device_vop(void) {}
#endif

/* --------------------------------------------------------------------
 *  VIP
 * -------------------------------------------------------------------- */
#if defined(CONFIG_VIDEO_SOCLE_VIP) || defined(CONFIG_VIDEO_SOCLE_VIP_MODULE)

static int camera_set_capture(struct soc_camera_platform_info *info,
			      int enable)
{

	return 0;
}


static struct soc_camera_platform_info camera_info = {
	.iface = 0,
	.format_name = "YUV422",
	.format_depth = 16,
	.format = {
		.pixelformat = V4L2_PIX_FMT_YUV422P,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.width = 640,
		.height = 480,
	},
	.bus_param = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER | SOCAM_DATAWIDTH_8 | SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_HIGH,
	.set_capture = camera_set_capture,
};

static struct platform_device camera_device = {
	.name		= "soc_camera_platform",
	.dev		= {
		.platform_data	= &camera_info,
	},
};


int __init platform_resource_setup_memory(struct platform_device *pdev,
					  char *name, unsigned long memsize)
{
	struct resource *r;
	dma_addr_t dma_handle;
	void *buf;

	r = pdev->resource + pdev->num_resources - 1;
	if (r->flags) {
		pr_warning("%s: unable to find empty space for resource\n",
			name);
		return -EINVAL;
	}

	//memchunk_cmdline_override(name, &memsize);
	//if (!memsize)
	//	return 0;

	buf = dma_alloc_coherent(NULL, memsize, &dma_handle, GFP_KERNEL);
	if (!buf) {
		printk("%s: unable to allocate memory\n", name);
		return -ENOMEM;
	}
	//printk("platform_resource_setup_memory: buf=%x\n",buf);
	memset(buf, 0, memsize);

	r->flags = IORESOURCE_MEM;
	r->start = dma_handle;
	r->end = r->start + memsize - 1;
	r->name = name;
	//printk("platform_resource_setup_memory: r->start=%x\n",r->start);
	return 0;
}

static struct socle_vip_camera_info socle_vip_camera_info = {
	.flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER | SOCAM_DATAWIDTH_8 | SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_HIGH,
};

static struct resource socle_vip_resources[] = {
	[0] = {
		.start = SOCLE_VIP0,
		.end = SOCLE_VIP0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VIP0,
		.end = IRQ_VIP0,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		/* place holder for contiguous memory */
	},
};
static struct platform_device socle_device_vip = {
	.name		= "socle_camera",
	.resource = socle_vip_resources,
	.num_resources = ARRAY_SIZE(socle_vip_resources),
	.dev		= {
		.platform_data	= &socle_vip_camera_info,
		.coherent_dma_mask	= 0xffffffff,
	},
};

void __init socle_add_device_vip(void)
{
#if defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_VIP);
#endif
	platform_resource_setup_memory(&socle_device_vip, "vip", 2 << 20);
	platform_device_register(&socle_device_vip);
	platform_device_register(&camera_device);
}
#else
void __init socle_add_device_vip(void) {}
#endif

/* --------------------------------------------------------------------
 *  SOCLE CLCD
 * -------------------------------------------------------------------- */


#if defined(CONFIG_FB_SOCLELCD) || defined(CONFIG_FB_SOCLELCD_MODULE)

static struct resource socle_lcdc_resources[] = {
	[0] = {
		.start = SOCLE_LCD0,
		.end = SOCLE_LCD0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CLCD0,
		.end = IRQ_CLCD0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device socle_device_clcd = {
	.name		  = "socle-clcd",
	.id		  = 0x41110,
	.dev		= {
		.bus_id			= "mb:16",
		.coherent_dma_mask	= ~0,
	},
	.resource	= socle_lcdc_resources,
	.num_resources	= ARRAY_SIZE(socle_lcdc_resources),
};

void __init socle_add_device_clcd(void)
{
	platform_device_register(&socle_device_clcd);
}
#else
void __init socle_add_device_clcd(void) {}
#endif

/* --------------------------------------------------------------------
 *  HD VIP
 * -------------------------------------------------------------------- */
// cyli add
#if defined(CONFIG_VIDEO_SOCLE_HDVIP) || defined(CONFIG_VIDEO_SOCLE_HDVIP_MODULE)

static struct resource socle_hdvip_resources[] = {
	[0] = {
		.start = SOCLE_HDVIP0,
		.end = SOCLE_HDVIP0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_HDVIP0,
		.end = IRQ_HDVIP0,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		/* place holder for contiguous memory */
	},
};
static struct platform_device socle_device_hdvip = {
	.name		= "socle_hdvip",
	.resource = socle_hdvip_resources,
	.num_resources = ARRAY_SIZE(socle_hdvip_resources),
};


void __init socle_add_device_viphd(void)
{
	platform_device_register(&socle_device_hdvip);
}
#else
void __init socle_add_device_viphd(void) {}
#endif

/* --------------------------------------------------------------------
 *  SOCLE FB
 * -------------------------------------------------------------------- */

#if defined(CONFIG_FB_SOCLE) || defined(CONFIG_FB_SOCLE_MODULE)

static struct resource socle_lcdc_resources[] = {
	[0] = {
		.start = SOCLE_LCD0,
		.end = SOCLE_LCD0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CLCD0,
		.end = IRQ_CLCD0,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 socle_device_lcd_dmamask = 0xffffffffUL;
struct platform_device socle_lcd_device = {
	.name		  = "socle-lcd",
	.id		  = -1,
	.dev		= {
		.dma_mask		= &socle_device_lcd_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.resource	= socle_lcdc_resources,
	.num_resources	= ARRAY_SIZE(socle_lcdc_resources),
};

void __init socle_add_lcd_device(void)
{

#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	socle_scu_dev_enable(SOCLE_DEVCON_LCDC);
	socle_scu_lcdc_clk_input_mpll_outpput();
#endif

#ifdef CONFIG_ARCH_PDK_PC7210
	iowrite32((ioread32(0x1805810c) | 0xa0), 0x1805810c);		// LCD priority
#endif

#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	//iowrite32((ioread32(IO_ADDRESS(0x1800400c)) | 0x20), IO_ADDRESS(0x1800400c));		// LCD priority
	iowrite32((ioread32(IO_ADDRESS(0x1800400c)) | 0xa0), IO_ADDRESS(0x1800400c));		// LCD priority
#endif

	platform_device_register(&socle_lcd_device);
}
#else
void __init socle_add_lcd_device(void) {}
#endif


//leonid+
/* --------------------------------------------------------------------
 *  SOCLE LCD Lighter
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SOCLE_LCD_LIGHTER) || defined(CONFIG_SOCLE_LCD_LIGHTER_MODULE)

struct platform_device socle_device_lcd_lighter = {
	.name		  = "socle-lcd-lighter",
	.id		  = -1,
};

void __init socle_add_device_lcd_lighter(void)
{
	platform_device_register(&socle_device_lcd_lighter);
}
#else
void __init socle_add_device_lcd_lighter(void) {}
#endif


/* --------------------------------------------------------------------
 *  SOCLE LED Backlight
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SOCLE_LED_BACKLIGHT) || defined(CONFIG_SOCLE_LED_BACKLIGHT_MODULE)

struct platform_device socle_device_led_backlight = {
       .name             = "socle-led-backlight",
       .id               = -1,
};

void __init socle_add_device_led_backlight(void)
{
       platform_device_register(&socle_device_led_backlight);
}
#else
void __init socle_add_device_led_backlight(void) {}
#endif


/* --------------------------------------------------------------------
 *  SDHC Slave
 * -------------------------------------------------------------------- */
#if defined(CONFIG_MMC_SOCLE_SDHC_SLAVE) || defined(CONFIG_MMC_SOCLE_SDHC_SLAVE_MODULE)
static struct resource socle_sdhc_slave_resource[] = {
	[0] = {
		.start = SOCLE_SDCS0,
		.end   = SOCLE_SDCS0 + SZ_128K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDCS,
		.end   = IRQ_SDCS,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device socle_device_sdhc_slave = {
	.name		  = "sdhc-slave",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(socle_sdhc_slave_resource),
	.resource	  = socle_sdhc_slave_resource,
};

void __init socle_add_device_sdhc_slave(void)
{
	platform_device_register(&socle_device_sdhc_slave);
}
#else
void __init socle_add_device_sdhc_slave(void) {}
#endif

 /* --------------------------------------------------------------------
 *  F75111 Keypad
 * -------------------------------------------------------------------- */

#if defined(CONFIG_KEYBOARD_F75111) || defined(CONFIG_KEYBOARD_F75111_MODULE)
#include <linux/input.h>
static unsigned char f75111kpd_keycode[]  = {
        [0]             = KEY_F4, // END CALL
        [1]             = KEY_HOME,
        [2]             = KEY_RIGHT,
        [3]             = KEY_LEFT,
        [4]             = KEY_DOWN,
        [5]             = KEY_UP,
        [6]             = KEY_ENTER,
        [7]             = KEY_F5,  // CENTER
        [8]             = KEY_F1,  // MENU
        [9]             = KEY_ESC, // BACK
};

struct platform_device f75111_device_kpd = {
        .name   = "f75111-keypad",
        .id             = -1,
        .dev    = {
                .platform_data  = &f75111kpd_keycode,
        },
};

void __init f75111_add_device_kpd(void)
{
        platform_device_register(&f75111_device_kpd);
}

#else
void __init f75111_add_device_kpd(void) {}
#endif

//20100622 cyli+ for gpio power management
/* *********************************************************************
 *  SOCLE Service Driver
 * *********************************************************************/

/* --------------------------------------------------------------------
 *  SOCLE GPIO
 * -------------------------------------------------------------------- */

static struct resource socle_gpio_resource[] = {
	{
		.start = SOCLE_GPIO0,
		.end   = SOCLE_GPIO0 + SZ_4K - 1,
		.name = "GPIO 0",
		.flags = IORESOURCE_MEM,
	},
#if SOCLE_GPIO_GP1 == 1
	{
		.start = SOCLE_GPIO1,
		.end   = SOCLE_GPIO1 + SZ_4K - 1,
		.name = "GPIO 1",
		.flags = IORESOURCE_MEM,
	},
#endif
#if SOCLE_GPIO_GP2 == 1
	{
		.start = SOCLE_GPIO2,
		.end   = SOCLE_GPIO2 + SZ_4K - 1,
		.name = "GPIO 2",
		.flags = IORESOURCE_MEM,
	},
#endif
#if SOCLE_GPIO_GP3 == 1
	{
		.start = SOCLE_GPIO3,
		.end   = SOCLE_GPIO3 + SZ_4K - 1,
		.name = "GPIO 3",
		.flags = IORESOURCE_MEM,
	},
#endif

#ifdef SOCLE_GPIO_WITH_INT
	{
		.start = IRQ_GPIO0,
		.name = "GPIO INT Service 0",
		.flags = IORESOURCE_IRQ,
	},
#if SOCLE_GPIO_GP1 == 1
	{
		.start = IRQ_GPIO1,
		.name = "GPIO INT Service 1",
		.flags = IORESOURCE_IRQ,
	},
#endif
#if SOCLE_GPIO_GP2 == 1
	{
		.start = IRQ_GPIO2,
		.name = "GPIO INT Service 2",
		.flags = IORESOURCE_IRQ,
	},
#endif
#if SOCLE_GPIO_GP3 == 1
	{
		.start = IRQ_GPIO3,
		.name = "GPIO INT Service 3",
		.flags = IORESOURCE_IRQ,
	},
#endif
#endif	// SOCLE_GPIO_WITH_INT
};

static struct platform_device socle_device_gpio = {
	.name		= "socle-gpio",
	.id			= -1,
	.num_resources	= ARRAY_SIZE(socle_gpio_resource),
	.resource			= socle_gpio_resource,
};

extern void __init
socle_add_device_gpio(void)
{
	platform_device_register(&socle_device_gpio);
}

/* --------------------------------------------------------------------
 *  On2
 * -------------------------------------------------------------------- */
#if defined(CONFIG_UIO_ON2) || defined(CONFIG_UIO_ON2_MODULE)

#include <linux/uio_driver.h>

struct uio_info uioinfo= {
	.name="socle-on2",
	.version="0.0.1",
	.irq=-1,
};

static struct resource socle_on2_resource[] = {
	[0] = {
		.start = 0x20000000,
		.end   = 0x20000000 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = 0x48000000,
		.end   = 0x4fffffff,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device socle_device_on2 = {
	.name		  = "socle-on2",
	.num_resources	  = ARRAY_SIZE(socle_on2_resource),
	.resource	  = socle_on2_resource,
	 .dev    = {
                .platform_data  = &uioinfo,
        },
};

void __init socle_add_device_on2(void)
{
	platform_device_register(&socle_device_on2);
}
#else
void __init socle_add_device_on2(void) {}
#endif
