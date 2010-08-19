/*
 * include/asm-arm/arch-socle/otg_devices.h
 *
 * Definitions for related flags or structures for
 * Socle OTG devices
 *
 * Maintainer: Leonid Cheng
 *
 * Copyright (C) Socle Tech. Corp.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifdef __KERNEL__
#ifndef _SOCLE_OTG_DEVICE_H_
#define _SOCLE_OTG_DEVICE_H_

#include <linux/types.h>


struct gianfar_platform_data {
	/* device specific information */
	u32	device_flags;
	/* board specific information */
	u32	board_flags;
	u32	bus_id;
	u32	phy_id;
	u8	mac_addr[6];
};

struct gianfar_mdio_data {
	/* board specific information */
	int	irq[32];
};

/* Flags related to gianfar device features */
#define SOCLE_GIANFAR_DEV_HAS_GIGABIT		0x00000001
#define SOCLE_GIANFAR_DEV_HAS_COALESCE		0x00000002
#define SOCLE_GIANFAR_DEV_HAS_RMON			0x00000004
#define SOCLE_GIANFAR_DEV_HAS_MULTI_INTR	0x00000008
#define SOCLE_GIANFAR_DEV_HAS_CSUM			0x00000010
#define SOCLE_GIANFAR_DEV_HAS_VLAN			0x00000020
#define SOCLE_GIANFAR_DEV_HAS_EXTENDED_HASH	0x00000040
#define SOCLE_GIANFAR_DEV_HAS_PADDING		0x00000080

/* Flags in gianfar_platform_data */
#define SOCLE_GIANFAR_BRD_HAS_PHY_INTR	0x00000001 /* set or use a timer */
#define SOCLE_GIANFAR_BRD_IS_REDUCED	0x00000002 /* Set if RGMII, RMII */

struct socle_i2c_platform_data {
	/* device specific information */
	u32	device_flags;
};

/* Flags related to I2C device features */
#define SOCLE_I2C_DEV_SEPARATE_DFSRR	0x00000001
#define SOCLE_I2C_DEV_CLOCK_5200		0x00000002

enum socle_usb2_operating_modes {
	SOCLE_USB2_MPH_HOST,
	SOCLE_USB2_DR_HOST,
	SOCLE_USB2_DR_DEVICE,
	SOCLE_USB2_DR_OTG,
};

enum socle_usb2_phy_modes {
	SOCLE_USB2_PHY_NONE,
	SOCLE_USB2_PHY_ULPI,
	SOCLE_USB2_PHY_UTMI,
	SOCLE_USB2_PHY_UTMI_WIDE,
	SOCLE_USB2_PHY_SERIAL,
};

struct socle_usb2_platform_data {
	/* board specific information */
	enum socle_usb2_operating_modes	operating_mode;
	enum socle_usb2_phy_modes		phy_mode;
	unsigned int			port_enables;
};

/* Flags in socle_usb2_mph_platform_data */
#define SOCLE_USB2_PORT0_ENABLED	0x00000001
#define SOCLE_USB2_PORT1_ENABLED	0x00000002

struct socle_spi_platform_data {
	u32 	initial_spmode;	/* initial SPMODE value */
	u16	bus_num;

	/* board specific information */
	u16	max_chipselect;
	void	(*activate_cs)(u8 cs, u8 polarity);
	void	(*deactivate_cs)(u8 cs, u8 polarity);
	u32	sysclk;
};

/* Ethernet interface (phy management and speed)
*/
enum enet_interface {
	ENET_10_MII,		/* 10 Base T,   MII interface */
	ENET_10_RMII,		/* 10 Base T,  RMII interface */
	ENET_10_RGMII,		/* 10 Base T, RGMII interface */
	ENET_100_MII,		/* 100 Base T,   MII interface */
	ENET_100_RMII,		/* 100 Base T,  RMII interface */
	ENET_100_RGMII,		/* 100 Base T, RGMII interface */
	ENET_1000_GMII,		/* 1000 Base T,  GMII interface */
	ENET_1000_RGMII,	/* 1000 Base T, RGMII interface */
	ENET_1000_TBI,		/* 1000 Base T,   TBI interface */
	ENET_1000_RTBI		/* 1000 Base T,  RTBI interface */
};

struct ucc_geth_platform_data {
	/* device specific information */
	u32			device_flags;
	u32			phy_reg_addr;

	/* board specific information */
	u32			board_flags;
	u8			rx_clock;
	u8			tx_clock;
	u32			phy_id;
	enum enet_interface	phy_interface;
	u32			phy_interrupt;
	u8			mac_addr[6];
};

/* Flags related to UCC Gigabit Ethernet device features */
#define SOCLE_UGETH_DEV_HAS_GIGABIT		0x00000001
#define SOCLE_UGETH_DEV_HAS_COALESCE	0x00000002
#define SOCLE_UGETH_DEV_HAS_RMON		0x00000004

/* Flags in ucc_geth_platform_data */
#define SOCLE_UGETH_BRD_HAS_PHY_INTR		0x00000001
				/* if not set use a timer */

#endif /* _SOCLE_OTG_DEVICE_H_ */
#endif /* __KERNEL__ */
