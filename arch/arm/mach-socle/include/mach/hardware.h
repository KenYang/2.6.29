/*
 *  linux/include/asm-arm/arch-ldk/hardware.h
 *
 *  This file contains the hardware definitions of the LDK.
 *
 *  Copyright (C) 2006 Socle-tech Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

//#include <mach/regs-pci.h>

#define IO_BASE			0xF0000000                 // VA of IO 
/* macro to get at IO space when running virtually */
#ifdef CONFIG_MMU
#ifndef CONFIG_ARCH_MDK_3D
#define IO_ADDRESS(x) (((x) | 0xf0000000) - 0x01000000 )
#else
#define IO_ADDRESS(x) (((x) | 0xf0000000) )
#endif
#else
#define IO_ADDRESS(x) (x)
#endif

//#define PCIO_BASE		IO_ADDRESS(SOCLE_PCI_IO_BASE)
//#define PCIMEM_BASE		IO_ADDRESS(SOCLE_PCI_MEM_BASE)

//PCI
//#define PCIBIOS_MIN_IO						0x800000
//#define PCIBIOS_MIN_MEM 					0x4000000
#define PCIBIOS_MIN_IO					0
#define PCIBIOS_MIN_MEM 				0

#define pcibios_assign_all_busses()			0

#endif

