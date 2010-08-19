/* linux/include/asm-arm/arch-socle/regs-lcd.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#ifndef ___ASM_ARCH_REGS_LCD_H
#define ___ASM_ARCH_REGS_LCD_H 

/* LCD control registers */
#define SOCLE_LCD_CTRL0				0x00
#define SOCLE_LCD_CTRL1				0x04
#define SOCLE_LCD_YUV2RGB_CTRL			0x08
#define SOCLE_LCD_RESOLUTION			0x0C
#define SOCLE_LCD_H_TIMING			0x10
#define SOCLE_LCD_V_TIMING			0x14
#define SOCLE_LCD_STN_TIMING		0x18
#define SOCLE_LCD_LUT_ADDR			0x1C
#define SOCLE_LCD_PAGE0_ADDR		0x20
#define SOCLE_LCD_PAGE1_ADDR		0x24
#define SOCLE_LCD_LPAGE0_ADDR		0x28
#define SOCLE_LCD_LPAGE1_ADDR		0x2C
#define SOCLE_LCD_INTR_STS			0x40
#define SOCLE_LCD_INTR_EN			0x44
#define SOCLE_LCD_INTR_DIS			0x48
#define SOCLE_LCD_INTR_MASK		0x4C
#define SOCLE_LCD_Y_PAGE0_ADDR			0x50
#define SOCLE_LCD_Cb_PAGE0_ADDR			0x54
#define SOCLE_LCD_Cr_PAGE0_ADDR			0x58
#define SOCLE_LCD_Y_PAGE1_ADDR			0x60
#define SOCLE_LCD_Cb_PAGE1_ADDR			0x64
#define SOCLE_LCD_Cr_PAGE1_ADDR			0x68
#define SOCLE_LCD_INTR_VER			0xFC


//OFFSET 0x00 CTRL0
//OFFSET 0x00 CTRL0
#define SOCLE_LCD_CTRL0_PXCLK_POLAR		(1<<22)
#define SOCLE_LCD_CTRL0_COLOURSENSE		(1<<20)
#define SOCLE_LCD_CTRL0_COLOUR_RED		(1<<20)
#define SOCLE_LCD_CTRL0_COLOUR_GREEN		(2<<20)
#define SOCLE_LCD_CTRL0_COLOUR_BLUE		(3<<20)
#define SOCLE_LCD_CTRL0_LUMCONFIG			(1<<17)
#define SOCLE_LCD_CTRL0_SRAM				(1<<16)
#define SOCLE_LCD_CTRL0_PCLOCK				(1<<8)
#define SOCLE_LCD_CTRL0_COLOURDEP			(1<<7)
#define SOCLE_LCD_CTRL0_24BPP				(1<<7)

#define SOCLE_LCD_CTRL0_LUTEN				(1<<6)
#define SOCLE_LCD_CTRL0_PAGESWAP			(1<<5)
#define SOCLE_LCD_CTRL0_HSYNC				(1<<4)
#define SOCLE_LCD_CTRL0_VSYNC				(1<<3)
#define SOCLE_LCD_CTRL0_DTMG				(1<<2)
#define SOCLE_LCD_CTRL0_RGBHALT			(1<<1)
#define SOCLE_LCD_CTRL0_ENABLE			(1<<0)

//OFFSET 0x04 CTRL1 STN
//OFFSET 0x08 YUV format control
#define SOCLE_LCD_YUV2RGB_EN				(1<<0)
//yuv format support
#define SOCLE_LCD_YUV420					(0x0<<1)
#define SOCLE_LCD_YUV422					(0x1<<1)
//OFFSET 0x10 CTRL1 H_TIMING
#define SOCLE_LCD_HTIMING_FP				(1<<24)
#define SOCLE_LCD_HTIMING_BP				(1<<16)
#define SOCLE_LCD_HTIMING_HRESET			(1<<10)
#define SOCLE_LCD_HTIMING_HACTIVE			(1<<0)

//OFFSET 0x14 CTRL1 V_TIMING
#define SOCLE_LCD_VTIMING_FP				(1<<24)
#define SOCLE_LCD_VTIMING_BP				(1<<16)
#define SOCLE_LCD_VTIMING_VRESET			(1<<10)
#define SOCLE_LCD_VTIMING_VACTIVE			(1<<0)

//OFFSET 0x18 STN TIMING


//OFFSET 0x20 PAGE0

//OFFSET 0x24 PAGE1


//OFFSET 0x40 INTR STS
#define SOCLE_LCD_INTR_STS_PAGE0_READ	(1<<0)

//OFFSET 0x44 INTR EN
#define SOCLE_LCD_INTR_EN_PAGE0_READ		(1<<0)

#endif /* ___ASM_ARCH_REGS_LCD_H */



