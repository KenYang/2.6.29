/* arch/arm/mach-socle/include/mach/regs-pc9223-scu.h
 *
 * Copyright (c) 2006 Socle-tech Corp
 *		      http://www.socle-tech.com.tw/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __SOCLE_PC9223_SCU_H
#define __SOCLE_PC9223_SCU_H                     1

/*	SCU clock define	*/
#define SCU_CLOCK_REG(n,m,od)	((n<<15) + (m << 3) + od)

#define SCU_CPU_CLOCK_33        SCU_CLOCK_REG   (1, 21, 3)
#define SCU_CPU_CLOCK_66        SCU_CLOCK_REG   (1, 21, 1)
#define SCU_CPU_CLOCK_80        SCU_CLOCK_REG   (1, 39, 2)
#define SCU_CPU_CLOCK_100       SCU_CLOCK_REG   (1, 49, 2)
#define SCU_CPU_CLOCK_132       SCU_CLOCK_REG   (1, 21, 0)
#define SCU_CPU_CLOCK_133       SCU_CLOCK_REG   (5, 265, 3)
#define SCU_CPU_CLOCK_150       SCU_CLOCK_REG   (5, 74, 0)
#define SCU_CPU_CLOCK_166       SCU_CLOCK_REG   (5, 82, 0)
#define SCU_CPU_CLOCK_200       SCU_CLOCK_REG   (2, 49, 0)
#define SCU_CPU_CLOCK_240       SCU_CLOCK_REG   (1, 39, 0)
#define SCU_CPU_CLOCK_252       SCU_CLOCK_REG   (0, 20, 0)
#define SCU_CPU_CLOCK_264       SCU_CLOCK_REG   (1, 43, 0)
#define SCU_CPU_CLOCK_266       SCU_CLOCK_REG   (5, 265, 1)
#define SCU_CPU_CLOCK_280       SCU_CLOCK_REG   (2, 69, 0)
#define SCU_CPU_CLOCK_300       SCU_CLOCK_REG   (1, 49, 0)
#define SCU_CPU_CLOCK_320       SCU_CLOCK_REG   (2, 79, 0)
#define SCU_CPU_CLOCK_340       SCU_CLOCK_REG   (5, 169, 0)
#define SCU_CPU_CLOCK_350       SCU_CLOCK_REG   (11, 349, 0)
#define SCU_CPU_CLOCK_360       SCU_CLOCK_REG   (1, 59, 0)
#define SCU_CPU_CLOCK_380       SCU_CLOCK_REG   (2, 94, 0)
#define SCU_CPU_CLOCK_390       SCU_CLOCK_REG   (2, 96, 0)
//#define SCU_CPU_CLOCK_390       SCU_CLOCK_REG   (1, 64, 0)
#define SCU_CPU_CLOCK_400       SCU_CLOCK_REG   (2, 99, 0)
#define SCU_CPU_CLOCK_410       SCU_CLOCK_REG   (5, 204, 0)
#define SCU_CPU_CLOCK_420       SCU_CLOCK_REG   (1, 69, 0)
#define SCU_CPU_CLOCK_430       SCU_CLOCK_REG   (5, 214, 0)
#define SCU_CPU_CLOCK_440       SCU_CLOCK_REG   (5, 219, 0)
#define SCU_CPU_CLOCK_450       SCU_CLOCK_REG   (5, 225, 0)
#define SCU_CPU_CLOCK_460       SCU_CLOCK_REG   (5, 229, 0)
#define SCU_CPU_CLOCK_470       SCU_CLOCK_REG   (5, 234, 0)
#define SCU_CPU_CLOCK_480       SCU_CLOCK_REG   (1, 80, 0)
#define SCU_CPU_CLOCK_490       SCU_CLOCK_REG   (5, 244, 0)
#define SCU_CPU_CLOCK_500       SCU_CLOCK_REG   (5, 249, 0)
#define SCU_CPU_CLOCK_510       SCU_CLOCK_REG   (1, 84, 0)
#define SCU_CPU_CLOCK_520       SCU_CLOCK_REG   (2, 129, 0)
#define SCU_CPU_CLOCK_533       SCU_CLOCK_REG   (2, 132, 0)


#define SCU_UART_CLOCK_176	SCU_CLOCK_REG	(1, 31, 0)	


/*
 *  Register for SCU
 *  */
#define SOCLE_SCU_MPLLCON		0x00		/*	Main PLL control register	*/
#define SOCLE_SCU_UPLLCON		0x04		/*	UART PLL control register	*/
#define SOCLE_SCU_MCLKEN		0x10		/*	Main clock enable register	*/
#define SOCLE_SCU_ACLKEN		0x14		/*	Application clock enable 	*/
#define SOCLE_SCU_MCLKDIV		0x18		/*	Main clock divide register	*/
#define SOCLE_SCU_PWMCON		0x1C		/*	Power management  register	*/
#define SOCLE_SCU_SWRST		0x20		/*	Software reset register		*/
#define SOCLE_SCU_REMAP		0x24		/*	Decoder remap contrl register*/
#define SOCLE_SCU_DEVCON		0x28		/*	System device mode setting  register	*/
#define SOCLE_SCU_INFORM0		0x34		/*	User defined register		*/
#define SOCLE_SCU_INFORM1		0x38		/*	User defined register		*/
#define SOCLE_SCU_INFORM2		0x3C		/*	User defined register		*/
#define SOCLE_SCU_INFORM3		0x40		/*	User defined register		*/
#define SOCLE_SCU_CID			0x44		/*	Chip ID register		*/

/*	Main PLL control register	0x00 */
#define SCU_MPLLCON_PLL_RFEN		(1<<29)
#define SCU_MPLLCON_PLL_FBEN		(1<<28)
#define SCU_MPLLCON_PLL_RFSLIP		(1<<27)
#define SCU_MPLLCON_PLL_FBSLIP		(1<<26)
#define SCU_MPLLCON_PLL_ENSAT		(1<<25)
#define SCU_MPLLCON_PLL_FASTEN	(1<<24)
#define SCU_MPLLCON_PLL_BYPASS	(1<<23)
#define SCU_MPLLCON_PLL_RST		(1<<22)
#define SCU_MPLLCON_PLL_PWR_DN	(1<<21)
#define SCU_MPLLCON_PLL_MASK		0x1FFFFF
#define SCU_MPLLCON_N_MASK		(0x3F << 15)
#define SCU_MPLLCON_N				15
#define SCU_MPLLCON_M_MASK		(0xFFF << 3)
#define SCU_MPLLCON_M				3
#define SCU_MPLLCON_OD_MASK		0x3
#define SCU_MPLLCON_OD				0

/*	UART PLL control register	0x04 */

#define SCU_UPLLCON_PLL_RFEN		(1<<29)
#define SCU_UPLLCON_PLL_FBEN		(1<<28)
#define SCU_UPLLCON_PLL_RFSLIP		(1<<27)
#define SCU_UPLLCON_PLL_FBSLIP		(1<<26)
#define SCU_UPLLCON_PLL_ENSAT		(1<<25)
#define SCU_UPLLCON_PLL_FASTEN	(1<<24)
#define SCU_UPLLCON_PLL_BYPASS	(1<<23)
#define SCU_UPLLCON_PLL_RST		(1<<22)
#define SCU_UPLLCON_PLL_PWR_DN	(1<<21)
#define SCU_UPLLCON_PLL_MASK		0x1FFFFF
#define SCU_UPLLCON_N_MASK		(0x3F << 15)
#define SCU_UPLLCON_N				15
#define SCU_UPLLCON_M_MASK		(0xFFF << 3)
#define SCU_UPLLCON_M				3
#define SCU_UPLLCON_OD_MASK		0x3
#define SCU_UPLLCON_OD				0

/*	Main clock enable register	0x10 */
#define SCU_MCLK_ROM			(1<<29)
#define SCU_MCLK_DCM			(1<<27)
#define SCU_MCLK_STSDR			(1<<26)
#define SCU_MCLK_NFC			(1<<25)
#define SCU_MCLK_HDMA			(1<<24)
#define SCU_MCLK_VOP			(1<<23)
#define SCU_MCLK_VIP			(1<<22)
#define SCU_MCLK_OTG0			(1<<21)
#define SCU_MCLK_SDMMC			(1<<20)
#define SCU_MCLK_OTG1			(1<<19)
#define SCU_MCLK_MAC			(1<<18)
#define SCU_MCLK_CLCD			(1<<17)
#define SCU_MCLK_GPIO3			(1<<16)
#define SCU_MCLK_GPIO2			(1<<15)
#define SCU_MCLK_GPIO1			(1<<14)
#define SCU_MCLK_GPIO0			(1<<13)
#define SCU_MCLK_ADC			(1<<12)
#define SCU_MCLK_PWM			(1<<11)
#define SCU_MCLK_WDT			(1<<10)
#define SCU_MCLK_RTC			(1<<9)
#define SCU_MCLK_TIMER			(1<<8)
#define SCU_MCLK_I2S			(1<<6)
#define SCU_MCLK_I2C			(1<<5)
#define SCU_MCLK_SPI1			(1<<4)
#define SCU_MCLK_SPI0			(1<<3)
#define SCU_MCLK_UART2			(1<<2)
#define SCU_MCLK_UART1			(1<<1)
#define SCU_MCLK_UART0			(1<<0)

/*	Application clock enable 	0x14 */
#define SCU_ACLK_SDC_HARDMACRO	(1<<20)
#define SCU_ACLK_MAC_HARDMACRO	(1<<19)
#define SCU_ACLK_LCDC_HARDMACRO	(1<<18)
#define SCU_ACLK_NFC_HARDMACRO	(1<<17)
#define SCU_ACLK_OTG0_HARDMACRO	(1<<16)
#define SCU_ACLK_OTG1_HARDMACRO	(1<<15)
#define SCU_ACLK_ST_SDR		(1<<14)
#define SCU_ACLK_UART		(1<<13)
#define SCU_ACLK_VIP		(1<<12)
#define SCU_ACLK_LCD_VOP	(1<<11)
#define SCU_ACLK_MAC		(1<<10)
#define SCU_ACLK_VIP_HARDMACRO	(1<<9)
#define SCU_ACLK_VOP_HARDMACRO	(1<<8)
#define SCU_ACLK_UTMI_OTG1	(1<<7)
#define SCU_ACLK_UTMI_OTG0	(1<<6)
#define SCU_ACLK_ADC		(1<<4)
#define SCU_ACLK_I2S		(1<<3)
#define SCU_ACLK_UART2		(1<<2)
#define SCU_ACLK_UART1		(1<<1)
#define SCU_ACLK_UART0		(1<<0)

/*	Main clock divide register	0x0018 */
#define SCU_MCLKDIV_PLL_LOCK_PERIOD_M		(0x7FF <<17)		/*	MPLL/UPLL lock period mask		*/
#define SCU_MCLKDIV_PLL_LOCK_PERIOD_S		17				/*	MPLL/UPLL lock period shift		*/
#define SCU_MCLKDIV_ADC_CLK_DIV_M		(0xFF <<9)		/*	ADC clock divider mask		*/
#define SCU_MCLKDIV_ADC_CLK_DIV_S		9				/*	ADC clock divider shift		*/
#define SCU_MCLKDIV_UART2_CLK_M		(0x3 <<7)		/*	UART2 clock frequency setting mask		*/
#define SCU_MCLKDIV_UART2_CLK_S		7				/*	UART2 clock frequency setting shift		*/
#define SCU_MCLKDIV_UART2_CLK_24		(0x0)			/*	UART2 clock frequency setting. 00: 24MHz (MOSC_XIN)	*/
#define SCU_MCLKDIV_UART2_CLK_UPLL	(0x1 << 7)		/*	UART2 clock frequency setting. 01: UPLL frequency	*/
#define SCU_MCLKDIV_UART2_CLK_UPLL_2	(0x2 << 7)		/*	UART2 clock frequency setting. 02: UPLL/2 frequency	*/
#define SCU_MCLKDIV_UART2_CLK_UPLL_4	(0x3 << 7)		/*	UART2 clock frequency setting. 03: UPLL/4 frequency	*/
#define SCU_MCLKDIV_UART1_CLK_M		(0x3 << 5)		/*	UART1 clock frequency setting mask		*/
#define SCU_MCLKDIV_UART1_CLK_S		5				/*	UART1 clock frequency setting shift		*/
#define SCU_MCLKDIV_UART1_CLK_24		(0x0)			/*	UART1 clock frequency setting. 00: 24MHz (MOSC_XIN)	*/
#define SCU_MCLKDIV_UART1_CLK_UPLL	(0x1 << 5)		/*	UART1 clock frequency setting. 01: UPLL frequency	*/
#define SCU_MCLKDIV_UART1_CLK_UPLL_2	(0x2 << 5)		/*	UART1 clock frequency setting. 02: UPLL/2 frequency	*/
#define SCU_MCLKDIV_UART1_CLK_UPLL_4	(0x3 << 5)		/*	UART1 clock frequency setting. 03: UPLL/4 frequency	*/
#define SCU_MCLKDIV_UART0_CLK_M		(0x3 << 3)		/*	UART0 clock frequency setting mask		*/
#define SCU_MCLKDIV_UART0_CLK_S		3				/*	UART0 clock frequency setting shift		*/
#define SCU_MCLKDIV_UART0_CLK_24		(0x0)			/*	UART0 clock frequency setting. 00: 24MHz (MOSC_XIN)	*/
#define SCU_MCLKDIV_UART0_CLK_UPLL	(0x1 << 3)		/*	UART0 clock frequency setting. 01: UPLL frequency	*/
#define SCU_MCLKDIV_UART0_CLK_UPLL_2	(0x2 << 3)		/*	UART0 clock frequency setting. 02: UPLL/2 frequency	*/
#define SCU_MCLKDIV_UART0_CLK_UPLL_4	(0x3 << 3)		/*	UART0 clock frequency setting. 03: UPLL/4 frequency	*/
#define SCU_MCLKDIV_UART0_CLK_24		(0x0)			/*	UART0 clock frequency setting. 00: 24MHz (MOSC_XIN)	*/


#define SCU_MCLKDIV_UART_CLK_24		(0x0)			/*	UART0 clock frequency setting. 00: 24MHz (MOSC_XIN)	*/
#define SCU_MCLKDIV_UART_CLK_UPLL		(0x1)			/*	UART0 clock frequency setting. 01: UPLL frequency	*/
#define SCU_MCLKDIV_UART_CLK_UPLL_2	(0x2)			/*	UART0 clock frequency setting. 02: UPLL/2 frequency	*/
#define SCU_MCLKDIV_UART_CLK_UPLL_4	(0x3)			/*	UART0 clock frequency setting. 03: UPLL/4 frequency	*/

	/* CPU/AHB clock ratio	*/
#define SCU_MCLKDIV_CLK_RATIO_8_1			7
#define SCU_MCLKDIV_CLK_RATIO_4_1			3
#define SCU_MCLKDIV_CLK_RATIO_3_1			2
#define SCU_MCLKDIV_CLK_RATIO_2_1			1		
#define SCU_MCLKDIV_CLK_RATIO_1_1			0				
#define SCU_MCLKDIV_CLK_RATIO_MASK			7				
#define SCU_MCLKDIV_CLK_RATIO_S				0

/*	Power management  register	 0x001C	*/
#define SCU_PWMCON_SDR_WIDTH			(1<<10)	
#define SCU_PWMCON_SDR_WIDTH_32		(1<<10)	
#define SCU_PWMCON_SDR_WIDTH_16		0	
#define SCU_PWMCON_DCM_MODE			(1<<9)		
#define SCU_PWMCON_DCM_MODE_DCM		(1<<9)	
#define SCU_PWMCON_DCM_MODE_NOR		0		
#define SCU_PWMCON_TPS_MAC			(1<<7)		
#define SCU_PWMCON_RPS_MAC			(1<<6)		
#define SCU_PWMCON_BOOT_MODE				(0x3<<4)		
#define SCU_PWMCON_BOOT_MODE_NOR_16	(0x3<<4)		
#define SCU_PWMCON_BOOT_MODE_NOR_8		(0x2<<4)		
#define SCU_PWMCON_BOOT_MODE_NAND		(0x1<<4)		
#define SCU_PWMCON_BOOT_MODE_ISP		(0x0)		
#define SCU_PWMCON_STANDBYWFI		(1<<2)		
#define SCU_PWMCON_PWR_STOP			(1<<1)		
#define SCU_PWMCON_PWR_NOR			(1<<0)		

/*	System device mode setting  register	0x0028 */

#define SCU_DEVCON_LCD_CLK_MPLL_OUTPUT	(1<<31)
#define SCU_DEVCON_HDMA45_SPI1			(1<<30)
#define SCU_DEVCON_HDMA45_SPI0			0
#define UART0_WITH_HDMA				0
#define UART1_WITH_HDMA				1
#define UART2_WITH_HDMA				2
#define SCU_DEVCON_UART_HDMA01_M	(3<<26)
#define SCU_DEVCON_UART_HDMA01_S		26
#define SCU_DEVCON_UART_HDMA23_M	(3<<24)
#define SCU_DEVCON_UART_HDMA23_S		24
#define SCU_DEVCON_WDT_RST			(1<<23)
#define SCU_DEVCON_SW_RST				(1<<22)
#define SCU_DEVCON_NFC_GPIO			(1<<21)
#define SCU_DEVCON_MAC_GPIO			(1<<20)
#define SCU_DEVCON_TMR_GPIO			(1<<19)
#define SCU_DEVCON_PWM0_GPIO			(1<<18)
#define SCU_DEVCON_PWM1_GPIO			(1<<17)
#define SCU_DEVCON_FIQ_POLAR_HIGH	(1<<16)
#define SCU_DEVCON_INT0				(3<<14)
#define SCU_DEVCON_INT0_EXT_INT0		(1<<14)
#define SCU_DEVCON_INT0_NFIQ			(2<<14)
#define SCU_DEVCON_INT1				(1<<13)
#define SCU_DEVCON_LCD_GPIO			(3<<11)
#define SCU_DEVCON_LCD_GPIO_LCD		(1<<11)
#define SCU_DEVCON_LCD_GPIO_VOP		(2<<11)
#define SCU_DEVCON_VIP_GPIO			(1<<10)
#define SCU_DEVCON_SPI0_GPIO			(1<<9)
#define SCU_DEVCON_SPI1_GPIO			(1<<8)
#define SCU_DEVCON_UDC_UHC			(1<<7)
#define SCU_DEVCON_I2S_GPIO			(3<<6)
#define SCU_DEVCON_I2S_GPIO_TX		(1<<6)
#define SCU_DEVCON_I2S_GPIO_RX		(2<<6)
#define SCU_DEVCON_I2S_GPIO_TX_RX		(3<<6)
#define SCU_DEVCON_I2C_GPIO			(1<<5)
#define SCU_DEVCON_SDMMC_GPIO		(1<<4)
#define SCU_DEVCON_UART0_GPIO			(1<<3)
#define SCU_DEVCON_UART1_GPIO			(1<<2)
#define SCU_DEVCON_UART2_GPIO			(1<<1)
#define SCU_DEVCON_APB_RESP			(1<<0)

#endif

