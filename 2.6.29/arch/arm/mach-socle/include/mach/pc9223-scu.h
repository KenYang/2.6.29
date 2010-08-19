/********************************************************************************
* File Name     : arch/arm/mach-socle/include/mach/pc9223-scu.h
* Author        : Leonid Cheng
* Description   : Socle PC9223 SCU Service Header
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
*      1. 2009/08/04 leonid cheng create this file
*
********************************************************************************/

#ifndef __SOCLE_PC9223_SCU_H_INCLUDED
#define __SOCLE_PC9223_SCU_H_INCLUDED


#if 0
#define MPLL_XIN                                        12288000                //(Hz)
#else
#define MPLL_XIN                                        12000000                //(Hz)
#endif

#define UPLL_XIN                                        11059200                //(Hz)





	/* CPU clock */
#define SOCLE_SCU_CPU_CLOCK_33		0
#define SOCLE_SCU_CPU_CLOCK_66		1
#define SOCLE_SCU_CPU_CLOCK_80		2
#define SOCLE_SCU_CPU_CLOCK_100		3
#define SOCLE_SCU_CPU_CLOCK_132		4
#define SOCLE_SCU_CPU_CLOCK_133		5
#define SOCLE_SCU_CPU_CLOCK_166		6
#define SOCLE_SCU_CPU_CLOCK_200		7
#define SOCLE_SCU_CPU_CLOCK_240		8
#define SOCLE_SCU_CPU_CLOCK_252 	9
#define SOCLE_SCU_CPU_CLOCK_258		10
#define SOCLE_SCU_CPU_CLOCK_264		11	
#define SOCLE_SCU_CPU_CLOCK_266		12
#define SOCLE_SCU_CPU_CLOCK_280		13
#define SOCLE_SCU_CPU_CLOCK_300		14
#define SOCLE_SCU_CPU_CLOCK_320		15
#define SOCLE_SCU_CPU_CLOCK_340		16
#define SOCLE_SCU_CPU_CLOCK_350		17
#define SOCLE_SCU_CPU_CLOCK_360		18
#define SOCLE_SCU_CPU_CLOCK_380		19
#define SOCLE_SCU_CPU_CLOCK_390		20
#define SOCLE_SCU_CPU_CLOCK_400		21
#define SOCLE_SCU_CPU_CLOCK_410		22
#define SOCLE_SCU_CPU_CLOCK_420		23
#define SOCLE_SCU_CPU_CLOCK_430		24
#define SOCLE_SCU_CPU_CLOCK_440		25
#define SOCLE_SCU_CPU_CLOCK_450		26
#define SOCLE_SCU_CPU_CLOCK_460 	27
#define SOCLE_SCU_CPU_CLOCK_470 	28
#define SOCLE_SCU_CPU_CLOCK_480 	29
#define SOCLE_SCU_CPU_CLOCK_490 	30
#define SOCLE_SCU_CPU_CLOCK_500 	31
#define SOCLE_SCU_CPU_CLOCK_510 	32
#define SOCLE_SCU_CPU_CLOCK_520 	33
#define SOCLE_SCU_CPU_CLOCK_533 	34
#define SOCLE_SCU_CPU_CLOCK_150 	35

	/* UART clock */
#define SOCLE_SCU_UART_CLOCK_176	0

	/* CPU/AHB clock ratio	*/
#define SOCLE_SCU_CLOCK_RATIO_1_1	0
#define SOCLE_SCU_CLOCK_RATIO_2_1	1
#define SOCLE_SCU_CLOCK_RATIO_3_1	2
#define SOCLE_SCU_CLOCK_RATIO_4_1	3
#define SOCLE_SCU_CLOCK_RATIO_8_1	4

	/*	hclk enable 	*/
#define SOCLE_SCU_HCLK_CLCD			0
#define SOCLE_SCU_HCLK_MAC			1
#define SOCLE_SCU_HCLK_OTG1			2
#define SOCLE_SCU_HCLK_OTG0			3
#define SOCLE_SCU_HCLK_VIP			4
#define SOCLE_SCU_HCLK_VOP			5
#define SOCLE_SCU_HCLK_HDMA			6
#define SOCLE_SCU_HCLK_NFC			7
#define SOCLE_SCU_HCLK_STSDR		8
#define SOCLE_SCU_HCLK_DCM			9
#define SOCLE_SCU_HCLK_ROM			10
#define SOCLE_SCU_HCLK_NUM		11

	/*	pclk enable 	*/
#define SOCLE_SCU_PCLK_UART0		0
#define SOCLE_SCU_PCLK_UART1		1
#define SOCLE_SCU_PCLK_UART2		2
#define SOCLE_SCU_PCLK_SPI0			3
#define SOCLE_SCU_PCLK_SPI1			4
#define SOCLE_SCU_PCLK_I2C			5
#define SOCLE_SCU_PCLK_I2S			6
#define SOCLE_SCU_PCLK_SDMMC		7
#define SOCLE_SCU_PCLK_TIMER		8
#define SOCLE_SCU_PCLK_RTC			9
#define SOCLE_SCU_PCLK_WDT			10
#define SOCLE_SCU_PCLK_PWM			11
#define SOCLE_SCU_PCLK_ADC			12
#define SOCLE_SCU_PCLK_GPIO0		13
#define SOCLE_SCU_PCLK_GPIO1		14
#define SOCLE_SCU_PCLK_GPIO2		15
#define SOCLE_SCU_PCLK_GPIO3		16
#define SOCLE_SCU_PCLK_NUM		17

	/*	aclk enable 	*/
#define SOCLE_SCU_ACLK_UCLK_FOR_UART0				0
#define SOCLE_SCU_ACLK_UCLK_FOR_UART1				1
#define SOCLE_SCU_ACLK_UCLK_FOR_UART2				2
#define SOCLE_SCU_ACLK_I2SCLK_FOR_I2S				3
#define SOCLE_SCU_ACLK_ADCCLK_FOR_ADC				4
#define SOCLE_SCU_ACLK_UTMICLK_FOR_OTG0				5
#define SOCLE_SCU_ACLK_UTMICLK_FOR_OTG1				6
#define SOCLE_SCU_ACLK_RMIICLK_FOR_MAC				7
#define SOCLE_SCU_ACLK_LCDCLK_VOPPCLK_FOR_LCDC_VOP	8
#define SOCLE_SCU_ACLK_VIP							9
#define SOCLE_SCU_ACLK_UART							10
#define SOCLE_SCU_ACLK_SD_CLKOUT_FOR_STSDR			11
#define SOCLE_SCU_ACLK_NUM			12




#define SOCLE_SCU_UART_CLK_24			0
#define SOCLE_SCU_UART_CLK_UPLL		1
#define SOCLE_SCU_UART_CLK_UPLL_2	 	2
#define SOCLE_SCU_UART_CLK_UPLL_4	 	3



	/* PLL lock period	*/
extern void socle_scu_pll_lock_period_set (int period);		//period minimum value is 2
extern int socle_scu_pll_lock_period_get (void);
extern void socle_scu_adc_clk_div_set (int div);		//div minimum value is 2
extern int socle_scu_adc_clk_div_get (void);
extern int socle_scu_uart_clk_24_set (int uart);
extern int socle_scu_uart_clk_upll_set (int uart);
extern int socle_scu_uart_clk_upll_2_set (int uart);
extern int socle_scu_uart_clk_upll_4_set (int uart);
extern int socle_scu_uart_clk_get (int uart);


	/*	SDRAM data bus width status	*/
#define SOCLE_SCU_SDRAM_BUS_WIDTH_32	 1
#define SOCLE_SCU_SDRAM_BUS_WIDTH_16	 0
	/*	DCM mode setting status	*/
#define SOCLE_SCU_DCM_MODE_DCM			 1
#define SOCLE_SCU_DCM_MODE_NOR			 0
	/*	USB mode setting status	*/
#define SOCLE_SCU_USB_MODE_UDC			 1
#define SOCLE_SCU_USB_MODE_UHC			 0

	/*	Boot source selection status	*/
#define SOCLE_SCU_BOOT_NOR_16	 	3
#define SOCLE_SCU_BOOT_NOR_8	 	2
#define SOCLE_SCU_BOOT_NAND	 	1
#define SOCLE_SCU_BOOT_ISP_ROM	0

extern int socle_scu_sdram_bus_width_status (void);
extern int socle_scu_dcm_mode_status (void);
extern int socle_scu_usb_mode_status (void);
extern int socle_scu_boot_source_status (void);
extern int socle_scu_tps_mac_status (void);
extern int socle_scu_tps_mac_status (void);
extern int socle_scu_auto_boot_fail_status (void);
extern void socle_scu_pw_standbywfi_enable (int i);
extern void socle_scu_stop_mode_enable (int i);
extern void socle_scu_slow_mode_disable (int i);

#define SOCLE_DEVCON_NFC				0
#define SOCLE_DEVCON_MAC				1
#define SOCLE_DEVCON_TMR				2
#define SOCLE_DEVCON_PWM0				3
#define SOCLE_DEVCON_PWM1				4
#define SOCLE_DEVCON_EXT_INT0			5
#define SOCLE_DEVCON_EXT_INT0_NFIQ	6
#define SOCLE_DEVCON_EXT_INT1			7
#define SOCLE_DEVCON_LCDC				8
#define SOCLE_DEVCON_LCDC_VOP			9
#define SOCLE_DEVCON_SPI0				10
#define SOCLE_DEVCON_SPI1				11
#define SOCLE_DEVCON_I2S_TX			12
#define SOCLE_DEVCON_I2S_RX			13
#define SOCLE_DEVCON_I2S_TX_RX		14
#define SOCLE_DEVCON_I2C				15
#define SOCLE_DEVCON_SDMMC			16
#define SOCLE_DEVCON_UART0			17
#define SOCLE_DEVCON_UART1			18
#define SOCLE_DEVCON_UART2			19
#define SOCLE_DEVCON_VIP				24



extern int socle_scu_dev_enable(u32 dev);
extern int socle_scu_dev_disable(u32 dev);

extern void socle_scu_lcdc_clk_input_mpll_outpput(void);
extern void socle_scu_lcdc_clk_input_mpll_xin(void);
extern void socle_scu_hdma_req45_spi0(void);
extern void socle_scu_hdma_req45_spi1(void);
extern void socle_scu_uhc0_48clock_input_upll(void);
extern void socle_scu_uhc0_48clock_input_otg_phy(void);
extern void socle_scu_uhc1_48clock_input_upll(void);
extern void socle_scu_uhc1_48clock_input_otg_phy(void);
extern int socle_scu_hdma_req01_uart(int uart);
extern int socle_scu_hdma_req23_uart(int uart);
extern void socle_scu_wdt_reset_enable(int en);
extern void socle_scu_sw_reset_enable(int en);
extern void socle_scu_nfiq_polarity_high(int en);



#define SOCLE_SCU_SDC_HARDMACRO		0
#define SOCLE_SCU_MAC_HARDMACRO		1
#define SOCLE_SCU_LCDC_HARDMACRO	2
#define SOCLE_SCU_OTG0_HARDMACRO	3
#define SOCLE_SCU_OTG1_HARDMACRO	4
#define SOCLE_SCU_NFC_HARDMACRO		5
#define SOCLE_SCU_VIP_HARDMACRO		6
#define SOCLE_SCU_VOP_HARDMACRO		7

extern int socle_scu_hardmacro_chip_stop_mode_disable (int chip);
extern int socle_scu_hardmacro_chip_stop_mode_no_pwr_control (int chip);

#endif



