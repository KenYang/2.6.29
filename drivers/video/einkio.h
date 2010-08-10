/*
 * This file is EPD Driver .
 *
 * Copyright (c) 2007 PRIME VIEW INTERNATIONAL CO., LTD.  All rights reserved.
 */

#ifndef __EINKIO_H_
#define __EINKIO_H_

#ifdef CONFIG_ARCH_PDK_PC9220
#include <asm/arch/gpio.h>

#define EINK_SET_WUP()				socle_gpio_set_value_with_mask(PN,(1<<4),(1<<4)) 
#define EINK_CLR_WUP()				socle_gpio_set_value_with_mask(PN,0,(1<<4))

#define EINK_SET_CD_CMD()			socle_gpio_set_value_with_mask(PN,(1<<5),(1<<5)) 
#define EINK_SET_CD_DAT()			socle_gpio_set_value_with_mask(PN,0,(1<<5))

#define EINK_SET_RW_RD()			socle_gpio_set_value_with_mask(PN,(1<<7),(1<<7))
#define EINK_SET_RW_WR()			socle_gpio_set_value_with_mask(PN,0,(1<<7))

#define EINK_DATA_BASE				IO_ADDRESS(SOCLE_STMEM_BANK1)


#else
#include <asm/arch/regs-mp-gpio.h>
#include <asm/arch/mp-gpio.h>

#define EINK_SET_WUP()	socle_mp_gpio_set_port_num_value(MP_PO, 7, 1)
#define EINK_CLR_WUP()	socle_mp_gpio_set_port_num_value(MP_PO, 7, 0)

#define EINK_SET_CD_CMD()	socle_mp_gpio_set_port_num_value(MP_PP, 1, 1)
#define EINK_SET_CD_DAT()	socle_mp_gpio_set_port_num_value(MP_PP, 1, 0)

#define EINK_SET_DS()	socle_mp_gpio_set_port_num_value(MP_PP, 3, 0)
#define EINK_CLR_DS()	socle_mp_gpio_set_port_num_value(MP_PP, 3, 1)

#define EINK_SET_ACK_INPUT()		socle_mp_gpio_set_port_num_direction(MP_PP, 5, 0)
#define EINK_GET_ACK()		(socle_mp_gpio_get_port_value(MP_PP) & 0x20)

#define EINK_SET_RW_WR()	socle_mp_gpio_set_port_num_value(MP_PP, 7, 0)
#define EINK_SET_RW_RD()	socle_mp_gpio_set_port_num_value(MP_PP, 7, 1)

#define EINK_DATA_BASE		IO_ADDRESS(SOCLE_NOR_FLASH0)


#endif 


#ifdef Eink_Dbg
#undef	Eink_Dbg
#endif

//#define Eink_Dbg

#ifdef Eink_Dbg
#define EPRINTF(a,b...) printk("%s() " a, __FUNCTION__ , ## b)
#else
#define EPRINTF(a,b...)
#endif 

#define BYTE unsigned char
#define bool unsigned char

#define TimeOutValue    	0x1000000
#define TAckTimeOutValue 	100

#define H_D0		S3C2410_GPC0
#define H_D1		S3C2410_GPC1
#define H_D2		S3C2410_GPC2
#define H_D3		S3C2410_GPC3
#define H_D4		S3C2410_GPC4
#define H_D5		S3C2410_GPC5
#define H_D6		S3C2410_GPC6
#define H_D7		S3C2410_GPC7
#define H_WUP 	S3C2410_GPC8
#define H_DS 		S3C2410_GPC9
#define H_CD 		S3C2410_GPC10
#define H_RW 		S3C2410_GPC11
#define H_ACK 	S3C2410_GPC12
#define H_NRST 	S3C2410_GPC13
#define H_PWR 	S3C2410_GPC14

#define  dc_LoadImage         0xA0
#define  dc_StopNewImage      0xA1
#define  dc_DisplayImage      0xA2
#define  dc_PartialImage      0xB0
#define  dc_DisplayPartial    0xB1
#define  dc_Reset             0xEE
#define  dc_SetDepth          0xF3
#define  dc_EraseDisplay      0xA3
#define  dc_Rotate            0xF5
#define  dc_Positive          0xF7
#define  dc_Negative          0xF8
#define  dc_GoToNormal        0xF0
#define  dc_GoToSleep         0xF1
#define  dc_GoToStandBy       0xF2
#define  dc_WriteToFlash      0x01
#define  dc_ReadFromFlash     0x02
#define  dc_Init              0xA4
#define  dc_AutoRefreshOn     0xF9
#define  dc_AutoRefreshOff    0xFA
#define  dc_SetRefresh        0xFB
#define  dc_ForcedRefresh     0xFC
#define  dc_GetRefresh        0xFD
#define  dc_RestoreImage      0xA5
#define  dc_ControllerVersion 0xE0
#define  dc_SoftwareVersion   0xE1
#define  dc_DisplaySize       0xE2
#define  dc_GetStatus         0xAA
#define  dc_Temperature       0x21
#define  dc_WriteRegister     0x10
#define  dc_ReadRegister      0x11
#define  dc_Abort             0xA1

#define  dc_EinkShow          0xC0
#define  dc_EraseEink         0xC1
#define  dc_EinkRotate        0xC2
#define  dc_EinkSet   				0xC3
#define  dc_EinkInit          0xC4

#define EINK_Send_Data 0
#define EINK_Send_Command 1

#define Dat_Read     0
#define Cmd_Write    1
#define Dat_Write    2

struct My_Image{
	unsigned char   Command;
	unsigned char   *EinkBase;
	unsigned short  Cmd_Dat;
	unsigned short 	Xpos;
	unsigned short 	Ypos;
	unsigned short 	Height;
	unsigned short 	Width;
	unsigned int    BytesToSend;
	unsigned char 	flags;
};

typedef struct {
   	unsigned int port;
   	unsigned char pullup;
   	unsigned char mode;
} EINK_IO_INFO;

#endif	// __EINKIO_H__
