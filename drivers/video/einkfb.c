/*
 *  linux/drivers/video/einkfb.c -- Virtual frame buffer device
 *
 *      Copyright (C) 2002 James Simmons
 *
 *	Copyright (C) 1997 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 * ChangeLog:
 * 2008-03-26: CY Li Porting to Socle Platform
 *
 */
//#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <linux/fb.h>
#include <linux/init.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "einkio.h"
  
#define NO_ARG 0xFFFF
#define VIDEOSIZE 256*1024
//static void *VideoMemory = NULL;
static u_long videomemorysize = VIDEOSIZE;
static BYTE *I_Send_Data = NULL;

struct fb_var_screeninfo einkfb_default = {
	.xres =		600,
	.yres =		800,
	.xres_virtual =	600,
	.yres_virtual =	800,
	.xoffset = 0,
	.yoffset = 0,
	.bits_per_pixel = 4,
	.grayscale = 1,
	.red 		= {0,2,0},		
	.green 	= {0,2,0},	
	.blue 	= {0,2,0},
	.transp = {0,0,0},
	.nonstd = 0,
	.activate =	~FB_ACTIVATE_MASK & FB_ACTIVATE_NOW,
	.height = 800,
	.width = 600,
	.accel_flags = 0,
	//.sync = FB_SYNC_ON_GREEN,
	.vmode = ~FB_VMODE_MASK & FB_VMODE_NONINTERLACED,
};

struct fb_fix_screeninfo einkfb_fix = {
	.id =		"Eink FB",
	.smem_len = 256*1024,
	.type =		FB_TYPE_PLANES,
	.type_aux =0,
	.visual =	FB_VISUAL_DIRECTCOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.line_length = 300,
	.mmio_start = 0,
	.mmio_len = 0,
	.accel =	FB_ACCEL_NONE,
};

static int einkfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info);
static int einkfb_ioctl( struct fb_info *info, unsigned int cmd, unsigned long arg );
static int einkfb_open( struct fb_info *info, int user );
static int einkfb_release( struct fb_info *info, int user );

static struct fb_ops einkfb_ops = {
	.owner		  = THIS_MODULE,
	.fb_setcolreg	= einkfb_setcolreg,
	.fb_ioctl	  = einkfb_ioctl,
	.fb_open   	= einkfb_open,
  .fb_release = einkfb_release,
};

#if 0
static EINK_IO_INFO eink_CtrIO[] = {
    { H_PWR	,GPIO_PULLUP_EN  ,GPIO_MODE_OUT},
    { H_WUP	,GPIO_PULLUP_EN  ,GPIO_MODE_OUT},
    { H_DS	,GPIO_PULLUP_EN  ,GPIO_MODE_OUT},
    { H_CD	,GPIO_PULLUP_EN  ,GPIO_MODE_OUT},
    { H_RW	,GPIO_PULLUP_EN  ,GPIO_MODE_OUT},
    { H_NRST,GPIO_PULLUP_EN  ,GPIO_MODE_OUT},
    { H_ACK	,GPIO_PULLUP_DIS ,GPIO_MODE_IN },
};

static EINK_IO_INFO eink_IoData[] = {
    { H_D0, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
    { H_D1, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
    { H_D2, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
    { H_D3, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
    { H_D4, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
    { H_D5, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
    { H_D6, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
    { H_D7, GPIO_PULLUP_DIS ,GPIO_MODE_IN },
};
#endif


static inline void write_data_byte(unsigned char byteData)
{
#if 0
	unsigned char i;
	for(i=0;i<8;i++)
		write_gpio_bit(eink_IoData[i].port,((byteData>>i)&0x01));
#else
//	while (EINK_GET_ACK()) {
//		printk("wait for ack!\n");
//	}
	iowrite8(byteData, EINK_DATA_BASE);
//	while (EINK_GET_ACK()) {
//		printk("wait for ack!\n");
//	}
#endif
}

static inline unsigned char read_data_byte(void)
{
#if 0
	unsigned char i,j;
	unsigned char ByteData =0;
	for(i=0;i<8;i++) {
		j = read_gpio_bit(eink_IoData[i].port);   	
		if(j)
			ByteData |= ( 0x01<<i );
	}
	return ByteData;
#else
	return (ioread8(EINK_DATA_BASE));
#endif
}

#if 0
static void Set_Eink_DataBus(unsigned char mode)
{
	set_gpio_ctrl(H_D0,GPIO_PULLUP_DIS ,mode);
	set_gpio_ctrl(H_D1,GPIO_PULLUP_DIS ,mode);
	set_gpio_ctrl(H_D2,GPIO_PULLUP_DIS ,mode);
	set_gpio_ctrl(H_D3,GPIO_PULLUP_DIS ,mode);
	set_gpio_ctrl(H_D4,GPIO_PULLUP_DIS ,mode);
	set_gpio_ctrl(H_D5,GPIO_PULLUP_DIS ,mode);
	set_gpio_ctrl(H_D6,GPIO_PULLUP_DIS ,mode);
	set_gpio_ctrl(H_D7,GPIO_PULLUP_DIS ,mode);
}
#endif

static int EINK_Write(unsigned char Data ,unsigned char cmd)
{
#if 0
		int Check =0;
   	int  TimeOut =0; 
   	unsigned int iACK =0;  
   	EPRINTF("__EINK_Write %X \n",Data);
   	write_gpio_bit(H_RW,0);  //RW=0 for Write	
		write_gpio_bit(H_CD,cmd);	//CD=0 for data CD=1 for command
		// Set data port as output
		Set_Eink_DataBus(GPIO_MODE_OUT);
		//Write Data
		write_data_byte(Data);	
		write_gpio_bit(H_DS,0);	//DS=0, controller makes H_ACK active ��Low��.
		TimeOut=0;
		//waits until H_ACK is ��Low�� for tACK time
		do{
				iACK = read_gpio_bit(H_ACK);
				//EPRINTF("__ACK value %d \n",iACK);
				TimeOut++;
   	}while (iACK && (TimeOut < TimeOutValue));
    EPRINTF("__ACK value_1 %d \n",iACK);
		EPRINTF("__Transfer timeout value %d (max:167772116) \n",TimeOut);
		udelay(2);
		write_gpio_bit(H_DS,1);	//DS=1
		TimeOut =0;
		//waits until H_ACK is ��HIGH��.
		do{
			iACK = read_gpio_bit(H_ACK);
			TimeOut++;
   	}while (!iACK && (TimeOut < TAckTimeOutValue));
   	EPRINTF("__ACK value_2 %d \n",iACK);
    Check = (TimeOut < TimeOutValue);
    EPRINTF("__Data ready timeout value %d (max:100) \n",TimeOut);
   	if(!Check)
   	{
   			//if timeOut then reset eink
   			EPRINTF("__EINK_reset\n");
				write_gpio_bit(H_NRST, 0);
				udelay(5);
				write_gpio_bit(H_NRST, 1);
   	}
   	return Check;
#else
	EINK_SET_RW_WR();

	if (EINK_Send_Command == cmd) {
		EPRINTF("__EINK_Write cmd %X\n", Data);
		EINK_SET_CD_CMD();
	} else if (EINK_Send_Data == cmd) {
		EPRINTF("__EINK_Write data %X\n", Data);
		EINK_SET_CD_DAT();
	} else {
		EPRINTF("Invalidate command\n");
		return -1;
	}

	//Write Data
	write_data_byte(Data);

	return 1;
#endif
}

static int WriteDataBuf(BYTE *Data,unsigned long len)
{
#if 0
	 	int Check = 0;
   	int  TimeOut = 0; 
   	unsigned int iACK =0;
   	int i;
   	BYTE TmpData = 0;
   	EPRINTF("\n");
   	
   	write_gpio_bit(H_RW,0);  	//RW=0 for Write	
		write_gpio_bit(H_CD,0);		//CD=0 for data
		// Set data port as output
		Set_Eink_DataBus(GPIO_MODE_OUT);
		for(i=0 ; i<len ; i++)
		{			  
				TmpData = *(Data+i);			
				//Write Data
				write_data_byte(TmpData);	
				write_gpio_bit(H_DS,0);		//DS=0, for write
				//waits until H_ACK is ��Low�� for tACK time
				TimeOut =0;
				do{
						iACK = read_gpio_bit(H_ACK);
						TimeOut++;
   			}while (iACK && (TimeOut < TimeOutValue));
    		//Check = (TimeOut < TAckTimeOutValue);
				//EPRINTF("EINK_Write TAck timeout value: %d (max:100)\n",TimeOut);
				udelay(2);
		    write_gpio_bit(H_DS,1);	//DS=1
				TimeOut =0;
				//waits until H_ACK is ��Low��.
				do{
						iACK = read_gpio_bit(H_ACK);
						TimeOut++;
   			}while (!iACK && (TimeOut < TAckTimeOutValue));
    		Check = (TimeOut < TimeOutValue);
    		/*if(i<100)
    			printk("(%d) <%X> ",i,TmpData);
    		if( i<100 && !(i%10) )
    			printk("\n");
    		if(i>59950 && i<60050)
    			printk("(%d) <%X> ",i,TmpData);
    		if(i>59950 && i<60050 && !(i%10))
    			printk("\n");
    		if(i>119900)
    			printk("(%d) <%X> ",i,TmpData);
    		if( i>119900 && !(i%10) )
    			printk("\n");*/
    		//EPRINTF("(%d) <%X> ",i,TmpData);
   			if(!Check)
   			{
   					//if timeOut then reset eink
						write_gpio_bit(H_NRST, 0);
						udelay(5);
						write_gpio_bit(H_NRST, 1);
						EPRINTF("__EINK_Write Reset \n");
						return Check;
   			}
   	}
   	return Check;
#else
	int i;
	EPRINTF("\n");

	EINK_SET_RW_WR();
	EINK_SET_CD_DAT();

#if 0
{
	int i;
	for (i = 0; i < 10; i++)
		printk("%x ", Data[i]);
	printk("\n");
}
#endif
	printk("Data addr=%x\n", (int)Data);
mdelay(5000);
	for (i = 0; i < len; i++) {
//		udelay(50);
		write_data_byte(Data[i]);
	}

	return 1;
#endif
}

// ***********************************************************/

static int ReadData(unsigned char *Data )
{
#if 0
		int Check = 0;
   	int  TimeOut = 0; 
   	unsigned int iACK =0;
   	 
   	//Set data port as input
		Set_Eink_DataBus(GPIO_MODE_IN);
		write_gpio_bit(H_CD,0);	//CD=0 for data
   	write_gpio_bit(H_RW,1);  //RW=1 for Read	
		write_gpio_bit(H_DS,0);	//DS=0, controller makes H_ACK active ��High��.
		TimeOut =0;
		//waits until H_ACK is ��Low�� for tACK time
		do{
			iACK = read_gpio_bit(H_ACK);
			TimeOut++;
   	}while (iACK && (TimeOut < TimeOutValue));
    //Check = (TimeOut < TAckTimeOutValue);
		EPRINTF("__Transfer timeout value %d (max:167772116) \n",TimeOut);		
		Check = (TimeOut < TimeOutValue);
		
		if(Check)
   	{ 
   		//Read Data
   		 *Data= read_data_byte();
   		 udelay(2);   			
   		 EPRINTF("__EINK_Read Data value %X \n",*Data);   			
   		 write_gpio_bit(H_DS,1);	//DS=1
   			
		   TimeOut =0;
		  //waits until H_ACK is ��High��.
		   do{
		  	iACK = read_gpio_bit(H_ACK);
		  	TimeOut++;
   	     }while (!iACK && (TimeOut < TAckTimeOutValue));
      EPRINTF("__Data ready timeout value %d (max:100) \n",TimeOut);  			
   	}else{
   				//if timeOut then reset eink
				write_gpio_bit(H_NRST, 0);
				udelay(1);
				write_gpio_bit(H_NRST, 1);
				EPRINTF("__EINK_Read Reset \n");
   	}
   	return Check;
#else
	EPRINTF("\n");

//	EINK_SET_CD_DAT();
	EINK_SET_RW_RD();

	*Data= read_data_byte();

	return 1;
#endif
}
  
  
static void einkfb_wakeup(void)
{
#if 0
  	int Check = 0;
  	int  TimeOut = 0;   
  	unsigned int iACK;
   
    write_gpio_bit(H_WUP, 1);//wup =1
  	write_gpio_bit(H_DS, 0);//ds = 0 	
    udelay(1000);
  	//waits until H_ACK is ��Low�� for tACK time
		do{
			iACK = read_gpio_bit(H_ACK);
			TimeOut++;
   	}while (iACK && (TimeOut < TimeOutValue));
    //Check = (TimeOut < TAckTimeOutValue);
		udelay(2);
		EPRINTF("__TAck timeout value: %d %d (max:167772116)\n",iACK,TimeOut);
		write_gpio_bit(H_WUP, 0);//wup =1
		udelay(1000);
		write_gpio_bit(H_DS, 1);//ds = 0
		/*TimeOut =0;
		//waits until H_ACK is ��Low��.
		do{
			iACK = read_gpio_bit(H_ACK);
			TimeOut++;
   	}while (iACK==0 && (TimeOut < TimeOutValue));
    Check = (TimeOut < TimeOutValue);
    EPRINTF("EINK_Wake UP Transfer timeout value %d (max:167772116) \n",TimeOut);*/
    Check = (TimeOut < TimeOutValue);
		if(!Check)
   	{
   			//if timeOut then reset eink
				write_gpio_bit(H_NRST, 0);
				udelay(500);
				write_gpio_bit(H_NRST, 1);
				EPRINTF("__EINK_Wake Reset \n");
   	}
  	//write_gpio_bit(H_WUP, 0);//wup =1
  	//write_gpio_bit(H_DS, 1);//ds = 0
#else
	EPRINTF("\n");

	EINK_SET_WUP();

	//only for DS = 0
	write_data_byte(0x0);

	EINK_CLR_WUP();
#endif
}

static int ImageHandler ( struct My_Image *ImageAttr, BYTE *Image_Data, bool Partial)
{	
	int DataCot =0;
	u8  BPP =2;
	int BytesSend =0;
	u8  BitCount = 0;
	int Xp = 0;
	int Yp = 0;
	u8  Data = 0;

	EPRINTF("\n");

	EPRINTF("__Image_Data addr: %x\n", (int)Image_Data);
	BytesSend = ((((ImageAttr->Width * ImageAttr->Height) - 1) / (8 / BPP)) + 1);
	EPRINTF("__BytesSend: %d BytesToSend %d\n", BytesSend, ImageAttr->BytesToSend);                        
	if (Partial) {
		*(I_Send_Data+0) = ((ImageAttr->Xpos) >> 8)& 0xFF;
		*(I_Send_Data+1) = (ImageAttr->Xpos)& 0xFF;
		*(I_Send_Data+2) = ((ImageAttr->Ypos) >> 8)& 0xFF;
		*(I_Send_Data+3) = (ImageAttr->Ypos)& 0xFF;
		*(I_Send_Data+4) = ((ImageAttr->Xpos + ImageAttr->Width - 1) >> 8)& 0xFF;
		*(I_Send_Data+5) = (ImageAttr->Xpos + ImageAttr->Width - 1)& 0xFF;
		*(I_Send_Data+6) = ((ImageAttr->Ypos + ImageAttr->Height - 1) >> 8)& 0xFF;
		*(I_Send_Data+7) = (ImageAttr->Ypos + ImageAttr->Height - 1)& 0xFF;
	}
	EPRINTF("__X:%d,Y:%d,W:%d,H:%d\n",ImageAttr->Xpos,ImageAttr->Ypos,
					ImageAttr->Width,ImageAttr->Height);
	EPRINTF("__X1:%d,X2:%d,Y1:%d,Y2:%d \n",(*(I_Send_Data+0)<<8) | *(I_Send_Data+1),
					(*(I_Send_Data+4)<<8) | *(I_Send_Data+5),(*(I_Send_Data+2)<<8) | *(I_Send_Data+3),
					(*(I_Send_Data+6)<<8) | *(I_Send_Data+7));
printk("cyli fix format\n\n");
	while (BytesSend > 0) {
		Data = (Data << BPP);
       	if ((Xp < ImageAttr->Width) && (Yp < ImageAttr->Height)) {
			DataCot = (((ImageAttr->Ypos+Yp)*(einkfb_default.width)) + ImageAttr->Xpos + Xp) / 2;
			if (((ImageAttr->Xpos + Xp) % 2) == 0)
				//Data |= ((*(Image_Data+DataCot) & 0x0F )>>2);
				Data |= ((*(Image_Data+DataCot) & 0x03 ));
			else
				//Data |= (((*(Image_Data+DataCot) & 0xF0 ) >> 4 )>>2);
				Data |= (((*(Image_Data+DataCot) & 0x30 ) >> 4 ));
		}

		BitCount++;
		if (BitCount == (8 /BPP)) {
			*(I_Send_Data+ImageAttr->BytesToSend) = Data;
			ImageAttr->BytesToSend++;
			BitCount = 0;
			Data = 0;
			BytesSend--;
		}

		Xp++;
		if (Xp >= ImageAttr->Width) {
			Xp = 0;
			Yp++;
		}
	}

	EPRINTF("__BytesToSend %d\n",ImageAttr->BytesToSend);
	EPRINTF("__DataCot: %d, BytesSend: %d, X: %d, Y: %d \n",DataCot,BytesSend,Xp,Yp);

	return 0;
}

static int SendCommand(struct My_Image *D_Image, BYTE *ImageData)
{
	u32 iLen;
	int Check;

	EPRINTF("__ cmd : %X data: %d\n", D_Image->Command, D_Image->Cmd_Dat);
	
	switch (D_Image->Command) {
	case dc_Init: 
	case dc_GoToSleep:
	case dc_GoToNormal:
	case dc_EraseDisplay:
	case dc_Rotate: 
	case dc_SetDepth:
	case dc_DisplayImage:
	case dc_StopNewImage:
	case dc_DisplayPartial:
  case dc_ForcedRefresh:
		Check = EINK_Write(D_Image->Command,EINK_Send_Command);
		mdelay(50);
		if (D_Image->Cmd_Dat!=NO_ARG)
			Check = EINK_Write(D_Image->Cmd_Dat,EINK_Send_Data);
		break;
	case dc_LoadImage:
	case dc_PartialImage:
		iLen = D_Image->BytesToSend;
		EPRINTF("__Data Len: %d \n",iLen);
		Check = EINK_Write(D_Image->Command,EINK_Send_Command);
		mdelay(50);
		WriteDataBuf(ImageData,iLen);
		break;
	case dc_GetStatus:
		Check = EINK_Write(D_Image->Command,EINK_Send_Command);
		Check = ReadData(ImageData);
		break;
	default:
		EPRINTF("__this send is a poor cmd!\n");
		return -1;
	}

	return 1;
}

static void einkfb_rotate(int angle)
{
	struct My_Image ImageCnt;
	//BYTE *Image_Data;
 
	EPRINTF("\n" );
	//Image_Data = info->screen_base;

	switch (angle) {
	case 0:
	case 360:
		ImageCnt.Cmd_Dat = 0x00;
		break;
  	case 90:
  		ImageCnt.Cmd_Dat = 0x01;
  		break;
  	case 180:
  		ImageCnt.Cmd_Dat = 0x02;
  		break;
  	case 270:
  		ImageCnt.Cmd_Dat = 0x03;
  		break;
  	default:
  		EPRINTF("__nonsupport this mode\n");
		return;
	}

	ImageCnt.Command = dc_Rotate;
	SendCommand(&ImageCnt, NULL);
}

static void einkfb_Erase( struct My_Image *ImageCnt, bool Partial)
{  
	u8  BPP =2;
	int BytesSend =0;
	 
	EPRINTF("\n" );	 
	if (Partial) {      
		*(I_Send_Data+0) = ((ImageCnt->Xpos) >> 8)& 0xFF;
		*(I_Send_Data+1) = (ImageCnt->Xpos)& 0xFF;
		*(I_Send_Data+2) = ((ImageCnt->Ypos) >> 8)& 0xFF;
		*(I_Send_Data+3) = (ImageCnt->Ypos)& 0xFF;
		*(I_Send_Data+4) = ((ImageCnt->Xpos + ImageCnt->Width - 1) >> 8)& 0xFF;
		*(I_Send_Data+5) = (ImageCnt->Xpos + ImageCnt->Width - 1)& 0xFF;
		*(I_Send_Data+6) = ((ImageCnt->Ypos + ImageCnt->Height - 1) >> 8)& 0xFF;
		*(I_Send_Data+7) = (ImageCnt->Ypos + ImageCnt->Height - 1)& 0xFF;

		BytesSend = ((((ImageCnt->Width * ImageCnt->Height) - 1) / (8 / BPP)) + 1);
		ImageCnt->Command  = dc_PartialImage;
		ImageCnt->BytesToSend = 8;
		while (BytesSend > 0) {
			*(I_Send_Data+ImageCnt->BytesToSend) = 0xFF;
			ImageCnt->BytesToSend++;
			BytesSend--;
		}
		SendCommand(ImageCnt, I_Send_Data);

   	 	ImageCnt->Command = dc_DisplayPartial;
   	 	ImageCnt->Cmd_Dat = NO_ARG;
		SendCommand(ImageCnt, NULL);
	} else {
		ImageCnt->Command = dc_EraseDisplay;
		ImageCnt->Cmd_Dat = 1;
		SendCommand(ImageCnt, NULL);
	}
}
 /*
  */
static int einkfb_ioctl( struct fb_info *info, unsigned int cmd, unsigned long arg )
{
	u8 Check;
	bool Partial =0;
	BYTE *Image_Data;
	struct My_Image ImageCnt;
	
	EPRINTF("__ cmd : %X data: %lx\n", cmd, arg);
  
	switch (cmd) {
	case dc_EinkShow: {	
		ImageCnt.Command = dc_ForcedRefresh;
    ImageCnt.Cmd_Dat = NO_ARG;
    Check = SendCommand(&ImageCnt, NULL);   
		if (copy_from_user(&ImageCnt, (struct My_Image *)arg, sizeof(struct My_Image )))
			return -EFAULT; 			  			
		EPRINTF( "__X:%d Y:%d width:%d height:%d\n",ImageCnt.Xpos,ImageCnt.Ypos,
					ImageCnt.Width,ImageCnt.Height);
		info->screen_base = ImageCnt.EinkBase;
		Image_Data = ImageCnt.EinkBase;                                 
		EPRINTF( "__EinkBase:%x\n", (int)ImageCnt.EinkBase);
#if 0
{
	int i;
	for (i = 0; i < 10; i++) {
		printk("%x ", ImageCnt.EinkBase[i]);
		I_Send_Data[i] = i;
	}
	printk("\n");
	printk("I_Send_Data addr=%x\n", (int)I_Send_Data);
}
#endif
		if (ImageCnt.Width%4 != 0)
			ImageCnt.Width += (4-ImageCnt.Width%4);
		if (ImageCnt.Height%4 != 0)
			ImageCnt.Height += (4-ImageCnt.Height%4);
		EPRINTF( "__X:%d Y:%d width:%d height:%d\n",ImageCnt.Xpos,ImageCnt.Ypos,
					ImageCnt.Width,ImageCnt.Height );
		/*if(ImageCnt.Width>600 || ImageCnt.Height>800)
			return -1;	*/
		Partial = ! (((ImageCnt.Width == 600) && (ImageCnt.Height == 800)) ||
					((ImageCnt.Width == 800) && (ImageCnt.Height == 600)));  

		if (!Partial) {
			if (ImageCnt.Width > ImageCnt.Height)
				ImageCnt.Cmd_Dat = 0x00;
			else
				ImageCnt.Cmd_Dat = 0x01;
			ImageCnt.Command = dc_Rotate; 
//			Check = SendCommand( &ImageCnt, NULL);
        
			ImageCnt.Command = dc_LoadImage;
			ImageCnt.BytesToSend = 0;
			ImageHandler (&ImageCnt, Image_Data, Partial);
			Check = SendCommand(&ImageCnt, I_Send_Data);    
		} else {
			ImageCnt.Command = dc_PartialImage;
			ImageCnt.BytesToSend  = 8;
			ImageHandler (&ImageCnt, Image_Data, Partial);
			Check = SendCommand(&ImageCnt, I_Send_Data);  
		}

		ImageCnt.Command = dc_StopNewImage;
		ImageCnt.Cmd_Dat = NO_ARG;
		Check = SendCommand(&ImageCnt, NULL);


		if (!Partial) {
			ImageCnt.Command = dc_DisplayImage;
		} else {		
			ImageCnt.Command = dc_DisplayPartial;
		}
		ImageCnt.Cmd_Dat = NO_ARG;
		Check = SendCommand(&ImageCnt, NULL);
   	 		 	
		/*ImageCnt.Command = dc_StopNewImage; 	
		ImageCnt.Cmd_Dat = NO_ARG;
		Check = SendCommand( &ImageCnt, NULL );

		ImageCnt.Command = dc_GetStatus; 	
		ImageCnt.Cmd_Dat = NO_ARG;
		Check = SendCommand( &ImageCnt, &ImageCnt.flags );*/	
		break;
	}

	case dc_EraseEink: {
		if (copy_from_user(&ImageCnt, (struct My_Image *)arg, sizeof(struct My_Image )))
			return -EFAULT; 			  			
		EPRINTF( "__X:%d Y:%d width:%d height:%d\n",ImageCnt.Xpos,ImageCnt.Ypos,
					ImageCnt.Width,ImageCnt.Height );	 
		if (ImageCnt.Width%4 != 0)
			ImageCnt.Width += (4-ImageCnt.Width%4);
		if (ImageCnt.Height%4 != 0)
			ImageCnt.Height += (4-ImageCnt.Height%4);
		EPRINTF( "__X:%d Y:%d width:%d height:%d\n",ImageCnt.Xpos,ImageCnt.Ypos,
					ImageCnt.Width,ImageCnt.Height );

		Partial = ! (((ImageCnt.Width == 600) && (ImageCnt.Height == 800)) ||
					((ImageCnt.Width == 800) && (ImageCnt.Height == 600)));                                 			
		einkfb_Erase(&ImageCnt,Partial);
		break;
	}
	case dc_EinkRotate:
		einkfb_rotate(arg);
		break;
	case dc_EinkSet:
		if (arg == 1)
			ImageCnt.Cmd_Dat = 0x00;
	       else
			ImageCnt.Cmd_Dat = 0x02;
		ImageCnt.Command = dc_SetDepth;  
		Check = SendCommand(&ImageCnt, NULL); 
		break;
	case dc_EinkInit:
		ImageCnt.Command = dc_Init;
		ImageCnt.Cmd_Dat = 1;
		Check = SendCommand(&ImageCnt, NULL);
		break;
	default:
		EPRINTF("__this is a poor cmd!\n");
		return -EINVAL;
	}

	return 0;
}

static int einkfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	return 0;
}

//static int IoNumber = 0;
static int einkfb_open ( struct fb_info *info, int user)
{
#if 0
	int i;
	struct My_Image ImageCnt;

	EPRINTF("\n");
	IoNumber = 	sizeof(eink_CtrIO) / sizeof(eink_CtrIO[0]);
	for (i = 0; i < IoNumber; i++) {
		set_gpio_ctrl(eink_CtrIO[i].port,eink_CtrIO[i].pullup ,eink_CtrIO[i].mode);
	}
	Set_Eink_DataBus(GPIO_MODE_IN);
	write_gpio_bit(H_PWR,  1);
	udelay(10);
	write_gpio_bit(H_NRST, 1);
	udelay(500);
	write_gpio_bit(H_WUP, 0);
	write_gpio_bit(H_NRST, 0);
	udelay(5);
	write_gpio_bit(H_NRST, 1);
#else
	struct My_Image ImageCnt;

//	EINK_SET_ACK_INPUT();

	EPRINTF("\n");
//mark for demo
//	einkfb_wakeup();
//	mdelay(1000);

//	einkfb_Erase(&ImageCnt, 0);
//	mdelay(1000);
		
#endif

	if (einkfb_default.bits_per_pixel == 1)
		ImageCnt.Cmd_Dat = 0x00;
	else
		ImageCnt.Cmd_Dat = 0x02;
	ImageCnt.Command = dc_SetDepth;
	SendCommand(&ImageCnt, NULL);
	mdelay(1000);

		ImageCnt.Command = dc_GetStatus; 	
		ImageCnt.Cmd_Dat = NO_ARG;
		ImageCnt.flags = 0x0;
		printk("pre status = %x\n", ImageCnt.flags);
		SendCommand(&ImageCnt, &ImageCnt.flags);
		printk("after status = %x\n\n", ImageCnt.flags);

	if (!(I_Send_Data = vmalloc(120008)))
		return -ENOMEM;        
	EPRINTF("__send addr: %x\n", (int)I_Send_Data);

	return 0;
}

static int einkfb_release( struct fb_info *info, int user)
{
	EPRINTF("\n");
#if 0
	int i=0;
	EPRINTF("\n");

	for (i = 0; i < IoNumber; i++) {
		set_gpio_ctrl(eink_CtrIO[i].port,eink_CtrIO[i].pullup ,eink_CtrIO[i].mode);
		if(eink_CtrIO[i].mode==GPIO_MODE_OUT)
			write_gpio_bit(eink_CtrIO[i].port, 1);
	}
	Set_Eink_DataBus(GPIO_MODE_IN);
#endif
	vfree(I_Send_Data);
	return 0;
}

#ifndef MODULE
static int __init einkfb_setup(char *options)
{
	char *this_opt;

	if (!options || !*options)
		return 1;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
  }
	return 1;
}
#endif  /*  MODULE  */


static int einkfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/*
	 *  Some very basic checks
	 */
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;
	if (var->bits_per_pixel <= 1)
		var->bits_per_pixel = 1;
	else if (var->bits_per_pixel>1 && var->bits_per_pixel<=4)
		var->bits_per_pixel = 4;
	else if (var->bits_per_pixel>4 && var->bits_per_pixel<=8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel>8 && var->bits_per_pixel<=16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel>16 && var->bits_per_pixel<=24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel>24 && var->bits_per_pixel<=32)
		var->bits_per_pixel = 32;
	else
		return -EINVAL;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	switch (var->bits_per_pixel) {
	case 1:
	case 4:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 16:		/* RGBA 5551 */
		if (var->transp.length) {
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 10;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else {	/* RGB 565 */
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 11;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;
	return 0;
}

static int __init einkfb_probe(struct platform_device *dev)
{

	struct fb_info *info;
	int retval = -ENOMEM;

// addr for demo
	struct My_Image ImageCnt;

	//einkfb_wakeup();
	
	einkfb_Erase(&ImageCnt, 0);
	mdelay(1000);

//	if (!(VideoMemory = vmalloc(videomemorysize)))
//		return retval;  
//	memset(VideoMemory, 0, videomemorysize);

	info = framebuffer_alloc(0, &dev->dev);
	if (!info)
		return retval;
	info->fbops = &einkfb_ops;
	info->var = einkfb_default;
	info->fix = einkfb_fix;
	info->fix.smem_len = VIDEOSIZE;
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;	
	//info->screen_base = (char __iomem *)VideoMemory;
	info->screen_size = videomemorysize;
	//printk("virtual addr: %x\n",(u32)info->screen_base);
	
	retval = fb_alloc_cmap(&info->cmap, 16, 0);
	if (retval < 0){
			printk("rigister cmap buffer err !\n");
			goto err;}

	einkfb_check_var(&info->var,info);
		
	retval = register_framebuffer(info);
	if (retval < 0){
			printk("rigister frame buffer err !\n");
			goto err1;}
			
	platform_set_drvdata(dev, info);

	printk(KERN_INFO
	  "fb%d: Virtual frame buffer device, using %ldK of video memory\n",info->node,videomemorysize>>10);

	return 0;
err:
	fb_dealloc_cmap(&info->cmap);
err1: 
	framebuffer_release(info);

	return retval;
}

#ifdef CONFIG_PM
/* suspend and resume support for the lcd controller */
static int einkfb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct My_Image ImageCnt;
	//BYTE *Image_Data;
	struct fb_info *info;
	info =  platform_get_drvdata(dev);
#if 0
	write_gpio_bit(H_NRST, 0);
	udelay(500);
	write_gpio_bit(H_NRST, 1);
#endif
	//Image_Data = info->screen_base;
	ImageCnt.Command = dc_GoToSleep;
	ImageCnt.Cmd_Dat = NO_ARG;
	SendCommand(&ImageCnt, NULL);
	//einkfb_stop_lcd(info);
	/* sleep before disabling the clock, we need to ensure
	* the LCD DMA engine is not going to get back on the bus
	* before the clock goes off again (bjd) */
	msleep(1);
	return 0;
}

static int einkfb_resume(struct platform_device *dev)
{
	struct My_Image ImageCnt;
	//BYTE *Image_Data;
	struct fb_info *info;
	info = platform_get_drvdata(dev);
  
	//Image_Data = info->screen_base;
	einkfb_wakeup();
	ImageCnt.Command = dc_GoToNormal;
	ImageCnt.Cmd_Dat = NO_ARG;
	SendCommand(&ImageCnt, NULL);
	msleep(1);
	//einkfb_init_registers(info);
	return 0;
}

#else
#define s3c2410fb_suspend NULL
#define s3c2410fb_resume  NULL
#endif

static struct platform_driver einkfb_driver = {
	.probe	= einkfb_probe,
#ifdef CONFIG_PM
	.suspend	= einkfb_suspend,
	.resume	= einkfb_resume,
#endif
	.driver = {
		.name	= "einkfb",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device *einkfb_device;

static int __init einkfb_init(void)
{
	int ret = 0;

#ifndef MODULE
	char *option = NULL;
	if (fb_get_options("einkfb", &option))
		return -ENODEV;
	einkfb_setup(option);
#endif

	ret = platform_driver_register(&einkfb_driver);
	if (!ret) {
		EPRINTF("__device alloc\n");
		einkfb_device = platform_device_alloc("einkfb", 0);
		
		if (einkfb_device){
			EPRINTF("__device add\n");
			ret = platform_device_add(einkfb_device);
		}
		else
			ret = -ENOMEM;
		if (ret) {
			EPRINTF("__why close it\n");
			platform_device_put(einkfb_device);
			platform_driver_unregister(&einkfb_driver);
		}
	}

	return ret;
}

module_init(einkfb_init);

#ifdef MODULE
static void __exit einkfb_exit(void)
{
	platform_device_unregister(einkfb_device);
	platform_driver_unregister(&einkfb_driver);
}

module_exit(einkfb_exit);

MODULE_LICENSE("GPL");
#endif				/* MODULE */
