/*
 * linux/drivers/video/soclefb.c
 *	Copyright (c) Arnaud Patard, Ben Dooks
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    SOCLE LCD Controller Frame Buffer Driver
 *
 * ChangeLog
 * 2007-06-28: Ryan Chen Create it
 *
 */


#ifdef  CONFIG_DOUBLE_BUFFER
/* Android will use double buffer in video if there is enough */
#define NUMBER_OF_BUFFERS 2
#else
#define NUMBER_OF_BUFFERS 1
#endif

#define T_PANEL 0
#define T_TV 1
#define T_HDMI 2

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach/map.h>
#include <mach/regs-lcd.h>
#include <mach/socle-scu.h>
static wait_queue_head_t wq;


#include <linux/spi/spi.h>
struct spi_device lcd_spi;

struct soclefb_lcd_panel {
	struct fb_videomode	mode;
	signed short		width;	/* width in mm */
	signed short		height;	/* height in mm */
	u32		  bpp;
};

struct soclefb_info {
	struct platform_device *pdev;
	struct fb_info		*info;	
	struct soclefb_lcd_panel *panel;
	struct resource *reg_res;
	struct resource *fb_res;
	void __iomem *regs;
	void __iomem *addr;
	int addr_assign;
	int irq;
	unsigned int pseudo_palette[16];
	unsigned int ctrl0;
	unsigned int htiming;
	unsigned int vtiming;
	unsigned int resolution;	
};


struct soclefb_lcd_panel panel_35 = {
		.mode	= {
			.name			= "AM320240L4TMQW00H",
			.refresh			= 60,
			.xres			= 320,
			.yres			= 240,
			.pixclock			= 166666,
			.left_margin		= 38,
			.right_margin		= 20,
			.upper_margin	= 15,
			.lower_margin		= 5,
			.hsync_len		= 30,
			.vsync_len		= 3,
			.sync			= 0,
			.vmode			= FB_VMODE_NONINTERLACED,
		},
		.width	= 70,
		.height	= 52,	
		.bpp	= 16,
};

struct soclefb_lcd_panel panel_48 = {
		.mode	= {
			.name			= "LMS480KC02",
			.refresh			= 60,
			.xres			= 800,
			.yres			= 480,
			.pixclock			= 37037,
			.left_margin		= 87,
			.right_margin		= 16,
			.upper_margin	= 18,
			.lower_margin		= 8,
			.hsync_len		= 12,
			.vsync_len		= 1,
			.sync			= 0,
			.vmode			= FB_VMODE_NONINTERLACED,
		},
    .width=104,
    .height=62,
		.bpp	= 16,

};

struct soclefb_lcd_panel panel_70 = {
		.mode	= {
			.name			= "MTF-TW70SP911-LB",
			.refresh			= 60,
			.xres			= 800,
			.yres			= 480,
			.pixclock			= 37037,
			.left_margin		= 150,
			.right_margin		= 50,
			.upper_margin	= 33,
			.lower_margin		= 10,
			.hsync_len		= 56,
			.vsync_len		= 2,
			.sync			= 0,
			.vmode			= FB_VMODE_NONINTERLACED,
		},
		.width  = 152,
		.height	= 91,
		.bpp	= 16,
};

struct soclefb_lcd_panel vga = {
		.mode	= {
			.name			= "vga",                                                    
			.refresh			= 60,                                                   
			.xres			= 640,                                                      
			.yres			= 480,                                                      
			.pixclock			= 37037,                                                
			.left_margin		= 96,                                                 
			.right_margin		= 26,                                                 
			.upper_margin	= 33,                                                   
			.lower_margin		= 10,                                                 
			.hsync_len		= 96,                                                   
			.vsync_len		= 2,                                                    
			.sync			= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,                                                        
			.vmode			= FB_VMODE_NONINTERLACED,            
		},
		.width  = -1,
		.height	= -1,
		.bpp	= 16,

};

struct soclefb_lcd_panel svga = {                                            
		.mode	= {                                                               
			.name			= "svga",                                                    
			.refresh			= 60,                                                   
			.xres			= 800,                                                      
			.yres			= 600,                                                      
			.pixclock			= 37037,                                                
			.left_margin		= 128,                                                 
			.right_margin		= 24,                                                 
			.upper_margin	= 22,                                                   
			.lower_margin		= 1,                                                 
			.hsync_len		= 72,                                                   
			.vsync_len		= 2,                                                    
			.sync			= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,                                                     
			.vmode			= FB_VMODE_NONINTERLACED,                                 
		},                                                                      
		.width  = -1,                                                          
		.height	= -1,                                                         
		.bpp	= 16,                                                                   
                                                                            
};

struct soclefb_lcd_panel xvga = {                                            
		.mode	= {                                                               
			.name			= "xvga",                                                    
			.refresh			= 43,                                                   
			.xres			= 1024,                                                      
			.yres			= 768,                                                      
			.pixclock			= 18518,                                                
			.left_margin		= 56,                                                 
			.right_margin		= 8,                                                 
			.upper_margin	= 40,                                                   
			.lower_margin		= 1,                                                 
			.hsync_len		= 176,                                                   
			.vsync_len		= 8,                                                    
			.sync			= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,                                                     
			.vmode			= FB_VMODE_NONINTERLACED,                                 
		},                                                                      
		.width  = -1,                                                          
		.height	= -1,                                                                
		.bpp	= 16,	                                     
};





/* Debugging stuff */

#define FBDBG 0

#define dprintk(msg...)	if (FBDBG) { printk(KERN_DEBUG "soclefb: " msg); }

/* useful functions */

#if (NUMBER_OF_BUFFERS > 1)
/* Pan the display if device supports it. */
static int socle_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct soclefb_info *sfb = info->par;
	u32 addr = var->yoffset * info->fix.line_length + info->fix.smem_start;;
	writel(SOCLE_LCD_INTR_EN_PAGE0_READ, sfb->regs + SOCLE_LCD_INTR_EN);
	writel(addr, sfb->regs + SOCLE_LCD_PAGE0_ADDR);
	interruptible_sleep_on(&wq);
	return 0;
}
#endif

static int soclefb_init_registers(struct soclefb_info *sfb)
{
	/* Initialise LCD with values from haret */

	/* Disable LCD controller */	
	//printk("soclefb: ctrl=%x,ht=%x,vt=%x,addr=%x\n",sfb->ctrl0,sfb->htiming,sfb->vtiming, sfb->info->fix.smem_start);
	writel(sfb->ctrl0 & ~SOCLE_LCD_CTRL0_ENABLE , sfb->regs + SOCLE_LCD_CTRL0);


	//Set Panel timing
		
	writel(sfb->htiming, sfb->regs + SOCLE_LCD_H_TIMING);
	writel(sfb->vtiming, sfb->regs + SOCLE_LCD_V_TIMING);
	if(readl(sfb->regs + SOCLE_LCD_INTR_VER)==0x0b01) 
		writel(sfb->resolution, sfb->regs + SOCLE_LCD_RESOLUTION);
	writel(sfb->info->fix.smem_start, sfb->regs + SOCLE_LCD_PAGE0_ADDR);

	/* Enable LCD controller */
	
	sfb->ctrl0 |= SOCLE_LCD_CTRL0_ENABLE;
	writel(sfb->ctrl0 , sfb->regs + SOCLE_LCD_CTRL0);
	
	return 0;
}


/*
 *      soclefb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */

static int soclefb_set_par(struct fb_info *info)
{
	struct soclefb_info *sfb = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int in_clk,pix_clk,clk_div=2;

	sfb->info->fix.line_length     = (var->xres*var->bits_per_pixel)/8;

	if(readl(sfb->regs + SOCLE_LCD_INTR_VER)!=0x0b01) {
		if(var->xres==1024) {
			sfb->htiming=(var->left_margin <<16) | (var->right_margin << 24) |0x2 ;	
		}
		else
			sfb->htiming=(var->left_margin <<16) | (var->right_margin << 24) | (var->xres & 0x3fc);

		if(var->hsync_len!=1)
			sfb->htiming |= ((var->hsync_len>>2) <<10);
		sfb->vtiming= (var->upper_margin << 16) | (var->lower_margin << 24) | (var->yres);
		if(var->vsync_len!=1)
			sfb->vtiming |= ((var->vsync_len>>2) <<10);
		
	}
	else {
		sfb->htiming=(var->left_margin <<8) | (var->right_margin << 20) ;
		if(var->hsync_len!=1)
			sfb->htiming |= (var->hsync_len>>2);		
		sfb->vtiming=(var->upper_margin << 8) | (var->lower_margin << 20) | (var->vsync_len);
		sfb->resolution=(var->yres<<16) | (var->xres);
	}
	
	sfb->ctrl0 |= var->sync & FB_SYNC_HOR_HIGH_ACT  ? 0 : SOCLE_LCD_CTRL0_HSYNC;
	sfb->ctrl0 |= var->sync & FB_SYNC_VERT_HIGH_ACT ? 0 : SOCLE_LCD_CTRL0_VSYNC;
		
	sfb->panel->bpp=var->bits_per_pixel;
	if(sfb->panel->bpp==16) { 
		sfb->ctrl0 &= ~SOCLE_LCD_CTRL0_24BPP;
		sfb->ctrl0 |= SOCLE_LCD_CTRL0_COLOUR_GREEN;		
	}
	else
		sfb->ctrl0 |= SOCLE_LCD_CTRL0_24BPP;

#if defined(CONFIG_ARCH_PDK_PC9220) || defined(CONFIG_ARCH_PDK_PC9223)
	in_clk=socle_scu_apb_clock_get()/1000000;  //get_apb_clk
#elif defined(CONFIG_ARCH_MDK_3D)
	in_clk=54;
#else
	in_clk=socle_scu_ahb_clock_get()/1000000;
#endif
	dprintk("in_clk=%d\n",in_clk);
	pix_clk=(1000000/(var->pixclock));
	
	while(pix_clk) {
		if(pix_clk*clk_div>=in_clk)
			break;
		clk_div++;
	}
	if(clk_div>255)
		clk_div=255;
	sfb->ctrl0 &= ~(0xff<<8);
	sfb->ctrl0 |= (clk_div<<8);
		
 	soclefb_init_registers(sfb);
	return 0;
}


static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int soclefb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{


	struct soclefb_info *sfb = info->par;
	unsigned int val;


	switch (sfb->info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = sfb->info->pseudo_palette;
			val  = chan_to_field(red,   &sfb->info->var.red);
			val |= chan_to_field(green, &sfb->info->var.green);
			val |= chan_to_field(blue,  &sfb->info->var.blue);

			pal[regno] = val;
		}
		break;

	case FB_VISUAL_PSEUDOCOLOR:
//		printk("FB_VISUAL_PSEUDOCOLOR \n");		
		if (regno < 256) {
			/* currently assume RGB 5-6-5 mode */

			val  = ((red   >>  0) & 0xf800);
			val |= ((green >>  5) & 0x07e0);
			val |= ((blue  >> 11) & 0x001f);
		}

		break;

	default:
		return 1;   /* unknown type */
	}

	return 0;
}


/**
 *      soclefb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
  *	Returns negative errno on error, or zero on success.
 *
 */
static int soclefb_blank(int blank_mode, struct fb_info *info)
{
//	dprintk("blank(mode=%d, info=%p)\n", blank_mode, info);
	//RGB halt 
	struct soclefb_info *sfb=info->par;
	if(blank_mode == 1)
		writel(readl(sfb->regs + SOCLE_LCD_CTRL0) | SOCLE_LCD_CTRL0_RGBHALT , sfb->regs + SOCLE_LCD_CTRL0);
	else
		writel(readl(sfb->regs + SOCLE_LCD_CTRL0) & ~SOCLE_LCD_CTRL0_RGBHALT , sfb->regs + SOCLE_LCD_CTRL0);
	return 0;
}

static int soclefb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{

	

	var->xres_virtual = var->xres;
	var->yres_virtual =var->yres * NUMBER_OF_BUFFERS;
		
	if (var->bits_per_pixel == 16) {
		var->blue.length	= 5;
		var->blue.offset	= 0;	
		var->green.length	= 6;
		var->green.offset	= var->blue.offset + var->blue.length;	
		var->red.length = 5;
		var->red.offset = var->green.offset + var->green.length;
		var->transp.length = 0;
		var->transp.offset = 0;
		
	} 
	else if(var->bits_per_pixel ==24 ||var->bits_per_pixel ==32){
		var->blue.length	= 8;
		var->blue.offset	= 0;	
		var->green.length	= 8;
		var->green.offset	= var->blue.offset + var->blue.length;	
		var->red.length = 8;
		var->red.offset = var->green.offset + var->green.length;
		var->transp.length=8;
		var->transp.offset=24;
	}
	else {
		printk("!!!!soclefb_check_var err\n");
		return -EINVAL;
		}	

	return 0;
}

	
#define YUV_MODE 0x4630

static int socle_ioctl(struct fb_info *info, unsigned int cmd,unsigned long arg)
{

	struct soclefb_info *sfb = info->par;
	switch (cmd) {
	case YUV_MODE:
		if(arg==0)
			writel(readl(sfb->regs + SOCLE_LCD_YUV2RGB_CTRL) & ~SOCLE_LCD_YUV2RGB_EN , sfb->regs + SOCLE_LCD_YUV2RGB_CTRL);
		else {
			u32 y_addr,u_addr,v_addr;
			writel(readl(sfb->regs + SOCLE_LCD_YUV2RGB_CTRL) | SOCLE_LCD_YUV2RGB_EN , sfb->regs + SOCLE_LCD_YUV2RGB_CTRL);
			writel(readl(sfb->regs + SOCLE_LCD_YUV2RGB_CTRL) | SOCLE_LCD_YUV420, sfb->regs + SOCLE_LCD_YUV2RGB_CTRL);
			y_addr=info->fix.smem_start;
			u_addr=y_addr+ (info->var.xres*info->var.yres);
			v_addr=u_addr+(info->var.xres*info->var.yres/4);
			writel(y_addr,sfb->regs + SOCLE_LCD_Y_PAGE0_ADDR);
			writel(u_addr,sfb->regs + SOCLE_LCD_Cb_PAGE0_ADDR);
			writel(v_addr,sfb->regs + SOCLE_LCD_Cr_PAGE0_ADDR);
		}
		return 0;	
	default:
		printk("soclefb: error command");
		return -EINVAL;
	}
		
}


static struct fb_ops soclefb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= soclefb_check_var,
	.fb_set_par	= soclefb_set_par,
	.fb_blank	= soclefb_blank,
	.fb_setcolreg	= soclefb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_ioctl = socle_ioctl,
#if (NUMBER_OF_BUFFERS > 1)
	.fb_pan_display = socle_fb_pan_display,
#endif
};




static void socle_fbmem_free(struct soclefb_info *sfb)
{
	if(sfb->addr_assign) 
		iounmap(sfb->info->screen_base);
	else
		dma_free_writecombine(&sfb->pdev->dev, sfb->info->fix.smem_len, sfb->info->screen_base, sfb->info->fix.smem_start);
}


#define SET_TX_RX_LEN(tx, rx)	(((tx) << 16) | (rx))

static char init_array[]={	0x00,0x63,	0x01,0x55,	0x02,0x01,	0x03,0x0c,	0x04,0x00,	0x05,0x11,	0x06,0x00,	0x07,0x00,	0x08,0x44,	0x09,0x01,	0x0a,0x30,	0x0b,0x1a,	0x0c,0x29,	0x0d,0x3f,	0x0e,0x20,	0x0f,0x20,	0x10,0xfa,	0x11,0xfa,	0x12,0x0a,	0x13,0x20,	0x14,0x20,	0x15,0x20,	0x16,0x80,	0x17,0x80,	0x18,0x80,	0x20,0x00,	0x21,0x0c,	0x22,0xd9,	0x23,0x19,	0x24,0x56,	0x25,0x88,	0x26,0xb9,	0x27,0x4b,	0x28,0xbe,	0x29,0x21,	0x2a,0x77,	0x2b,0xff,	0x2c,0x40,	0x2d,0x95,	0x2e,0xfe,	0x50,0x00,	0x51,0x0c,	0x52,0xd9,	0x53,0x19,	0x54,0x56,	0x55,0x88,	0x56,0xb9,	0x57,0x4b,	0x58,0xbe,	0x59,0x21,	0x5a,0x77,	0x5b,0xff,	0x5c,0x40,	0x5d,0x95,	0x5e,0xfe,	0x2f,0x21};

static int
samsung_panel_reg_write(u8 reg, u8 val)
{
	int err = 0;
	u8 tx_buf[2] = {0};
	struct spi_message msg;
	struct spi_transfer xfer;
	
	tx_buf[0] = reg;
	tx_buf[1] = val;
	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(struct spi_transfer));
	xfer.tx_buf = tx_buf;
	xfer.len = SET_TX_RX_LEN(2, 0);
	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(&lcd_spi, &msg);
	return err;
}


static int samsung_panel_init(void)
{
	int count,i;
	lcd_spi.bits_per_word=8;
	lcd_spi.chip_select=0;
	lcd_spi.max_speed_hz=1000000;
	lcd_spi.mode=SPI_MODE_3;
	lcd_spi.master=spi_busnum_to_master(0);
	if(lcd_spi.master==NULL) {
		printk("master= null\n");
		return -1;
	}
	spi_setup(&lcd_spi);

	count=sizeof(init_array)/2;	
	for(i=0;i<count;i++) {
		if(samsung_panel_reg_write(init_array[2*i],init_array[2*i+1]))
			return -1;
	}
	return 0;
	
}

static irqreturn_t
soclefb_isr(int irq, void *parm)
{
	u32 status;
	struct soclefb_info *sfb=parm;
	status = readl(sfb->regs + SOCLE_LCD_INTR_STS);
	writel(SOCLE_LCD_INTR_EN_PAGE0_READ, sfb->regs + SOCLE_LCD_INTR_DIS);
	if (status & SOCLE_LCD_INTR_STS_PAGE0_READ)
		wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}

static void soclefb_stop_lcd(struct soclefb_info *sfb)
{
	//unsigned long flags;

	//local_irq_save(flags);

	writel(readl(sfb->regs + SOCLE_LCD_CTRL0) & (~SOCLE_LCD_CTRL0_ENABLE), sfb->regs + SOCLE_LCD_CTRL0);

	//local_irq_restore(flags);
}

static int socle_fb_setup(struct soclefb_info *sfb)
{
	char *options = NULL;
	char *this_opt;
	int type=T_PANEL;
	
	sfb->info->fix.smem_start=0;
	sfb->addr_assign=0;
	sfb->panel = &panel_70;
	fb_get_options("soclefb",&options);
	
  if(options) {	
  	while ((this_opt = strsep(&options, ",")) != NULL) {
  		if (!strncmp(this_opt, "type=", 5)) {
  			if(!strncmp(this_opt+5, "PANEL35", 7))
  				sfb->panel	= &panel_35;
  			else if(!strncmp(this_opt+5, "PANEL48", 7)) {
  				sfb->panel	= &panel_48;
  				if(samsung_panel_init())
						printk("samsung panel init fail\n");
  			}
				else if(!strncmp(this_opt+5, "PANEL70", 7))
  				sfb->panel	= &panel_70;
				else if(!strncmp(this_opt+5, "TV", 2)) {
  				sfb->panel = &vga;
  				type=T_TV;
  			}
  			else if(!strncmp(this_opt+5, "HDMI", 4)) {
  				sfb->panel = &svga;
  				type=T_HDMI;
  			}
  		}
  		else if(!strncmp(this_opt, "size=", 5)) {
  			if(type!=T_PANEL) {
  				if(!strncmp(this_opt+5, "640x480", 7))
  					sfb->panel = &vga;
  				else if(!strncmp(this_opt+5, "800x600", 7))
  					sfb->panel = &svga;
  				else if(type==T_HDMI) {
  					if(!strncmp(this_opt+5, "1024x768", 8))
  						sfb->panel = &xvga;
  				}
  			}
  		}
  		else if(!strncmp(this_opt, "bpp=", 4)) {
  			if(!strncmp(this_opt+4, "32", 2))
  					sfb->panel->bpp = 32;
  			else if(!strncmp(this_opt+4, "16", 2))
  					sfb->panel->bpp = 16;
  		}
  		else if(!strncmp(this_opt, "addr=", 5)) {
  			sfb->info->fix.smem_start = PHYS_OFFSET + memparse(this_opt + 5, NULL);
  			sfb->addr_assign=1;
  		}
  	}
  }

	return 0;
}

static int __init soclefb_probe(struct platform_device *pdev)
{
	struct soclefb_info *sfb;
	struct fb_info	   *info;
	struct device *dev = &pdev->dev;
	int ret;

	info = framebuffer_alloc(sizeof(struct soclefb_info), dev);

	if (!info) {
		dev_err(dev, "cannot allocate memory\n");
		return -ENOMEM;
	}


	sfb = info->par;
	sfb->info = info;
	sfb->pdev = pdev;
	
  strcpy(info->fix.id, sfb->pdev->name);
  
  sfb->reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!sfb->reg_res) {
  	dev_err(dev, "register resources unusable\n");
    ret = -ENXIO;
    goto free_info;
  }
  
  sfb->irq = platform_get_irq(pdev, 0);
	if (!sfb->irq) {
  	dev_err(dev, "unable to get irq\n");
    ret = -ENXIO;
    goto free_info;
  }
  
  
	info->fix.mmio_start	= sfb->reg_res->start;
	info->fix.mmio_len	= sfb->reg_res->end - sfb->reg_res->start + 1;
	
	if (!request_mem_region(info->fix.mmio_start, info->fix.mmio_len, pdev->name)) {
		dev_err(dev, "cannot request LCDC registers\n");
		ret = -EBUSY;
		goto free_info;
	}

	sfb->regs = ioremap(info->fix.mmio_start, info->fix.mmio_len);
	if (!sfb->regs) {
		dev_err(dev, "cannot map LCDC registers\n");
		ret = -ENOMEM;
		goto free_res;
	}
	
	socle_fb_setup(sfb);
	
	info->fix.smem_len = sfb->panel->mode.xres * sfb->panel->mode.yres * sfb->panel->bpp / 8 * NUMBER_OF_BUFFERS;
	if(sfb->addr_assign) {
		if (!request_mem_region(info->fix.smem_start, info->fix.smem_len, pdev->name)) {
			dev_err(dev, "cannot request LCDC mem\n");
			ret = -EBUSY;
			goto free_io;
		}

		info->screen_base = ioremap(info->fix.smem_start, info->fix.smem_len);
		if (!info->screen_base) {
			dev_err(dev, "cannot map LCDC mem\n");
			ret = -ENOMEM;
			goto free_addr;
		}
	}
	else {
		info->screen_base = dma_alloc_writecombine(dev, info->fix.smem_len,
					(dma_addr_t *)&info->fix.smem_start, GFP_KERNEL);
		if (!info->screen_base) {
			dev_err(dev, "cannot alloc LCDC mem\n");
			ret = -ENOMEM;
			goto free_io;
		}
	}	
	

	info->fix.type	    	= FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux	    = 0;

#if (NUMBER_OF_BUFFERS > 1)
	info->fix.ypanstep = 1;
#else
	info->fix.ypanstep	    = 0;
#endif
	info->fix.xpanstep	    = 0;
	info->fix.ywrapstep	    = 0;
	info->fix.visual =	FB_VISUAL_TRUECOLOR,
	info->fix.accel	    	= FB_ACCEL_NONE;

	info->var.nonstd	    = 0;
	info->var.activate	    = FB_ACTIVATE_NOW;
	info->var.height	    = sfb->panel->height;
	info->var.width	    = sfb->panel->width;
	info->var.accel_flags     = 0;

	info->fbops		    = &soclefb_ops;
	info->flags		    = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette      = sfb->pseudo_palette;

	info->var.xres	    = sfb->panel->mode.xres;
	info->var.xres_virtual    = sfb->panel->mode.xres;
	info->var.yres	    = sfb->panel->mode.yres;
	info->var.yres_virtual    = sfb->panel->mode.yres * NUMBER_OF_BUFFERS;

	info->var.bits_per_pixel  = sfb->panel->bpp;
	info->var.grayscale	= 0;
	info->var.pixclock	= sfb->panel->mode.pixclock;

	info->var.left_margin	= sfb->panel->mode.left_margin;
	info->var.right_margin	= sfb->panel->mode.right_margin;
	info->var.upper_margin	= sfb->panel->mode.upper_margin;
	info->var.lower_margin	= sfb->panel->mode.lower_margin;

	info->var.hsync_len	= sfb->panel->mode.hsync_len;
	info->var.vsync_len	= sfb->panel->mode.vsync_len;
	info->var.sync		= sfb->panel->mode.sync;
	info->var.vmode		= sfb->panel->mode.vmode;
	
	/*
	 * Allocate colourmap.
	 */
	ret=fb_alloc_cmap(&(info->cmap), 256, 0);
	if(ret) {
		dev_err(dev, "Alloc color map failed\n");
		goto free_mem;
	}

	ret = request_irq(sfb->irq, soclefb_isr, IRQF_SHARED, pdev->name, sfb);
	if (ret) {
		dev_err(dev, "Can't request LCD irq");
		ret = -EBUSY;
		goto free_cmap;
	}
	init_waitqueue_head(&wq);
	
	ret = soclefb_check_var(&info->var, info);
	if (ret)
		goto free_irq;
		
	soclefb_set_par(info);
	platform_set_drvdata(pdev, sfb);
	ret = register_framebuffer(info);
	if (!ret)
		return 0;

	dev_err(dev, "Failed to register framebuffer device: %d\n", ret);
	soclefb_stop_lcd(sfb);	
	platform_set_drvdata(pdev, NULL);
free_irq:
	free_irq(sfb->irq,sfb);
free_cmap:
	fb_dealloc_cmap(&info->cmap);
free_mem:	
	socle_fbmem_free(sfb);	
free_addr:
	if(sfb->addr_assign)
		release_mem_region(info->fix.smem_start, info->fix.smem_len);
free_io:
	iounmap(sfb->regs);
free_res:
 	release_mem_region(info->fix.mmio_start, info->fix.mmio_len);
free_info:
	framebuffer_release(info);
	return ret;

}



/*
 *  Cleanup
 */
static int soclefb_remove(struct platform_device *pdev)
{
	struct soclefb_info	   *sfb = platform_get_drvdata(pdev);

	soclefb_stop_lcd(sfb);
	free_irq(sfb->irq, sfb);
	fb_dealloc_cmap(&sfb->info->cmap);
	socle_fbmem_free(sfb);	
	if(sfb->addr_assign)
		release_mem_region(sfb->info->fix.smem_start, sfb->info->fix.smem_len);
	iounmap(sfb->regs);
 	release_mem_region(sfb->info->fix.mmio_start, sfb->info->fix.mmio_len);
	framebuffer_release(sfb->info);
	
	printk("soclefb_remove \n");
	return 0;
}

#ifdef CONFIG_PM

/* suspend and resume support for the lcd controller */

static int soclefb_suspend(struct platform_device *dev, pm_message_t state)
{

	printk("soclefb_suspend \n");
/*
	struct fb_info	   *fbinfo = platform_get_drvdata(dev);
	struct soclefb_info *info = fbinfo->par;

	soclefb_stop_lcd(info);

	msleep(1);
	clk_disable(info->clk);
*/
	return 0;
}

static int soclefb_resume(struct platform_device *dev)
{
	printk("soclefb_resume \n");
/*
	struct fb_info	   *fbinfo = platform_get_drvdata(dev);
	struct soclefb_info *info = fbinfo->par;

	clk_enable(info->clk);
	msleep(1);

	soclefb_init_registers(info);
*/
	return 0;
}

#else
#define soclefb_suspend NULL
#define soclefb_resume  NULL
#endif

static struct platform_driver soclefb_driver = {
	.probe		= soclefb_probe,
	.remove		= soclefb_remove,
	.suspend		= soclefb_suspend,
	.resume		= soclefb_resume,
	.driver		= {
		.name	= "socle-lcd",
		.owner	= THIS_MODULE,
	},
};

int __devinit soclefb_init(void)
{
	return platform_driver_register(&soclefb_driver);
}

static void __exit soclefb_cleanup(void)
{
	platform_driver_unregister(&soclefb_driver);
}

late_initcall(soclefb_init);
module_exit(soclefb_cleanup);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("Framebuffer driver for the Socle LCD");
MODULE_LICENSE("GPL");
