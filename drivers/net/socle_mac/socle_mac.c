/* socle_mac.c: ethernet driver for the SOCLE MAC core with AMBA interface.

	SOCLE Technology Corp.
	14F-2, No.287, Section 2, Kwan-Fu Road, 
	Hsin-Chu City, Taiwan
	http://www.socle-tech.com.tw
*/

/* The include file section.  We start by doing checks and fix-ups for
   missing compile flags. */
#ifndef __KERNEL__
#define __KERNEL__
#endif
#define LINUX26

#if !defined(__OPTIMIZE__)
#warning  You must compile this file with the correct options!
#warning  See the last lines of the source file.
#error You must compile this driver with "-O".
#endif
/*
#if defined(CONFIG_SMP)  &&  ! defined(__SMP__)
#define __SMP__
#endif
#if defined(CONFIG_MODVERSIONS) && defined(MODULE) && ! defined(MODVERSIONS)
#define MODVERSIONS
#endif
*/

#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/unistd.h>


//
#include <linux/mii.h>
#include <linux/skbuff.h>
#include <linux/dma-mapping.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>

#include <linux/delay.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>

#include <asm/cacheflush.h>

#include <linux/sockios.h>
#include <linux/ethtool.h>

#include <mach/reg-mac.h>

/* SOCLE_MAC:
	Before we use this driver there have some Pre-define value should be include
	in  socle_mac_driver_value.h" 
	All Board Specification Configure should be included*/
	
#define	FINAL_RELEASE_VERSION

#define DRV_NAME	"socle_mac"
#define DRV_VERSION	"v1.0"
#define DRV_RELDATE	"FEB 23, 2004"
#define DRV_BUSMODE	"Embedded AMBA"

#define	CONFIG_SOCLEMAC_BASE_ADDR1 IO_ADDRESS(SOCLE_MAC0)
#define	CONFIG_SOCLEMAC_INTV1 IRQ_MAC0

#ifdef SOCLE_MAC1
#define CONFIG_SOCLEMAC_BASE_ADDR2 IO_ADDRESS(SOCLE_MAC1)
#define	CONFIG_SOCLEMAC_INTV2 IRQ_MAC1
#define MAC_MAX_UNITS (2)
#else 
#define MAC_MAX_UNITS (1)
#endif

#define SOCLE_MAC_PHY_IP101A

#ifdef SOCLE_MAC_PHY_IP101A
#define	PHY_ID0			0x243
#define	PHY_ID1			0xC54
#elif defined SOCLE_MAC_PHY_RTL8201
#define	PHY_ID0			0x0000
#define	PHY_ID1			0x8201
#else
#error unknown SOCLE MAC PHY
#endif


#define SQ_MAC_DBG
#ifdef SQ_MAC_DBG
	#define MAC_DBG(fmt, args...) printk("\nMDIO: " fmt, ## args)
#else
	#define MAC_DBG(fmt, args...)
#endif




/* SOCLE_MAC:
	For ARM and MIPS processors, endianess can be either big or little
	depending on how the hardware is wired and configured.  The SOCLE MAC
	core can be configured by the driver for either case, so it is necessary
	to uncomment one of the following lines which will cause the appropriate
	bits to be set or cleared below in csr0 (bits 7 and 20).
*/

#define DEFAULT_CSR0_SETTING	(CSR0_BAR_RXH | CSR0_DSL_VAL(0) | CSR0_PBL_VAL(16) | CSR0_TAP_VAL(1))

#if defined(STORE_AND_FORWARD) 	/* FPGA emulation we could not get full speed */
#define DEFAULT_CSR6_SETTING	(CSR6_SF |  CSR6_FD)
#else
#define DEFAULT_CSR6_SETTING	(CSR6_TR_VAL(0) | CSR6_FD)
#endif

#define DEFAULT_CSR11_SETTING	(CSR11_TT_VAL(0)|CSR11_NTP_VAL(0)|CSR11_RT_VAL(0)|CSR11_NRP_VAL(0))

/* Operational parameters that are set at compile time. */
/* Keep the descriptor ring sizes a power of two for efficiency.
   The Tx queue length limits transmit packets to a portion of the available
   ring entries.  It should be at least one element less to allow multicast
   filter setup frames to be queued.  It must be at least four for hysteresis.
   Making the Tx queue too long decreases the effectiveness of channel
   bonding and packet priority.
   Large receive rings waste memory and confound network buffer limits.
   These values have been carefully studied: changing these might mask a
   problem, it won't fix it.
*/
#if 0
#define TX_RING_SIZE	32
#define TX_QUEUE_LEN	10
#define RX_RING_SIZE	32
#else
#define TX_RING_SIZE		32
#define TX_QUEUE_LEN		20
#define RX_RING_SIZE		64
#endif

/* Operational parameters that usually are not changed. */
#define LINK_CHECK_FAST	(1*HZ)
#define LINK_CHECK_SLOW	(30*HZ)
/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT  	(6*HZ)

/* Preferred skbuff allocation size. */
#define PKT_BUF_SZ		1536

/* Version strings */
static const char version1[] =
DRV_NAME " : " DRV_VERSION " " DRV_RELDATE " "  DRV_BUSMODE " " "Written by Sherlock\n";
static const char version2[] =
"             http://www.socle-tech.com.tw\n";

/* SOCLE_MAC:
	CSR7 interrupt enable settings,
	we define an explict interrupt enable bitmap for the SOCLE MAC Modules.
	It enables all interrupts except for ETE. */
#define SOCLE_MAC_INTR	(					\
							CSR7_NIS		\
                         |	CSR7_AIS		\
/*                         |	CSR7_ERI	*/	\
/*                         |	CSR7_GPTE	*/	\
/*                         |	CSR7_ETI	*/	\
                         |	CSR7_RPS		\
                         |	CSR7_RU			\
                         |	CSR7_RI			\
                         |	CSR7_UNF		\
                         |	CSR7_TU			\
                         |	CSR7_TPS		\
                         |	CSR7_TI			\
                        )
                         

#if 1
#define DEFAULT_IFPORTE		(options_AutoNeg)
//#define DEFAULT_IFPORTE		(options_100baseTxF)
//#define DEFAULT_IFPORTE		(options_100baseTxH)
//#define DEFAULT_IFPORTE		(options_10baseTxF)
//#define DEFAULT_IFPORTE		(options_10baseTxH)
#else   //+Tom's testing
#define DEFAULT_IFPORTE		(options_FullDuplex)
#endif

#define SOCLE_MAC_HAVE_PRIVATE_IOCTL


#ifndef MODULE
#define MODULE 1
#endif

#ifndef TRUE
#define TRUE -1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*  The possible media types that can be set in options[] are: */
static const char * const medianame[] = {
	"None",
	"AutoNeg",
	"100baseTxF",
	"100baseTxH",
	"10baseTxF",
	"10baseTxH",
	"FullDuplex",
	"HalfDuplex",
	"100Base",
	"10Base",
};

enum {
	options_None		=0,
	options_AutoNeg		=1,
	options_100baseTxF	=2,
	options_100baseTxH	=3,
	options_10baseTxF	=4,
	options_10baseTxH	=5,
	options_FullDuplex	=6,
	options_HalfDuplex	=7,
	options_100Base		=8,
	options_10Base		=9,
};



//#define SOCLE_MAC_DEBUG

#ifdef SOCLE_MAC_DEBUG
	#define MAC_KERN_INFO		KERN_INFO   
	#define MAC_KERN_DEBUG		KERN_INFO	
	#define MAC_KERN_ERR    	KERN_INFO    
	#define MAC_KERN_WARNING	KERN_INFO 
	#define DEBUG_LEVEL			31
	#define socle_mac_hwcsrshow(x)
	#define socle_mac_desshow(x,y,z)
#else
	#define MAC_KERN_INFO		KERN_INFO   
	#define MAC_KERN_DEBUG		KERN_DEBUG	
	#define MAC_KERN_ERR    	KERN_ERR    
	#define MAC_KERN_WARNING	KERN_WARNING 
	#define DEBUG_LEVEL			1
	#define socle_mac_hwcsrshow(x)
	#define socle_mac_desshow(x,y,z)
#endif
//+arthur 0315
/*#define GPIO_BASE		0xE00A0000  //Virtual base
#define GPIO_PBDR		0x00000008  //phy 1E8A0008
#define GPIO_PBCON		0x0000000C  //phy 1E8A000C
*/
/* Condensed operations for readability. */
#define virt_to_le32desc(addr)  cpu_to_le32(virt_to_phys(addr))
#define le32desc_to_virt(addr)  le32_to_cpu(bus_to_virt(addr))


/* +peter */
/* drivers/net/net_init.c */
/* dev_alloc_skb() */
/*	alloc_etherdev()*/
/* virt_to_le32desc(addr) if no direct mapping*/
/*		if (((u32)skb >> 28) != 0xc) */
/*		printk("skb=0x%x\n",(u32)skb); */

/*skb_reserve(skb, 2); */
/*eth_copy_and_sum(skb, cap->rx_skbuff[entry].skb->tail, pkt_len, 0); */
/*skb_put(skb, pkt_len); */
/* KSEG1ADDR */
struct net_device *DEV=NULL;
#define UP_IRQ_BASE 0
/**************************************************************************************
   Modules parameter	
   The user-configurable values.
   These may be modified when a driver module is loaded.
**************************************************************************************/   
#define	mac_max_dev  MAC_MAX_UNITS;	/* Maximum device for modules install */
#define	mac_active_dev  0				/* The first one intsall device */

//20080416 leonid- for probe modify
/*
static unsigned int mac_amba_base_address[MAC_MAX_UNITS] =
	{
		  CONFIG_SOCLEMAC_BASE_ADDR1
		#if (MAC_MAX_UNITS > 1)		  
		, CONFIG_SOCLEMAC_BASE_ADDR2
		#endif			
	};

static unsigned int mac_amba_irq[MAC_MAX_UNITS] =
	{
		  (CONFIG_SOCLEMAC_INTV1+UP_IRQ_BASE)
		#if (MAC_MAX_UNITS > 1)		  
		, (CONFIG_SOCLEMAC_INTV2+UP_IRQ_BASE)
		#endif	
	};	
*/	
	
static int	mac_debug_level = DEBUG_LEVEL;	/* Message enable: 0..31 = no..all messages. */
static int	max_interrupt_work = 25;		/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
static int	rx_copythreshold = 1518;		/* Set the copy breakpoint for the copy-only-tiny-buffer Rx structure. */
static int	csr0 = 0;						/* csr0 setting value */
static int	multicast_filter_limit = 16;	/* Maximum number of multicast addresses to filter (vs. rx-all-multicast). */
											/* This value does not apply to the 512 bit hash table. */
static int	mtu[MAC_MAX_UNITS] = {0, };		/* Jumbo MTU for interfaces. */


MODULE_AUTHOR("SOCLE_MAC.c: Sherlock Wang at Socle-Tech http://www.socle-tech.com.tw");
MODULE_DESCRIPTION("SOCLE MAC AMBA ethernet driver");
MODULE_LICENSE("GPL");
#ifdef MODULE_PARM_DESC
//MODULE_PARM_DESC(mac_max_dev, 
//				 "SOCLE MAC driver maximum devices to install");
//MODULE_PARM_DESC(mac_active_dev, 
//				 "SOCLE MAC driver install from which one");
//20080416 leonid- for probe modify
/*
MODULE_PARM_DESC(mac_amba_base_address, 
				 "SOCLE MAC driver AMBA base address");
MODULE_PARM_DESC(mac_amba_irq, 
				 "SOCLE MAC driver IRQ number");	
*/
				 			 
MODULE_PARM_DESC(mac_debug_level, 
				 "SOCLE MAC driver message level (0-31)");
MODULE_PARM_DESC(max_interrupt_work,
				 "SOCLE MAC driver maximum events handled per interrupt");
MODULE_PARM_DESC(rx_copythreshold,
				 "SOCLE MAC threshold in bytes for copy-only-tiny-frames");
MODULE_PARM_DESC(csr0,
				 "SOCLE MAC driver CSR0 setting");				 				 
MODULE_PARM_DESC(multicast_filter_limit,
				 "SOCLE MAC threshold for switching to Rx-all-multicast");
MODULE_PARM_DESC(if_speed,
				 "SOCLE MAC force transceiver type or fixed speed+duplex");
MODULE_PARM_DESC(mtu, 
				 "SOCLE MAC Jumbo MTU for interfaces");
int options_if=0;
module_param_named(if_speed, options_if, uint, S_IRUGO);
#endif

/* SOCLE_MAC:
	For the SOCLE_MAC core with AMBA, it may be necessary to change this
	selection based on the CPU used and the hardware design. */
#undef inb
#undef inw
#undef inl
#undef outb
#undef outw
#undef outl
#define inb readb
#define inw readw
#define inl readl
#define outb writeb
#define outw writew
#define outl writel


/*
				Theory of Operation

I. Board Compatibility

This device driver is designed for the Socle MAC IP or SOC Modules.
For SOC Modules, fully test on Socle uDK and LeopartII Platform. 

II. Board-specific settings

Please check the menuconfig and the include file SOLCE_MAC_Driver_Value.c. 
We remove all Board-specific settings & Complier time setting to menuconfig and this file.
The most important setting is 
	- Platform type 
	- System Endian
	- Address Map
	- IRQ Vector
	- Performace Parameter
	- EEPROM
Some boards have EEPROMs mount on MAC modules.  The factory default
is usually "autoselect".  This should only be overridden when using
transceiver connections without link beat e.g. 10base2 or AUI, or (rarely!)
for forcing full-duplex when used with old link partners that do not do
autonegotiation.

III. Driver operation

IIIa. Ring buffers

The Socle MAC can use either ring buffers or lists of Tx and Rx descriptors.
This driver uses statically allocated rings of Rx and Tx descriptors, set at
compile time by RX/TX_RING_SIZE.  This version of the driver allocates skbuffs
for the Rx ring buffers at open() time and passes the skb->data field  as 
receive data buffers.  When an incoming frame is less than rx_copythreshold bytes
long, a fresh skbuff is allocated and the frame is copied to the new skbuff.  
When the incoming frame is larger, the skbuff is passed directly up the protocol 
stack and replaced by a newly allocated skbuff.

The rx_copythreshold value is chosen to trade-off the memory wasted by
using a full-sized skbuff for small frames vs. the copying costs of larger
frames.  For small frames the copying cost is negligible (esp. considering
that we are pre-loading the cache with immediately useful header
information).  For large frames the copying cost is non-trivial, and the
larger copy might flush the cache of useful data.  

IIIC. Synchronization
The driver runs as two independent, single-threaded flows of control.  One
is the send-packet routine, which enforces single-threaded use by the
dev->tbusy flag.  The other thread is the interrupt handler, which is single
threaded by the hardware and other software.

The send packet thread has partial control over the Tx ring and 'dev->tbusy'
flag.  It sets the tbusy flag whenever it is queuing a Tx packet. If the next
queue slot is empty, it clears the tbusy flag when finished otherwise it sets
the 'tp->tx_full' flag.

The interrupt handler has exclusive control over the Rx ring and records stats
from the Tx ring.  (The Tx-done interrupt can not be selectively turned off, so
we cannot avoid the interrupt overhead by having the Tx routine reap the Tx
stats.)	 After reaping the stats, it marks the queue entry as empty by setting
the 'base' to zero.	 If the 'tp->tx_full' flag is set, it clears both the
tx_full and tbusy flags.

IV. Notes
The driver parameter have 3 way to setting
	1.	Load time paramter set
	2.	SROM (EEPROM) Conternt
	3.  Complier time default
the setting priority is 1 > 2 > 3
When load time not issue parameter value, we will try to use EEPROM content. If both 
not issue then will use the default setting.

IVb. References

http://www.socle-tech.com.tw

IVc. Errata

*/

/* The SOCLE MAC Rx and Tx buffer descriptors. */
struct socle_mac_rx_desc {
	s32 status;
	s32 length;
	u32 buffer1, buffer2;				/* We use only buffer 1.  */
};

struct socle_mac_tx_desc {
	s32 status;
	s32 length;
	u32 buffer1, buffer2;				/* We use only buffer 1.  */
};
//leonid+
struct ring_info {
	struct sk_buff	*skb;
	dma_addr_t	mapping;
};

#define PRIV_ALIGN	15	/* Required alignment mask */


/* SOCLE_MAC:
	The SOCLE MAC device driver private struct.
*/
struct socle_mac_private {
	/* Please carefully design the memory. Make sure the DMA access is longword aligned */ 
	
	//leonid- 
// 	struct	socle_mac_rx_desc 	rx_ring_a[RX_RING_SIZE];
// 	struct	socle_mac_tx_desc 	tx_ring_a[TX_RING_SIZE];
//	u16 						setup_frame_a[96];	/* Pseudo-Tx frame to init address table. */

	struct	socle_mac_rx_desc	*rx_ring;
	struct	socle_mac_tx_desc	*tx_ring;
	u16 						*setup_frame;	/* Pseudo-Tx frame to init address table. */

//leonid+	
	dma_addr_t rx_ring_dma;		
	dma_addr_t tx_ring_dma;
	dma_addr_t setup_frame_dma;	
		
//leonid+	
	/* The saved address of a sent-in-place packet/buffer, for skfree(). */
	struct ring_info tx_skbuff[TX_RING_SIZE];
	/* The addresses of receive-in-place skbuffs. */
	struct ring_info rx_skbuff[RX_RING_SIZE];	
	
// 	/* The saved addresses of Rx/Tx-in-place packet buffers. */
// 	struct	sk_buff* 			tx_skbuff[TX_RING_SIZE];
// 	struct	sk_buff* 			rx_skbuff[RX_RING_SIZE];
 	struct	net_device 			*next_module;
	
	/* Multicast filter control. */
	int 						multicast_filter_limit;

	int 						chip_id;
	int 						max_interrupt_work;
	int 						msg_level;
	unsigned int 				csr0;				/* Current CSR0 settings. */
	unsigned int 				csr6;				/* Current CSR6 settings. */
	unsigned int 				csr11;				/* Current CSR11 settings. */
	
	spinlock_t 					lock;

	struct 	net_device_stats 	stats;
			
	/* Note: cache line pairing and isolation of Rx vs. Tx indicies. */
	unsigned int 				cur_rx;				/* Producer ring indices */
	unsigned int 				dirty_rx;			/* consumer ring indices */
	unsigned int 				rx_buf_sz;			/* Based on MTU+slack. */
	int 						rx_copythreshold;

	unsigned int 				cur_tx;
	unsigned int 				dirty_tx;
	
	unsigned int 				tx_full:1;			/* The Tx queue is full. */
	unsigned int 				rx_dead:1;			/* We have no Rx buffers. */
	
	/* Media selection state. */
	unsigned int 				full_duplex:1;		/* Full-duplex operation requested. */
	unsigned int 				full_duplex_lock:1;
	unsigned int 				speed_100M:1;		/* Speed_100M operation requested. */
	unsigned int 				speed_100M_lock:1;	
	
	unsigned int 				medialock:1;		/* Do not sense media type. */
	unsigned int 				mediasense:1;		/* Media sensing in progress. */
	unsigned int 				medialink:1;		/* Media link success. */
	unsigned int 				medianeq:1;			/* Media Auto Neq success. */
	
	struct 	timer_list 			timer;				/* Media selection timer. */

	spinlock_t 					mii_lock;	
	u16 						mii_lpar;			/* MII Link partner ability. */
	u16 						mii_advertise;		/* MII advertise, from SROM table. */
	signed char 				phys;				/* MII device addresses. */

};


/* SOCLE_MAC:
	The function pre-claim.
*/

/********************************** Function Part 			***************************/
static int						socle_mac_open(struct net_device *dev);
static int						socle_mac_start_xmit(struct sk_buff *skb, struct net_device *dev);
static void 					socle_mac_clean_tx_ring(struct net_device *dev);
static void						socle_mac_down(struct net_device *dev);
static int 						socle_mac_close(struct net_device *dev);
static struct net_device_stats 	*socle_mac_get_stats(struct net_device *dev);

#ifdef SOCLE_MAC_HAVE_PRIVATE_IOCTL
static int 						netdev_ethtool_ioctl(struct net_device *dev, void *useraddr);
static int 						private_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
#endif
static void 					set_rx_mode(struct net_device *dev);
static void 					socle_mac_timer(unsigned long data);

/********************************** Call Function Part 		***************************/
static int						socle_mac_start_link(struct net_device *dev);
static int						socle_mac_mii_scan(struct net_device *dev);
static void 					check_media_link(struct net_device *dev, int init);
//0927 arthur
//static void 					wait_rxstop(struct net_device *dev);
static void 					wait_txstop(struct net_device *dev);
static void 					wait_txrxstop(struct net_device *dev);
static void 					start_rxtx(struct net_device *dev);
static void 					restart_rxtx(struct net_device *dev);
static void						updata_csr6_speed_duplex(struct net_device *dev, int init);
static int  					check_speed_duplex(struct net_device *dev, int init);
static inline u32 				ether_crc(int length, unsigned char *data);
static inline u32 				ether_crc_le(int length, unsigned char *data);
static void 					socle_mac_init_ring(struct net_device *dev);
static void 					empty_rings(struct net_device *dev);

/********************************** Interrupt Function Part ***************************/
static irqreturn_t				socle_mac_interrupt(int irq, void *dev_instance, struct pt_regs *regs);
static void 					socle_mac_tx_timeout(struct net_device *dev);
static int 						socle_mac_refill_rx(struct net_device *dev);
static int 						socle_mac_rx(struct net_device *dev);




/* A list of all installed SOCLE MAC devices. */
//static struct net_device *root_socle_mac_dev = NULL;

#include "socle_mdio.c"


/* socle_mac_start_link()
   Start the link, typically called at socle_mac_probe() time but sometimes later with
   multiport cards. */
static int socle_mac_start_link(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);

	// For MII Phy chip section  
	{
		int phy;
		
		/* Find the connected MII xcvrs.
		   Doing this in open() would allow detecting external xcvrs later,
		   but takes much time. */

		phy = socle_mac_mii_scan(dev);
		
		if(phy == MAC_MAX_PHY){
			printk(MAC_KERN_INFO "%s: ***WARNING***: No MII PHY found!\n",
				   dev->name);						   
			cap->phys = MAC_MAX_PHY;
			return FALSE;
		}else{		
			u16 mii_cr;
			u16 mii_advert;	
			u16 mii_status;
			u16 to_advert;

	    		cap->phys = phy;				
			mii_cr = sMacMiiPhyRead(dev, phy, MII_PHY_CR);
			mii_advert = sMacMiiPhyRead(dev, phy, MII_PHY_ANA);	
			mii_status = sMacMiiPhyRead(dev, phy, MII_PHY_SR);
	
		//+arthur rmii
//		printk("socle_mac_start_link %s: MII PHY #%d : config<%4.4x> status<%4.4x> advertising<%4.4x>.\n",
//				   dev->name, phy, mii_cr, mii_status, mii_advert);
				   
			if (cap->mii_advertise)	/* only CSMA support by phy now */
				to_advert = cap->mii_advertise | MII_PHY_ANA_CSMA;
			else					/* Leave unchanged. */
				cap->mii_advertise = to_advert = mii_advert;
			if (mac_debug_level > NETIF_MSG_DRV)
			printk(MAC_KERN_DEBUG "%s:  MII PHY #%d :"
				   "config<%4.4x> status<%4.4x> advertising<%4.4x>.\n",
				   dev->name, phy, mii_cr, mii_status, mii_advert);
				
			if (mii_advert != to_advert) 
			{
				if (mac_debug_level > NETIF_MSG_DRV)
				printk(MAC_KERN_DEBUG "%s:  Advertising %4.4x on PHY %d,"
					   " previously advertising %4.4x.\n",
					   dev->name, to_advert, phy, mii_advert);
				sMacMiiPhyWrite(dev, phy, MII_PHY_ANA, to_advert); 
			}
			if(cap->medialock)
			{
				//Even force mode we still need our partner know so enable the restart
				//auto negotiation 
				u16 mii_mode = 0;
				if(cap->full_duplex)
					mii_mode |= MII_PHY_CR_FDX;
				
				if(cap->speed_100M){	
					mii_mode |= MII_PHY_CR_100M;
//+arthur rmii
#ifdef CONFIG_SOCLE_RMII
					outl(inl(dev->base_addr + CSR_OFFSET(CSR6))|CSR6_SPEED, dev->base_addr + CSR_OFFSET(CSR6));
#endif
				}
#ifdef CONFIG_SOCLE_RMII										
				else{
					outl(inl(dev->base_addr + CSR_OFFSET(CSR6))&(~CSR6_SPEED), dev->base_addr + CSR_OFFSET(CSR6));
				}
#endif					
				printk(MAC_KERN_DEBUG "%s:  MediaLock MII Mode %4.4x on PHY %d\n"
					   ,dev->name, mii_mode, phy);
				/* force MII mode setting */
				sMacMiiPhyWrite(dev, phy, MII_PHY_CR, mii_mode|MII_PHY_CR_RAN); 
			}
			else
			{
				/* Enable autonegotiation */
				sMacMiiPhyWrite(dev, phy, MII_PHY_CR, MII_PHY_CR_AUTO|MII_PHY_CR_RAN); 
			}
			// Because we Re-enable autonegotiation
			// Need a dumy Readback status
			mii_status = sMacMiiPhyRead(dev, phy, MII_PHY_SR);
/*		//+arthur rmii
		printk("socle_mac_start_link %s: MII PHY #%d : config<%4.4x> status<%4.4x> advertising<%4.4x>.\n",
				   dev->name, phy, mii_cr, mii_status, mii_advert);
		//+arthur rmii
		printk("socle_mac_start_link %s: csr6<%8.8x>.\n", dev->name, (int)inl(dev->base_addr + CSR_OFFSET(CSR6)));
*/
		}
	}
	
	return TRUE;
} /* socle_mac_start_link() */

//20080512 leonid+ 
int socle_mac_mii_scan(struct net_device *pDrvCtrl)		/* pointer to device control structure */
{
    u32	phyAddr;
    u32	phy_id0;
    u32	phy_id1;

	/* Find first PHY attached to Socle MAC */
	for (phyAddr = 0; phyAddr < MAC_MAX_PHY; phyAddr++){
		phy_id0 = sMacMiiPhyRead(pDrvCtrl, phyAddr, MII_PHY_ID0);
		MAC_DBG("\nMII ID0<%4.4x> \n",phy_id0);		//channing

	    if (phy_id0 == PHY_ID0){ 
	    	phy_id1 = sMacMiiPhyRead(pDrvCtrl, phyAddr, MII_PHY_ID1);
			MAC_DBG("\nMII ID1<%4.4x> \n",phy_id1);		//channing
	   		if (phy_id1 == PHY_ID1){ 
				if (mac_debug_level > NETIF_MSG_DRV)
				printk(MAC_KERN_DEBUG "%s:  MII PHY #%d : ID0<%4.4x> ID1<%4.4x>\n",
					   pDrvCtrl->name, phyAddr, phy_id0, phy_id1);
			break;
    			}
		}
	}

	return phyAddr;
}

#if 0
void socle_mac_phy_clock_on_test(void)
{	
	static struct net_device *socle_mac;
	struct socle_mac_private *cap;
	int ret;
	u32 cr;

	socle_mac = root_socle_mac_dev;

	while(1){
//		cap = (struct socle_mac_private *)socle_mac->priv;
		cap = netdev_priv(socle_mac);
		
		if(cap->phys == MAC_MAX_PHY){
			ret=socle_mac_mii_scan(socle_mac);
			if (ret == 0){
				//printk("mac phy not off\n");
				return;
			}
		}
		
		cr = sMacMiiPhyRead(socle_mac, cap->phys, MII_PHY_CR);
		sMacMiiPhyWrite(socle_mac, cap->phys, MII_PHY_CR, (cr & ~MII_PHY_CR_OFF));

		cr = sMacMiiPhyRead(socle_mac, cap->phys, MII_PHY_CR);

		//printk("PHY reg0 : %x\n", sMacMiiPhyRead(socle_mac, cap->phy, MII_PHY_CR));
		//printk("mac phy clock on\n");

		if(cap->next_module == NULL)
			break;
		socle_mac = cap->next_module;
	}

	return ;
}

EXPORT_SYMBOL(socle_mac_phy_clock_on_test);

void socle_mac_phy_clock_off_test(void)
{	
	static struct net_device *socle_mac;
	struct socle_mac_private *cap;
	int ret;
	u32 cr;

	socle_mac = root_socle_mac_dev;

	
	while(1){
//		cap = (struct socle_mac_private *)socle_mac->priv;
		cap = netdev_priv(socle_mac);
		
		if(cap->phys == MAC_MAX_PHY){
			ret=socle_mac_mii_scan(socle_mac);
			if (ret == 0){
				//printk("mac phy not off\n");
				return;
			}
		}
		
		cr = sMacMiiPhyRead(socle_mac, cap->phys, MII_PHY_CR);
		sMacMiiPhyWrite(socle_mac, cap->phys, MII_PHY_CR, (cr | MII_PHY_CR_OFF));
		cr = sMacMiiPhyRead(socle_mac, cap->phys, MII_PHY_CR);

		//printk("PHY reg0 : %x\n", sMacMiiPhyRead(socle_mac, cap->phy, MII_PHY_CR));
		//printk("mac phy clock off\n");

		if(cap->next_module == NULL)
			break;
		socle_mac = cap->next_module;
	}

	return ;
}

EXPORT_SYMBOL(socle_mac_phy_clock_off_test);
#endif


/* socle_mac_open()
	Open Driver. The real working function. */
static int
socle_mac_open(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	long ioaddr = dev->base_addr;
	
	int retval;

	if (cap->msg_level & NETIF_MSG_IFUP)
		printk(MAC_KERN_DEBUG "%s: START OPEN --- socle_mac_open().\n",
								dev->name);	
	
	outl(CSR0_SWR, ioaddr + CSR_OFFSET(CSR0));

	/* This would be done after interrupts are initialized, but we do not want
	   to frob the transceiver only to fail later. */
	
	//First Set the triger level of interrupt controller  
	if ( (retval = request_irq(dev->irq, (irq_handler_t)socle_mac_interrupt, IRQF_SHARED, dev->name, dev) ) ) 
	{
		return retval;
	}

	outl(0x00000001, ioaddr + CSR_OFFSET(CSR0)); //+grant
	udelay(100);
	/* Deassert reset.
	   Wait the specified 50 PCI cycles after a reset by initializing
	   Tx and Rx queues and the address filter list. */
	outl(cap->csr0, ioaddr + CSR_OFFSET(CSR0));
	udelay(100);

	if (cap->msg_level & NETIF_MSG_IFUP)
		printk(MAC_KERN_DEBUG "%s: socle_mac_open() irq %d.\n",
										dev->name, dev->irq);

	socle_mac_init_ring(dev);

	/* CSR 3 contains the address ofthe first descriptor in a receive
		descriptor list.  This address should be longword aligned 
		(RLA(1..0)=00+.  */
	/* CSR 4 contains the address of the first descriptor in a transmit
		descriptor list.  This address should be longword aligned 
		(TLA(1..0)=00).  */
	outl(cap->rx_ring_dma, ioaddr + CSR_OFFSET(CSR3));
	outl(cap->tx_ring_dma, ioaddr + CSR_OFFSET(CSR4));

	
	/* Check media link ok */
	check_media_link(dev,1);
	
	set_rx_mode(dev);

	/* Start the Tx to process setup frame. */
	outl(cap->csr6, ioaddr + CSR_OFFSET(CSR6));
	outl(cap->csr6 | CSR6_ST, ioaddr + CSR_OFFSET(CSR6));

	printk(MAC_KERN_DEBUG
		"%s: Enable Interrupt ... Now!!!!!\n",
			dev->name);
			
	/* Clear Interrupt. */
	outl(SOCLE_MAC_INTR, ioaddr + CSR_OFFSET(CSR5));
	/* Enable interrupts by setting the interrupt mask. */
	outl(SOCLE_MAC_INTR, ioaddr + CSR_OFFSET(CSR7));	
	
	outl(cap->csr6 | CSR6_ST | CSR6_SR, ioaddr + CSR_OFFSET(CSR6));
	outl(0, ioaddr + CSR_OFFSET(CSR2));		/* Rx poll demand */

	if (cap->msg_level & NETIF_MSG_IFUP)
		printk(MAC_KERN_DEBUG
		"%s: Done socle_mac_open(), CSR0<%8.8x>, CSR5<%8.8x> CSR6<%8.8x> CSR7<%8.8x>.\n",
			dev->name, (int)inl(ioaddr + CSR_OFFSET(CSR0))
					 , (int)inl(ioaddr + CSR_OFFSET(CSR5))
					 , (int)inl(ioaddr + CSR_OFFSET(CSR6))
					 , (int)inl(ioaddr + CSR_OFFSET(CSR7)));

	/* Set the timer to switch to check for link beat and perhaps switch
	   to an alternate media type. */
	if(cap->medialink && cap->medianeq)
	{
		//Link OK use Slow Timer
		cap->timer.expires = jiffies + LINK_CHECK_SLOW;
	}
	else
	{
		cap->timer.expires = jiffies + LINK_CHECK_FAST;
	}	
	add_timer(&cap->timer);
	
	netif_start_queue(dev);

	/* Show MAC CSR content */
	socle_mac_hwcsrshow(dev);
	
	return 0;
} /* socle_mac_open() */


/* socle_mac_init_ring() */
/*	Initialize the Rx and Tx rings, along with various 'dev' bits. */
static void socle_mac_init_ring(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int i;
	struct	socle_mac_rx_desc	*phypt_rx_ring;
	struct	socle_mac_tx_desc	*phypt_tx_ring;

	cap->rx_dead = cap->tx_full = 0;
	cap->cur_rx = cap->cur_tx = 0;
	cap->dirty_rx = cap->dirty_tx = 0;

	cap->rx_buf_sz = dev->mtu + 18;
	if (cap->rx_buf_sz < PKT_BUF_SZ)
		cap->rx_buf_sz = PKT_BUF_SZ;

	for (i = 0; i < RX_RING_SIZE; i++) 
	{
		phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[i]));
		phypt_rx_ring->status = 0x00000000;
		phypt_rx_ring->length = cpu_to_le32(cap->rx_buf_sz);
		//phypt_rx_ring->buffer2 = virt_to_le32desc(&cap->rx_ring_a[i+1]);
		phypt_rx_ring->buffer2 = cpu_to_le32(cap->rx_ring_dma + sizeof(struct socle_mac_rx_desc) * (i + 1));
		cap->rx_skbuff[i].skb = NULL;
	}
	/* Mark the last entry as wrapping the ring. */
	phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[RX_RING_SIZE-1]));
	phypt_rx_ring->length |= cpu_to_le32(RDESC1_RER);
	//phypt_rx_ring->buffer2 = virt_to_le32desc(&cap->rx_ring_a[0]);
	phypt_rx_ring->buffer2 = cpu_to_le32(cap->rx_ring_dma);

	for (i = 0; i < RX_RING_SIZE; i++) 
	{
		dma_addr_t mapping;	//leonid+
		/* Note the receive buffer must be longword aligned.
		   dev_alloc_skb() provides 16 byte alignment.  But do *not*
		   use skb_reserve() to align the IP header! */
		//-+arthur
		struct sk_buff *skb = dev_alloc_skb(cap->rx_buf_sz);
//		struct sk_buff *skb = alloc_skb(cap->rx_buf_sz, GFP_ATOMIC);
		cap->rx_skbuff[i].skb = skb;
		if (skb == NULL)
			break;
		//leonid+	
		mapping = dma_map_single(NULL, skb->data, skb->len, DMA_FROM_DEVICE);	 	
		cap->rx_skbuff[i].mapping = mapping;
		
			
		skb->dev = dev;			/* Mark as being used by this device. */
		phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[i]));
		phypt_rx_ring->status = cpu_to_le32(RDESC0_OWN);
		cap->rx_ring[i].buffer1 = cpu_to_le32(mapping);	
		//phypt_rx_ring->buffer1 = virt_to_le32desc(skb->tail);
	}
	cap->dirty_rx = (unsigned int)(i - RX_RING_SIZE);

	/* The Tx buffer descriptor is filled in as needed, but we
	   do need to clear the ownership bit. */
	for (i = 0; i < TX_RING_SIZE; i++) 
	{
		phypt_tx_ring = (struct	socle_mac_tx_desc *)(&(cap->tx_ring[i]));
		
		cap->tx_skbuff[i].skb = NULL;
		cap->tx_skbuff[i].mapping = 0;	//leonid
		phypt_tx_ring->status = 0x00000000;
		//phypt_tx_ring->buffer2 = virt_to_le32desc(&cap->tx_ring_a[i+1]);
		phypt_tx_ring->buffer2 = cpu_to_le32(cap->tx_ring_dma + sizeof(struct socle_mac_tx_desc) * (i + 1));
	}
	phypt_tx_ring = (struct	socle_mac_tx_desc *)(&(cap->tx_ring[TX_RING_SIZE-1]));
	//phypt_tx_ring->buffer2 = virt_to_le32desc(&cap->tx_ring_a[0]);
	phypt_tx_ring->buffer2 = cpu_to_le32(cap->tx_ring_dma);
	
} /* socle_mac_init_ring() */

 
/* check_media_link() */
static void check_media_link(struct net_device *dev, int init)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int mii_sr;

	mii_sr = sMacMiiPhyRead(dev, cap->phys, MII_PHY_SR);
	if (cap->msg_level & NETIF_MSG_LINK)
		printk(MAC_KERN_DEBUG "%s: Check MII PHY %d, status %4.4x.\n",
			   dev->name, cap->phys, mii_sr);
	
	if(!(cap->medialink))
	{
		if(mii_sr & MII_PHY_SR_LNK)
		{
			printk(MAC_KERN_INFO "%s: MII PHY %d : Get New Media Link.\n",
			   dev->name, cap->phys);
			cap->medialink = TRUE;
		}	   
	}	
	else
	{
		if(!(mii_sr & MII_PHY_SR_LNK))
		{
			printk(MAC_KERN_INFO "%s: MII PHY %d : Media Link Lost.\n",
			   dev->name, cap->phys);
			cap->medialink = FALSE;
		}	
	}
		
	//Check speed & deuplex for csr6
	if(cap->medialink || init)
		check_speed_duplex(dev, init);
} /* check_media_link() */

/* wait_txstop() */
static void wait_txstop(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	long ioaddr = dev->base_addr;
	int csr6 = inl(ioaddr + CSR_OFFSET(CSR6));
	int csr5;
	
	if(csr6 & CSR6_ST)
	{
		csr6 &= (~(CSR6_ST));
		
		outl(cap->csr6, ioaddr + CSR_OFFSET(CSR6));	
		
		do{
			csr5= inl(ioaddr + CSR_OFFSET(CSR5));
		}
		while((csr5 & CSR5_TS_MSK) != CSR5_TS_ST);
	}
}

/* wait_txrxstop() */
static void wait_txrxstop(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	long ioaddr = dev->base_addr;
	int csr6 = inl(ioaddr + CSR_OFFSET(CSR6));
	int csr5;
	
	if(csr6 & (CSR6_ST|CSR6_SR))
	{
		csr6 &= (~(CSR6_ST|CSR6_SR));
		
		outl(cap->csr6, ioaddr + CSR_OFFSET(CSR6));	
		
		do{
			csr5= inl(ioaddr + CSR_OFFSET(CSR5));
			//Sherlock
			//printk(MAC_KERN_INFO "%s: CSR5<%8.8x> Wait TxRxStop...\n", dev->name, csr5);
		}
		while((csr5 & (CSR5_TS_MSK|CSR5_RS_MSK)) != (CSR5_TS_ST|CSR5_RS_ST));
	}
}

/* start_rxtx() */
static void start_rxtx(struct net_device *dev)
{
	long ioaddr = dev->base_addr;
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	
	outl( (CSR6_ST | CSR6_SR | cap->csr6), ioaddr + CSR_OFFSET(CSR6));	
}

/* restart_rxtx() */
static void restart_rxtx(struct net_device *dev)
{
	wait_txrxstop(dev);
	start_rxtx(dev);
}

/* updata_csr6_speed_duplex() */
static void updata_csr6_speed_duplex(struct net_device *dev, int init)
{
	long ioaddr = dev->base_addr;
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int csr6;
			  
	csr6 = cap->csr6;
	//arthur rmii
//	printk("updata_csr6_speed_duplex %s: old csr6<%8.8x> speed<%d>\n",
//			   dev->name, csr6, cap->speed_100M);
			   
	if(cap->full_duplex)
		cap->csr6 |= CSR6_FD;
	else
		cap->csr6 &= ~CSR6_FD;		//leonid+
	if(cap->speed_100M){
	//	cap->csr6 |= CSR6_TTM;
		cap->csr6 &= (~CSR6_TTM);
//+arthur rmii
#ifdef CONFIG_SOCLE_RMII
		//outl(inl(ioaddr + CSR_OFFSET(CSR0))|CSR6_SPEED, ioaddr + CSR_OFFSET(CSR6));
			cap->csr6 |= CSR6_SPEED;
#endif
	}
#ifdef CONFIG_SOCLE_RMII
	else{
		cap->csr6 |= CSR6_TTM;			//leonid+
		//outl(inl(ioaddr + CSR_OFFSET(CSR0))&(~CSR6_SPEED), ioaddr + CSR_OFFSET(CSR6));
			cap->csr6 &= (~CSR6_SPEED);
	}
#endif
	if( (csr6 != cap->csr6) || (init) )
	{
		printk(MAC_KERN_INFO "%s: Set Media Mode <%s><%s>\n"
			   , dev->name
			   , cap->speed_100M ? "100M" : "10M"
			   , cap->full_duplex ? "Full" : "Half");
		if (mac_debug_level > NETIF_MSG_DRV)
		printk(MAC_KERN_DEBUG "%s: updata opeartion mode old csr6<%8.8x> new csr6<%8.8x>\n",
			   dev->name, csr6, cap->csr6);
			   
		csr6 = inl(ioaddr + CSR_OFFSET(CSR6));
		
		wait_txrxstop(dev);
		
		csr6 = (csr6 & (CSR6_ST | CSR6_SR)) | cap->csr6;
		outl(csr6, ioaddr + CSR_OFFSET(CSR6));		
	}
	//arthur rmii
//	printk("updata_csr6_speed_duplex %s: new csr6<%8.8x>\n",
//		dev->name, (int)inl(ioaddr + CSR_OFFSET(CSR6)));
	
}	/* updata_csr6_speed_duplex() */	


/* check_speed_duplex() */
/*
  Check the MII negotiated duplex, and change the CSR6 setting if
  required.
  Return 0 if everything is OK.
  Return < 0 if the transceiver auot neg is fail.
  */
static int check_speed_duplex(struct net_device *dev, int init)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int mii_sr;
	//arthur rmii
//	printk("check_speed_duplex %s medialock<%d> init<%d>\n", dev->name, cap->medialock, init);
				   
	if(cap->medialock)
	{	
		updata_csr6_speed_duplex(dev, init);
		return 0;
	}
	
	mii_sr = sMacMiiPhyRead(dev, cap->phys, MII_PHY_SR);
	
	if(mii_sr & MII_PHY_SR_AN)
	{
		int mii_anlpa;
		int negotiated;
				
		cap->medianeq = TRUE;
		
		mii_anlpa = sMacMiiPhyRead(dev, cap->phys, MII_PHY_ANLPA);
		negotiated = mii_anlpa & cap->mii_advertise;
		
		if (cap->msg_level & NETIF_MSG_TIMER)
			printk(MAC_KERN_DEBUG "%s: MII link partner %4.4x, negotiated %4.4x.\n",
				   dev->name, mii_anlpa, negotiated);
		
		if (!(cap->speed_100M_lock))
		{
			if(negotiated & MII_PHY_ANA_100M)
				cap->speed_100M = TRUE;
			else
				cap->speed_100M = FALSE;			
		}
		
		if (!(cap->full_duplex_lock))
		{
			if(negotiated & MII_PHY_ANA_FDAM)
				cap->full_duplex = TRUE;
			else
				cap->full_duplex = FALSE;			
		}	
		
		updata_csr6_speed_duplex(dev, init);	
		
		return 0;
	}
	
	return -1;
} /* check_speed_duplex() */



/* The little-endian AUTODIN32 ethernet CRC calculation.
   N.B. Do not use for bulk data, use a table-based routine instead.
   This is common code and should be moved to net/core/crc.c */
static unsigned const ethernet_polynomial_le = 0xedb88320U;
static inline u32 ether_crc_le(int length, unsigned char *data)
{
	u32 crc = 0xffffffff;	/* Initial value. */
	while(--length >= 0) {
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 8; --bit >= 0; current_octet >>= 1) {
			if ((crc ^ current_octet) & 1) {
				crc >>= 1;
				crc ^= ethernet_polynomial_le;
			} else
				crc >>= 1;
		}
	}
	return crc;
}
static unsigned const ethernet_polynomial = 0x04c11db7U;
static inline u32 ether_crc(int length, unsigned char *data)
{
	int crc = -1;

	while(--length >= 0) {
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 0; bit < 8; bit++, current_octet >>= 1)
			crc = (crc << 1) ^
				((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
	}
	return crc;
}

/* set_rx_mode() */
static void set_rx_mode(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	long ioaddr = dev->base_addr;
	int csr6;
	
	csr6 = inl(ioaddr + CSR_OFFSET(CSR6)) 
			& (~(CSR6_RA|CSR6_PM|CSR6_PR|CSR6_IF|CSR6_PB|CSR6_HO|CSR6_HP));

	cap->csr6 = cap->csr6
			&(~(CSR6_RA|CSR6_PM|CSR6_PR|CSR6_IF|CSR6_PB|CSR6_HO|CSR6_HP));
//F200.13.des (2V03H00B01) workaround solution...arthur 0209
//	if (dev->flags & IFF_PROMISC) 
//	{
		if (mac_debug_level > NETIF_MSG_DRV)
		printk(MAC_KERN_DEBUG "%s: Rx Promiscuous mode enabled.\n", dev->name);			
		/* Set promiscuous. */		
		cap->csr6 |= CSR6_PM | CSR6_PR;
		csr6 |= CSR6_PM | CSR6_PR;
		/* Unconditionally log net taps. */		

	outl(csr6, ioaddr + CSR_OFFSET(CSR6));
} /* set_rx_mode() */

/* socle_mac_timer() */
/* 2 Function control by this mac timer handle
	1. Check Mii status for Link & Auot neq
	2. Tx timeout check
*/
static void socle_mac_timer(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
//	unsigned long timeout;
	
	/* Check Mii status for Link & Auot neq */
	if (cap->msg_level & NETIF_MSG_TIMER)
		printk(MAC_KERN_DEBUG "%s: Media selection tick\n",
			   dev->name);
			   
	check_media_link(dev, 0);	

	if(cap->medialink && cap->medianeq)
	{
		//Link OK use Slow Timer
		cap->timer.expires = jiffies + LINK_CHECK_SLOW;
//+arthur 0318		timeout= jiffies + LINK_CHECK_SLOW;
	}
	else
	{
		//Link Down use Fast Timer
		cap->timer.expires = jiffies + LINK_CHECK_FAST;
//+arthur0318		timeout= jiffies + LINK_CHECK_FAST;
	}
	add_timer(&cap->timer);
//+arthur0318	mod_timer(&cap->timer,timeout);
} /* socle_mac_timer() */


/* socle_mac_start_xmit() */
static int
socle_mac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int entry, q_used_cnt;
	unsigned long eflags_irqsave;
	u32 flag;	
	struct	socle_mac_tx_desc	*phypt_tx_ring;
	dma_addr_t mapping;
	
	if (cap->msg_level & NETIF_MSG_TX_QUEUED)
		printk(MAC_KERN_DEBUG "%s: socle_mac_start_xmit ... go\n",
			   dev->name);

	/* Caution: the write order is important here, set the field
	   with the ownership bits last. */
	
	spin_lock_irqsave(&cap->lock, eflags_irqsave);
		
	/* Calculate the next Tx descriptor entry. */
	entry = cap->cur_tx % TX_RING_SIZE;
	q_used_cnt = cap->cur_tx - cap->dirty_tx;
	phypt_tx_ring = (struct	socle_mac_tx_desc *)(&(cap->tx_ring[entry]));
//leonid+
	cap->tx_skbuff[entry].skb = skb;
	mapping = dma_map_single(NULL, skb->data, skb->len, DMA_TO_DEVICE);
	cap->tx_skbuff[entry].mapping = mapping;
	cap->tx_ring[entry].buffer1 = cpu_to_le32(mapping);
	
	
// 	cap->tx_skbuff[entry].skb = skb;
// 	phypt_tx_ring->buffer1 = virt_to_le32desc(skb->data);
	
// #ifdef LINUX26
// 	dmac_clean_range((unsigned long)skb->data,(unsigned long)(skb->data+2048));
// #endif

	if (q_used_cnt < TX_QUEUE_LEN/2) 
	{/* Typical path */
		flag = TDESC1_LS|TDESC1_FS; /* No interrupt */
	} 
	else if (q_used_cnt == TX_QUEUE_LEN/2) 
	{
		flag = TDESC1_IC|TDESC1_LS|TDESC1_FS; /* Tx-done intr. */
	} 
	else if (q_used_cnt < TX_QUEUE_LEN) 
	{
		flag = TDESC1_LS|TDESC1_FS; /* No Tx-done intr. */
	} 
	else 
	{		/* Leave room for set_rx_mode() to fill entries. */
		cap->tx_full = 1;
		flag = TDESC1_IC|TDESC1_LS|TDESC1_FS; /* Tx-done intr. */
		netif_stop_queue(dev);		//20100622 leonid+ for tx overwrite issue
	}
	
	if (entry == TX_RING_SIZE-1)
		flag = TDESC1_IC|TDESC1_LS|TDESC1_FS|TDESC1_TER;

	phypt_tx_ring->length = cpu_to_le32(skb->len | flag);
	phypt_tx_ring->status = cpu_to_le32(TDESC0_OWN);
	cap->cur_tx++;

	dev->trans_start = jiffies;
	
	/* Show MAC DES content */
	socle_mac_desshow(dev, 1, (int *)(phypt_tx_ring));
	
	/* Trigger an immediate transmit demand. */
	outl(1, dev->base_addr + CSR_OFFSET(CSR1));
	
	spin_unlock_irqrestore(&cap->lock, eflags_irqsave);
	
	return 0;
} /* socle_mac_start_xmit() */


/* empty_rings() */
static void empty_rings(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int i;
	struct	socle_mac_rx_desc	*phypt_rx_ring;
	
	/* Free all the skbuffs in the Rx queue. */
	for (i = 0; i < RX_RING_SIZE; i++) 
	{
		phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[i]));		
		phypt_rx_ring->status = 0;		/* Not owned. */
		phypt_rx_ring->length = 0;
		phypt_rx_ring->buffer1 = cpu_to_le32(0xBADF00D0); /* An invalid address. */
		
		if (cap->rx_skbuff[i].skb) {
			dma_unmap_single(NULL, cap->rx_skbuff[i].mapping, cap->rx_skbuff[i].skb->len, DMA_FROM_DEVICE);	//leonid+
			dev_kfree_skb(cap->rx_skbuff[i].skb);
		}
		cap->rx_skbuff[i].skb = NULL;		
	}
	for (i = 0; i < TX_RING_SIZE; i++) 
	{
		if (cap->tx_skbuff[i].skb) {
			dma_unmap_single(NULL, cap->tx_skbuff[i].mapping, cap->tx_skbuff[i].skb->len, DMA_TO_DEVICE);	//leonid+
			dev_kfree_skb(cap->tx_skbuff[i].skb);
		}
		cap->tx_skbuff[i].skb = NULL;
	}
} /* empty_rings() */

static void socle_mac_clean_tx_ring(struct net_device *dev)
{
//	struct socle_mac_private	*cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	struct socle_mac_tx_desc	*phypt_tx_ring;
	unsigned int status;
	
	for ( ; cap->cur_tx - cap->dirty_tx > 0; cap->dirty_tx++) 
	{
		int entry = cap->dirty_tx % TX_RING_SIZE;
		phypt_tx_ring = (struct	socle_mac_tx_desc *)(&(cap->tx_ring[entry]));	//leonid c
		status = le32_to_cpu(phypt_tx_ring->status);
		
		if (status & TDESC0_OWN){
			cap->stats.tx_errors++;	/* It wasn't Txed */
			phypt_tx_ring->status = 0;
		}		
		
		/* Check for Tx filter setup frames. */
		if (cap->tx_skbuff[entry].skb == NULL) {
			/* test because dummy frames not mapped */
			if (cap->tx_skbuff[entry].mapping){
				dma_unmap_single(NULL, cap->tx_skbuff[entry].mapping, 
						cap->tx_skbuff[entry].skb->len, DMA_TO_DEVICE);
				cap->tx_skbuff[entry].mapping = 0;
			}
			continue;
		}

		dma_unmap_single(NULL, cap->tx_skbuff[entry].mapping, 
				cap->tx_skbuff[entry].skb->len, DMA_TO_DEVICE);

		/* Free the original skb. */
			dev_kfree_skb_any(cap->tx_skbuff[entry].skb);
			cap->tx_skbuff[entry].skb = NULL;
			cap->tx_skbuff[entry].mapping = 0;		
	}
}

static void socle_mac_down (struct net_device *dev)
{
	long ioaddr = dev->base_addr;
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	unsigned long flags;

	del_timer_sync (&cap->timer);

	spin_lock_irqsave (&cap->lock, flags);

	/* Disable interrupts by clearing the interrupt mask. */
	outl (0x00000000, ioaddr + CSR_OFFSET(CSR7));

	/* Stop the Tx and Rx processes. */
	wait_txrxstop(dev);

	/* prepare receive buffers */
	socle_mac_refill_rx(dev);

	/* release any unconsumed transmit buffers */
	socle_mac_clean_tx_ring(dev);
	
	if (inl (ioaddr + CSR_OFFSET(CSR6)) != 0xffffffff)
		cap->stats.rx_missed_errors += inl (ioaddr + CSR_OFFSET(CSR8)) & 0xffff;

	spin_unlock_irqrestore (&cap->lock, flags);

	init_timer(&cap->timer);
	cap->timer.data = (unsigned long)dev;
	cap->timer.function = socle_mac_timer;
}

/* socle_mac_close() */
static int socle_mac_close(struct net_device *dev)
{
	long ioaddr = dev->base_addr;
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);

	if (cap->msg_level & NETIF_MSG_IFUP)
		printk(MAC_KERN_DEBUG "%s: START CLOSE --- socle_mac_close().\n",
								dev->name);	
								
	netif_stop_queue(dev);

	socle_mac_down (dev);
	
	free_irq(dev->irq, dev);

	empty_rings(dev);
#ifdef LINUX24
	MOD_DEC_USE_COUNT;	
#endif
	if (cap->msg_level & NETIF_MSG_IFDOWN)
		printk(MAC_KERN_DEBUG "%s: SHUTTING DOWN ethercard, status was %8.8x.\n",
			   dev->name, (int)inl(ioaddr + CSR_OFFSET(CSR5)));

	return 0;
} /* socle_mac_close() */

static struct net_device_stats *socle_mac_get_stats(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	long ioaddr = dev->base_addr;
		
	if (netif_running(dev))
	{		
		unsigned long flags;
		int csr8;
				
		spin_lock_irqsave (&cap->lock, flags);
		
		csr8 = inl(ioaddr + CSR_OFFSET(CSR8));
		cap->stats.rx_missed_errors += (csr8 & CSR8_MFC_MSK);
		
		spin_unlock_irqrestore(&cap->lock, flags);
	}		
	return &cap->stats;
}

#ifdef HAVE_PRIVATE_IOCTL
static int netdev_ethtool_ioctl(struct net_device *dev, void *useraddr)
{
	u32 ethcmd;

	if (copy_from_user(&ethcmd, useraddr, sizeof(ethcmd)))
		return -EFAULT;

    switch (ethcmd) 
    {
        case ETHTOOL_GDRVINFO: 
        {
			struct ethtool_drvinfo info = {ETHTOOL_GDRVINFO};
			strcpy(info.driver, DRV_NAME);
			strcpy(info.version, DRV_VERSION);
			strcpy(info.bus_info, DRV_BUSMODE);
			if (copy_to_user(useraddr, &info, sizeof(info)))
				return -EFAULT;
			return 0;
		}
    }

	return -EOPNOTSUPP;
}

/* private_ioctl() */
/* Provide ioctl() calls to examine the MII xcvr state.
   The two numeric constants are because some clueless person
   changed value for the symbolic name.
 */
static int private_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
//   struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	u16 *data = (u16 *)&rq->ifr_data;
	u32 *data32 = (void *)&rq->ifr_data;
	unsigned int phy = cap->phys;
	unsigned int regnum = data[1];

	switch(cmd) {
	case SIOCETHTOOL:			/* Ethtool interface		*/
		return netdev_ethtool_ioctl(dev, (void *) rq->ifr_data);
			
	case SIOCGMIIPHY:			/* Get address of MII PHY in use. */
	case SIOCDEVPRIVATE:		/* for binary compat */
		/* SIOCGMIIPHY: Get the address of the PHY in use. */
		data[0] = phy;		
	case SIOCGMIIREG:			/* Read MII PHY register. */
	case SIOCDEVPRIVATE+1:		/* for binary compat */
		 printk("Dump MII PHY register: PHY = %d data=%d %d %d\n", phy, data[0], data[1], data[2]);
                /* SIOCGMIIREG: Read the specified MII register. */
                if(phy > 31)
                        return -ENODEV;
                if (regnum & ~0x1f)
                        return -EINVAL;
                printk("MII_PHY_REG%d = %08x\n", regnum, sMacMiiPhyRead(dev, phy, regnum));
		return 0;
	case SIOCSMIIREG:			/* Write MII PHY register. */
	case SIOCDEVPRIVATE+2:		/* for binary compat */
		/* SIOCSMIIREG: Write the specified MII register */
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (regnum & ~0x1f)
			return -EINVAL;
		if(phy > 31)
			return -ENODEV;
			
		if (data[0] == phy) 
		{
			u16 value = data[2];
			switch (regnum) 
			{
				case MII_PHY_CR: /* Check for autonegotiation on or reset. */
					if( !(value & (MII_PHY_CR_RESET|MII_PHY_CR_AUTO)) )
					//No Auto Neg & No Reset is force the speed and duplex
					{
						cap->full_duplex_lock = 1;
						cap->speed_100M_lock = 1;
					}					
					if (cap->full_duplex_lock)
						cap->full_duplex = (value & MII_PHY_CR_FDX) ? 1 : 0;
					if (cap->speed_100M_lock)
						cap->speed_100M = (value & MII_PHY_CR_100M) ? 1 : 0;
				break;
				case MII_PHY_ANA: 
					cap->mii_advertise = data[2]; 
				break;
			}
		}
		sMacMiiPhyWrite(dev, data[0], regnum, data[2]);		
		return 0;
		
	case SIOCGIFMAP:
		data32[0] = cap->msg_level;
		data32[1] = cap->multicast_filter_limit;
		data32[2] = cap->max_interrupt_work;
		data32[3] = cap->rx_copythreshold;
		return 0;
	case SIOCSIFMAP:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		cap->msg_level = data32[0];
		cap->multicast_filter_limit = data32[1];
		cap->max_interrupt_work = data32[2];
		cap->rx_copythreshold = data32[3];
		return 0;
	default:
		return -EOPNOTSUPP;
	}

	return -EOPNOTSUPP;
} /* private_ioctl() */
#endif  /* HAVE_PRIVATE_IOCTL */

/* socle_mac_tx_timeout() */
static void socle_mac_tx_timeout(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;	
	struct socle_mac_private *cap = netdev_priv(dev);
	long ioaddr = dev->base_addr;
	unsigned long eflags_irqsave;	
	
	spin_lock_irqsave(&cap->lock, eflags_irqsave);

#ifndef	FINAL_RELEASE_VERSION
	if (cap->msg_level & NETIF_MSG_TX_ERR) 
	{
		int i;
		for (i = 0; i < RX_RING_SIZE; i++) 
		{
			u8 *buf;
			int j;
			struct	socle_mac_rx_desc	*phypt_rx_ring;
			
			//+peter
			phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[i]));
//			buf = (u8 *)KSEG1ADDR(le32desc_to_virt(phypt_rx_ring->buffer1));
//			buf = (u8 *)(le32desc_to_virt(phypt_rx_ring->buffer1));
			buf = (u8 *)(phypt_rx_ring->buffer1);
			
			printk(MAC_KERN_DEBUG "%2d: %8.8x %8.8x %8.8x %8.8x  "
				   "%2.2x %2.2x %2.2x.\n",
				   i, (unsigned int)le32_to_cpu(phypt_rx_ring->status),
				   (unsigned int)le32_to_cpu(phypt_rx_ring->length),
				   (unsigned int)le32_to_cpu(phypt_rx_ring->buffer1),
				   (unsigned int)le32_to_cpu(phypt_rx_ring->buffer2),
				   buf[0], buf[1], buf[2]);
			for (j = 0; buf[j] != 0xee && j < 1600; j++)
				if (j < 100) printk(KERN_DEBUG " %2.2x", buf[j]);
				
			printk(KERN_DEBUG " j=%d.\n", j);
		}
		
		printk(MAC_KERN_DEBUG "  Rx ring %8.8x: ", (int)cap->rx_ring);
		for (i = 0; i < RX_RING_SIZE; i++)
		{
			struct	socle_mac_rx_desc	*phypt_rx_ring;
			phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[i]));
			
			printk(KERN_DEBUG " %8.8x", (unsigned int)le32_to_cpu(phypt_rx_ring->status));
		}
			
		printk("\n" MAC_KERN_DEBUG "  Tx ring %8.8x: ", (int)cap->tx_ring);
		for (i = 0; i < TX_RING_SIZE; i++)
		{
			struct	socle_mac_tx_desc	*phypt_tx_ring;
			phypt_tx_ring = (struct	socle_mac_tx_desc *)(&(cap->tx_ring[i]));
			
			printk(KERN_DEBUG " %8.8x", (unsigned int)le32_to_cpu(phypt_tx_ring->status));
		}	
		printk("\n");
	}
#endif
	check_media_link(dev, 0);

	cap->stats.tx_errors++;

	restart_rxtx(dev);
	/* Trigger an immediate transmit demand. */
	outl(1, ioaddr + CSR_OFFSET(CSR1));	
	dev->trans_start = jiffies;
	
	netif_wake_queue(dev);
				
	spin_unlock_irqrestore(&cap->lock, eflags_irqsave);
	
	return;
} /* socle_mac_tx_timeout() */

extern void cache_invalidate(void);

/* socle_mac_interrupt() */
/* The interrupt handler does all of the Rx thread work and cleans up
   after the Tx thread. */
irqreturn_t socle_mac_interrupt(int irq, void *dev_instance, struct pt_regs *regs)
{
	struct net_device *dev = (struct net_device *)dev_instance;
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	long ioaddr = dev->base_addr;
	int work_budget = cap->max_interrupt_work;
	int csr5 = 0;
	DEV=dev;
	
	csr5 = inl(ioaddr + CSR_OFFSET(CSR5));

	if((csr5 & CSR5_ALLINT_MASK) == 0) 
		return IRQ_NONE;

//+arthur for testing 0315
/*	u32 tmp;
	u32 msk;
	tmp = inl(GPIO_BASE+GPIO_PBDR);
	msk = (tmp&0x20);
	outl( msk ? (tmp & (~ msk)) : (tmp |0x20 ), GPIO_BASE+GPIO_PBDR);
*/	
	do {		
		//+arthur
//	if (irq == 10)
//		printk("%s:interrupt coming csr5=%#8.8x!\n", dev->name, csr5);
#ifdef CONFIG_SOCLE_CACHE
	cache_invalidate();
#endif
				   
		if ((csr5 & (CSR5_NIS|CSR5_AIS)) == 0){
			/* Acknowledge all of the current interrupt sources ASAP. */
			outl(csr5 , ioaddr + CSR_OFFSET(CSR5));
			printk(MAC_KERN_WARNING "%s: WARNING !!!!!!!!!!!!!!!!!!! NO NIS AIS interrupt  csr5=%#8.8x new csr5=%#8.8x!\n",
				   dev->name, csr5, (int)inl(dev->base_addr + CSR_OFFSET(CSR5)));
			break;
		}
		
		/* Acknowledge all of the current interrupt sources ASAP. */
		outl(csr5 , ioaddr + CSR_OFFSET(CSR5));
		
		if (cap->msg_level & NETIF_MSG_INTR)
			printk(MAC_KERN_DEBUG "%s: interrupt  csr5=%#8.8x new csr5=%#8.8x.\n",
				   dev->name, csr5, (int)inl(dev->base_addr + CSR_OFFSET(CSR5)));
		
		if (csr5 & (CSR5_RI | CSR5_RU)){
			work_budget -= socle_mac_rx(dev);
			socle_mac_refill_rx(dev);
		}
		
		if (csr5 & (CSR5_TU | CSR5_TPS | CSR5_TI)) 
		{
			unsigned int dirty_tx;
			
			spin_lock(&cap->lock);
			
			for (dirty_tx = cap->dirty_tx; cap->cur_tx - dirty_tx > 0; dirty_tx++) 
			{
				int entry = dirty_tx % TX_RING_SIZE;
				int status;
				struct	socle_mac_tx_desc	*phypt_tx_ring;
				
				phypt_tx_ring = (struct	socle_mac_tx_desc *)(&(cap->tx_ring[entry]));
				status = le32_to_cpu(phypt_tx_ring->status);

				if (status & TDESC0_OWN)
					break;			/* It still has not been Txed */
				
				if (cap->full_duplex)
				{
					/* Full Duplex don't check the error cause by collision */
					status = status & (~(TDESC0_LO | TDESC0_NC | TDESC0_ES | TDESC0_LC | TDESC0_EC | TDESC0_DE | TDESC0_CC_MSK));
					if(status)
						status = status | TDESC0_ES;
				}		 
					
				/* Skip the Rx filter setup frames. */
				if (cap->tx_skbuff[entry].skb == NULL)
				  continue;

				if (status & TDESC0_ES) 
				{
					/* There was an major error, log it. */
					if (cap->msg_level & NETIF_MSG_TX_ERR)
						printk(MAC_KERN_DEBUG "%s: Transmit error, Tx status<%8.8x>.\n",
							   dev->name, status);
					cap->stats.tx_errors++;
					if(cap->speed_100M == FALSE)
						udelay(100);
					if (status & (TDESC0_EC)) 			cap->stats.tx_aborted_errors++;
					if (status & (TDESC0_LO|TDESC0_NC)) cap->stats.tx_carrier_errors++;
					if (status & (TDESC0_LC)) 			cap->stats.tx_window_errors++;
					if (status & (TDESC0_UF)) 			cap->stats.tx_fifo_errors++;
#ifdef ETHER_STATS
					if (status & (TDESC0_EC)) 			cap->stats.collisions16++;
#endif
				} 				
				else 
				{
					if (cap->msg_level & NETIF_MSG_TX_DONE)
						printk(MAC_KERN_DEBUG "%s: Transmit complete, status<%8.8x>.\n", dev->name, status);
#ifdef ETHER_STATS
					if (status & TDESC0_DE) 			cap->stats.tx_deferred++;
#endif
					cap->stats.tx_bytes += cap->tx_skbuff[entry].skb->len;
					cap->stats.collisions += TDESC0_CC_GET(status);
					cap->stats.tx_packets++;
				}
					
				/* Free the original skb. */
				dev_kfree_skb_irq(cap->tx_skbuff[entry].skb);
				cap->tx_skbuff[entry].skb = NULL;
				if (cap->tx_skbuff[entry].mapping)
					dma_unmap_single(NULL, cap->tx_skbuff[entry].mapping, cap->tx_skbuff[entry].skb->len, DMA_TO_DEVICE);	//leonid+				
			}

#ifndef FINAL_RELEASE_VERSION
			if (cap->cur_tx - dirty_tx > TX_RING_SIZE) 
			{
				printk(MAC_KERN_ERR "%s: Out-of-sync dirty pointer, %d vs. %d, full=%d.\n",
					   dev->name, dirty_tx, cap->cur_tx, cap->tx_full);
				dirty_tx += TX_RING_SIZE;
			}
#endif

			if (cap->tx_full && cap->cur_tx - dirty_tx  < TX_QUEUE_LEN - 4) 
			{
				/* The ring is no longer full, clear tbusy. */
				cap->tx_full = 0;
				netif_wake_queue(dev);
			}

			cap->dirty_tx = dirty_tx;
			
			spin_unlock(&cap->lock);
		}
		

		if (cap->rx_dead) 
		{
			socle_mac_rx(dev);
			socle_mac_refill_rx(dev);
			if (cap->cur_rx - cap->dirty_rx < RX_RING_SIZE - 3) 
			{
			/* +grant
			 * 	printk(MAC_KERN_ERR "%s: Restarted Rx at %d / %d.\n",
					   dev->name, cap->cur_rx, cap->dirty_rx);
			 */		   
				outl(0, ioaddr + CSR_OFFSET(CSR2));		/* Rx poll demand   */
				start_rxtx(dev); //+grant
				cap->rx_dead = 0;
			}
		}

		/* Log errors. */
		if (csr5 & CSR5_AIS) 
		{	/* Abnormal error summary bit. */
			if (csr5 == 0xffffffff)
				break;
				
			if (csr5 & CSR5_UNF) 
			{
				if ((cap->csr6 & CSR6_TR_MSK) != CSR6_TR_MSK)
					cap->csr6 += CSR6_TR_STEP;	/* Bump up the Tx threshold */
				else
					cap->csr6 |= CSR6_SF;		/* Store-n-forward. */
					
				if (cap->msg_level & NETIF_MSG_TX_ERR)
					printk(MAC_KERN_WARNING "%s: Tx threshold increased, "
						   "new CSR6<%8.8x>.\n", dev->name, cap->csr6);
				//+grant
				restart_rxtx(dev);
				outl(0, ioaddr + CSR_OFFSET(CSR1));
			}
			
			if (csr5 & CSR5_TPS) 
			{
				/* This is normal when changing Tx modes. */
				if (cap->msg_level & NETIF_MSG_LINK)
					printk(MAC_KERN_WARNING "%s: The transmitter stopped."
						   "  CSR5 is<%8.8x>, CSR6<%8.8x>, new CSR6<%8.8x>.\n",
						   dev->name, csr5, (int)inl(ioaddr + CSR_OFFSET(CSR6)), cap->csr6);
			}
			
		//	if (csr5 & CSR5_UNF) 
		//	{
				/* Restart the transmit process. */
		//		restart_rxtx(dev);
		//		outl(0, ioaddr + CSR_OFFSET(CSR1));
		//	}
	
			if (csr5 & (CSR5_RPS | CSR5_RU)) 
			{
				/* Missed a Rx frame or mode change. */
				cap->stats.rx_missed_errors += inl(ioaddr + CSR_OFFSET(CSR8)) & CSR8_MFC_MSK;
				socle_mac_rx(dev);
				socle_mac_refill_rx(dev);
				outl(0, ioaddr + CSR_OFFSET(CSR2)); //+grant

				if (csr5 & CSR5_RU)
				{
					cap->rx_dead = 1;
					
				}
			}
			
			if (csr5 & CSR5_GPTE) 
			{
				if (cap->msg_level & NETIF_MSG_INTR)
					printk(MAC_KERN_ERR "%s: MAC Embedded Timer Interrupt.\n",
						   dev->name);
			}
		}
		
		cap->csr11 = (0x8b240000|( cap->csr11 & (~(CSR11_TT_MSK|CSR11_NTP_MSK|CSR11_RT_MSK|CSR11_NRP_MSK))));
		outl(cap->csr11, ioaddr + CSR_OFFSET(CSR11));

		csr5 = inl(ioaddr + CSR5);		
	} while ((csr5 & CSR5_ALLINT_MASK) != 0);
	socle_mac_refill_rx(dev); //+grant
	
	if (cap->msg_level & NETIF_MSG_INTR)
		printk(MAC_KERN_DEBUG "%s: exiting interrupt, csr5=%#4.4x.\n",
			   dev->name, (int)inl(ioaddr + CSR_OFFSET(CSR5)) );

	return IRQ_HANDLED;
} /* socle_mac_amba_interrupt() */


/* Refill the Rx ring buffers. */
static int socle_mac_refill_rx(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int entry;
	int work_done = 0;	
	struct	socle_mac_rx_desc	*phypt_rx_ring;
	
	for (; cap->cur_rx - cap->dirty_rx > 0; cap->dirty_rx++) 
	{
		entry = cap->dirty_rx % RX_RING_SIZE;
		phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[entry]));
		
		if (cap->rx_skbuff[entry].skb == NULL) 
		{
			struct sk_buff *skb;
			//-+arthur
			skb = cap->rx_skbuff[entry].skb = dev_alloc_skb(cap->rx_buf_sz);
//			skb = cap->rx_skbuff[entry].skb = alloc_skb(cap->rx_buf_sz, GFP_ATOMIC);
			if (skb == NULL) 
			{
				if (cap->cur_rx - cap->dirty_rx == RX_RING_SIZE)
					printk(MAC_KERN_ERR "%s: No kernel memory to allocate "
						   "receive buffers.\n", dev->name);
				break;
			}
			skb->dev = dev;			/* Mark as being used by this device. */
			phypt_rx_ring->buffer1 = virt_to_le32desc(skb->tail);
			work_done++;
		}
		phypt_rx_ring->status = cpu_to_le32(RDESC0_OWN);
	}

	return work_done;
}

/* socle_mac_rx() */
static int socle_mac_rx(struct net_device *dev)
{
//	struct socle_mac_private *cap = (struct socle_mac_private *)dev->priv;
	struct socle_mac_private *cap = netdev_priv(dev);
	int entry = cap->cur_rx % RX_RING_SIZE;
	int rx_work_limit = cap->dirty_rx + RX_RING_SIZE - cap->cur_rx;
	int work_done = 0;	
	s32 status;
	long ioaddr = dev->base_addr;
	struct	socle_mac_rx_desc	*phypt_rx_ring;
			   
	/* If we own the next entry, it is a new packet. Send it up. */
	phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[entry]));
	status = le32_to_cpu(phypt_rx_ring->status);
	
	while ( ! (status & RDESC0_OWN) ) 
	{
		if (cap->msg_level & NETIF_MSG_RX_STATUS)
			printk(MAC_KERN_DEBUG "%s: In socle_mac_rx(), entry %d status<%8.8x>.\n",
				   dev->name, entry, status);
		
		socle_mac_desshow(dev, 0, (int *)(phypt_rx_ring));
				   
		if (--rx_work_limit < 0){
			break;
		}
		//HW Bug of ES not include CS TL
		if (cap->full_duplex){
			if((status & (RDESC0_CE|RDESC0_TL|RDESC0_RF|RDESC0_DE)) == 0){ 
				status &= ~(RDESC0_ES|RDESC0_CS);
			}else
				status &= ~RDESC0_CS;
			if( status & (RDESC0_TL) )		//20080428 leonid+ for full duplex ignore collision bit
				status |= RDESC0_ES;
		}else{
			if( status & (RDESC0_TL) )
				status |= RDESC0_ES;
		}
			
		if ( (status & (RDESC0_ES|RDESC0_FD|RDESC0_LS)) != (RDESC0_FD|RDESC0_LS) ) 
		{
			if ( (status & (RDESC0_FD|RDESC0_LS)) != (RDESC0_FD|RDESC0_LS) )
			{
				/* Ingore earlier buffers. */
				if ((status & 0xffff) != 0x7fff) 
				{
					if (cap->msg_level & NETIF_MSG_RX_ERR)
						printk(MAC_KERN_WARNING "%s: Oversized Ethernet frame "
							   "spanned multiple buffers, status<%8.8x>!\n",
							   dev->name, status);
					cap->stats.rx_length_errors++;
				}
			} 
			else if (status & RDESC0_ES) 
			{
				/* There was a fatal error. */
				if (cap->msg_level & NETIF_MSG_RX_ERR)
					printk(MAC_KERN_DEBUG "%s: Receive error, Rx status<%8.8x>.\n",
						   dev->name, status);
						   
				cap->stats.rx_errors++; /* end of a packet.*/
				if (status & (RDESC0_RF|RDESC0_TL)) 	cap->stats.rx_length_errors++;
				//Dribbling bit error --- workaround soultion 0307 +arthur
				if (status & (RDESC0_DB)){
					printk("Dribbling bit error.....\n");
					cap->stats.rx_frame_errors++;
					//wait_txrxstop(dev);
					wait_txstop(dev);	//wait tx process stop
					udelay(100);		//leonid+ 20100505
					
					outl(CSR0_SWR, ioaddr + CSR_OFFSET(CSR0));		
					udelay(100);
//					outl(cap->csr0, ioaddr + CSR_OFFSET(CSR0));
//					udelay(100);
					
					socle_mac_init_ring(dev);
					//outl(virt_to_phys(cap->rx_ring_a), ioaddr + CSR_OFFSET(CSR3));
					outl(cap->rx_ring_dma, ioaddr + CSR_OFFSET(CSR3));
					outl(cap->tx_ring_dma, ioaddr + CSR_OFFSET(CSR4));
					/* Check media link ok */
					check_media_link(dev,1);
					set_rx_mode(dev);
					/* Start the Tx to process setup frame. */
					outl(cap->csr6, ioaddr + CSR_OFFSET(CSR6));
					outl(cap->csr6 | CSR6_ST, ioaddr + CSR_OFFSET(CSR6));

					/* Clear Interrupt. */
					outl(SOCLE_MAC_INTR, ioaddr + CSR_OFFSET(CSR5));
					/* Enable interrupts by setting the interrupt mask. */
					outl(SOCLE_MAC_INTR, ioaddr + CSR_OFFSET(CSR7));	
	
					outl(cap->csr6 | CSR6_ST | CSR6_SR, ioaddr + CSR_OFFSET(CSR6));
					outl(0, ioaddr + CSR_OFFSET(CSR2));		/* Rx poll demand */

					netif_start_queue(dev);
					cap->cur_rx++;
					work_done++;
					return 0;
				}
				if (status & (RDESC0_CE)) 		cap->stats.rx_crc_errors++;
				if (status & (RDESC0_FIFOERR)) 			
				{
					long ioaddr = dev->base_addr;
					
					cap->stats.rx_fifo_errors++;
					// SOCLE MAC when FIFO error must re-start Rx
					outl(0, ioaddr + CSR_OFFSET(CSR2));		/* Rx poll demand */
					//Sherlock 
					printk(MAC_KERN_INFO "%s: Warnning:Receive FIFO error!!!!!!!!!\n",
						   dev->name);
				}
			}
		} 
		else 
		{
			/* Omit the four octet CRC from the length. */
			short pkt_len = RDESC0_FL_GET(status) - 4;
			struct sk_buff *skb;
			long ioaddr = dev->base_addr;	//+arthur 0321
			
#ifndef FINAL_RELEASE_VERSION
			if (pkt_len > 1518) 
			{
				printk(MAC_KERN_WARNING "%s: Bogus packet size of %d (%#x).\n",
					   dev->name, pkt_len, pkt_len);
				pkt_len = 1518;
				cap->stats.rx_length_errors++;
			}
#endif
			/* Check if the packet is long enough to accept without copying
			   to a minimally-sized skbuff. */
			if (pkt_len < cap->rx_copythreshold
				//-+arthur
				&& (skb = dev_alloc_skb(pkt_len + 2)) != NULL) 
				//&& (skb = alloc_skb(pkt_len + 2, GFP_ATOMIC)) != NULL)
			{
				skb->dev = dev;
				skb_reserve(skb, 2);	/* 16 byte align the IP header */
				dma_cache_maint ((const void*)cap->rx_skbuff[entry].skb->tail,
						2048 /*pkt_len*/,DMA_FROM_DEVICE);
//				eth_copy_and_sum(skb, cap->rx_skbuff[entry].skb->tail, pkt_len, 0); //peter+ API change 2.6.19 -> 2.6.25
				skb_copy_to_linear_data (skb,  cap->rx_skbuff[entry].skb->tail, pkt_len);
				skb_put(skb, pkt_len);
				work_done++;
			} 
			else 
			{	/* Pass up the skb already on the Rx ring. */
				dma_cache_maint ((const void*)cap->rx_skbuff[entry].skb->tail,
						2048 /*pkt_len*/,DMA_FROM_DEVICE);
				skb_put(skb = cap->rx_skbuff[entry].skb, pkt_len);
				cap->rx_skbuff[entry].skb = NULL;
			}
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
			dev->last_rx = jiffies;
			cap->stats.rx_packets++;
			cap->stats.rx_bytes += pkt_len;
			//+arthur 0321
			outl(1, ioaddr + CSR_OFFSET(CSR2));		/* Rx poll demand */
		}
		cap->cur_rx++;
		entry = cap->cur_rx % RX_RING_SIZE;
		phypt_rx_ring = (struct	socle_mac_rx_desc *)(&(cap->rx_ring[entry]));
		status = le32_to_cpu(phypt_rx_ring->status);
	}

	return work_done;
} /* socle_mac_rx() */

//20080416 leonid+ for probe method with devices.c
static int socle_ether_probe(struct platform_device *pdev)
{
	int module_installed = 0;
	struct resource *res;
	int irq;
	u32 base;	
	struct net_device 			*dev;
	struct socle_mac_private 	*cap;
	/* See note below on the multiport cards. */
	unsigned char * 			mac_srom_pt = NULL;
	static unsigned char 		last_phys_addr[6] = {0x00, 'S', 'O', 'c', 'l', 'e'};
	int i, ret;
	int index = pdev->id;
	
	printk("socle_ether_probe\n");
	
	/* Find and claim our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "register resources unusable\n");
		ret = -ENXIO;
		goto free_something_1;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "unable to get irq\n");
		ret = -ENXIO;
		goto free_something_1;
	}
	
	if (!request_mem_region(res->start, (res->end - res->start) + 1, pdev->name)) {
                ret = -EBUSY;
		dev_err(&pdev->dev, "cannot request mem region\n");
                goto free_something_1;
        }	

	base = (u32)ioremap(res->start, (res->end - res->start) + 1);
	if (!base) {
		dev_err(&pdev->dev, "cannot map registers\n");
		ret = -ENOMEM;
		goto release_mem;
	}

	printk(MAC_KERN_DEBUG "Probe MAC module at <%8.8x> IRQ %d\n",
							base, irq);	
	if ( (inl(base + CSR_OFFSET(CSR5)) == 0xffffffff) 
	   ||(inl(base + CSR_OFFSET(CSR5)) == 0x00000000) )
	{
		printk(MAC_KERN_ERR "The SOCLE MAC moudle at 0x%08x is not functioning.\n", base);
		goto unmap_regs;
	}

	/* alloc_etherdev ensures aligned and zeroed private structures */
	dev = alloc_etherdev (sizeof (*cap));
	if (!dev)
	{
		printk(MAC_KERN_ERR "!!!! MAC ERROR : alloc_etherdev Fail.\n");
		goto unmap_regs;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);		//20100610 leonid+ for suspend/resume
//////////////////////////

	cap = netdev_priv(dev);


///////////////////////////
//	cap = dev->priv; 
#if 1
	//peter todo: iounmap
// 	cap->rx_ring = 
// 		(struct  socle_mac_rx_desc *)
// 		ioremap_nocache (virt_to_phys (cap->rx_ring_a),sizeof(struct  socle_mac_rx_desc) *RX_RING_SIZE);
		
cap->rx_ring = dma_alloc_coherent(NULL, sizeof(struct socle_mac_rx_desc) * RX_RING_SIZE,
							&cap->rx_ring_dma,
							GFP_KERNEL);
							
//printk("cap->rx_ring = 0x%08x\n",cap->rx_ring);
//printk("virt_to_phys(cap->rx_ring) = 0x%08x\n",virt_to_phys(cap->rx_ring));
//printk("cap->rx_ring_dma = 0x%08x\n",cap->rx_ring_dma);

														

		
// 	cap->tx_ring = 
// 		(struct  socle_mac_tx_desc *)
// 		ioremap_nocache (virt_to_phys (cap->tx_ring_a),
// 							      sizeof (struct
// 								      socle_mac_tx_desc) * TX_RING_SIZE);
cap->tx_ring = dma_alloc_coherent(NULL, sizeof(struct socle_mac_tx_desc) * TX_RING_SIZE,
							&cap->tx_ring_dma,
							GFP_KERNEL);		
// 	cap->setup_frame =
// 		(u16*) ioremap_nocache (virt_to_phys (cap->setup_frame_a), (sizeof (u16) * 96));
		
cap->setup_frame = dma_alloc_coherent(NULL, (sizeof (u16) * 96),
							&cap->setup_frame_dma,
							GFP_KERNEL);


#else
	cap->rx_ring=cap->rx_ring_a;
	cap->tx_ring=cap->tx_ring_a;
	cap->setup_frame=cap->setup_frame_a;
#endif
	/* Template give a device name for debug/information */
//	strcpy(dev->name, "Probe ... Socle MAC ");
//	strcpy(dev->name, "soclemac%d");
	strcpy(dev->name, "eth%d");
//	strcpy(dev->name, "soclemac");
	 
	/* Recode every install socle mac modules 
	   First Install, Last Remove */
	//cap->next_module = root_socle_mac_dev;
	//root_socle_mac_dev = dev;

//	printk(MAC_KERN_INFO "%s : %s at %#3lx IRQ %d\n",
//		   dev->name, "SOCLE MAC core w/AMBA", base, irq);

	/* initial spinlock flag */
	spin_lock_init(&cap->lock);
	spin_lock_init(&cap->mii_lock);
	init_timer(&cap->timer);
	cap->timer.data = (unsigned long)dev;
	cap->timer.function = socle_mac_timer;
	
	/* Reset the MAC chip */
	outl(CSR0_SWR, base + CSR_OFFSET(CSR0));

	//Fisrt try use EEPROM setting
	if(mac_srom_pt == NULL)
	{	
		last_phys_addr[0] = pdev->id * 4;
		memcpy(dev->dev_addr,&last_phys_addr,6); 
		dev->if_port = DEFAULT_IFPORTE;
		cap->csr0 = DEFAULT_CSR0_SETTING;
		cap->csr6 = DEFAULT_CSR6_SETTING;
		cap->csr11 = DEFAULT_CSR11_SETTING;
		if (mac_debug_level > NETIF_MSG_DRV)
			printk(MAC_KERN_DEBUG "%s : No SROM use Default Setting : \n" MAC_KERN_DEBUG "    CSR0<%8.8x> CSR6<%8.8x> CSR11<%8.8x> IFPORT<%d : %s>\n",
				dev->name, cap->csr0, cap->csr6, cap->csr11, dev->if_port, medianame[dev->if_port]);
	}	

	//Second check driver parameter override the setting
	if(options_if)
	{
		dev->if_port = options_if;		
		printk(MAC_KERN_DEBUG "%s : Module load force Transceiver selection to %s.\n",
			dev->name, medianame[dev->if_port]);
	}
	switch(dev->if_port)
	{
		case options_AutoNeg :	//Auto Neg
			cap->medialock = 0;
			cap->full_duplex_lock = 0;
			cap->speed_100M_lock = 0;	
			cap->mii_advertise = 0;	  
		break;
		case options_100baseTxF :	//100TxF
			cap->medialock = 1;
			cap->full_duplex_lock = 1;
			cap->speed_100M_lock = 1;
			cap->full_duplex = 1;	
			cap->speed_100M = 1;	
			cap->mii_advertise = MII_PHY_ANA_100F;  
		break;
		case options_100baseTxH :	//100TxH
			cap->medialock = 1;
			cap->full_duplex_lock = 1;
			cap->speed_100M_lock = 1;
			cap->full_duplex = 0;	
			cap->speed_100M = 1;
			cap->mii_advertise = MII_PHY_ANA_100H; 	  
		break;
		case options_10baseTxF :	//10TxF
			cap->medialock = 1;
			cap->full_duplex_lock = 1;
			cap->speed_100M_lock = 1;
			cap->full_duplex = 1;	
			cap->speed_100M = 0;	
			cap->mii_advertise = MII_PHY_ANA_10F;   
		break;
		case options_10baseTxH :	//10TxH
			cap->medialock = 1;
			cap->full_duplex_lock = 1;
			cap->speed_100M_lock = 1;
			cap->full_duplex = 0;	
			cap->speed_100M = 0;	  
			cap->mii_advertise = MII_PHY_ANA_10H; 
		break;
		case options_FullDuplex :	//FullDuplex
			cap->medialock = 0;
			cap->full_duplex_lock = 1;
			cap->speed_100M_lock = 0;
			cap->full_duplex = 1;	
			cap->mii_advertise = MII_PHY_ANA_FDAM;  
		break;								
		case options_HalfDuplex :	//HalfDuplex
			cap->medialock = 0;
			cap->full_duplex_lock = 1;
			cap->speed_100M_lock = 0;
			cap->full_duplex = 0;	
			cap->mii_advertise = MII_PHY_ANA_HDAM;  
		break;		
		case options_100Base :	//100base
			cap->medialock = 0;
			cap->full_duplex_lock = 0;
			cap->speed_100M_lock = 1;
			cap->speed_100M = 1;	  
			cap->mii_advertise = MII_PHY_ANA_100M; 
		break;		
		case options_10Base :	//10base
			cap->medialock = 0;
			cap->full_duplex_lock = 0;
			cap->speed_100M_lock = 1;
			cap->speed_100M = 0;	  
			cap->mii_advertise = MII_PHY_ANA_10M; 
		break;		
		default :
		break;
	}
//	printk(MAC_KERN_DEBUG "%s : Module load if_port be %s.\n",
//			dev->name, medianame[dev->if_port]);
			
	if(mtu[index] > 0)
	{
		dev->mtu = mtu[index];
		printk(MAC_KERN_DEBUG "%s : Module load force MTU %d.\n",
			dev->name, dev->mtu);
	}
	if(csr0)
	{
		cap->csr0 = csr0;
		printk(MAC_KERN_DEBUG "%s : Module load force csr0<%8.8x>.\n",
			dev->name, cap->csr0);							
	}

	printk(MAC_KERN_INFO "%s : MAC address <",
		   dev->name);	
	for (i = 0; i < 6; i++)
	{
		printk("%c%2.2X", i ? ':' : ' ', dev->dev_addr[i]);	
	}	
	printk(" >\n");	
		
	dev->base_addr = base;
	dev->irq = irq;
	
	cap->msg_level = (1 << mac_debug_level) - 1;
	cap->chip_id = index; 

	cap->rx_copythreshold = rx_copythreshold;
	cap->max_interrupt_work = max_interrupt_work;
	cap->multicast_filter_limit = multicast_filter_limit;
	
	/* The entries in the device structure. */
	dev->open = socle_mac_open;
	dev->stop = socle_mac_close;
	dev->hard_start_xmit = socle_mac_start_xmit;	
	dev->get_stats = socle_mac_get_stats;
	dev->do_ioctl = private_ioctl;
	dev->set_multicast_list = set_rx_mode;
	
	dev->tx_timeout = socle_mac_tx_timeout;
	dev->watchdog_timeo = TX_TIMEOUT;
	
	if(socle_mac_start_link(dev) == FALSE)
	{
		printk(MAC_KERN_ERR "%s : socle_mac_start_link Fail!!!!!!!!!!!!\n",
			dev->name);
		// Release the un-success intall device	
		//root_socle_mac_dev = cap->next_module;			
		goto free_dev;
	}	
	
//	dev->name[0] = 0;	//EOF
	if (register_netdev(dev))
	{
		printk(MAC_KERN_ERR "Probe ... Socle MAC %d  : register_netdev Fail!!!!!!!!!!!!\n",
			index);
		// Release the un-success intall device	
		
		//root_socle_mac_dev = cap->next_module;		
		goto free_dev;
	}
	
//	printk(MAC_KERN_INFO "Probe ... Socle MAC %d  : Success register be netdev : %s.\n",
//			index, dev->name);
	//+Peter
	ether_setup(dev);

	if (dev== NULL){	
		//Any wrong cleanup all install module before
		printk(MAC_KERN_DEBUG "Probe MAC module Fail !!!!!!!!!!! at <%8.8x> IRQ %d\n",
							base, irq);
		goto unregister_dev;
	}

	module_installed = TRUE;
	printk(MAC_KERN_DEBUG "Probe MAC module at <%8.8x> IRQ %d : Pass---------------------\n",
							base, irq);
		
	return 0;

unregister_dev:
	unregister_netdev(dev);	
free_dev:
		dma_free_coherent (NULL,
			     sizeof(struct socle_mac_rx_desc) * RX_RING_SIZE,
			     cap->rx_ring, cap->rx_ring_dma);
		dma_free_coherent (NULL,
			     sizeof(struct socle_mac_tx_desc) * TX_RING_SIZE,
			     cap->tx_ring, cap->tx_ring_dma);
		dma_free_coherent (NULL,
			     (sizeof (u16) * 96),
			     cap->setup_frame, cap->setup_frame_dma);
	kfree(dev);
unmap_regs:
	iounmap((void *)base);
release_mem:
	release_mem_region(res->start, (res->end - res->start) + 1);
free_something_1:
	return ret;
	
} /* socle_mac_probe() */

/* cleanup_mac() */
static int __devexit socle_ether_remove(struct platform_device *pdev)
{
	//struct net_device *next_dev;
	struct resource *res;
	struct net_device *ndev = platform_get_drvdata(pdev);

	/* No need to check MOD_IN_USE, as sys_delete_module() checks. */
	//while (root_socle_mac_dev) 
	{
		//struct socle_mac_private *cap = netdev_priv(root_socle_mac_dev);
		struct socle_mac_private *cap = netdev_priv(ndev);
		
		//next_dev = cap->next_module;
		
		//unregister_netdev(root_socle_mac_dev);	
		unregister_netdev(ndev);	
		
		//leonid+
		dma_free_coherent (NULL,
			     sizeof(struct socle_mac_rx_desc) * RX_RING_SIZE,
			     cap->rx_ring, cap->rx_ring_dma);
		dma_free_coherent (NULL,
			     sizeof(struct socle_mac_tx_desc) * TX_RING_SIZE,
			     cap->tx_ring, cap->tx_ring_dma);
		dma_free_coherent (NULL,
			     (sizeof (u16) * 96),
			     cap->setup_frame, cap->setup_frame_dma);
		
		//kfree(root_socle_mac_dev);
		//iounmap((void *)root_socle_mac_dev->base_addr);
		kfree(ndev);
		iounmap((void *)ndev->base_addr);
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        	release_mem_region(res->start, (res->end - res->start) + 1);
		
		//root_socle_mac_dev = next_dev;
	}

	return 0;
} /* cleanup_mac() */

#ifdef CONFIG_PM
static int
socle_ether_suspend (struct platform_device *pdev, pm_message_t msg)
{
        struct net_device *ndev = platform_get_drvdata(pdev);

	printk("socle_ether_suspend : %s\n", pdev->name);
		
        if (ndev) {
                if (netif_running(ndev)) {
                        netif_device_detach(ndev);
                        socle_mac_close(ndev);
                }
        }
	
        return 0;
}

static int 
socle_ether_resume(struct platform_device *pdev)
{
        struct net_device *ndev = platform_get_drvdata(pdev);

	printk("socle_ether_resume : %s\n", pdev->name);

        if (ndev) {
                if (netif_running(ndev)) {
                        socle_mac_open(ndev);
                        netif_device_attach(ndev);
                }
        }

	return 0;
}
#else
#define socle_ether_suspend	NULL
#define socle_ether_resume	NULL
#endif


static struct platform_driver socle_ether_driver = {
	.probe		= socle_ether_probe,
	.remove		= __devexit_p(socle_ether_remove),
	.suspend		= socle_ether_suspend,
	.resume		= socle_ether_resume,
	.driver		= {
		.name	= "socle_ether",	//20080416 leonid+ for device naming with devices.c
		.owner	= THIS_MODULE,
	},
};

static int __init socle_ether_init(void)
{
	return platform_driver_register(&socle_ether_driver);
}

static void __exit socle_ether_exit(void)
{
	platform_driver_unregister(&socle_ether_driver);
}

module_init(socle_ether_init)
module_exit(socle_ether_exit)

