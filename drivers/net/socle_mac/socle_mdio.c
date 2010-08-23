/* socle_mii.c: ethernet mii interface access marco and function.

	SOCLE Technology Corp.
	14F-2, No.287, Section 2, Kwan-Fu Road, 
	Hsin-Chu City, Taiwan
	http://www.socle-tech.com.tw
*/


//for Tom's testing udelay (1) -> udelay (2)
/*
 * MII read/write access macros
 */
#define	sMAC_MII_BIT_READ(devAdrs, pBData)                           		\
    	{                                                           		\
    	outl(CSR9_MII_RD, devAdrs + CSR_OFFSET(CSR9));						\
        udelay (1);                                            			\
        outl(CSR9_MII_RD | CSR9_MDC, devAdrs + CSR_OFFSET(CSR9));			\
        udelay (1);                                                     	\
        *(pBData) |= CSR9_MII_DBIT_RD (inl (devAdrs + CSR_OFFSET(CSR9)));   \
    	}

#define	sMAC_MII_BIT_WRITE(devAdrs, data)                                 	\
    	{                                                               	\
    	outl(  CSR9_MII_DBIT_WR(data) | CSR9_MII_WR							\
    		 , devAdrs + CSR_OFFSET(CSR9));									\
        udelay (1);                                                 		\
    	outl(  CSR9_MII_DBIT_WR(data) | CSR9_MII_WR | CSR9_MDC				\
    		 , devAdrs + CSR_OFFSET(CSR9));									\
        udelay (1);                                                 		\
        }

#define	sMAC_MII_RTRISTATE(devAdrs)                                       	\
		{                                                               	\
        int retBData;                                                     	\
        sMAC_MII_BIT_READ ((devAdrs), &retBData);                          	\
        }

#define	sMAC_MII_WTRISTATE(devAdrs)                                       	\
		{                                                               	\
        sMAC_MII_BIT_WRITE((devAdrs), 0x1);                               	\
        sMAC_MII_BIT_WRITE((devAdrs), 0x0);                               	\
		}

#define sMAC_MII_WRITE(devAdrs, data, bitCount)                           	\
    	{                                                               	\
        int i=(bitCount);                                               	\
                                                                        	\
        while (i--)                                                    	 	\
            sMAC_MII_BIT_WRITE ((devAdrs), ((data) >> i) & 0x1);          	\
        }

#define	sMAC_MII_READ(devAdrs, pData, bitCount)                           	\
    	{                                                               	\
        int i=(bitCount);                                               	\
                                                                        	\
        while (i--)                                                     	\
            {                                                           	\
            *(pData) <<= 1;                                             	\
            sMAC_MII_BIT_READ ((devAdrs), (pData));                       	\
            }                                                           	\
        }

//#define SQ_MDIO_DBG
#ifdef SQ_MDIO_DBG
	#define MDIO_DBG(fmt, args...) printk("\nMDIO: " fmt, ## args)
#else
	#define MDIO_DBG(fmt, args...)
#endif
//#define CONFIG_SQ_NO_PHY	


/*******************************************************************************
*
* sMacMiiPhyRead - read a PHY device register via MII
*
* RETURNS: the contents of a PHY device register.
*/

int mii_reg[]={
0x3100,	
0x786d,
0x243,
0xc54,
0x1e1,
0x41e1,
};

int data_flag = 0;

/*
MDIO: phyAdrs = 0x0 ,phyReg = 0x2 retVal = 0xffff
MDIO:MII ID0<ffff>
MDIO: phyAdrs = 0x1 ,phyReg = 0x2 retVal = 0x243
MDIO:MII ID0<0243>
MDIO: phyAdrs = 0x1 ,phyReg = 0x3 retVal = 0xc54
MDIO:MII ID1<0c54>
MDIO:MII PHY #1
MDIO: phyAdrs = 0x1 ,phyReg = 0x0 retVal = 0x3100
MDIO: phyAdrs = 0x1 ,phyReg = 0x4 retVal = 0x1e1
MDIO: phyAdrs = 0x1 ,phyReg = 0x1 retVal = 0x786d
MDIO: phyAdrs = 0x1 ,phyReg = 0x0 data = 0x1200
MDIO: phyAdrs = 0x1 ,phyReg = 0x1 retVal = 0x7849<7>Probe MAC module at <f7060000> IRQ 15 : Pass		
MDIO: phyAdrs = 0x1 ,phyReg = 0x0 data = 0x1200
MDIO: phyAdrs = 0x1 ,phyReg = 0x1 retVal = 0x784
MDIO: phyAdrs = 0x1 ,phyReg = 0x1 retVal = 0x786d<6>eth0: MII PHY 1 : Get New Media Link.
MDIO: phyAdrs = 0x1 ,phyReg = 0x1 retVal = 0x786d
MDIO: phyAdrs = 0x1 ,phyReg = 0x5 retVal = 0x41e1<6>eth0: Set Media Mode <100M><Full>
MDIO: phyAdrs = 0x1 ,phyReg = 0x1 retVal = 0x786d
MDIO: phyAdrs = 0x1 ,phyReg = 0x5 retVal = 0x41e1
*/

u16 sMacMiiPhyRead
    (
    struct net_device	*	pDrvCtrl,		/* pointer to device control structure */
    u32			phyAdrs, 					// PHY address to access
    u32			phyReg   					// PHY register to read
    )
{
//struct socle_mac_private *cap = (struct socle_mac_private *)pDrvCtrl->priv;
struct socle_mac_private *cap = netdev_priv(pDrvCtrl);
unsigned long flags;
u16 retVal=0;

#ifdef CONFIG_SQ_NO_PHY
	if(phyAdrs == 0x01) {	
		if(phyReg < (sizeof(mii_reg)/4)) {
			
			if( data_flag == 0x1200) {
				data_flag = 0;	
				retVal = 0x7849;
			} else {
				retVal = (mii_reg[phyReg]);
			}
		} else {
			retVal = 0xffff;				
		}				
	} else {
		retVal = 0xffff;
	}

#else
	spin_lock_irqsave(&cap->mii_lock, flags);
	
    /* Write 34-bit preamble */
    sMAC_MII_WRITE (pDrvCtrl->base_addr, MII_PREAMBLE, 32);

    /* start of frame + op-code nibble */
    sMAC_MII_WRITE (pDrvCtrl->base_addr, MII_SOF | MII_RD, 4);

    /* device address */
    sMAC_MII_WRITE (pDrvCtrl->base_addr, phyAdrs, 5);
    sMAC_MII_WRITE (pDrvCtrl->base_addr, phyReg, 5);

    /* turn around */
    sMAC_MII_RTRISTATE (pDrvCtrl->base_addr);

    /* read data */
    sMAC_MII_READ (pDrvCtrl->base_addr, &retVal, 16);

	/* SW ISSUE */
	/* turn around */
    sMAC_MII_RTRISTATE (pDrvCtrl->base_addr);

	spin_unlock_irqrestore(&cap->mii_lock, flags);
	
#endif	
	MDIO_DBG("phyAdrs = 0x%x ,phyReg = 0x%x retVal = 0x%x",phyAdrs,phyReg,retVal);
	
    return (retVal);
}

/*******************************************************************************
*
* sMacMiiPhyWrite - write to a PHY device register via MII
*
* RETURNS: none
*/

void sMacMiiPhyWrite
    (
    struct net_device	*	pDrvCtrl,		/* pointer to device control structure */
    u32			phyAdrs,					// PHY address to access
    u32			phyReg, 					// PHY register to write
    u16			data    					// Data to write
    )
{
//struct socle_mac_private *cap = (struct socle_mac_private *)pDrvCtrl->priv;
struct socle_mac_private *cap = netdev_priv(pDrvCtrl);
unsigned long flags;

#ifdef CONFIG_SQ_NO_PHY
	if(phyAdrs == 0x01) {	
		if(phyReg == 0x00) {
			data_flag = data;
		}}
#endif 

	spin_lock_irqsave(&cap->mii_lock, flags);

    /* write 34-bit preamble */
    sMAC_MII_WRITE (pDrvCtrl->base_addr, MII_PREAMBLE, 32);
    
    /* start of frame + op-code nibble */
    sMAC_MII_WRITE (pDrvCtrl->base_addr, MII_SOF | MII_WR, 4);

    /* device address */
    sMAC_MII_WRITE (pDrvCtrl->base_addr, phyAdrs, 5);
    sMAC_MII_WRITE (pDrvCtrl->base_addr, phyReg, 5);

    /* turn around */
    sMAC_MII_WTRISTATE (pDrvCtrl->base_addr);

    /* write data */
    sMAC_MII_WRITE (pDrvCtrl->base_addr, data, 16);
    
    spin_unlock_irqrestore(&cap->mii_lock, flags);
 
	MDIO_DBG("phyAdrs = 0x%x ,phyReg = 0x%x data = 0x%x",phyAdrs,phyReg,data);    
}
