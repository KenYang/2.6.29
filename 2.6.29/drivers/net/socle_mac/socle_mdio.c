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


/*******************************************************************************
*
* sMacMiiPhyRead - read a PHY device register via MII
*
* RETURNS: the contents of a PHY device register.
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
}
