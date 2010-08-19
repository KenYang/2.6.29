#ifndef __I2S_H_INCLUDED
#define __I2S_H_INCLUDED

#include <linux/types.h>

struct socle_i2s_platform_data {
	u8 tx_dma_ch;
	u8 rx_dma_ch;
	u8 tx_dma_hdreq;
	u8 rx_dma_hdreq;
	u8 fifo_depth;
	u8 burst_type;
};

#endif
