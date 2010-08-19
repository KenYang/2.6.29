#ifndef ASOC_CODEC_H_INCLUDED
#define ASOC_CODEC_H_INCLUDED

struct asoc_setup_data {
	int            i2c_bus;
	unsigned short i2c_address;
	int            dac_clk;
#if 0
#define ASOC_DAC_CLK_SYSCLK 0
#define ASOC_DAC_CLK_WSPLL  1
#endif
};

#endif
