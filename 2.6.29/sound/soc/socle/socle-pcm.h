/*
 *  socle-pcm.h --
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  ALSA PCM interface
 */

#ifndef SOCLE_PCM_H_INCLUDED
#define SOCLE_PCM_H_INCLUDED

#define ST_RUNNING		(1<<0)
#define ST_OPENED		(1<<1)

/*FIXME*//* temp */
struct socle_dma_client {
	char *name;
};

struct socle_pcm_dma_params {
	struct socle_dma_client *client;	/* stream identifier */
	int channel;				/* Channel ID */ /* request number */
	dma_addr_t dma_addr;	/* physicla address */
	int data_width;			/* Size of the DMA transfer */
	int burst;
};

#define SOCLE_DAI_I2S			0

/* platform data */
extern struct snd_soc_platform socle_snd_soc_platform;

extern struct snd_pcm_substream *playback_substream;
extern struct snd_pcm_substream *capture_substream;
#endif
