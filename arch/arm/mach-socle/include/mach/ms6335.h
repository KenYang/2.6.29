#ifndef __MS6335_H_INCLUDED
#define __MS6335_H_INCLUDED

#define MS6335_SLAVE_ADDR_DAC 0x46
#define MS6335_CLK 33868000

/*
 *  Register mapping
 *  */
// 20080804 cyli fix
#define MS6335_FUNC_VOL_CTRL 	0x0
#define MS6335_FUNC_VOL_L_CTRL 	(0x1 << 5)
#define MS6335_FUNC_VOL_R_CTRL 	(0x2 << 5)
#define MS6335_FUNC_PWR_MODE 	(0x3 << 5)

/*
 *  MS6335_FUNC_VOL CTRL
 *  */

/*
 *  MS6335_FUNC_PWR_DWN_MODE
 *  */
#define MS6335_PWR_DWN_CAPGD		(0x1 << 4)
#define MS6335_PWR_DWN_OPAPD		(0x1 << 3)
#define MS6335_PWR_DWN_DACPD		(0x1 << 2)
#define MS6335_PWR_DWN_HPPD		(0x1 << 1)
#define MS6335_PWR_DWN_DACM		0x1


/*
 *  MS6335_FUNC_PWR_MODE
 *  */
#define MS6335_PWR_MODE_DAC_ACTIVE		0x7b
#define MS6335_PWR_MODE_HP_OP_ACTIVE		0x61
#define MS6335_PWR_MODE_DAC_MUTE_OFF		0x60
#define MS6335_PWR_MODE_DAC_GAIN		0x1b
#define MS6335_PWR_MODE_DAC_INACTIVE		0x83
#define MS6335_PWR_MODE_PRE_PWR_OFF		0x83
#define MS6335_PWR_MODE_PWR_DOWN		0x6f



/*
 *  MS6335 Audio PWR
 *  */
#define MS6335_AUDIO_I2S_MODE			0x81


extern int ms6335_dac_power_switch(int sw);
extern int ms6335_dac_master_volume_l(int val);
extern int ms6335_dac_master_volume_r(int val);

#endif
