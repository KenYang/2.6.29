#ifndef __SOCLE_ADC_H_INCLUDED
#define __SOCLE_ADC_H_INCLUDED

#define XP	0
#define YP	1
#define ZX	2
#define ZY	3

int socle_adc_read_data(unsigned int ch);
void socle_adc_touch_switch(int mode);

#endif	//__SOCLE_ADC_H_INCLUDED
