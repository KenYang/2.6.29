#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <mach/regs-adc.h>
#include <mach/socle-adc.h>

//#define CONFIG_SQ_ADC_DEBUG
#ifdef CONFIG_SQ_ADC_DEBUG
	#define DIA_ADC_DBG(fmt, args...) printk("\nDIA_adc: " fmt, ## args)
#else
	#define DIA_ADC_DBG(fmt, args...)
#endif

enum adc_func {
ADC_FUNC_READ_CH,
};	


struct sq_adct *adc_p;
struct sq_adc_driver *adc_drv;

//----------------------------------------------------------------------
//sq_dia_adc
//----------------------------------------------------------------------
/* sq_adc_driver */
/*

*/

extern  int sq_dia_adc(unsigned int  func,unsigned long  arg)
{

	unsigned int *p;
	
	p=(unsigned int  *)arg;
	
	DIA_ADC_DBG("\n[adc]func=0x%x p=0x%x \n",func,*p);	
	
	switch (func){
	case ADC_FUNC_READ_CH:		                
		*(p+1) = socle_adc_read_data(*p);     
	break;
	default:
		return -1;
	};	

	DIA_ADC_DBG("[adc]p=0x%x \n",*(p+1));	

	return 0;
}




MODULE_AUTHOR("Channing Lan");
MODULE_DESCRIPTION("SQ driver dia adc");
MODULE_LICENSE("GPL");



