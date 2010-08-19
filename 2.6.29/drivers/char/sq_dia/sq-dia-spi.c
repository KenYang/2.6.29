#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <mach/regs-spit.h>
#include <linux/platform_device.h>


//#define CONFIG_SQ_spiT_DEBUG
#ifdef CONFIG_SQ_spiT_DEBUG
	#define DIA_spi_DBG(fmt, args...) printk("\nDIA_spi: " fmt, ## args)
#else
	#define DIA_spi_DBG(fmt, args...)
#endif

enum spi_func {
spi_FUNC_GET_spiT_STRUCTURE,
spi_FUNC_RELEASE_spiT_STRUCTURE,
spi_FUNC_CLAIM_spi_LOCK,				
spi_FUNC_RELEASE_spi_LOCK,	
spi_FUNC_RESET,			
spi_FUNC_OUTPUT_ENABLE,		
spi_FUNC_ENABLE,			
spi_FUNC_CAPTURE_MODE_ENABLE,	
spi_FUNC_CLEAR_INTERRUPT,	
spi_FUNC_ENABLE_INTERRUPT,	
spi_FUNC_SINGLE_COUNTER_MODE_ENABLE,
spi_FUNC_SET_COUNTER,		
spi_FUNC_READ_HRC,		
spi_FUNC_READ_LRC,		
spi_FUNC_WRITE_HRC,		
spi_FUNC_WRITE_LRC,		
spi_FUNC_READ_PRESCALE_FACTOR,
spi_FUNC_WRITE_PRESCALE_FACTOR,
};	


struct sq_spit *spi_p;
struct sq_spit_driver *spi_drv;

//----------------------------------------------------------------------
//sq_dia_spi
//----------------------------------------------------------------------


extern  int sq_dia_spi(unsigned int  func,unsigned long  arg)
{

	unsigned int *p;
	
	p=(unsigned int  *)arg;
	
	DIA_spi_DBG("\n[spi]func=0x%x p=0x%x \n",func,*p);	
	
	switch (func){
	case spi_FUNC_GET_spiT_STRUCTURE:			
		DIA_spi_DBG("spi_FUNC_GET_spiT_STRUCTURE:			");	
		
		spi_p = get_sq_spit_structure(*p);
		if (NULL == spi_p) {
			DIA_spi_DBG("Can't get spiT structure (spi_bklgt)!!\n");
			return -1;
		}		
		
		spi_drv = spi_p->drv;	
	break;				
	case spi_FUNC_RELEASE_spiT_STRUCTURE:			
		DIA_spi_DBG("spi_FUNC_RELEASE_spiT_STRUCTURE:");  
		release_sq_spit_structure(*p);   
	break;	
	case spi_FUNC_CLAIM_spi_LOCK:			
		DIA_spi_DBG("spi_FUNC_CLAIM_spi_LOCK:");
		spi_drv->claim_spi_lock();	
	break;				
	case spi_FUNC_RELEASE_spi_LOCK:			
		DIA_spi_DBG("spi_FUNC_RELEASE_spi_LOCK:");   
		spi_drv->release_spi_lock();  
	break;
	case spi_FUNC_RESET:			        
		DIA_spi_DBG("spi_FUNC_RESET:");   
		spi_drv->reset(spi_p);	  
	break;
	case spi_FUNC_OUTPUT_ENABLE:		        
		DIA_spi_DBG("spi_FUNC_OUTPUT_ENABLE:");  
		spi_drv->output_enable(spi_p,*p);   
	break;
	case spi_FUNC_ENABLE:			        
		DIA_spi_DBG("spi_FUNC_ENABLE:");
		spi_drv->enable(spi_p,*p);    
	break;
	case spi_FUNC_CAPTURE_MODE_ENABLE:	        
		DIA_spi_DBG("spi_FUNC_CAPTURE_MODE_ENABLE:"); 
		spi_drv->capture_mode_enable(spi_p,*p);    
	break;
	case spi_FUNC_CLEAR_INTERRUPT:	                
		DIA_spi_DBG("spi_FUNC_CLEAR_INTERRUPT:");
		spi_drv->clear_interrupt(spi_p);     
	break;
	case spi_FUNC_ENABLE_INTERRUPT:	                
		DIA_spi_DBG("spi_FUNC_ENABLE_INTERRUPT:"); 
		spi_drv->enable_interrupt(spi_p,*p);    
	break;
	case spi_FUNC_SINGLE_COUNTER_MODE_ENABLE:       
		DIA_spi_DBG("spi_FUNC_SINGLE_COUNTER_MODE_ENABLE:");
		spi_drv->single_counter_mode_enable(spi_p,*p);     
	break;
	case spi_FUNC_SET_COUNTER:		        
		DIA_spi_DBG("spi_FUNC_SET_COUNTER:");
		spi_drv->set_counter(spi_p,*p);     
	break;
	case spi_FUNC_READ_HRC:		                
		DIA_spi_DBG("spi_FUNC_READ_HRC:");
		*p = spi_drv->read_hrc(spi_p);     
	break;
	case spi_FUNC_READ_LRC:		                
		DIA_spi_DBG("spi_FUNC_READ_LRC:");
		*p = spi_drv->read_lrc(spi_p);     
	break;
	case spi_FUNC_WRITE_HRC:		        
		DIA_spi_DBG("spi_FUNC_WRITE_HRC:");
		spi_drv->write_hrc(spi_p,*p);     
	break;
	case spi_FUNC_WRITE_LRC:		        
		DIA_spi_DBG("spi_FUNC_WRITE_LRC:");
		spi_drv->write_lrc(spi_p,*p);     
	break;
	case spi_FUNC_READ_PRESCALE_FACTOR:             
		DIA_spi_DBG("spi_FUNC_READ_PRESCALE_FACTOR:");
		*p = spi_drv->read_prescale_factor(spi_p);

	break;
	case spi_FUNC_WRITE_PRESCALE_FACTOR:            
		DIA_spi_DBG("spi_FUNC_WRITE_PRESCALE_FACTOR:");
		spi_drv->write_prescale_factor(spi_p,*p);     
	break;
	default:
		return -1;
	};	

		DIA_spi_DBG("[spi]p=0x%x \n",*p);	

	return 0;
}




MODULE_AUTHOR("Channing Lan");
MODULE_DESCRIPTION("SQ driver dia spi");
MODULE_LICENSE("GPL");



