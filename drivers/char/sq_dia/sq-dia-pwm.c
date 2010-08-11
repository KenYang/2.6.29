#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <mach/regs-pwmt.h>
#include <linux/platform_device.h>


#define CONFIG_SQ_PWMT_DEBUG
#ifdef CONFIG_SQ_PWMT_DEBUG
	#define DIA_PWM_DBG(fmt, args...) printk("\nDIA_PWM: " fmt, ## args)
#else
	#define DIA_PWM_DBG(fmt, args...)
#endif

enum pwm_func {
PWM_FUNC_GET_PWMT_STRUCTURE,
PWM_FUNC_RELEASE_PWMT_STRUCTURE,
PWM_FUNC_CLAIM_PWM_LOCK,				
PWM_FUNC_RELEASE_PWM_LOCK,	
PWM_FUNC_RESET,			
PWM_FUNC_OUTPUT_ENABLE,		
PWM_FUNC_ENABLE,			
PWM_FUNC_CAPTURE_MODE_ENABLE,	
PWM_FUNC_CLEAR_INTERRUPT,	
PWM_FUNC_ENABLE_INTERRUPT,	
PWM_FUNC_SINGLE_COUNTER_MODE_ENABLE,
PWM_FUNC_SET_COUNTER,		
PWM_FUNC_READ_HRC,		
PWM_FUNC_READ_LRC,		
PWM_FUNC_WRITE_HRC,		
PWM_FUNC_WRITE_LRC,		
PWM_FUNC_READ_PRESCALE_FACTOR,
PWM_FUNC_WRITE_PRESCALE_FACTOR,
};	




//----------------------------------------------------------------------
//sq_dia_pwm
//----------------------------------------------------------------------
/* sq_pwmt_driver */
/*
extern struct sq_pwmt *get_sq_pwmt_structure(int num);
extern int release_sq_pwmt_structure(int num);
static void sq_pwmt_claim_lock(void);
static void sq_pwmt_release_lock(void);
static void sq_pwmt_reset(struct sq_pwmt *p_pwm_st);
static void sq_pwmt_output_enable(struct sq_pwmt *p_pwm_st, int en);
static void sq_pwmt_enable(struct sq_pwmt *p_pwm_st, int en);
static void sq_pwmt_capture_mode_enable(struct sq_pwmt *p_pwm_st, int en);
static void sq_pwmt_clear_interrupt(struct sq_pwmt *p_pwm_st);
static void sq_pwmt_enable_interrupt(struct sq_pwmt *p_pwm_st, int en);
static void sq_pwmt_single_counter_mode_enable(struct sq_pwmt *p_pwm_st, int en);
static void sq_pwmt_set_counter(struct sq_pwmt *p_pwm_st, unsigned int data);
static unsigned int sq_pwmt_read_hrc(struct sq_pwmt *p_pwm_st);
static unsigned int sq_pwmt_read_lrc(struct sq_pwmt *p_pwm_st);
static void sq_pwmt_write_hrc(struct sq_pwmt *p_pwm_st, unsigned int data);
static void sq_pwmt_write_lrc(struct sq_pwmt *p_pwm_st, unsigned int data);
static unsigned int sq_pwmt_read_prescale_factor(struct sq_pwmt *p_pwm_st);
static void sq_pwmt_write_prescale_factor(struct sq_pwmt *p_pwm_st, unsigned int data);
*/
#define get_sq_pwmt_structure			get_socle_pwmt_structure             
#define release_sq_pwmt_structure               release_socle_pwmt_structure         
#define sq_pwmt_claim_lock                      socle_pwmt_claim_lock                
#define sq_pwmt_release_lock                    socle_pwmt_release_lock              
#define sq_pwmt_reset                           socle_pwmt_reset                     
#define sq_pwmt_output_enable                   socle_pwmt_output_enable             
#define sq_pwmt_enable                          socle_pwmt_enable                    
#define sq_pwmt_capture_mode_enable             socle_pwmt_capture_mode_enable       
#define sq_pwmt_clear_interrupt                 socle_pwmt_clear_interrupt           
#define sq_pwmt_enable_interrupt                socle_pwmt_enable_interrupt          
#define sq_pwmt_single_counter_mode_enable      socle_pwmt_single_counter_mode_enable
#define sq_pwmt_set_counter                     socle_pwmt_set_counter               
#define sq_pwmt_read_hrc                        socle_pwmt_read_hrc                  
#define sq_pwmt_read_lrc                        socle_pwmt_read_lrc                  
#define sq_pwmt_write_hrc                       socle_pwmt_write_hrc                 
#define sq_pwmt_write_lrc                       socle_pwmt_write_lrc                 
#define sq_pwmt_read_prescale_factor            socle_pwmt_read_prescale_factor      
#define sq_pwmt_write_prescale_factor           socle_pwmt_write_prescale_factor     
#define sq_pwmt					socle_pwmt
#define sq_pwmt_driver				socle_pwmt_driver
struct sq_pwmt *pwm_p;
struct sq_pwmt_driver *pwm_drv;


extern  int sq_dia_pwm(unsigned int  func,unsigned long  arg)
{

	unsigned int *p;
	
	p=(unsigned int  *)arg;

	
	DIA_PWM_DBG("\n[pwm]func=0x%x p=0x%x \n",func,*p);	
//	if((func!=PWM_FUNC_GET_PWMT_STRUCTURE) && (func!=PWM_FUNC_RELEASE_PWMT_STRUCTURE)) {
//		pwm_drv->claim_pwm_lock();
//	}
	switch (func){
	case PWM_FUNC_GET_PWMT_STRUCTURE:			
		DIA_PWM_DBG("PWM_FUNC_GET_PWMT_STRUCTURE:");		
		*(p+1) = (unsigned int)get_sq_pwmt_structure(*p);
		if ((unsigned int)NULL == (unsigned int)*(p+1)) {	
			DIA_PWM_DBG("Can't get PWMT %d structure ,return 0x%x!!\n",*p,*(p+1));
			return -1;
		}		
		pwm_p = (struct sq_pwmt *)*(p+1) ;	
		pwm_drv = pwm_p->drv;	
	break;				
	case PWM_FUNC_RELEASE_PWMT_STRUCTURE:			
		DIA_PWM_DBG("PWM_FUNC_RELEASE_PWMT_STRUCTURE:");  
		*(p+1) = (unsigned int)release_sq_pwmt_structure(*p);  
		if ((unsigned int)NULL != (unsigned int)*(p+1)) {
			DIA_PWM_DBG("Can't release PWMT %d structure ,return 0x%x!!\n",*p,*(p+1));
			return -1;
		}			 
	break;	
	case PWM_FUNC_CLAIM_PWM_LOCK:			
		DIA_PWM_DBG("PWM_FUNC_CLAIM_PWM_LOCK:");
		pwm_drv->claim_pwm_lock();	
	break;				
	case PWM_FUNC_RELEASE_PWM_LOCK:			
		DIA_PWM_DBG("PWM_FUNC_RELEASE_PWM_LOCK:");   
		pwm_drv->release_pwm_lock();  
	break;
	case PWM_FUNC_RESET:			        
		DIA_PWM_DBG("PWM_FUNC_RESET:");   
		pwm_drv->reset(pwm_p);	  
	break;
	case PWM_FUNC_OUTPUT_ENABLE:		        
		DIA_PWM_DBG("PWM_FUNC_OUTPUT_ENABLE:");  
		pwm_drv->output_enable(pwm_p,*p);   
	break;
	case PWM_FUNC_ENABLE:			        
		DIA_PWM_DBG("PWM_FUNC_ENABLE:");
		pwm_drv->enable(pwm_p,*p);    
	break;
	case PWM_FUNC_CAPTURE_MODE_ENABLE:	        
		DIA_PWM_DBG("PWM_FUNC_CAPTURE_MODE_ENABLE:"); 
		pwm_drv->capture_mode_enable(pwm_p,*p);    
	break;
	case PWM_FUNC_CLEAR_INTERRUPT:	                
		DIA_PWM_DBG("PWM_FUNC_CLEAR_INTERRUPT:");
		pwm_drv->clear_interrupt(pwm_p);     
	break;
	case PWM_FUNC_ENABLE_INTERRUPT:	                
		DIA_PWM_DBG("PWM_FUNC_ENABLE_INTERRUPT:"); 
		pwm_drv->enable_interrupt(pwm_p,*p);    
	break;
	case PWM_FUNC_SINGLE_COUNTER_MODE_ENABLE:       
		DIA_PWM_DBG("PWM_FUNC_SINGLE_COUNTER_MODE_ENABLE:");
		pwm_drv->single_counter_mode_enable(pwm_p,*p);     
	break;
	case PWM_FUNC_SET_COUNTER:		        
		DIA_PWM_DBG("PWM_FUNC_SET_COUNTER:");
		pwm_drv->set_counter(pwm_p,*p);     
	break;
	case PWM_FUNC_READ_HRC:		                
		DIA_PWM_DBG("PWM_FUNC_READ_HRC:");
		*p = pwm_drv->read_hrc(pwm_p);     
	break;
	case PWM_FUNC_READ_LRC:		                
		DIA_PWM_DBG("PWM_FUNC_READ_LRC:");
		*p = pwm_drv->read_lrc(pwm_p);     
	break;
	case PWM_FUNC_WRITE_HRC:		        
		DIA_PWM_DBG("PWM_FUNC_WRITE_HRC:");
		pwm_drv->write_hrc(pwm_p,*p);     
	break;
	case PWM_FUNC_WRITE_LRC:		        
		DIA_PWM_DBG("PWM_FUNC_WRITE_LRC:");
		pwm_drv->write_lrc(pwm_p,*p);     
	break;
	case PWM_FUNC_READ_PRESCALE_FACTOR:             
		DIA_PWM_DBG("PWM_FUNC_READ_PRESCALE_FACTOR:");
		*p = pwm_drv->read_prescale_factor(pwm_p);

	break;
	case PWM_FUNC_WRITE_PRESCALE_FACTOR:            
		DIA_PWM_DBG("PWM_FUNC_WRITE_PRESCALE_FACTOR:");
		pwm_drv->write_prescale_factor(pwm_p,*p);     
	break;
	default:
		return -1;
	};	

		DIA_PWM_DBG("[pwm]p=0x%x \n",*p);	

//	if((func!=PWM_FUNC_GET_PWMT_STRUCTURE) | (func!=PWM_FUNC_RELEASE_PWMT_STRUCTURE)) {
//		pwm_drv->release_pwm_lock();  
//	}		
		
	return 0;
	
}




MODULE_AUTHOR("Channing Lan");
MODULE_DESCRIPTION("SQ driver dia pwm");
MODULE_LICENSE("GPL");



