#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
//#include <mach/regs-scu.h>
#include <mach/regs-pc9223-scu.h>
#include <mach/pc9223-scu.h>
//#include <mach/sq8000-scu.h>
#include <linux/platform_device.h>

#define CONFIG_SQ_SCU_DEBUG
#ifdef CONFIG_SQ_SCU_DEBUG
	#define DIA_SCU_DBG(fmt, args...) printk("\nDIA_SCU: " fmt, ## args)
#else
	#define DIA_SCU_DBG(fmt, args...)
#endif




//----------------------------------------------------------------------
//sq_dia_scu
//----------------------------------------------------------------------
/* sq_scut_driver */
/*
// UPLL configuration 
extern int sq_scu_upll_set (int clock);
extern int sq_scu_upll_get (void);

// UPLL power down/normal	
extern void sq_scu_upll_normal (void);
extern void sq_scu_upll_power_down (void);
extern int sq_scu_upll_status_get (void);

// MPLL configuration 
extern int sq_scu_mpll_set (int clock);	
extern int sq_scu_mpll_get (void);

// MPLL power down/normal	
extern void sq_scu_mpll_normal (void);	
extern void sq_scu_mpll_power_down (void);
extern int sq_scu_mpll_status_get (void);		//return 1:normal	0:slow

//	mclk enable/disable	
extern int sq_scu_mclk_enable (int ip);
extern int sq_scu_mclk_disable (int ip);

//	aclk enable/disable	
extern int sq_scu_aclk_enable (int ip);
extern int sq_scu_aclk_disable (int ip);

// read chip ID	
extern u32 sq_scu_chip_id (void);

//	SCU_PLLPARAM_A	
// UPLL configuration 
extern int sq_scu_upll_set (int clock);
extern int sq_scu_upll_get (void);
// CPLL configuration 
extern int sq_scu_cpll_set (int clock);
extern int sq_scu_cpll_get (void);			//return cpll clock value

//	SCU_PLLPARAM_B	
// UPLL power down/normal	
extern void sq_scu_upll_normal (void);
extern void sq_scu_upll_power_down (void);
extern int sq_scu_upll_status_get(void);		//1:normal , 0:down 
// CPLL power down/normal	
extern void sq_scu_cpll_normal (void);
extern void sq_scu_cpll_power_down (void);
extern int sq_scu_cpll_status_get(void);		//1:normal , 0:down 
// CPU/AHB clock ratio	
extern int sq_scu_clock_ratio_set (int ratio);
extern int sq_scu_clock_ratio_get (void);

// PLL lock period	
extern void sq_scu_pll_lock_period_set (int period);		//period minimum value is 2
extern int sq_scu_pll_lock_period_get (void);
extern void sq_scu_adc_clk_div_set (int div);		//div minimum value is 2
extern int sq_scu_adc_clk_div_get (void);
extern int sq_scu_uart_clk_24_set (int uart);
extern int sq_scu_uart_clk_upll_set (int uart);
extern int sq_scu_uart_clk_upll_2_set (int uart);
extern int sq_scu_uart_clk_upll_4_set (int uart);
extern int sq_scu_uart_clk_get (int uart);

extern int sq_scu_sdram_bus_width_status (void);
extern int sq_SCU_Dcm_mode_status (void);
extern int sq_scu_usb_mode_status (void);
extern int sq_scu_boot_source_status (void);
extern int sq_scu_tps_mac_status (void);
extern int sq_scu_tps_mac_status (void);
extern int sq_scu_auto_boot_fail_status (void);
extern void sq_scu_pw_standbywfi_enable (int i);
extern void sq_scu_stop_mode_enable (int i);
extern void sq_scu_slow_mode_disable (int i);


extern void sq_scu_sw_reset(void);

extern void sq_scu_sw_remap(void);

extern unsigned long __init get_pll_clock(void);

extern int sq_scu_devenable(u32 dev);
extern int sq_scu_devdisable(u32 dev);

extern void sq_scu_lcdc_clk_input_mpll_outpput(void);
extern void sq_scu_lcdc_clk_input_mpll_xin(void);
extern void sq_scu_hdma_req45_spi0(void);
extern void sq_scu_hdma_req45_spi1(void);
extern void sq_scu_uhc0_48clock_input_upll(void);
extern void sq_scu_uhc0_48clock_input_otg_phy(void);
extern void sq_scu_uhc1_48clock_input_upll(void);
extern void sq_scu_uhc1_48clock_input_otg_phy(void);
extern int sq_scu_hdma_req01_uart(int uart);
extern int sq_scu_hdma_req23_uart(int uart);
extern void sq_scu_wdt_reset_enable(int en);
extern void sq_scu_sw_reset_enable(int en);
extern void sq_scu_nfiq_polarity_high(int en);


//	SCU_INFORM	
extern void sq_scu_info0_set (u32 inf);		
extern u32 sq_scu_info0_get (void);						//return information0 value
extern void sq_scu_info1_set (u32 inf);		
extern u32 sq_scu_info1_get (void);						//return information1 value	
extern void sq_scu_info2_set (u32 inf);		
extern u32 sq_scu_info2_get (void);						//return information2 value	
extern void sq_scu_info3_set (u32 inf);		
extern u32 sq_scu_info3_get (void);						//return information3 value	


extern unsigned long sq_get_cpu_clock (void);		//return CPU clock (Hz)
extern unsigned long sq_get_ahb_clock (void);		//return AHB clock (Hz)
extern unsigned long sq_get_apb_clock (void);		//return APB clock (Hz)
extern unsigned long sq_get_uart_clock (int uart);		//return UART clock (Hz)

*/

#define sq_scu_upll_set				socle_scu_upll_set                              
#define sq_scu_upll_get	                        socle_scu_upll_get	                        
#define sq_scu_upll_normal                      socle_scu_upll_normal                           
#define sq_scu_upll_power_down                  socle_scu_upll_power_down                       
#define sq_scu_upll_status_get                  socle_scu_upll_status_get                       
#define sq_scu_mpll_set	                        socle_scu_mpll_set	                        
#define sq_scu_mpll_get	                        socle_scu_mpll_get	                        
#define sq_scu_mpll_normal	                socle_scu_mpll_normal	                        
#define sq_scu_mpll_power_down                  socle_scu_mpll_power_down                       
#define sq_scu_mpll_status_get                  socle_scu_mpll_status_get                       
#define sq_scu_mclk_enable                      socle_scu_mclk_enable                           
#define sq_scu_mclk_disable	                socle_scu_mclk_disable	                        
#define sq_scu_aclk_enable                      socle_scu_aclk_enable                           
#define sq_scu_aclk_disable                     socle_scu_aclk_disable                          
#define sq_scu_chip_id                          socle_scu_chip_id                               
#define sq_scu_upll_set                         socle_scu_upll_set                              
#define sq_scu_upll_get                         socle_scu_upll_get                              
#define sq_scu_cpll_set                         socle_scu_cpll_set                              
#define sq_scu_cpll_get		                socle_scu_cpll_get		                
#define sq_scu_upll_normal                      socle_scu_upll_normal                           
#define sq_scu_upll_power_down                  socle_scu_upll_power_down                       
#define sq_scu_upll_status_get                  socle_scu_upll_status_get                       
#define sq_scu_cpll_normal                      socle_scu_cpll_normal                           
#define sq_scu_cpll_power_down                  socle_scu_cpll_power_down                       
#define sq_scu_cpll_status_get                  socle_scu_cpll_status_get                       
#define sq_scu_clock_ratio_set                  socle_scu_clock_ratio_set                       
#define sq_scu_clock_ratio_get                  socle_scu_clock_ratio_get                       
#define sq_scu_pll_lock_period_set              socle_scu_pll_lock_period_set                   
#define sq_scu_pll_lock_period_get              socle_scu_pll_lock_period_get                   
#define sq_scu_adc_clk_div_set                  socle_scu_adc_clk_div_set                       
#define sq_scu_adc_clk_div_get                  socle_scu_adc_clk_div_get                       
#define sq_scu_uart_clk_24_set                  socle_scu_uart_clk_24_set                       
#define sq_scu_uart_clk_upll_set                socle_scu_uart_clk_upll_set                     
#define sq_scu_uart_clk_upll_2_set              socle_scu_uart_clk_upll_2_set                   
#define sq_scu_uart_clk_upll_4_set              socle_scu_uart_clk_upll_4_set                   
#define sq_scu_uart_clk_get                     socle_scu_uart_clk_get                          
#define sq_scu_sdram_bus_width_status           socle_scu_sdram_bus_width_status                
#define sq_SCU_Dcm_mode_status                  socle_SCU_Dcm_mode_status                       
#define sq_scu_usb_mode_status                  socle_scu_usb_mode_status                       
#define sq_scu_boot_source_status               socle_scu_boot_source_status                    
#define sq_scu_tps_mac_status                   socle_scu_tps_mac_status                        
#define sq_scu_tps_mac_status                   socle_scu_tps_mac_status                        
#define sq_scu_auto_boot_fail_status            socle_scu_auto_boot_fail_status                 
#define sq_scu_pw_standbywfi_enable             socle_scu_pw_standbywfi_enable                  
#define sq_scu_stop_mode_enable                 socle_scu_stop_mode_enable                      
#define sq_scu_slow_mode_disable                socle_scu_slow_mode_disable                     
#define sq_scu_sw_reset                         socle_scu_sw_reset                              
#define sq_scu_sw_remap                         socle_scu_sw_remap                              
#define sq_scu_devenable                        socle_scu_devenable                             
#define sq_scu_devdisable                       socle_scu_devdisable                            
#define sq_scu_lcdc_clk_input_mpll_outpput      socle_scu_lcdc_clk_input_mpll_outpput           
#define sq_scu_lcdc_clk_input_mpll_xin          socle_scu_lcdc_clk_input_mpll_xin               
#define sq_scu_hdma_req45_spi0                  socle_scu_hdma_req45_spi0                       
#define sq_scu_hdma_req45_spi1                  socle_scu_hdma_req45_spi1                       
#define sq_scu_uhc0_48clock_input_upll          socle_scu_uhc0_48clock_input_upll               
#define sq_scu_uhc0_48clock_input_otg_phy       socle_scu_uhc0_48clock_input_otg_phy            
#define sq_scu_uhc1_48clock_input_upll          socle_scu_uhc1_48clock_input_upll               
#define sq_scu_uhc1_48clock_input_otg_phy       socle_scu_uhc1_48clock_input_otg_phy            
#define sq_scu_hdma_req01_uart                  socle_scu_hdma_req01_uart                       
#define sq_scu_hdma_req23_uart                  socle_scu_hdma_req23_uart                       
#define sq_scu_wdt_reset_enable                 socle_scu_wdt_reset_enable                      
#define sq_scu_sw_reset_enable                  socle_scu_sw_reset_enable                       
#define sq_scu_nfiq_polarity_high	        socle_scu_nfiq_polarity_high	                
#define sq_scu_info0_set		        socle_scu_info0_set		                
#define sq_scu_info0_get			socle_scu_info0_get						
#define sq_scu_info1_set		        socle_scu_info1_set		                
#define sq_scu_info1_get			socle_scu_info1_get						
#define sq_scu_info2_set		        socle_scu_info2_set		                
#define sq_scu_info2_get			socle_scu_info2_get						
#define sq_scu_info3_set		        socle_scu_info3_set		                
#define sq_scu_info3_get			socle_scu_info3_get						
#define sq_get_cpu_clock		        socle_get_cpu_clock		                
#define sq_get_ahb_clock		        socle_get_ahb_clock		                
#define sq_get_apb_clock		        socle_get_apb_clock		                
#define sq_get_uart_clock		        socle_get_uart_clock		                
                                                




struct scu_func_s {
	char msg[20];
	unsigned int (*func)(unsigned int);
};


enum scu_func {
SCU_MPLL_SET,
SCU_MPLL_GET,
SCU_MPLL_NORMAL,
SCU_MPLL_POWER_DOWN,
SCU_MPLL_STATUS_GET,
SCU_MCLK_ENABLE,
SCU_MCLK_DISABLE,
SCU_ACLK_ENABLE,
SCU_ACLK_DISABLE,
SCU_CHIP_ID,
SCU_UPLL_SET,
SCU_UPLL_GET,
SCU_UPLL_NORMAL,
SCU_UPLL_POWER_DOWN,
SCU_UPLL_STATUS_GET,
SCU_CLOCK_RATIO_SET,
SCU_CLOCK_RATIO_GET,
SCU_PLL_LOCK_PERIOD_SET,
SCU_PLL_LOCK_PERIOD_GET,
SCU_ADC_CLK_DIV_SET,
SCU_ADC_CLK_DIV_GET,
SCU_UART_CLK_24_SET,
SCU_UART_CLK_UPLL_SET,
SCU_UART_CLK_UPLL_2_SET,
SCU_UART_CLK_UPLL_4_SET,
SCU_UART_CLK_GET,
SCU_SDRAM_BUS_WIDTH_STATUS,
SCU_DCM_MODE_STATUS,
SCU_USB_MODE_STATUS,
SCU_BOOT_SOURCE_STATUS,
SCU_TPS_MAC_STATUS,
SCU_AUTO_BOOT_FAIL_STATUS,
SCU_PW_STANDBYWFI_ENABLE,
SCU_STOP_MODE_ENABLE,
SCU_SLOW_MODE_DISABLE,
SCU_SW_RESET,
SCU_SW_REMAP,
SCU_GET_PLL_CLOCK,
SCU_DEVENABLE,
SCU_DEVDISABLE,
SCU_LCDC_CLK_INPUT_MPLL_OUTPPUT,
SCU_LCDC_CLK_INPUT_MPLL_XIN,
SCU_HDMA_REQ45_SPI0,
SCU_HDMA_REQ45_SPI1,
SCU_UHC0_48CLOCK_INPUT_UPLL,
SCU_UHC0_48CLOCK_INPUT_OTG_PHY,
SCU_UHC1_48CLOCK_INPUT_UPLL,
SCU_UHC1_48CLOCK_INPUT_OTG_PHY,
SCU_HDMA_REQ01_UART,
SCU_HDMA_REQ23_UART,
SCU_WDT_RESET_ENABLE,
SCU_SW_RESET_ENABLE,
SCU_NFIQ_POLARITY_HIGH,
SCU_INFO0_SET,
SCU_INFO0_GET,
SCU_INFO1_SET,
SCU_INFO1_GET,
SCU_INFO2_SET,
SCU_INFO2_GET,
SCU_INFO3_SET,
SCU_INFO3_GET,
SCU_GET_CPU_CLOCK,
SCU_GET_AHB_CLOCK,
SCU_GET_APB_CLOCK,
SCU_GET_UART_CLOCK
};	



struct scu_func_s scu_func[] = {                                                                       
{"SCU_MPLL_SET",                                sq_scu_mpll_set},                       
{"SCU_MPLL_GET",                                sq_scu_mpll_get},                       
{"SCU_MPLL_NORMAL",                             sq_scu_mpll_normal},                    
{"SCU_MPLL_POWER_DOWN",                         sq_scu_mpll_power_down},                
{"SCU_MPLL_STATUS_GET",                         sq_scu_mpll_status_get},                
{"SCU_MCLK_ENABLE",                             sq_scu_mclk_enable},                    
{"SCU_MCLK_DISABLE",                            sq_scu_mclk_disable},                   
{"SCU_ACLK_ENABLE",                             sq_scu_aclk_enable},                    
{"SCU_ACLK_DISABLE",                            sq_scu_aclk_disable},                   
{"SCU_CHIP_ID",                                 sq_scu_chip_id},                        
{"SCU_UPLL_SET",                                sq_scu_upll_set},                       
{"SCU_UPLL_GET",                                sq_scu_upll_get},                                            
{"SCU_UPLL_NORMAL",                             sq_scu_upll_normal},                    
{"SCU_UPLL_POWER_DOWN",                         sq_scu_upll_power_down},                
{"SCU_UPLL_STATUS_GET",                         sq_scu_upll_status_get},                               
{"SCU_CLOCK_RATIO_SET",                         sq_scu_clock_ratio_set},                
{"SCU_CLOCK_RATIO_GET",                         sq_scu_clock_ratio_get},                
{"SCU_PLL_LOCK_PERIOD_SET",                     sq_scu_pll_lock_period_set},            
{"SCU_PLL_LOCK_PERIOD_GET",                     sq_scu_pll_lock_period_get},            
{"SCU_ADC_CLK_DIV_SET",                         sq_scu_adc_clk_div_set},                
{"SCU_ADC_CLK_DIV_GET",                         sq_scu_adc_clk_div_get},                
{"SCU_UART_CLK_24_SET",                         sq_scu_uart_clk_24_set},                
{"SCU_UART_CLK_UPLL_SET",                       sq_scu_uart_clk_upll_set},              
{"SCU_UART_CLK_UPLL_2_SET",                     sq_scu_uart_clk_upll_2_set},            
{"SCU_UART_CLK_UPLL_4_SET",                     sq_scu_uart_clk_upll_4_set},            
{"SCU_UART_CLK_GET",                            sq_scu_uart_clk_get},                   
{"SCU_SDRAM_BUS_WIDTH_STATUS",                  sq_scu_sdram_bus_width_status},         
{"SCU_DCM_MODE_STATUS",                         sq_scu_dcm_mode_status},             
{"SCU_USB_MODE_STATUS",                         sq_scu_usb_mode_status},                
{"SCU_BOOT_SOURCE_STATUS",                      sq_scu_boot_source_status},             
{"SCU_TPS_MAC_STATUS",                          sq_scu_tps_mac_status},                                
{"SCU_AUTO_BOOT_FAIL_STATUS",                   sq_scu_auto_boot_fail_status},          
{"SCU_PW_STANDBYWFI_ENABLE",                    sq_scu_pw_standbywfi_enable},           
{"SCU_STOP_MODE_ENABLE",                        sq_scu_stop_mode_enable},               
{"SCU_SLOW_MODE_DISABLE",                       sq_scu_slow_mode_disable},              
{"SCU_SW_RESET",                                sq_scu_sw_reset},                       
{"SCU_SW_REMAP",                                sq_scu_sw_remap},                       
{"SCU_GET_PLL_CLOCK",                           get_pll_clock},                         
{"SCU_DEV_ENABLE",                              sq_scu_dev_enable},                     
{"SCU_DEV_DISABLE",                             sq_scu_dev_disable},                    
{"SCU_LCDC_CLK_INPUT_MPLL_OUTPPUT",             sq_scu_lcdc_clk_input_mpll_outpput},    
{"SCU_LCDC_CLK_INPUT_MPLL_XIN",                 sq_scu_lcdc_clk_input_mpll_xin},        
{"SCU_HDMA_REQ45_SPI0",                         sq_scu_hdma_req45_spi0},                
{"SCU_HDMA_REQ45_SPI1",                         sq_scu_hdma_req45_spi1},                
{"SCU_UHC0_48CLOCK_INPUT_UPLL ",                sq_scu_uhc0_48clock_input_upll},        
{"SCU_UHC0_48CLOCK_INPUT_OTG_PHY ",             sq_scu_uhc0_48clock_input_otg_phy},     
{"SCU_UHC1_48CLOCK_INPUT_UPLL ",                sq_scu_uhc1_48clock_input_upll},        
{"SCU_UHC1_48CLOCK_INPUT_OTG_PHY ",             sq_scu_uhc1_48clock_input_otg_phy},     
{"SCU_HDMA_REQ01_UART",                         sq_scu_hdma_req01_uart},                
{"SCU_HDMA_REQ23_UART",                         sq_scu_hdma_req23_uart},                
{"SCU_WDT_RESET_ENABLE",                        sq_scu_wdt_reset_enable},               
{"SCU_SW_RESET_ENABLE",                         sq_scu_sw_reset_enable},                
{"SCU_NFIQ_POLARITY_HIGH",                      sq_scu_nfiq_polarity_high},             
{"SCU_INFO0_SET",                               sq_scu_info0_set},                      
{"SCU_INFO0_GET",                               sq_scu_info0_get},                      
{"SCU_INFO1_SET",                               sq_scu_info1_set},                      
{"SCU_INFO1_GET",                               sq_scu_info1_get},                      
{"SCU_INFO2_SET",                               sq_scu_info2_set},                      
{"SCU_INFO2_GET",                               sq_scu_info2_get},                      
{"SCU_INFO3_SET",                               sq_scu_info3_set},                      
{"SCU_INFO3_GET",                               sq_scu_info3_get},                      
{"GET_CPU_CLOCK",                               sq_get_cpu_clock},                      
{"GET_AHB_CLOCK",                               sq_get_ahb_clock},                      
{"GET_APB_CLOCK",                               sq_get_apb_clock},                      
{"GET_UART_CLOCK",                              sq_get_uart_clock} 
};




extern  int sq_dia_scu(unsigned int  func,unsigned long  arg)
{

	unsigned int *p;
	struct scu_func_s *f;
	p=(unsigned int  *)arg;
	*(p+1) = 0;
	
				
	DIA_SCU_DBG("%s",scu_func[func].msg);	
	f = (struct scu_func_s *)&scu_func[func];
		
	*(p+1) = f->func(*p);	
	
	DIA_SCU_DBG("\n[scu]func=0x%x in=0x%x out=%x ,out=%d \n",func,*p,*(p+1),*(p+1));	
	return 0;
}




MODULE_AUTHOR("Channing Lan");
MODULE_DESCRIPTION("SQ driver dia scu");
MODULE_LICENSE("GPL");



