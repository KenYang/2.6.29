#ifndef __ASM_ARCH_SOCLE_H
#define __ASM_ARCH_SOCLE_H

// SOCLE Service Driver
extern void __init socle_add_device_gpio(void);			//20100622 cyli+ for gpio power management

extern void __init socle_add_device_flash(void);
extern void __init socle_add_device_rtc(void);
extern void __init ldk_add_device_scu(void);
extern void __init socle_add_device_adc(void);
extern void __init socle_add_device_i2c(void);
extern void __init socle_add_device_snd_i2s(void);
extern void __init socle_add_device_eth(void);
extern void __init socle_add_device_udc(void);
extern void __init socle_add_device_nand(void);

extern void __init socle_add_device_ehci(void);
extern void __init socle_add_device_ohci(void);

extern void __init socle_add_device_watchdog(void);

extern void __init socle_sdmmc_add_device_mmc(void);
extern void __init socle_spi_add_device(void);

extern void __init socle_add_device_clcd(void); //ARM

extern void __init socle_add_lcd_device(void); //cade

extern void __init socle_add_device_hdma_pseudo(void);

extern void __init cdk_add_device_kpd(void);
extern void __init pdk_add_device_kpd(void);
extern void __init socle_add_device_fake_battery(void);
extern void __init socle_add_device_fake_rfkill(void);

//ADC dependence device 
extern void __init socle_add_device_adc_battery(void);
extern void __init socle_add_device_adc_ts(void);

extern struct sys_timer socle_timer;

extern void __init socle_add_device_inr(void);

extern void __init socle_add_device_vop(void);
extern void __init socle_add_device_vip(void);


extern void __init socle_add_device_mailbox(void);
//msmv ******************************************
extern void __init  socle_add_device_current_sense(void);	
extern void __init  socle_add_device_msmv(void);
extern void __init  msmv_add_device_kpd(void);		
extern void __init  f75111_add_device_kpd(void);

#ifdef CONFIG_ANDROID_SYSTEM
extern void __init  socle_add_device_android_adb(void);          //20090527 leonid+ for adb
#endif


extern void __init  socle_add_device_otg_udc(void);
extern void __init socle_sdhc_add_device_mci(void);
extern void __init socle_add_device_sdhc_slave(void);

extern void __init socle_add_device_lcd_lighter(void);		//leonid+
extern void __init socle_add_device_led_backlight(void);

extern void __init socle_add_device_nfc(void);

#if defined(CONFIG_SQ_GDR)
extern void __init socle_del_device_nfc(void);
#endif 


extern void __init socle_add_device_on2(void);

#endif
