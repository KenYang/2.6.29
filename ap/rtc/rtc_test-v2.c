/********************************************************************************
*	Real Time Clock Driver Test/Example Program
*
*	Compile with:
*		gcc -s -Wall -Wstrict-prototypes rtc.c -o rtc
*
*	Copyright (C) 1996, Paul Gortmaker.
*	Released under the GNU General Public License, version 2,
*	included herein by reference.
*
*   Version      : 2,0,0,1
*   History      : 
*    1. 2006/08/25 ryan chen create this file
*    2. 2006/12/25 cyli modify this file
*    3. 2010/08/13 cyli modify month setting for RTC_WKALM_SET and RTC_WKALM_RD
*    
********************************************************************************/

#define SOCLE_VERSION "01.01"
#include <stdio.h> 
#include <linux/rtc.h> 
#include <sys/ioctl.h> 
#include <sys/time.h> 
#include <sys/types.h> 
#include <fcntl.h> 
#include <unistd.h> 
#include <errno.h>
#include <stdlib.h>	//leonid+ for exit warning

//Socle RTC Ioctl
#define RTC_PWR_OFF		_IO('p', 0x23)


static void menu(void);
static int auto_alarm_test_tm(int fd);
static int auto_pwoff_alarm_test_tm(int fd);
static int auto_alarm_test_tmdt(int fd);
static int auto_pwoff_alarm_test_tmdt(int fd);
static int auto_alarm_test_tm_manually(int fd);


static void
menu(void)
{
	printf("\n============ Real Time Clock (RTC) Application ============\n");
	printf("1. Auto Alarm Test with Time\n");
	printf("2. Auto Poweroff Alarm Test with Time\n");
	printf("3. Auto Alarm Test with Time and Date\n");
	printf("4. Auto Poweroff Alarm Test with Time and Date\n");
	printf("5. Auto Alarm Test with Time manually\n");
	printf("\n");
	printf("6. RTC_SET_TIME\n");
	printf("7. RTC_RD_TIME\n");
	printf("8. RTC_ALM_SET\n");
	printf("9. RTC_ALM_READ\n");
	printf("a. RTC_WKALM_SET\n");
	printf("b. RTC_WKALM_RD\n");
	printf("c. RTC_IRQP_SET\n");
	printf("d. RTC_IRQP_READ\n");
	printf("e. Enter RTC Power Off Mode\n");
	printf("\n");
	printf("?. Display Menu\n");
	printf("X. Exit\n");
	printf("=========== (C) 2003-2006 Socle Technology Corp. ==========\n");
}


int main (int argc,	char *argv[])
{
	int fd, retval;
	unsigned long freq;
	char test_item;
	struct rtc_time tm;
	struct rtc_wkalrm alarm;
	
	if( argc >= 2 && !strcmp(argv[1], "-v")) {
		printf("version: %s\n", SOCLE_VERSION);
		return 0;
	}
	
	fd = open ("/dev/rtc0", O_RDONLY);
	if (fd == -1) { 
		perror("Can't open /dev/rtc0!\n");
		exit(errno); 
	}
	
	menu();
	
	while(1){
		printf("RTC>");

		test_item = fgetc(stdin);
		
		if (0xa == test_item)	//if press enter key
			continue;
		else
			if(0xa != fgetc(stdin)){	//if input more than 2 char
				while(0xa != fgetc(stdin));
				menu();
				continue;
			}
		
		switch(test_item){
		case '1':	//Auto Alarm Test with Time
			auto_alarm_test_tm(fd);
			break;

		case '2':	//Auto Poweroff Alarm Test with Time
			auto_pwoff_alarm_test_tm(fd);
			break;
			
		case '3':	//Auto Alarm Test with Time and Date
			auto_alarm_test_tmdt(fd);
			break;
			
		case '4':	//Auto Poweroff Alarm Test with Time and Date
			auto_pwoff_alarm_test_tmdt(fd);
			break;

		case '5':	//Auto Alarm Test with Time manually
			auto_alarm_test_tm_manually(fd);
			break;

		case '6':	//RTC_SET_TIME
			printf("Please input the RTC time, ex: on Monday, 5:30:40 PM\n");
			printf("Set Time = 1:17:30:40, Your Time = ");
			scanf("%d:%d:%d:%d", &tm.tm_wday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
			fgetc(stdin);
			printf("Input Time = w(%d) %02d:%02d:%02d\n", tm.tm_wday, tm.tm_hour, tm.tm_min, tm.tm_sec);

			printf("Please input the RTC date, ex: on 2006-05-23\n");
			printf("Set Date = 2006-05-23, Your Date = ");
			scanf("%d-%d-%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday);
			tm.tm_mon--;
			tm.tm_year -= 1900;
			fgetc(stdin);
			printf("Input Date = %d/%d/%d\n", tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday);
			
			retval = ioctl(fd, RTC_SET_TIME, &tm); 
			if (retval == -1) { 
				perror("ioctl RTC_SET_TIME fail!\n");
				exit(errno); 
			}

			break;

		case '7':	//RTC_RD_TIME
			retval = ioctl(fd, RTC_RD_TIME, &tm); 
			if (retval == -1) { 
				perror("ioctl RTC_RD_TIME fail!\n");
				exit(errno); 
			}

			printf("RTC date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", tm.tm_year+1900,
						tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_wday);

			break;
			
		case '8':	//RTC_ALM_SET
			printf("Please input the RTC alarm time, ex: 5:30:40 PM\n");
			printf("Set Time = 17:30:40, Your Time = ");
			scanf("%d:%d:%d", &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
			fgetc(stdin);
			printf("Input Time = %02d:%02d:%02d\n", tm.tm_hour, tm.tm_min, tm.tm_sec);

			retval = ioctl(fd, RTC_ALM_SET, &tm); 
			if (retval == -1) { 
				perror("ioctl RTC_ALM_SET fail!\n");
				exit(errno); 
			}
			
			break;
			
		case '9':	//RTC_ALM_READ
			retval = ioctl(fd, RTC_ALM_READ, &tm); 
			if (retval == -1) { 
				perror("ioctl RTC_ALM_READ fail!\n");
				exit(errno); 
			}
			
			printf("RTC alarm date/time is %02d:%02d:%02d\n", tm.tm_hour, tm.tm_min, tm.tm_sec);
			
			break;
			
		case 'a':	//RTC_WKALM_SET
		case 'A':
			printf("Please input the RTC alarm time, ex: on Monday, 5:30:40 PM\n");
			printf("Set Time = 1:17:30:40, Your Time = ");
			scanf("%d:%d:%d:%d", &alarm.time.tm_wday, &alarm.time.tm_hour,
								&alarm.time.tm_min, &alarm.time.tm_sec);
			fgetc(stdin);
			printf("Input Time = %d:%02d:%02d:%02d\n", alarm.time.tm_wday, alarm.time.tm_hour,
								alarm.time.tm_min, alarm.time.tm_sec);

			printf("Please input the RTC alarm date, ex: on 2006-05-23\n");
			printf("Set Date = 2006-05-23, Your Date = ");
			scanf("%d-%d-%d", &alarm.time.tm_year, &alarm.time.tm_mon, &alarm.time.tm_mday);
			alarm.time.tm_mon--;
			alarm.time.tm_year -= 1900;
			fgetc(stdin);
			printf("Input Date = %d/%d/%d\n", alarm.time.tm_year+1900, alarm.time.tm_mon+1, alarm.time.tm_mday);

			retval = ioctl(fd, RTC_WKALM_SET, &alarm); 
			if (retval == -1) { 
				perror("ioctl RTC_WKALM_SET fail!\n");
				exit(errno); 
			}

			break;
			
		case 'b':	//RTC_WKALM_RD
		case 'B':
			retval = ioctl(fd, RTC_WKALM_RD, &alarm); 
			if (retval == -1) { 
				perror("ioctl RTC_WKALM_RD fail!\n");
				exit(errno); 
			}

			printf("RTC alarm date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", alarm.time.tm_year+1900, 
								alarm.time.tm_mon+1, alarm.time.tm_mday, alarm.time.tm_hour, alarm.time.tm_min,
								alarm.time.tm_sec, alarm.time.tm_wday);

			break;
			
		case 'c':	//RTC_IRQP_SET
		case 'C':
			printf("Periodic IRQ rate = ");
			scanf("%ld", &freq);
			fgetc(stdin);
			
			retval = ioctl(fd, RTC_IRQP_SET, freq); 
			if (retval == -1) { 
				perror("ioctl RTC_IRQP_SET fail!\n");
				exit(errno); 
			}

			break;
			
		case 'd':	//RTC_IRQP_READ
		case 'D':
			retval = ioctl(fd, RTC_IRQP_READ, &freq); 
			if (retval == -1) { 
				perror("ioctl RTC_IRQP_READ fail!\n");
				exit(errno); 
			}

			printf("Periodic IRQ rate is %ldHz\n", freq);

			break;
			
		case 'e':	//Enter RTC Power Off Mode
		case 'E':
			printf("Reminder: You must set up the wakeup alarm first!\n");
			printf("Are you sure? (y/n)... ");
			test_item = fgetc(stdin);
			fgetc(stdin);
			
			if (('y' == test_item) || ('Y' == test_item)) {
				printf("Sleeping...\n");
				retval = ioctl(fd, RTC_PWR_OFF, 0); 
				if (retval == -1) { 
					perror("ioctl RTC_PWR_OFF fail!\n");
					exit(errno); 
				}
			}

			break;

		case '?':
			menu();
			break;

		case 'x':
		case 'X':
			close(fd);
			return 0;
			
		default:
			printf("Please select again! [%c]\n", test_item);
			
		}	
	}

	return 0;
} /* end main */ 


static int
auto_alarm_test_tm(int fd)
{
	int retval; 
	unsigned long freq;
	struct rtc_time rtc_tm;

	/* Read the RTC time/date */ 
	retval = ioctl(fd, RTC_RD_TIME, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_RD_TIME fail!\n");
		exit(errno); 
	}
	rtc_tm.tm_mon++;
	fprintf(stderr, "Current RTC date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", rtc_tm.tm_year+1900,
				rtc_tm.tm_mon, rtc_tm.tm_mday, rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec, rtc_tm.tm_wday);

	/* Set the alarm to 10 sec in the future, and check for rollover */ 
	rtc_tm.tm_sec += 10; 
	if (rtc_tm.tm_sec >= 60) { 
		rtc_tm.tm_sec %= 60;
		rtc_tm.tm_min++; 
	}

	if  (rtc_tm.tm_min == 60) { 
		rtc_tm.tm_min = 0;
		rtc_tm.tm_hour++; 
	} 
	
	if  (rtc_tm.tm_hour == 24) 
		rtc_tm.tm_hour = 0;
							
	retval = ioctl(fd, RTC_ALM_SET, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_ALM_SET fail!\n");
		exit(errno); 
	}
	
	/* Read the current alarm settings */ 
	retval = ioctl(fd, RTC_ALM_READ, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_ALM_READ fail!\n");
		exit(errno); 
	}
	fprintf(stderr, "Alarm time now is set to %02d:%02d:%02d\n", rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
// 	fprintf(stderr, "Alarm time now is set to %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", rtc_tm.tm_year+1900,
// 				rtc_tm.tm_mon, rtc_tm.tm_mday, rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec, rtc_tm.tm_wday);


// 	/* Enable alarm interrupts */ 
// 	retval = ioctl(fd, RTC_AIE_ON, 0); 
// 	if (retval == -1) { 
// 		perror("ioctl RTC_AIE_ON fail!\n");
// 		exit(errno); 
// 	}

	fprintf(stderr, "Waiting 10 seconds for alarm...\n");

// 	/* Disable alarm interrupts */ 
// 	retval = ioctl(fd, RTC_AIE_OFF, 0); 
// 	if (retval == -1) { 
// 		perror("ioctl");
// 		exit(errno); 
// 	}

	/* Read periodic IRQ rate */ 
	retval = ioctl(fd, RTC_IRQP_READ, &freq); 
	if (retval == -1) { 
		perror("ioctl");
		exit(errno); 
	} 
	fprintf(stderr, "Periodic IRQ rate is %ldHz\n", freq);
	
	return 0;
}


static int
auto_pwoff_alarm_test_tm(int fd)
{
	int retval;
	struct rtc_time rtc_tm;

	/* Read the RTC time/date */ 
	retval = ioctl(fd, RTC_RD_TIME, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_RD_TIME fail!\n");
		exit(errno); 
	}
	rtc_tm.tm_mon++;
	fprintf(stderr, "Current RTC date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", rtc_tm.tm_year+1900,
				rtc_tm.tm_mon, rtc_tm.tm_mday, rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec, rtc_tm.tm_wday);

	/* Set the alarm to 10 sec in the future, and check for rollover */ 
	rtc_tm.tm_sec += 10; 
	if (rtc_tm.tm_sec >= 60) { 
		rtc_tm.tm_sec %= 60;
		rtc_tm.tm_min++; 
	}

	if  (rtc_tm.tm_min == 60) { 
		rtc_tm.tm_min = 0;
		rtc_tm.tm_hour++; 
	} 
	
	if  (rtc_tm.tm_hour == 24) 
		rtc_tm.tm_hour = 0;
							
	retval = ioctl(fd, RTC_ALM_SET, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_ALM_SET fail!\n");
		exit(errno); 
	}
	
	/* Read the current alarm settings */ 
	retval = ioctl(fd, RTC_ALM_READ, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_ALM_READ fail!\n");
		exit(errno); 
	}
	fprintf(stderr, "Alarm time now is set to %02d:%02d:%02d\n", rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
// 	fprintf(stderr, "Alarm time now is set to %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", rtc_tm.tm_year+1900,
// 				rtc_tm.tm_mon, rtc_tm.tm_mday, rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec, rtc_tm.tm_wday);

	fprintf(stderr, "Waiting 10 seconds for poweroff alarm...\n");

	/* Enter RTC Power Off Mode */
	fprintf(stderr, "Sleeping...\n");
	retval = ioctl(fd, RTC_PWR_OFF, 0); 
	if (retval == -1) { 
		perror("ioctl RTC_PWR_OFF fail!\n");
		exit(errno); 
	}

	return 0;
}

static int
auto_alarm_test_tmdt(int fd)
{
	int retval; 
	unsigned long freq;
	struct rtc_time tm;
	struct rtc_wkalrm alarm;
	
	//Set date/time: 2006-12-31 23:59:55 w(2)
	tm.tm_wday = 2;
	tm.tm_year = 2006 - 1900;
	tm.tm_mon = 12 - 1;
	tm.tm_mday = 31;
	tm.tm_hour = 23;
	tm.tm_min = 59;
	tm.tm_sec = 55;
	
	fprintf(stderr, "Set RTC date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", tm.tm_year+1900,
				tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_wday);
	
	retval = ioctl(fd, RTC_SET_TIME, &tm); 
	if (retval == -1) { 
		perror("ioctl RTC_SET_TIME fail!\n");
		exit(errno); 
	}

	//Set alarm date/time: 2007-01-01 00:00:05 w(3)
	alarm.time.tm_wday = 3;
	alarm.time.tm_year = 2007 - 1900;
	alarm.time.tm_mon = 1 - 1;
	alarm.time.tm_mday = 1;
	alarm.time.tm_hour = 0;
	alarm.time.tm_min = 0;
	alarm.time.tm_sec = 5;

	fprintf(stderr, "Set RTC alarm date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", alarm.time.tm_year+1900, 
						alarm.time.tm_mon+1, alarm.time.tm_mday, alarm.time.tm_hour, alarm.time.tm_min,
						alarm.time.tm_sec, alarm.time.tm_wday);
	
	retval = ioctl(fd, RTC_WKALM_SET, &alarm); 
	if (retval == -1) { 
		perror("ioctl RTC_WKALM_SET fail!\n");
		exit(errno); 
	}

	fprintf(stderr, "Waiting 10 seconds for alarm...\n");


	/* Read periodic IRQ rate */ 
	retval = ioctl(fd, RTC_IRQP_READ, &freq); 
	if (retval == -1) { 
		perror("ioctl");
		exit(errno); 
	} 
	fprintf(stderr, "Periodic IRQ rate is %ldHz\n", freq);
	
	return 0;
}

static int
auto_pwoff_alarm_test_tmdt(int fd)
{
	int retval; 
	struct rtc_time tm;
	struct rtc_wkalrm alarm;
	
	//Set date/time: 2006-12-31 23:59:55 w(2)
	tm.tm_wday = 2;
	tm.tm_year = 2006 - 1900;
	tm.tm_mon = 12 - 1;
	tm.tm_mday = 31;
	tm.tm_hour = 23;
	tm.tm_min = 59;
	tm.tm_sec = 55;
	
	fprintf(stderr, "Set RTC date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", tm.tm_year+1900,
				tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, tm.tm_wday);
	
	retval = ioctl(fd, RTC_SET_TIME, &tm); 
	if (retval == -1) { 
		perror("ioctl RTC_SET_TIME fail!\n");
		exit(errno); 
	}

	//Set alarm date/time: 2007-01-01 00:00:05 w(3)
	alarm.time.tm_wday = 3;
	alarm.time.tm_year = 2007 - 1900;
	alarm.time.tm_mon = 1 - 1;
	alarm.time.tm_mday = 1;
	alarm.time.tm_hour = 0;
	alarm.time.tm_min = 0;
	alarm.time.tm_sec = 5;

	fprintf(stderr, "Set RTC alarm date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", alarm.time.tm_year+1900, 
						alarm.time.tm_mon+1, alarm.time.tm_mday, alarm.time.tm_hour, alarm.time.tm_min,
						alarm.time.tm_sec, alarm.time.tm_wday);
	
	retval = ioctl(fd, RTC_WKALM_SET, &alarm); 
	if (retval == -1) { 
		perror("ioctl RTC_WKALM_SET fail!\n");
		exit(errno); 
	}

	fprintf(stderr, "Waiting 10 seconds for poweroff alarm...\n");

	/* Enter RTC Power Off Mode */
	fprintf(stderr, "Sleeping...\n");
	retval = ioctl(fd, RTC_PWR_OFF, 0); 
	if (retval == -1) { 
		perror("ioctl RTC_PWR_OFF fail!\n");
		exit(errno); 
	}
	
	return 0;
}



static int
auto_alarm_test_tm_manually(int fd)
{
	int retval, w_tm_s; 
	unsigned long freq;
	struct rtc_time rtc_tm;

	printf("Please input waiting time (seconds) = ");
	scanf("%d", &w_tm_s);
	fgetc(stdin);
	
	/* Read the RTC time/date */ 
	retval = ioctl(fd, RTC_RD_TIME, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_RD_TIME fail!\n");
		exit(errno); 
	}
	rtc_tm.tm_mon++;
	fprintf(stderr, "Current RTC date/time is %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", rtc_tm.tm_year+1900,
				rtc_tm.tm_mon, rtc_tm.tm_mday, rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec, rtc_tm.tm_wday);

	/* Set the alarm to w_tm_s sec in the future, and check for rollover */ 
	rtc_tm.tm_sec += w_tm_s; 
	if (rtc_tm.tm_sec >= 60) { 
		rtc_tm.tm_sec %= 60;
		rtc_tm.tm_min++; 
	}

	if  (rtc_tm.tm_min == 60) { 
		rtc_tm.tm_min = 0;
		rtc_tm.tm_hour++; 
	} 
	
	if  (rtc_tm.tm_hour == 24) 
		rtc_tm.tm_hour = 0;
							
	retval = ioctl(fd, RTC_ALM_SET, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_ALM_SET fail!\n");
		exit(errno); 
	}
	
	/* Read the current alarm settings */ 
	retval = ioctl(fd, RTC_ALM_READ, &rtc_tm); 
	if (retval == -1) { 
		perror("ioctl RTC_ALM_READ fail!\n");
		exit(errno); 
	}
	fprintf(stderr, "Alarm time now is set to %02d:%02d:%02d\n", rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
// 	fprintf(stderr, "Alarm time now is set to %04d-%02d-%02d, %02d:%02d:%02d w(%d)\n", rtc_tm.tm_year+1900,
// 				rtc_tm.tm_mon, rtc_tm.tm_mday, rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec, rtc_tm.tm_wday);


// 	/* Enable alarm interrupts */ 
// 	retval = ioctl(fd, RTC_AIE_ON, 0); 
// 	if (retval == -1) { 
// 		perror("ioctl RTC_AIE_ON fail!\n");
// 		exit(errno); 
// 	}

	fprintf(stderr, "Waiting %d seconds for alarm...\n", w_tm_s);

// 	/* Disable alarm interrupts */ 
// 	retval = ioctl(fd, RTC_AIE_OFF, 0); 
// 	if (retval == -1) { 
// 		perror("ioctl");
// 		exit(errno); 
// 	}

	/* Read periodic IRQ rate */ 
	retval = ioctl(fd, RTC_IRQP_READ, &freq); 
	if (retval == -1) { 
		perror("ioctl");
		exit(errno); 
	} 
	fprintf(stderr, "Periodic IRQ rate is %ldHz\n", freq);
	
	return 0;
}



