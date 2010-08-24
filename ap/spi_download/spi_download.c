#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>


#define SPI_DL_OPEN()	\
	fd = open("/dev/spi_download",O_RDWR);	\
	if ( fd== -1 ) {			\
		perror("\nopen faild");		\
		exit(-1);			\
	}
	

#define SPI_DL_CLOSE()	\
	rc = close(fd);		\
	if ( rc == -1 ) {			\
		perror("\nclose faild");	\
		exit(-1);			\
	}
	
#define SPI_DL_WRITE(buf,len)			\
	rc = write(fd, buf, len);		\
	if ( rc == -1 ) {                       \
		perror("\nread faild\n");       \
		exit(-1);                       \
	}


#define SPI_DL_READ(buf)			\
	rc = read(fd, buf, 0);			\
	if ( rc == -1 ) {			\
		perror("\nread faild\n");	\
		exit(-1);			\
	}


#define SPI_DL_IOCTL(cmd,buf)			\
	rc = ioctl(fd,cmd ,buf);		\
	if ( rc == -1 ) {			\
		perror("\nioctrl faild\n");	\
		exit(-1);			\
	}




#define SPI_FILE_OPEN()	\
	fd = open("/home/data",O_RDONLY);	\
	if ( fd == -1 ) {			\
		perror("\nopen faild");		\
		exit(-1);			\
	}



#define SPI_FILE_CLOSE()			\
	rc = close(file_fd);		\
	if ( rc == -1 ) {			\
		perror("\nclose faild");	\
		exit(-1);			\
	}




/******************************************************************************
*
******************************************************************************/

unsigned int dl_write_cmd(unsigned int func,unsigned int data)
{
	int fd;
	int rc = 0,cmd;
	unsigned int p[2];
	
	p[0] = data;
//	p[1] = 0;
	SPI_DL_OPEN();
	cmd = func;
	SPI_DL_IOCTL(cmd,&p);	
	SPI_DL_CLOSE();
//	printf("\n data %d return 0x%x \n",p[0],p[1]);
	return p[1];
}

unsigned int dl_read_cmd(unsigned int func)
{
	int fd;
	int rc = 0,data,cmd;
	
	SPI_DL_OPEN();
	cmd = func;
	SPI_DL_IOCTL(cmd,&data);	
	SPI_DL_CLOSE();
	
	return data;	
}

/*
unsigned int dl_write_buf(unsigned int buf,unsigned int len)
{
	int fd;
	int rc = 0;	
	unsigned int *p;
	
	p = buf;

	SPI_DL_OPEN();
	SPI_DL_WRITE(p,len);		
	SPI_DL_CLOSE();
//	printf("\n data %d return 0x%x \n",p[0],p[1]);
	return 0;
}

*/


#define SET_PROB_LOW	1
#define SET_PROB_HIGH	2
#define GET_INITB	3
#define GET_DONE	4
#define SET_RST_LOW	5
#define SET_RST_HIGH	6
#define SET_RST		7
#define GET_MAX_BUF     8
#define SET_PROB_LH	9

#define dl_set_prob_low()			dl_write_cmd(SET_PROB_LOW,0)  
#define dl_set_prob_high()			dl_write_cmd(SET_PROB_HIGH,0) 
#define dl_get_initb()				dl_read_cmd(GET_INITB) 
#define dl_get_done()				dl_read_cmd(GET_DONE) 
#define dl_set_rst_low()			dl_write_cmd(SET_RST_LOW,0) 
#define dl_set_rst_high()			dl_write_cmd(SET_RST_HIGH,0) 
#define dl_set_rst()				dl_write_cmd(SET_RST,0) 
#define dl_get_buf_size()			dl_read_cmd(GET_MAX_BUF) 
#define dl_set_prob_low_high()			dl_write_cmd(SET_PROB_LH,0) 

#define SPI_CTRL_GPIO_EN 0
#define	MAX_BUFF_SIZE		65536


int main ( int argc, char *argv[] )
{
	int fd,file_fd,total_r=0,i;
	int rc = 10,data,cmd,c,len,buf_size=0,file_size=0;
    	//char buf[128]={0};
     	char *p,wait_done_count=0;   	
    	int ret;
 	char *buf = (char *)malloc(MAX_BUFF_SIZE);

	printf("\n done = 0x%x",dl_get_done());	
 do{
/*
Step1:   
     PROG_B  先送Low ( 大於 1us ) 再送high.
*/
//while(1){
	dl_set_prob_low_high();
//	}

/*
Step2:
     Check INIT_B 直到它為High.
*/
	while(dl_get_initb() == 0);
 	printf("\n done = 0x%x",dl_get_done()); 
/*
Step3:
     讀檔案, 轉換成SPI data送出. MSB first ?  byte or word ?  忘了, 再請你試一下.
*/
	
		file_fd = open("/home/sq8006gdr.bit",O_RDONLY);	
		
		if ( file_fd == -1 ) {		
			perror("\nopen faild");		
			exit(-1);		
		}
	
		do {
	
	
			total_r = read(file_fd , buf, MAX_BUFF_SIZE);
//			printf("\ntotal_r = %d ",total_r);						
			if ( total_r == 0 ) {			
				printf("\n^^ read ok\n");	
				
				SPI_FILE_CLOSE();			
				break;			
			}
			if ( total_r == -1 ) {			
				perror("\n^^ read err\n");	
				
				SPI_FILE_CLOSE();			
				break;			
			}
							
	
/*
			c = total_r;//total_r;
			p = buf;
			while(c--) {
				printf("%c",*p++);
			}	
			printf("\n");	
*/
	
			//total_r = read(file_fd,buf,sizeof(buf));
	
			file_size += total_r;
			//printf("\ntotal_r = %d file_size = %d ",total_r,file_size);
				
			//while(1){	
			if(total_r) {
				SPI_DL_OPEN();
				SPI_DL_WRITE(buf,total_r);	
				SPI_DL_CLOSE();
			}//}
			
			//dl_write_buf(buf,buf_size);

		} while(1);

		
	
/*
Step4:
     送完後,  check DONE 為high 代表完成, 若為Low, 回到 Step1 retry
 */    
         //printf("^^ wait done\n");
         wait_done_count++;
         
         if(wait_done_count > 10) {
         	printf("ERR:No Done\n");
         	return -1;
         	
         }
         
	} while(dl_get_done() == 0);
	//} while(1);	
	
/*	
	while(dl_get_done() == 0){
	printf("\n done = 0x%x",dl_get_done()); 
	}
*/	
	//printf("\n done = 0x%x",dl_get_done());	
/*	     
Step5:
     FPGA_rst 送High, 維持約100ms後, 送Low. 然後大功告成!!
*/ 	
	dl_set_rst();	
	printf("^^ send rst ok\n");	
	printf("\ntotal_r = %d file_size = %d ",total_r,file_size);
	
	free(buf);	
	
				
	return 0;
}


