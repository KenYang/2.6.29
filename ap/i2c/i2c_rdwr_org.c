/* /home/mooij/test_i2c.c
*
* Copyright (c) 2010 SQ Tech. CO., LTD. <http://www.sq.com.tw/>
* mooijKuo <mooijKuo@sq.com.tw>
*
* USER layer - i2c
*
* THIS SOFTWARE IS PROVIDED UNDER LICENSE AND CONTAINS PROPRIETARY
* AND CONFIDENTIAL MATERIAL WHICH IS THE PROPERTY OF SQ TECH.
*
* Modifications:
* $Id$
*
*/

#include <stdio.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <assert.h>
#include <string.h>


#define I2C_RETRIES		0x0701
#define I2C_TIMEOUT		0x0702
#define I2C_I2C_SLAVE_FORCE	0x0706
#define I2C_RDWR		0x0707

#define I2C_M_TEN	0x0010
#define I2C_M_RD	0x0001

struct i2c_msg
{
	__u16 addr;
	__u16 flags;
	__u16 len;
	__u8 *buf;
};

struct i2c_rdwr_ioctl_data
{
	struct i2c_msg *msgs;
	int nmsgs;
};


	int fd,ret;


char i2c_read_byte(char id,char addr)
{

	struct i2c_rdwr_ioctl_data alldata;
	
	
	alldata.nmsgs = 2;
	alldata.msgs = (struct i2c_msg*)malloc(alldata.nmsgs * sizeof(struct i2c_msg));
	if(!alldata.msgs){
	printf("Memory alloc error!\n");
	close(fd);
	return 0;
	}

	(alldata.msgs[0]).addr   = id;                   //slave address
	(alldata.msgs[0]).flags  = 0;                      //flag = 0(write) , 1(read)
	(alldata.msgs[0]).len    = 1;
	(alldata.msgs[0]).buf    = (__u8 *)malloc(1);
	(alldata.msgs[0]).buf[0] = addr;                   //sub address (offset)
	(alldata.msgs[1]).addr   = id;
	(alldata.msgs[1]).flags  = I2C_M_RD;
	(alldata.msgs[1]).len    = 1;
	(alldata.msgs[1]).buf    = (__u8 *)malloc(1);
	(alldata.msgs[1]).buf[0] = 0;

	ret = ioctl(fd,I2C_RDWR,(unsigned long)&alldata);
	if(ret < 0) {
		printf("R ER id = 0x%2x ,address = 0x%2x val=0x%2x ,ret=0x%x\n",id,addr,(alldata.msgs[1]).buf[0],ret);	
	} else {
		printf("R OK id = 0x%2x ,address = 0x%2x val=0x%2x\n",id,addr,(alldata.msgs[1]).buf[0]);
	}

	free((alldata.msgs[0]).buf);
	free((alldata.msgs[1]).buf);
	free(alldata.msgs);	
	
	return alldata.msgs[0].buf[1];
}


int i2c_write_byte(char id,char addr,char val)
{

	struct i2c_rdwr_ioctl_data alldata;
	

	alldata.nmsgs = 1;
	alldata.msgs = (struct i2c_msg*)malloc(alldata.nmsgs * sizeof(struct i2c_msg));
	if(!alldata.msgs){
	printf("Memory alloc error!\n");
	close(fd);
	return 0;
	}

	(alldata.msgs[0]).addr   = id;
	(alldata.msgs[0]).flags  = 0;
	(alldata.msgs[0]).len    = 2;
	(alldata.msgs[0]).buf    = (__u8 *)malloc(2);
	(alldata.msgs[0]).buf[0] = addr;
	(alldata.msgs[0]).buf[1] = val;  //要寫的值

	ret = ioctl(fd,I2C_RDWR,(unsigned long)&alldata);
	if(ret < 0)
		printf("W ER id = 0x%2x ,address = 0x%2x val=0x%2x ret=0x%4x\n",id,addr,val,ret);
	else
		printf("W OK id = 0x%2x ,address = 0x%2x val=0x%2x \n",id,addr,val);	
	
	
	free((alldata.msgs[0]).buf);
	free(alldata.msgs);
	sleep(1);  //沒有sleep,會無法讀取.	
	
	return 0;		
}




int main(int argc,char **argv)
{
	struct i2c_rdwr_ioctl_data alldata;
	int address;

	if(argc<2){
		printf("please input ./i2c 'value'.\n");
		return 0;
	}

	printf("Open device : /dev/i2c-0\n");
	fd = open("/dev/i2c-0", O_RDWR);
	if(!fd){
	printf("Error on opening the device file!\n");
	return 0;
	}

	ioctl(fd,I2C_TIMEOUT,1);
	ioctl(fd,I2C_RETRIES,1);

/*
	printf("--------------------poll id:--------------------\n\n");
	for (address = 0x0;address<0x80;address++){
		ret = i2c_read_byte(address,0);
			
	}	
	*/

/*
		adv739x_write(0x17,0x02);
		adv739x_write(0x00,0x1c);
		adv739x_write(0x00,0x10);
		adv739x_write(0x01,0x00);		
		adv739x_write(0x80,0x10);
		adv739x_write(0x82,0xcb);
*/
	ret = i2c_read_byte(0x2a,0x17);
	ret = i2c_read_byte(0x2a,0x00);
	ret = i2c_read_byte(0x2a,0x01);
	ret = i2c_read_byte(0x2a,0x80);	
	ret = i2c_read_byte(0x2a,0x82);	
	
	
	ret = i2c_write_byte(0x2a,0x17,0x02);
	ret = i2c_write_byte(0x2a,0x00,0x1c);
	ret = i2c_write_byte(0x2a,0x00,0x10);	
	ret = i2c_write_byte(0x2a,0x01,0x00);
	ret = i2c_write_byte(0x2a,0x80,0x10);	
	ret = i2c_write_byte(0x2a,0x82,0xcb);		
	
	ret = i2c_read_byte(0x2a,0x17);
	ret = i2c_read_byte(0x2a,0x00);
	ret = i2c_read_byte(0x2a,0x01);
	ret = i2c_read_byte(0x2a,0x80);	
	ret = i2c_read_byte(0x2a,0x82);		



	close(fd);
	return 0;
}
