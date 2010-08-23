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

int main(int argc,char **argv)
{
	struct i2c_rdwr_ioctl_data alldata;
	int fd,ret;
	int address;
	int dev_tbl[20],j=0,i,tmp1,tmp2;

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

	for (address = 0x0;address<0x80;address++){
	//printf("Set address = %x\n",address);

	//printf("--------------------read original value:--------------------\n\n");

	alldata.nmsgs = 2;
	alldata.msgs = (struct i2c_msg*)malloc(alldata.nmsgs * sizeof(struct i2c_msg));
		if(!alldata.msgs){
			printf("Memory alloc error!\n");
			close(fd);
			return 0;
		}

	(alldata.msgs[0]).addr   = address;                   //slave address
	(alldata.msgs[0]).flags  = 0;                      //flag = 0(write) , 1(read)
	(alldata.msgs[0]).len    = 1;
	(alldata.msgs[0]).buf    = (__u8 *)malloc(1);
	(alldata.msgs[0]).buf[0] = 0x00;                   //sub address (offset)
	(alldata.msgs[1]).addr   = address;
	(alldata.msgs[1]).flags  = I2C_M_RD;
	(alldata.msgs[1]).len    = 1;
	(alldata.msgs[1]).buf    = (__u8 *)malloc(1);
	(alldata.msgs[1]).buf[0] = 0;

	ret = ioctl(fd,I2C_RDWR,(unsigned long)&alldata);
	if(ret < 0) {
		printf("Error durning I2C_RDWR ioctl with error code:%d\n",ret);		
	} else {
		printf("address = %x -original value=%x\n",address,(alldata.msgs[1]).buf[0]);
	}	dev_tbl[j++] = address;
	

	free((alldata.msgs[0]).buf);
	free((alldata.msgs[1]).buf);
	free(alldata.msgs);

	}
	
	while(1);
	





//	if(j) 
	{
//	for (i=0 ; i < j;i++)
	{
	//address = dev_tbl[i];
	address = 0x50;
	printf("Set address = %x\n",address);

	//printf("--------------------read original value:--------------------\n\n");

	alldata.nmsgs = 2;
	alldata.msgs = (struct i2c_msg*)malloc(alldata.nmsgs * sizeof(struct i2c_msg));
		if(!alldata.msgs){
			printf("Memory alloc error!\n");
			close(fd);
			return 0;
		}

	(alldata.msgs[0]).addr   = address;                   //slave address
	(alldata.msgs[0]).flags  = 0;                      //flag = 0(write) , 1(read)
	(alldata.msgs[0]).len    = 1;
	(alldata.msgs[0]).buf    = (__u8 *)&tmp1;
	(alldata.msgs[0]).buf[0] = 0x00;                   //sub address (offset)
	(alldata.msgs[1]).addr   = address;
	(alldata.msgs[1]).flags  = I2C_M_RD;
	(alldata.msgs[1]).len    = 1;
	(alldata.msgs[1]).buf    = (__u8 *)&tmp2;
	(alldata.msgs[1]).buf[0] = 0;

	ret = ioctl(fd,I2C_RDWR,(unsigned long)&alldata);
	if(ret < 0)
		printf("Error durning I2C_RDWR ioctl with error code:%d\n",ret);
	else
		printf("original value=%x\n",(alldata.msgs[1]).buf[0]);

	printf("--------------------read end.--------------------\n\n");

	free((alldata.msgs[0]).buf);
	free((alldata.msgs[1]).buf);
	free(alldata.msgs);

	printf("--------------------write to change the value:--------------------\n\n");

	alldata.nmsgs = 1;
	alldata.msgs = (struct i2c_msg*)malloc(alldata.nmsgs * sizeof(struct i2c_msg));
	if(!alldata.msgs){
	printf("Memory alloc error!\n");
	close(fd);
	return 0;
	}

	(alldata.msgs[0]).addr   = address;
	(alldata.msgs[0]).flags  = 0;
	(alldata.msgs[0]).len    = 2;
	(alldata.msgs[0]).buf    = (__u8 *)malloc(2);
	(alldata.msgs[0]).buf[0] = 0x00;
//	(alldata.msgs[0]).buf[1] = 0x5f;  //要寫的值
	int value;
	sscanf(argv[1],"%x",&value);
	(alldata.msgs[0]).buf[1] = value;

	ret = ioctl(fd,I2C_RDWR,(unsigned long)&alldata);
	if(ret < 0)
		printf("Error durning I2C_RDWR ioctl with error code:%d\n",ret);
	else
		printf("written %x to the register.\n",(alldata.msgs[0]).buf[1]);

	printf("--------------------write end.--------------------\n\n");

	free((alldata.msgs[0]).buf);
	free(alldata.msgs);
	sleep(1);  //沒有sleep,會無法讀取.

	printf("--------------------read again:--------------------\n\n");

	alldata.nmsgs = 2;
	alldata.msgs = (struct i2c_msg*)malloc(alldata.nmsgs * sizeof(struct i2c_msg));
	if(!alldata.msgs){
	printf("Memory alloc error!\n");
	close(fd);
	return 0;
	}

	(alldata.msgs[0]).addr   = address;                   //slave address
	(alldata.msgs[0]).flags  = 0;                      //flag = 0(write) , 1(read)
	(alldata.msgs[0]).len    = 1;
	(alldata.msgs[0]).buf    = (__u8 *)&tmp1;
	(alldata.msgs[0]).buf[0] = 0x00;                   //sub address (offset)
	(alldata.msgs[1]).addr   = address;
	(alldata.msgs[1]).flags  = I2C_M_RD;
	(alldata.msgs[1]).len    = 1;
	(alldata.msgs[1]).buf    = (__u8 *)&tmp2;
	(alldata.msgs[1]).buf[0] = 0;

	ret = ioctl(fd,I2C_RDWR,(unsigned long)&alldata);
	if(ret < 0)
		printf("Error durning I2C_RDWR ioctl with error code:%d\n",ret);
	else
		printf("changed value=%x\n",(alldata.msgs[1]).buf[0]);

	printf("--------------------read end.--------------------\n\n");

	}}

	free((alldata.msgs[0]).buf);
	free((alldata.msgs[1]).buf);
	free(alldata.msgs);

	close(fd);
	return 0;
}
