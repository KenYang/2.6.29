#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>


#define	FD_OV7725_0	"/dev/i2c-0"
#define	FD_OV7725_1	"/dev/i2c-1"
#define	FD_SENSOR	"/home/ov7725_init"

#define	MAX_INIT	200

#define	MAX_FD		2
#define	MAX_REG		0xAC


#define	I2C_SLAVE	0x0703
#define I2C_RETRIES	0x0701
#define I2C_TIMEOUT	0x0702

#if 0
unsigned char  OV7725_initcode[][2] = {
	{0x09,	0x01	},
	{0x0c,	0x00	},
	{0x0d,	0x41	},
	{0x0f,	0xc5	},
	{0x11,	0x01	},
	{0x12,	0x20	}, //20
	{0x13,	0xff	},
	{0x14,	0x11	},
	{0x15,   0x41   } ,   //2010/03/22 add
	{0x17,	0x22	},
	{0x18,	0xa4	},
	{0x19,	0x07	},
	{0x1a,	0xf0	},
	{0x22,	0x7f	},
	{0x23,	0x03	},
	{0x24,	0x40	},
	{0x25,	0x30	},
	{0x26,	0xa1	},
	{0x29,	0xa0	},
	{0x2a,	0x00	},
	{0x2b,	0x62	},
	{0x2c,	0xf0	},
	{0x32,	0x00	},
	{0x3d,	0x03	},
	{0x42,	0x7f	},
	{0x46,	0x05	},
	{0x47,	0x00	},
	{0x48,	0x00	},
	{0x49,	0x10	},
	{0x4a,	0x00	},
	{0x4b,	0x10	},
	{0x4c,	0x15	},
	{0x4d,	0x09	},
	{0x63,	0xe0	},
	{0x64,	0xff	},
	{0x65,	0x20	},
	{0x66,	0x00	},
	{0x67,	0x48	},
	{0x6b,	0xaa	},
	{0x7e,	0x0c	},
	{0x7f,	0x16	},
	{0x80,	0x2a	},
	{0x81,	0x4e	},
	{0x82,	0x61	},
	{0x83,	0x6f	},
	{0x84,	0x7b	},
	{0x85,	0x86	},
	{0x86,	0x8e	},
	{0x87,	0x97	},
	{0x88,	0xa4	},
	{0x89,	0xaf	},
	{0x8a,	0xc5	},
	{0x8b,	0xd7	},
	{0x8c,	0xe8	},
	{0x8d,	0x20	},
	{0x90,	0x05	},
	{0x91,	0x01	},
	{0x92,	0x05	},
	{0x93,	0x00	},
	{0x94,	0x75	},
	{0x95,	0x70	},
	{0x96,	0x05	},
	{0x97,	0x22	},
	{0x98,	0x63	},
	{0x99,	0x85	},
	{0x9a,	0x1e	},
	{0x9b,	0x08	},
	{0x9c,	0x20	},
	{0x9e,	0x81	},
	{0xa6,	0x04	}
};
#endif


int main()
{
	int fd, idx=0, i=0;
	int ret;
	int err_count=0;

	int fd_array[MAX_FD];
	int fd_init;

	unsigned char OV7725_initcode[MAX_INIT][2];
	unsigned char buff[100];
	int	size_init=0;


	if(0>(fd_init=open(FD_SENSOR, O_RDWR))){
		printf("read ov7725 initial code\n");
		return 1;
	}


	while (2==(read(fd_init, &OV7725_initcode[size_init][0], 2))) {
//		printf("read data1=0x%02X  data2=0x%02X\n", OV7725_initcode[size_init][0],OV7725_initcode[size_init][1]);
		++size_init;
	}


	if(0>(fd_array[0]=open(FD_OV7725_0, O_RDWR))){
		printf("open i2c dev %s fail, error code=%d\n", FD_OV7725_0, errno);
		return 1;
	}
	
	if(0>(fd_array[1]=open(FD_OV7725_1, O_RDWR))){
		printf("open i2c dev %s fail, error code=%d\n", FD_OV7725_1, errno);
		return 1;
	}

	for(i=0; i<MAX_FD; ++i) {
		ret=ioctl(fd_array[i], I2C_TIMEOUT, 1);
		//printf("set timeout, return value = %d\n", ret);
		ret=ioctl(fd_array[i], I2C_RETRIES, 1);
		//printf("set retries, return value = %d\n", ret);

		//printf("set SLAVE=%d\n", 0x21);
		ret=ioctl(fd_array[i], I2C_SLAVE, 0x21);
		//printf("return value = %d\n", ret);
		//printf("total %d pair needed to trans\n", sizeof(OV7725_initcode)/2);
//		for(idx=0;idx<sizeof(OV7725_initcode)/2; ++idx) {
		for(idx=0;idx<size_init; ++idx) {
			ret=write(fd_array[i], &OV7725_initcode[idx][0], 2);
			if(2!=ret) {
				printf("write data fail, set reg=0x%02X, set data=0x%02X\n", OV7725_initcode[idx][0], OV7725_initcode[idx][1]);
				++err_count;
			}
//			else
//				printf("write data sucess, set reg=0x%02X, set data=0x%02X\n", OV7725_initcode[idx][0], OV7725_initcode[idx][1]);


		}
	}

	printf("sensor inital code transfer complete, %d  error\n", err_count);

}
