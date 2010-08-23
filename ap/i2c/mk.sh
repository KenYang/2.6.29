
export PATH=/usr/bin:$PATH


SRC_FILE=i2c_rdwr_org.c
TAR_NAME=i2c


rm $TAR_NAME
#/opt/crosstool/arm-2010q1/bin/arm-none-linux-gnueabi-gcc spi_$SRC_FILE -o $TAR_NAME -static 
/opt/crosstool/arm-2010q1/bin/arm-none-linux-gnueabi-gcc $SRC_FILE -o $TAR_NAME
ls -al i2c

/opt/crosstool/arm-2010q1/bin/arm-none-linux-gnueabi-strip -s $TAR_NAME
ls -al i2c



cp $TAR_NAME /nfs/pc9223/rootfs_pc9223_v1/home

