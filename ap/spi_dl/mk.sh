
export PATH=/usr/bin:$PATH

#rm ../../usr/bin/dl
#rm dl


#/opt/crosstool/arm-2010q1/bin/arm-none-linux-gnueabi-gcc spi_download.c -o dl -static 
/opt/crosstool/arm-2010q1/bin/arm-none-linux-gnueabi-gcc spi_download.c -o dl
ls -al dl

/opt/crosstool/arm-2010q1/bin/arm-none-linux-gnueabi-strip -s dl
ls -al dl


#cp dl ../../usr/bin
cp dl /nfs/pc9223/rootfs_pc9223_v1/home
cp download.sh /nfs/pc9223/rootfs_pc9223_v1/home
