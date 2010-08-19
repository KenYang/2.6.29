#!/usr/bin

PATH=/usr/bin:$PATH

#MODULES_DIR=/nfs/arm9_fs_k27src_3.4.5/lib/modules
#MODULES_DIR=/nfs/rootfs_git/RootFileSystem/arm9_fs_k27src_3.4.5/lib/modules
MODULES_DIR=/nfs/pc9223/rootfs_pc9223_v1/lib/modules
make modules

#cp -f ./drivers/char/sq_dia/sq-dia.ko /nfs/rootfs_git/RootFileSystem/arm9_fs_k27src_3.4.5/lib/modules

cp -f ./drivers/char/sq_dia/sq-dia.ko $MODULES_DIR
cp -f ./drivers/spi/sq_spi_download.ko $MODULES_DIR

cp -f ./drivers/i2c/chips/ov7725.ko $MODULES_DIR
cp -f ./drivers/i2c/busses/sq-i2c-gpio.ko $MODULES_DIR
