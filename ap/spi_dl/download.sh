
rm -f /dev/spi_download
insmod /lib/modules/sq_spi_download.ko
mknod /dev/spi_download c 252 0
sync
#./dl
#./pwm
#./scu
