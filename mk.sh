#!/usr/bin

PATH=/usr/bin:$PATH


make uImage
cp -f ./arch/arm/boot/uImage /tftpboot/uImage_k29_src
cp -f .config sq8006.config
cp -f ./arch/arm/boot/uImage ./uImage_k29_src
