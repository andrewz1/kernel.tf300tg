#!/bin/bash

export LANG=C
export LC_ALL=C
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-
export PATH=/opt/arm-eabi-4.7/bin:$PATH
export KBUILD_BUILD_USER="andrew_z1"
export KBUILD_BUILD_HOST="home"

echo 12 >.version
make -j2 V=1 TARGET_USE_NEW_TOOLCHAIN=1 zImage
[ $? -eq 0 ] || exit
make -j2 V=1 TARGET_USE_NEW_TOOLCHAIN=1 modules
[ $? -eq 0 ] || exit
rm -f /opt/TF300TG/bin/WW_epad-user-10.6.2.8/_blob/LNX/zImage
cp arch/arm/boot/zImage /opt/TF300TG/bin/WW_epad-user-10.6.2.8/_blob/LNX/zImage
