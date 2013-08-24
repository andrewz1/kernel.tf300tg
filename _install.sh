#!/bin/bash

export LANG=C
export LC_ALL=C
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-
export PATH=/opt/arm-eabi-4.7/bin:$PATH
export KBUILD_BUILD_USER="andrew_z1"
export KBUILD_BUILD_HOST="home"

inst_P=/opt/TF300TG/v1

mkdir -p $inst_P
rm -rf $inst_P/*

cp -a arch/arm/boot/zImage $inst_P/
find . -name '*.ko' -exec cp -a {} $inst_P/ \;
find $inst_P/ -name '*.ko' -exec arm-eabi-strip --strip-unneeded {} \;

chown 0.0 $inst_P/*
chmod 644 $inst_P/*
