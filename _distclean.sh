#!/bin/bash

export LANG=C
export LC_ALL=C
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-
export PATH=/opt/arm-eabi-4.7/bin:$PATH
export KBUILD_BUILD_USER="andrew_z1"
export KBUILD_BUILD_HOST="home"

make -j2 V=1 distclean
rm -f scripts/dtc/dtc-lexer.lex.c scripts/dtc/dtc-parser.tab.c scripts/dtc/dtc-parser.tab.h
