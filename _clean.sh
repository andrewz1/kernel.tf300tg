#!/bin/bash

export LANG=C
export LC_ALL=C
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-
export PATH=/opt/arm-eabi-4.7/bin:$PATH
export KBUILD_BUILD_USER="andrew_z1"
export KBUILD_BUILD_HOST="home"

make -j2 V=1 clean
rm -f .version* Module.symvers
rm -f scripts/basic/docproc scripts/basic/fixdep scripts/bin2c scripts/kallsyms scripts/kconfig/conf scripts/mod/mk_elfconfig scripts/mod/modpost
rm -f scripts/kconfig/*.zconf.c scripts/kconfig/*.lex.c scripts/kconfig/zconf.hash.c scripts/kconfig/zconf.tab.c scripts/mod/elfconfig.h
rm -f include/linux/version.h
rm -rf include/config
rm -rf include/generated
