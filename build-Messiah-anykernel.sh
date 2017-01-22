#!/bin/bash

#
#  Build Script for Render Kernel for OPO!
#  Based off AK'sbuild script - Thanks!
#

# Bash Color
green='\033[01;32m'
red='\033[01;31m'
blink_red='\033[05;31m'
restore='\033[0m'

clear

# Resources
THREAD="-j$(grep -c ^processor /proc/cpuinfo)"
KERNEL="zImage"
DTBIMAGE="dtb"
DEFCONFIG="lineageos_bacon_defconfig"

# Kernel Details
VER=Messiah

# Vars
export LOCALVERSION=~`echo $VER`
export ARCH=arm
export SUBARCH=arm
export KBUILD_BUILD_USER=DM47021

# Paths
KERNEL_DIR=`pwd`
REPACK_DIR="/home/dm47021/Android/kernel/oneplus_one/kernel_oneplus_msm8974/OPO-AnyKernel"
PATCH_DIR="/home/dm47021/Android/kernel/oneplus_one/kernel_oneplus_msm8974/OPO-AnyKernel/patch"
MODULES_DIR="/home/dm47021/Android/kernel/oneplus_one/kernel_oneplus_msm8974/OPO-AnyKernel/modules"
ZIP_MOVE="$/home/dm47021/Android/kernel/oneplus_one/kernel_oneplus_msm8974/zips/opo-cm-zips"
ZIMAGE_DIR="/home/dm47021/Android/kernel/oneplus_one/kernel_oneplus_msm8974/arch/arm/boot"
VARIANT="OPO"

# Functions

function clean_all {
		cd $REPACK_DIR
		rm -rf $MODULES_DIR/*
		rm -rf $KERNEL
		rm -rf $DTBIMAGE
		cd $KERNEL_DIR
		echo
		make clean && make mrproper
}

function make_kernel {
		echo
		make $DEFCONFIG
		make $THREAD
		cp -vr $ZIMAGE_DIR/$KERNEL $REPACK_DIR
}

function make_modules {
		rm `echo $MODULES_DIR"/*"`
		find $KERNEL_DIR -name '*.ko' -exec cp -v {} $MODULES_DIR \;
}

function make_dtb {
		$REPACK_DIR/tools/dtbToolCM -2 -o $REPACK_DIR/$DTBIMAGE -s 2048 -p scripts/dtc/ arch/arm/boot/
}

function make_zip {
		cd $REPACK_DIR
		zip -r9 MessiahKernel-CM14_"$VARIANT"-R.zip *
		mv MessiahKernel-CM14_"$VARIANT"-R.zip $ZIP_MOVE
		cd $KERNEL_DIR
}


DATE_START=$(date +"%s")

echo -e "${green}"
echo "Render Kernel Creation Script:"
echo -e "${restore}"

echo "Pick Toolchain..."
select choice in linaro-4.9-cortex-a15
do
case "$choice" in
	"linaro-4.9-cortex-a15")
		export CROSS_COMPILE=/home/dm47021/Android/toolchains/linaro-4.9-cortex-a15/bin/arm-cortex_a15-linux-gnueabihf-
		break;;
	"UBER-5.2")
		export CROSS_COMPILE=${HOME}/android/source/toolchains/UBER-arm-eabi-5.2-08062015/bin/arm-eabi-
		break;;
esac
done

while read -p "Do you want to clean stuffs (y/n)? " cchoice
do
case "$cchoice" in
	y|Y )
		clean_all
		echo
		echo "All Cleaned now."
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo

while read -p "Do you want to build kernel (y/n)? " dchoice
do
case "$dchoice" in
	y|Y)
		make_kernel
		make_dtb
		make_modules
		make_zip
		break
		;;
	n|N )
		break
		;;
	* )
		echo
		echo "Invalid try again!"
		echo
		;;
esac
done

echo -e "${green}"
echo "-------------------"
echo "Build Completed in:"
echo "-------------------"
echo -e "${restore}"

DATE_END=$(date +"%s")
DIFF=$(($DATE_END - $DATE_START))
echo "Time: $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds."
echo
