#!/bin/bash

# Set Default Path
TOP_DIR=$PWD
KERNEL_PATH="/home/simone/neak-nexus7"

# Set toolchain and root filesystem path
TOOLCHAIN="/home/simone/android-toolchain-eabi-4.7/bin/arm-eabi-"
ROOTFS_PATH=$1

# Exports
export KERNEL_VERSION="N.E.A.K-N7-1.0x"
export KERNELDIR=$KERNEL_PATH

# Compile
make -j`grep 'processor' /proc/cpuinfo | wc -l` ARCH=arm CROSS_COMPILE=$TOOLCHAIN >> compile.log 2>&1 || exit -1

# Recompile to make modules working
make -j`grep 'processor' /proc/cpuinfo | wc -l` ARCH=arm CROSS_COMPILE=$TOOLCHAIN || exit -1

# Copy Kernel Image
rm -f $KERNEL_PATH/releasetools/tar/$KERNEL_VERSION.tar
rm -f $KERNEL_PATH/releasetools/zip/$KERNEL_VERSION.zip
cp -f $KERNEL_PATH/arch/arm/boot/zImage .

# Create ramdisk.cpio archive
cd $ROOTFS_PATH
find . | cpio -o -H newc > ../ramdisk.cpio
cd ..

# Make boot.img
./mkbootimg --base 0 --pagesize 2048 --kernel_offset 0x10008000 --ramdisk_offset 0x11000000 --second_offset 0x10f00000 --tags_offset 0x10000100 --kernel zImage --ramdisk ramdisk.cpio -o boot.img

# Copy boot.img
cp boot.img $KERNEL_PATH/releasetools/zip
cp boot.img $KERNEL_PATH/releasetools

# Creating flashable zip and renaming boot.img
cd $KERNEL_PATH/releasetools/zip
zip -0 -r $KERNEL_VERSION.zip *
cd ..
mv boot.img $KERNEL_VERSION.img

# Cleanup
rm $KERNEL_PATH/releasetools/zip/boot.img
rm $KERNEL_PATH/zImage
