#!/bin/bash

ROOTFS_PATH="/home/simone/neak-nexus7/ramdisk"

echo "Building N.E.A.K. Nexus 7..."

# Cleanup
./clean.sh

# Making .config
make neak_defconfig

# Compiling
./build.sh $ROOTFS_PATH
