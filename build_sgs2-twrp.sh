#!/bin/bash
#
######################################################
## DreamKernel Compile Script for Samsung Galaxy S2 ##
## GT-I9100 Devices.					   ##
## Script is based on gokhanmorals buildscript	   ## 
## This Script will build the Kernel,		   ##
## add the payload (Superuser.apk and su Binary)	   ##
## to the compiled zImage and also will add a 	   ##
## Custom "TOUCH enabled" Recovery to the Kernel	   ##
######################################################
#
## Colors for error/info messages (Red,Green,Yellow and bolded)
#
TXTRED='\e[0;31m' 		# Red
TXTGRN='\e[0;32m' 		# Green
TXTYLW='\e[0;33m' 		# Yellow
BLDRED='\e[1;31m' 		# Red-Bold
BLDGRN='\e[1;32m' 		# Green-Bold
BLDYLW='\e[1;33m' 		# Yellow-Bold
TXTCLR='\e[0m'    		# Text Reset
#
# Directory Settings
#
export KERNELDIR=`readlink -f .`
export INITRAMFS_SOURCE=`readlink -f $KERNELDIR/../initramfs-twrp`
export PARENT_DIR=`readlink -f ..`
export INITRAMFS_TMP="/tmp/initramfs-twrpsource"
export RELEASEDIR=`readlink -f $KERNELDIR/../releases`

#
# Version of this Build
#
KRNRLS="DreamKernel-v3.3.4TWRP"
KBUILD_BUILD_HOST=`hostname | sed 's|ip-projects.de|dream-irc.com|g'`
HOSTNAME=$KBUILD_BUILD_HOST
#
# Target Settings
#
export ARCH=arm
export CROSS_COMPILE=$PARENT_DIR/arm-galaxys2-androideabi/bin/galaxys2-
export USE_SEC_FIPS_MODE=true


if [ "${1}" != "" ];
then
  if [ -d  $1 ];
  then
    export KERNELDIR=`readlink -f ${1}`
    echo -e "${TXTGRN}Using alternative Kernel Directory: ${KERNELDIR}${TXTCLR}"
  else
    echo -e "${BLDRED}Error: ${1} is not a directory !${TXTCLR}"
    echo -e "${BLDRED}Nothing todo, Exiting ... !${TXTCLR}"
    exit 1
  fi
fi

# remove Files of old/previous Builds
#
echo -e "${TXTYLW}Deleting Files of previous Builds ...${TXTCLR}"
cd $KERNELDIR/
make -j10 distclean
rm -rvf $INITRAMFS_TMP
rm -vf $INITRAMFS_TMP.cpio
rm -fv $KERNELDIR/zImage
rm -vf $KERNELDIR/*-modules.log
rm -vf $KERNELDIR/*-zImage.log

# Start the Build
#
echo -e "${TXTYLW}CleanUP done, starting kernel Build ...${TXTCLR}"
echo
echo -e "${TXTYLW}Creating default kernel Config (dream_sgs2_defconfig):${TXTCLR}"
make dream_sgs2_defconfig
echo

echo -e "${TXTYLW}Changing kernelVersions String to match TWRP Build:${TXTCLR}"
sed -i 's|DreamKernel-v2.7.4|DreamKernel-v3.3.4TWRP|g' .config

. $KERNELDIR/.config
nice -n 10 make -j12 KBUILD_BUILD_HOST="$HOSTNAME" modules 2>&1 | tee $0-modules.log || exit 1
sleep 2

echo -e "${TXTGRN}Build: Stage 1 successfully completed${TXTCLR}"

# copy initramfs files to tmp directory
#
echo -e "${TXTGRN}Copying initramfs Filesystem to: ${INITRAMFS_TMP}${TXTCLR}"
cp -vax $INITRAMFS_SOURCE $INITRAMFS_TMP
sleep 1

# remove repository realated files
#
echo -e "${TXTGRN}Deleting Repository related Files (.git, .hg etc)${TXTCLR}"
find $INITRAMFS_TMP -name .git -exec rm -rvf {} \;
find $INITRAMFS_TMP -name EMPTY_DIRECTORY -exec rm -rvf {} \;
rm -rvf $INITRAMFS_TMP/.hg
sleep 1

## Remove Audio Library's for SGH-I777 Devices
rm -fv $INITRAMFS_TMP/res/misc/libaudio.tar.xz

# copy modules into initramfs
#
echo -e "${TXTGRN}Copying Modules to initramfs: ${INITRAMFS_TMP}/lib/modules${TXTCLR}"
mkdir -pv $INITRAMFS_TMP/lib/modules
find -name '*.ko' -exec cp -av {} $INITRAMFS_TMP/lib/modules/ \;
sleep 1

echo -e "${TXTGRN}Striping Modules to save space${TXTCLR}"
${CROSS_COMPILE}strip --strip-unneeded $INITRAMFS_TMP/lib/modules/*
sleep 1

# create the initramfs cpio archive
#
echo -e "${TXTYLW}Creating initial Ram Filesystem: ${INITRAMFS_TMP}.cpio ${TXTCLR}"
cd $INITRAMFS_TMP
find | fakeroot cpio -H newc -o > $INITRAMFS_TMP.cpio 2>/dev/null
ls -lh $INITRAMFS_TMP.cpio
cd -
sleep 1

# Start Final Kernel Build
#
echo -e "${TXTYLW}Starting final Build: Stage 2${TXTCLR}"
nice -n 10 make -j12 KBUILD_BUILD_HOST="$HOSTNAME" CONFIG_INITRAMFS_SOURCE="$INITRAMFS_TMP.cpio" zImage 2>&1 | tee $0-zImage.log || exit 1
sleep 1
$KERNELDIR/mkshbootimg.py $KERNELDIR/zImage $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/payload.tar $KERNELDIR/recovery.tar.xz

echo -e "${TXTGRN}Final Build: Stage 2 completed successfully!${TXTCLR}"

# Create ODIN Flashable TAR archiv
#
#
# cp $KERNELDIR/arch/arm/boot/zImage .
ARCNAME="$KRNRLS-`date +%Y%m%d%H%M%S`"

echo -e "${BLDRED}creating ODIN-Flashable TAR: and CWM flashable ZIP: ${ARCNAME}.tar/.zip ${TXTCLR}"
## ODIN
tar cfv $ARCNAME.tar zImage
mv -v $ARCNAME.tar $RELEASEDIR

## CWM
cp -v $RELEASEDIR/updater-template.zip $RELEASEDIR/$ARCNAME-CWM.zip
zip -u $RELEASEDIR/$ARCNAME-CWM.zip zImage


## List the Files
echo -e "${TXTYLW}$(ls -lh $KERNELDIR/zImage)${TXTCLR}"
echo -e "${TXTGRN}$(ls -lh $RELEASEDIR/$ARCNAME.tar)${TXTCLR}"
echo -e "${TXTGRN}$(ls -lh $RELEASEDIR/$ARCNAME-CWM.zip)${TXTCLR}"

echo

echo -e "${BLDGRN}	#############################	${TXTCLR}"
echo -e "${TXTRED}	# Script completed, exiting #	${TXTCLR}"
echo -e "${BLDGRN}	#############################	${TXTCLR}"
