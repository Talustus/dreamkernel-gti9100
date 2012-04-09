#!/bin/sh
export KERNELDIR=`readlink -f .`
export INITRAMFS_SOURCE=`readlink -f $KERNELDIR/../initramfs-ics`
export PARENT_DIR=`readlink -f ..`

if [ "${1}" != "" ];then
  export KERNELDIR=`readlink -f ${1}`
fi

INITRAMFS_TMP="/tmp/initramfs-source"

if [ ! -f $KERNELDIR/.config ];
then
  make dream_defconfig
fi

. $KERNELDIR/.config

export ARCH=arm
export CROSS_COMPILE=$PARENT_DIR/toolchain-galaxys2/bin/galaxy-

cd $KERNELDIR/
rm -v compile.log
nice -n 10 make -j4 | tee compile.log || exit 1

# remove previous initramfs files
rm -rvf $INITRAMFS_TMP
rm -rvf $INITRAMFS_TMP.cpio

# copy initramfs files to tmp directory
cp -vax $INITRAMFS_SOURCE $INITRAMFS_TMP

# remove repository realated files
find $INITRAMFS_TMP -name .git -exec rm -rvf {} \;
find $INITRAMFS_TMP -name EMPTY_DIRECTORY -exec rm -rvf {} \;
rm -rvf $INITRAMFS_TMP/.hg

# copy modules into initramfs
mkdir -pv $INITRAMFS_TMP/lib/modules
find -name '*.ko' -exec cp -av {} $INITRAMFS_TMP/lib/modules/ \;

# create the initramfs cpio archive
#cd $INITRAMFS_TMP
#find | fakeroot cpio -H newc -o > $INITRAMFS_TMP.cpio 2>/dev/null
#ls -lh $INITRAMFS_TMP.cpio
#cd -

nice -n 10 make -j3 zImage CONFIG_INITRAMFS_SOURCE="$INITRAMFS_TMP.cpio" || exit 1

cp $KERNELDIR/arch/arm/boot/zImage zImage
KRNRLS="DreamKernel-1.0RC3"
ARCNAME="$KRNRLS-`date +%Y%m%d%H%M%S`.tar"
echo "creating ODIN-Flashable TAR: ${ARCNAME}"
tar cfv $ARCNAME zImage
ls -la $ARCNAME
#
#$KERNELDIR/mkshbootimg.py $KERNELDIR/zImage $KERNELDIR/arch/arm/boot/zImage $KERNELDIR/../payload.cpio $KERNELDIR/../recovery.cpio.xz

