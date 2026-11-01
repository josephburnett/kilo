#!/bin/sh
# Copy the Pi boot files, config.txt and a kernel image onto a mounted FAT partition.
#
#   kernel/tools/make-sd.sh <mountpoint> [--netboot|--kernel]
#
#   --netboot  (default) the network loader, build/netboot/kernel7.img: the bench card
#   --kernel   the mind kernel, build/kernel7.img: the production card
#
# Never partitions or formats anything. Refuses unless <mountpoint> is the root
# of a mounted vfat filesystem that is empty or holds only Pi boot files. A
# Raspberry Pi OS card (cmdline.txt, overlays/, ...) is refused: this would
# replace its Linux kernel7.img and config.txt.
#
#   --force-dir  TEST ONLY: skip the mount and filesystem checks (a plain directory).
set -eu

here=$(cd "$(dirname "$0")" && pwd)
kernel=$here/..
fw=$kernel/circle/boot

usage() { echo "usage: $0 <mountpoint> [--netboot|--kernel]" >&2; exit 2; }

dir=
img=$kernel/build/netboot/kernel7.img
force=0
for a in "$@"; do
	case $a in
	--netboot) img=$kernel/build/netboot/kernel7.img ;;
	--kernel) img=$kernel/build/kernel7.img ;;
	--force-dir) force=1 ;;
	-*) usage ;;
	*) [ -z "$dir" ] || usage; dir=$a ;;
	esac
done
[ -n "$dir" ] || usage
[ -d "$dir" ] || { echo "$dir: not a directory" >&2; exit 1; }
dir=$(realpath "$dir")

if [ $force = 0 ]; then
	target=$(findmnt -n -o TARGET --target "$dir")
	fstype=$(findmnt -n -o FSTYPE --target "$dir")
	if [ "$target" != "$dir" ]; then
		echo "$dir is not a mount point (it is inside $target)" >&2; exit 1
	fi
	if [ "$fstype" != vfat ] && [ "$fstype" != msdos ]; then
		echo "$dir is $fstype, not FAT" >&2; exit 1
	fi
fi

# Everything on the partition must be something a Pi boot partition made by
# this script (or by Circle's boot/ directory) would hold. A directory we
# cannot list is refused, never taken for empty.
if ! [ -r "$dir" ] || ! [ -x "$dir" ] || ! [ -w "$dir" ]; then
	echo "$dir: cannot list or write it" >&2; exit 1
fi
listing=$(ls -A "$dir") || { echo "$dir: cannot list it" >&2; exit 1; }
bad=
while IFS= read -r name; do
	[ -n "$name" ] || continue
	case $name in
	bootcode.bin|start*.elf|fixup*.dat|config.txt|kernel*.img|armstub*.bin|*.dtb) ;;
	LICENCE.broadcom|COPYING.linux) ;;
	"System Volume Information"|.Trashes|.fseventsd|.Spotlight-V100) ;;
	*) bad="$bad '$name'" ;;
	esac
done <<EOF
$listing
EOF
if [ -n "$bad" ]; then
	echo "$dir holds files a Kilo boot card does not:$bad" >&2
	echo "Refusing. Clear the partition yourself if it really is the card to use." >&2
	exit 1
fi

for f in bootcode.bin start_cd.elf fixup_cd.dat LICENCE.broadcom; do
	[ -f "$fw/$f" ] || { echo "missing $fw/$f (run: make -C kernel boot)" >&2; exit 1; }
done
[ -f "$img" ] || { echo "missing $img (run: make -C kernel)" >&2; exit 1; }

for f in bootcode.bin start_cd.elf fixup_cd.dat LICENCE.broadcom; do
	cp "$fw/$f" "$dir/$f"
done
cp "$kernel/boot/config.txt" "$dir/config.txt"
cp "$img" "$dir/kernel7.img"
sync
echo "wrote to $dir: bootcode.bin start_cd.elf fixup_cd.dat LICENCE.broadcom config.txt kernel7.img (from ${img#$kernel/})"
