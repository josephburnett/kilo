#!/bin/sh
# Push a kernel image to the netboot loader running on the Pi; it boots it from RAM.
#
#   kernel/tools/netboot.sh [--http] <ip> [img]
#
# img defaults to kernel/build/kernel7.img. TFTP (port 69) through tftp_put.py
# by default, or HTTP (port 8080) with curl. The loader accepts only names
# kernel*.img, so the image is always sent as kernel7.img. It holds at most
# 2 MB (Circle's KERNEL_MAX_SIZE).
set -eu

here=$(cd "$(dirname "$0")" && pwd)
http=0
if [ "${1:-}" = "--http" ]; then http=1; shift; fi
[ $# -ge 1 ] && [ $# -le 2 ] || { echo "usage: $0 [--http] <ip> [img]" >&2; exit 2; }
ip=$1
img=${2:-$here/../build/kernel7.img}
[ -f "$img" ] || { echo "no image: $img (run: make -C kernel)" >&2; exit 1; }

size=$(wc -c < "$img")
if [ "$size" -gt 2097152 ]; then
	echo "$img is $size bytes; the loader takes at most 2097152" >&2
	exit 1
fi

if [ $http = 1 ]; then
	out=$(curl -sS --max-time 30 -H "Expect:" -F "kernelimg=@$img;filename=kernel7.img" "http://$ip:${NETBOOT_HTTP_PORT:-8080}/index.html")
	case $out in
	*"Now booting"*) echo "sent $size bytes to http://$ip:${NETBOOT_HTTP_PORT:-8080}/, the loader is booting it" ;;
	*) echo "the loader did not accept the image:" >&2; echo "$out" | sed 's/<[^>]*>//g' | grep -v '^\s*$' >&2; exit 1 ;;
	esac
else
	# NETBOOT_TFTP_PORT and NETBOOT_HTTP_PORT are for tests against local stubs only.
	python3 "$here/tftp_put.py" --name kernel7.img --port "${NETBOOT_TFTP_PORT:-69}" "$ip" "$img"
fi
