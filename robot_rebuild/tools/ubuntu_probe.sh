#!/bin/sh
# Read-only board inventory. Does not change DT, networking, packages or flash.
set -eu
uname -a
cat /etc/os-release
printf '\nUSB serial identities\n'
ls -l /dev/serial/by-id/ 2>/dev/null || true
ls -l /dev/serial/by-path/ 2>/dev/null || true
printf '\nVideo identities\n'
ls -l /dev/v4l/by-id/ 2>/dev/null || true
command -v v4l2-ctl >/dev/null && v4l2-ctl --list-devices || true
printf '\nNetwork controller mapping (FPGA GMAC1 must be fe1c0000)\n'
for n in /sys/class/net/*; do
    printf '%s -> ' "${n##*/}"
    readlink -f "$n/device" || true
done
ip -br address
printf '\nPython\n'
python3 --version
