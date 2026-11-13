#!/bin/sh
# The speakers' guard (bsd/dev/speaker_guard.c) on this machine: tools/speaker_guard_test.c built against it and run.
# The kernel's header is all it needs, and an empty sys/ioccom.h beside it (the ioctl numbers are not used).
set -e
cd "$(dirname "$0")/.."
SRC=bsd
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir -p "$tmp/sys"
cp $SRC/sys/mapple_audio.h "$tmp/sys/"
printf '#define _IOW(g, n, t) 0\n' > "$tmp/sys/ioccom.h"
cc -O2 -Wall -DKERNEL -I"$tmp" tools/speaker_guard_test.c $SRC/dev/speaker_guard.c -lm -o "$tmp/guard_test"
"$tmp/guard_test"
