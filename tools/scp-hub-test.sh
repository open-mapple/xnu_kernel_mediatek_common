#!/bin/sh
# The SCP sensor hub (bsd/dev/mtk_scp_hub.h) and the SPU messages and attitude (MappleSPU.h), on this machine.
set -e
cd "$(dirname "$0")/.."
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cc -O2 -Wall -Wextra tools/scp_hub_test.c -o "$tmp/scp_hub_test"
"$tmp/scp_hub_test"
# what locationd reads from the SPU, built from the hub's data (iokit/Kernel/arm/hid/MappleSPU.h)
cc -O2 -Wall -Wextra tools/spu_msg_test.c -o "$tmp/spu_msg_test" -lm
"$tmp/spu_msg_test"
