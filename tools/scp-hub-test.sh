#!/bin/sh
# The SCP sensor hub's framing, ring, time and unit conversions (bsd/dev/mtk_scp_hub.h), on this machine.
set -e
cd "$(dirname "$0")/.."
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cc -O2 -Wall -Wextra tools/scp_hub_test.c -o "$tmp/scp_hub_test"
"$tmp/scp_hub_test"
