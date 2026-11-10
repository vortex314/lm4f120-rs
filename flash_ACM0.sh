#!/usr/bin/env bash
# Build and flash the LM4F120 firmware.
set -euo pipefail

cd "$(dirname "$0")"

echo "==> cargo build --release"
cargo build --release

SYSROOT="$(rustc --print sysroot)"
HOST="$(rustc -vV | awk '/^host:/ { print $2 }')"
SYSDIR="$SYSROOT/lib/rustlib/$HOST/bin"

echo "==> objcopy ELF -> firmware.bin"
LD_LIBRARY_PATH="$SYSROOT/lib" "$SYSDIR/rust-objcopy" \
    -O binary target/thumbv7em-none-eabihf/release/lm4f120-hello firmware.bin

echo "==> lm4flash firmware.bin"
lm4flash firmware.bin -s 0E10333C

echo "==> done"
