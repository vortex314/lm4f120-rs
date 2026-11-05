# lm4f120-rs

Embedded Rust for the **TI Stellaris LM4F120H5QR** (EK-LM4F120XL LaunchPad) —
a Cortex-M4F "Tiva-C class" microcontroller.

The repository contains a peripheral access layer (PAL) / HAL crate plus an
async demo application that exercises the board end-to-end: PLL clocking,
digital GPIO, UART (polled and DMA), SPI, I²C, and an Embassy async runtime
with multi-tasking and channels.

## Repository layout

```
lm4f120-rs/
├── lm4f120-hal/        # no_std HAL crate (embedded-hal 1.0 drivers)
│   ├── src/
│   │   ├── lib.rs      # crate root + register helpers + re-exports
│   │   ├── gpio.rs     # digital GPIO (typestate pins)
│   │   ├── uart.rs     # UART0-7 (embedded-io + µDMA)
│   │   ├── spi.rs      # SSI0-1 in SPI mode
│   │   ├── i2c.rs      # I2C0-3 master
│   │   ├── dma.rs      # µDMA controller
│   │   └── sysctl.rs   # clock gating + 80 MHz PLL setup
│   ├── examples/       # gpio_blink, uart_echo, spi_loopback, i2c_scan
│   └── README.md
├── src/main.rs         # async demo application
├── memory.x            # linker memory layout (256 KB flash / 32 KB SRAM)
├── Cargo.toml          # binary crate + workspace deps
├── flash.sh            # build + objcopy + lm4flash
└── firmware.bin        # generated flash image
```

## Features

### `lm4f120-hal`

- **Clock** — `setup_80mhz()` drives the PLL (16 MHz crystal → 400 MHz VCO → ÷5)
  for an 80 MHz system clock; drivers take the system clock frequency
  explicitly.
- **GPIO** — typestate pins `Pin<PORT, MODE, N>` for ports A–H implementing
  `embedded_hal::digital::{OutputPin, InputPin, StatefulOutputPin}` with
  pull-up/down and drive-strength control.
- **UART** — UART0–7 implementing `embedded_io::{Read, Write}` (+ `ReadReady`),
  with per-UART pin muxing and baud-rate configuration.
- **SPI** — SSI0–1 implementing `embedded_hal::spi::SpiBus<u8>`
  (read/write/transfer/transfer_in_place/flush).
- **I²C** — I2C0–3 master implementing `embedded_hal::i2c::I2c` (via
  `transaction`), with SCL/SDA open-drain pin muxing.
- **µDMA** — 32-channel micro-DMA controller with a software-managed control
  table; `Uart` exposes `dma_read`/`dma_write`/`dma_rx_done`/`dma_tx_done`.

### `lm4f120-hello` demo application

- Embassy async executor (`platform-cortex-m` + `executor-thread`).
- A hand-rolled SysTick time driver for `embassy-time` (multi-slot alarm table,
  1 kHz tick at 80 MHz).
- Concurrent tasks:
  - RGB LED colour cycling (`led_task`).
  - Channel ping-pong (`requester` ↔ `responder` over `embassy_sync::channel`)
    reporting round-trips/second.
  - Per-UART DMA loopback tasks (UART1–7) reporting buffer round-trip time.
- Cycle-accurate timing derived from the SysTick counter (12.5 ns resolution).

Example steady-state output at 80 MHz on UART0 (`/dev/ttyACM0`, 115200 8N1):

```
channel ping-pong ready
speed: 89108 round-trips/s
uart1 latency: 5555 us      # 64-byte DMA loopback, baud-limited
```

## Getting started

### Prerequisites

- Rust stable + the `thumbv7em-none-eabihf` target:
  `rustup target add thumbv7em-none-eabihf`
- `lm4flash` (Debian/Ubuntu: `sudo apt install lm4flash`)
- Linux serial access (the user should be in the `dialout` group)

### Build

```sh
cargo build --release           # uses the thumbv7em-none-eabihf target by default
```

### Flash

```sh
./flash.sh                      # build + objcopy ELF→bin + lm4flash
```

Flashing requires access to the ICDI debug interface (root, or a udev rule for
`1cbe:00fd`). The demo runs immediately after flashing.

### Serial monitor

```sh
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

The UART1–7 loopback tasks only report when their TX/RX pins are jumpered
together (loopback cable).

## Issues encountered

These are documented so they're not re-hit by future readers:

- **ICDI firmware v1.00** — the first board (ICDI serial `0E10D30A`, `bcdDevice
  1.00`) never answered either `lm4flash` (GDB Remote Serial Protocol) or
  OpenOCD's `ti-icdi`; both hung during the debug handshake. A second board
  with newer ICDI firmware (`9270`) flashed correctly.
- **embedded-hal 1.0 breaking changes** — 1.0 removed the `serial` module
  (byte streams moved to `embedded-io`), made `i2c::I2c::transaction` the
  required method (read/write/write_read became derived), made `SpiBus::flush`
  required, and added `Error::kind()`.
- **Embassy timer driver** — a single (waker, deadline) slot dropped the second
  task's timer when two tasks awaited `Timer::after` with the same deadline;
  fixed with a small multi-slot alarm table.
- **µDMA (five bugs)** — (1) the µDMA clock gate `RCGCDMA` was never enabled;
  (2) UART2–7 use *secondary* channel mappings and need `CHASGN` programming
  (only UART0/1 are defaults); (3) the control table was emitted into flash
  (`.rodata`) because it was an immutable `static` — it must be writable RAM;
  (4) the control-table entry stride was `N×32` interleaved instead of `N×16`
  primary + a separate alternate array; (5) the transfer-mode field was left at
  `0` (Stop) instead of `1` (Basic), so the controller never transferred.
- **rust-analyzer proc-macro mismatch** — the extension's bundled server
  (0.4.2322, proc-macro protocol 5) was older than the toolchain's proc-macro
  server (protocol 6). Fixed by pointing at the rustup binary
  (`rust-analyzer.server.path`).
- **rust-analyzer `can't find crate for 'test'`** — rust-analyzer checked
  `--all-targets` on a `#![no_std]` crate, whose test harness needs the `test`
  crate (std). Fixed with `rust-analyzer.check.allTargets = false`.
- **Cargo config merge** — parent and HAL both passed `-Tlink.x`, concatenating
  `rustflags` and defining `memory.x` twice; fixed by keeping the linker flag
  only in the top-level crate.

## Author

- **Author:** vortex314 (Lieven Merckx) <lieven.merckx@gmail.com>
- **Repository:** https://github.com/vortex314/lm4f120-rs
- **AI assistance:** developed with the help of **DeepSeek** (DeepSeek TUI,
  DeepSeek V4).

## License

MIT — see [LICENSE](LICENSE). You are free to use, copy, modify, merge,
publish, distribute, sublicense, and sell copies of the software, provided the
copyright notice and permission notice are included.
