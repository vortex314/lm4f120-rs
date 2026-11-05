# lm4f120-hal

Embedded-hal drivers for the TI Stellaris **LM4F120H5QR** (a Tiva-C class
Cortex-M4F MCU, as found on the EK-LM4F120XL LaunchPad).

The crate provides blocking, polling-based drivers for the on-chip peripherals,
implementing the [`embedded-hal`] 1.0 traits. It is `#![no_std]` and
register-level — no runtime or executor is required.

## Supported peripherals

| Module | Peripheral | Trait(s) implemented |
|--------|-----------|----------------------|
| [`gpio`] | GPIO ports A–H | `digital::{OutputPin, InputPin, StatefulOutputPin}` |
| [`uart`] | UART0–7 | `embedded_io::{Read, Write}` |
| [`spi`]  | SSI0–1 (SPI mode) | `spi::SpiBus<u8>` |
| [`i2c`]  | I2C0–3 (master) | `i2c::I2c` |

Note: embedded-hal 1.0 no longer defines a UART/serial abstraction; byte
streams moved to the companion [`embedded-io`] crate, which is re-exported as
`lm4f120_hal::embedded_io`.

## Clocking

Drivers take the system clock frequency explicitly (e.g. `Uart::new(115200,
16_000_000)`). The chip runs from its default 16 MHz precision internal
oscillator unless the application reconfigures the PLL.

## Usage

### Digital GPIO

```rust
use lm4f120_hal::gpio::{Pin, PortF, Output, Input, Pull};

let mut led = Pin::<PortF, Output, 1>::new(); // PF1
led.set_high();

let button = Pin::<PortF, Input, 4>::new(Pull::Up); // PF4, pull-up
if button.is_high() { /* ... */ }
```

### UART

```rust
use lm4f120_hal::embedded_io::Write;
use lm4f120_hal::uart::{Uart, Uart0};

let mut uart = Uart::<Uart0>::new(115200, 16_000_000);
let _ = uart.write(b"hello\r\n");
```

### SPI

```rust
use lm4f120_hal::embedded_hal::spi::{MODE_0, SpiBus};
use lm4f120_hal::spi::{Spi, Ssi0};

let mut spi = Spi::<Ssi0>::new(MODE_0, 1_000_000, 16_000_000);
let mut buf = [0xAA, 0x55];
spi.transfer_in_place(&mut buf).unwrap();
```

### I2C

```rust
use lm4f120_hal::embedded_hal::i2c::I2c;
use lm4f120_hal::i2c::{I2c, I2c0};

let mut i2c = I2c::<I2c0>::new(100_000, 16_000_000);
let mut data = [0u8; 1];
i2c.read(0x48, &mut data).ok(); // read one byte from address 0x48
```

## Pin mappings

Default alternate-function pins used by the drivers:

| Peripheral | RX / MISO / SDA | TX / MOSI / SCL |
|-----------|-----------------|-----------------|
| UART0 | PA0 | PA1 |
| UART1 | PB0 | PB1 |
| UART2 | PD6 | PD7 |
| UART3 | PC6 | PC7 |
| UART4 | PC4 | PC5 |
| UART5 | PE4 | PE5 |
| UART6 | PD4 | PD5 |
| UART7 | PE0 | PE1 |
| SSI0 | PA4 (MISO) | PA5 (MOSI), PA2 (SCK) |
| SSI1 | PF0 (MISO) | PF1 (MOSI), PF2 (SCK) |
| I2C0 | PB3 (SDA) | PB2 (SCL) |
| I2C1 | PA7 (SDA) | PA6 (SCL) |
| I2C2 | PE5 (SDA) | PE4 (SCL) |
| I2C3 | PD1 (SDA) | PD0 (SCL) |

## Examples

Run `cargo build --examples` (or `--release --examples`) to compile the
samples; flash them with `lm4flash` as you would any firmware image.

- `gpio_blink` — blinks the on-board RGB LED (PF1).
- `uart_echo` — echoes bytes received on UART0.
- `spi_loopback` — performs an SPI transfer (SSI0).
- `i2c_scan` — scans the I²C bus (I2C0) and prints found addresses over UART0.

## License

MIT OR Apache-2.0.

[`embedded-hal`]: https://docs.rs/embedded-hal
[`embedded-io`]: https://docs.rs/embedded-io
[`gpio`]: gpio
[`uart`]: uart
[`spi`]: spi
[`i2c`]: i2c
