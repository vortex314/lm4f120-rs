#![no_std]
#![no_main]

//! Perform repeated SPI transfers on SSI0.
//!
//! With MISO and MOSI looped together on the board, the bytes echo back
//! unchanged; otherwise the received bytes reflect whatever the slave returns.

use cortex_m_rt::entry;
use lm4f120_hal::embedded_hal::spi::{MODE_0, SpiBus};
use lm4f120_hal::spi::{Spi, Ssi0};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let mut spi = Spi::<Ssi0>::new(MODE_0, 1_000_000, 16_000_000);
    let mut buf = [0xAAu8, 0x55, 0x01, 0x02, 0x03];

    loop {
        spi.transfer_in_place(&mut buf).ok();
        delay();
    }
}

fn delay() {
    for _ in 0..1_000_000 {
        cortex_m::asm::nop();
    }
}
