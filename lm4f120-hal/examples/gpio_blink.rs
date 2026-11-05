#![no_std]
#![no_main]

//! Blink the on-board RGB LED (PF1 = red, PF2 = blue, PF3 = green) using the
//! lm4f120-hal GPIO driver.

use cortex_m_rt::entry;
use lm4f120_hal::gpio::{Output, Pin, PortF};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let mut red = Pin::<PortF, Output, 1>::new();
    let mut blue = Pin::<PortF, Output, 2>::new();
    let mut green = Pin::<PortF, Output, 3>::new();

    loop {
        // Red.
        red.set_high();
        green.set_low();
        blue.set_low();
        delay();

        // Green.
        red.set_low();
        green.set_high();
        blue.set_low();
        delay();

        // Blue.
        red.set_low();
        green.set_low();
        blue.set_high();
        delay();
    }
}

fn delay() {
    for _ in 0..2_000_000 {
        cortex_m::asm::nop();
    }
}
