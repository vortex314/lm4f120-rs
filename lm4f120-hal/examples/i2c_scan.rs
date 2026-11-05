#![no_std]
#![no_main]

//! Scan the I2C bus on I2C0 and print the addresses of responding devices
//! over UART0.

use cortex_m_rt::entry;
use lm4f120_hal::embedded_hal::i2c::I2c as _;
use lm4f120_hal::embedded_io::Write;
use lm4f120_hal::i2c::{I2c, I2c0};
use lm4f120_hal::uart::{Uart, Uart0};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let mut i2c = I2c::<I2c0>::new(100_000, 16_000_000);
    let mut uart = Uart::<Uart0>::new(115200, 16_000_000);
    let _ = uart.write(b"I2C scan start\r\n");

    loop {
        for addr in 0x08..=0x77u8 {
            let mut byte = [0u8; 1];
            if i2c.read(addr, &mut byte).is_ok() {
                print_hex(&mut uart, addr);
                let _ = uart.write(b"\r\n");
            }
        }
        let _ = uart.write(b"scan complete\r\n");
        delay();
    }
}

fn print_hex(uart: &mut Uart<Uart0>, byte: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let _ = uart.write(&[HEX[(byte >> 4) as usize]]);
    let _ = uart.write(&[HEX[(byte & 0x0F) as usize]]);
}

fn delay() {
    for _ in 0..2_000_000 {
        cortex_m::asm::nop();
    }
}
