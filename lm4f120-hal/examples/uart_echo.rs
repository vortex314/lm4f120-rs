#![no_std]
#![no_main]

//! Echo bytes received on UART0.

use cortex_m_rt::entry;
use lm4f120_hal::embedded_io::{Read, Write};
use lm4f120_hal::uart::{Uart, Uart0};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let mut uart = Uart::<Uart0>::new(115200, 16_000_000);
    let _ = uart.write(b"UART0 echo ready\r\n");

    loop {
        let mut buf = [0u8; 1];
        if uart.read(&mut buf).is_ok() {
            let _ = uart.write(&buf);
        }
    }
}
