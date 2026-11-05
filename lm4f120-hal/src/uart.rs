//! UART serial driver ([`embedded_io`] `Read`/`Write`).
//!
//! embedded-hal 1.0 no longer provides a UART abstraction; byte streams moved
//! to the companion [`embedded_io`] crate, which this driver implements.

use core::marker::PhantomData;

use embedded_io::{ErrorType, Read, Write};

use crate::sysctl;

// UART register offsets.
const UARTDR: u32 = 0x000;
const UARTRSR: u32 = 0x004;
const UARTFR: u32 = 0x018;
const UARTIBRD: u32 = 0x024;
const UARTFBRD: u32 = 0x028;
const UARTLCRH: u32 = 0x02C;
const UARTCTL: u32 = 0x030;

/// A UART peripheral instance.
pub trait UartInstance {
    /// Base address of the UART register block.
    const BASE: u32;
    /// UART number used for clock gating.
    const INDEX: u32;
    /// Configure the RX/TX pins for their UART alternate function.
    fn configure_pins();
}

fn uart_pin(port_base: u32, port_index: u32, pin: u8) {
    // Route the pin to the UART alternate function (GPIOPCTL mux value 0x1).
    sysctl::enable_gpio(port_index);
    unsafe {
        let afsel = port_base + 0x420;
        let den = port_base + 0x51C;
        let pctl = port_base + 0x52C;
        crate::write_reg(afsel, crate::read_reg(afsel) | (1 << pin));
        crate::write_reg(den, crate::read_reg(den) | (1 << pin));
        let v = crate::read_reg(pctl);
        crate::write_reg(pctl, (v & !(0xF << (4 * pin))) | (1 << (4 * pin)));
    }
}

/// UART0: RX = PA0, TX = PA1.
pub struct Uart0;
impl UartInstance for Uart0 {
    const BASE: u32 = 0x4000_C000;
    const INDEX: u32 = 0;
    fn configure_pins() {
        uart_pin(0x4000_4000, 0, 0); // PA0 RX
        uart_pin(0x4000_4000, 0, 1); // PA1 TX
    }
}

/// UART1: RX = PB0, TX = PB1.
pub struct Uart1;
impl UartInstance for Uart1 {
    const BASE: u32 = 0x4000_D000;
    const INDEX: u32 = 1;
    fn configure_pins() {
        uart_pin(0x4000_5000, 1, 0); // PB0 RX
        uart_pin(0x4000_5000, 1, 1); // PB1 TX
    }
}

/// UART2: RX = PD6, TX = PD7.
pub struct Uart2;
impl UartInstance for Uart2 {
    const BASE: u32 = 0x4000_E000;
    const INDEX: u32 = 2;
    fn configure_pins() {
        uart_pin(0x4000_7000, 3, 6); // PD6 RX
        uart_pin(0x4000_7000, 3, 7); // PD7 TX
    }
}

/// UART3: RX = PC6, TX = PC7.
pub struct Uart3;
impl UartInstance for Uart3 {
    const BASE: u32 = 0x4000_F000;
    const INDEX: u32 = 3;
    fn configure_pins() {
        uart_pin(0x4000_6000, 2, 6); // PC6 RX
        uart_pin(0x4000_6000, 2, 7); // PC7 TX
    }
}

/// UART4: RX = PC4, TX = PC5.
pub struct Uart4;
impl UartInstance for Uart4 {
    const BASE: u32 = 0x4001_0000;
    const INDEX: u32 = 4;
    fn configure_pins() {
        uart_pin(0x4000_6000, 2, 4); // PC4 RX
        uart_pin(0x4000_6000, 2, 5); // PC5 TX
    }
}

/// UART5: RX = PE4, TX = PE5.
pub struct Uart5;
impl UartInstance for Uart5 {
    const BASE: u32 = 0x4001_1000;
    const INDEX: u32 = 5;
    fn configure_pins() {
        uart_pin(0x4002_4000, 4, 4); // PE4 RX
        uart_pin(0x4002_4000, 4, 5); // PE5 TX
    }
}

/// UART6: RX = PD4, TX = PD5.
pub struct Uart6;
impl UartInstance for Uart6 {
    const BASE: u32 = 0x4001_2000;
    const INDEX: u32 = 6;
    fn configure_pins() {
        uart_pin(0x4000_7000, 3, 4); // PD4 RX
        uart_pin(0x4000_7000, 3, 5); // PD5 TX
    }
}

/// UART7: RX = PE0, TX = PE1.
pub struct Uart7;
impl UartInstance for Uart7 {
    const BASE: u32 = 0x4001_3000;
    const INDEX: u32 = 7;
    fn configure_pins() {
        uart_pin(0x4002_4000, 4, 0); // PE0 RX
        uart_pin(0x4002_4000, 4, 1); // PE1 TX
    }
}

/// UART error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    /// Receive overrun.
    Overrun,
    /// Framing error.
    Framing,
    /// Parity error.
    Parity,
    /// Break condition.
    Break,
}

impl embedded_io::Error for UartError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// A UART peripheral in 8N1 mode.
pub struct Uart<I: UartInstance> {
    _i: PhantomData<I>,
}

impl<I: UartInstance> Uart<I> {
    /// Enable the UART and configure it for the given baud rate (8 data bits,
    /// no parity, one stop bit) at `sysclk` Hz.
    pub fn new(baud: u32, sysclk: u32) -> Self {
        sysctl::enable_uart(I::INDEX);
        I::configure_pins();

        let base = I::BASE;
        unsafe {
            // Disable while configuring.
            crate::write_reg(base + UARTCTL, 0);

            // BRD = sysclk / (16 * baud). Split into integer and fractional parts.
            let divisor = 16 * baud;
            let ibrd = sysclk / divisor;
            let fbrd = ((sysclk % divisor) * 64 + divisor / 2) / divisor;
            crate::write_reg(base + UARTIBRD, ibrd);
            crate::write_reg(base + UARTFBRD, fbrd);

            // 8 bits, FIFO enabled, no parity, 1 stop bit.
            crate::write_reg(base + UARTLCRH, 0x70);
            // Enable UART, TX and RX.
            crate::write_reg(base + UARTCTL, 0x0301);
        }

        Self { _i: PhantomData }
    }
}

impl<I: UartInstance> ErrorType for Uart<I> {
    type Error = UartError;
}

impl<I: UartInstance> Write for Uart<I> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, UartError> {
        unsafe {
            for &b in buf {
                // Wait for the transmit FIFO to have room (TXFF, bit 5).
                while crate::read_reg(I::BASE + UARTFR) & (1 << 5) != 0 {
                    core::hint::spin_loop();
                }
                crate::write_reg(I::BASE + UARTDR, b as u32);
            }
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), UartError> {
        unsafe {
            // Wait for the transmit FIFO to empty (TXFE, bit 7).
            while crate::read_reg(I::BASE + UARTFR) & (1 << 7) == 0 {
                core::hint::spin_loop();
            }
        }
        Ok(())
    }
}

impl<I: UartInstance> Read for Uart<I> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, UartError> {
        if buf.is_empty() {
            return Ok(0);
        }

        unsafe {
            // Block until a byte is available (RXFE, bit 4).
            while crate::read_reg(I::BASE + UARTFR) & (1 << 4) != 0 {
                core::hint::spin_loop();
            }

            // Check and clear receive errors (UARTRSR bits 3:0 = OE, BE, PE, FE).
            let rsr = crate::read_reg(I::BASE + UARTRSR);
            if rsr & 0x0F != 0 {
                crate::write_reg(I::BASE + UARTRSR, 0); // clear via write to ECR
                return Err(if rsr & 0x08 != 0 {
                    UartError::Overrun
                } else if rsr & 0x02 != 0 {
                    UartError::Parity
                } else if rsr & 0x01 != 0 {
                    UartError::Framing
                } else {
                    UartError::Break
                });
            }

            buf[0] = (crate::read_reg(I::BASE + UARTDR) & 0xFF) as u8;
            Ok(1)
        }
    }
}
