#![no_std]

//! Embedded-hal drivers for the TI Stellaris LM4F120H5QR (Tiva-C class) MCU.
//!
//! This crate provides blocking, polling-based drivers for the on-chip
//! peripherals, implementing the [`embedded-hal`] 1.0 traits:
//!
//! - [`gpio`] — digital I/O with typestate pins ([`embedded_hal::digital`]).
//! - [`uart`] — UART serial ([`embedded_hal::serial`]).
//! - [`spi`] — SSI in SPI mode ([`embedded_hal::spi`]).
//! - [`i2c`] — I²C master ([`embedded_hal::i2c`]).
//!
//! The chip runs from its default 16 MHz precision internal oscillator unless
//! the application reconfigures the PLL. Clock gating uses the per-peripheral
//! run-mode clock registers (RCGCGPIO, RCGCUART, RCGCSSI, RCGCI2C).

pub mod gpio;
pub mod i2c;
pub mod spi;
mod sysctl;
pub mod uart;

pub use gpio::{Input, Output, Pin, Port, PortA, PortB, PortC, PortD, PortE, PortF, PortG, PortH, Pull};
pub use i2c::{I2c, I2c0, I2c1, I2c2, I2c3, I2cError, I2cInstance};
pub use spi::{Spi, SpiError, SpiInstance, Ssi0, Ssi1};
pub use uart::{
    Uart, Uart0, Uart1, Uart2, Uart3, Uart4, Uart5, Uart6, Uart7, UartError, UartInstance,
};
pub use sysctl::setup_80mhz;

// Re-export the trait crates so downstream crates don't need their own copies.
pub use embedded_hal;
pub use embedded_io;

#[inline(always)]
pub(crate) unsafe fn read_reg(addr: u32) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

#[inline(always)]
pub(crate) unsafe fn write_reg(addr: u32, val: u32) {
    core::ptr::write_volatile(addr as *mut u32, val)
}
