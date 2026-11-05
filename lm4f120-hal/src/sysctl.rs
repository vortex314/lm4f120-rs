//! System control (clock gating) helpers.

/// SYSCTL base address.
pub const SYSCTL: u32 = 0x400F_E000;

const RCGCGPIO: u32 = SYSCTL + 0x608;
const RCGCUART: u32 = SYSCTL + 0x618;
const RCGCSSI: u32 = SYSCTL + 0x61C;
const RCGCI2C: u32 = SYSCTL + 0x620;

fn enable(reg: u32, index: u32) {
    unsafe {
        crate::write_reg(reg, crate::read_reg(reg) | (1 << index));
        // Let the peripheral clock settle before it is accessed.
        for _ in 0..32 {
            core::hint::spin_loop();
        }
    }
}

/// Enable the clock for a GPIO port (index = port letter A=0 .. H=7).
pub fn enable_gpio(index: u32) {
    enable(RCGCGPIO, index);
}

/// Enable the clock for a UART module (index = UART number).
pub fn enable_uart(index: u32) {
    enable(RCGCUART, index);
}

/// Enable the clock for an SSI module (index = SSI number).
pub fn enable_ssi(index: u32) {
    enable(RCGCSSI, index);
}

/// Enable the clock for an I²C module (index = I²C number).
pub fn enable_i2c(index: u32) {
    enable(RCGCI2C, index);
}
