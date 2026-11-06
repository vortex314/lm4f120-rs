//! System control (clock gating) helpers.

/// SYSCTL base address.
pub const SYSCTL: u32 = 0x400F_E000;

const RCGCGPIO: u32 = SYSCTL + 0x608;
const RCGCUART: u32 = SYSCTL + 0x618;
const RCGCSSI: u32 = SYSCTL + 0x61C;
const RCGCI2C: u32 = SYSCTL + 0x620;
const RCGCPWM: u32 = SYSCTL + 0x640;

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

/// Enable the clock for a PWM module (index = PWM module number).
pub fn enable_pwm(index: u32) {
    enable(RCGCPWM, index);
}

/// Configure the system clock to 80 MHz using the PLL.
///
/// The on-chip PLL multiplies the 16 MHz main crystal up to a 400 MHz VCO,
/// which is then divided by 5 to produce 80 MHz. The device must have a 16 MHz
/// crystal fitted (the EK-LM4F120XL LaunchPad does).
pub fn setup_80mhz() {
    const RCC: u32 = SYSCTL + 0x060;
    const RCC2: u32 = SYSCTL + 0x070;
    const RIS: u32 = SYSCTL + 0x050;

    unsafe {
        // 1. Run from the 16 MHz crystal (bypass the PLL), enable the main
        //    oscillator, and tell the PLL the crystal is 16 MHz.
        //    RCC: BYPASS=1, XTAL=16 MHz, OSCSRC=main oscillator, MOSCDIS=0.
        crate::write_reg(RCC, (1 << 11) | (0x15 << 6));

        // 2. Wait for the PLL to lock (RIS bit 6 = PLL lock).
        while crate::read_reg(RIS) & (1 << 6) == 0 {
            core::hint::spin_loop();
        }

        // 3. Switch to the PLL at 80 MHz.
        //    RCC2: USERCC2=1, DIV400=1 (400 MHz VCO), SYSDIV2=2 (÷5 → 80 MHz).
        crate::write_reg(RCC2, (1 << 31) | (1 << 30) | (2 << 23));
    }
}
