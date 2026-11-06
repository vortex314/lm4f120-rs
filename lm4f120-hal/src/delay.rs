//! Busy-wait delay using the Cortex-M SysTick timer.
//!
//! Implements [`embedded_hal::delay::DelayNs`]. The timer runs from the
//! processor clock, so pass the same frequency the other drivers take
//! (16 MHz from the reset oscillator, or 80 MHz after PLL setup).

use embedded_hal::delay::DelayNs;

// SysTick (Cortex-M core peripheral) register block.
const SYST_BASE: u32 = 0xE000_E010;
const SYST_CSR: u32 = 0x00; // control and status
const SYST_RVR: u32 = 0x04; // reload value
const SYST_CVR: u32 = 0x08; // current value

// SYST_CSR bits.
const CSR_ENABLE: u32 = 1 << 0;
const CSR_CLKSOURCE: u32 = 1 << 2; // 1 = processor clock
const CSR_COUNTFLAG: u32 = 1 << 16;

/// SysTick is a 24-bit down counter; a single shot spans 1..=2^24 clock
/// cycles. A reload value of 0 would pin the counter at zero and never set
/// COUNTFLAG, so each shot is kept in 2..=2^24 cycles.
const MAX_TICKS: u32 = 0x0100_0000;

/// Busy-wait delay, clocked by the processor clock.
pub struct Delay {
    /// Processor clock frequency in Hz.
    sysclk: u32,
}

impl Delay {
    /// Create a delay provider for the given processor clock (`sysclk` in Hz).
    pub fn new(sysclk: u32) -> Self {
        Self { sysclk }
    }

    /// Block for `ticks` processor-clock cycles.
    fn delay_ticks(&mut self, ticks: u32) {
        let mut remaining = ticks;
        while remaining > 0 {
            let chunk = remaining.clamp(2, MAX_TICKS);
            unsafe {
                crate::write_reg(SYST_BASE + SYST_RVR, chunk - 1);
                // Writing CVR clears the counter and the COUNTFLAG.
                crate::write_reg(SYST_BASE + SYST_CVR, 0);
                crate::write_reg(SYST_BASE + SYST_CSR, CSR_ENABLE | CSR_CLKSOURCE);
                while crate::read_reg(SYST_BASE + SYST_CSR) & CSR_COUNTFLAG == 0 {
                    core::hint::spin_loop();
                }
                crate::write_reg(SYST_BASE + SYST_CSR, 0);
            }
            remaining = remaining.saturating_sub(chunk);
        }
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // ticks = ns * sysclk / 1e9, computed in u64 to avoid overflow. At
        // least one cycle so sub-cycle requests still spin briefly.
        let ticks = (ns as u64 * self.sysclk as u64 / 1_000_000_000u64) as u32;
        self.delay_ticks(ticks.max(1));
    }
}
