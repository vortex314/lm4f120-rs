//! PWM output via the motion-control PWM module ([`embedded_hal::pwm`]).

use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::pwm::{ErrorType, SetDutyCycle};

use crate::sysctl;

// PWM module register offsets.
const PWM_ENABLE: u32 = 0x008;
// Each generator (0..=3) occupies a 0x40-byte block starting at 0x040.
const GEN_CTL: u32 = 0x000;
const GEN_LOAD: u32 = 0x010;
const GEN_CMPA: u32 = 0x018;
const GEN_CMPB: u32 = 0x01C;
const GEN_GENA: u32 = 0x020;
const GEN_GENB: u32 = 0x024;

// Generator CTL bits. Down-count mode is bit 1 clear.
const GEN_ENABLE: u32 = 1 << 0;

// GENA/GENB value for left-aligned (down-count) PWM: drive high on load,
// drive low on compare-down.
const GENX_DOWN: u32 = 0x8C; // ACTLOAD_ONE | ACTCMPAD_ZERO

/// A single PWM output channel.
pub trait PwmChannel {
    /// PWM module base address.
    const BASE: u32;
    /// Generator index (0..=3).
    const GEN: u32;
    /// Output within the generator: 0 = A (CMPA), 1 = B (CMPB).
    const OUTPUT: u32;
    /// Configure the output pin for its PWM alternate function.
    fn configure_pin();
}

fn pwm_pin(port_base: u32, port_index: u32, pin: u8, mux: u32) {
    sysctl::enable_gpio(port_index);
    unsafe {
        let afsel = port_base + 0x420;
        let den = port_base + 0x51C;
        let pctl = port_base + 0x52C;
        crate::write_reg(afsel, crate::read_reg(afsel) | (1 << pin));
        crate::write_reg(den, crate::read_reg(den) | (1 << pin));
        let v = crate::read_reg(pctl);
        crate::write_reg(pctl, (v & !(0xF << (4 * pin))) | (mux << (4 * pin)));
    }
}

/// M0PWM0 on PB6.
pub struct M0Pwm0;
impl PwmChannel for M0Pwm0 {
    const BASE: u32 = 0x4002_8000;
    const GEN: u32 = 0;
    const OUTPUT: u32 = 0;
    fn configure_pin() {
        pwm_pin(0x4000_5000, 1, 6, 4); // PB6, mux 4
    }
}

/// M0PWM1 on PB7.
pub struct M0Pwm1;
impl PwmChannel for M0Pwm1 {
    const BASE: u32 = 0x4002_8000;
    const GEN: u32 = 0;
    const OUTPUT: u32 = 1;
    fn configure_pin() {
        pwm_pin(0x4000_5000, 1, 7, 4); // PB7, mux 4
    }
}

/// A PWM channel.
pub struct Pwm<CH: PwmChannel> {
    /// Period minus one (the LOAD register value); also the maximum duty.
    load: u16,
    _ch: PhantomData<CH>,
}

impl<CH: PwmChannel> Pwm<CH> {
    /// Enable the channel at `freq_hz`, clocked by the system clock (`sysclk`
    /// in Hz).
    ///
    /// The PWM clock is divided as needed so the requested period fits the
    /// 16-bit LOAD register. The divider is global to the PWM modules, and
    /// `M0Pwm0`/`M0Pwm1` share generator 0, so create them with the same
    /// `freq_hz`.
    pub fn new(freq_hz: u32, sysclk: u32) -> Self {
        sysctl::enable_pwm(0); // PWM0
        CH::configure_pin();

        let base = CH::BASE;
        let load = unsafe {
            // Choose the smallest PWM clock divider (1, 2, 4, ... 64) whose
            // period = pwm_clk / freq_hz fits in 2..=65536 ticks.
            let mut divider = 1u32;
            let mut load = 0xFFFFu32;
            let mut found = false;
            while divider <= 64 {
                let period = (sysclk / divider).div_ceil(freq_hz.max(1));
                if period <= 65536 {
                    load = period.saturating_sub(1).max(1);
                    found = true;
                    break;
                }
                divider *= 2;
            }
            if !found {
                // Slower than the hardware can produce; use the slowest setting.
                divider = 64;
                load = 0xFFFF;
            }

            set_pwm_clock_divider(divider);

            let gen = base + 0x040 + CH::GEN * 0x40;
            // Configure the generator: down count, left-aligned PWM.
            crate::write_reg(gen + GEN_CTL, 0); // disable while configuring
            crate::write_reg(gen + GEN_LOAD, load);
            crate::write_reg(gen + GEN_GENA, GENX_DOWN);
            crate::write_reg(gen + GEN_GENB, GENX_DOWN);
            crate::write_reg(gen + GEN_CMPA, load); // start at 0% duty
            crate::write_reg(gen + GEN_CMPB, load);

            // Enable the output, then the generator.
            let out = CH::GEN * 2 + CH::OUTPUT;
            crate::write_reg(base + PWM_ENABLE, crate::read_reg(base + PWM_ENABLE) | (1 << out));
            crate::write_reg(gen + GEN_CTL, GEN_ENABLE);

            load
        };

        Self { load: load as u16, _ch: PhantomData }
    }

    fn cmp_reg(&self) -> u32 {
        let gen = CH::BASE + 0x040 + CH::GEN * 0x40;
        if CH::OUTPUT == 0 {
            gen + GEN_CMPA
        } else {
            gen + GEN_CMPB
        }
    }
}

impl<CH: PwmChannel> ErrorType for Pwm<CH> {
    type Error = Infallible;
}

impl<CH: PwmChannel> SetDutyCycle for Pwm<CH> {
    fn max_duty_cycle(&self) -> u16 {
        self.load
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        // Duty 0 -> CMP = load (always low); duty load -> CMP = 0 (always high).
        let cmp = self.load.saturating_sub(duty.min(self.load));
        unsafe { crate::write_reg(self.cmp_reg(), cmp as u32) };
        Ok(())
    }
}

/// Set the PWM clock divider (SYSCTL_RCC PWMDIV field), which is shared by
/// both PWM modules.
fn set_pwm_clock_divider(divider: u32) {
    const RCC: u32 = sysctl::SYSCTL + 0x060;
    const PWMDIV_M: u32 = 0x000E_0000; // bits [19:17]
    const USEPWMDIV: u32 = 1 << 20;
    let value = if divider <= 1 {
        0 // no division
    } else {
        // divider = 2^(field + 1); field = log2(divider) - 1.
        USEPWMDIV | ((divider.trailing_zeros() - 1) << 17)
    };
    unsafe {
        crate::write_reg(RCC, (crate::read_reg(RCC) & !(PWMDIV_M | USEPWMDIV)) | value);
    }
}
