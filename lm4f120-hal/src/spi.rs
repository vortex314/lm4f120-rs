//! SSI in SPI mode ([`embedded_hal::spi`]).

use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::spi::{ErrorType, Mode, SpiBus};

use crate::sysctl;

// SSI register offsets.
const SSICR0: u32 = 0x000;
const SSICR1: u32 = 0x004;
const SSIDR: u32 = 0x008;
const SSISR: u32 = 0x00C;
const SSICPSR: u32 = 0x010;

// SSICR1 bits.
const SSE: u32 = 1 << 1; // SSI enable (master when MS, bit 2, is left clear)

// SSISR bits.
const TNF: u32 = 1 << 1; // transmit FIFO not full
const RNE: u32 = 1 << 2; // receive FIFO not empty
const BSY: u32 = 1 << 4; // SSI busy

/// An SSI peripheral instance.
pub trait SpiInstance {
    /// Base address of the SSI register block.
    const BASE: u32;
    /// SSI number used for clock gating.
    const INDEX: u32;
    /// Configure the SCK/MOSI/MISO pins for their SSI alternate function.
    fn configure_pins();
}

fn ssi_pin(port_base: u32, port_index: u32, pin: u8) {
    // SSI pins use GPIOPCTL mux value 0x2 on this family.
    sysctl::enable_gpio(port_index);
    unsafe {
        // PF0 (NMI) and PD7 are locked at reset; unlock before touching the
        // commit-protected AFSEL/DEN registers. Only PF0 is used here (SSI1Rx).
        if (port_index == 5 && pin == 0) || (port_index == 3 && pin == 7) {
            crate::write_reg(port_base + 0x520, 0x4C4F_434B); // GPIOLOCK key
            crate::write_reg(
                port_base + 0x524,
                crate::read_reg(port_base + 0x524) | (1 << pin), // GPIOCR commit
            );
        }
        let afsel = port_base + 0x420;
        let den = port_base + 0x51C;
        let pctl = port_base + 0x52C;
        crate::write_reg(afsel, crate::read_reg(afsel) | (1 << pin));
        crate::write_reg(den, crate::read_reg(den) | (1 << pin));
        let v = crate::read_reg(pctl);
        crate::write_reg(pctl, (v & !(0xF << (4 * pin))) | (2 << (4 * pin)));
    }
}

/// SSI0: SCK = PA2, MOSI = PA5, MISO = PA4 (LaunchPad-style layout).
pub struct Ssi0;
impl SpiInstance for Ssi0 {
    const BASE: u32 = 0x4000_8000;
    const INDEX: u32 = 0;
    fn configure_pins() {
        ssi_pin(0x4000_4000, 0, 2); // PA2 SCK
        ssi_pin(0x4000_4000, 0, 4); // PA4 MISO
        ssi_pin(0x4000_4000, 0, 5); // PA5 MOSI
    }
}

/// SSI1: SCK = PF2, MOSI = PF1, MISO = PF0.
pub struct Ssi1;
impl SpiInstance for Ssi1 {
    const BASE: u32 = 0x4000_9000;
    const INDEX: u32 = 1;
    fn configure_pins() {
        ssi_pin(0x4002_5000, 5, 0); // PF0 MISO
        ssi_pin(0x4002_5000, 5, 1); // PF1 MOSI
        ssi_pin(0x4002_5000, 5, 2); // PF2 SCK
    }
}

/// SSI in SPI mode.
pub struct Spi<I: SpiInstance> {
    _i: PhantomData<I>,
}

/// SPI errors. The polling driver never reports a blocking error, so this is
/// [`Infallible`].
pub type SpiError = Infallible;

impl<I: SpiInstance> Spi<I> {
    /// Enable the SSI in master mode at the given SPI clock frequency (`sysclk`
    /// is the system clock in Hz). Data size is 8 bits.
    pub fn new(mode: Mode, freq_hz: u32, sysclk: u32) -> Self {
        sysctl::enable_ssi(I::INDEX);
        I::configure_pins();

        let base = I::BASE;
        unsafe {
            // Disable while configuring.
            crate::write_reg(base + SSICR1, 0);

            // SSInClk = sysclk / (cpsdvsr * (1 + scr)). Pick the fastest clock
            // at or below freq_hz. cpsdvsr is even (2..=254); scr is 0..=255.
            // If freq_hz is below sysclk / (254 * 256), this clamps to the
            // slowest setting the hardware can produce.
            let target = sysclk.div_ceil(freq_hz.max(1)).max(2);
            let mut cpsdvsr = 2u32;
            while cpsdvsr < 254 && cpsdvsr * 256 < target {
                cpsdvsr += 2;
            }
            let scr = if cpsdvsr * 256 >= target {
                target.div_ceil(cpsdvsr) - 1
            } else {
                255u32
            };
            crate::write_reg(base + SSICPSR, cpsdvsr);

            // CR0: SCR [15:8], SPH [7], SPO [6], FRF [5:4] = Motorola (0),
            // DSS [3:0] = 7 (8-bit).
            let spo = if mode.polarity == embedded_hal::spi::Polarity::IdleHigh { 1u32 } else { 0 };
            let sph = if mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition { 1u32 } else { 0 };
            let cr0 = 7 | (spo << 6) | (sph << 7) | (scr << 8);
            crate::write_reg(base + SSICR0, cr0);

            // CR1: enable (SSE); the MS bit is left clear for master mode.
            crate::write_reg(base + SSICR1, SSE);
        }

        Self { _i: PhantomData }
    }
}

impl<I: SpiInstance> ErrorType for Spi<I> {
    type Error = SpiError;
}

impl<I: SpiInstance> SpiBus<u8> for Spi<I> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(0x00);
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &w in words {
            let _ = self.transfer_byte(w);
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let len = read.len().max(write.len());
        for i in 0..len {
            let tx = write.get(i).copied().unwrap_or(0x00);
            let rx = self.transfer_byte(tx);
            if let Some(r) = read.get_mut(i) {
                *r = rx;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(*w);
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        unsafe {
            // Wait for the SSI to go idle (BSY).
            while crate::read_reg(I::BASE + SSISR) & BSY != 0 {
                core::hint::spin_loop();
            }
        }
        Ok(())
    }
}

impl<I: SpiInstance> Spi<I> {
    fn transfer_byte(&mut self, tx: u8) -> u8 {
        let base = I::BASE;
        unsafe {
            // Wait for room in the TX FIFO (TNF).
            while crate::read_reg(base + SSISR) & TNF == 0 {
                core::hint::spin_loop();
            }
            crate::write_reg(base + SSIDR, tx as u32);
            // Wait for RX data (RNE).
            while crate::read_reg(base + SSISR) & RNE == 0 {
                core::hint::spin_loop();
            }
            (crate::read_reg(base + SSIDR) & 0xFF) as u8
        }
    }
}
