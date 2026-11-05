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

            // Pick a prescaler that produces a clock <= freq_hz.
            let mut cpsdvsr = 2u32;
            while cpsdvsr < 254 && sysclk / cpsdvsr > freq_hz {
                cpsdvsr += 2;
            }
            crate::write_reg(base + SSICPSR, cpsdvsr);

            // CR0: 8-bit data (DSS=7), Motorola frame (FRF=0), SPI mode, SCR=0.
            let spo = if mode.polarity == embedded_hal::spi::Polarity::IdleHigh { 1u32 } else { 0 };
            let sph = if mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition { 1u32 } else { 0 };
            let cr0 = (7 << 8) | (spo << 5) | (sph << 4);
            crate::write_reg(base + SSICR0, cr0);

            // CR1: enable, master mode.
            crate::write_reg(base + SSICR1, (1 << 1) | (1 << 2));
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
}

impl<I: SpiInstance> Spi<I> {
    fn transfer_byte(&mut self, tx: u8) -> u8 {
        let base = I::BASE;
        unsafe {
            // Wait for TX FIFO not full (TFF, bit 1).
            while crate::read_reg(base + SSISR) & (1 << 1) != 0 {
                core::hint::spin_loop();
            }
            crate::write_reg(base + SSIDR, tx as u32);
            // Wait for RX FIFO not empty (RNE, bit 2).
            while crate::read_reg(base + SSISR) & (1 << 2) == 0 {
                core::hint::spin_loop();
            }
            (crate::read_reg(base + SSIDR) & 0xFF) as u8
        }
    }
}
