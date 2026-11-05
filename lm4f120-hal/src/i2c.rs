//! I²C master driver ([`embedded_hal::i2c`]).

use core::marker::PhantomData;

use embedded_hal::i2c::{ErrorType, I2c as I2cTrait, Operation, SevenBitAddress};

use crate::sysctl;

// I2C master register offsets.
const I2CMSA: u32 = 0x000;
const I2CMCS: u32 = 0x004;
const I2CMDR: u32 = 0x008;
const I2CMTPR: u32 = 0x00C;
const I2CMCR: u32 = 0x020;

// I2CMCS bits.
const RUN: u32 = 1 << 0;
const START: u32 = 1 << 1;
const STOP: u32 = 1 << 2;
const ACK: u32 = 1 << 3;
const ERROR: u32 = 1 << 5;
const BUSY: u32 = 1 << 6;
const DATACK: u32 = 1 << 10;

/// An I²C peripheral instance.
pub trait I2cInstance {
    /// Base address of the I²C register block.
    const BASE: u32;
    /// I²C number used for clock gating.
    const INDEX: u32;
    /// Configure the SCL/SDA pins for their I²C alternate function.
    fn configure_pins();
}

fn i2c_pin(port_base: u32, port_index: u32, pin: u8) {
    // I²C pins use GPIOPCTL mux value 0x3, and are open-drain.
    sysctl::enable_gpio(port_index);
    unsafe {
        let afsel = port_base + 0x420;
        let den = port_base + 0x51C;
        let pctl = port_base + 0x52C;
        let odr = port_base + 0x50C;
        crate::write_reg(afsel, crate::read_reg(afsel) | (1 << pin));
        crate::write_reg(den, crate::read_reg(den) | (1 << pin));
        crate::write_reg(odr, crate::read_reg(odr) | (1 << pin));
        let v = crate::read_reg(pctl);
        crate::write_reg(pctl, (v & !(0xF << (4 * pin))) | (3 << (4 * pin)));
    }
}

/// I2C0: SCL = PB2, SDA = PB3.
pub struct I2c0;
impl I2cInstance for I2c0 {
    const BASE: u32 = 0x4002_0000;
    const INDEX: u32 = 0;
    fn configure_pins() {
        i2c_pin(0x4000_5000, 1, 2); // PB2 SCL
        i2c_pin(0x4000_5000, 1, 3); // PB3 SDA
    }
}

/// I2C1: SCL = PA6, SDA = PA7.
pub struct I2c1;
impl I2cInstance for I2c1 {
    const BASE: u32 = 0x4002_1000;
    const INDEX: u32 = 1;
    fn configure_pins() {
        i2c_pin(0x4000_4000, 0, 6); // PA6 SCL
        i2c_pin(0x4000_4000, 0, 7); // PA7 SDA
    }
}

/// I2C2: SCL = PE4, SDA = PE5.
pub struct I2c2;
impl I2cInstance for I2c2 {
    const BASE: u32 = 0x4002_2000;
    const INDEX: u32 = 2;
    fn configure_pins() {
        i2c_pin(0x4002_4000, 4, 4); // PE4 SCL
        i2c_pin(0x4002_4000, 4, 5); // PE5 SDA
    }
}

/// I2C3: SCL = PD0, SDA = PD1.
pub struct I2c3;
impl I2cInstance for I2c3 {
    const BASE: u32 = 0x4002_3000;
    const INDEX: u32 = 3;
    fn configure_pins() {
        i2c_pin(0x4000_7000, 3, 0); // PD0 SCL
        i2c_pin(0x4000_7000, 3, 1); // PD1 SDA
    }
}

/// I²C error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    /// Bus error or clock timeout.
    Bus,
    /// Arbitration lost.
    ArbitrationLost,
    /// Address or data not acknowledged.
    Nack,
}

impl embedded_hal::i2c::Error for I2cError {}

/// An I²C master peripheral.
pub struct I2c<I: I2cInstance> {
    _i: PhantomData<I>,
}

impl<I: I2cInstance> I2c<I> {
    /// Enable the I²C master at the given SCL frequency (`sysclk` in Hz).
    pub fn new(freq_hz: u32, sysclk: u32) -> Self {
        sysctl::enable_i2c(I::INDEX);
        I::configure_pins();

        let base = I::BASE;
        unsafe {
            // Enable master function (MFE, bit 4 of I2CMCR).
            crate::write_reg(base + I2CMCR, crate::read_reg(base + I2CMCR) | (1 << 4));

            // SCL = sysclk / (2 * (1 + TPR) * (SCL_LP + SCL_HP)); SCL_LP=6, SCL_HP=4.
            let tpr = sysclk / (20 * freq_hz.max(1)) - 1;
            crate::write_reg(base + I2CMTPR, tpr.min(0x7F));
        }

        Self { _i: PhantomData }
    }

    fn wait_idle(&mut self) -> Result<(), I2cError> {
        unsafe {
            while crate::read_reg(I::BASE + I2CMCS) & BUSY != 0 {
                core::hint::spin_loop();
            }
            if crate::read_reg(I::BASE + I2CMCS) & ERROR != 0 {
                return Err(I2cError::Bus);
            }
            Ok(())
        }
    }

    fn send_byte(&mut self, addr: u8, byte: u8, start: bool, stop: bool) -> Result<(), I2cError> {
        let base = I::BASE;
        unsafe {
            crate::write_reg(base + I2CMSA, ((addr as u32 & 0x7F) << 1) | 0);
            crate::write_reg(base + I2CMDR, byte as u32);
            let mut cs = RUN;
            if start {
                cs |= START;
            }
            if stop {
                cs |= STOP;
            }
            crate::write_reg(base + I2CMCS, cs);
        }
        self.wait_idle()?;
        // DATACK is cleared if the slave did not acknowledge the data.
        if unsafe { crate::read_reg(I::BASE + I2CMCS) & DATACK } == 0 {
            return Err(I2cError::Nack);
        }
        Ok(())
    }

    fn recv_byte(&mut self, addr: u8, start: bool, stop: bool) -> Result<u8, I2cError> {
        let base = I::BASE;
        unsafe {
            crate::write_reg(base + I2CMSA, ((addr as u32 & 0x7F) << 1) | 1);
            let mut cs = RUN;
            if start {
                cs |= START;
            }
            if !stop {
                cs |= ACK;
            }
            if stop {
                cs |= STOP;
            }
            crate::write_reg(base + I2CMCS, cs);
        }
        self.wait_idle()?;
        Ok(unsafe { (crate::read_reg(I::BASE + I2CMDR) & 0xFF) as u8 })
    }
}

impl<I: I2cInstance> ErrorType for I2c<I> {
    type Error = I2cError;
}

impl<I: I2cInstance> I2cTrait for I2c<I> {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let addr = address;

        let total: usize = operations
            .iter()
            .map(|op| match op {
                Operation::Read(b) => b.len(),
                Operation::Write(b) => b.len(),
            })
            .sum();

        let mut idx = 0usize;
        let mut prev_read = false;
        let mut first = true;

        for op in operations.iter_mut() {
            match op {
                Operation::Write(data) => {
                    for &b in data.iter() {
                        let last = idx == total - 1;
                        let start = first || prev_read;
                        self.send_byte(addr, b, start, last)?;
                        prev_read = false;
                        first = false;
                        idx += 1;
                    }
                }
                Operation::Read(data) => {
                    for b in data.iter_mut() {
                        let last = idx == total - 1;
                        let start = first || !prev_read;
                        *b = self.recv_byte(addr, start, last)?;
                        prev_read = true;
                        first = false;
                        idx += 1;
                    }
                }
            }
        }
        Ok(())
    }
}
