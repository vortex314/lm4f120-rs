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

// I2CMCS is written and read through the same bits; the meaning of the low
// four bits depends on the direction of the access (TM4C123/LM4F120 manual,
// "I2C Master Control/Status").
//
// Write side:
const RUN: u32 = 1 << 0; // start the operation
const START: u32 = 1 << 1; // generate START / repeated START
const STOP: u32 = 1 << 2; // generate STOP
const ACK: u32 = 1 << 3; // acknowledge the received byte
//
// Read side:
const BUSY: u32 = 1 << 0; // controller busy (mirrors RUN while active)
const ERROR: u32 = 1 << 1; // last operation ended in error
const ADRACK: u32 = 1 << 2; // 1 = address was NOT acknowledged
const DATACK: u32 = 1 << 3; // 1 = data was NOT acknowledged
const ARBLST: u32 = 1 << 4; // arbitration lost
// Bit 5 is IDLE and bit 6 is BUSBSY; bit 7 is CLKTO (clock timeout, only
// active when I2CMCLKOCNT is configured). None are needed by this driver.

/// An I²C peripheral instance.
pub trait I2cInstance {
    /// Base address of the I²C register block.
    const BASE: u32;
    /// I²C number used for clock gating.
    const INDEX: u32;
    /// Configure the SCL/SDA pins for their I²C alternate function.
    fn configure_pins();
}

/// Configure a GPIO pin for its I²C alternate function (PCTL mux value 0x3).
///
/// SDA pins must be open-drain; SCL pins stay push-pull (matching TivaWare's
/// `GPIOPinTypeI2C` / `GPIOPinTypeI2CSCL`).
fn i2c_pin(port_base: u32, port_index: u32, pin: u8, open_drain: bool) {
    sysctl::enable_gpio(port_index);
    unsafe {
        let afsel = port_base + 0x420;
        let den = port_base + 0x51C;
        let pctl = port_base + 0x52C;
        let odr = port_base + 0x50C;
        crate::write_reg(afsel, crate::read_reg(afsel) | (1 << pin));
        crate::write_reg(den, crate::read_reg(den) | (1 << pin));
        if open_drain {
            crate::write_reg(odr, crate::read_reg(odr) | (1 << pin));
        }
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
        i2c_pin(0x4000_5000, 1, 2, false); // PB2 SCL
        i2c_pin(0x4000_5000, 1, 3, true); // PB3 SDA
    }
}

/// I2C1: SCL = PA6, SDA = PA7.
pub struct I2c1;
impl I2cInstance for I2c1 {
    const BASE: u32 = 0x4002_1000;
    const INDEX: u32 = 1;
    fn configure_pins() {
        i2c_pin(0x4000_4000, 0, 6, false); // PA6 SCL
        i2c_pin(0x4000_4000, 0, 7, true); // PA7 SDA
    }
}

/// I2C2: SCL = PE4, SDA = PE5.
pub struct I2c2;
impl I2cInstance for I2c2 {
    const BASE: u32 = 0x4002_2000;
    const INDEX: u32 = 2;
    fn configure_pins() {
        i2c_pin(0x4002_4000, 4, 4, false); // PE4 SCL
        i2c_pin(0x4002_4000, 4, 5, true); // PE5 SDA
    }
}

/// I2C3: SCL = PD0, SDA = PD1.
pub struct I2c3;
impl I2cInstance for I2c3 {
    const BASE: u32 = 0x4002_3000;
    const INDEX: u32 = 3;
    fn configure_pins() {
        i2c_pin(0x4000_7000, 3, 0, false); // PD0 SCL
        i2c_pin(0x4000_7000, 3, 1, true); // PD1 SDA
    }
}

/// I²C error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    /// ERROR set with neither ADRACK nor DATACK (bus error, clock timeout).
    Bus,
    /// Arbitration lost.
    ArbitrationLost,
    /// The slave did not acknowledge its address.
    NackAddress,
    /// The slave did not acknowledge a data byte.
    NackData,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
        match self {
            I2cError::Bus => ErrorKind::Bus,
            I2cError::ArbitrationLost => ErrorKind::ArbitrationLoss,
            I2cError::NackAddress => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            I2cError::NackData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
        }
    }
}

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

            // SCL = sysclk / (2 * (1 + TPR) * (SCL_LP + SCL_HP)); SCL_LP = 6,
            // SCL_HP = 4. TPR is a 7-bit field; clamp rather than underflow.
            let denom = 20u64 * freq_hz.max(1) as u64;
            let tpr = (sysclk as u64 / denom).saturating_sub(1).min(0x7F) as u32;
            crate::write_reg(base + I2CMTPR, tpr);
        }

        Self { _i: PhantomData }
    }

    /// Wait for the current operation to finish and check its status.
    ///
    /// The status bits are only valid once BUSY clears, so the completing
    /// status word is captured from the same read that observes BUSY = 0.
    fn wait_done(&mut self) -> Result<(), I2cError> {
        let base = I::BASE;
        let mcs = loop {
            let v = unsafe { crate::read_reg(base + I2CMCS) };
            if v & BUSY == 0 {
                break v;
            }
            core::hint::spin_loop();
        };
        if mcs & ARBLST != 0 {
            // ERROR is set alongside ARBLST; check it first because the bus
            // now belongs to another master and must not be touched.
            return Err(I2cError::ArbitrationLost);
        }
        if mcs & ERROR != 0 {
            // Release the bus. After an ADRACK/DATACK error the controller
            // holds the bus until an explicit STOP is generated (TivaWare
            // I2C_MASTER_CMD_BURST_SEND_ERROR_STOP writes STOP alone).
            unsafe { crate::write_reg(base + I2CMCS, STOP) };
            return Err(if mcs & ADRACK != 0 {
                I2cError::NackAddress
            } else if mcs & DATACK != 0 {
                I2cError::NackData
            } else {
                I2cError::Bus
            });
        }
        Ok(())
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
        self.wait_done()
    }

    fn recv_byte(&mut self, addr: u8, start: bool, stop: bool, ack: bool) -> Result<u8, I2cError> {
        let base = I::BASE;
        unsafe {
            crate::write_reg(base + I2CMSA, ((addr as u32 & 0x7F) << 1) | 1);
            let mut cs = RUN;
            if start {
                cs |= START;
            }
            if ack {
                cs |= ACK;
            }
            if stop {
                cs |= STOP;
            }
            crate::write_reg(base + I2CMCS, cs);
        }
        self.wait_done()?;
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

        // A zero-length transaction is an address probe: send the address
        // with no data and let ADRACK tell whether a device is present.
        if total == 0 {
            unsafe {
                crate::write_reg(I::BASE + I2CMSA, (addr as u32 & 0x7F) << 1);
                crate::write_reg(I::BASE + I2CMCS, RUN | START | STOP);
            }
            return self.wait_done();
        }

        let mut idx = 0usize;
        let mut prev_read = false;
        let mut first = true;

        for i in 0..operations.len() {
            // Whether the last byte of this operation is followed by more
            // read bytes (empty operations are skipped): that byte must be
            // ACKed only then, and NACKed before a repeated START or STOP.
            let next_is_read = operations[i + 1..]
                .iter()
                .find(|o| match o {
                    Operation::Read(b) => !b.is_empty(),
                    Operation::Write(b) => !b.is_empty(),
                })
                .is_some_and(|o| matches!(o, Operation::Read(_)));

            match &mut operations[i] {
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
                    let len = data.len();
                    for (j, b) in data.iter_mut().enumerate() {
                        let last = idx == total - 1;
                        let start = first || !prev_read;
                        let ack = j != len - 1 || next_is_read;
                        *b = self.recv_byte(addr, start, last, ack)?;
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
