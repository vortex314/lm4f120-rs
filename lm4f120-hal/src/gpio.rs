//! Digital GPIO with typestate pins.
//!
//! A pin is `Pin<PORT, MODE, N>` where `PORT` is one of [`PortA`]…[`PortH`],
//! `MODE` is [`Input`] or [`Output`], and `N` is the pin number (0..=7).

use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

use crate::sysctl;

// Register offsets, relative to a port base address.
const GPIODIR: u32 = 0x400;
const GPIOAFSEL: u32 = 0x420;
const GPIODR2R: u32 = 0x500;
const GPIODR8R: u32 = 0x508;
const GPIOPUR: u32 = 0x510;
const GPIOPDR: u32 = 0x514;
const GPIODEN: u32 = 0x51C;
const GPIOPCTL: u32 = 0x52C;

/// A GPIO port (one of the port marker types).
pub trait Port {
    /// Base address of the port's register block.
    const BASE: u32;
    /// Port index used for clock gating (A=0 .. H=7).
    const INDEX: u32;
}

macro_rules! ports {
    ($($name:ident = $base:expr, $index:expr;)+) => {
        $(
            #[doc = concat!("GPIO port marker for ", stringify!($name), ".")]
            pub struct $name;
            impl Port for $name {
                const BASE: u32 = $base;
                const INDEX: u32 = $index;
            }
        )+
    };
}

ports! {
    PortA = 0x4000_4000, 0;
    PortB = 0x4000_5000, 1;
    PortC = 0x4000_6000, 2;
    PortD = 0x4000_7000, 3;
    PortE = 0x4002_4000, 4;
    PortF = 0x4002_5000, 5;
    PortG = 0x4002_6000, 6;
    PortH = 0x4002_7000, 7;
}

/// Resistor configuration for an input pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Pull {
    /// No pull resistor.
    None,
    /// Pull-up resistor.
    Up,
    /// Pull-down resistor.
    Down,
}

/// Pin mode marker: digital input.
pub struct Input;
/// Pin mode marker: digital output.
pub struct Output;

/// A single GPIO pin in a specific mode.
pub struct Pin<P: Port, MODE, const N: u8> {
    _p: PhantomData<(P, MODE)>,
}

/// GPIODATA is aliased to 256 locations; address bits [9:2] form the pin mask.
/// Writing to `BASE + (1 << (N + 2))` addresses pin `N` alone.
#[inline(always)]
fn data_addr<P: Port, const N: u8>() -> u32 {
    P::BASE + (1u32 << (N as u32 + 2))
}

fn init_port<P: Port>() {
    sysctl::enable_gpio(P::INDEX);
}

unsafe fn set_afsel<P: Port, const N: u8>(enabled: bool) {
    let reg = P::BASE + GPIOAFSEL;
    let v = crate::read_reg(reg);
    crate::write_reg(reg, if enabled { v | (1 << N) } else { v & !(1 << N) });
}

unsafe fn set_den<P: Port, const N: u8>(enabled: bool) {
    let reg = P::BASE + GPIODEN;
    let v = crate::read_reg(reg);
    crate::write_reg(reg, if enabled { v | (1 << N) } else { v & !(1 << N) });
}

unsafe fn set_dir<P: Port, const N: u8>(output: bool) {
    let reg = P::BASE + GPIODIR;
    let v = crate::read_reg(reg);
    crate::write_reg(reg, if output { v | (1 << N) } else { v & !(1 << N) });
}

unsafe fn set_pur<P: Port, const N: u8>(enabled: bool) {
    let reg = P::BASE + GPIOPUR;
    let v = crate::read_reg(reg);
    crate::write_reg(reg, if enabled { v | (1 << N) } else { v & !(1 << N) });
}

unsafe fn set_pdr<P: Port, const N: u8>(enabled: bool) {
    let reg = P::BASE + GPIOPDR;
    let v = crate::read_reg(reg);
    crate::write_reg(reg, if enabled { v | (1 << N) } else { v & !(1 << N) });
}

unsafe fn configure_output<P: Port, const N: u8>() {
    set_afsel::<P, N>(false);
    set_den::<P, N>(true);
    set_dir::<P, N>(true);
    // 8 mA drive strength (the on-board RGB LED expects a stiff drive).
    let reg = P::BASE + GPIODR8R;
    crate::write_reg(reg, crate::read_reg(reg) | (1 << N));
}

unsafe fn configure_input<P: Port, const N: u8>(pull: Pull) {
    set_afsel::<P, N>(false);
    set_den::<P, N>(true);
    set_dir::<P, N>(false);
    set_pur::<P, N>(pull == Pull::Up);
    set_pdr::<P, N>(pull == Pull::Down);
}

impl<P: Port, const N: u8> Pin<P, Output, N> {
    /// Configure the pin as a push-pull digital output and return it.
    pub fn new() -> Self {
        init_port::<P>();
        unsafe { configure_output::<P, N>() };
        Self { _p: PhantomData }
    }

    /// Reconfigure this pin as an input with the given pull resistor.
    pub fn into_input(self, pull: Pull) -> Pin<P, Input, N> {
        unsafe { configure_input::<P, N>(pull) };
        Pin { _p: PhantomData }
    }

    /// Drive the pin high.
    pub fn set_high(&mut self) {
        unsafe { crate::write_reg(data_addr::<P, N>(), 0xFF) };
    }

    /// Drive the pin low.
    pub fn set_low(&mut self) {
        unsafe { crate::write_reg(data_addr::<P, N>(), 0x00) };
    }

    /// Toggle the pin.
    pub fn toggle(&mut self) {
        let now_high = self.is_set_high();
        if now_high {
            self.set_low();
        } else {
            self.set_high();
        }
    }

    /// Read back the pin state.
    pub fn is_set_high(&self) -> bool {
        unsafe { crate::read_reg(data_addr::<P, N>()) & 0x01 != 0 }
    }
}

impl<P: Port, const N: u8> Pin<P, Input, N> {
    /// Configure the pin as a digital input and return it.
    pub fn new(pull: Pull) -> Self {
        init_port::<P>();
        unsafe { configure_input::<P, N>(pull) };
        Self { _p: PhantomData }
    }

    /// Reconfigure this pin as a push-pull digital output.
    pub fn into_output(self) -> Pin<P, Output, N> {
        unsafe { configure_output::<P, N>() };
        Pin { _p: PhantomData }
    }

    /// Read the pin state.
    pub fn is_high(&self) -> bool {
        unsafe { crate::read_reg(data_addr::<P, N>()) & 0x01 != 0 }
    }

    /// Read the pin state (inverted).
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }
}

impl<P: Port, const N: u8> ErrorType for Pin<P, Output, N> {
    type Error = Infallible;
}

impl<P: Port, const N: u8> ErrorType for Pin<P, Input, N> {
    type Error = Infallible;
}

impl<P: Port, const N: u8> OutputPin for Pin<P, Output, N> {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.set_high();
        Ok(())
    }
}

impl<P: Port, const N: u8> StatefulOutputPin for Pin<P, Output, N> {
    fn is_set_high(&mut self) -> Result<bool, Infallible> {
        Ok(self.is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Infallible> {
        Ok(!self.is_set_high())
    }

    fn toggle(&mut self) -> Result<(), Infallible> {
        self.toggle();
        Ok(())
    }
}

impl<P: Port, const N: u8> InputPin for Pin<P, Input, N> {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(self.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(self.is_low())
    }
}
