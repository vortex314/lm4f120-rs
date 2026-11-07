//! Hand-rolled SysTick time driver for `embassy-time`, plus cycle-accurate
//! timing helpers.
//!
//! `lm4f120-hal` provides no timer, so this module drives Embassy's time
//! source with a 1 kHz SysTick running off the 80 MHz system clock. It also
//! exposes [`read_cycles`] for the cycle-accurate elapsed-time measurements
//! used by the UART loopback tasks in `main`.

use core::cell::{Cell, RefCell};
use core::task::Waker;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use critical_section::Mutex;
use embassy_time_driver::{Driver, TICK_HZ};

/// System clock frequency after `setup_80mhz()` (16 MHz crystal → PLL → ÷5).
pub const CLOCK_HZ: u32 = 80_000_000;

const SYSTICK_RELOAD: u32 = (CLOCK_HZ as u64 / TICK_HZ) as u32 - 1;

/// Arms SysTick as the Embassy time source: core clock, 1 kHz, interrupt on.
pub fn init_systick(syst: &mut SYST) {
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(SYSTICK_RELOAD);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}

// ===== SysTick time driver =====

const MAX_TIMERS: usize = 16;

struct Alarm {
    at: u64,
    waker: Option<Waker>,
}

impl Alarm {
    const fn empty() -> Self {
        Self { at: 0, waker: None }
    }
}

static TICKS: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
static ALARMS: Mutex<RefCell<[Alarm; MAX_TIMERS]>> = Mutex::new(RefCell::new([
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
    Alarm::empty(),
]));

struct SysTickDriver;

impl SysTickDriver {
    const fn new() -> Self {
        Self
    }
}

static DRIVER: SysTickDriver = SysTickDriver::new();

impl Driver for SysTickDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| TICKS.borrow(cs).get())
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut alarms = ALARMS.borrow(cs).borrow_mut();

            // If this waker is already scheduled, just move its deadline.
            for a in alarms.iter_mut() {
                if let Some(w) = &a.waker {
                    if w.will_wake(waker) {
                        a.at = at;
                        return;
                    }
                }
            }

            // Otherwise, add to a free slot.
            for a in alarms.iter_mut() {
                if a.waker.is_none() {
                    a.at = at;
                    a.waker = Some(waker.clone());
                    return;
                }
            }

            // Table full: replace the farthest-out deadline.
            let mut far = 0usize;
            let mut far_at = alarms[0].at;
            for (i, a) in alarms.iter().enumerate() {
                if a.at > far_at {
                    far = i;
                    far_at = a.at;
                }
            }
            alarms[far].at = at;
            alarms[far].waker = Some(waker.clone());
        });
    }
}

#[no_mangle]
fn _embassy_time_now() -> u64 {
    <SysTickDriver as Driver>::now(&DRIVER)
}

#[no_mangle]
fn _embassy_time_schedule_wake(at: u64, waker: &Waker) {
    <SysTickDriver as Driver>::schedule_wake(&DRIVER, at, waker)
}

#[cortex_m_rt::exception]
unsafe fn SysTick() {
    let due: [Option<Waker>; MAX_TIMERS] = critical_section::with(|cs| {
        let t = TICKS.borrow(cs).get() + 1;
        TICKS.borrow(cs).set(t);
        let mut alarms = ALARMS.borrow(cs).borrow_mut();
        let mut due = core::array::from_fn(|_| None);
        let mut n = 0;
        for a in alarms.iter_mut() {
            if a.waker.is_some() && t >= a.at {
                due[n] = a.waker.take();
                n += 1;
                if n == MAX_TIMERS {
                    break;
                }
            }
        }
        due
    });
    for w in due.into_iter().flatten() {
        w.wake();
    }
}

/// Cycle-accurate elapsed time since boot, derived from the SysTick counter
/// and the tick counter maintained by the time driver (80 MHz resolution).
const SYST_CVR: u32 = 0xE000E018;

pub fn read_cycles() -> u64 {
    critical_section::with(|cs| {
        let ticks = TICKS.borrow(cs).get();
        let cvr = unsafe { core::ptr::read_volatile(SYST_CVR as *const u32) } & 0xFFFFFF;
        let period = (SYSTICK_RELOAD + 1) as u64;
        ticks * period + (period - 1 - cvr as u64)
    })
}
