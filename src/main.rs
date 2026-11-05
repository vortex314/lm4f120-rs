#![no_std]
#![no_main]

//! Embassy multi-task version.
//!
//! Embassy has no HAL for the LM4F120/Tiva-C family, so this uses the
//! platform-agnostic `embassy-executor` + `embassy-time` with a hand-rolled
//! SysTick time driver. The RGB LED and the UART run as two concurrent async
//! tasks; the GPIO/UART register code is unchanged.

use core::cell::{Cell, RefCell};
use core::panic::PanicInfo;
use core::task::Waker;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_time_driver::{Driver, TICK_HZ};

// ===== Register map (LM4F120H5QR) =====

const SYSCTL_BASE: u32 = 0x400F_E000;
const RCGC1: u32 = SYSCTL_BASE + 0x104;
const RCGC2: u32 = SYSCTL_BASE + 0x108;

const GPIOA_BASE: u32 = 0x4000_4000;
const GPIOA_AFSEL: u32 = GPIOA_BASE + 0x420;
const GPIOA_DEN: u32 = GPIOA_BASE + 0x51C;
const GPIOA_PCTL: u32 = GPIOA_BASE + 0x52C;

const GPIOF_BASE: u32 = 0x4002_5000;
const GPIOF_DATA: u32 = GPIOF_BASE + 0x3FC;
const GPIOF_DIR: u32 = GPIOF_BASE + 0x400;
const GPIOF_AFSEL: u32 = GPIOF_BASE + 0x420;
const GPIOF_DEN: u32 = GPIOF_BASE + 0x51C;
const GPIOF_DR8R: u32 = GPIOF_BASE + 0x508;

const UART0_BASE: u32 = 0x4000_C000;
const UART0_DR: u32 = UART0_BASE + 0x000;
const UART0_FR: u32 = UART0_BASE + 0x018;
const UART0_IBRD: u32 = UART0_BASE + 0x024;
const UART0_FBRD: u32 = UART0_BASE + 0x028;
const UART0_LCRH: u32 = UART0_BASE + 0x02C;
const UART0_CTL: u32 = UART0_BASE + 0x030;

const LED_RED: u32 = 1 << 1;
const LED_BLUE: u32 = 1 << 2;
const LED_GREEN: u32 = 1 << 3;
const LED_MASK: u32 = LED_RED | LED_BLUE | LED_GREEN;

// 16 MHz PIOSC system clock; 1 kHz tick -> SysTick reload = 15999.
const CLOCK_HZ: u32 = 16_000_000;
const SYSTICK_RELOAD: u32 = (CLOCK_HZ as u64 / TICK_HZ) as u32 - 1;

#[inline(always)]
unsafe fn wr(addr: u32, val: u32) {
    core::ptr::write_volatile(addr as *mut u32, val);
}

#[inline(always)]
unsafe fn rd(addr: u32) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

fn uart_putc(c: u8) {
    unsafe {
        while rd(UART0_FR) & (1 << 5) != 0 {}
        wr(UART0_DR, c as u32);
    }
}

fn uart_write(s: &str) {
    for &c in s.as_bytes() {
        uart_putc(c);
    }
}

fn init_peripherals() {
    unsafe {
        wr(RCGC1, 0x01);
        wr(RCGC2, (1 << 0) | (1 << 5));
        for _ in 0..64 {
            cortex_m::asm::nop();
        }

        wr(GPIOA_AFSEL, rd(GPIOA_AFSEL) | 0x03);
        wr(GPIOA_PCTL, (rd(GPIOA_PCTL) & !0xFF) | 0x11);
        wr(GPIOA_DEN, rd(GPIOA_DEN) | 0x03);

        wr(GPIOF_AFSEL, rd(GPIOF_AFSEL) & !LED_MASK);
        wr(GPIOF_DEN, rd(GPIOF_DEN) | LED_MASK);
        wr(GPIOF_DIR, rd(GPIOF_DIR) | LED_MASK);
        wr(GPIOF_DR8R, rd(GPIOF_DR8R) | LED_MASK);
        wr(GPIOF_DATA, 0);

        wr(UART0_CTL, 0);
        wr(UART0_IBRD, 8);
        wr(UART0_FBRD, 44);
        wr(UART0_LCRH, 0x70);
        wr(UART0_CTL, 0x0301);
    }
}

fn init_systick(syst: &mut SYST) {
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(SYSTICK_RELOAD);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}

// ===== SysTick time driver =====
//
// embassy-time calls `schedule_wake(deadline, task_waker)` once per pending
// timer, each with its own waker. The driver therefore keeps a small table of
// (deadline, waker) pairs and wakes every entry whose deadline has passed on
// each SysTick tick. A critical section guards the table; Cortex-M4 has no
// 64-bit atomics so the tick counter is also guarded.

const MAX_TIMERS: usize = 4;

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

// ===== Tasks =====

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    init_systick(&mut cp.SYST);
    init_peripherals();

    spawner.spawn(led_task().unwrap());
    spawner.spawn(uart_task().unwrap());
}

#[embassy_executor::task]
async fn led_task() {
    let sequence = [LED_RED, LED_GREEN, LED_BLUE];
    let mut i = 0usize;
    loop {
        unsafe { wr(GPIOF_DATA, sequence[i]); }
        Timer::after(Duration::from_millis(400)).await;
        i = (i + 1) % sequence.len();
    }
}

#[embassy_executor::task]
async fn uart_task() {
    loop {
        uart_write("hello world 2\r\n");
        Timer::after(Duration::from_millis(400)).await;
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
