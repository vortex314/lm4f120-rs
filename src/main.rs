#![no_std]
#![no_main]

//! Embassy multi-task demo using `lm4f120-hal` for GPIO and UART.
//!
//! The RGB LED (led_task) and UART0 (uart_task) run as two concurrent async
//! tasks on the Embassy executor. Peripherals come from `lm4f120-hal`; the
//! SysTick time driver is still hand-rolled because the HAL has no timer.

use core::cell::{Cell, RefCell};
use core::panic::PanicInfo;
use core::task::Waker;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embassy_time_driver::{Driver, TICK_HZ};

use lm4f120_hal::embedded_io::Write;
use lm4f120_hal::gpio::{Output, Pin, PortF};
use lm4f120_hal::setup_80mhz;
use lm4f120_hal::uart::{Uart, Uart0};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

const CLOCK_HZ: u32 = 80_000_000;
const SYSTICK_RELOAD: u32 = (CLOCK_HZ as u64 / TICK_HZ) as u32 - 1;

fn init_systick(syst: &mut SYST) {
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(SYSTICK_RELOAD);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}

// ===== SysTick time driver =====

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

// ===== Channels =====

static REQUEST: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();
static REPLY: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();

// ===== Tasks =====

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    setup_80mhz();

    let mut cp = cortex_m::Peripherals::take().unwrap();
    init_systick(&mut cp.SYST);

    spawner.spawn(led_task().unwrap());
    spawner.spawn(requester().unwrap());
    spawner.spawn(responder().unwrap());
}

#[embassy_executor::task]
async fn led_task() {
    let mut red = Pin::<PortF, Output, 1>::new();
    let mut blue = Pin::<PortF, Output, 2>::new();
    let mut green = Pin::<PortF, Output, 3>::new();
    let mut i = 0u8;

    loop {
        red.set_low();
        blue.set_low();
        green.set_low();
        match i % 3 {
            0 => red.set_high(),
            1 => green.set_high(),
            _ => blue.set_high(),
        }
        i = i.wrapping_add(1);
        Timer::after(Duration::from_millis(400)).await;
    }
}

/// Writes `n` as a decimal string over the UART.
fn write_u32(uart: &mut Uart<Uart0>, mut n: u32) {
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if n == 0 {
            break;
        }
    }
    let _ = uart.write(&buf[i..]);
}

/// Sends a request over the channel, waits for the reply, and reports the
/// round-trip rate once per second.
#[embassy_executor::task]
async fn requester() {
    let mut uart = Uart::<Uart0>::new(115200, CLOCK_HZ);
    let _ = uart.write(b"channel ping-pong ready\r\n");
    let mut n = 0u32;

    loop {
        let mut count = 0u32;
        let deadline = Instant::now() + Duration::from_secs(1);
        while Instant::now() < deadline {
            REQUEST.send(n).await;
            let _reply = REPLY.receive().await;
            count += 1;
            n = n.wrapping_add(1);
        }

        let _ = uart.write(b"speed: ");
        write_u32(&mut uart, count);
        let _ = uart.write(b" round-trips/s\r\n");
    }
}

/// Receives requests and echoes back a reply.
#[embassy_executor::task]
async fn responder() {
    loop {
        let req = REQUEST.receive().await;
        REPLY.send(req.wrapping_add(1)).await;
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
