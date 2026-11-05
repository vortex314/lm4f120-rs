# Embassy on the LM4F120

This document explains how the [Embassy] async runtime is integrated onto the
TI Stellaris LM4F120H5QR (a Cortex-M4F "Tiva-C class" part). Embassy has no HAL
for this family, so the integration uses the platform-agnostic `embassy-executor`
and `embassy-time` crates with a **hand-rolled SysTick time driver**, while
GPIO and UART come from `lm4f120-hal`.

## Why this shape?

Embassy's own HALs (stm32, nrf, rp, …) each provide a hardware-timer-backed
time driver. There is no `embassy-lm4f120`, so we supply the missing piece
ourselves — a `Driver` implementation — and everything else (executor, timers,
channels, tasks) is stock Embassy.

## Dependencies

```toml
embassy-executor = { version = "0.10", features = ["platform-cortex-m", "executor-thread"] }
embassy-time = { version = "0.5", features = ["tick-hz-1_000"] }
embassy-time-driver = "0.2"
embassy-sync = "0.8"

cortex-m = { version = "0.7", features = ["critical-section-single-core"] }
critical-section = "1"
```

- `platform-cortex-m` — selects the Cortex-M platform glue (the `sev`/`wfe`
  pender and the `#[embassy_executor::main]` entry macro).
- `executor-thread` — the single-core thread-mode executor (`poll(); wfe();`).
- `tick-hz-1_000` — 1 kHz time base (1 ms resolution). Chosen because a 1 MHz
  tick would fire a SysTick interrupt every 80 cycles at 80 MHz — faster than
  the handler itself.
- `critical-section-single-core` — provides the `critical-section` implementation
  (PRIMASK-based) that both Embassy internals and the time driver need.

## Architecture

```
            ┌────────────────────────────────────────────────┐
            │                embassy-executor                │
            │   #[embassy_executor::main] -> Executor::run() │
            │         loop { poll(); wfe(); }                │
            └───────────────┬────────────────────────────────┘
                            │ spawn / poll futures
        ┌───────────────────┼────────────────────┐
        │                   │                    │
   ┌────▼─────┐      ┌──────▼──────┐      ┌──────▼──────┐
   │ led_task │      │ requester/  │      │ uart1..7    │
   │ (GPIO)   │      │ responder   │      │ loopback    │
   └──────────┘      │ (channels)  │      │ (µDMA)      │
                     └─────────────┘      └─────────────┘
                             │
                    embassy-time (Timer)
                             │ schedule_wake(at, waker)
                     ┌───────▼────────┐
                     │ SysTickDriver  │  <- the only custom part
                     │  (hand-rolled) │
                     └───────┬────────┘
                             │ SysTick IRQ (1 kHz)
                     ┌───────▼────────┐
                     │  Cortex-M4F    │
                     └────────────────┘
```

## The time driver

`embassy-time-driver` 0.2 exposes a minimal trait:

```rust
pub trait Driver: Send + Sync + 'static {
    fn now(&self) -> u64;
    fn schedule_wake(&self, at: u64, waker: &Waker);
}
```

embassy-time-driver also declares two symbols it expects the application to
provide, and resolves them through `extern "Rust"`:

```rust
#[no_mangle]
fn _embassy_time_now() -> u64;

#[no_mangle]
fn _embassy_time_schedule_wake(at: u64, waker: &Waker);
```

Our driver forwards these to a `SysTickDriver`:

```rust
static DRIVER: SysTickDriver = SysTickDriver::new();

#[no_mangle]
fn _embassy_time_now() -> u64 {
    <SysTickDriver as Driver>::now(&DRIVER)
}

#[no_mangle]
fn _embassy_time_schedule_wake(at: u64, waker: &Waker) {
    <SysTickDriver as Driver>::schedule_wake(&DRIVER, at, waker)
}
```

### Tick counter

`now()` returns a 64-bit elapsed-tick count. The SysTick is configured for a
1 kHz tick, so `SYSTICK_RELOAD = 80_000_000 / 1000 - 1 = 79999`.

The counter is extended to 64 bits in software. Cortex-M4 has no 64-bit
atomics, so it is guarded by a critical section instead:

```rust
static TICKS: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
```

### The alarm table (the important part)

`schedule_wake(at, waker)` is called **once per pending timer, each with that
timer's own waker** — not once for "the next deadline". A driver that stores a
single (waker, deadline) slot therefore silently drops all but the most recent
timer. This bit us: two tasks awaiting `Timer::after` with the same deadline
ended up with one timer lost.

The fix is a small multi-slot table:

```rust
const MAX_TIMERS: usize = 16; // >= number of concurrently-pending timers

struct Alarm {
    at: u64,
    waker: Option<Waker>,
}

static ALARMS: Mutex<RefCell<[Alarm; MAX_TIMERS]>> = Mutex::new(RefCell::new([/* … */]));
```

`schedule_wake`:

1. If a slot already holds an equivalent waker (`Waker::will_wake`), move its
   deadline — this handles re-armed/periodic timers.
2. Otherwise take a free slot and store `(at, waker.clone())`.
3. If the table is full, replace the farthest-out deadline (bounded fallback).

### SysTick interrupt handler

```rust
#[cortex_m_rt::exception]
unsafe fn SysTick() {
    let due: [Option<Waker>; MAX_TIMERS] = critical_section::with(|cs| {
        let t = TICKS.borrow(cs).get() + 1;
        TICKS.borrow(cs).set(t);
        // Collect every alarm whose deadline has passed.
        let mut due = core::array::from_fn(|_| None);
        for a in alarms { if a.waker.is_some() && t >= a.at { due[i] = a.waker.take(); } }
        due
    });
    for w in due.into_iter().flatten() {
        w.wake(); // wake OUTSIDE the critical section
    }
}
```

Two details matter:

- Wakers are **collected inside** the critical section but **woken outside** it.
  `Waker::wake` can re-enter Embassy (which may take its own critical section),
  so waking under our critical section would deadlock.
- `due` is a fixed-size array (`core::array::from_fn`), so the handler is
  allocation-free.

## Executor and tasks

`#[embassy_executor::main]` replaces `#[cortex_m_rt::entry]` and generates the
thread-mode executor:

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    setup_80mhz();                 // PLL -> 80 MHz, must run before SysTick init
    lm4f120_hal::dma::init();

    let mut cp = cortex_m::Peripherals::take().unwrap();
    init_systick(&mut cp.SYST);    // 1 kHz tick -> the time driver above

    spawner.spawn(led_task().unwrap());
    spawner.spawn(requester().unwrap());
    spawner.spawn(responder().unwrap());
    // … one loopback task per UART …
}
```

Ordering is deliberate: the clock must be at 80 MHz *before* SysTick is
configured, because `SYSTICK_RELOAD` is computed from `CLOCK_HZ`.

In Embassy 0.10, `#[embassy_executor::task]` functions return
`Result<SpawnToken, SpawnError>` (the arena allocation is fallible), and
`Spawner::spawn` takes the token and returns `()`:

```rust
spawner.spawn(led_task().unwrap());
```

## Channels

Tasks communicate through `embassy-sync` channels, which need a raw-mutex type.
The PRIMASK-based critical-section mutex is used:

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

static REQUEST: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();
static REPLY:   Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();

// requester
REQUEST.send(n).await;
let _reply = REPLY.receive().await;

// responder
let req = REQUEST.receive().await;
REPLY.send(req.wrapping_add(1)).await;
```

`send`/`receive` are async (they yield when blocked); `try_receive` is the
non-blocking form used to drain a report queue without blocking.

## Pitfalls found while integrating

- **Single-waker timer bug** — the largest one; see the alarm-table section.
  It only manifests with ≥ 2 tasks awaiting timers concurrently.
- **No 64-bit atomics on Cortex-M4** — `AtomicU64` does not exist for
  `thumbv7em-none-eabihf`; the tick counter uses `critical_section` + `Cell`.
- **Tick rate vs. CPU speed** — 1 MHz tick is impossible at 80 MHz (interrupt
  every 80 cycles). 1 kHz is the pragmatic choice here.
- **`schedule_wake` re-entrancy** — wake wakers outside the critical section.
- **Table sizing** — `MAX_TIMERS` must cover the worst-case number of
  concurrently-pending timers (LED + all loopback tasks here), or the
  farthest-out timer gets evicted and lost.
- **`#[no_mangle]` symbol ABI** — the exported symbols must match the exact
  names and signatures embassy-time-driver declares (`extern "Rust"`).
- **Clock before SysTick** — `setup_80mhz()` must precede `init_systick()`,
  otherwise the time base is computed for the wrong clock.

## Limitations & pitfalls for users

Things to know when you build on top of this code — distinct from the
integration bugs above, these are intentional design constraints and untested
paths.

### HAL drivers

- **Blocking, polled I/O.** `Uart::read` busy-waits for a byte; if no data ever
  arrives it blocks forever. `Uart::write` busy-waits on the TX FIFO. SPI and
  I²C likewise poll. Inside an Embassy task, a blocking call with no data
  starves every other task (see the executor notes below).
- **No interrupt-driven or async I/O in the HAL itself.** The only background
  mechanism is µDMA, and even its completion is polled (`dma_rx_done`/
  `dma_tx_done`). There is no Embassy HAL — async I/O requires you to add it.
- **No "taken" guard.** `Uart`, `Spi`, `I2c` are zero-sized types with no
  singleton check. Creating two handles for the same peripheral is allowed and
  each `new()` re-runs the init, resetting the peripheral. It's on you to keep
  one handle per peripheral.
- **SPI and I²C are untested on hardware** — they compile and implement the
  traits correctly, but have only been exercised in a build, not against real
  devices.
- **I²C timing is approximate.** The `MTPR` calculation uses fixed
  SCL low/high periods, so the exact SCL frequency isn't guaranteed (functional
  for standard-mode devices, but don't rely on a precise clock).
- **SPI is 8-bit only.** SSI data size is fixed to 8 bits; there's no 16-bit
  word path exposed.
- **DMA buffer lifetime.** `dma_read`/`dma_write` read/write the buffer
  asynchronously. The buffer must stay valid and untouched until the matching
  `dma_*_done` returns true. Transfer size is capped at 1024 bytes (the control
  word's 10-bit size field).
- **DMA is a global singleton.** One shared control table, and per-UART channel
  assignments are hardcoded (UART2–7 use secondary `CHASGN` mappings). If your
  app re-maps µDMA channels, it can conflict with the UART drivers.

### Clock & setup

- **`setup_80mhz()` assumes a 16 MHz crystal** (the LaunchPad has one) and
  **busy-waits forever** if the PLL never locks (e.g., crystal absent/failed).
- **There is no runtime clock detection.** Every driver takes `sysclk` as a
  parameter; pass the wrong value and baud/SPI/I²C rates are silently wrong.
- **Order matters:** `setup_80mhz()` must run before `init_systick()` and before
  any peripheral is configured with a timing derived from the clock.

### Embassy executor

- **Cooperative single-core.** A task that fails to yield (a tight loop, or a
  blocking `Uart::read` with no data) blocks all other tasks. Keep busy
  sections short, use timeouts, or `.await` periodically.
- **Fixed `MAX_TIMERS` (16).** More than that many concurrently-awaiting
  `Timer` futures evicts the farthest-out deadline — a *silent* lost wakeup, not
  a crash. Size it to your worst case.
- **1 ms time base.** `Timer`/`Duration` resolve to 1 ms (the `tick-hz-1_000`
  choice). Sub-millisecond measurement uses the `read_cycles()` SysTick-counter
  trick in `main.rs` instead.
- **No allocator.** Task arenas, channels, and the alarm table are fixed-size
  statics. A full channel blocks the sender (`send().await`) or drops
  (`try_send`), rather than growing.

### Demo application

- **Loopback cables are assumed** on UART1–7; a UART without a cable silently
  skips its report (the task times out).
- **UART0 is the monitor.** It's owned by the `requester` task for speed/latency
  output; repurposing it means reworking that task.
- **Fault diagnostics re-init UART0.** The panic/HardFault handlers construct a
  fresh `Uart::<Uart0>` to print, which reconfigures UART0 — if the fault
  itself is UART0-related, the diagnostic may never appear.

### Flashing

- Flashing needs access to the ICDI (`1cbe:00fd`) — root, or a udev rule.
- Boards with ICDI firmware v1.00 do not answer `lm4flash` or OpenOCD's
  `ti-icdi` (they hang during the debug handshake). Update the ICDI firmware or
  use a board with newer ICDI firmware.

## References

- [embassy-rs/embassy](https://github.com/embassy-rs/embassy)
- `embassy-time-driver` `Driver` trait and `time_driver_impl!` macro
- `embassy-executor` platform layer (`platform-cortex-m`, `executor-thread`)

[Embassy]: https://embassy.dev
