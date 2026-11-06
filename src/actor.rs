//! Actor model built on Embassy channels.
//!
//! Each actor owns a resource (here, the RGB LED pins) and services a mailbox
//! — an Embassy [`Channel`] — that other tasks post [`Message`]s to. Crucially,
//! the actor is *always* parked at the mailbox receive point: a handler never
//! blocks on a timer internally. Anything time-based is expressed as a timer
//! that fires back into the mailbox as a [`Message::Timer`] carrying the id of
//! the timer that expired.
//!
//! Timers are spawned as small one-shot tasks; a periodic timer is simply a
//! one-shot that the actor re-arms each time it fires. Every timer carries a
//! unique `u32` id, so the actor can tell which timer fired and silently ignore
//! stale timers left over from a superseded LED program.

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Timer};

use lm4f120_hal::gpio::{Output, Pin, PortF};

/// An RGB colour the on-board LED can display.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Color {
    Red,
    Green,
    Blue,
}

/// Messages understood by the LED actor.
///
/// This enum is the single, extensible message type of the actor system: add a
/// variant here and handle it in [`LedActor::run`] to grow the protocol.
#[derive(Clone, Copy, Debug)]
pub enum Message {
    /// Light `color` for `duration`, then turn the LED off.
    LedPulse {
        duration: Duration,
        color: Color,
    },
    /// Blink `color`, each on/off phase lasting `interval`.
    LedBlink {
        interval: Duration,
        color: Color,
    },
    /// A timer armed by the actor has fired. `id` identifies the timer.
    Timer {
        id: u32,
    },
}

/// Mailbox of the LED actor: a static channel so any task can reach the actor
/// without passing sender handles around.
static LED_MAILBOX: Channel<CriticalSectionRawMutex, Message, 8> = Channel::new();

/// A sender handle for posting messages to the LED actor.
pub fn led_mailbox() -> Sender<'static, CriticalSectionRawMutex, Message, 8> {
    LED_MAILBOX.sender()
}

/// One-shot timer: after `duration`, posts [`Message::Timer`] with `id` to the
/// actor's mailbox and exits. Re-armed by the actor for periodic behaviour.
///
/// `pool_size` must be > 1: several timers overlap (a pulse timer is still
/// pending while the next pulse arms, and a blink tick is armed while an older
/// tick has not fired). The default pool size is 1, which would make the second
/// concurrent spawn fail with `SpawnError::Busy`.
#[embassy_executor::task(pool_size = 8)]
async fn oneshot_timer(
    sender: Sender<'static, CriticalSectionRawMutex, Message, 8>,
    id: u32,
    duration: Duration,
) {
    Timer::after(duration).await;
    sender.send(Message::Timer { id }).await;
}

/// Arm a one-shot timer that posts `Timer { id }` after `duration`.
fn arm_oneshot(spawner: &Spawner, id: u32, duration: Duration) {
    let sender = LED_MAILBOX.sender();
    // `oneshot_timer` returns `Result<SpawnToken, SpawnError>`; with the pool
    // sized correctly this won't fail, but don't panic if it ever does.
    if let Ok(token) = oneshot_timer(sender, id, duration) {
        let _ = spawner.spawn(token);
    }
}

/// Blink state kept between timer ticks. The on/off phase is tracked explicitly
/// rather than read back from the pin, so the actor never depends on GPIO
/// read-back semantics.
struct Blink {
    /// Id of the currently armed tick timer.
    timer: u32,
    color: Color,
    interval: Duration,
    /// Current phase: `true` = LED lit, `false` = LED off.
    on: bool,
}

/// The LED actor: sole owner of the RGB pins, driven entirely by its mailbox.
pub struct LedActor {
    red: Pin<PortF, Output, 1>,
    blue: Pin<PortF, Output, 2>,
    green: Pin<PortF, Output, 3>,
    /// Id of the one-shot timer that will turn the LED off after a pulse.
    pulse_done: Option<u32>,
    /// Active blink program, if any.
    blink: Option<Blink>,
    /// Monotonic source of unique timer ids.
    next_id: u32,
}

impl LedActor {
    pub fn new() -> Self {
        Self {
            red: Pin::<PortF, Output, 1>::new(),
            blue: Pin::<PortF, Output, 2>::new(),
            green: Pin::<PortF, Output, 3>::new(),
            pulse_done: None,
            blink: None,
            next_id: 0,
        }
    }

    fn alloc_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1);
        id
    }

    fn all_off(&mut self) {
        self.red.set_low();
        self.blue.set_low();
        self.green.set_low();
    }

    fn set_color(&mut self, color: Color, on: bool) {
        match color {
            Color::Red => {
                if on {
                    self.red.set_high();
                } else {
                    self.red.set_low();
                }
            }
            Color::Green => {
                if on {
                    self.green.set_high();
                } else {
                    self.green.set_low();
                }
            }
            Color::Blue => {
                if on {
                    self.blue.set_high();
                } else {
                    self.blue.set_low();
                }
            }
        }
    }

    /// Run the actor loop, always parked at the mailbox receive point.
    pub async fn run(mut self, spawner: Spawner) {
        let receiver = LED_MAILBOX.receiver();

        loop {
            let msg = receiver.receive().await;

            match msg {
                Message::LedPulse { duration, color } => {
                    // A new program supersedes any pending pulse/blink timers.
                    self.blink = None;
                    self.pulse_done = None;
                    self.all_off();
                    self.set_color(color, true);

                    let id = self.alloc_id();
                    self.pulse_done = Some(id);
                    arm_oneshot(&spawner, id, duration);
                }
                Message::LedBlink { interval, color } => {
                    self.pulse_done = None;
                    self.blink = None;
                    self.all_off();
                    self.set_color(color, true);

                    let id = self.alloc_id();
                    self.blink = Some(Blink { timer: id, color, interval, on: true });
                    arm_oneshot(&spawner, id, interval);
                }
                Message::Timer { id } => {
                    if self.pulse_done == Some(id) {
                        // Pulse elapsed: turn the LED off.
                        self.pulse_done = None;
                        self.all_off();
                    } else {
                        // Is this the active blink tick timer?
                        let tick = match self.blink.as_ref() {
                            Some(b) if b.timer == id => Some((b.color, b.interval, b.on)),
                            _ => None,
                        };
                        if let Some((color, interval, on)) = tick {
                            // Flip the phase and re-arm for the next tick.
                            let on = !on;
                            self.set_color(color, on);
                            let next = self.alloc_id();
                            if let Some(b) = self.blink.as_mut() {
                                b.timer = next;
                                b.on = on;
                            }
                            arm_oneshot(&spawner, next, interval);
                        }
                        // A timer id matching neither active timer is stale
                        // (superseded by a newer program) and is ignored.
                    }
                }
            }
        }
    }
}
