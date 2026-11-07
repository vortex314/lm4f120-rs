#![no_std]
#![no_main]

//! Embassy multi-task demo using `lm4f120-hal` for GPIO and UART.
//!
//! The RGB LED is driven by an actor (`actor::LedActor`) that owns the pins
//! and services an Embassy channel mailbox; UART ping-pong and loopback tasks
//! run alongside it on the Embassy executor. Peripherals come from
//! `lm4f120-hal`; the hand-rolled SysTick time driver lives in the `embassy`
//! module.

use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};

use lm4f120_hal::embedded_io::Write;
use lm4f120_hal::setup_80mhz;
use lm4f120_hal::uart::{Uart, Uart0, Uart1, Uart2, Uart3, Uart4, Uart5, Uart6, Uart7};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

mod actor;
mod embassy;

use actor::{Color, LedActor, Message};
use embassy::{init_systick, read_cycles, CLOCK_HZ};

// ===== Channels =====

static REQUEST: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();
static REPLY: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();

/// A loopback latency measurement reported by one of the UART tasks.
struct Latency {
    uart: u8,
    us: u32,
}

static LATENCY: Channel<CriticalSectionRawMutex, Latency, 8> = Channel::new();

// ===== Tasks =====

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    setup_80mhz();
    lm4f120_hal::dma::init();

    let mut cp = cortex_m::Peripherals::take().unwrap();
    init_systick(&mut cp.SYST);

    spawner.spawn(led_actor_task(spawner.clone()).unwrap());
    spawner.spawn(led_demo().unwrap());
    spawner.spawn(requester().unwrap());
    spawner.spawn(responder().unwrap());
    spawner.spawn(uart1_loopback().unwrap());
    spawner.spawn(uart2_loopback().unwrap());
    spawner.spawn(uart3_loopback().unwrap());
    spawner.spawn(uart4_loopback().unwrap());
    spawner.spawn(uart5_loopback().unwrap());
    spawner.spawn(uart6_loopback().unwrap());
    spawner.spawn(uart7_loopback().unwrap());
}

/// Embassy task that hosts the LED actor: the actor owns the RGB pins and
/// services its mailbox from here for as long as the firmware runs.
#[embassy_executor::task]
async fn led_actor_task(spawner: Spawner) {
    LedActor::new().run(spawner).await;
}

/// Demonstration sender: drives the LED actor through its mailbox, showing the
/// two message kinds (`LedPulse` and `LedBlink`) working together.
#[embassy_executor::task]
async fn led_demo() {
    let mailbox = actor::led_mailbox();

    // Pulse each colour once, then settle into a steady green blink.
    for color in [Color::Red, Color::Green, Color::Blue] {
        mailbox
            .send(Message::LedPulse {
                duration: Duration::from_millis(250),
                color,
            })
            .await;
        Timer::after(Duration::from_millis(50)).await;
    }

    mailbox
        .send(Message::LedBlink {
            interval: Duration::from_millis(400),
            color: Color::Green,
        })
        .await;
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

        // Drain and print any UART loopback latency reports.
        while let Ok(r) = LATENCY.try_receive() {
            let _ = uart.write(b"uart");
            write_u32(&mut uart, r.uart as u32);
            let _ = uart.write(b" latency: ");
            write_u32(&mut uart, r.us);
            let _ = uart.write(b" us\r\n");
        }
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

/// Generate a loopback task for one UART: DMA-send a 64-byte buffer, DMA-receive
/// it back (external loopback cable), and report the round-trip time.
macro_rules! loopback_task {
    ($name:ident, $uart:ty, $id:expr) => {
        #[embassy_executor::task]
        async fn $name() {
            let mut uart = <$uart>::new(115200, CLOCK_HZ);
            uart.enable_dma();
            let mut tx = [0u8; 64];
            let mut rx = [0u8; 64];
            for (i, b) in tx.iter_mut().enumerate() {
                *b = i as u8;
            }
            loop {
                // Arm the receive DMA, then start the transmit DMA.
                uart.dma_read(&mut rx);
                let start = read_cycles();
                uart.dma_write(&tx);

                // Wait for the receive to complete (~20 ms timeout for UARTs
                // without a loopback cable).
                let timeout = start + 1_600_000;
                while !uart.dma_rx_done() {
                    if read_cycles() > timeout {
                        break;
                    }
                }

                if uart.dma_rx_done() {
                    let us = ((read_cycles() - start) / 80) as u32;
                    LATENCY.send(Latency { uart: $id, us }).await;
                }
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    };
}

loopback_task!(uart1_loopback, Uart<Uart1>, 1);
loopback_task!(uart2_loopback, Uart<Uart2>, 2);
loopback_task!(uart3_loopback, Uart<Uart3>, 3);
loopback_task!(uart4_loopback, Uart<Uart4>, 4);
loopback_task!(uart5_loopback, Uart<Uart5>, 5);
loopback_task!(uart6_loopback, Uart<Uart6>, 6);
loopback_task!(uart7_loopback, Uart<Uart7>, 7);

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let mut uart = Uart::<Uart0>::new(115200, CLOCK_HZ);
    let _ = uart.write(b"PANIC\r\n");
    loop {
        cortex_m::asm::nop();
    }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    let mut uart = Uart::<Uart0>::new(115200, CLOCK_HZ);
    let _ = uart.write(b"HARDFAULT\r\n");
    loop {
        cortex_m::asm::nop();
    }
}
