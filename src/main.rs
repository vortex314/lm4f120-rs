#![no_std]
#![no_main]

//! Embassy multi-task demo using `lm4f120-hal` for GPIO and UART.
//!
//! The RGB LED is driven by an actor (`actor::LedActor`). The board's host link
//! (UART0) is owned by a COBS actor (`cobs_actor::CobsActor`) that exchanges
//! COBS-framed messages with the Linux `lm4f120-bridge`, which forwards them
//! over UDP. UART1-7 run DMA loopback demos whose latency reports are sent to
//! the host as COBS messages, alongside a periodic CBOR sample. Peripherals
//! come from `lm4f120-hal`; the hand-rolled SysTick time driver lives in the
//! `embassy` module.

use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use lm4f120_hal::embedded_io::Write;
use lm4f120_hal::setup_80mhz;
use lm4f120_hal::uart::{Uart, Uart0, Uart1, Uart2, Uart3, Uart4, Uart5, Uart6, Uart7};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use minicbor::Encoder;

mod actor;
mod cobs_actor;
mod cobs_frame;
mod embassy;

use actor::{Color, LedActor, Message};
use cobs_actor::CobsActor;
use embassy::{init_systick, read_cycles, CLOCK_HZ};

// ===== Channels =====

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

    spawner.spawn(cobs_actor_task().unwrap());
    spawner.spawn(cobs_heartbeat().unwrap());
    spawner.spawn(cbor_demo().unwrap());
    spawner.spawn(latency_reporter().unwrap());
    spawner.spawn(led_actor_task(spawner.clone()).unwrap());
    spawner.spawn(led_demo().unwrap());
    spawner.spawn(uart1_loopback().unwrap());
    spawner.spawn(uart2_loopback().unwrap());
    spawner.spawn(uart3_loopback().unwrap());
    spawner.spawn(uart4_loopback().unwrap());
    spawner.spawn(uart5_loopback().unwrap());
    spawner.spawn(uart6_loopback().unwrap());
    spawner.spawn(uart7_loopback().unwrap());
}

/// Hosts the COBS actor on UART0, the board's host-facing serial link.
#[embassy_executor::task]
async fn cobs_actor_task() {
    CobsActor::<Uart0>::new(115200, CLOCK_HZ).run().await;
}

/// Periodic heartbeat: posts a small COBS message once a second (board → host).
#[embassy_executor::task]
async fn cobs_heartbeat() {
    let mut n = 0u32;
    loop {
        let payload = [
            0x02,
            (n & 0xFF) as u8,
            ((n >> 8) & 0xFF) as u8,
            ((n >> 16) & 0xFF) as u8,
            ((n >> 24) & 0xFF) as u8,
        ];
        cobs_actor::send(&payload);
        n = n.wrapping_add(1);
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Encode a CBOR array `[42, 3.14, "hello"]` into `buf` and return its length.
fn encode_cbor_sample(buf: &mut [u8]) -> usize {
    let total = buf.len();
    let mut enc = Encoder::new(&mut buf[..]);
    let _ = enc.array(3);
    let _ = enc.i32(42);
    let _ = enc.f32(3.14f32);
    let _ = enc.str("hello");
    let rest = enc.into_writer();
    total - rest.len()
}

/// Periodically sends the CBOR sample message to the host (board → bridge).
#[embassy_executor::task]
async fn cbor_demo() {
    loop {
        let mut payload = [0u8; 64];
        payload[0] = 0x03; // message type: CBOR
        let n = encode_cbor_sample(&mut payload[1..]);
        cobs_actor::send(&payload[..1 + n]);
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// Forwards UART loopback latency reports to the host as COBS messages.
#[embassy_executor::task]
async fn latency_reporter() {
    loop {
        let r = LATENCY.receive().await;
        let payload = [
            0x01,
            r.uart,
            (r.us & 0xFF) as u8,
            ((r.us >> 8) & 0xFF) as u8,
            ((r.us >> 16) & 0xFF) as u8,
            ((r.us >> 24) & 0xFF) as u8,
        ];
        cobs_actor::send(&payload);
    }
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
