//! COBS serial actor: sole owner of a UART, exchanging COBS-framed messages
//! with the host (the Linux `lm4f120-bridge`).
//!
//! The actor polls the UART for incoming bytes, accumulates them into complete
//! COBS frames, decodes them, and echoes each decoded payload back out the same
//! UART. Outgoing messages can also be posted from anywhere via [`send`]; they
//! are COBS-encoded and transmitted on the actor's next poll tick.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

use lm4f120_hal::embedded_io::{Read, ReadReady, Write};
use lm4f120_hal::uart::{Uart, UartInstance};

use crate::cobs_frame::{self, MAX_ENCODED, MAX_FRAME, MAX_PAYLOAD};

/// An outgoing COBS message posted to the actor's mailbox.
pub struct CobsMessage {
    pub data: [u8; MAX_PAYLOAD],
    pub len: usize,
}

/// Mailbox for application → actor messages (capacity 8).
static COBS_MAILBOX: Channel<CriticalSectionRawMutex, CobsMessage, 8> = Channel::new();

/// Post a payload to the COBS actor for transmission. Non-blocking: if the
/// mailbox is full the message is dropped.
pub fn send(payload: &[u8]) {
    let mut msg = CobsMessage {
        data: [0u8; MAX_PAYLOAD],
        len: 0,
    };
    let n = payload.len().min(MAX_PAYLOAD);
    msg.data[..n].copy_from_slice(&payload[..n]);
    msg.len = n;
    let _ = COBS_MAILBOX.try_send(msg);
}

/// The COBS actor: owns a UART and runs the framed serial exchange.
pub struct CobsActor<I: UartInstance> {
    uart: Uart<I>,
    /// Raw bytes of the frame currently being received (before the delimiter).
    rx_raw: [u8; MAX_ENCODED],
    rx_len: usize,
    /// Reusable transmit buffer (encoded bytes + delimiter).
    tx_buf: [u8; MAX_FRAME],
}

impl<I: UartInstance> CobsActor<I> {
    pub fn new(baud: u32, sysclk: u32) -> Self {
        Self {
            uart: Uart::<I>::new(baud, sysclk),
            rx_raw: [0u8; MAX_ENCODED],
            rx_len: 0,
            tx_buf: [0u8; MAX_FRAME],
        }
    }

    /// Run the actor loop: poll RX, echo complete frames, transmit mailbox
    /// messages, then yield for a short interval.
    pub async fn run(mut self) {
        loop {
            self.poll_rx();
            while let Ok(msg) = COBS_MAILBOX.try_receive() {
                self.write_frame(&msg.data[..msg.len]);
            }
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Drain all available RX bytes, decoding and echoing complete frames.
    fn poll_rx(&mut self) {
        let mut decoded = [0u8; MAX_PAYLOAD];
        while self.uart.read_ready().unwrap_or(false) {
            let mut byte = [0u8; 1];
            if self.uart.read(&mut byte).is_err() {
                continue;
            }
            let b = byte[0];
            if b == 0x00 {
                // Frame delimiter: decode the accumulated bytes (if any).
                if self.rx_len > 0 {
                    if let Some(n) =
                        cobs_frame::decode_frame(&self.rx_raw[..self.rx_len], &mut decoded)
                    {
                        self.write_frame(&decoded[..n]);
                    }
                    self.rx_len = 0;
                }
            } else if self.rx_len < self.rx_raw.len() {
                self.rx_raw[self.rx_len] = b;
                self.rx_len += 1;
            } else {
                // Overflow: drop the partial frame and resync on the next
                // delimiter.
                self.rx_len = 0;
            }
        }
    }

    /// COBS-encode `payload` and write it (with delimiter) to the UART.
    fn write_frame(&mut self, payload: &[u8]) {
        let n = cobs_frame::encode_frame(payload, &mut self.tx_buf);
        let _ = self.uart.write(&self.tx_buf[..n]);
    }
}
