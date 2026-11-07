//! COBS serial <-> UDP bridge for the LM4F120.
//!
//! Reads COBS-framed messages from a serial port and forwards their decoded
//! payloads as UDP datagrams, and vice versa. The UDP socket binds to an
//! ephemeral (random) port; the chosen port is printed on startup. Each
//! forwarded packet is logged in hex.
//!
//! Logging goes through the `log` facade with a small custom backend that
//! prints a timestamp, level, file and line number for every line. Set
//! `RUST_LOG` (e.g. `RUST_LOG=debug`) to change the verbosity.
//!
//! Two directions:
//! - UDP -> serial: each datagram is COBS-encoded, framed with a `0x00`
//!   delimiter, and written to the serial port. The sender is remembered as
//!   the default reply destination.
//! - serial -> UDP: COBS frames are decoded and each payload is sent as a UDP
//!   datagram to `--udp-target` (if given) or the last UDP sender.

use std::env;
use std::fmt::Write as _;
use std::io::{Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use log::{error, info, warn, LevelFilter, Log, Metadata, Record};

const MAX_PAYLOAD: usize = 256;
const MAX_ENCODED: usize = MAX_PAYLOAD + (MAX_PAYLOAD + 253) / 254;
const MAX_FRAME: usize = MAX_ENCODED + 1;

/// Minimal `log` backend: timestamp + level + file:line + message.
struct Logger;

impl Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let ts = chrono::Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
        println!(
            "{} {:<5} {}:{}  {}",
            ts,
            record.level(),
            record.file().unwrap_or("?"),
            record.line().unwrap_or(0),
            record.args()
        );
    }

    fn flush(&self) {}
}

fn init_logging() {
    let filter = env::var("RUST_LOG")
        .ok()
        .and_then(|s| s.parse::<LevelFilter>().ok())
        .unwrap_or(LevelFilter::Info);
    log::set_max_level(filter);
    let _ = log::set_logger(&Logger);
}

fn usage() -> ! {
    eprintln!(
        "usage: lm4f120-bridge <serial-port> [--baud BPS] [--udp-target IP:PORT]"
    );
    std::process::exit(2);
}

/// Format `bytes` as space-separated lowercase hex (e.g. `01 02 03`).
fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 3);
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        let _ = write!(s, "{:02x}", b);
    }
    s
}

fn encode_frame(payload: &[u8], out: &mut [u8]) -> usize {
    if payload.is_empty() {
        return 0;
    }
    let n = cobs::encode(payload, out);
    out[n] = 0x00;
    n + 1
}

fn decode_frame(encoded: &[u8], out: &mut [u8]) -> Option<usize> {
    cobs::decode(encoded, out).ok()
}

fn main() {
    init_logging();

    let args: Vec<String> = env::args().collect();
    let mut baud = 115200u32;
    let mut udp_target: Option<SocketAddr> = None;
    let mut port: Option<String> = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--baud" => {
                i += 1;
                baud = args
                    .get(i)
                    .and_then(|s| s.parse().ok())
                    .unwrap_or_else(|| usage());
            }
            "--udp-target" => {
                i += 1;
                udp_target = args.get(i).and_then(|s| s.parse().ok());
                if udp_target.is_none() {
                    error!("invalid --udp-target");
                    std::process::exit(2);
                }
            }
            s if s.starts_with("--") => usage(),
            s => {
                if port.is_none() {
                    port = Some(s.to_string());
                } else {
                    usage();
                }
            }
        }
        i += 1;
    }

    let port = port.unwrap_or_else(|| usage());

    // Open the serial port (8N1, no flow control).
    let serial = serialport::new(&port, baud)
        .data_bits(serialport::DataBits::Eight)
        .flow_control(serialport::FlowControl::None)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .timeout(Duration::from_millis(200))
        .open()
        .unwrap_or_else(|e| {
            error!("failed to open {}: {}", port, e);
            std::process::exit(1);
        });

    // Bind UDP to an ephemeral port (0 = let the OS choose one).
    let udp = UdpSocket::bind("0.0.0.0:0").unwrap_or_else(|e| {
        error!("failed to bind UDP socket: {}", e);
        std::process::exit(1);
    });
    let local_addr = udp.local_addr().unwrap();
    info!("serial: {} @ {} baud", port, baud);
    info!("udp: listening on {}", local_addr);
    if let Some(t) = udp_target {
        info!("udp: serial->udp forwarded to {} (fixed)", t);
    } else {
        info!("udp: serial->udp forwarded to last UDP sender");
    }

    // Independent reader and writer halves for the serial port, plus a clone
    // of the UDP socket for the receive thread.
    let mut rx_serial = serial.try_clone().unwrap();
    let mut tx_serial = serial.try_clone().unwrap();
    drop(serial);
    let udp_recv = udp.try_clone().unwrap();

    // Last UDP sender, used as the serial->udp destination when no --udp-target.
    let last_peer: Arc<Mutex<Option<SocketAddr>>> = Arc::new(Mutex::new(None));

    // Thread: UDP -> serial.
    let peer_tx = Arc::clone(&last_peer);
    let udp_handle = std::thread::spawn(move || {
        let mut rx = [0u8; MAX_PAYLOAD];
        let mut frame = [0u8; MAX_FRAME];
        loop {
            let (n, peer) = match udp_recv.recv_from(&mut rx) {
                Ok(v) => v,
                Err(e) => {
                    error!("udp recv error: {}", e);
                    continue;
                }
            };
            *peer_tx.lock().unwrap() = Some(peer);
            let n = n.min(MAX_PAYLOAD);
            info!("udp->serial {:>3}B {}  from {}", n, hex(&rx[..n]), peer);
            let m = encode_frame(&rx[..n], &mut frame);
            if tx_serial.write_all(&frame[..m]).is_err() {
                error!("serial write error");
                break;
            }
        }
    });

    // Main thread: serial -> UDP.
    let mut raw = [0u8; MAX_ENCODED];
    let mut raw_len = 0usize;
    let mut decoded = [0u8; MAX_PAYLOAD];
    let mut byte = [0u8; 1];
    loop {
        match rx_serial.read(&mut byte) {
            Ok(0) => continue, // timeout
            Ok(_) => {
                let b = byte[0];
                if b == 0x00 {
                    if raw_len > 0 {
                        match decode_frame(&raw[..raw_len], &mut decoded) {
                            Some(n) => {
                                if n > 1 && decoded[0] == 0x03 {
                                    info!("cbor: {}", minicbor::display(&decoded[1..n]));
                                }
                                match udp_target.or(*last_peer.lock().unwrap()) {
                                    Some(d) => {
                                        info!(
                                            "serial->udp {:>3}B {}  to {}",
                                            n,
                                            hex(&decoded[..n]),
                                            d
                                        );
                                        let _ = udp.send_to(&decoded[..n], d);
                                    }
                                    None => {
                                        info!(
                                            "serial->udp {:>3}B {}  (dropped: no udp peer yet)",
                                            n,
                                            hex(&decoded[..n])
                                        );
                                    }
                                }
                            }
                            None => {
                                warn!(
                                    "serial->udp invalid frame ({} B): {}",
                                    raw_len,
                                    hex(&raw[..raw_len])
                                );
                            }
                        }
                        raw_len = 0;
                    }
                } else if raw_len < raw.len() {
                    raw[raw_len] = b;
                    raw_len += 1;
                } else {
                    // Overflow: drop the partial frame, resync on next delimiter.
                    raw_len = 0;
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => {
                error!("serial read error: {}", e);
                break;
            }
        }
    }

    let _ = udp_handle.join();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cobs_roundtrip() {
        let payload = [1u8, 2, 0, 3, 0, 0, 4, 5];
        let mut frame = [0u8; MAX_FRAME];
        let n = encode_frame(&payload, &mut frame);
        assert_eq!(frame[n - 1], 0x00);
        let mut out = [0u8; MAX_PAYLOAD];
        let m = decode_frame(&frame[..n - 1], &mut out).unwrap();
        assert_eq!(&out[..m], &payload);
    }

    #[test]
    fn cobs_empty_and_zero() {
        // Empty payloads emit no bytes: COBS cannot distinguish an empty
        // message from "no message", so they are skipped.
        let mut frame = [0u8; MAX_FRAME];
        assert_eq!(encode_frame(&[], &mut frame), 0);

        // A non-empty payload of all zeros round-trips.
        let payload = [0u8, 0u8, 0u8];
        let n = encode_frame(&payload, &mut frame);
        assert_eq!(frame[n - 1], 0x00);
        let mut out = [0u8; MAX_PAYLOAD];
        let m = decode_frame(&frame[..n - 1], &mut out).unwrap();
        assert_eq!(&out[..m], &payload);
    }

    #[test]
    fn hex_formatting() {
        assert_eq!(hex(&[]), "");
        assert_eq!(hex(&[0x01, 0x02, 0xff]), "01 02 ff");
        assert_eq!(hex(&[0x00]), "00");
    }

    #[test]
    fn cbor_sample() {
        use minicbor::Encoder;
        let mut buf = [0u8; 64];
        let total = buf.len();
        let mut enc = Encoder::new(&mut buf[..]);
        let _ = enc.array(3);
        let _ = enc.i32(42);
        let _ = enc.f32(3.14f32);
        let _ = enc.str("hello");
        let rest = enc.into_writer();
        let n = total - rest.len();

        // CBOR array [42, 3.14, "hello"].
        assert_eq!(
            &buf[..n],
            &[
                0x83, 0x18, 0x2a, 0xfa, 0x40, 0x48, 0xf5, 0xc3, 0x65, 0x68, 0x65, 0x6c, 0x6c,
                0x6f
            ]
        );

        let s = format!("{}", minicbor::display(&buf[..n]));
        assert!(s.starts_with("[42,"), "unexpected display: {}", s);
        assert!(s.ends_with("\"hello\"]"), "unexpected display: {}", s);
    }
}
