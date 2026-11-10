//! egui manager for COBS serial <-> UDP bridges.
//!
//! Loads a YAML config listing serial ports, detects which devices are present
//! under `/dev`, and shows a table with one row per configured port. Each row
//! shows the port name, device path, USB VID:PID, status, packets
//! sent/received, the UDP port, the last decoded CBOR message, and a button to
//! start/stop the bridge. A panel at the bottom shows the last few INFO/ERROR
//! log lines.

use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::JoinHandle;
use std::time::Duration;

use eframe::egui;

const MAX_PAYLOAD: usize = 256;
const MAX_ENCODED: usize = MAX_PAYLOAD + (MAX_PAYLOAD + 253) / 254;
const MAX_FRAME: usize = MAX_ENCODED + 1;

// ===== Config =====

#[derive(Debug, Clone, serde::Deserialize)]
struct Config {
    #[serde(default)]
    ports: Vec<PortConfig>,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct PortConfig {
    name: String,
    port: String,
    #[serde(default = "default_baud")]
    baud: u32,
    /// UDP port to bind; `0` means an ephemeral port.
    #[serde(default)]
    udp_port: u16,
}

fn default_baud() -> u32 {
    115200
}

impl Config {
    fn load(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
        serde_yaml::from_str(&text).map_err(|e| e.to_string())
    }
}

// ===== COBS framing =====

fn encode_frame(payload: &[u8], out: &mut [u8]) -> usize {
    if payload.is_empty() {
        return 0;
    }
    let n = cobs::encode(payload, out);
    out[n] = 0x00;
    n + 1
}

fn decode_frame(encoded: &[u8], out: &mut [u8]) -> Option<usize> {
    cobs::decode(encoded, out).ok().map(|r| r.frame_size())
}

// ===== Log buffer =====

#[derive(Clone, Copy, PartialEq, Eq)]
enum LogLevel {
    Info,
    Error,
}

impl LogLevel {
    fn as_str(self) -> &'static str {
        match self {
            LogLevel::Info => "INFO",
            LogLevel::Error => "ERROR",
        }
    }
}

#[derive(Clone)]
struct LogLine {
    time: String,
    level: LogLevel,
    text: String,
}

/// Global ring buffer of recent log lines, shared with the bridge threads.
struct LogBuffer {
    lines: Mutex<VecDeque<LogLine>>,
}

impl LogBuffer {
    const CAPACITY: usize = 200;

    fn push(&self, level: LogLevel, text: impl Into<String>) {
        let mut lines = self.lines.lock().unwrap();
        lines.push_back(LogLine {
            time: time_hms(),
            level,
            text: text.into(),
        });
        while lines.len() > Self::CAPACITY {
            lines.pop_front();
        }
    }

    fn last(&self, n: usize) -> Vec<LogLine> {
        let lines = self.lines.lock().unwrap();
        lines.iter().rev().take(n).rev().cloned().collect()
    }
}

impl Default for LogBuffer {
    fn default() -> Self {
        Self {
            lines: Mutex::new(VecDeque::new()),
        }
    }
}

static LOG: OnceLock<LogBuffer> = OnceLock::new();

fn log() -> &'static LogBuffer {
    LOG.get_or_init(LogBuffer::default)
}

/// Current local time as `HH:MM:SS`.
fn time_hms() -> String {
    unsafe {
        let now = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&now, &mut tm);
        format!("{:02}:{:02}:{:02}", tm.tm_hour, tm.tm_min, tm.tm_sec)
    }
}

// ===== Per-port bridge =====

struct PortRuntime {
    config: PortConfig,
    detected: bool,
    vid_pid: String,
    tx: Arc<AtomicU64>,
    rx: Arc<AtomicU64>,
    bound_udp: Arc<Mutex<Option<u16>>>,
    status: Arc<Mutex<String>>,
    last_cbor: Arc<Mutex<String>>,
    shutdown: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl PortRuntime {
    fn new(config: PortConfig) -> Self {
        Self {
            config,
            detected: false,
            vid_pid: "—".to_string(),
            tx: Arc::new(AtomicU64::new(0)),
            rx: Arc::new(AtomicU64::new(0)),
            bound_udp: Arc::new(Mutex::new(None)),
            status: Arc::new(Mutex::new("stopped".to_string())),
            last_cbor: Arc::new(Mutex::new("—".to_string())),
            shutdown: Arc::new(AtomicBool::new(false)),
            handle: None,
        }
    }

    fn is_running(&self) -> bool {
        self.handle.as_ref().map_or(false, |h| !h.is_finished())
    }

    fn start(&mut self) {
        if self.is_running() {
            return;
        }
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
        self.shutdown.store(false, Ordering::Relaxed);
        let config = self.config.clone();
        let tx = Arc::clone(&self.tx);
        let rx = Arc::clone(&self.rx);
        let bound = Arc::clone(&self.bound_udp);
        let status = Arc::clone(&self.status);
        let last_cbor = Arc::clone(&self.last_cbor);
        let shutdown = Arc::clone(&self.shutdown);
        self.handle = Some(std::thread::spawn(move || {
            bridge_thread(config, tx, rx, bound, status, last_cbor, shutdown);
        }));
    }

    fn stop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }

    fn udp_text(&self) -> String {
        match *self.bound_udp.lock().unwrap() {
            Some(p) => p.to_string(),
            None => {
                if self.config.udp_port == 0 {
                    "ephemeral".to_string()
                } else {
                    self.config.udp_port.to_string()
                }
            }
        }
    }

    fn status_text(&self) -> String {
        if self.is_running() {
            "running".to_string()
        } else {
            self.status.lock().unwrap().clone()
        }
    }

    fn cbor_text(&self) -> String {
        self.last_cbor.lock().unwrap().clone()
    }
}

fn bridge_thread(
    config: PortConfig,
    tx: Arc<AtomicU64>,
    rx: Arc<AtomicU64>,
    bound_udp: Arc<Mutex<Option<u16>>>,
    status: Arc<Mutex<String>>,
    last_cbor: Arc<Mutex<String>>,
    shutdown: Arc<AtomicBool>,
) {
    let mut serial = match serialport::new(&config.port, config.baud)
        .timeout(Duration::from_millis(10))
        .open()
    {
        Ok(s) => s,
        Err(e) => {
            *status.lock().unwrap() = format!("open failed: {}", e);
            log().push(
                LogLevel::Error,
                format!("{}: open failed: {}", config.port, e),
            );
            return;
        }
    };

    let udp = match UdpSocket::bind(("0.0.0.0", config.udp_port)) {
        Ok(u) => u,
        Err(e) => {
            *status.lock().unwrap() = format!("udp bind failed: {}", e);
            log().push(
                LogLevel::Error,
                format!("{}: udp bind failed: {}", config.port, e),
            );
            return;
        }
    };
    let _ = udp.set_nonblocking(true);
    let bound = udp.local_addr().ok().map(|a| a.port());
    *bound_udp.lock().unwrap() = bound;
    *status.lock().unwrap() = "running".to_string();
    log().push(
        LogLevel::Info,
        format!(
            "{}: bridge started (udp {})",
            config.port,
            bound
                .map(|p| p.to_string())
                .unwrap_or_else(|| "?".to_string())
        ),
    );

    let mut raw = [0u8; MAX_ENCODED];
    let mut raw_len = 0usize;
    let mut decoded = [0u8; MAX_PAYLOAD];
    let mut udp_buf = [0u8; MAX_PAYLOAD];
    let mut frame = [0u8; MAX_FRAME];
    let mut chunk = [0u8; 256];
    let mut peer: Option<SocketAddr> = None;

    while !shutdown.load(Ordering::Relaxed) {
        // Serial -> UDP.
        loop {
            let avail = match serial.bytes_to_read() {
                Ok(a) => a,
                Err(_) => break,
            };
            if avail == 0 {
                break;
            }
            let want = avail.min(chunk.len() as u32) as usize;
            let read = match serial.read(&mut chunk[..want]) {
                Ok(r) => r,
                Err(_) => break,
            };
            if read == 0 {
                break;
            }
            for &b in &chunk[..read] {
                if b == 0 {
                    if raw_len > 0 {
                        match decode_frame(&raw[..raw_len], &mut decoded) {
                            Some(n) => {
                                if let Some(p) = peer {
                                    let _ = udp.send_to(&decoded[..n], p);
                                }
                                rx.fetch_add(1, Ordering::Relaxed);
                                // Message type 0x03 carries a CBOR payload.
                                if n > 1 && decoded[0] == 0x03 {
                                    *last_cbor.lock().unwrap() =
                                        format!("{}", minicbor::display(&decoded[1..n]));
                                }
                            }
                            None => {
                                log().push(
                                    LogLevel::Error,
                                    format!(
                                        "{}: COBS parse failed ({} bytes)",
                                        config.port, raw_len
                                    ),
                                );
                            }
                        }
                        raw_len = 0;
                    }
                } else if raw_len < raw.len() {
                    raw[raw_len] = b;
                    raw_len += 1;
                } else {
                    log().push(
                        LogLevel::Error,
                        format!("{}: COBS frame too long (>{})", config.port, raw.len()),
                    );
                    raw_len = 0;
                }
            }
        }

        // UDP -> Serial.
        match udp.recv_from(&mut udp_buf) {
            Ok((n, p)) => {
                peer = Some(p);
                let n = n.min(MAX_PAYLOAD);
                let m = encode_frame(&udp_buf[..n], &mut frame);
                if m > 0 && serial.write_all(&frame[..m]).is_err() {
                    *status.lock().unwrap() = "serial write error".to_string();
                    log().push(
                        LogLevel::Error,
                        format!("{}: serial write error", config.port),
                    );
                    break;
                }
                tx.fetch_add(1, Ordering::Relaxed);
            }
            Err(ref e) if e.kind() == ErrorKind::WouldBlock => {}
            Err(_) => {}
        }

        std::thread::sleep(Duration::from_millis(1));
    }

    *status.lock().unwrap() = "stopped".to_string();
    log().push(
        LogLevel::Info,
        format!("{}: bridge stopped", config.port),
    );
}

// ===== Detection =====

/// List plausible serial device paths under `/dev` (Linux).
fn available_ports() -> Vec<String> {
    let mut ports = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/dev") {
        for e in entries.flatten() {
            let name = e.file_name().to_string_lossy().into_owned();
            if name.starts_with("ttyUSB") || name.starts_with("ttyACM") || name.starts_with("ttyS")
            {
                ports.push(format!("/dev/{}", name));
            }
        }
    }
    ports.sort();
    ports
}

/// Read the USB vendor:product id of a serial device from sysfs (Linux).
/// Returns e.g. `"1A86:7523"`, or `None` for non-USB ports / a missing device.
fn usb_vid_pid(dev: &str) -> Option<String> {
    let name = dev.rsplit('/').next()?;
    let base = format!("/sys/class/tty/{}/device", name);
    for path in [format!("{}/uevent", base), format!("{}/../uevent", base)] {
        if let Ok(text) = std::fs::read_to_string(&path) {
            for line in text.lines() {
                if let Some(rest) = line.strip_prefix("PRODUCT=") {
                    let mut it = rest.split('/');
                    if let (Some(vid), Some(pid)) = (it.next(), it.next()) {
                        return Some(format!("{}:{}", vid.to_uppercase(), pid.to_uppercase()));
                    }
                }
            }
        }
    }
    None
}

// ===== App =====

struct App {
    ports: Vec<PortRuntime>,
    config_path: String,
}

impl App {
    fn new(config: Config, config_path: String) -> Self {
        let mut app = Self {
            ports: config.ports.into_iter().map(PortRuntime::new).collect(),
            config_path,
        };
        app.refresh_detection();
        log().push(
            LogLevel::Info,
            format!("loaded config with {} port(s)", app.ports.len()),
        );
        app
    }

    fn refresh_detection(&mut self) {
        let available = available_ports();
        for rt in &mut self.ports {
            rt.detected = available.iter().any(|a| a == &rt.config.port);
            rt.vid_pid = usb_vid_pid(&rt.config.port).unwrap_or_else(|| "—".to_string());
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::bottom("logs").show(ctx, |ui| {
            ui.add_space(2.0);
            ui.label(egui::RichText::new("Log").strong());
            ui.separator();
            for line in log().last(5) {
                let color = match line.level {
                    LogLevel::Info => egui::Color32::LIGHT_GRAY,
                    LogLevel::Error => egui::Color32::LIGHT_RED,
                };
                ui.colored_label(
                    color,
                    format!("[{}] [{}] {}", line.time, line.level.as_str(), line.text),
                );
            }
            ui.add_space(2.0);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Serial bridge manager");
                if ui.button("Refresh").clicked() {
                    self.refresh_detection();
                }
            });
            ui.label(format!("config: {}", self.config_path));
            ui.separator();

            egui::Grid::new("ports")
                .striped(true)
                .min_col_width(70.0)
                .show(ui, |ui| {
                    ui.strong("Name");
                    ui.strong("Device");
                    ui.strong("VID:PID");
                    ui.strong("Status");
                    ui.strong("Sent");
                    ui.strong("Received");
                    ui.strong("CBOR");
                    ui.strong("UDP port");
                    ui.strong("");
                    ui.end_row();

                    for rt in &mut self.ports {
                        let name = rt.config.name.clone();
                        let running = rt.is_running();
                        let detected = rt.detected;
                        let status = rt.status_text();
                        let tx = rt.tx.load(Ordering::Relaxed);
                        let rx = rt.rx.load(Ordering::Relaxed);
                        let udp = rt.udp_text();
                        let cbor = rt.cbor_text();

                        ui.label(&name);
                        ui.label(&rt.config.port);
                        ui.label(&rt.vid_pid);
                        if running {
                            ui.colored_label(egui::Color32::GREEN, "running");
                        } else if detected {
                            ui.colored_label(egui::Color32::LIGHT_BLUE, &status);
                        } else {
                            ui.colored_label(egui::Color32::GRAY, "not detected");
                        }
                        ui.label(tx.to_string());
                        ui.label(rx.to_string());
                        ui.label(cbor);
                        ui.label(udp);
                        if running {
                            if ui.button("Disable").clicked() {
                                rt.stop();
                            }
                        } else if ui.button("Enable").clicked() {
                            rt.start();
                        }
                        ui.end_row();
                    }
                });
        });

        ctx.request_repaint_after(Duration::from_secs(1));
    }
}

fn main() -> eframe::Result {
    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config.yaml".to_string());
    let config = Config::load(&config_path).unwrap_or_else(|e| {
        eprintln!("warning: failed to load {}: {}", config_path, e);
        Config { ports: Vec::new() }
    });

    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "lm4f120 bridge manager",
        options,
        Box::new(move |_cc| Ok(Box::new(App::new(config, config_path)))),
    )
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
    fn config_parse() {
        let cfg: Config = serde_yaml::from_str(
            "ports:\n  - name: A\n    port: /dev/ttyACM0\n    baud: 115200\n    udp_port: 5001\n",
        )
        .unwrap();
        assert_eq!(cfg.ports.len(), 1);
        assert_eq!(cfg.ports[0].name, "A");
        assert_eq!(cfg.ports[0].udp_port, 5001);
    }

    #[test]
    fn vid_pid_parse() {
        // PRODUCT=1a86/7523/100 -> 1A86:7523 (mirrors usb_vid_pid parsing).
        let rest = "1a86/7523/100";
        let mut it = rest.split('/');
        let (vid, pid) = (it.next().unwrap(), it.next().unwrap());
        assert_eq!(format!("{}:{}", vid.to_uppercase(), pid.to_uppercase()), "1A86:7523");
    }

    #[test]
    fn log_buffer_keeps_last_n() {
        let buf = LogBuffer::default();
        for i in 0..10 {
            buf.push(LogLevel::Info, format!("msg {}", i));
        }
        let last = buf.last(3);
        assert_eq!(last.len(), 3);
        assert_eq!(last[0].text, "msg 7");
        assert_eq!(last[2].text, "msg 9");
    }
}
