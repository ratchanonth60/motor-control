use std::io::{Read, Write};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::time::Duration;

use crate::protocol::{self, FrameParser, MotorValues};

pub enum ToWorker {
    Send(Vec<u8>),
    Disconnect,
}

pub enum FromWorker {
    Connected,
    Disconnected(String),
    Values(MotorValues),
    Log(String),
}

/// Runs on a background thread: owns the serial port, forwards outgoing frames,
/// and parses incoming frames into `FromWorker` events for the UI to poll.
pub fn run(
    port_name: String,
    baud: u32,
    to_worker: Receiver<ToWorker>,
    from_worker: Sender<FromWorker>,
) {
    let port = serialport::new(&port_name, baud)
        .timeout(Duration::from_millis(20))
        .open();

    let mut port = match port {
        Ok(p) => p,
        Err(e) => {
            let _ = from_worker.send(FromWorker::Disconnected(format!(
                "Failed to open {port_name}: {e}"
            )));
            return;
        }
    };

    let _ = from_worker.send(FromWorker::Connected);

    let mut parser = FrameParser::new();
    let mut read_buf = [0u8; 512];

    loop {
        match to_worker.try_recv() {
            Ok(ToWorker::Send(bytes)) => {
                if let Err(e) = port.write_all(&bytes) {
                    let _ = from_worker.send(FromWorker::Disconnected(format!("Write error: {e}")));
                    return;
                }
            }
            Ok(ToWorker::Disconnect) => {
                let _ = from_worker.send(FromWorker::Disconnected("Disconnected".to_string()));
                return;
            }
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => {
                return;
            }
        }

        match port.read(&mut read_buf) {
            Ok(0) => {}
            Ok(n) => {
                parser.feed(&read_buf[..n]);
                while let Some(payload) = parser.next_frame() {
                    if let Some(values) = protocol::parse_get_values(&payload) {
                        let _ = from_worker.send(FromWorker::Values(values));
                    } else if let Some(text) = protocol::parse_print(&payload) {
                        let _ = from_worker.send(FromWorker::Log(text));
                    } else {
                        let _ = from_worker.send(FromWorker::Log(format!(
                            "RX frame ({} bytes): {}",
                            payload.len(),
                            hex(&payload)
                        )));
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                let _ = from_worker.send(FromWorker::Disconnected(format!("Read error: {e}")));
                return;
            }
        }
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}
