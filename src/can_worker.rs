use std::sync::mpsc::{Receiver, Sender};

use crate::mit_protocol::{MitFeedback, MotorLimits};
use crate::servo_can::ServoCanValues;

/// Which protocol is running over this CAN connection, and any protocol-specific
/// state needed to decode incoming frames.
// On non-Linux builds the stub `run()` below never inspects these — that's expected,
// not a bug, since real CAN I/O only exists on Linux (SocketCAN).
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub enum CanProtocol {
    Mit(MotorLimits),
    ServoCan,
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub enum ToCanWorker {
    /// Standard (11-bit) id frame send: used by MIT mode.
    SendStd(u16, [u8; 8]),
    /// Extended (29-bit) id frame send: used by servo-mode-over-CAN.
    SendExt(u32, Vec<u8>),
    Disconnect,
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub enum FromCanWorker {
    Connected,
    Disconnected(String),
    MitFeedback(MitFeedback),
    ServoValues(ServoCanValues),
    Log(String),
}

/// Runs on a background thread: owns a CAN socket, forwards outgoing frames, and parses
/// incoming frames according to `protocol` into `FromCanWorker` events.
///
/// CAN control (SocketCAN) is Linux-only; on other platforms this immediately reports
/// that the transport isn't available rather than failing to build.
#[cfg(target_os = "linux")]
pub fn run(
    iface: String,
    protocol: CanProtocol,
    to_worker: Receiver<ToCanWorker>,
    from_worker: Sender<FromCanWorker>,
) {
    linux::run(iface, protocol, to_worker, from_worker)
}

#[cfg(not(target_os = "linux"))]
pub fn run(
    iface: String,
    _protocol: CanProtocol,
    _to_worker: Receiver<ToCanWorker>,
    from_worker: Sender<FromCanWorker>,
) {
    let _ = from_worker.send(FromCanWorker::Disconnected(format!(
        "Failed to open CAN interface {iface}: CAN control requires SocketCAN, which is Linux-only. \
         Use a Serial transport instead on this platform."
    )));
}

#[cfg(target_os = "linux")]
mod linux {
    use std::sync::mpsc::{Receiver, Sender, TryRecvError};
    use std::time::Duration;

    use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id, Socket, StandardId};

    use crate::mit_protocol;
    use crate::servo_can;

    use super::{CanProtocol, FromCanWorker, ToCanWorker};

    pub fn run(
        iface: String,
        protocol: CanProtocol,
        to_worker: Receiver<ToCanWorker>,
        from_worker: Sender<FromCanWorker>,
    ) {
        let socket = match CanSocket::open(&iface) {
            Ok(s) => s,
            Err(e) => {
                let _ = from_worker.send(FromCanWorker::Disconnected(format!(
                    "Failed to open CAN interface {iface}: {e}"
                )));
                return;
            }
        };
        if let Err(e) = socket.set_read_timeout(Duration::from_millis(20)) {
            let _ = from_worker.send(FromCanWorker::Disconnected(format!(
                "Failed to set read timeout on {iface}: {e}"
            )));
            return;
        }

        let _ = from_worker.send(FromCanWorker::Connected);

        loop {
            match to_worker.try_recv() {
                Ok(ToCanWorker::SendStd(can_id, data)) => {
                    let Some(id) = StandardId::new(can_id) else {
                        let _ = from_worker.send(FromCanWorker::Log(format!(
                            "Invalid standard CAN id: {can_id}"
                        )));
                        continue;
                    };
                    if !send_frame(&socket, Id::Standard(id), &data, &from_worker) {
                        return;
                    }
                }
                Ok(ToCanWorker::SendExt(can_id, data)) => {
                    let Some(id) = ExtendedId::new(can_id) else {
                        let _ = from_worker.send(FromCanWorker::Log(format!(
                            "Invalid extended CAN id: {can_id}"
                        )));
                        continue;
                    };
                    if !send_frame(&socket, Id::Extended(id), &data, &from_worker) {
                        return;
                    }
                }
                Ok(ToCanWorker::Disconnect) => {
                    let _ = from_worker.send(FromCanWorker::Disconnected("Disconnected".to_string()));
                    return;
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => return,
            }

            match socket.read_frame() {
                Ok(CanFrame::Data(frame)) => {
                    let data = frame.data();
                    match &protocol {
                        CanProtocol::Mit(limits) => {
                            // MIT feedback uses a standard (11-bit) id; servo-mode telemetry uses
                            // an extended (29-bit) id. Without this check, servo-mode frames
                            // (which are also 8 bytes) get misread as MIT feedback.
                            if matches!(frame.id(), Id::Standard(_)) && data.len() == 8 {
                                let mut arr = [0u8; 8];
                                arr.copy_from_slice(data);
                                let fb = mit_protocol::unpack_reply(limits, &arr);
                                let _ = from_worker.send(FromCanWorker::MitFeedback(fb));
                            }
                        }
                        CanProtocol::ServoCan => {
                            if let Id::Extended(id) = frame.id() {
                                let raw_id = id.as_raw();
                                if servo_can::function_id(raw_id) == servo_can::UPLOAD_FUNCTION_ID
                                    && let Some(v) = servo_can::parse_upload(data)
                                {
                                    let _ = from_worker.send(FromCanWorker::ServoValues(v));
                                }
                            }
                        }
                    }
                }
                Ok(_) => {}
                Err(e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => {
                    let _ = from_worker.send(FromCanWorker::Disconnected(format!("CAN read error: {e}")));
                    return;
                }
            }
        }
    }

    /// Sends a frame; on write failure, reports disconnection and returns false so the caller can stop.
    fn send_frame(socket: &CanSocket, id: Id, data: &[u8], from_worker: &Sender<FromCanWorker>) -> bool {
        match CanFrame::new(id, data) {
            Some(frame) => match socket.write_frame(&frame) {
                Ok(()) => true,
                Err(e) => {
                    let _ = from_worker.send(FromCanWorker::Disconnected(format!("CAN write error: {e}")));
                    false
                }
            },
            None => {
                let _ = from_worker.send(FromCanWorker::Log("Failed to build CAN frame".into()));
                true
            }
        }
    }
}
