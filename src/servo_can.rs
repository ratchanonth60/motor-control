//! AK-series driver "servo mode" CAN protocol (manual section 5.1 / 5.2.1).
//!
//! Extended (29-bit) CAN identifier: `(control_mode << 8) | motor_id`.
//! Unlike the serial framing (section 5.2.2), values here use the raw
//! big-endian int32/int16 scalings given by the C example code in 5.1.x
//! directly — notably position is `*10000` here, not `*1_000_000` like the
//! serial `COMM_SET_POS` command.

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
#[allow(clippy::enum_variant_names)] // mirrors the manual's CAN_PACKET_SET_* names verbatim
pub enum CanPacketId {
    SetDuty = 0,
    SetCurrent = 1,
    SetCurrentBrake = 2,
    SetRpm = 3,
    SetPos = 4,
    SetOriginHere = 5,
    SetPosSpd = 6,
}

/// Builds the extended CAN identifier for a servo-mode control frame.
pub fn can_id(mode: CanPacketId, motor_id: u8) -> u32 {
    ((mode as u32) << 8) | motor_id as u32
}

pub fn set_duty(motor_id: u8, duty: f32) -> (u32, Vec<u8>) {
    (can_id(CanPacketId::SetDuty, motor_id), (duty * 100_000.0).to_be_bytes().to_vec())
}

fn i32_bytes(v: i32) -> Vec<u8> {
    v.to_be_bytes().to_vec()
}

pub fn set_current(motor_id: u8, amps: f32) -> (u32, Vec<u8>) {
    (can_id(CanPacketId::SetCurrent, motor_id), i32_bytes((amps * 1000.0) as i32))
}

pub fn set_current_brake(motor_id: u8, amps: f32) -> (u32, Vec<u8>) {
    (can_id(CanPacketId::SetCurrentBrake, motor_id), i32_bytes((amps * 1000.0) as i32))
}

pub fn set_rpm(motor_id: u8, erpm: f32) -> (u32, Vec<u8>) {
    (can_id(CanPacketId::SetRpm, motor_id), i32_bytes(erpm as i32))
}

/// Position in degrees. Note the CAN scale (`*10000`) differs from the serial `COMM_SET_POS` scale (`*1000000`).
pub fn set_pos(motor_id: u8, degrees: f32) -> (u32, Vec<u8>) {
    (can_id(CanPacketId::SetPos, motor_id), i32_bytes((degrees * 10_000.0) as i32))
}

/// `permanent`: false = temporary origin (cleared on power loss), true = permanent zero point (dual-encoder models only).
pub fn set_origin(motor_id: u8, permanent: bool) -> (u32, Vec<u8>) {
    (can_id(CanPacketId::SetOriginHere, motor_id), vec![if permanent { 1 } else { 0 }])
}

/// Position (degrees), speed (ERPM), acceleration (ERPM/s) position-velocity loop command.
///
/// The wire format packs speed/accel into int16 fields at 1 unit = 10 ERPM (or ERPM/s), per
/// the manual's `spd/10.0`, `RPA/10.0` packing in `comm_can_set_pos_spd` (5.1.7) — so callers
/// pass true ERPM/ERPM-per-s here (range up to ±327670) rather than the pre-divided wire value.
pub fn set_pos_spd(motor_id: u8, degrees: f32, erpm: i32, accel_erpm_per_s: i32) -> (u32, Vec<u8>) {
    let to_wire_i16 = |v: i32| -> i16 { (v / 10).clamp(i16::MIN as i32, i16::MAX as i32) as i16 };
    let mut payload = Vec::with_capacity(8);
    payload.extend_from_slice(&((degrees * 10_000.0) as i32).to_be_bytes());
    payload.extend_from_slice(&to_wire_i16(erpm).to_be_bytes());
    payload.extend_from_slice(&to_wire_i16(accel_erpm_per_s).to_be_bytes());
    (can_id(CanPacketId::SetPosSpd, motor_id), payload)
}

/// Servo-mode timed telemetry upload frame (manual 5.2.1, function id 0x29).
#[derive(Clone, Copy, Debug, Default)]
pub struct ServoCanValues {
    pub position_deg: f32,
    pub speed_erpm: f32,
    pub current_a: f32,
    pub temperature_c: i8,
    pub fault_code: u8,
}

pub const UPLOAD_FUNCTION_ID: u32 = 0x29;

/// Extracts the function id (top byte) from an extended servo-mode CAN identifier.
pub fn function_id(ext_id: u32) -> u32 {
    ext_id >> 8
}

pub fn parse_upload(data: &[u8]) -> Option<ServoCanValues> {
    if data.len() != 8 {
        return None;
    }
    let pos_raw = i16::from_be_bytes([data[0], data[1]]);
    let speed_raw = i16::from_be_bytes([data[2], data[3]]);
    let current_raw = i16::from_be_bytes([data[4], data[5]]);
    Some(ServoCanValues {
        position_deg: pos_raw as f32 * 0.1,
        speed_erpm: speed_raw as f32 * 10.0,
        current_a: current_raw as f32 * 0.01,
        temperature_c: data[6] as i8,
        fault_code: data[7],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_real_captured_frame() {
        // Captured live from candump can0: `00002900 [8] FF A8 00 00 00 00 32 00`
        // extended id 0x2900 = function 0x29 (telemetry upload), motor id 0x00.
        let ext_id = 0x2900u32;
        assert_eq!(function_id(ext_id), UPLOAD_FUNCTION_ID);
        assert_eq!(ext_id & 0xFF, 0); // motor id

        let data = [0xFF, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00];
        let v = parse_upload(&data).unwrap();
        assert!((v.position_deg - (-8.8)).abs() < 1e-4);
        assert_eq!(v.speed_erpm, 0.0);
        assert_eq!(v.current_a, 0.0);
        assert_eq!(v.temperature_c, 50);
        assert_eq!(v.fault_code, 0);
    }

    #[test]
    fn set_pos_matches_can_example_scale() {
        // 180 degrees -> 1,800,000 (pos * 10000), per comm_can_set_pos in the manual.
        let (id, data) = set_pos(1, 180.0);
        assert_eq!(id, (4u32 << 8) | 1);
        assert_eq!(data, 1_800_000i32.to_be_bytes().to_vec());
    }

    #[test]
    fn set_rpm_matches_can_example_scale() {
        let (id, data) = set_rpm(1, 1000.0);
        assert_eq!(id, (3u32 << 8) | 1);
        assert_eq!(data, 1000i32.to_be_bytes().to_vec());
    }

    #[test]
    fn set_pos_spd_packs_erpm_at_1_to_10_scale() {
        // 180 deg, 5000 ERPM, 30000 ERPM/s -> wire speed=500, wire accel=3000
        let (id, data) = set_pos_spd(1, 180.0, 5000, 30000);
        assert_eq!(id, (6u32 << 8) | 1);
        let mut expected = 1_800_000i32.to_be_bytes().to_vec();
        expected.extend_from_slice(&500i16.to_be_bytes());
        expected.extend_from_slice(&3000i16.to_be_bytes());
        assert_eq!(data, expected);
    }
}
