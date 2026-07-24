//! AK-series driver "servo mode" serial protocol (manual section 5.2.2).
//!
//! Frame: 0x02, len(1 byte), payload[len], crc16_hi, crc16_lo, 0x03
//! crc16 is CRC-16/XMODEM (poly 0x1021, init 0x0000) computed over `payload` only.

const CRC16_TAB: [u16; 256] = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
];

pub fn crc16(buf: &[u8]) -> u16 {
    let mut cksum: u16 = 0;
    for &b in buf {
        let idx = (((cksum >> 8) ^ b as u16) & 0xFF) as usize;
        cksum = CRC16_TAB[idx] ^ (cksum << 8);
    }
    cksum
}

/// Command/frame identifiers used by the servo-mode serial protocol (manual 5.2.2).
#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum CommId {
    FwVersion = 0,
    JumpToBootloader = 1,
    EraseNewApp = 2,
    WriteNewAppData = 3,
    GetValues = 4,
    SetDuty = 5,
    SetCurrent = 6,
    SetCurrentBrake = 7,
    SetRpm = 8,
    SetPos = 9,
    SetHandbrake = 10,
    SetDetect = 11,
    /// Sends an ASCII debug/console command (manual 5.3.1) — used by MIT mode's serial
    /// port for `encoder`, `calibrate`, `exit`, etc. Response comes back as `Print`.
    TerminalCmd = 20,
    /// ASCII text printed by the driver's debug console, in reply to a `TerminalCmd`.
    Print = 21,
    RotorPosition = 22,
    GetValuesSetup = 50,
    SetPosSpd = 91,
    SetPosMulti = 92,
    SetPosSingle = 93,
    SetPosUnlimited = 94,
    SetPosOrigin = 95,
}

fn encode_frame(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len() + 5);
    out.push(0x02);
    out.push(payload.len() as u8);
    out.extend_from_slice(payload);
    let crc = crc16(payload);
    out.push((crc >> 8) as u8);
    out.push((crc & 0xFF) as u8);
    out.push(0x03);
    out
}

fn i32_payload(id: CommId, value: i32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(5);
    payload.push(id as u8);
    payload.extend_from_slice(&value.to_be_bytes());
    payload
}

pub fn set_duty(duty: f32) -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetDuty, (duty * 100_000.0) as i32))
}

pub fn set_current(amps: f32) -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetCurrent, (amps * 1000.0) as i32))
}

pub fn set_current_brake(amps: f32) -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetCurrentBrake, (amps * 1000.0) as i32))
}

pub fn set_rpm(erpm: f32) -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetRpm, erpm as i32))
}

pub fn set_pos(degrees: f32) -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetPos, (degrees * 1_000_000.0) as i32))
}

pub fn set_handbrake(amps: f32) -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetHandbrake, (amps * 1000.0) as i32))
}

pub fn set_pos_spd(degrees: f32, erpm: i32, accel: i32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(13);
    payload.push(CommId::SetPosSpd as u8);
    payload.extend_from_slice(&((degrees * 1000.0) as i32).to_be_bytes());
    payload.extend_from_slice(&erpm.to_be_bytes());
    payload.extend_from_slice(&accel.to_be_bytes());
    encode_frame(&payload)
}

pub fn set_pos_multi_loop() -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetPosMulti, 0))
}

pub fn set_pos_single_loop() -> Vec<u8> {
    encode_frame(&i32_payload(CommId::SetPosSingle, 0))
}

/// `permanent`: false = temporary origin (cleared on power loss), true = permanent zero point.
pub fn set_origin(permanent: bool) -> Vec<u8> {
    let payload = vec![CommId::SetPosOrigin as u8, if permanent { 1 } else { 0 }];
    encode_frame(&payload)
}

pub fn get_values() -> Vec<u8> {
    encode_frame(&[CommId::GetValues as u8])
}

/// Builds an ASCII debug/console command frame (manual 5.3.1.1), e.g. `terminal_cmd("calibrate")`.
pub fn terminal_cmd(text: &str) -> Vec<u8> {
    let mut payload = Vec::with_capacity(text.len() + 1);
    payload.push(CommId::TerminalCmd as u8);
    payload.extend_from_slice(text.as_bytes());
    encode_frame(&payload)
}

/// Parses a `Print` reply frame's payload into the driver's console text, if it is one.
pub fn parse_print(payload: &[u8]) -> Option<String> {
    if payload.first().copied() != Some(CommId::Print as u8) {
        return None;
    }
    Some(String::from_utf8_lossy(&payload[1..]).into_owned())
}

/// Parsed reply to `get_values()` (manual 5.2.2.1 §1).
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorValues {
    pub mos_temp_c: f32,
    pub motor_temp_c: f32,
    pub output_current_a: f32,
    pub input_current_a: f32,
    pub id_current_a: f32,
    pub iq_current_a: f32,
    pub duty: f32,
    pub speed_erpm: f32,
    pub input_voltage_v: f32,
    pub fault_code: u8,
    pub position_deg: f32,
    pub motor_id: u8,
    pub vd_v: f32,
    pub vq_v: f32,
}

pub fn fault_code_name(code: u8) -> &'static str {
    match code {
        0 => "None",
        1 => "Over voltage",
        2 => "Under voltage",
        3 => "Driver fault",
        4 => "Motor over-current",
        5 => "MOS over-temperature",
        6 => "Motor over-temperature",
        7 => "Driver gate over-voltage",
        8 => "Driver gate under-voltage",
        9 => "MCU under-voltage",
        10 => "Booting from watchdog reset",
        11 => "Encoder SPI fault",
        12 => "Encoder sin/cos below min amplitude",
        13 => "Encoder sin/cos above max amplitude",
        14 => "Flash corruption",
        15 => "Current sensor 1 offset fault",
        16 => "Current sensor 2 offset fault",
        17 => "Current sensor 3 offset fault",
        18 => "Unbalanced currents",
        _ => "Unknown",
    }
}

fn get_i16(buf: &[u8], idx: &mut usize) -> i16 {
    let v = i16::from_be_bytes([buf[*idx], buf[*idx + 1]]);
    *idx += 2;
    v
}

fn get_i32(buf: &[u8], idx: &mut usize) -> i32 {
    let v = i32::from_be_bytes([buf[*idx], buf[*idx + 1], buf[*idx + 2], buf[*idx + 3]]);
    *idx += 4;
    v
}

/// Parses the payload of a `COMM_GET_VALUES` (id 0x04) reply frame.
/// `payload` is the frame's data section, i.e. `[0x04, ...]`, with the leading id byte still present.
pub fn parse_get_values(payload: &[u8]) -> Option<MotorValues> {
    // id(1) + mos(2) + motor(2) + out_i(4) + in_i(4) + id_i(4) + iq_i(4) + duty(2) + speed(4)
    // + voltage(2) + reserved(24) + status(1) + pos(4) + motor_id(1) + reserved(6) + vd(4) + vq(4)
    const MIN_LEN: usize = 1 + 2 + 2 + 4 + 4 + 4 + 4 + 2 + 4 + 2 + 24 + 1 + 4 + 1 + 6 + 4 + 4;
    if payload.len() < MIN_LEN || payload[0] != CommId::GetValues as u8 {
        return None;
    }
    let mut i = 1usize;
    let mos_temp_c = get_i16(payload, &mut i) as f32 / 10.0;
    let motor_temp_c = get_i16(payload, &mut i) as f32 / 10.0;
    let output_current_a = get_i32(payload, &mut i) as f32 / 100.0;
    let input_current_a = get_i32(payload, &mut i) as f32 / 100.0;
    let id_current_a = get_i32(payload, &mut i) as f32 / 100.0;
    let iq_current_a = get_i32(payload, &mut i) as f32 / 100.0;
    let duty = get_i16(payload, &mut i) as f32 / 1000.0;
    let speed_erpm = get_i32(payload, &mut i) as f32;
    let input_voltage_v = get_i16(payload, &mut i) as f32 / 10.0;
    i += 24; // reserved
    let fault_code = payload[i];
    i += 1;
    let position_deg = get_i32(payload, &mut i) as f32 / 1000.0;
    let motor_id = payload[i];
    i += 1;
    i += 6; // reserved
    let vd_v = get_i32(payload, &mut i) as f32 / 1000.0;
    let vq_v = get_i32(payload, &mut i) as f32 / 1000.0;

    Some(MotorValues {
        mos_temp_c,
        motor_temp_c,
        output_current_a,
        input_current_a,
        id_current_a,
        iq_current_a,
        duty,
        speed_erpm,
        input_voltage_v,
        fault_code,
        position_deg,
        motor_id,
        vd_v,
        vq_v,
    })
}

/// A validated, extracted frame's payload (header/len/crc/tail already stripped and checked).
pub struct FrameParser {
    buf: Vec<u8>,
}

impl FrameParser {
    pub fn new() -> Self {
        Self { buf: Vec::new() }
    }

    pub fn feed(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Pops the next complete, CRC-valid frame's payload out of the buffer, if any.
    /// Resyncs past malformed data automatically.
    pub fn next_frame(&mut self) -> Option<Vec<u8>> {
        loop {
            let start = self.buf.iter().position(|&b| b == 0x02)?;
            if start > 0 {
                self.buf.drain(0..start);
            }
            // buf[0] == 0x02
            if self.buf.len() < 2 {
                return None;
            }
            let len = self.buf[1] as usize;
            let total = 2 + len + 2 + 1;
            if self.buf.len() < total {
                return None;
            }
            let payload = self.buf[2..2 + len].to_vec();
            let crc_hi = self.buf[2 + len];
            let crc_lo = self.buf[2 + len + 1];
            let tail = self.buf[2 + len + 2];
            let crc = crc16(&payload);
            if tail == 0x03 && crc_hi == (crc >> 8) as u8 && crc_lo == (crc & 0xFF) as u8 {
                self.buf.drain(0..total);
                return Some(payload);
            } else {
                // Not a valid frame at this position; drop the leading 0x02 and resync.
                self.buf.drain(0..1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_manual_examples() {
        assert_eq!(crc16(&[0x04]), 0x4084);
        assert_eq!(crc16(&[0x08, 0x00, 0x00, 0x03, 0xE8]), 0x2B58);
        assert_eq!(crc16(&[0x09, 0x0A, 0xBA, 0x95, 0x00]), 0x1EF7);
    }

    #[test]
    fn set_rpm_matches_manual_example() {
        assert_eq!(set_rpm(1000.0), vec![0x02, 0x05, 0x08, 0x00, 0x00, 0x03, 0xE8, 0x2B, 0x58, 0x03]);
        assert_eq!(
            set_rpm(-1000.0),
            vec![0x02, 0x05, 0x08, 0xFF, 0xFF, 0xFC, 0x18, 0x43, 0x78, 0x03]
        );
    }

    #[test]
    fn set_pos_matches_manual_example() {
        assert_eq!(
            set_pos(180.0),
            vec![0x02, 0x05, 0x09, 0x0A, 0xBA, 0x95, 0x00, 0x1E, 0xF7, 0x03]
        );
        assert_eq!(
            set_pos(90.0),
            vec![0x02, 0x05, 0x09, 0x05, 0x5D, 0x4A, 0x80, 0x7B, 0x29, 0x03]
        );
    }

    #[test]
    fn set_duty_matches_manual_example() {
        assert_eq!(
            set_duty(0.20),
            vec![0x02, 0x05, 0x05, 0x00, 0x00, 0x4E, 0x20, 0x29, 0xF6, 0x03]
        );
        assert_eq!(
            set_duty(-0.20),
            vec![0x02, 0x05, 0x05, 0xFF, 0xFF, 0xB1, 0xE0, 0x77, 0x85, 0x03]
        );
    }

    #[test]
    fn set_current_matches_manual_example() {
        assert_eq!(
            set_current(5.0),
            vec![0x02, 0x05, 0x06, 0x00, 0x00, 0x13, 0x88, 0x8B, 0x25, 0x03]
        );
    }

    #[test]
    fn set_pos_spd_matches_manual_example() {
        // 180 degrees, 5000 ERPM, accel 30000
        assert_eq!(
            set_pos_spd(180.0, 5000, 30000),
            vec![
                0x02, 0x0D, 0x5B, 0x00, 0x02, 0xBF, 0x20, 0x00, 0x00, 0x13, 0x88, 0x00, 0x00,
                0x75, 0x30, 0xA5, 0xAC, 0x03
            ]
        );
    }

    #[test]
    fn get_values_matches_manual_example() {
        assert_eq!(get_values(), vec![0x02, 0x01, 0x04, 0x40, 0x84, 0x03]);
    }

    #[test]
    fn terminal_cmd_matches_manual_examples() {
        assert_eq!(
            terminal_cmd("encoder"),
            vec![0x02, 0x08, 0x14, 0x65, 0x6E, 0x63, 0x6F, 0x64, 0x65, 0x72, 0xB0, 0x4C, 0x03]
        );
        assert_eq!(
            terminal_cmd("calibrate"),
            vec![
                0x02, 0x0A, 0x14, 0x63, 0x61, 0x6C, 0x69, 0x62, 0x72, 0x61, 0x74, 0x65, 0x76, 0xA5,
                0x03
            ]
        );
        assert_eq!(
            terminal_cmd("exit"),
            vec![0x02, 0x05, 0x14, 0x65, 0x78, 0x69, 0x74, 0x96, 0xC3, 0x03]
        );
    }

    #[test]
    fn parse_print_decodes_console_text() {
        let payload = [vec![0x15u8], b"hello".to_vec()].concat();
        assert_eq!(parse_print(&payload), Some("hello".to_string()));
        assert_eq!(parse_print(&[0x04, 0x01]), None);
    }

    #[test]
    fn frame_parser_extracts_and_resyncs() {
        let mut p = FrameParser::new();
        let mut junk = vec![0xAA, 0xBB];
        junk.extend_from_slice(&set_rpm(1000.0));
        junk.extend_from_slice(&set_pos(90.0));
        p.feed(&junk);
        let f1 = p.next_frame().unwrap();
        assert_eq!(f1, vec![0x08, 0x00, 0x00, 0x03, 0xE8]);
        let f2 = p.next_frame().unwrap();
        assert_eq!(f2, vec![0x09, 0x05, 0x5D, 0x4A, 0x80]);
        assert!(p.next_frame().is_none());
    }
}
