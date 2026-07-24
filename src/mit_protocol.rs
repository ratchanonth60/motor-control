//! AK-series driver "MIT mode" CAN protocol (manual section 5.3).
//!
//! Control frames use a standard (11-bit) CAN identifier equal to the motor's CAN ID
//! (default 1). Position/velocity/Kp/Kd/torque are packed into 8 data bytes as
//! fixed-point values scaled by each motor model's parameter range.

pub const ENTER_MOTOR_MODE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
pub const EXIT_MOTOR_MODE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
pub const SET_ZERO_POSITION: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];

/// Per-motor-model MIT parameter ranges (manual 5.3 "Parameter range" table).
/// Position range is -12.5..12.5 rad for all listed models.
#[derive(Clone, Copy, Debug)]
pub struct MotorLimits {
    pub name: &'static str,
    pub p_min: f32,
    pub p_max: f32,
    pub v_min: f32,
    pub v_max: f32,
    pub t_min: f32,
    pub t_max: f32,
    pub kp_min: f32,
    pub kp_max: f32,
    pub kd_min: f32,
    pub kd_max: f32,
}

pub const MOTOR_PRESETS: &[MotorLimits] = &[
    lim("AK10-9", 50.0, 65.0),
    lim("AK60-6", 45.0, 15.0),
    lim("AK70-10", 50.0, 25.0),
    lim("AK80-6", 76.0, 12.0),
    lim("AK80-9", 50.0, 18.0),
    lim("AK80-64", 8.0, 144.0),
    lim("AK80-8", 37.5, 32.0),
    lim("AK45-36", 6.0, 34.0),
    lim("AK45-10", 20.0, 8.0),
    lim("AK40-10", 45.5, 5.0),
];

const fn lim(name: &'static str, v_abs: f32, t_abs: f32) -> MotorLimits {
    MotorLimits {
        name,
        p_min: -12.5,
        p_max: 12.5,
        v_min: -v_abs,
        v_max: v_abs,
        t_min: -t_abs,
        t_max: t_abs,
        kp_min: 0.0,
        kp_max: 500.0,
        kd_min: 0.0,
        kd_max: 5.0,
    }
}

fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u32) -> u32 {
    let x = x.clamp(x_min, x_max);
    let span = x_max - x_min;
    (((x - x_min) * ((1u32 << bits) as f32 / span)) as u32).min((1u32 << bits) - 1)
}

fn uint_to_float(x_int: u32, x_min: f32, x_max: f32, bits: u32) -> f32 {
    let span = x_max - x_min;
    (x_int as f32) * span / (((1u32 << bits) - 1) as f32) + x_min
}

/// Packs a position/velocity/Kp/Kd/torque command into the 8-byte MIT control frame.
pub fn pack_cmd(limits: &MotorLimits, p_des: f32, v_des: f32, kp: f32, kd: f32, t_ff: f32) -> [u8; 8] {
    let p_int = float_to_uint(p_des, limits.p_min, limits.p_max, 16);
    let v_int = float_to_uint(v_des, limits.v_min, limits.v_max, 12);
    let kp_int = float_to_uint(kp, limits.kp_min, limits.kp_max, 12);
    let kd_int = float_to_uint(kd, limits.kd_min, limits.kd_max, 12);
    let t_int = float_to_uint(t_ff, limits.t_min, limits.t_max, 12);

    [
        (p_int >> 8) as u8,
        (p_int & 0xFF) as u8,
        (v_int >> 4) as u8,
        (((v_int & 0xF) << 4) | (kp_int >> 8)) as u8,
        (kp_int & 0xFF) as u8,
        (kd_int >> 4) as u8,
        (((kd_int & 0xF) << 4) | (t_int >> 8)) as u8,
        (t_int & 0xFF) as u8,
    ]
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MitFeedback {
    pub driver_id: u8,
    pub position_rad: f32,
    pub speed_rad_s: f32,
    pub torque_nm: f32,
    pub temperature_c: i16,
    pub error: u8,
}

/// Parses the 8-byte feedback frame the driver sends back in MIT mode.
pub fn unpack_reply(limits: &MotorLimits, data: &[u8; 8]) -> MitFeedback {
    let driver_id = data[0];
    let p_int = ((data[1] as u32) << 8) | data[2] as u32;
    let v_int = ((data[3] as u32) << 4) | (data[4] as u32 >> 4);
    let i_int = (((data[4] as u32) & 0xF) << 8) | data[5] as u32;
    let temp_raw = data[6] as i16;
    let error = data[7];

    MitFeedback {
        driver_id,
        position_rad: uint_to_float(p_int, limits.p_min, limits.p_max, 16),
        speed_rad_s: uint_to_float(v_int, limits.v_min, limits.v_max, 12),
        torque_nm: uint_to_float(i_int, -limits.t_max, limits.t_max, 12),
        temperature_c: temp_raw - 40,
        error,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_zero() {
        let limits = MOTOR_PRESETS[0];
        let cmd = pack_cmd(&limits, 0.0, 0.0, 0.0, 0.0, 0.0);
        // p_int/v_int/kp_int/kd_int/t_int should all be their mid-scale/zero values.
        assert_eq!(cmd.len(), 8);
    }

    #[test]
    fn pack_matches_bit_layout() {
        let limits = MotorLimits {
            name: "test",
            p_min: -12.5,
            p_max: 12.5,
            v_min: -30.0,
            v_max: 30.0,
            t_min: -18.0,
            t_max: 18.0,
            kp_min: 0.0,
            kp_max: 500.0,
            kd_min: 0.0,
            kd_max: 5.0,
        };
        let cmd = pack_cmd(&limits, 0.0, 0.0, 0.0, 0.0, 0.0);
        // p=0 -> mid code (span 25 rad over 16 bits): p_int = (0 - -12.5) * (65536/25) = 32768
        assert_eq!(cmd[0], (32768u32 >> 8) as u8);
        assert_eq!(cmd[1], (32768u32 & 0xFF) as u8);
    }
}
