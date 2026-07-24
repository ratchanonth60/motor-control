use std::sync::mpsc::{Receiver, Sender};
use std::time::{Duration, Instant};

use eframe::egui;

use crate::can_worker::{self, CanProtocol, FromCanWorker, ToCanWorker};
use crate::mit_protocol::{self, MitFeedback, MotorLimits, MOTOR_PRESETS};
use crate::protocol::{self, fault_code_name, MotorValues};
use crate::serial_worker::{self, FromWorker, ToWorker};
use crate::servo_can::{self, ServoCanValues};

#[derive(PartialEq, Clone, Copy)]
enum Mode {
    Servo,
    Mit,
}

#[derive(PartialEq, Clone, Copy)]
enum ServoTransport {
    Serial,
    Can,
}

#[derive(PartialEq, Clone, Copy)]
enum MitTransport {
    Can,
    Serial,
}

#[derive(PartialEq, Clone, Copy)]
enum ControlTab {
    Duty,
    Current,
    Brake,
    Velocity,
    Position,
    PosVelocity,
    Handbrake,
    LoopMode,
    Origin,
}

/// Tabs with no documented CAN equivalent (manual 5.1's CAN_PACKET_ID enum has no
/// handbrake or loop-mode-switch entries — those are serial-only COMM_PACKET_ID commands).
fn tab_supports_can(tab: ControlTab) -> bool {
    !matches!(tab, ControlTab::Handbrake | ControlTab::LoopMode)
}

pub struct MotorApp {
    mode: Mode,
    servo_transport: ServoTransport,

    // ---- Servo mode: serial transport ----
    ports: Vec<String>,
    selected_port: Option<String>,
    baud: u32,
    serial_connected: bool,
    serial_status: String,
    to_serial: Option<Sender<ToWorker>>,
    from_serial: Option<Receiver<FromWorker>>,
    values: Option<MotorValues>,
    last_values_at: Option<Instant>,
    poll_live: bool,
    last_poll_sent: Instant,
    serial_log: Vec<String>,

    // ---- Servo mode: CAN transport ----
    servo_can_iface: String,
    servo_can_motor_id: u8,
    servo_can_connected: bool,
    servo_can_status: String,
    to_servo_can: Option<Sender<ToCanWorker>>,
    from_servo_can: Option<Receiver<FromCanWorker>>,
    servo_can_values: Option<ServoCanValues>,
    servo_can_last_values_at: Option<Instant>,
    servo_can_log: Vec<String>,

    // ---- Servo mode: shared control inputs (used by both transports) ----
    tab: ControlTab,
    duty: f32,
    current_a: f32,
    brake_a: f32,
    rpm: f32,
    pos_deg: f32,
    pos_spd_deg: f32,
    pos_spd_erpm: i32,
    pos_spd_accel: i32,
    handbrake_a: f32,
    origin_permanent: bool,

    // ---- MIT mode ----
    mit_transport: MitTransport,
    can_iface: String,
    mit_can_id: u16,
    motor_preset: usize,
    mit_connected: bool,
    mit_status: String,
    to_mit: Option<Sender<ToCanWorker>>,
    from_mit: Option<Receiver<FromCanWorker>>,
    mit_control_enabled: bool,
    mit_target_pos_rad: f32,
    mit_target_vel_rad_s: f32,
    mit_kp: f32,
    mit_kd: f32,
    mit_torque_nm: f32,
    mit_feedback: Option<MitFeedback>,
    mit_last_feedback_at: Option<Instant>,
    mit_send_stream: bool,
    mit_last_stream_sent: Instant,
    mit_log: Vec<String>,
}

impl Default for MotorApp {
    fn default() -> Self {
        Self {
            mode: Mode::Servo,
            // Default to Serial: confirmed working directly over the R-Link's UART line
            // at 921600 baud, with no CAN adapter or SocketCAN setup required.
            servo_transport: ServoTransport::Serial,

            ports: Vec::new(),
            selected_port: None,
            baud: 921_600,
            serial_connected: false,
            serial_status: "Not connected".to_string(),
            to_serial: None,
            from_serial: None,
            values: None,
            last_values_at: None,
            poll_live: true,
            last_poll_sent: Instant::now(),
            serial_log: Vec::new(),

            servo_can_iface: "can0".to_string(),
            servo_can_motor_id: 1,
            servo_can_connected: false,
            servo_can_status: "Not connected".to_string(),
            to_servo_can: None,
            from_servo_can: None,
            servo_can_values: None,
            servo_can_last_values_at: None,
            servo_can_log: Vec::new(),

            tab: ControlTab::Velocity,
            duty: 0.0,
            current_a: 0.0,
            brake_a: 0.0,
            rpm: 0.0,
            pos_deg: 0.0,
            pos_spd_deg: 0.0,
            pos_spd_erpm: 5000,
            pos_spd_accel: 30000,
            handbrake_a: 0.0,
            origin_permanent: false,

            // Real-time position/velocity/torque control is only documented over CAN
            // (manual 5.3) — serial only exposes debug/calibration commands (5.3.1).
            mit_transport: MitTransport::Can,
            can_iface: "can0".to_string(),
            mit_can_id: 1,
            motor_preset: 0,
            mit_connected: false,
            mit_status: "Not connected".to_string(),
            to_mit: None,
            from_mit: None,
            mit_control_enabled: false,
            mit_target_pos_rad: 0.0,
            mit_target_vel_rad_s: 0.0,
            mit_kp: 0.0,
            mit_kd: 0.0,
            mit_torque_nm: 0.0,
            mit_feedback: None,
            mit_last_feedback_at: None,
            mit_send_stream: false,
            mit_last_stream_sent: Instant::now(),
            mit_log: Vec::new(),
        }
    }
}

impl MotorApp {
    pub fn new() -> Self {
        let mut app = Self::default();
        app.refresh_ports();
        app
    }

    fn current_limits(&self) -> MotorLimits {
        MOTOR_PRESETS[self.motor_preset]
    }

    // ---------------- Servo mode: serial transport ----------------

    fn refresh_ports(&mut self) {
        self.ports = serialport::available_ports()
            .map(|ports| ports.into_iter().map(|p| p.port_name).collect())
            .unwrap_or_default();
        if self.selected_port.is_none() {
            self.selected_port = self.ports.first().cloned();
        }
    }

    fn serial_connect(&mut self) {
        let Some(port_name) = self.selected_port.clone() else {
            self.serial_status = "No port selected".to_string();
            return;
        };
        let (to_worker_tx, to_worker_rx) = std::sync::mpsc::channel();
        let (from_worker_tx, from_worker_rx) = std::sync::mpsc::channel();
        let baud = self.baud;
        std::thread::spawn(move || {
            serial_worker::run(port_name, baud, to_worker_rx, from_worker_tx);
        });
        self.to_serial = Some(to_worker_tx);
        self.from_serial = Some(from_worker_rx);
        self.serial_status = "Connecting...".to_string();
    }

    fn serial_disconnect(&mut self) {
        if let Some(tx) = &self.to_serial {
            let _ = tx.send(ToWorker::Disconnect);
        }
        self.to_serial = None;
        self.from_serial = None;
        self.serial_connected = false;
        self.serial_status = "Not connected".to_string();
    }

    fn serial_send(&mut self, frame: Vec<u8>) {
        if let Some(tx) = &self.to_serial {
            let _ = tx.send(ToWorker::Send(frame));
        } else {
            self.serial_status = "Not connected".to_string();
        }
    }

    fn drain_serial_events(&mut self) {
        let mut disconnect_msg: Option<String> = None;
        if let Some(rx) = &self.from_serial {
            while let Ok(event) = rx.try_recv() {
                match event {
                    FromWorker::Connected => {
                        self.serial_connected = true;
                        self.serial_status = "Connected".to_string();
                    }
                    FromWorker::Disconnected(msg) => disconnect_msg = Some(msg),
                    FromWorker::Values(v) => {
                        self.values = Some(v);
                        self.last_values_at = Some(Instant::now());
                    }
                    FromWorker::Log(msg) => push_capped(&mut self.serial_log, msg),
                }
            }
        }
        if let Some(msg) = disconnect_msg {
            self.serial_connected = false;
            self.serial_status = msg.clone();
            push_capped(&mut self.serial_log, msg);
            self.to_serial = None;
            self.from_serial = None;
        }
    }

    // ---------------- Servo mode: CAN transport ----------------

    fn servo_can_connect(&mut self) {
        let (to_tx, to_rx) = std::sync::mpsc::channel();
        let (from_tx, from_rx) = std::sync::mpsc::channel();
        let iface = self.servo_can_iface.clone();
        std::thread::spawn(move || {
            can_worker::run(iface, CanProtocol::ServoCan, to_rx, from_tx);
        });
        self.to_servo_can = Some(to_tx);
        self.from_servo_can = Some(from_rx);
        self.servo_can_status = "Connecting...".to_string();
    }

    fn servo_can_disconnect(&mut self) {
        if let Some(tx) = &self.to_servo_can {
            let _ = tx.send(ToCanWorker::Disconnect);
        }
        self.to_servo_can = None;
        self.from_servo_can = None;
        self.servo_can_connected = false;
        self.servo_can_status = "Not connected".to_string();
    }

    fn servo_can_send(&mut self, id: u32, data: Vec<u8>) {
        if let Some(tx) = &self.to_servo_can {
            let _ = tx.send(ToCanWorker::SendExt(id, data));
        } else {
            self.servo_can_status = "Not connected".to_string();
        }
    }

    fn drain_servo_can_events(&mut self) {
        let mut disconnect_msg: Option<String> = None;
        if let Some(rx) = &self.from_servo_can {
            while let Ok(event) = rx.try_recv() {
                match event {
                    FromCanWorker::Connected => {
                        self.servo_can_connected = true;
                        self.servo_can_status = "Connected".to_string();
                    }
                    FromCanWorker::Disconnected(msg) => disconnect_msg = Some(msg),
                    FromCanWorker::ServoValues(v) => {
                        self.servo_can_values = Some(v);
                        self.servo_can_last_values_at = Some(Instant::now());
                    }
                    FromCanWorker::MitFeedback(_) => {}
                    FromCanWorker::Log(msg) => push_capped(&mut self.servo_can_log, msg),
                }
            }
        }
        if let Some(msg) = disconnect_msg {
            self.servo_can_connected = false;
            self.servo_can_status = msg.clone();
            push_capped(&mut self.servo_can_log, msg);
            self.to_servo_can = None;
            self.from_servo_can = None;
        }
    }

    // ---------------- Servo mode: transport-agnostic dispatch ----------------

    /// Sends a servo-mode control command over whichever transport is currently selected.
    fn servo_dispatch(&mut self, serial_frame: Vec<u8>, can_frame: (u32, Vec<u8>)) {
        match self.servo_transport {
            ServoTransport::Serial => self.serial_send(serial_frame),
            ServoTransport::Can => {
                let (id, data) = can_frame;
                self.servo_can_send(id, data);
            }
        }
    }

    fn servo_stop(&mut self) {
        let motor_id = self.servo_can_motor_id;
        self.servo_dispatch(
            protocol::set_current_brake(0.0),
            servo_can::set_current_brake(motor_id, 0.0),
        );
        self.duty = 0.0;
        self.current_a = 0.0;
        self.brake_a = 0.0;
        self.rpm = 0.0;
    }

    // ---------------- MIT mode ----------------

    fn mit_connect(&mut self) {
        let (to_tx, to_rx) = std::sync::mpsc::channel();
        let (from_tx, from_rx) = std::sync::mpsc::channel();
        let iface = self.can_iface.clone();
        let limits = self.current_limits();
        std::thread::spawn(move || {
            can_worker::run(iface, CanProtocol::Mit(limits), to_rx, from_tx);
        });
        self.to_mit = Some(to_tx);
        self.from_mit = Some(from_rx);
        self.mit_status = "Connecting...".to_string();
    }

    fn mit_disconnect(&mut self) {
        if let Some(tx) = &self.to_mit {
            let _ = tx.send(ToCanWorker::Disconnect);
        }
        self.to_mit = None;
        self.from_mit = None;
        self.mit_connected = false;
        self.mit_control_enabled = false;
        self.mit_status = "Not connected".to_string();
    }

    fn mit_send(&mut self, data: [u8; 8]) {
        let id = self.mit_can_id;
        if let Some(tx) = &self.to_mit {
            let _ = tx.send(ToCanWorker::SendStd(id, data));
        } else {
            self.mit_status = "Not connected".to_string();
        }
    }

    fn mit_enable(&mut self) {
        self.mit_send(mit_protocol::ENTER_MOTOR_MODE);
        self.mit_control_enabled = true;
    }

    fn mit_disable(&mut self) {
        self.mit_send(mit_protocol::EXIT_MOTOR_MODE);
        self.mit_control_enabled = false;
    }

    fn mit_set_zero(&mut self) {
        self.mit_send(mit_protocol::SET_ZERO_POSITION);
    }

    fn mit_send_command(&mut self) {
        let limits = self.current_limits();
        let cmd = mit_protocol::pack_cmd(
            &limits,
            self.mit_target_pos_rad,
            self.mit_target_vel_rad_s,
            self.mit_kp,
            self.mit_kd,
            self.mit_torque_nm,
        );
        self.mit_send(cmd);
    }

    fn mit_stop(&mut self) {
        self.mit_target_vel_rad_s = 0.0;
        self.mit_torque_nm = 0.0;
        self.mit_kp = 0.0;
        let limits = self.current_limits();
        let pos = self.mit_target_pos_rad;
        let kd = self.mit_kd.max(0.5); // small damping so it doesn't free-spin
        let cmd = mit_protocol::pack_cmd(&limits, pos, 0.0, 0.0, kd, 0.0);
        self.mit_send(cmd);
        self.mit_disable();
    }

    fn drain_mit_events(&mut self) {
        let mut disconnect_msg: Option<String> = None;
        if let Some(rx) = &self.from_mit {
            while let Ok(event) = rx.try_recv() {
                match event {
                    FromCanWorker::Connected => {
                        self.mit_connected = true;
                        self.mit_status = "Connected".to_string();
                    }
                    FromCanWorker::Disconnected(msg) => disconnect_msg = Some(msg),
                    FromCanWorker::MitFeedback(fb) => {
                        self.mit_feedback = Some(fb);
                        self.mit_last_feedback_at = Some(Instant::now());
                    }
                    FromCanWorker::ServoValues(_) => {}
                    FromCanWorker::Log(msg) => push_capped(&mut self.mit_log, msg),
                }
            }
        }
        if let Some(msg) = disconnect_msg {
            self.mit_connected = false;
            self.mit_control_enabled = false;
            self.mit_status = msg.clone();
            push_capped(&mut self.mit_log, msg);
            self.to_mit = None;
            self.from_mit = None;
        }
    }

    // ---------------- UI ----------------

    /// Port/baud/connect controls for the shared serial connection — used by both
    /// Servo mode's Serial transport and MIT mode's Serial (debug-command) transport,
    /// since both talk to the same physical UART.
    fn serial_connect_controls(&mut self, ui: &mut egui::Ui) {
        ui.label("Port:");
        egui::ComboBox::from_id_salt("port_combo")
            .selected_text(self.selected_port.clone().unwrap_or_else(|| "-".to_string()))
            .show_ui(ui, |ui| {
                for p in self.ports.clone() {
                    ui.selectable_value(&mut self.selected_port, Some(p.clone()), p);
                }
            });
        if ui.button("Refresh").clicked() {
            self.refresh_ports();
        }
        ui.label("Baud:");
        ui.add(egui::DragValue::new(&mut self.baud).range(9600..=2_000_000));

        if !self.serial_connected {
            if ui.button("Connect").clicked() {
                self.serial_connect();
            }
        } else if ui.button("Disconnect").clicked() {
            self.serial_disconnect();
        }
    }

    fn show_servo_ui(&mut self, ui: &mut egui::Ui) {
        if self.serial_connected
            && self.servo_transport == ServoTransport::Serial
            && self.poll_live
            && self.last_poll_sent.elapsed() > Duration::from_millis(100)
        {
            self.serial_send(protocol::get_values());
            self.last_poll_sent = Instant::now();
        }

        let (connected, status_text): (bool, String) = match self.servo_transport {
            ServoTransport::Serial => (self.serial_connected, self.serial_status.clone()),
            ServoTransport::Can => (self.servo_can_connected, self.servo_can_status.clone()),
        };

        egui::Panel::top("servo_connection").show(ui, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Transport:").strong());
                ui.selectable_value(&mut self.servo_transport, ServoTransport::Serial, "Serial");
                ui.selectable_value(&mut self.servo_transport, ServoTransport::Can, "CAN");
            });
            ui.separator();

            match self.servo_transport {
                ServoTransport::Serial => {
                    ui.horizontal(|ui| {
                        self.serial_connect_controls(ui);
                        ui.separator();
                        ui.checkbox(&mut self.poll_live, "Live telemetry (10 Hz)");
                        ui.separator();
                        ui.colored_label(
                            if connected { egui::Color32::GREEN } else { egui::Color32::GRAY },
                            &status_text,
                        );
                    });
                }
                ServoTransport::Can => {
                    ui.horizontal(|ui| {
                        ui.label("CAN interface:");
                        ui.text_edit_singleline(&mut self.servo_can_iface);
                        ui.label("Motor CAN ID:");
                        let mut id_val = self.servo_can_motor_id as i32;
                        if ui.add(egui::DragValue::new(&mut id_val).range(0..=255)).changed() {
                            self.servo_can_motor_id = id_val as u8;
                        }

                        if !self.servo_can_connected {
                            if ui.button("Connect").clicked() {
                                self.servo_can_connect();
                            }
                        } else if ui.button("Disconnect").clicked() {
                            self.servo_can_disconnect();
                        }

                        ui.separator();
                        ui.colored_label(
                            if connected { egui::Color32::GREEN } else { egui::Color32::GRAY },
                            &status_text,
                        );
                    });
                    ui.small("Telemetry arrives automatically (driver broadcasts it) — no polling needed.");
                }
            }
            ui.add_space(4.0);
        });

        egui::Panel::right("servo_telemetry").min_size(260.0).show(ui, |ui| {
            ui.heading("Telemetry");
            match self.servo_transport {
                ServoTransport::Serial => {
                    if let Some(v) = &self.values {
                        egui::Grid::new("telemetry_grid").num_columns(2).striped(true).show(ui, |ui| {
                            ui.label("MOS temp");
                            ui.label(format!("{:.1} °C", v.mos_temp_c));
                            ui.end_row();
                            ui.label("Motor temp");
                            ui.label(format!("{:.1} °C", v.motor_temp_c));
                            ui.end_row();
                            ui.label("Output current");
                            ui.label(format!("{:.2} A", v.output_current_a));
                            ui.end_row();
                            ui.label("Input current");
                            ui.label(format!("{:.2} A", v.input_current_a));
                            ui.end_row();
                            ui.label("Id current");
                            ui.label(format!("{:.2} A", v.id_current_a));
                            ui.end_row();
                            ui.label("Iq current");
                            ui.label(format!("{:.2} A", v.iq_current_a));
                            ui.end_row();
                            ui.label("Duty");
                            ui.label(format!("{:.3}", v.duty));
                            ui.end_row();
                            ui.label("Speed");
                            ui.label(format!("{:.0} ERPM", v.speed_erpm));
                            ui.end_row();
                            ui.label("Input voltage");
                            ui.label(format!("{:.1} V", v.input_voltage_v));
                            ui.end_row();
                            ui.label("Position");
                            ui.label(format!("{:.3} °", v.position_deg));
                            ui.end_row();
                            ui.label("Motor ID");
                            ui.label(format!("{}", v.motor_id));
                            ui.end_row();
                            ui.label("Vd / Vq");
                            ui.label(format!("{:.2} V / {:.2} V", v.vd_v, v.vq_v));
                            ui.end_row();
                            ui.label("Fault");
                            let color = if v.fault_code == 0 { egui::Color32::GREEN } else { egui::Color32::RED };
                            ui.colored_label(color, fault_code_name(v.fault_code));
                            ui.end_row();
                        });
                        if let Some(t) = self.last_values_at {
                            ui.small(format!("updated {:.1}s ago", t.elapsed().as_secs_f32()));
                        }
                    } else {
                        ui.label("No data yet. Connect and enable live telemetry, or click \"Get values\" below.");
                    }
                    if ui.button("Get values (once)").clicked() {
                        self.serial_send(protocol::get_values());
                    }
                }
                ServoTransport::Can => {
                    if let Some(v) = &self.servo_can_values {
                        egui::Grid::new("servo_can_grid").num_columns(2).striped(true).show(ui, |ui| {
                            ui.label("Position");
                            ui.label(format!("{:.1} °", v.position_deg));
                            ui.end_row();
                            ui.label("Speed");
                            ui.label(format!("{:.0} ERPM", v.speed_erpm));
                            ui.end_row();
                            ui.label("Current");
                            ui.label(format!("{:.2} A", v.current_a));
                            ui.end_row();
                            ui.label("Temperature");
                            ui.label(format!("{} °C", v.temperature_c));
                            ui.end_row();
                            ui.label("Fault");
                            let color = if v.fault_code == 0 { egui::Color32::GREEN } else { egui::Color32::RED };
                            ui.colored_label(color, fault_code_name(v.fault_code));
                            ui.end_row();
                        });
                        if let Some(t) = self.servo_can_last_values_at {
                            ui.small(format!("updated {:.1}s ago", t.elapsed().as_secs_f32()));
                        }
                    } else {
                        ui.label("No telemetry yet. Connect and wait for the driver's broadcast frames.");
                    }
                }
            }

            let log = match self.servo_transport {
                ServoTransport::Serial => &self.serial_log,
                ServoTransport::Can => &self.servo_can_log,
            };
            ui.separator();
            ui.heading("Log");
            egui::ScrollArea::vertical().max_height(240.0).stick_to_bottom(true).show(ui, |ui| {
                for line in log {
                    ui.small(line);
                }
            });
        });

        egui::Panel::bottom("servo_stop").show(ui, |ui| {
            ui.add_space(4.0);
            let stop_btn = egui::Button::new(
                egui::RichText::new("STOP").size(20.0).color(egui::Color32::WHITE),
            )
            .fill(egui::Color32::from_rgb(180, 30, 30));
            if ui.add_sized([ui.available_width(), 40.0], stop_btn).clicked() {
                self.servo_stop();
            }
            ui.add_space(4.0);
        });

        egui::CentralPanel::default().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.tab, ControlTab::Duty, "Duty cycle");
                ui.selectable_value(&mut self.tab, ControlTab::Current, "Current");
                ui.selectable_value(&mut self.tab, ControlTab::Brake, "Current brake");
                ui.selectable_value(&mut self.tab, ControlTab::Velocity, "Velocity");
                ui.selectable_value(&mut self.tab, ControlTab::Position, "Position");
                ui.selectable_value(&mut self.tab, ControlTab::PosVelocity, "Position+Velocity");
                ui.selectable_value(&mut self.tab, ControlTab::Handbrake, "Handbrake");
                ui.selectable_value(&mut self.tab, ControlTab::LoopMode, "Loop mode");
                ui.selectable_value(&mut self.tab, ControlTab::Origin, "Set origin");
            });
            ui.separator();

            if self.servo_transport == ServoTransport::Can && !tab_supports_can(self.tab) {
                ui.colored_label(
                    egui::Color32::YELLOW,
                    "Not available over CAN — the manual's CAN control-mode set has no handbrake or \
                     loop-mode-switch command. Switch transport to Serial to use this.",
                );
                return;
            }

            let motor_id = self.servo_can_motor_id;
            match self.tab {
                ControlTab::Duty => {
                    ui.label("Duty cycle (-1.0 .. 1.0, typical 0.005 - 0.95)");
                    ui.add(egui::Slider::new(&mut self.duty, -1.0..=1.0));
                    if ui.button("Send duty").clicked() {
                        let d = self.duty;
                        self.servo_dispatch(protocol::set_duty(d), servo_can::set_duty(motor_id, d));
                    }
                }
                ControlTab::Current => {
                    ui.label("Iq current (A) — torque = Iq × Kt");
                    ui.add(egui::Slider::new(&mut self.current_a, -60.0..=60.0));
                    if ui.button("Send current").clicked() {
                        let c = self.current_a;
                        self.servo_dispatch(protocol::set_current(c), servo_can::set_current(motor_id, c));
                    }
                }
                ControlTab::Brake => {
                    ui.label("Brake current (A) — holds position, monitor motor temperature");
                    ui.add(egui::Slider::new(&mut self.brake_a, 0.0..=60.0));
                    if ui.button("Send brake current").clicked() {
                        let c = self.brake_a;
                        self.servo_dispatch(
                            protocol::set_current_brake(c),
                            servo_can::set_current_brake(motor_id, c),
                        );
                    }
                }
                ControlTab::Velocity => {
                    ui.label("Speed (ERPM, electrical RPM)");
                    ui.add(egui::Slider::new(&mut self.rpm, -100_000.0..=100_000.0));
                    if ui.button("Send velocity").clicked() {
                        let r = self.rpm;
                        self.servo_dispatch(protocol::set_rpm(r), servo_can::set_rpm(motor_id, r));
                    }
                }
                ControlTab::Position => {
                    ui.label("Target position (degrees)");
                    ui.add(egui::Slider::new(&mut self.pos_deg, -36_000.0..=36_000.0));
                    ui.label("Motor moves to this position at maximum speed.");
                    if ui.button("Send position").clicked() {
                        let p = self.pos_deg;
                        self.servo_dispatch(protocol::set_pos(p), servo_can::set_pos(motor_id, p));
                    }
                }
                ControlTab::PosVelocity => {
                    ui.label("Target position (degrees)");
                    ui.add(egui::Slider::new(&mut self.pos_spd_deg, -36_000.0..=36_000.0));
                    ui.label("Speed (ERPM)");
                    ui.add(egui::Slider::new(&mut self.pos_spd_erpm, -100_000..=100_000));
                    ui.label("Acceleration (ERPM/s)");
                    ui.add(egui::Slider::new(&mut self.pos_spd_accel, 0..=100_000));
                    if ui.button("Send position+velocity").clicked() {
                        let (p, s, a) = (self.pos_spd_deg, self.pos_spd_erpm, self.pos_spd_accel);
                        self.servo_dispatch(
                            protocol::set_pos_spd(p, s, a),
                            servo_can::set_pos_spd(motor_id, p, s, a),
                        );
                    }
                }
                ControlTab::Handbrake => {
                    ui.label("Handbrake current (A)");
                    ui.add(egui::Slider::new(&mut self.handbrake_a, 0.0..=60.0));
                    if ui.button("Send handbrake").clicked() {
                        let c = self.handbrake_a;
                        self.serial_send(protocol::set_handbrake(c));
                    }
                }
                ControlTab::LoopMode => {
                    ui.label("Multi-loop mode: position range ±100 turns.");
                    if ui.button("Set multi-loop mode").clicked() {
                        self.serial_send(protocol::set_pos_multi_loop());
                    }
                    ui.add_space(8.0);
                    ui.label("Single-loop mode: position range 0-360°.");
                    if ui.button("Set single-loop mode").clicked() {
                        self.serial_send(protocol::set_pos_single_loop());
                    }
                }
                ControlTab::Origin => {
                    ui.checkbox(&mut self.origin_permanent, "Permanent zero point (dual-encoder models only)");
                    ui.label("Unchecked = temporary origin, cleared on power loss.");
                    if ui.button("Set current position as origin").clicked() {
                        let permanent = self.origin_permanent;
                        self.servo_dispatch(
                            protocol::set_origin(permanent),
                            servo_can::set_origin(motor_id, permanent),
                        );
                    }
                }
            }
        });
    }

    fn show_mit_ui(&mut self, ui: &mut egui::Ui) {
        if self.mit_connected
            && self.mit_control_enabled
            && self.mit_send_stream
            && self.mit_last_stream_sent.elapsed() > Duration::from_millis(20)
        {
            self.mit_send_command();
            self.mit_last_stream_sent = Instant::now();
        }

        egui::Panel::top("mit_connection").show(ui, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Transport:").strong());
                ui.selectable_value(&mut self.mit_transport, MitTransport::Can, "CAN");
                ui.selectable_value(&mut self.mit_transport, MitTransport::Serial, "Serial");
            });
            ui.separator();

            match self.mit_transport {
                MitTransport::Can => {
                    ui.horizontal(|ui| {
                        ui.label("CAN interface:");
                        ui.text_edit_singleline(&mut self.can_iface);

                        ui.label("Motor CAN ID:");
                        let mut id_val = self.mit_can_id as i32;
                        if ui.add(egui::DragValue::new(&mut id_val).range(1..=127)).changed() {
                            self.mit_can_id = id_val as u16;
                        }

                        ui.label("Model:");
                        egui::ComboBox::from_id_salt("motor_preset")
                            .selected_text(MOTOR_PRESETS[self.motor_preset].name)
                            .show_ui(ui, |ui| {
                                for (i, m) in MOTOR_PRESETS.iter().enumerate() {
                                    ui.selectable_value(&mut self.motor_preset, i, m.name);
                                }
                            });

                        if !self.mit_connected {
                            if ui.button("Connect").clicked() {
                                self.mit_connect();
                            }
                        } else if ui.button("Disconnect").clicked() {
                            self.mit_disconnect();
                        }

                        ui.separator();
                        ui.colored_label(
                            if self.mit_connected { egui::Color32::GREEN } else { egui::Color32::GRAY },
                            &self.mit_status,
                        );
                    });
                    ui.horizontal(|ui| {
                        let limits = self.current_limits();
                        ui.small(format!(
                            "pos ±{:.1} rad, vel ±{:.1} rad/s, torque ±{:.1} N·m, Kp 0-{:.0}, Kd 0-{:.1}",
                            limits.p_max, limits.v_max, limits.t_max, limits.kp_max, limits.kd_max
                        ));
                    });
                }
                MitTransport::Serial => {
                    ui.horizontal(|ui| {
                        self.serial_connect_controls(ui);
                        ui.separator();
                        ui.colored_label(
                            if self.serial_connected { egui::Color32::GREEN } else { egui::Color32::GRAY },
                            &self.serial_status,
                        );
                    });
                    ui.small(
                        "Manual 5.3.1 only documents debug/calibration commands over serial — \
                         real-time position/velocity/torque control is CAN-only (5.3). Switch \
                         transport to CAN for that.",
                    );
                }
            }
            ui.add_space(4.0);
        });

        egui::Panel::right("mit_telemetry").min_size(260.0).show(ui, |ui| {
            match self.mit_transport {
                MitTransport::Can => {
                    ui.heading("Feedback");
                    if let Some(fb) = &self.mit_feedback {
                        egui::Grid::new("mit_grid").num_columns(2).striped(true).show(ui, |ui| {
                            ui.label("Driver ID");
                            ui.label(format!("{}", fb.driver_id));
                            ui.end_row();
                            ui.label("Position");
                            ui.label(format!("{:.3} rad", fb.position_rad));
                            ui.end_row();
                            ui.label("Speed");
                            ui.label(format!("{:.3} rad/s", fb.speed_rad_s));
                            ui.end_row();
                            ui.label("Torque");
                            ui.label(format!("{:.3} N·m", fb.torque_nm));
                            ui.end_row();
                            ui.label("Temperature");
                            ui.label(format!("{} °C", fb.temperature_c));
                            ui.end_row();
                            ui.label("Error");
                            let color = if fb.error == 0 { egui::Color32::GREEN } else { egui::Color32::RED };
                            ui.colored_label(color, format!("{}", fb.error));
                            ui.end_row();
                        });
                        if let Some(t) = self.mit_last_feedback_at {
                            ui.small(format!("updated {:.1}s ago", t.elapsed().as_secs_f32()));
                        }
                    } else {
                        ui.label("No feedback yet. Connect, enable control, and send a command.");
                    }
                    ui.separator();
                    ui.heading("Log");
                    egui::ScrollArea::vertical().max_height(240.0).stick_to_bottom(true).show(ui, |ui| {
                        for line in &self.mit_log {
                            ui.small(line);
                        }
                    });
                }
                MitTransport::Serial => {
                    ui.heading("Debug console");
                    ui.label("Responses to debug commands (encoder, calibrate, exit) appear here.");
                    ui.separator();
                    egui::ScrollArea::vertical().max_height(400.0).stick_to_bottom(true).show(ui, |ui| {
                        for line in &self.serial_log {
                            ui.small(line);
                        }
                    });
                }
            }
        });

        if self.mit_transport == MitTransport::Can {
            egui::Panel::bottom("mit_stop").show(ui, |ui| {
                ui.add_space(4.0);
                let stop_btn = egui::Button::new(
                    egui::RichText::new("STOP").size(20.0).color(egui::Color32::WHITE),
                )
                .fill(egui::Color32::from_rgb(180, 30, 30));
                if ui.add_sized([ui.available_width(), 40.0], stop_btn).clicked() {
                    self.mit_stop();
                }
                ui.add_space(4.0);
            });
        }

        egui::CentralPanel::default().show(ui, |ui| match self.mit_transport {
            MitTransport::Can => {
                ui.horizontal(|ui| {
                    if !self.mit_control_enabled {
                        if ui.button("Enable control").clicked() {
                            self.mit_enable();
                        }
                    } else if ui.button("Disable control").clicked() {
                        self.mit_disable();
                    }
                    if ui.button("Set zero position").clicked() {
                        self.mit_set_zero();
                    }
                    ui.checkbox(&mut self.mit_send_stream, "Stream command @ 50 Hz");
                });
                ui.separator();

                let limits = self.current_limits();
                ui.label("Target position (rad)");
                ui.add(egui::Slider::new(&mut self.mit_target_pos_rad, limits.p_min..=limits.p_max));
                ui.label("Target velocity (rad/s)");
                ui.add(egui::Slider::new(&mut self.mit_target_vel_rad_s, limits.v_min..=limits.v_max));
                ui.label("Kp (position gain)");
                ui.add(egui::Slider::new(&mut self.mit_kp, limits.kp_min..=limits.kp_max));
                ui.label("Kd (velocity gain)");
                ui.add(egui::Slider::new(&mut self.mit_kd, limits.kd_min..=limits.kd_max));
                ui.label("Feed-forward torque (N·m)");
                ui.add(egui::Slider::new(&mut self.mit_torque_nm, limits.t_min..=limits.t_max));

                ui.add_space(8.0);
                ui.add_enabled_ui(!self.mit_send_stream, |ui| {
                    if ui.button("Send command once").clicked() {
                        self.mit_send_command();
                    }
                });
                if !self.mit_control_enabled {
                    ui.colored_label(egui::Color32::YELLOW, "Control is not enabled — click \"Enable control\" first.");
                }
            }
            MitTransport::Serial => {
                ui.label("Debug/calibration commands (manual 5.3.1.1):");
                ui.add_space(8.0);
                if ui.button("Get encoder value").clicked() {
                    self.serial_send(protocol::terminal_cmd("encoder"));
                }
                ui.add_space(4.0);
                if ui
                    .button(
                        egui::RichText::new("Calibrate ⚠ motor rotates for ~30s")
                            .color(egui::Color32::from_rgb(200, 120, 0)),
                    )
                    .clicked()
                {
                    self.serial_send(protocol::terminal_cmd("calibrate"));
                }
                ui.add_space(4.0);
                if ui.button("Exit debug console").clicked() {
                    self.serial_send(protocol::terminal_cmd("exit"));
                }
            }
        });
    }
}

fn push_capped(log: &mut Vec<String>, msg: String) {
    log.push(msg);
    if log.len() > 200 {
        log.remove(0);
    }
}

impl eframe::App for MotorApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.drain_serial_events();
        self.drain_servo_can_events();
        self.drain_mit_events();

        egui::Panel::top("mode_switch").show(ui, |ui| {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Mode:").strong());
                ui.selectable_value(&mut self.mode, Mode::Servo, "Servo mode");
                ui.selectable_value(&mut self.mode, Mode::Mit, "MIT mode (CAN)");
            });
            ui.add_space(4.0);
        });

        match self.mode {
            Mode::Servo => self.show_servo_ui(ui),
            Mode::Mit => self.show_mit_ui(ui),
        }

        if self.serial_connected || self.servo_can_connected || self.mit_connected {
            ctx.request_repaint_after(Duration::from_millis(20));
        }
    }
}
