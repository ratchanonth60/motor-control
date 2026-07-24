mod app;
mod can_worker;
mod mit_protocol;
mod protocol;
mod serial_worker;
mod servo_can;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default().with_inner_size([900.0, 640.0]),
        ..Default::default()
    };
    eframe::run_native(
        "AK Series Motor Control",
        options,
        Box::new(|_cc| Ok(Box::new(app::MotorApp::new()))),
    )
}
