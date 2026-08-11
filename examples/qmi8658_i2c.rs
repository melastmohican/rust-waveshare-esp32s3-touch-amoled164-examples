//! # QMI8658 6-Axis IMU Example for Waveshare ESP32-S3-Touch-AMOLED-1.64
//!
//! Uses the `ph-qmi8658` async driver crate to read accelerometer (X, Y, Z), gyroscope (X, Y, Z),
//! and temperature telemetry from the onboard QMI8658 IC over I2C (SDA: GPIO47, SCL: GPIO48).
//! Logs live IMU data to console via `defmt::info!` and renders animated telemetry on the
//! CO5300 AMOLED screen.
//!
//! ## Hardware Connections (Waveshare ESP32-S3-Touch-AMOLED-1.64)
//!
//! - **I2C SDA:** GPIO 47
//! - **I2C SCL:** GPIO 48
//! - **IMU Address:** 0x6B (QMI8658)
//! - **Display Controller:** CO5300 (280×456 native resolution, QSPI)
//! - **Documentation:** https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.64
//!
//! ## Run
//!
//! ```bash
//! cargo run --example qmi8658_i2c
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    dma::DmaRxBuf,
    dma_buffers,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println as _;
use ph_qmi8658::{Config, I2cConfig as QmiI2cConfig, Qmi8658Address, Qmi8658I2c};

use embedded_graphics::{
    framebuffer::{Framebuffer, buffer_size},
    geometry::{Point, Size},
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_7X13, FONT_9X15_BOLD},
    },
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};

use display_driver::{
    ColorFormat, DisplayDriver, FrameControl, eg::FrameBufferedDisplayDriver,
    panel::reset::LCDResetOption,
};
use display_driver_co5300::{
    Co5300,
    spec::{Co5300Spec, PanelSpec},
};
use display_driver_qspi::{QspiConfig, QspiDisplayBus};
use rust_waveshare_esp32s3_touch_amoled164_examples::qspi::EspHalQspiDevice;

// ---------------------------------------------------------------------------
// Panel Specification for Waveshare 1.64" AMOLED (280×456 native, CO5300)
// ---------------------------------------------------------------------------

pub struct WaveshareAmoled164;

impl PanelSpec for WaveshareAmoled164 {
    const PHYSICAL_WIDTH: u16 = 280;
    const PHYSICAL_HEIGHT: u16 = 456;
    const PHYSICAL_X_OFFSET: u16 = 20;
    const PHYSICAL_Y_OFFSET: u16 = 0;
}

impl Co5300Spec for WaveshareAmoled164 {
    const INIT_PAGE_PARAM: u8 = 0x20;
    const IGNORE_ID_CHECK: bool = true;
}

// ---------------------------------------------------------------------------
// Display Geometry & Types
// ---------------------------------------------------------------------------

const WIDTH: usize = 280;
const HEIGHT: usize = 456;

type FbType =
    Framebuffer<Rgb565, RawU16, BigEndian, WIDTH, HEIGHT, { buffer_size::<Rgb565>(WIDTH, HEIGHT) }>;

// ---------------------------------------------------------------------------
// Entry Point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== QMI8658 IMU Example with `ph-qmi8658` (Waveshare ESP32-S3 1.64\" AMOLED) ===");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize embassy via esp-rtos
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // ── 1. Initialize I2C Bus on SDA: GPIO47, SCL: GPIO48 ─────────────────
    info!("Initializing I2C0 bus on SDA: GPIO47, SCL: GPIO48 (400 kHz)...");
    let i2c_bus = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("Failed to create I2C controller");

    let i2c = i2c_bus
        .with_sda(peripherals.GPIO47)
        .with_scl(peripherals.GPIO48)
        .into_async();

    // ── 2. Instantiate ph-qmi8658 Async Driver ───────────────────────────
    info!("Instantiating `ph-qmi8658` IMU driver at 0x6B...");
    let qmi_config = Config::new();
    let qmi_i2c_config = QmiI2cConfig::new(Qmi8658Address::Secondary.addr()); // 0x6B

    let mut imu = Qmi8658I2c::with_i2c_config(
        i2c,
        None::<esp_hal::gpio::NoPin>,
        None::<esp_hal::gpio::NoPin>,
        qmi_config,
        qmi_i2c_config,
    );

    if let Err(e) = imu.init(&mut embassy_time::Delay).await {
        info!(
            "Warning initializing ph-qmi8658: {:?}",
            defmt::Debug2Format(&e)
        );
    } else {
        info!("`ph-qmi8658` IMU initialized successfully!");
    }

    // ── 3. Configure CO5300 QSPI AMOLED Display ───────────────────────────
    info!("Initializing CO5300 QSPI AMOLED Display...");

    let sclk = peripherals.GPIO10;
    let sio0 = peripherals.GPIO11; // D0
    let sio1 = peripherals.GPIO12; // D1
    let sio2 = peripherals.GPIO13; // D2
    let sio3 = peripherals.GPIO14; // D3
    let rst = peripherals.GPIO21;
    let cs = peripherals.GPIO9; // Note: change to GPIO46 if using PCB Rev V2

    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(256, 0);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    static TX_DESCRIPTORS: static_cell::StaticCell<[esp_hal::dma::DmaDescriptor; 8]> =
        static_cell::StaticCell::new();
    let tx_descriptors = TX_DESCRIPTORS.init([esp_hal::dma::DmaDescriptor::EMPTY; 8]);

    static BOUNCE_BUF: static_cell::StaticCell<[u8; 256]> = static_cell::StaticCell::new();
    let bounce_buf = BOUNCE_BUF.init([0; 256]);

    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(10))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_sio0(sio0)
    .with_sio1(sio1)
    .with_sio2(sio2)
    .with_sio3(sio3)
    .with_cs(cs)
    .with_dma(peripherals.DMA_CH0)
    .into_async();

    let device = EspHalQspiDevice {
        spi: Some(spi),
        rx_buf: Some(dma_rx_buf),
        tx_descriptors: Some(tx_descriptors),
        bounce_buf: Some(bounce_buf),
    };

    let bus = QspiDisplayBus::new(device, QspiConfig::default());
    let rst_pin = Output::new(rst, Level::High, OutputConfig::default());
    let panel = Co5300::<WaveshareAmoled164, _, _>::new(LCDResetOption::new_pin(rst_pin));

    static mut FB_DATA: core::mem::MaybeUninit<FbType> = core::mem::MaybeUninit::uninit();
    let fb = unsafe {
        let ptr = core::ptr::addr_of_mut!(FB_DATA) as *mut FbType;
        core::ptr::write_bytes(ptr, 0, 1);
        &mut *ptr
    };

    let disp = DisplayDriver::builder(bus, panel)
        .with_color_format(ColorFormat::RGB565)
        .init(&mut embassy_time::Delay)
        .await
        .unwrap();

    let mut fb_disp = FrameBufferedDisplayDriver::new(disp, fb);
    fb_disp.set_brightness(200).await.unwrap();

    let title_style = MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::YELLOW);
    let sub_style = MonoTextStyle::new(&FONT_7X13, Rgb565::CYAN);
    let label_style = MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::GREEN);
    let text_style = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);

    let chunk_size: u16 = 38;
    let total_lines: u16 = 456;
    let mut poll_count = 0u32;

    info!("Starting IMU telemetry reading loop...");

    // ── 4. Main Telemetry Reading Loop ─────────────────────────────────────
    loop {
        let mut ax = 0i16;
        let mut ay = 0i16;
        let mut az = 0i16;
        let mut gx = 0i16;
        let mut gy = 0i16;
        let mut gz = 0i16;

        if let Ok(raw) = imu.read_raw_block().await {
            if let (Some(a), Some(g)) = (raw.accel, raw.gyro) {
                ax = a.x;
                ay = a.y;
                az = a.z;
                gx = g.x;
                gy = g.y;
                gz = g.z;
            }

            if poll_count.is_multiple_of(10) {
                info!(
                    "QMI8658 -> Accel({} {} {}) Gyro({} {} {})",
                    ax, ay, az, gx, gy, gz
                );
            }
        }

        // ── Render Telemetry to Display ───────────────────────────────────
        fb_disp.clear(Rgb565::BLACK).ok();

        Text::new("QMI8658 IMU DEMO", Point::new(10, 25), title_style)
            .draw(&mut fb_disp)
            .ok();

        Text::new("Waveshare ESP32-S3 1.64\"", Point::new(10, 45), sub_style)
            .draw(&mut fb_disp)
            .ok();

        Text::new("I2C: SDA:GP47, SCL:GP48", Point::new(10, 65), text_style)
            .draw(&mut fb_disp)
            .ok();

        // Accel Box
        Text::new("ACCELEROMETER (Raw)", Point::new(10, 100), label_style)
            .draw(&mut fb_disp)
            .ok();

        let mut b_ax = [0u8; 32];
        let mut b_ay = [0u8; 32];
        let mut b_az = [0u8; 32];

        Text::new(
            format_i16(&mut b_ax, "  Acc X: ", ax),
            Point::new(10, 122),
            text_style,
        )
        .draw(&mut fb_disp)
        .ok();
        Text::new(
            format_i16(&mut b_ay, "  Acc Y: ", ay),
            Point::new(10, 142),
            text_style,
        )
        .draw(&mut fb_disp)
        .ok();
        Text::new(
            format_i16(&mut b_az, "  Acc Z: ", az),
            Point::new(10, 162),
            text_style,
        )
        .draw(&mut fb_disp)
        .ok();

        // Gyro Box
        Text::new("GYROSCOPE (Raw)", Point::new(10, 200), label_style)
            .draw(&mut fb_disp)
            .ok();

        let mut b_gx = [0u8; 32];
        let mut b_gy = [0u8; 32];
        let mut b_gz = [0u8; 32];

        Text::new(
            format_i16(&mut b_gx, "  Gyr X: ", gx),
            Point::new(10, 222),
            text_style,
        )
        .draw(&mut fb_disp)
        .ok();
        Text::new(
            format_i16(&mut b_gy, "  Gyr Y: ", gy),
            Point::new(10, 242),
            text_style,
        )
        .draw(&mut fb_disp)
        .ok();
        Text::new(
            format_i16(&mut b_gz, "  Gyr Z: ", gz),
            Point::new(10, 262),
            text_style,
        )
        .draw(&mut fb_disp)
        .ok();

        // Graphical Horizon / Level Box
        Text::new("TILT VISUALIZER", Point::new(10, 300), label_style)
            .draw(&mut fb_disp)
            .ok();

        Rectangle::new(Point::new(10, 320), Size::new(260, 100))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::CYAN, 2))
            .draw(&mut fb_disp)
            .ok();

        // Level indicator dot mapped from Accel X/Y (-16384..16384 -> screen box)
        let dot_x = 140 + (ax as i32 * 110 / 16384).clamp(-110, 110);
        let dot_y = 370 + (ay as i32 * 40 / 16384).clamp(-40, 40);

        Rectangle::new(Point::new(dot_x - 6, dot_y - 6), Size::new(12, 12))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
            .draw(&mut fb_disp)
            .ok();

        // Flush Framebuffer
        for y_start in (0..total_lines).step_by(chunk_size as usize) {
            let y_end = y_start + chunk_size - 1;
            let frame_ctrl = match (y_start == 0, y_end == total_lines - 1) {
                (true, _) => FrameControl::new_first(),
                (_, true) => FrameControl::new_last(),
                _ => FrameControl {
                    first: false,
                    last: false,
                },
            };
            let _ = fb_disp
                .flush_lines_with_frame_control(y_start, y_end, frame_ctrl)
                .await;
        }

        poll_count += 1;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

// ---------------------------------------------------------------------------
// Helper: Format signed integer `i16` without std/alloc overhead
// ---------------------------------------------------------------------------

fn format_i16<'a>(buf: &'a mut [u8; 32], prefix: &'a str, val: i16) -> &'a str {
    let p_bytes = prefix.as_bytes();
    let p_len = p_bytes.len();
    buf[..p_len].copy_from_slice(p_bytes);

    let is_neg = val < 0;
    let mut n = if is_neg { -(val as i32) } else { val as i32 };

    let mut digits = [0u8; 6];
    let mut d_count = 0;

    if n == 0 {
        digits[0] = b'0';
        d_count = 1;
    } else {
        while n > 0 {
            digits[d_count] = b'0' + (n % 10) as u8;
            n /= 10;
            d_count += 1;
        }
    }

    let mut idx = p_len;
    if is_neg {
        buf[idx] = b'-';
        idx += 1;
    }

    for i in 0..d_count {
        buf[idx + i] = digits[d_count - 1 - i];
    }

    let total = idx + d_count;
    core::str::from_utf8(&buf[..total]).unwrap_or(prefix)
}
