//! # I2C Scanner Example for Waveshare ESP32-S3-Touch-AMOLED-1.64
//!
//! Scans the official onboard I2C bus (SDA: GPIO47, SCL: GPIO48) for connected devices
//! (such as the QMI8658 6-axis IMU at address 0x6B, FT3168 touch controller at 0x38, etc.).
//!
//! Logs scan progress and detected addresses via `defmt::info!` and renders
//! formatted scan results on the onboard CO5300 AMOLED screen.
//!
//! ## Hardware Connections (Waveshare ESP32-S3-Touch-AMOLED-1.64)
//!
//! - **I2C SDA:** GPIO 47
//! - **I2C SCL:** GPIO 48
//! - **Display Controller:** CO5300 (280×456 native resolution, QSPI)
//! - **Documentation:** https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.64
//!
//! ## Run
//!
//! ```bash
//! cargo run --example i2c_scan
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_println as _;
use esp_hal::{
    delay::Delay,
    dma::DmaRxBuf,
    dma_buffers,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::Rate,
    timer::timg::TimerGroup,
};

use embedded_graphics::{
    framebuffer::{buffer_size, Framebuffer},
    geometry::Point,
    mono_font::{ascii::{FONT_7X13, FONT_9X15_BOLD}, MonoTextStyle},
    pixelcolor::{
        raw::{BigEndian, RawU16},
        Rgb565,
    },
    prelude::*,
    text::Text,
};

use display_driver::{
    eg::FrameBufferedDisplayDriver, panel::reset::LCDResetOption, ColorFormat, DisplayDriver,
    FrameControl,
};
use display_driver_co5300::{
    spec::{Co5300Spec, PanelSpec},
    Co5300,
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
// Display Geometry
// ---------------------------------------------------------------------------

const WIDTH: usize = 280;
const HEIGHT: usize = 456;

type FbType = Framebuffer<
    Rgb565,
    RawU16,
    BigEndian,
    WIDTH,
    HEIGHT,
    { buffer_size::<Rgb565>(WIDTH, HEIGHT) },
>;

// ---------------------------------------------------------------------------
// Helper: Identify known I2C device addresses
// ---------------------------------------------------------------------------

fn get_device_desc(addr: u8) -> &'static str {
    match addr {
        0x15 => "CST816 Touch",
        0x20 => "TCA9554 IO Expander",
        0x38 => "FT3168 Touch",
        0x3C | 0x3D => "SSD1306 OLED",
        0x51 | 0x52 => "PCF8563 RTC",
        0x5D => "GT911 Touch",
        0x68 => "DS3231 / MPU6050",
        0x6A | 0x6B => "QMI8658 IMU",
        _ => "Unknown Device",
    }
}

// ---------------------------------------------------------------------------
// Entry Point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== ESP32-S3 I2C Scanner (Waveshare 1.64\" AMOLED) ===");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize embassy via esp-rtos
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let delay = Delay::new();

    // ── 0. Hardware Reset Pulse for Peripherals ───────────────────────────
    info!("Pulsing hardware reset pins...");
    let mut lcd_rst_pin = Output::new(unsafe { peripherals.GPIO21.clone_unchecked() }, Level::Low, OutputConfig::default());

    delay.delay_millis(20);
    lcd_rst_pin.set_high();
    delay.delay_millis(100);

    let scan_range = 0x08u8..=0x77u8;

    let mut found_addrs = [0u8; 16];
    let mut found_count = 0usize;

    // ── 1. Scan Onboard I2C Bus (SDA: GPIO47, SCL: GPIO48) ───────────────
    info!("Scanning I2C0 (SDA: GPIO47, SCL: GPIO48)...");

    if let Ok(i2c_bus) = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    ) {
        let mut i2c = i2c_bus
            .with_sda(peripherals.GPIO47)
            .with_scl(peripherals.GPIO48);

        for addr in scan_range {
            let mut dummy = [0u8; 1];
            // Probe address using register write/read or direct read/write
            let is_found = i2c.write_read(addr, &[0x00], &mut dummy).is_ok()
                || i2c.write(addr, &[0x00]).is_ok()
                || i2c.read(addr, &mut dummy).is_ok();

            if is_found {
                let desc = get_device_desc(addr);
                info!("I2C -> FOUND 0x{:02X} ({})", addr, desc);
                if found_count < found_addrs.len() {
                    found_addrs[found_count] = addr;
                    found_count += 1;
                }
            }
        }
    } else {
        info!("Failed to initialize I2C bus on GPIO47/GPIO48");
    }

    info!("Scan Complete. Devices found: {}", found_count);

    // ── 2. Configure CO5300 QSPI Display to render results ───────────────
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

    // ── 3. Render scan UI ──────────────────────────────────────────────────
    fb_disp.clear(Rgb565::BLACK).unwrap();

    let title_style = MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::YELLOW);
    let section_style = MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::CYAN);
    let text_style = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);
    let found_style = MonoTextStyle::new(&FONT_7X13, Rgb565::GREEN);
    let none_style = MonoTextStyle::new(&FONT_7X13, Rgb565::RED);

    let mut y: i32 = 25;

    Text::new("I2C BUS SCANNER", Point::new(10, y), title_style)
        .draw(&mut fb_disp)
        .unwrap();
    y += 20;

    Text::new("Waveshare ESP32-S3 1.64\"", Point::new(10, y), text_style)
        .draw(&mut fb_disp)
        .unwrap();
    y += 25;

    Text::new("I2C: SDA:GP47 SCL:GP48", Point::new(10, y), section_style)
        .draw(&mut fb_disp)
        .unwrap();
    y += 20;

    if found_count == 0 {
        Text::new("  No devices found", Point::new(10, y), none_style)
            .draw(&mut fb_disp)
            .unwrap();
        y += 20;
    } else {
        for i in 0..found_count {
            let addr = found_addrs[i];
            let desc = get_device_desc(addr);
            let mut line_buf = [0u8; 36];
            let line_str = format_found_line(&mut line_buf, addr, desc);
            Text::new(line_str, Point::new(10, y), found_style)
                .draw(&mut fb_disp)
                .unwrap();
            y += 20;
        }
    }

    y += 30;
    Text::new("Scan Complete.", Point::new(10, y), title_style)
        .draw(&mut fb_disp)
        .unwrap();

    // ── 4. Flush Framebuffer to Display ───────────────────────────────────
    let chunk_size: u16 = 38;
    let total_lines: u16 = 456;

    for y_start in (0..total_lines).step_by(chunk_size as usize) {
        let y_end = y_start + chunk_size - 1;
        let is_first = y_start == 0;
        let is_last = y_end == total_lines - 1;

        let frame_ctrl = match (is_first, is_last) {
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

    info!("Display output updated.");

    let mut tick = 0u32;
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(5)).await;
        tick += 1;
        info!("Scanner active, tick {}...", tick);
    }
}

// ---------------------------------------------------------------------------
// Format `  0xXX: Description`
// ---------------------------------------------------------------------------

fn format_found_line<'a>(buf: &'a mut [u8; 36], addr: u8, desc: &str) -> &'a str {
    let hex_chars = b"0123456789ABCDEF";
    let prefix = b"  0x00: ";
    let prefix_len = prefix.len();

    buf[..prefix_len].copy_from_slice(prefix);
    buf[4] = hex_chars[((addr >> 4) & 0xF) as usize];
    buf[5] = hex_chars[(addr & 0xF) as usize];

    let desc_bytes = desc.as_bytes();
    let copy_len = desc_bytes.len().min(buf.len() - prefix_len);
    buf[prefix_len..prefix_len + copy_len].copy_from_slice(&desc_bytes[..copy_len]);

    let total_len = prefix_len + copy_len;
    core::str::from_utf8(&buf[..total_len]).unwrap_or("  0x??: Unknown")
}
