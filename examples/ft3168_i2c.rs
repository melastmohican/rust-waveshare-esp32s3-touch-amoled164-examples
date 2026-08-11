//! # FT3168 Touch Controller Example for Waveshare ESP32-S3-Touch-AMOLED-1.64
//!
//! Uses the `ft3x68-rs` driver crate to interface with the onboard FocalTech FT3168 IC
//! over I2C (SDA: GPIO47, SCL: GPIO48, Reset: GPIO8).
//! Logs touch point coordinates `(X, Y)` to the console via `defmt::info!`
//! and draws interactive visual touch feedback on the CO5300 AMOLED screen.
//!
//! ## Hardware Connections (Waveshare ESP32-S3-Touch-AMOLED-1.64)
//!
//! - **I2C SDA:** GPIO 47
//! - **I2C SCL:** GPIO 48
//! - **Touch Reset:** GPIO 8
//! - **Touch IC Address:** 0x38 (FT3168)
//! - **Display Controller:** CO5300 (280×456 native resolution, QSPI)
//! - **Documentation:** https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.64
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ft3168_i2c
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
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
use ft3x68_rs::{FT3168_DEVICE_ADDRESS, Ft3x68Driver, ResetInterface};

use embedded_graphics::{
    framebuffer::{Framebuffer, buffer_size},
    geometry::Point,
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_7X13, FONT_9X15_BOLD},
    },
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
    primitives::{Circle, PrimitiveStyle},
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
// Reset Driver Implementation for ft3x68-rs using ESP32-S3 GPIO8
// ---------------------------------------------------------------------------

pub struct GpioReset<'a> {
    pin: Output<'a>,
}

impl<'a> GpioReset<'a> {
    pub fn new(pin: Output<'a>) -> Self {
        Self { pin }
    }
}

impl<'a> ResetInterface for GpioReset<'a> {
    type Error = core::convert::Infallible;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();
        self.pin.set_low();
        delay.delay_millis(20);
        self.pin.set_high();
        delay.delay_millis(100);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Display Geometry & Types
// ---------------------------------------------------------------------------

const WIDTH: usize = 280;
const HEIGHT: usize = 456;

type FbType =
    Framebuffer<Rgb565, RawU16, BigEndian, WIDTH, HEIGHT, { buffer_size::<Rgb565>(WIDTH, HEIGHT) }>;

#[derive(Debug, Clone, Copy)]
pub struct TouchData {
    pub x: u16,
    pub y: u16,
}

// ---------------------------------------------------------------------------
// Entry Point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== FT3168 Touch Example with `ft3x68-rs` (Waveshare ESP32-S3 1.64\" AMOLED) ===");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize embassy via esp-rtos
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let delay = Delay::new();

    // ── 1. Create GPIO8 Reset Driver ──────────────────────────────────────
    info!("Configuring GPIO8 hardware reset driver...");
    let touch_rst_pin = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    let reset = GpioReset::new(touch_rst_pin);

    // ── 2. Initialize I2C Bus on SDA: GPIO47, SCL: GPIO48 ─────────────────
    info!("Initializing I2C0 bus on SDA: GPIO47, SCL: GPIO48 (400 kHz)...");
    let i2c_bus = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("Failed to create I2C controller");

    let i2c = i2c_bus
        .with_sda(peripherals.GPIO47)
        .with_scl(peripherals.GPIO48);

    // ── 3. Instantiate Ft3x68Driver ────────────────────────────────────────
    info!("Instantiating `ft3x68-rs` touch driver...");
    let mut touch_driver = Ft3x68Driver::new(i2c, FT3168_DEVICE_ADDRESS, reset, delay);

    if let Err(e) = touch_driver.initialize() {
        info!(
            "Warning initializing ft3x68-rs driver: {:?}",
            defmt::Debug2Format(&e)
        );
    }

    if let Ok(dev_id) = touch_driver.read_device_id() {
        info!("`ft3x68-rs` -> Device ID: 0x{:02X}", dev_id);
    }

    // ── 4. Configure CO5300 QSPI AMOLED Display ───────────────────────────
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
    let text_style = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);
    let active_style = MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::GREEN);

    let chunk_size: u16 = 38;
    let total_lines: u16 = 456;

    // Initial Screen Draw
    render_ui(
        &mut fb_disp,
        None,
        &title_style,
        &sub_style,
        &text_style,
        &active_style,
    );

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

    info!("Touch monitoring active using `ft3x68-rs`!");

    let mut last_touch: Option<TouchData> = None;
    let mut poll_count = 0u32;

    // ── 5. Main Touch Loop using `ft3x68-rs` ──────────────────────────────
    loop {
        let current_touch = match touch_driver.touch1() {
            Ok(ft3x68_rs::TouchState::Pressed(point)) => Some(TouchData {
                x: point.x,
                y: point.y,
            }),
            _ => None,
        };

        // If touch state changed, update UI and log
        let should_redraw = match (last_touch, current_touch) {
            (None, Some(t)) => {
                info!("Touch Down -> X: {}, Y: {}", t.x, t.y);
                true
            }
            (Some(prev), Some(t)) => {
                let moved = (prev.x as i32 - t.x as i32).abs() > 2
                    || (prev.y as i32 - t.y as i32).abs() > 2;
                if moved {
                    info!("Touch Move -> X: {}, Y: {}", t.x, t.y);
                }
                moved
            }
            (Some(_), None) => {
                info!("Touch Lifted");
                true
            }
            (None, None) => false,
        };

        if should_redraw {
            render_ui(
                &mut fb_disp,
                current_touch,
                &title_style,
                &sub_style,
                &text_style,
                &active_style,
            );

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

            last_touch = current_touch;
        }

        poll_count += 1;
        if poll_count.is_multiple_of(100) {
            info!("Touch loop active (poll count: {})...", poll_count);
        }

        embassy_time::Timer::after(embassy_time::Duration::from_millis(30)).await;
    }
}

// ---------------------------------------------------------------------------
// UI Drawing Helpers
// ---------------------------------------------------------------------------

fn render_ui<D>(
    target: &mut D,
    touch: Option<TouchData>,
    title_style: &MonoTextStyle<Rgb565>,
    sub_style: &MonoTextStyle<Rgb565>,
    text_style: &MonoTextStyle<Rgb565>,
    active_style: &MonoTextStyle<Rgb565>,
) where
    D: DrawTarget<Color = Rgb565>,
{
    target.clear(Rgb565::BLACK).ok();

    // Title
    Text::new("ft3x68-rs TOUCH DEMO", Point::new(10, 25), *title_style)
        .draw(target)
        .ok();

    Text::new("Waveshare ESP32-S3 1.64\"", Point::new(10, 45), *sub_style)
        .draw(target)
        .ok();

    Text::new("I2C: SDA:GP47, SCL:GP48", Point::new(10, 65), *text_style)
        .draw(target)
        .ok();

    // Draw status / touch info
    if let Some(t) = touch {
        let mut buf_x = [0u8; 32];
        let mut buf_y = [0u8; 32];

        let str_x = format_num(&mut buf_x, "  X Coord: ", t.x);
        let str_y = format_num(&mut buf_y, "  Y Coord: ", t.y);

        Text::new(
            "STATUS: TOUCH DETECTED!",
            Point::new(10, 100),
            *active_style,
        )
        .draw(target)
        .ok();

        Text::new(str_x, Point::new(10, 125), *text_style)
            .draw(target)
            .ok();

        Text::new(str_y, Point::new(10, 145), *text_style)
            .draw(target)
            .ok();

        // Draw touch marker at (X, Y) constrained to screen size (280x456)
        let cx = (t.x as i32).clamp(10, 270);
        let cy = (t.y as i32).clamp(10, 446);

        Circle::new(Point::new(cx - 15, cy - 15), 30)
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3))
            .draw(target)
            .ok();

        Circle::new(Point::new(cx - 4, cy - 4), 8)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
            .draw(target)
            .ok();
    } else {
        Text::new("STATUS: WAITING FOR TOUCH", Point::new(10, 100), *sub_style)
            .draw(target)
            .ok();

        Text::new("Touch panel surface to", Point::new(10, 130), *text_style)
            .draw(target)
            .ok();

        Text::new(
            "see live X/Y coordinates.",
            Point::new(10, 150),
            *text_style,
        )
        .draw(target)
        .ok();
    }
}

fn format_num<'a>(buf: &'a mut [u8; 32], prefix: &'a str, num: u16) -> &'a str {
    let p_bytes = prefix.as_bytes();
    let p_len = p_bytes.len();
    buf[..p_len].copy_from_slice(p_bytes);

    let mut n = num;
    let mut digits = [0u8; 5];
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

    for i in 0..d_count {
        buf[p_len + i] = digits[d_count - 1 - i];
    }

    let total = p_len + d_count;
    core::str::from_utf8(&buf[..total]).unwrap_or(prefix)
}
