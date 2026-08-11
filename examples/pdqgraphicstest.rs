//! # PDQ Graphicstest Benchmark Example
//!
//! Benchmark graphics test suite for the Waveshare ESP32-S3-Touch-AMOLED-1.64
//! display using `embedded-graphics` and Embassy, inspired by Xark's classic
//! PDQ graphicstest sketch for Arduino.
//!
//! ## Hardware
//!
//! - **Board:** Waveshare ESP32-S3-Touch-AMOLED-1.64
//! - **Display Controller:** CO5300 (280×456 native resolution)
//! - **Documentation:** https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.64
//!
//! ## Run
//!
//! ```bash
//! cargo run --example pdqgraphicstest
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    dma::DmaRxBuf,
    dma_buffers,
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println as _;

use embedded_graphics::{
    draw_target::DrawTarget,
    framebuffer::{Framebuffer, buffer_size},
    geometry::{Point, Size},
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_7X13, FONT_9X18_BOLD},
    },
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
    primitives::{
        Circle, CornerRadii, Line, PrimitiveStyle, Rectangle, RoundedRectangle, Triangle,
    },
    text::{Alignment, Text},
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
// Panel Specification for Waveshare 1.64" AMOLED (280×456, CO5300)
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
// Display geometry & type aliases
// ---------------------------------------------------------------------------

const WIDTH: usize = 280;
const HEIGHT: usize = 456;

type FbType =
    Framebuffer<Rgb565, RawU16, BigEndian, WIDTH, HEIGHT, { buffer_size::<Rgb565>(WIDTH, HEIGHT) }>;

// ---------------------------------------------------------------------------
// Helper to flush framebuffer to display in 12 38-line DMA chunks
// ---------------------------------------------------------------------------

async fn flush_framebuffer<B, P, C, R, BO, const W: usize, const H: usize, const N: usize>(
    fb_disp: &mut FrameBufferedDisplayDriver<'_, B, P, C, R, BO, W, H, N>,
) where
    B: display_driver::DisplayBus,
    P: display_driver::Panel<B>,
    C: embedded_graphics::pixelcolor::PixelColor<Raw = R>,
    R: embedded_graphics::pixelcolor::raw::RawData,
    BO: embedded_graphics::pixelcolor::raw::ByteOrder,
{
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

        if let Err(err) = fb_disp
            .flush_lines_with_frame_control(y_start, y_end, frame_ctrl)
            .await
        {
            defmt::error!(
                "Failed to flush lines {}..={}: {}",
                y_start,
                y_end,
                defmt::Debug2Format(&err)
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Benchmark Test Functions
// ---------------------------------------------------------------------------

fn test_fill_screen<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::WHITE).unwrap();
    display.clear(Rgb565::RED).unwrap();
    display.clear(Rgb565::GREEN).unwrap();
    display.clear(Rgb565::BLUE).unwrap();
    display.clear(Rgb565::BLACK).unwrap();
    start.elapsed().as_micros() as u32
}

fn test_text<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();

    let style_white = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);
    let style_red = MonoTextStyle::new(&FONT_7X13, Rgb565::RED);
    let style_green = MonoTextStyle::new(&FONT_7X13, Rgb565::GREEN);
    let style_blue = MonoTextStyle::new(&FONT_7X13, Rgb565::BLUE);
    let style_yellow = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::YELLOW);
    let style_cyan = MonoTextStyle::new(&FONT_7X13, Rgb565::CYAN);
    let style_magenta = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::MAGENTA);

    Text::new("Hello World!", Point::new(10, 20), style_white)
        .draw(display)
        .unwrap();

    Text::new("RED ", Point::new(10, 40), style_red)
        .draw(display)
        .unwrap();
    Text::new("GREEN ", Point::new(65, 40), style_green)
        .draw(display)
        .unwrap();
    Text::new("BLUE", Point::new(140, 40), style_blue)
        .draw(display)
        .unwrap();

    Text::new("1234.56", Point::new(10, 65), style_yellow)
        .draw(display)
        .unwrap();
    Text::new("0xDEADBEEF", Point::new(10, 90), style_white)
        .draw(display)
        .unwrap();

    Text::new("Groop,", Point::new(10, 115), style_cyan)
        .draw(display)
        .unwrap();
    Text::new("I implore thee,", Point::new(10, 140), style_magenta)
        .draw(display)
        .unwrap();
    Text::new(
        "my foonting turlingdromes.",
        Point::new(10, 160),
        style_white,
    )
    .draw(display)
    .unwrap();
    Text::new(
        "And hooptiously drangle me",
        Point::new(10, 180),
        style_green,
    )
    .draw(display)
    .unwrap();
    Text::new(
        "with crinkly bindlewurdles,",
        Point::new(10, 200),
        style_cyan,
    )
    .draw(display)
    .unwrap();
    Text::new("Or I will rend thee", Point::new(10, 220), style_red)
        .draw(display)
        .unwrap();
    Text::new("in the gobberwartsb", Point::new(10, 240), style_magenta)
        .draw(display)
        .unwrap();
    Text::new(
        "with my blurglecruncheon,",
        Point::new(10, 260),
        style_yellow,
    )
    .draw(display)
    .unwrap();
    Text::new("see if I don't!", Point::new(10, 280), style_white)
        .draw(display)
        .unwrap();

    start.elapsed().as_micros() as u32
}

fn test_pixels<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    let w = 280i32;
    let h = 456i32;
    for y in 0..h {
        for x in 0..w {
            let r = (x & 0x1F) as u8;
            let g = (y & 0x3F) as u8;
            let b = ((x * y) & 0x1F) as u8;
            Pixel(Point::new(x, y), Rgb565::new(r, g, b))
                .draw(display)
                .unwrap();
        }
    }
    start.elapsed().as_micros() as u32
}

fn test_lines<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let style = PrimitiveStyle::with_stroke(Rgb565::BLUE, 1);

    let mut x2 = 0;
    while x2 < w {
        Line::new(Point::new(0, 0), Point::new(x2, h - 1))
            .into_styled(style)
            .draw(display)
            .unwrap();
        x2 += 6;
    }
    let mut y2 = 0;
    while y2 < h {
        Line::new(Point::new(0, 0), Point::new(w - 1, y2))
            .into_styled(style)
            .draw(display)
            .unwrap();
        y2 += 6;
    }

    x2 = 0;
    while x2 < w {
        Line::new(Point::new(w - 1, 0), Point::new(x2, h - 1))
            .into_styled(style)
            .draw(display)
            .unwrap();
        x2 += 6;
    }
    y2 = 0;
    while y2 < h {
        Line::new(Point::new(w - 1, 0), Point::new(0, y2))
            .into_styled(style)
            .draw(display)
            .unwrap();
        y2 += 6;
    }

    x2 = 0;
    while x2 < w {
        Line::new(Point::new(0, h - 1), Point::new(x2, 0))
            .into_styled(style)
            .draw(display)
            .unwrap();
        x2 += 6;
    }
    y2 = 0;
    while y2 < h {
        Line::new(Point::new(0, h - 1), Point::new(w - 1, y2))
            .into_styled(style)
            .draw(display)
            .unwrap();
        y2 += 6;
    }

    x2 = 0;
    while x2 < w {
        Line::new(Point::new(w - 1, h - 1), Point::new(x2, 0))
            .into_styled(style)
            .draw(display)
            .unwrap();
        x2 += 6;
    }
    y2 = 0;
    while y2 < h {
        Line::new(Point::new(w - 1, h - 1), Point::new(0, y2))
            .into_styled(style)
            .draw(display)
            .unwrap();
        y2 += 6;
    }

    start.elapsed().as_micros() as u32
}

fn test_fast_lines<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;

    let red_style = PrimitiveStyle::with_stroke(Rgb565::RED, 1);
    let blue_style = PrimitiveStyle::with_stroke(Rgb565::BLUE, 1);

    let mut y = 0;
    while y < h {
        Line::new(Point::new(0, y), Point::new(w - 1, y))
            .into_styled(red_style)
            .draw(display)
            .unwrap();
        y += 5;
    }

    let mut x = 0;
    while x < w {
        Line::new(Point::new(x, 0), Point::new(x, h - 1))
            .into_styled(blue_style)
            .draw(display)
            .unwrap();
        x += 5;
    }

    start.elapsed().as_micros() as u32
}

fn test_filled_rects<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let n = w.min(h);
    let cx = w / 2;
    let cy = h / 2;

    let mut i = n;
    while i > 0 {
        let i2 = i / 2;
        let color = Rgb565::new((i & 0x1F) as u8, (i & 0x3F) as u8, 0);
        Rectangle::new(Point::new(cx - i2, cy - i2), Size::new(i as u32, i as u32))
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(display)
            .unwrap();
        i -= 6;
    }

    start.elapsed().as_micros() as u32
}

fn test_rects<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let n = w.min(h);
    let cx = w / 2;
    let cy = h / 2;

    let style = PrimitiveStyle::with_stroke(Rgb565::GREEN, 1);
    let mut i = 2;
    while i < n {
        let i2 = i / 2;
        Rectangle::new(Point::new(cx - i2, cy - i2), Size::new(i as u32, i as u32))
            .into_styled(style)
            .draw(display)
            .unwrap();
        i += 6;
    }

    start.elapsed().as_micros() as u32
}

fn test_filled_triangles<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let cx1 = w / 2 - 1;
    let cy1 = h / 2 - 1;
    let cn = (cx1 - 1).min(cy1 - 1).min(85);
    let cn1 = cn - 1;

    let mut i = cn1;
    while i > 10 {
        let color = Rgb565::new(0, (i & 0x3F) as u8, (i & 0x1F) as u8);
        Triangle::new(
            Point::new(cx1, cy1 - i),
            Point::new(cx1 - i, cy1 + i),
            Point::new(cx1 + i, cy1 + i),
        )
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display)
        .unwrap();
        i -= 5;
    }

    start.elapsed().as_micros() as u32
}

fn test_triangles<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let cx1 = w / 2 - 1;
    let cy1 = h / 2 - 1;
    let cn = (cx1 - 1).min(cy1 - 1).min(85);

    let mut i = 0;
    while i < cn {
        let color = Rgb565::new(0, 0, (i & 0x1F) as u8);
        Triangle::new(
            Point::new(cx1, cy1 - i),
            Point::new(cx1 - i, cy1 + i),
            Point::new(cx1 + i, cy1 + i),
        )
        .into_styled(PrimitiveStyle::with_stroke(color, 1))
        .draw(display)
        .unwrap();
        i += 5;
    }

    start.elapsed().as_micros() as u32
}

fn test_filled_circles<D>(display: &mut D, radius: u32) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let r2 = (radius * 2) as i32;

    let style = PrimitiveStyle::with_fill(Rgb565::MAGENTA);
    let mut x = radius as i32;
    while x < w {
        let mut y = radius as i32;
        while y < h {
            Circle::new(Point::new(x - radius as i32, y - radius as i32), radius * 2)
                .into_styled(style)
                .draw(display)
                .unwrap();
            y += r2;
        }
        x += r2;
    }

    start.elapsed().as_micros() as u32
}

fn test_circles<D>(display: &mut D, radius: u32) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    let w = 280i32;
    let h = 456i32;
    let r2 = (radius * 2) as i32;
    let w1 = w + radius as i32;
    let h1 = h + radius as i32;

    let style = PrimitiveStyle::with_stroke(Rgb565::WHITE, 1);
    let mut x = 0i32;
    while x < w1 {
        let mut y = 0i32;
        while y < h1 {
            Circle::new(Point::new(x - radius as i32, y - radius as i32), radius * 2)
                .into_styled(style)
                .draw(display)
                .unwrap();
            y += r2;
        }
        x += r2;
    }

    start.elapsed().as_micros() as u32
}

fn test_filled_round_rects<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let n1 = w.min(h) - 1;
    let cx = w / 2;
    let cy = h / 2;

    let mut i = n1;
    while i > 20 {
        let i2 = i / 2;
        let corner = (i / 8) as u32;
        let color = Rgb565::new(0, (i & 0x3F) as u8, 0);
        RoundedRectangle::new(
            Rectangle::new(Point::new(cx - i2, cy - i2), Size::new(i as u32, i as u32)),
            CornerRadii::new(Size::new(corner, corner)),
        )
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display)
        .unwrap();
        i -= 6;
    }

    start.elapsed().as_micros() as u32
}

fn test_round_rects<D>(display: &mut D) -> u32
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let start = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let w = 280i32;
    let h = 456i32;
    let n1 = w.min(h) - 1;
    let cx = w / 2;
    let cy = h / 2;

    let mut i = 20;
    while i < n1 {
        let i2 = i / 2;
        let corner = (i / 8) as u32;
        let color = Rgb565::new((i & 0x1F) as u8, 0, 0);
        RoundedRectangle::new(
            Rectangle::new(Point::new(cx - i2, cy - i2), Size::new(i as u32, i as u32)),
            CornerRadii::new(Size::new(corner, corner)),
        )
        .into_styled(PrimitiveStyle::with_stroke(color, 1))
        .draw(display)
        .unwrap();
        i += 6;
    }

    start.elapsed().as_micros() as u32
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== PDQ Graphicstest Benchmark (Waveshare ESP32-S3-Touch-AMOLED-1.64) ===");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize embassy via esp-rtos
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // ── Pin assignments for Waveshare ESP32-S3-Touch-AMOLED-1.64 ──────────
    let sclk = peripherals.GPIO10;
    let sio0 = peripherals.GPIO11; // D0
    let sio1 = peripherals.GPIO12; // D1
    let sio2 = peripherals.GPIO13; // D2
    let sio3 = peripherals.GPIO14; // D3
    let rst = peripherals.GPIO21;
    let cs = peripherals.GPIO9; // Rev V1 PCB (change to GPIO46 if Rev V2)

    // ── SPI + DMA (QSPI 1-1-4) ────────────────────────────────────────────
    info!("Configuring SPI2 + DMA (QSPI 1-1-4 mode, 10 MHz)...");

    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(256, 0);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    static TX_DESCRIPTORS: static_cell::StaticCell<[esp_hal::dma::DmaDescriptor; 8]> =
        static_cell::StaticCell::new();
    let tx_descriptors = TX_DESCRIPTORS.init([esp_hal::dma::DmaDescriptor::EMPTY; 8]);

    static BOUNCE_BUF: static_cell::StaticCell<[u8; 256]> = static_cell::StaticCell::new();
    let bounce_buf = BOUNCE_BUF.init([0; 256]);

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
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

    // ── Display bus & panel ────────────────────────────────────────────────
    let device = EspHalQspiDevice {
        spi: Some(spi),
        rx_buf: Some(dma_rx_buf),
        tx_descriptors: Some(tx_descriptors),
        bounce_buf: Some(bounce_buf),
    };

    let bus = QspiDisplayBus::new(device, QspiConfig::default());
    let rst_pin = Output::new(rst, Level::High, OutputConfig::default());
    let panel = Co5300::<WaveshareAmoled164, _, _>::new(LCDResetOption::new_pin(rst_pin));

    // ── Static Framebuffer (280×456 RGB565) ──────────────────────────────
    static mut FB_DATA: core::mem::MaybeUninit<FbType> = core::mem::MaybeUninit::uninit();
    let fb = unsafe {
        let ptr = core::ptr::addr_of_mut!(FB_DATA) as *mut FbType;
        core::ptr::write_bytes(ptr, 0, 1);
        &mut *ptr
    };

    // ── Initialize Display Driver ─────────────────────────────────────────
    let disp = DisplayDriver::builder(bus, panel)
        .with_color_format(ColorFormat::RGB565)
        .init(&mut embassy_time::Delay)
        .await
        .unwrap();
    info!("Display initialized.");

    let mut fb_disp = FrameBufferedDisplayDriver::new(disp, fb);
    fb_disp.set_brightness(200).await.unwrap();

    // ── Execute Benchmark Suite ───────────────────────────────────────────
    info!("Starting PDQ Graphicstest benchmark suite...");

    // Helper closure to run test, log metric, flush screen, and delay for visibility
    macro_rules! run_bench {
        ($name:expr, $expr:expr, $fb:expr) => {{
            let duration_us = $expr;
            info!("{}:\t{} us", $name, duration_us);
            flush_framebuffer($fb).await;
            Timer::after(Duration::from_millis(150)).await;
            duration_us
        }};
    }

    let us_fill = run_bench!("Screen fill", test_fill_screen(&mut fb_disp), &mut fb_disp);
    let us_text = run_bench!("Text", test_text(&mut fb_disp), &mut fb_disp);
    let us_pixels = run_bench!("Pixels", test_pixels(&mut fb_disp), &mut fb_disp);
    let us_lines = run_bench!("Lines", test_lines(&mut fb_disp), &mut fb_disp);
    let us_fast_lines = run_bench!(
        "Horiz/Vert Lines",
        test_fast_lines(&mut fb_disp),
        &mut fb_disp
    );
    let us_filled_rects = run_bench!(
        "Rectangles (filled)",
        test_filled_rects(&mut fb_disp),
        &mut fb_disp
    );
    let us_rects = run_bench!(
        "Rectangles (outline)",
        test_rects(&mut fb_disp),
        &mut fb_disp
    );
    let us_filled_triangles = run_bench!(
        "Triangles (filled)",
        test_filled_triangles(&mut fb_disp),
        &mut fb_disp
    );
    let us_triangles = run_bench!(
        "Triangles (outline)",
        test_triangles(&mut fb_disp),
        &mut fb_disp
    );
    let us_filled_circles = run_bench!(
        "Circles (filled)",
        test_filled_circles(&mut fb_disp, 10),
        &mut fb_disp
    );
    let us_circles = run_bench!(
        "Circles (outline)",
        test_circles(&mut fb_disp, 10),
        &mut fb_disp
    );
    let us_filled_round_rects = run_bench!(
        "RoundRects (filled)",
        test_filled_round_rects(&mut fb_disp),
        &mut fb_disp
    );
    let us_round_rects = run_bench!(
        "RoundRects (outline)",
        test_round_rects(&mut fb_disp),
        &mut fb_disp
    );

    // Measure single DMA flush speed
    let flush_start = Instant::now();
    flush_framebuffer(&mut fb_disp).await;
    let us_flush = flush_start.elapsed().as_micros() as u32;
    info!("Display Flush (DMA):\t{} us", us_flush);

    info!("=== Benchmark Complete! ===");

    // ── Render Benchmark Results Scorecard on AMOLED Display ──────────────
    fb_disp.clear(Rgb565::BLACK).unwrap();

    // Outer border
    Rectangle::new(Point::new(0, 0), Size::new(280, 456))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::CYAN, 2))
        .draw(&mut fb_disp)
        .unwrap();

    // Title banner
    let title_style = MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::MAGENTA);
    Text::with_alignment(
        "Rust GFX PDQ",
        Point::new(140, 22),
        title_style,
        Alignment::Center,
    )
    .draw(&mut fb_disp)
    .unwrap();

    let sub_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
    Text::with_alignment(
        "Benchmark (micro-secs)",
        Point::new(140, 36),
        sub_style,
        Alignment::Center,
    )
    .draw(&mut fb_disp)
    .unwrap();

    // Results table
    let label_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CYAN);
    let value_style = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);

    let results: [(&str, u32); 14] = [
        ("Screen fill", us_fill),
        ("Text", us_text),
        ("Pixels", us_pixels),
        ("Lines", us_lines),
        ("H/V Lines", us_fast_lines),
        ("Rectangles F", us_filled_rects),
        ("Rectangles", us_rects),
        ("Triangles F", us_filled_triangles),
        ("Triangles", us_triangles),
        ("Circles F", us_filled_circles),
        ("Circles", us_circles),
        ("RoundRects F", us_filled_round_rects),
        ("RoundRects", us_round_rects),
        ("Display Flush", us_flush),
    ];

    let mut y_offset = 55;
    for (name, us) in results.iter() {
        Text::new(name, Point::new(15, y_offset), label_style)
            .draw(&mut fb_disp)
            .unwrap();

        // Right-aligned formatted metric string
        let mut buf = [0u8; 16];
        let val_str = format_num(*us, &mut buf);
        Text::with_alignment(
            val_str,
            Point::new(265, y_offset),
            value_style,
            Alignment::Right,
        )
        .draw(&mut fb_disp)
        .unwrap();

        y_offset += 24;
    }

    let footer_style = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);
    Text::with_alignment(
        "Complete!",
        Point::new(140, 442),
        footer_style,
        Alignment::Center,
    )
    .draw(&mut fb_disp)
    .unwrap();

    flush_framebuffer(&mut fb_disp).await;
    info!("Scorecard displayed on screen.");

    #[allow(clippy::empty_loop)]
    loop {}
}

// ---------------------------------------------------------------------------
// Helper function to format u32 into decimal string without std
// ---------------------------------------------------------------------------

fn format_num(mut val: u32, buf: &mut [u8; 16]) -> &str {
    if val == 0 {
        buf[0] = b'0';
        return core::str::from_utf8(&buf[..1]).unwrap();
    }
    let mut idx = 16;
    while val > 0 && idx > 0 {
        idx -= 1;
        buf[idx] = b'0' + (val % 10) as u8;
        val /= 10;
    }
    core::str::from_utf8(&buf[idx..16]).unwrap()
}
