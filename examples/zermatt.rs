//! # Zermatt Image Display Example
//!
//! Displays a full-screen BMP image of Zermatt on the Waveshare 1.64" AMOLED display using `tinybmp`.
//!
//! ## Hardware
//!
//! - **Board:** Waveshare ESP32-S3-Touch-AMOLED-1.64
//! - **Display Controller:** CO5300 (280×456 native resolution)
//! - **Documentation:** https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.64
//!
//! ## Onboard Wiring / Pinout
//!
//! ```text
//!   +-------------------------------------------------+
//!   | ESP32-S3 Pin  <--->  CO5300 AMOLED Display      |
//!   +-------------------------------------------------+
//!   | GPIO10        <--->  QSPI SCLK (Clock)          |
//!   | GPIO11        <--->  QSPI D0 / SIO0 (Data 0)    |
//!   | GPIO12        <--->  QSPI D1 / SIO1 (Data 1)    |
//!   | GPIO13        <--->  QSPI D2 / SIO2 (Data 2)    |
//!   | GPIO14        <--->  QSPI D3 / SIO3 (Data 3)    |
//!   | GPIO9 (V1)    <--->  QSPI CS (Chip Select)      |
//!   | GPIO46 (V2)   <--->  QSPI CS (Chip Select)      |
//!   | GPIO21        <--->  LCD RST (Reset)            |
//!   +-------------------------------------------------+
//! ```
//!
//! ## Key Design Decisions
//!
//! 1. **Native Hardware Resolution (280×456)**: The CO5300 display hardware requires a 280×456
//!    native framebuffer size to prevent RAM wrap-around memory corruption across QSPI DMA transfers.
//! 2. **BMP Parsing with `tinybmp`**: Loads `zermatt_280x456.bmp` via `include_bytes!` zero-copy
//!    embedding into flash memory.
//! 3. **2-Line Alignment Constraint**: The CO5300 controller requires `y_alignment = 2`.
//!    Flushing is performed in 12 38-line chunks (`0..=37`, `38..=75`, ..., `418..=455`) so every
//!    chunk boundary starts on an even line index and has an even height.
//! 4. **DMA Payload Limits**: Each 38-line chunk payload is 21,280 bytes (`38 × 280 × 2`), fitting
//!    safely inside the static DMA descriptor capacity (32,736 bytes).
//!
//! ## Run
//!
//! ```bash
//! cargo run --example zermatt
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
    framebuffer::{Framebuffer, buffer_size},
    geometry::Point,
    image::Image,
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
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
use tinybmp::Bmp;

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
// Display geometry (280×456 native)
// ---------------------------------------------------------------------------

const WIDTH: usize = 280;
const HEIGHT: usize = 456;

type FbType =
    Framebuffer<Rgb565, RawU16, BigEndian, WIDTH, HEIGHT, { buffer_size::<Rgb565>(WIDTH, HEIGHT) }>;

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== Zermatt Image Display Example (Waveshare ESP32-S3-Touch-AMOLED-1.64) ===");

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
    let cs = peripherals.GPIO9; // Note: change to GPIO46 if using PCB Rev V2

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

    // ── Framebuffer ────────────────────────────────────────────────────────
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
    info!("Display initialised.");

    let mut fb_disp = FrameBufferedDisplayDriver::new(disp, fb);

    fb_disp.set_brightness(200).await.unwrap();
    info!("Brightness set to 200.");

    info!("Loading Zermatt BMP image (280x456)...");
    let bmp = Bmp::<Rgb565>::from_slice(include_bytes!("zermatt_280x456.bmp"))
        .expect("Failed to load BMP image");

    info!("Drawing Zermatt image to framebuffer...");
    fb_disp.clear(Rgb565::BLACK).unwrap();
    Image::new(&bmp, Point::new(0, 0))
        .draw(&mut fb_disp)
        .unwrap();

    // Flush framebuffer → display in 12 38-line chunks (456 lines total)
    info!("Flushing framebuffer to display...");

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

    info!("Zermatt image displayed! Entering idle loop.");
    let mut ticks = 0;
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
        ticks += 1;
        info!("Tick {}...", ticks);
    }
}
