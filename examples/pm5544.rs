//! # Philips PM5544 Test Pattern Demo for Waveshare ESP32-S3-Touch-AMOLED-1.64
//!
//! Displays the classic Philips PM5544 TV test pattern on the 280×456 CO5300 AMOLED display screen.
//!
//! - **Wikipedia Article:** https://en.wikipedia.org/wiki/Philips_circle_pattern
//! - **Display Controller:** CO5300 (280×456 native resolution, QSPI)
//! - **Documentation:** https://docs.waveshare.com/ESP32-S3-Touch-AMOLED-1.64
//!
//! ## Run
//!
//! ```bash
//! cargo run --example pm5544
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_println as _;
use esp_hal::{
    dma::DmaRxBuf,
    dma_buffers,
    gpio::{Level, Output, OutputConfig},
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
    image::Image,
    pixelcolor::{
        raw::{BigEndian, RawU16},
        Rgb565,
    },
    prelude::*,
};
use tinybmp::Bmp;

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
// Display Geometry & Types
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
// Entry Point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== PM5544 Test Pattern Example (Waveshare ESP32-S3 1.64\" AMOLED) ===");

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize embassy via esp-rtos
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // ── 1. Configure CO5300 QSPI AMOLED Display ───────────────────────────
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

    // Clear Screen to Black
    fb_disp.clear(Rgb565::BLACK).unwrap();

    // ── 2. Load and Draw PM5544 Test Pattern BMP ─────────────────────────
    info!("Loading PM5544 BMP image (280x456)...");
    let bmp_bytes = include_bytes!("pm5544.bmp");
    
    match Bmp::<Rgb565>::from_slice(bmp_bytes) {
        Ok(raw_image) => {
            info!("PM5544 BMP loaded successfully! Size: {}x{}", raw_image.size().width, raw_image.size().height);
            let image = Image::new(&raw_image, Point::new(0, 0));
            if let Err(e) = image.draw(&mut fb_disp) {
                info!("Failed to draw BMP image: {:?}", defmt::Debug2Format(&e));
            }
        }
        Err(e) => {
            info!("Error parsing PM5544 BMP slice: {:?}", defmt::Debug2Format(&e));
        }
    }

    // ── 3. Flush Framebuffer to Display ───────────────────────────────────
    info!("Flushing frame to AMOLED display screen...");
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

    info!("PM5544 test pattern displayed successfully!");

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}
