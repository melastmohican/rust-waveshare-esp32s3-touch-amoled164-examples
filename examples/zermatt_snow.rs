//! # Zermatt Image Falling Snow Example
//!
//! Displays a full-screen image of Zermatt on the Waveshare 1.64" AMOLED display with real-time
//! physics-animated falling snow.
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
//! 1. **Native Hardware Framebuffer (280×456)**: Preserves native 280×456 physical hardware
//!    dimensions so display controller QSPI windowing and DMA transfers run without memory corruption.
//! 2. **Landscape Physics Coordinate Mapping**: The physics grid operates on a 228 (width) × 140
//!    (height) cell layout. `render_flake` and `render_void` map landscape physics coordinates
//!    `(lx, ly)` to native panel coordinates `(nx, ny) = (279 - ly, lx)`, so snow falls vertically from
//!    top to bottom when the device is viewed in landscape mode.
//! 3. **2-Line Alignment Constraint**: The CO5300 controller requires `y_alignment = 2`.
//!    Flushing is performed in 12 38-line chunks (`0..=37`, `38..=75`, ..., `418..=455`) so every
//!    chunk boundary starts on an even line index and has an even height.
//! 4. **Platform-Independent PRNG**: Uses an atomic Linear Congruential Generator (`AtomicU32`)
//!    for lightweight, no_std random snowflake generation without platform-specific dependencies.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example zermatt_snow
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
    geometry::{Point, Size},
    image::{GetPixel, Image},
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
    primitives::Rectangle,
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
// Display & Physics Geometry
// ---------------------------------------------------------------------------

const WIDTH: usize = 280;
const HEIGHT: usize = 456;

// Physics grid in landscape orientation (456 wide top edge, 280 high vertical)
const PHY_DISP_RATIO: usize = 2; // Physical cell size in pixels (2x2)
const PHY_WIDTH: usize = 456 / PHY_DISP_RATIO; // 228 columns across 456px top edge
const PHY_HEIGHT: usize = 280 / PHY_DISP_RATIO; // 140 rows down 280px vertical height

const BITS_PER_CELL: usize = 1;
const CELLS_PER_BYTE: usize = 8 / BITS_PER_CELL;
const GRID_TOTAL_CELLS: usize = PHY_WIDTH * PHY_HEIGHT;
const GRID_SIZE_BYTES: usize = GRID_TOTAL_CELLS / CELLS_PER_BYTE;

const FLAKE_SIZE: i32 = 2; // Small 2x2 pixel snowflakes
const SNOW_COLOR: Rgb565 = Rgb565::WHITE;

type FbType =
    Framebuffer<Rgb565, RawU16, BigEndian, WIDTH, HEIGHT, { buffer_size::<Rgb565>(WIDTH, HEIGHT) }>;

// Simple atomic LCG PRNG
static RNG_STATE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(12345);

fn random_range(min: i32, max: i32) -> i32 {
    let mut state = RNG_STATE.load(core::sync::atomic::Ordering::Relaxed);
    state = state.wrapping_mul(1103515245).wrapping_add(12345);
    RNG_STATE.store(state, core::sync::atomic::Ordering::Relaxed);
    let random = (state / 65536) % 32768;
    min + (random as i32 % (max - min))
}

struct SnowGrid {
    grid: [u8; GRID_SIZE_BYTES],
}

impl SnowGrid {
    fn new() -> Self {
        Self {
            grid: [0u8; GRID_SIZE_BYTES],
        }
    }

    fn get_cell(&self, row: usize, col: usize) -> bool {
        let cell_index = row * PHY_WIDTH + col;
        let byte_index = cell_index / CELLS_PER_BYTE;
        let bit_index = cell_index % CELLS_PER_BYTE;
        (self.grid[byte_index] >> bit_index) & 1 == 1
    }

    fn set_cell(&mut self, row: usize, col: usize, value: bool) {
        let cell_index = row * PHY_WIDTH + col;
        let byte_index = cell_index / CELLS_PER_BYTE;
        let bit_index = cell_index % CELLS_PER_BYTE;

        if value {
            self.grid[byte_index] |= 1 << bit_index;
        } else {
            self.grid[byte_index] &= !(1 << bit_index);
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== Zermatt Falling Snow Display Example (Waveshare ESP32-S3-Touch-AMOLED-1.64) ===");

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
    let bmp_data = include_bytes!("zermatt_280x456.bmp");
    let bmp = Bmp::<Rgb565>::from_slice(bmp_data).expect("Failed to load BMP image");

    info!("Drawing initial Zermatt image to framebuffer...");
    fb_disp.clear(Rgb565::BLACK).unwrap();
    Image::new(&bmp, Point::new(0, 0))
        .draw(&mut fb_disp)
        .unwrap();

    let chunk_size: u16 = 38;
    let total_lines: u16 = 456;

    info!("Starting snow animation loop...");
    let mut snow_grid = SnowGrid::new();
    let mut frame_count = 0u32;

    loop {
        // Simulate falling snow (iterate from bottom row to top row of landscape view)
        for row in (0..PHY_HEIGHT - 1).rev() {
            for col in 0..PHY_WIDTH {
                if snow_grid.get_cell(row, col) {
                    let offset = random_range(-1, 2);
                    let future_col =
                        (col as i32 + offset).max(0).min(PHY_WIDTH as i32 - 1) as usize;

                    if !snow_grid.get_cell(row + 1, future_col) {
                        snow_grid.set_cell(row + 1, future_col, true);
                        render_flake(&mut fb_disp, row + 1, future_col);
                    }

                    snow_grid.set_cell(row, col, false);
                    render_void(&mut fb_disp, &bmp, row, col);
                }
            }
        }

        // Clear snowflakes that reached the bottom
        for col in 0..PHY_WIDTH {
            if snow_grid.get_cell(PHY_HEIGHT - 1, col) {
                snow_grid.set_cell(PHY_HEIGHT - 1, col, false);
                render_void(&mut fb_disp, &bmp, PHY_HEIGHT - 1, col);
            }
        }

        // Create new snow at top edge of landscape view
        for col in 0..PHY_WIDTH {
            if random_range(0, 20) < 1 {
                snow_grid.set_cell(0, col, true);
                render_flake(&mut fb_disp, 0, col);
            }
        }

        // Flush native framebuffer (280x456) to display in 12 38-line chunks
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

        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;

        frame_count += 1;
        if frame_count.is_multiple_of(50) {
            info!("Frame: {}", frame_count);
        }
    }
}

// Render a snowflake at landscape grid position (row, col) mapped to native panel
fn render_flake(target: &mut impl DrawTarget<Color = Rgb565>, row: usize, col: usize) {
    let lx = (col * PHY_DISP_RATIO) as i32; // 0..456 (horizontal in landscape)
    let ly = (row * PHY_DISP_RATIO) as i32; // 0..280 (vertical in landscape)

    // Map landscape (lx, ly) to native panel (nx, ny) where top-to-bottom falls in direction 279 -> 0
    let nx = 279 - ly;
    let ny = lx;

    let rect_x = nx - FLAKE_SIZE + 1;
    let rect_y = ny;

    let rect = Rectangle::new(
        Point::new(rect_x, rect_y),
        Size::new(FLAKE_SIZE as u32, FLAKE_SIZE as u32),
    );
    target.fill_solid(&rect, SNOW_COLOR).ok();
}

// Restore background image pixels at landscape grid position (row, col) mapped to native panel
fn render_void(
    target: &mut impl DrawTarget<Color = Rgb565>,
    bmp: &Bmp<Rgb565>,
    row: usize,
    col: usize,
) {
    let lx = (col * PHY_DISP_RATIO) as i32;
    let ly = (row * PHY_DISP_RATIO) as i32;

    let nx = 279 - ly;
    let ny = lx;

    let rect_x = nx - FLAKE_SIZE + 1;
    let rect_y = ny;

    let mut colors = [Rgb565::BLACK; 4];
    let mut idx = 0;

    for dy in 0..FLAKE_SIZE {
        for dx in 0..FLAKE_SIZE {
            let px = rect_x + dx;
            let py = rect_y + dy;
            colors[idx] = bmp.pixel(Point::new(px, py)).unwrap_or(Rgb565::BLACK);
            idx += 1;
        }
    }

    let rect = Rectangle::new(
        Point::new(rect_x, rect_y),
        Size::new(FLAKE_SIZE as u32, FLAKE_SIZE as u32),
    );
    target.fill_contiguous(&rect, colors).ok();
}
