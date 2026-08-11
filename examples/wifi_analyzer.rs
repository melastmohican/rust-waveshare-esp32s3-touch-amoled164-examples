//! # ESP32 Wi-Fi Analyzer Example
//!
//! Visualizes a 2.4 GHz Wi-Fi channel spectrum graph on the Waveshare
//! ESP32-S3-Touch-AMOLED-1.64 display using `embedded-graphics` and Embassy,
//! inspired by `ESPWiFiAnalyzer.ino`.
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
//! cargo run --example wifi_analyzer
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
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
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_7X13},
    },
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
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
// Wi-Fi Access Point Data Structure
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
pub struct WifiAccessPoint<'a> {
    pub ssid: &'a str,
    pub channel: u8, // 1..=13
    pub rssi: i32,   // -100 dBm to -30 dBm
    pub is_open: bool,
}

// Channel colors mapping for channels 1..=13
const CHANNEL_COLORS: [Rgb565; 13] = [
    Rgb565::RED,
    Rgb565::new(31, 35, 0), // Orange
    Rgb565::YELLOW,
    Rgb565::GREEN,
    Rgb565::CYAN,
    Rgb565::BLUE,
    Rgb565::MAGENTA,
    Rgb565::RED,
    Rgb565::new(31, 35, 0),
    Rgb565::YELLOW,
    Rgb565::GREEN,
    Rgb565::CYAN,
    Rgb565::BLUE,
];

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
// Helper function to render Wi-Fi Spectrum Graph onto Framebuffer
// ---------------------------------------------------------------------------

fn render_wifi_analyzer<D>(display: &mut D, ap_list: &[WifiAccessPoint], scan_num: u32)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    display.clear(Rgb565::BLACK).unwrap();

    let w = 280i32;

    // ── 1. Top Header Banner ────────────────────────────────────────────────
    Rectangle::new(Point::new(0, 0), Size::new(w as u32, 18))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::new(15, 0, 15)))
        .draw(display)
        .unwrap();

    // Segmented banner title text
    let title_style = MonoTextStyle::new(&FONT_7X13, Rgb565::WHITE);

    Text::new("ESP32 ", Point::new(10, 13), title_style)
        .draw(display)
        .unwrap();

    let wifi_style = MonoTextStyle::new(&FONT_7X13, Rgb565::CYAN);
    Text::new("WiFi ", Point::new(60, 13), wifi_style)
        .draw(display)
        .unwrap();

    let analyzer_style = MonoTextStyle::new(&FONT_7X13, Rgb565::YELLOW);
    Text::new("Analyzer", Point::new(100, 13), analyzer_style)
        .draw(display)
        .unwrap();

    // Scan cycle indicator
    let mut scan_buf = [0u8; 16];
    let scan_str = format_u32(scan_num, &mut scan_buf);
    let scan_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    Text::with_alignment(
        scan_str,
        Point::new(w - 10, 12),
        scan_style,
        Alignment::Right,
    )
    .draw(display)
    .unwrap();

    // ── 2. Channel Noise & Best Channel Calculation ─────────────────────────
    let mut ap_counts = [0u8; 13];
    let mut channel_noise = [0i32; 13];
    let mut peak_rssi = [-100i32; 13];
    let mut peak_ap_idx = [-1i8; 13];

    for (i, ap) in ap_list.iter().enumerate() {
        if (1..=13).contains(&ap.channel) {
            let ch_idx = (ap.channel - 1) as usize;
            ap_counts[ch_idx] += 1;

            if ap.rssi > peak_rssi[ch_idx] {
                peak_rssi[ch_idx] = ap.rssi;
                peak_ap_idx[ch_idx] = i as i8;
            }

            // Calculate channel overlap noise across adjacent channels
            let noise_val = (ap.rssi + 100).max(1);
            let noise_sq = noise_val * noise_val;

            for offset in -3i32..=3i32 {
                let target_ch = (ap.channel as i32) + offset;
                if (1..=13).contains(&target_ch) {
                    channel_noise[(target_ch - 1) as usize] += noise_sq / (offset.abs() + 1);
                }
            }
        }
    }

    // Find channel with lowest noise among channels 1..=11
    let mut min_noise = i32::MAX;
    for &n in &channel_noise[0..11] {
        if n < min_noise {
            min_noise = n;
        }
    }

    // Sub-header text: Networks found & recommended channels
    let info_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let mut num_buf = [0u8; 16];
    let count_str = format_u32(ap_list.len() as u32, &mut num_buf);
    Text::new(count_str, Point::new(10, 32), info_style)
        .draw(display)
        .unwrap();
    Text::new(" APs found. Best Ch:", Point::new(28, 32), info_style)
        .draw(display)
        .unwrap();

    let mut best_ch_x = 160i32;
    let rec_style = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);
    for ch in 1u8..=11u8 {
        if channel_noise[(ch - 1) as usize] == min_noise && best_ch_x < w - 20 {
            let mut ch_buf = [0u8; 16];
            let ch_str = format_u32(ch as u32, &mut ch_buf);
            Text::new(ch_str, Point::new(best_ch_x, 32), rec_style)
                .draw(display)
                .unwrap();
            best_ch_x += 18;
        }
    }

    // ── 3. Spectrum Graph Parameters ────────────────────────────────────────
    let graph_baseline = 415i32;
    let graph_height = 360i32;
    let channel_step_x = (w - 20) / 14; // ~18 px spacing per channel

    // ── 4. Plot AP Parabolic Spectrum Curves ────────────────────────────────
    for (i, ap) in ap_list.iter().enumerate() {
        if !(1..=13).contains(&ap.channel) {
            continue;
        }
        let ch_idx = (ap.channel - 1) as usize;
        let color = CHANNEL_COLORS[ch_idx];

        // Map RSSI (-100 dBm to -30 dBm) to curve peak height (10..340 px)
        let rssi_clamped = ap.rssi.clamp(-100, -30);
        let h_signal = (rssi_clamped + 100) * graph_height / 70;
        let center_x = 10 + (ap.channel as i32) * channel_step_x;
        let half_width = channel_step_x * 2; // Signal width spans 2 channels wide

        let mut prev_pt: Option<Point> = None;
        let step = 2i32;

        for dx in (-half_width)..=half_width {
            if dx % step != 0 && dx != half_width {
                continue;
            }
            let curr_x = center_x + dx;
            if curr_x < 0 || curr_x >= w {
                continue;
            }

            // Parabola formula: y = baseline - H * (1 - (dx / half_width)^2)
            let ratio_sq = (dx * dx * 1000) / (half_width * half_width);
            let y_offset = h_signal * (1000 - ratio_sq) / 1000;
            let curr_y = (graph_baseline - y_offset).clamp(40, graph_baseline);

            let curr_pt = Point::new(curr_x, curr_y);
            if let Some(prev) = prev_pt {
                Line::new(prev, curr_pt)
                    .into_styled(PrimitiveStyle::with_stroke(color, 1))
                    .draw(display)
                    .unwrap();
            }
            prev_pt = Some(curr_pt);
        }

        // Draw Peak SSID & RSSI Label for the strongest AP on each channel
        if peak_ap_idx[ch_idx] == i as i8 {
            let label_y = (graph_baseline - h_signal - 12).clamp(45, graph_baseline - 20);
            let label_x = (center_x - 30).clamp(5, w - 85);

            let label_style = MonoTextStyle::new(&FONT_6X10, color);

            // SSID string
            Text::new(ap.ssid, Point::new(label_x, label_y), label_style)
                .draw(display)
                .unwrap();

            // RSSI string + Open indicator
            let mut rssi_buf = [0u8; 16];
            let rssi_str = format_i32(ap.rssi, &mut rssi_buf);
            let rssi_y = label_y + 10;
            if rssi_y < graph_baseline - 5 {
                Text::new(rssi_str, Point::new(label_x, rssi_y), label_style)
                    .draw(display)
                    .unwrap();

                if ap.is_open {
                    Text::new(
                        "*",
                        Point::new(label_x + (rssi_str.len() as i32) * 6 + 2, rssi_y),
                        MonoTextStyle::new(&FONT_6X10, Rgb565::RED),
                    )
                    .draw(display)
                    .unwrap();
                }
            }
        }
    }

    // ── 5. Draw 2.4 GHz Baseline Axis & Legend ──────────────────────────────
    Line::new(Point::new(0, graph_baseline), Point::new(w, graph_baseline))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(display)
        .unwrap();

    // Render Channel numbers and AP counts below baseline
    for ch in 1u8..=13u8 {
        let ch_idx = (ch - 1) as usize;
        let center_x = 10 + (ch as i32) * channel_step_x;
        let color = CHANNEL_COLORS[ch_idx];

        // Channel Number label
        let mut ch_buf = [0u8; 16];
        let ch_str = format_u32(ch as u32, &mut ch_buf);
        let ch_style = MonoTextStyle::new(&FONT_6X10, color);
        Text::with_alignment(
            ch_str,
            Point::new(center_x, graph_baseline + 12),
            ch_style,
            Alignment::Center,
        )
        .draw(display)
        .unwrap();

        // AP Count label
        if ap_counts[ch_idx] > 0 {
            let mut count_buf = [0u8; 16];
            let count_str = format_u32(ap_counts[ch_idx] as u32, &mut count_buf);
            let count_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_LIGHT_GRAY);
            Text::with_alignment(
                count_str,
                Point::new(center_x, graph_baseline + 24),
                count_style,
                Alignment::Center,
            )
            .draw(display)
            .unwrap();
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("=== ESP32 Wi-Fi Analyzer Example (Waveshare ESP32-S3-Touch-AMOLED-1.64) ===");

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

    // ── Initial Wi-Fi AP Scan List ─────────────────────────────────────────
    let mut ap_list = [
        WifiAccessPoint {
            ssid: "Home_5G",
            channel: 1,
            rssi: -45,
            is_open: false,
        },
        WifiAccessPoint {
            ssid: "Guest_WiFi",
            channel: 1,
            rssi: -72,
            is_open: true,
        },
        WifiAccessPoint {
            ssid: "IoT_Net",
            channel: 3,
            rssi: -68,
            is_open: false,
        },
        WifiAccessPoint {
            ssid: "Office_AP",
            channel: 6,
            rssi: -38,
            is_open: false,
        },
        WifiAccessPoint {
            ssid: "Coffee_Shop",
            channel: 6,
            rssi: -62,
            is_open: true,
        },
        WifiAccessPoint {
            ssid: "Studio_Net",
            channel: 9,
            rssi: -54,
            is_open: false,
        },
        WifiAccessPoint {
            ssid: "Neighbor_2G",
            channel: 11,
            rssi: -78,
            is_open: false,
        },
        WifiAccessPoint {
            ssid: "Public_Free",
            channel: 11,
            rssi: -42,
            is_open: true,
        },
    ];

    info!("Starting Wi-Fi spectrum analyzer loop...");
    let mut scan_num = 1u32;

    loop {
        // Render updated Wi-Fi spectrum graph onto framebuffer
        render_wifi_analyzer(&mut fb_disp, &ap_list, scan_num);

        // Flush framebuffer to AMOLED display
        flush_framebuffer(&mut fb_disp).await;
        info!("Scan cycle {} displayed on screen.", scan_num);

        // Simulate slight RSSI fluctuations per scan cycle
        ap_list[0].rssi = -45 + ((scan_num as i32 * 3) % 7) - 3;
        ap_list[3].rssi = -38 + ((scan_num as i32 * 5) % 9) - 4;
        ap_list[7].rssi = -42 + ((scan_num as i32 * 2) % 5) - 2;

        scan_num += 1;
        Timer::after(Duration::from_secs(3)).await;
    }
}

// ---------------------------------------------------------------------------
// Numeric Formatting Helpers without std
// ---------------------------------------------------------------------------

fn format_u32(mut val: u32, buf: &mut [u8; 16]) -> &str {
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

fn format_i32(val: i32, buf: &mut [u8; 16]) -> &str {
    if val == 0 {
        buf[0] = b'0';
        return core::str::from_utf8(&buf[..1]).unwrap();
    }
    let is_neg = val < 0;
    let mut uval = if is_neg { -val as u32 } else { val as u32 };

    let mut idx = 16;
    while uval > 0 && idx > 0 {
        idx -= 1;
        buf[idx] = b'0' + (uval % 10) as u8;
        uval /= 10;
    }
    if is_neg && idx > 0 {
        idx -= 1;
        buf[idx] = b'-';
    }
    core::str::from_utf8(&buf[idx..16]).unwrap()
}
