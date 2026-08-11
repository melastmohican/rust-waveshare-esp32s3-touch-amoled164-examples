# Rust Embassy Examples for Waveshare ESP32-S3-Touch-AMOLED-1.64

This repository contains idiomatic Rust examples for the **Waveshare ESP32-S3-Touch-AMOLED-1.64** development board, built using the [Embassy](https://embassy.dev/) async framework, `esp-hal` (v1.1), `esp-rtos`, and `defmt` logging over USB-UART.

---

## Hardware Overview

- **Board:** Waveshare ESP32-S3-Touch-AMOLED-1.64
- **SoC:** ESP32-S3 (Dual-core Xtensa LX7 @ 240 MHz, Wi-Fi 4, Bluetooth 5 LE, 16MB Flash, 8MB PSRAM)
- **Onboard Peripherals:**
  - **Display:** 1.64-inch CO5300 QSPI AMOLED Panel (280×456 native resolution)
  - **Touch Controller:** FocalTech FT3168 Capacitive Touch IC (`0x38`)
  - **IMU:** QST QMI8658 6-Axis Motion Tracking Sensor (`0x6B`)

### Common Pin Assignments

| Peripheral / Interface | Signal | GPIO Pin | Notes |
|---|---|---|---|
| **I2C Bus (I2C0)** | **SDA** | GPIO47 | Shared bus (FT3168 Touch & QMI8658 IMU) |
| | **SCL** | GPIO48 | Shared bus (FT3168 Touch & QMI8658 IMU) |
| **Touch Controller** | **Reset** | GPIO8 | Active-LOW Hardware Reset |
| **AMOLED Display (QSPI)** | **SCLK** | GPIO10 | QSPI Clock |
| | **SIO0 / D0** | GPIO11 | QSPI Data Line 0 |
| | **SIO1 / D1** | GPIO12 | QSPI Data Line 1 |
| | **SIO2 / D2** | GPIO13 | QSPI Data Line 2 |
| | **SIO3 / D3** | GPIO14 | QSPI Data Line 3 |
| | **Reset** | GPIO21 | Active-LOW LCD Hardware Reset |
| | **CS** | GPIO9 | Active-LOW Chip Select (Rev V1=GPIO9, Rev V2=GPIO46) |

---

## Examples

### 1. I2C Bus Scanner (`i2c_scan`)

Scans the onboard `I2C0` bus (`SDA: GPIO47`, `SCL: GPIO48`) for connected peripherals (detecting FT3168 Touch at `0x38` and QMI8658 IMU at `0x6B`). Logs scan results via `defmt` and displays formatted detection status on the CO5300 AMOLED screen.

```bash
cargo run --example i2c_scan
```

---

### 2. FT3168 Capacitive Touch (`ft3168_i2c`)

Demonstrates touch screen input handling using the `ft3x68-rs` driver crate over `I2C0`. Resets the touch IC via `GPIO8`, reads `(X, Y)` touch event coordinates, logs events via `defmt`, and renders an interactive target indicator circle and coordinate readout live on the AMOLED display screen.

```bash
cargo run --example ft3168_i2c
```

---

### 3. QMI8658 6-Axis Motion Tracking IMU (`qmi8658_i2c`)

Reads accelerometer (X, Y, Z) and gyroscope (X, Y, Z) telemetry from the onboard QMI8658 sensor using the async `ph-qmi8658` driver crate over `I2C0` at address `0x6B`. Displays live numerical telemetry and an animated tilt visualizer box on the CO5300 AMOLED display screen.

```bash
cargo run --example qmi8658_i2c
```

---

### 4. CO5300 QSPI AMOLED Display Driver (`co5300-qspi`)

Demonstrates low-level QSPI communication with the CO5300 display controller (280×456 native resolution) using `display-driver-co5300` and `embedded-graphics`. Renders colorful geometry, text banners, and graphics primitives.

```bash
cargo run --example co5300-qspi
```

---

### 5. Philips PM5544 Test Pattern (`pm5544`)

Displays the classic Philips PM5544 TV test pattern centered on the 280×456 CO5300 AMOLED display screen using `embedded-graphics` and `tinybmp`.

```bash
cargo run --example pm5544
```

---

### 6. Zermatt Photo Viewer (`zermatt`)

Displays a full-screen BMP image of Zermatt on the 280×456 CO5300 AMOLED display screen using zero-copy flash embedding via `tinybmp`.

```bash
cargo run --example zermatt
```

---

### 7. Snow Animation Demo (`zermatt_snow`)

Overlays a real-time particle snow animation on top of the Zermatt mountain photo on the 280×456 AMOLED display screen using Embassy async timers.

```bash
cargo run --example zermatt_snow
```

---

## Prerequisites & Setup

### 1. Install Rust Xtensa Toolchain

Install `espup` to set up the Xtensa Rust compiler toolchain:

```bash
cargo install espup
espup install
source $HOME/export-esp.sh
```

### 2. Install espflash

Install `espflash` for flashing and serial monitoring over USB:

```bash
cargo install espflash
```

---

## License

Dual-licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
