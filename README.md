# AlphaLoc

AlphaLoc is an ESP32 application that allows embedding GPS location data directly into Sony Alpha camera photos via Bluetooth Low Energy (BLE). It acts as a bridge between a standard GPS module and your Sony camera, providing real-time geolocation tagging.

## Features

- **Automatic Geotagging**: Sends GPS coordinates (Latitude, Longitude, Altitude, Time) to the camera.
- **BLE Connection**: Emulates a smartphone location provider for Sony cameras.
- **Dual Configuration**: Configurable via a Web Interface (WiFi) or BLE characteristics.
- **Status Indicators**: NeoPixel (RGB LED) feedback for Camera connection, GPS fix, and WiFi status.
- **Low Power**: Supports disabling external peripherals (like Stemma QT) to save power.

## Hardware Support

The project is set up for **PlatformIO** and currently supports:

| Board | Environment | Pinout |
|-------|-------------|--------|
| **DFRobot Beetle ESP32-C6** | `esp32c6` | GPS TX: 5, GPS RX: 4, NeoPixel: 6 |
| **Adafruit Feather ESP32 V2** | `esp32-feather` | GPS TX: 8, GPS RX: 7, NeoPixel: 6 |

You can easily adapt it to other ESP32 boards by modifying `platformio.ini`.

## Usage

### 1. Initial Setup

1. Connect your GPS module (e.g., GY-GPS6MV2) to the defined TX/RX pins.
2. Flash the firmware using PlatformIO.
   ```bash
   pio run -e esp32c6 -t upload
   ```
3. Power on the device.

### 2. Pairing with Camera

1. On your Sony Alpha camera, go to **Network** -> **Bluetooth Settings**.
2. Enable **Bluetooth Function**.
3. Select **Pairing**.
4. The camera should discover "AlphaLoc". Select it to pair.
5. Once paired, go to **Location Info. Link Set.** and enable it.
6. The camera should now show the GPS icon (solid when receiving data).

### 3. LED Status Codes

The NeoPixel LED provides a visual "heartbeat" every 3 seconds. It flashes multiple colors in sequence to report the status of different subsystems:

**Sequence:** [Camera Status] -> [GPS Status] -> [WiFi Status] -> [Sleep]

1.  **First Flash: Camera Connection**
    *   🟢 **Green**: Connected to Camera via BLE.
    *   🔴 **Red**: Not connected.
2.  **Second Flash: GPS Fix**
    *   🟢 **Green**: Valid 3D GPS Fix acquired.
    *   🔴 **Red**: No fix (searching for satellites).
3.  **Third Flash: WiFi/Config (Optional)**
    *   Only appears during the startup "Config Window" (first 5 minutes).
    *   🔵 **Blue**: Web server is active.
    *   ⚫ **(Off)**: Config window closed, WiFi disabled to save power.

**Example**:
*   🔴-🔴-🔵: No Camera, No GPS, Config Mode Active (Just turned on).
*   🟢-🟢-⚫: Camera Connected, GPS Fixed, Normal Operation (Config closed).

### 4. Configuration
On startup, AlphaLoc enters a **Configuration Window** (default 5 minutes). During this time, you can change settings via WiFi or BLE.

#### Configuration Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| **Camera Name Prefix** | Filter which cameras to connect to. Useful if you have multiple Sonys. | `SonyA7` |
| **Camera MAC Prefix** | Connect only to a specific MAC address prefix. Empty matches any. | (Empty) |
| **TZ Offset** | Timezone offset in minutes (e.g., `60` for UTC+1). | `60` |
| **DST Offset** | Daylight Savings Time offset in minutes. | `60` |
| **WiFi Mode** | `0` = Access Point (AP), `1` = Station (Client). | Station |
| **WiFi SSID/Pass** | Credentials for connecting to your home WiFi. | `WiFi` / `changeme` |
| **AP SSID/Pass** | Credentials for the hotspot created by AlphaLoc. | `AlphaLoc` / `alphaloc1234` |
| **Max GPS Age** | How long (seconds) to reuse old coordinates if GPS signal is lost. | `300` |

#### Method A: WiFi Web Interface

By default, the device attempts to connect to a WiFi network.
- **Default SSID**: `WiFi`
- **Default Password**: `changeme`
- **Mode**: TCP Station (Client)

If you compile with `APP_WIFI_MODE_AP` (or change it via BLE/Code), it acts as an Access Point:
- **AP SSID**: `AlphaLoc`
- **AP Password**: `alphaloc1234`
- **IP**: `192.168.4.1`

Navigate to the device IP in your browser to access the config page.

#### Method B: BLE Configuration
You can use a generic BLE app (like nRF Connect) to write to the configuration service.
- **Service UUID**: `B1F0B4D5-797B-5A9E-5B4F-4A1F01007EA1`
- **Characteristics**:
  - Camera Configuration (Name/MAC prefix)
  - WiFi Settings (SSID, Password, Mode)
  - Timezone / DST offsets

## Configuration Options
- **Camera Name Prefix**: Limits connection to cameras starting with this name (Default: "SonyA7").
- **Timezone/DST**: Offset in minutes for timestamp correction.
- **Max GPS Age**: How long to keep sending the last known location if GPS signal is lost.

## Building

This project uses the Espressif IoT Development Framework (ESP-IDF) within PlatformIO.
Check `platformio.ini` for build flags and pin definitions.

```ini
; Example Override
build_flags =
  -DGPS_UART_TX_PIN=5
  -DGPS_UART_RX_PIN=4
```

### Build Environments

The `platformio.ini` file defines several build environments for different purposes:

*   **`env:esp32c6`**: The standard production build for the ESP32-C6. Logging is disabled for performance.
*   **`env:esp32c6-debug`**: A debugging environment.
    *   Enables verbose logging (`ALPHALOC_VERBOSE=1`).
    *   **Enables Fake GPS (`ALPHALOC_FAKE_GPS=1`)**: Simulates a stationary location (Munich) for testing without a GPS module or satellite lock.
*   **`env:esp32c6-debug-gps`**: Debugging environment using *real* GPS data but with verbose logging enabled.
*   **`env:esp32-feather`**: Production build for the Adafruit Feather ESP32 V2.

### Build Flags

You can customize the firmware behavior using these compilation flags in `platformio.ini`:

| Flag | Description | Default |
|------|-------------|---------|
| `ALPHALOC_WIFI_WEB` | Enable internal settings web server. Set to `0` to remove WiFi stack and save power/flash. | `1` |
| `ALPHALOC_VERBOSE` | Enable extensive debug logging to serial UART. | `0` |
| `ALPHALOC_FAKE_GPS` | Ignore UART GPS and output a static test location. | `0` |
| `ALPHALOC_NEOPIXEL_PIN`| GPIO pin number for the WS2812B NeoPixel. | (Board dependent) |
| `GPS_UART_TX_PIN` | TX Pin for GPS Serial (Connects to GPS RX). | (Board dependent) |
| `GPS_UART_RX_PIN` | RX Pin for GPS Serial (Connects to GPS TX). | (Board dependent) |
| `DALPHALOC_FACTORY_RESET` | If set to `1`, wipes NVS settings on boot. Dangerous. | Undefined |
