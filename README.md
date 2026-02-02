# AlphaLoc

AlphaLoc is an ESP32 application that allows embedding GPS location data directly into Sony Alpha camera photos via Bluetooth Low Energy (BLE). It acts as a bridge between a standard GPS module and your Sony camera, providing real-time geolocation tagging.

## Features

- **Automatic Geotagging**: Sends GPS coordinates (Latitude, Longitude, Altitude, Time) to the camera.
- **BLE Connection**: Emulates a smartphone location provider for Sony cameras.
- **Dual Configuration**: Configurable via a Web Interface (WiFi) or BLE characteristics.
- **Status Indicators**: NeoPixel (RGB LED) feedback for Camera connection, GPS fix, Battery, and WiFi status.
- **Low Power**: Supports disabling external peripherals (like Stemma QT) to save power.

## Hardware Support

The project is set up for **PlatformIO** and currently supports:

| Board | Environment | Pinout |
|-------|-------------|--------|
| [**DFRobot Beetle ESP32-C6**](https://wiki.dfrobot.com/SKU_DFR1117_Beetle_ESP32_C6) | `esp32c6` | GPS TX: 5, GPS RX: 4, NeoPixel: 6 |
| [**Adafruit Feather ESP32-S3**](https://learn.adafruit.com/adafruit-esp32-s3-feather/overview)| `esp32s3` | GPS TX: 38, GPS RX: 39, NeoPixel: 6 |

You can easily adapt it to other ESP32 boards by modifying `platformio.ini`.

### Parts List

The following is a parts list of what I ended up with.

- [Adafruit ESP32-S3 Feather](https://www.adafruit.com/product/5885), [Overview](https://learn.adafruit.com/adafruit-esp32-s3-feather/overview)
- [Adafruit Ultimate GPS featherwing](https://learn.adafruit.com/adafruit-ultimate-gps-featherwing)
- [SparkFun RGB LED Breakout - WS2812B](https://www.sparkfun.com/sparkfun-rgb-led-breakout-ws2812b.html)
- [Adafruit WiFi Antenna with w.FL / MHF3 / IPEX3 Connector](https://www.adafruit.com/product/5445)
    - This is optional if you don't have an ESP32 with a PCB antenna.
- Mini sliding switch (8.5mm x 3.7mm, 3 pin, slider 4mm tall)
- The antenna of a GY-GPS6MV2
- 2x Panasonic NCR18650GA Battery (3450mAh)
- Wires, electrical tape and whatnot

### Models

- [**Holder**](model/holder.3mf): Holder for the batteries, ESP32, optional wifi antenna, ...
- [**Cover**](model/cover.3mf): Cover with a press fit for the holder and room for the LED board.
- [**Cover Large**](model/cover_large.3mf): Same as cover but taller and with room for a larger, external GPS antenna.

## Usage

### 1. Initial Setup

1. Connect your GPS module (e.g., GY-GPS6MV2 or an Adafruit Ultimate GPS Featherwing) to the defined TX/RX pins.

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

The NeoPixel LED provides a visual "heartbeat" every 5 seconds. It flashes multiple colors in sequence to report the status of different subsystems:

**Sequence:** [Camera Status] -> [GPS Status] -> [Battery Status] -> [WiFi Status] -> [Sleep]

1.  **First Flash: Camera Connection**
    *   🟢 **Green**: Connected to camera and bonded.
    *   🔵 **Blue**: Connected to camera but not bonded yet.
    *   🔴 **Red**: Not connected.
2.  **Second Flash: GPS Fix**
    *   🟢 **Green**: Valid 3D GPS Fix acquired.
    *   🟣 **Violet**: *Fake* GPS fix (injected when `ALPHALOC_FAKE_GPS=1`).
    *   🔴 **Red**: No fix (searching for satellites).
3.  **Third Flash: Battery Level (Optional)**
    *   🟢 **Green**: > 50%
    *   🟡 **Yellow**: > 30%
    *   🔴 **Red**: <= 30%
4.  **Fourth Flash: WiFi/Config (Optional)**
    *   Only appears during the startup "Config Window" (first 5 minutes).
    *   🔵 **Blue**: Web server is active.
    *   ⚫ **(Off)**: Config window closed, WiFi disabled to save power.

**Example**:
*   🔴-🔴-🟢-🔵: No camera, no GPS, battery good, config mode active (just turned on).
*   🔵-🟢-🟡-⚫: Camera connected (not bonded yet), GPS fixed, battery medium, normal operation (config closed).
*   🟢-🟢-🔴-⚫: Camera connected (bonded), GPS fixed, battery low, normal operation (config closed).
*   🔵-🟣-🟢-⚫: Camera connected (not bonded yet), fake GPS fixed, battery good, normal operation (config closed).

### 4. Configuration

> **⚠️ SECURITY WARNING**: The BLE configuration service is **DISABLED by default** for security reasons. When enabled, it allows **UNAUTHENTICATED access** to device settings including WiFi passwords. Only enable it (`ALPHALOC_BLE_CONFIG=1`) in trusted environments or for initial setup, then rebuild with it disabled.

On startup, AlphaLoc enters a **Configuration Window** (default 5 minutes). During this time, you can change settings via WiFi (if `ALPHALOC_WIFI_WEB=1`) or BLE (if `ALPHALOC_BLE_CONFIG=1`).

<img width="300" height="450" alt="web" src="https://github.com/user-attachments/assets/bd50c0e7-f0e7-41f6-8e6a-0d7c2ba5bdf6" />

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

By default, the device creates an access point:

- **SSID**: `AlphaLoc`
- **Password**: `alphaloc1234`
- **IP**: `192.168.4.1` (direct link to webserver: http://192.168.4.1/)

See the build flag (`ALPHALOC_WIFI_WEB=1` in `platformio.ini`) to enable it acting as a wifi client:

- **Default SSID**: `WiFi`
- **Default Password**: `changeme`
- **IP**: assigned via DHCP

Navigate to the device IP in your browser to access the config page.

#### Method B: BLE Configuration

You can use a generic BLE app (like nRF Connect) to write to the configuration service.

**Service UUID**: `B1F0B4D5-797B-5A9E-5B4F-4A1F01007EA1`

| Characteristic Name | UUID (Base `B1F0...`) | R/W | Type (Over BLE) | Description |
| ------------------- | --------------------- | --- | --------------- | ----------- |
| Camera Name Prefix  | `...02007EA1`         | R/W | String          | Camera name identifier |
| Camera MAC Prefix   | `...03007EA1`         | R/W | String          | MAC address filter |
| Timezone Offset     | `...04007EA1`         | R/W | String (Int)    | Minutes from UTC |
| DST Offset          | `...05007EA1`         | R/W | String (Int)    | Daylight Savings offset (minutes) |
| Wifi Mode           | `...06007EA1`         | R/W | String ("0"/"1")| 0=AP, 1=Station |
| Wifi SSID           | `...07007EA1`         | R/W | String          | Station SSID |
| Wifi Password       | `...08007EA1`         | R/W | String          | Station Password |
| AP SSID             | `...09007EA1`         | R/W | String          | Access Point SSID |
| AP Password         | `...0A007EA1`         | R/W | String          | Access Point Password |
| Max GPS Age         | `...0B007EA1`         | R/W | String (Int)    | Max age of GPS lock in seconds |
| GPS Lock            | `...0C007EA1`         | R   | String (0/1)    | 1 when GPS lock is valid |
| GPS Satellites      | `...0D007EA1`         | R   | String (Int)    | Satellites in view (from GGA) |
| GPS Constellations  | `...0E007EA1`         | R   | String (Int)    | Bitmask: 1=GPS, 2=GLONASS |
| Camera Connected    | `...0F007EA1`         | R   | String (0/1)    | BLE camera link active |
| Camera Bonded       | `...10007EA1`         | R   | String (0/1)    | Link bonded (after pairing) |

## BLE Client Details (Camera Link)

AlphaLoc acts as a BLE client to Sony cameras

### Advertisement Filters

- **Manufacturer data**: company ID `0x012D`, product ID `0x0003`
- **Optional name prefix filter**: `camera_name_prefix` (if set)
- **Optional MAC prefix filter**: `camera_mac_prefix` (if set)

### Services and Identifiers

AlphaLoc looks for two Sony services (UUIDs are built from 16-bit IDs using the Sony base UUID):

```
Sony base UUID (constructed):
  00000000-0000-8000-ffff-ffff-ffff-ffff
  with 16-bit words inserted at positions:
    first = 0xDD00 / 0xFF00
    second = 0xDD00 / 0xFF00
```

It accepts either 128-bit UUIDs or 16-bit UUIDs:

- **Location Service**: `0xDD00`
- **Remote Service**: `0xFF00`

### Characteristic Discovery

Within those services, AlphaLoc searches for:

**Location Service (DD00)**

- `0xDD21`: Flags read to determine if TZ/DST are required
- `0xDD30`: Lock location (0x00 - Off, 0x01 - On)
- `0xDD31`: Enable location updates (0x00 - Off, 0x01 - On)
- `0xDD11`: Location write characteristic; used for sending GPS payloads

Data is a 95 byte buffer

| Offset    | Description                                                  | Remark                                           |
|-----------|--------------------------------------------------------------|--------------------------------------------------|
| [0:1]     | Payload Length (exclude these two bytes)                     | 0x5D = 93 bytes                                  |
| [2:4]     | Fixed Data                                                   | 0x0802FC                                         |
| [5]       | Flag of transmitting timezone offset and DST offset required | 0x03 for transmit and 0x00 for do not transmit   |
| [6:10]    | Fixed Data                                                   | 0x0000101010                                     |
| [11:14]   | Latitude (multiplied by 10000000)                            | 0x0BF79E5E = 200777310 / 10000000 = 20.077731    |
| [15:18]   | Longitude (multiplied by 10000000) 	                       | 0x41C385A7 = 1103332775 / 10000000 = 110.3332775 |
| [19:20]   | 	UTC Year                                                   | 0x07E4 = 2020                                    |
| [21]      | 	UTC Month                                                  | 0x0B = 11                                        |
| [22]      | 	UTC Day                                                    | 0x05 = 5                                         |
| [23]      | 	UTC Hour                                                   | 0x04 = 4                                         |
| [24]      | 	UTC Minute                                                 | 0x02 = 2                                         |
| [25]      | 	UTC Second                                                 | 0x2A = 42                                        |
| [26:90]   | 	Zeros  	                                                   | 0x00                                             |
| *[91:92]  | 	Difference between UTC and current timezone in minutes     | 0x01E0 = 480min = 8h (UTC+8)                     |
| *[93:94]  | 	Difference for DST in current timezone in minutes          |                                                  |

**Remote Service (FF00)**

- `0xFF02` (notification characteristic; CCCD `0x2902`)

### Connection/Discovery Flow (simplified)

```
ADV (Sony) --> connect
  |
  +--> discover DD00 (Location Service)
  |      +--> find DD11, DD21, DD30, DD31
  |      +--> read DD21 flag byte
  |      +--> enable location writes on DD11
  |
  +--> discover FF00 (Remote Service)
         +--> find FF02
         +--> find/guess CCCD (0x2902)
         +--> enable notifications
```

### Sending Location

Once services and characteristics are resolved:
- Builds a Sony-compatible location payload
- Writes to `DD11`
- Enables FF02 notifications after first location write (if required)

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
*   **`env:esp32s3`**: The standard production build for the ESP32-S3. Logging is disabled for performance.
*   **`env:esp32s3-debug`**: A debugging environment.
    *   Enables verbose logging (`ALPHALOC_VERBOSE=1`).
    *   **Enables Fake GPS (`ALPHALOC_FAKE_GPS=1`)**: Simulates a stationary location (Munich) for testing without a GPS module or satellite lock.
*   **`env:esp32s3-debug-gps`**: Debugging environment using *real* GPS data but with verbose logging enabled.

### Build Flags

You can customize the firmware behavior using these compilation flags in `platformio.ini`:

| Flag | Description | Default |
|------|-------------|---------|
| `ALPHALOC_WIFI_WEB` | Enable internal settings web server. Set to `0` to remove WiFi stack and save power/flash. | `1` |
| `ALPHALOC_WIFI_MODE` | Default WiFi mode (`0` -> `APP_WIFI_MODE_AP` or `1` -> `APP_WIFI_MODE_STA`) used on first boot or after factory reset. | `APP_WIFI_MODE_AP` |
| `ALPHALOC_BLE_CONFIG` | **Enable BLE configuration service.** Note: This allows reading and writing the config via BLE unauthenticated! | `0` (DISABLED) |
| `ALPHALOC_VERBOSE` | Enable extensive debug logging to serial UART. | `0` |
| `ALPHALOC_FAKE_GPS` | Ignore UART GPS and output a static test location. | `0` |
| `ALPHALOC_NEOPIXEL_PIN`| GPIO pin number for the WS2812B NeoPixel. | (Board dependent) |
| `ALPHALOC_BATTERY_MONITOR` | Enable MAX17048 / LC709203F battery monitor over I2C. | `0` |
| `ALPHALOC_BATTERY_SDA_PIN` | I2C SDA pin for battery monitor. | (Board dependent) |
| `ALPHALOC_BATTERY_SCL_PIN` | I2C SCL pin for battery monitor. | (Board dependent) |
| `ALPHALOC_BATTERY_I2C_POWER_PIN` | Optional power-enable pin for I2C battery monitor. | (Unset) |
| `GPS_UART_TX_PIN` | TX Pin for GPS Serial (Connects to GPS RX). | (Board dependent) |
| `GPS_UART_RX_PIN` | RX Pin for GPS Serial (Connects to GPS TX). | (Board dependent) |
| `DALPHALOC_FACTORY_RESET` | If set to `1`, wipes NVS settings on boot. Dangerous. | Undefined |

**Security Best Practices:**
- Keep `ALPHALOC_BLE_CONFIG=0` in production builds
- Only enable BLE config for initial setup in controlled environments
- Use WiFi config (`ALPHALOC_WIFI_WEB=1`) with strong AP password instead
- Change default WiFi credentials immediately after first boot

## References

There is a bunch of work done by others before me which greatly helped figuring out how the BLE protocol works with the Sony Alpha cameras. I'm sure I'm forgetting a bunch but to at least name a few:

- https://github.com/dzwiedziu-nkg/esp32-a7iv-rc
- https://github.com/ekutner/camera-gps-link
- https://github.com/whc2001/ILCE7M3ExternalGps
- And many more (search GitHub for one of the unique identifiers such as the service UUIDs).
