# Air Quality Station

Smart Air Quality Station based on Wemos D1 mini (ESP8266), Bosch BME680 (using BSEC Arduino Library to get IAQ (Indoor Air Quality) and other environmental data) and the PM2.5 sensor from IKEA VINDRIKTNING. Two digital inputs are also exposed.
WiFi and MQTT is used to send data to the Home Assistant.<br>
The project was created in PlatformIO 04.06.2022

[![ESP8266](https://img.shields.io/badge/ESP-8266-000000.svg?longCache=true&style=flat&colorA=AA101F)](https://www.espressif.com/en/products/socs/esp8266)<br>
[![Build with PlatformIO](https://img.shields.io/badge/Build%20with-PlatformIO-orange)](https://platformio.org/)<br>
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)

<br>

## Build and settings
1. Install (if you haven't already) [PlatformIO](https://platformio.org/) extension for VS Code.
2. Create a copy of `platformio_override.ini.example` and rename it to `platformio_override.ini`.
3. Set your own values in `platformio_override.ini`.
4. Update hardware and other settings in `lib/defs/def.h` file if needed.
5. Build and upload the project to your ESP device using PlatformIO.

> `platformio_override.ini` contains your credentials and is ignored by git - never commit it.

### Notes
- BSEC calibration state is saved to EEPROM (flash) when IAQ accuracy reaches 3 and then every 6 hours (`BSEC_STATE_SAVE_PERIOD`), so the calibration survives reboots.
- On ESP8266 the precompiled BSEC library does not fit into IRAM, so `scripts/bsec_irom.py` moves it to flash during the build.
- On a sensor error the status LED blinks for 10 seconds and the device restarts.

## Wiring (Wemos D1 mini)
| Signal                       | GPIO | Pin |
|------------------------------|------|-----|
| BME680 SDA                   | 12   | D6  |
| BME680 SCL                   | 14   | D5  |
| IKEA VINDRIKTNING TX (REST)  | 13   | D7  |
| Digital input 1 (pull-up)    | 5    | D1  |
| Digital input 2 (pull-up)    | 4    | D2  |

## BME680 Sensor
The BME680 is a low-power gas, pressure, humidity and temperature sensor. The sensor is based on a metal oxide (MOX) gas sensor and an integrated heater. The heater is used to heat up the MOX gas sensor and to desorb VOCs from the sensor surface.

## BSEC library
>*From BSEC Software Library:*<br><br>
Bosch Sensortec Environmental Cluster (BSEC) Software v1.4.8.0 released on July 8th, 2020<br><br>
The BSEC fusion library has been conceptualized to provide a higher-level signal processing and fusion for the BME680. The library receives compensated sensor values from the sensor API. It processes the BME680 signals to provide the requested sensor outputs.<br>
Key features:<br>
>- Precise calculation of ambient air temperature outside the device<br>
>- Precise calculation of ambient relative humidity outside the device
>- Precise calculation of pressure outside the device
>- Precise calculation of air quality (IAQ) level outside the device

## About IAQ value
>*From BSEC Software Library:*<br><br>
The IAQ scale ranges from **0** (clean air) to **500** (heavily polluted air). During operation, algorithms automatically calibrate and adapt themselves to the typical environments where the sensor is operated (e.g., home, workplace, inside a car, etc.).This automatic background calibration ensures that users experience consistent IAQ performance. The calibration process considers the recent measurement history (typ. up to four days) to ensure that **IAQ=25** corresponds to typical good air and **IAQ=250** indicates typical polluted air.

## About IAQ accuracy
 >*From BSEC Software Library:*<br><br>
The IAQ accuracy is reflects the current state of the background calibration process, such as:<br>
**IAQ Accuracy=0** could either mean:<br>
BSEC was just started, and the sensor is stabilizing (this lasts normally 5min in LP mode or 20min in ULP mode),
there was a timing violation (i.e. BSEC was called too early or too late), which should be indicated by a warning/error flag by BSEC,<br>
**IAQ Accuracy=1** means the background history of BSEC is uncertain. This typically means the gas sensor data was too stable for BSEC to clearly define its references,<br>
**IAQ Accuracy=2** means BSEC found a new calibration data and is currently calibrating,<br>
**IAQ Accuracy=3** means BSEC calibrated successfully.

## Home Assistant
The station announces itself via [MQTT discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery), no YAML is needed.
A single retained device-based discovery message is published to `homeassistant/device/air_quality_station/config`
after every connection to the broker and whenever Home Assistant sends `online` to `homeassistant/status`.
The payload is defined in `lib/ha_discovery/ha_discovery.h`.

| Entity | State topic | Unit |
|---|---|---|
| Temperature | `/air-quality-station/temperature` | °C |
| Humidity | `/air-quality-station/humidity` | % |
| Pressure | `/air-quality-station/pressure` | hPa |
| IAQ (static IAQ) | `/air-quality-station/iaq` | IAQ |
| IAQ Accuracy | `/air-quality-station/iaqAccuracy` | 0..3 |
| CO2 equivalent | `/air-quality-station/co2Equivalent` | ppm |
| Breath VOC equivalent | `/air-quality-station/breathVocEquivalent` | ppm |
| PM2.5 | `/air-quality-station/pm25` | µg/m³ |
| Digital Input 1, 2 | `/air-quality-station/dig_input_1`, `dig_input_2` | `1` / `0` |
| Uptime | `/air-quality-station/uptime` | s |
| Wi-Fi Signal | `/air-quality-station/rssi` | dBm |

Availability: `/air-quality-station/availability` (`online` / `offline`, retained, `offline` is the MQTT Last Will).
Values that are not available (PM2.5 before the first valid frame or when the sensor is silent, IAQ/CO2/VOC
while BSEC accuracy is 0) are not published at all, instead of sending a placeholder.

## Dependencies
All dependencies will be automatically installed by PlatformIO:
- BSEC Software Library@^1.8.1492
- PubSubClient@^2.8

## Copyright
Copyright (c) 2022 Sen Morgan. Licensed under the MIT license, see LICENSE.md