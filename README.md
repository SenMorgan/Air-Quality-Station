# Air Quality Station

Smart Air Quality Station based on NodeMCU v1.0 (ESP8266) and Bosch BME680 using BSEC Arduino Library to get IAQ (Indoor Air Quality) and other environmental data.
WiFi and MQTT is used to send data to the Home Assistant.<br>
The project was created in PlatformIO 04.06.2022

[![ESP8266](https://img.shields.io/badge/ESP-8266-000000.svg?longCache=true&style=flat&colorA=AA101F)](https://www.espressif.com/en/products/socs/esp8266)<br>
[![ESP32](https://img.shields.io/badge/ESP-32-000000.svg?longCache=true&style=flat&colorA=AA101F)](https://www.espressif.com/en/products/socs/esp32)<br>
[![Build with PlatformIO](https://img.shields.io/badge/Build%20with-PlatformIO-orange)](https://platformio.org/)<br>
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)

<br>

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

## Home Assistant YAML configuration
```yaml
sensor:
  - name: "Air Quality Station Temperature"
    state_topic: "/air-quality-station/temperature"
    icon: mdi:temperature-celsius
    device_class: temperature
    value_template: "{{ value }}"
    unit_of_measurement: "°C"
  - name: "Air Quality Station Humidity"
    state_topic: "/air-quality-station/humidity"
    device_class: humidity
    value_template: "{{ value }}"
    unit_of_measurement: "%"
  - name: "Air Quality Station Pressure"
    state_topic: "/air-quality-station/pressure"
    device_class: pressure
    value_template: "{{ value }}"
    unit_of_measurement: "hPa"
    # IAQ
  - name: "Air Quality Station IAQ"
    state_topic: "/air-quality-station/iaq"
    device_class: aqi
    value_template: "{{ value }}"
    unit_of_measurement: "IAQ"
    # IAQ accuracy
  - name: "Air Quality Station IAQ Accuracy"
    state_topic: "/air-quality-station/iaqAccuracy"
    value_template: "{{ value }}"
    unit_of_measurement: "IAQ Acc"
    # CO2 equivalent estimate [ppm]
  - name: "Air Quality Station CO2"
    state_topic: "/air-quality-station/co2Equivalent"
    device_class: carbon_dioxide
    value_template: "{{ value }}"
    unit_of_measurement: "ppm"
    # Breath VOC concentration estimate [ppm]
  - name: "Air Quality Station VOC"
    state_topic: "/air-quality-station/breathVocEquivalent"
    device_class: volatile_organic_compounds
    value_template: "{{ value }}"
    unit_of_measurement: "ppm"
```

## Settings
WiFi, OTA and MQTT settings must be set by renaming `platformio_override.ini.example` to `platformio_override.ini` and setting your own values.
Hardware and other settings are in `lib/defs/def.h` file.

## Dependencies
All dependencies will be automatically installed by PlatformIO:
- BSEC Software Library@1.6.1480
- PubSubClient@2.8.0

## Copyright
Copyright (c) 2022 Sen Morgan. Licensed under the MIT license, see LICENSE.md