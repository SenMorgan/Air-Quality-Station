/**
 * @file ha_discovery.h
 * @brief Home Assistant MQTT device-based discovery payload.
 *
 * The whole payload is a compile-time constant stored in flash (PROGMEM),
 * so it costs no RAM and can never be published empty or half-built.
 * It is streamed to the broker in small chunks, see publish_discovery().
 *
 * Notes:
 * - Topics are written in full: a root-level "~" is NOT inherited by the
 *   components in device-based discovery, only shared options are
 *   (availability, state/command topic, qos...).
 * - unique_id values must stay exactly as they are: they match the entities
 *   previously created from YAML and keep their history.
 */

#pragma once

#include <Arduino.h>
#include "def.h"

#define HA_DISCOVERY_TOPIC "homeassistant/device/air_quality_station/config"
#define HA_STATUS_TOPIC    "homeassistant/status"

// Common part of every component: platform, name, unique_id, default_entity_id, state topic.
// unique_id = mqtt_<platform>_air_quality_station_<key>
// default_entity_id = <platform>.air_quality_station_<key>
#define HA_CMP(platform, key, name, topic)                              \
    "\"" key "\":{"                                                     \
    "\"p\":\"" platform "\","                                           \
    "\"name\":\"" name "\","                                            \
    "\"uniq_id\":\"mqtt_" platform "_air_quality_station_" key "\","    \
    "\"default_entity_id\":\"" platform ".air_quality_station_" key "\"," \
    "\"stat_t\":\"" DEFAULT_TOPIC topic "\""

#define HA_MEASUREMENT ",\"stat_cla\":\"measurement\""
#define HA_DIAGNOSTIC  ",\"ent_cat\":\"diagnostic\""
#define HA_EXPIRE      ",\"exp_aft\":900"

// clang-format off
#define HA_DISCOVERY_JSON                                                                   \
    "{"                                                                                     \
    "\"dev\":{"                                                                             \
        "\"ids\":[\"air_quality_station\"],"                                                \
        "\"name\":\"Air Quality Station\","                                                 \
        "\"mf\":\"SenMorgan (DIY)\","                                                       \
        "\"mdl\":\"Air Quality Station (BME680 + IKEA Vindriktning)\","                     \
        "\"sw\":\"" FIRMWARE_VERSION "\","                                                  \
        "\"sa\":\"Kitchen\""                                                                \
    "},"                                                                                    \
    "\"o\":{\"name\":\"air-quality-station\",\"sw\":\"" FIRMWARE_VERSION "\"},"             \
    /* Shared by all components */                                                          \
    "\"avty_t\":\"" MQTT_AVAILABILITY_TOPIC "\","                                           \
    "\"pl_avail\":\"" MQTT_AVAILABILITY_MESSAGE "\","                                       \
    "\"pl_not_avail\":\"" MQTT_WILL_MESSAGE "\","                                           \
    "\"cmps\":{"                                                                            \
        HA_CMP("sensor", "temperature", "Temperature", "temperature")                       \
            ",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\""                          \
            HA_MEASUREMENT ",\"sug_dsp_prc\":1" HA_EXPIRE "},"                              \
        HA_CMP("sensor", "humidity", "Humidity", "humidity")                                \
            ",\"dev_cla\":\"humidity\",\"unit_of_meas\":\"%\""                              \
            HA_MEASUREMENT ",\"sug_dsp_prc\":1" HA_EXPIRE "},"                              \
        HA_CMP("sensor", "pressure", "Pressure", "pressure")                                \
            ",\"dev_cla\":\"atmospheric_pressure\",\"unit_of_meas\":\"hPa\""                \
            HA_MEASUREMENT ",\"sug_dsp_prc\":1" HA_EXPIRE "},"                              \
        HA_CMP("sensor", "iaq", "IAQ", "iaq")                                               \
            ",\"unit_of_meas\":\"IAQ\""                                                     \
            HA_MEASUREMENT ",\"sug_dsp_prc\":0" HA_EXPIRE ",\"ic\":\"mdi:air-filter\"},"    \
        HA_CMP("sensor", "iaq_accuracy", "IAQ Accuracy", "iaqAccuracy")                     \
            ",\"sug_dsp_prc\":0" HA_DIAGNOSTIC HA_EXPIRE ",\"ic\":\"mdi:check-decagram\"}," \
        HA_CMP("sensor", "co2", "CO2", "co2Equivalent")                                     \
            ",\"dev_cla\":\"carbon_dioxide\",\"unit_of_meas\":\"ppm\""                      \
            HA_MEASUREMENT ",\"sug_dsp_prc\":0" HA_EXPIRE "},"                              \
        HA_CMP("sensor", "voc", "VOC", "breathVocEquivalent")                               \
            ",\"dev_cla\":\"volatile_organic_compounds_parts\",\"unit_of_meas\":\"ppm\""    \
            HA_MEASUREMENT ",\"sug_dsp_prc\":2" HA_EXPIRE "},"                              \
        HA_CMP("sensor", "pm2_5", "PM2.5", "pm25")                                          \
            ",\"dev_cla\":\"pm25\",\"unit_of_meas\":\"µg/m³\""                              \
            HA_MEASUREMENT ",\"sug_dsp_prc\":0" HA_EXPIRE ",\"ic\":\"mdi:blur\"},"          \
        HA_CMP("binary_sensor", "digital_input_1", "Digital Input 1", "dig_input_1")        \
            ",\"pl_on\":\"1\",\"pl_off\":\"0\",\"ic\":\"mdi:toggle-switch-variant\"},"      \
        HA_CMP("binary_sensor", "digital_input_2", "Digital Input 2", "dig_input_2")        \
            ",\"pl_on\":\"1\",\"pl_off\":\"0\",\"ic\":\"mdi:toggle-switch-variant\"},"      \
        HA_CMP("sensor", "uptime", "Uptime", "uptime")                                      \
            ",\"dev_cla\":\"duration\",\"unit_of_meas\":\"s\",\"sug_dsp_prc\":0"            \
            HA_DIAGNOSTIC "},"                                                              \
        HA_CMP("sensor", "rssi", "Wi-Fi Signal", "rssi")                                    \
            ",\"dev_cla\":\"signal_strength\",\"unit_of_meas\":\"dBm\""                     \
            HA_MEASUREMENT ",\"sug_dsp_prc\":0" HA_DIAGNOSTIC "}"                           \
    "}"                                                                                     \
    "}"
// clang-format on

static const char HA_DISCOVERY_PAYLOAD[] PROGMEM = HA_DISCOVERY_JSON;

// Guard against ever publishing an empty retained config (it deletes the device in HA)
static_assert(sizeof(HA_DISCOVERY_PAYLOAD) > 1000, "Discovery payload looks truncated");
