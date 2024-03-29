/**
 * @file def.h
 * @author SenMorgan https://github.com/SenMorgan
 * @date 2022-06-05
 *
 * @copyright Copyright (c) 2021 Sen Morgan
 *
 */

#ifndef _DEF_H_
#define _DEF_H_

// MQTT definitions
#define DEFAULT_TOPIC             "/air-quality-station/"
#define MQTT_WILL_TOPIC           DEFAULT_TOPIC "availability"
#define MQTT_QOS                  1
#define MQTT_RETAIN               1
#define MQTT_WILL_MESSAGE         "offline"
#define MQTT_AVAILABILITY_TOPIC   DEFAULT_TOPIC "availability"
#define MQTT_AVAILABILITY_MESSAGE "online"
#define MQTT_UPTIME_TOPIC         DEFAULT_TOPIC "uptime"

// MQTT publish interval
#define MQTT_PUBLISH_INTERVAL       15000 // 15 seconds
// Reconnecting to MQTT server delay
#define RECONNECT_DELAY             5000
// Status LED error blinking brightness
#define STATUS_LED_ERROR_BRIGHTNESS 250
// Digital input debounce delay
#define DIG_IN_DEBOUNCE_DELAY       50

// IO pins
// #define LED_BUILTIN 2    // D4 - already defined in pins_arduino.h
#define SDA_PIN         12 // D6
#define SCL_PIN         14 // D5
#define IKEA_VKG_RX_PIN 13 // D7
#define IKEA_VKG_TX_PIN 16 // D0 - not used, but must be defined
#define DIG_INPUT_1_PIN 5  // D1
#define DIG_INPUT_2_PIN 4  // D2

#endif // _DEF_H_