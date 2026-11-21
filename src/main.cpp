#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include "bsec.h"
#include "def.h"
#include "ha_discovery.h"
#include "ikea_vkg.h"

/* Configure the BSEC library with information about the sensor
    18v/33v = Voltage at Vdd. 1.8V or 3.3V
    3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
    4d/28d = Operating age of the sensor in days
    generic_18v_3s_4d
    generic_18v_3s_28d
    generic_18v_300s_4d
    generic_18v_300s_28d
    generic_33v_3s_4d
    generic_33v_3s_28d
    generic_33v_300s_4d
    generic_33v_300s_28d

    https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/Units-and-Ranges-for-IAQ-IAQ-accuracy-Static-IAQ-CO2-equivalent/td-p/11582

    Can I use the IAQ output, or base my algorithm on it to fulfill my use-case ?
        Use IAQ output. (For most mobile applications such as
        a PERSONAL air quality monitor, this is the best option)
    Can I use the static IAQ (sIAQ) output, or base my algorithm on it to fulfill
        my use-case ? Use sIAQ output, or its derivatives, such as CO2 or bVOC estimation.
        (For most static applications, such as smart home or HVAC, this is the best option)
    Can I develop an algorithm based on the compensated gas resistance value ?
        Use compensated raw resistance. Some knowledge of MOX sensing technology is required,
        especially for baseline tracking, but you can leverage the humidity and
        temperature compensation from BSEC. This output is a gas resistance value
        in ohms after removing the effect of humidity and temperature.
*/
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASSWD;

particleSensorState_t state;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
static_assert(BSEC_MAX_STATE_BLOB_SIZE < 0xFF, "State size must fit the EEPROM marker byte");

String output;

// Debounced states of the digital inputs (pulled up, so idle state is HIGH)
uint8_t dig_input_1_state = HIGH, dig_input_2_state = HIGH;

// Set when Home Assistant comes online, discovery is (re)published from loop()
// because publishing inside the MQTT callback would overwrite its buffer
bool discoveryPending = false;
// Publish all states on the next sample instead of waiting for the interval,
// so freshly discovered entities get values right away
bool forceDataPublish = true;

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadBsecState(void);
void updateBsecState(void);
void read_publish_inputs(void);
uint8_t reconnect(void);
void publish_data(void);
bool publish_discovery(void);
void mqtt_callback(char *topic, uint8_t *payload, unsigned int length);

void setup(void)
{
    Serial.begin(115200);
    SerialCom::setup();

    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DIG_INPUT_1_PIN, INPUT_PULLUP);
    pinMode(DIG_INPUT_2_PIN, INPUT_PULLUP);

    iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." +
             String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) +
             "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();

    iaqSensor.setConfig(bsec_config_iaq);
    checkIaqSensorStatus();

    // Restore the calibration so IAQ accuracy does not restart from 0 after reboot
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
    loadBsecState();

    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();

    WiFi.mode(WIFI_STA);
    // Hostname must be set before WiFi.begin(), otherwise it is not applied
    WiFi.hostname(HOSTNAME);
    WiFi.begin(ssid, pass);

    // Arduino OTA initializing
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    ArduinoOTA.onProgress([](uint16_t progress, uint16_t total)
                          { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); });
    ArduinoOTA.onEnd([]()
                     { digitalWrite(LED_BUILTIN, 1); });

    // Make sure that status led is off
    digitalWrite(LED_BUILTIN, HIGH);

    // MQTT initializing
    // Connection itself is established in loop() once WiFi is up
    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
    mqttClient.setCallback(mqtt_callback);

    // Print the header
    output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
    Serial.println(output);
}

// Function that is looped forever
void loop(void)
{
    ArduinoOTA.handle();
    SerialCom::handleUart(state);

    uint8_t connectedToServer = mqttClient.loop();

    // Home Assistant restarted: announce the device again
    if (connectedToServer && discoveryPending)
    {
        discoveryPending = false;
        publish_discovery();
        connectedToServer = mqttClient.connected();
    }

    if (iaqSensor.run())
    { // If new data is available
        output = String(millis());
        output += ", " + String(iaqSensor.rawTemperature);
        output += ", " + String(iaqSensor.pressure);
        output += ", " + String(iaqSensor.rawHumidity);
        output += ", " + String(iaqSensor.gasResistance);
        output += ", " + String(iaqSensor.iaq);
        output += ", " + String(iaqSensor.iaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        output += ", " + String(iaqSensor.staticIaq);
        output += ", " + String(iaqSensor.co2Equivalent);
        output += ", " + String(iaqSensor.breathVocEquivalent);
        Serial.println(output);
        updateBsecState();
        if (connectedToServer)
        {
            publish_data();
        }
    }
    else
    {
        checkIaqSensorStatus();
    }

    // Reconnect is throttled: connect() blocks, and long blocking
    // breaks the BSEC sampling timing
    static uint32_t lastReconnectAttempt = 0;
    if (!connectedToServer && WiFi.status() == WL_CONNECTED &&
        (lastReconnectAttempt == 0 || millis() - lastReconnectAttempt >= RECONNECT_DELAY))
    {
        lastReconnectAttempt = millis();
        reconnect();
    }

    read_publish_inputs();

    yield();
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
    // Statuses persist between run() calls, so report only changes
    // to avoid flooding the serial port on every loop iteration
    static decltype(iaqSensor.bsecStatus) lastBsecStatus = BSEC_OK;
    static decltype(iaqSensor.bme68xStatus) lastBme68xStatus = BME68X_OK;

    if (iaqSensor.bsecStatus != lastBsecStatus)
    {
        lastBsecStatus = iaqSensor.bsecStatus;
        if (iaqSensor.bsecStatus < BSEC_OK)
        {
            Serial.println("BSEC error code : " + String(iaqSensor.bsecStatus));
            errLeds();
        }
        else if (iaqSensor.bsecStatus > BSEC_OK)
        {
            Serial.println("BSEC warning code : " + String(iaqSensor.bsecStatus));
        }
    }

    if (iaqSensor.bme68xStatus != lastBme68xStatus)
    {
        lastBme68xStatus = iaqSensor.bme68xStatus;
        if (iaqSensor.bme68xStatus < BME68X_OK)
        {
            Serial.println("BME68X error code : " + String(iaqSensor.bme68xStatus));
            errLeds();
        }
        else if (iaqSensor.bme68xStatus > BME68X_OK)
        {
            Serial.println("BME68X warning code : " + String(iaqSensor.bme68xStatus));
        }
    }
}

/**
 * @brief Restore BSEC calibration state from EEPROM.
 * Byte 0 holds the blob size as a "valid" marker, the blob follows.
 * An incompatible blob (e.g. after a BSEC update) is discarded instead of
 * being treated as a fatal error, to avoid a reboot loop.
 */
void loadBsecState(void)
{
    if (EEPROM.read(0) != BSEC_MAX_STATE_BLOB_SIZE)
    {
        Serial.println("No saved BSEC state, starting calibration from scratch");
        return;
    }

    for (uint16_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        bsecState[i] = EEPROM.read(i + 1);

    iaqSensor.setState(bsecState);
    if (iaqSensor.bsecStatus != BSEC_OK)
    {
        Serial.println("Saved BSEC state is invalid (code " + String(iaqSensor.bsecStatus) + "), erasing it");
        EEPROM.write(0, 0);
        EEPROM.commit();
        iaqSensor.bsecStatus = BSEC_OK;
        return;
    }
    Serial.println("BSEC state restored from EEPROM");
}

/**
 * @brief Save BSEC calibration state to EEPROM.
 * The first save happens as soon as the calibration is complete (accuracy 3),
 * then every BSEC_STATE_SAVE_PERIOD to limit flash wear.
 */
void updateBsecState(void)
{
    static bool savedOnce = false;
    static uint32_t lastSaveTime = 0;

    const bool firstSave = !savedOnce && iaqSensor.iaqAccuracy >= 3;
    const bool periodicSave = savedOnce && millis() - lastSaveTime >= BSEC_STATE_SAVE_PERIOD;
    if (!firstSave && !periodicSave)
        return;

    iaqSensor.getState(bsecState);
    if (iaqSensor.bsecStatus != BSEC_OK)
    {
        // Not fatal: just skip saving and retry on the next sample
        Serial.println("Failed to get BSEC state (code " + String(iaqSensor.bsecStatus) + ")");
        iaqSensor.bsecStatus = BSEC_OK;
        return;
    }

    for (uint16_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        EEPROM.write(i + 1, bsecState[i]);
    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();

    savedOnce = true;
    lastSaveTime = millis();
    Serial.println("BSEC state saved to EEPROM");
}

/**
 * @brief Blink the status LED for a while and restart the device.
 * Restarting (instead of halting forever) lets the station recover from
 * transient sensor failures and keeps it reachable for OTA updates.
 */
void errLeds(void)
{
    pinMode(LED_BUILTIN, OUTPUT);
    const uint32_t start = millis();
    while (millis() - start < ERROR_RESTART_DELAY)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
    ESP.restart();
}

/**
 * @brief Debounce a digital input.
 * The state is accepted only after the raw reading has stayed unchanged
 * for DIG_IN_DEBOUNCE_DELAY ms.
 *
 * @return true if the stable state has changed
 */
static bool debounceInput(uint8_t pin, uint8_t &stableState, uint8_t &lastRaw, uint32_t &lastChange)
{
    const uint8_t raw = digitalRead(pin);
    if (raw != lastRaw)
    {
        lastRaw = raw;
        lastChange = millis();
        return false;
    }
    if (raw != stableState && millis() - lastChange >= DIG_IN_DEBOUNCE_DELAY)
    {
        stableState = raw;
        return true;
    }
    return false;
}

void read_publish_inputs()
{
    static uint8_t last_raw_1 = HIGH, last_raw_2 = HIGH;
    static uint32_t last_change_1 = 0, last_change_2 = 0;

    if (debounceInput(DIG_INPUT_1_PIN, dig_input_1_state, last_raw_1, last_change_1))
    {
        mqttClient.publish(DEFAULT_TOPIC "dig_input_1", dig_input_1_state ? "1" : "0");
        Serial.println("dig_input_1_state changed: " + String(dig_input_1_state));
    }

    if (debounceInput(DIG_INPUT_2_PIN, dig_input_2_state, last_raw_2, last_change_2))
    {
        mqttClient.publish(DEFAULT_TOPIC "dig_input_2", dig_input_2_state ? "1" : "0");
        Serial.println("dig_input_2_state changed: " + String(dig_input_2_state));
    }
}

/**
 * @brief Connecting to MQTT server
 *
 * @return true if successfully reconnected to MQTT server, otherwise false
 */
uint8_t reconnect(void)
{
    if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                           MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
    {
        Serial.println("Successfully connected to " MQTT_SERVER);
        // Retained, so it overrides the retained "offline" last will message
        mqttClient.publish(MQTT_AVAILABILITY_TOPIC, MQTT_AVAILABILITY_MESSAGE, true);
        // Republish discovery when Home Assistant sends its "online" birth message
        if (!mqttClient.subscribe(HA_STATUS_TOPIC))
            Serial.println("Failed to subscribe to " HA_STATUS_TOPIC);
        if (!publish_discovery())
            return 0;
        return 1;
    }
    Serial.println("Can't connect to MQTT server...");
    return 0;
}

/**
 * @brief Handle incoming MQTT messages (only Home Assistant status for now)
 */
void mqtt_callback(char *topic, uint8_t *payload, unsigned int length)
{
    static const char online[] = "online";
    if (strcmp(topic, HA_STATUS_TOPIC) == 0 &&
        length == sizeof(online) - 1 && memcmp(payload, online, length) == 0)
    {
        Serial.println("Home Assistant is online, discovery will be republished");
        discoveryPending = true;
    }
}

/**
 * @brief Publish the retained Home Assistant discovery config.
 * The payload lives in flash and is streamed in small chunks, so no large
 * MQTT buffer is needed. If streaming fails midway the TCP connection is
 * closed: the broker drops the incomplete packet instead of storing a
 * partial (or empty) retained config, and discovery is retried on reconnect.
 *
 * @return true if the whole payload was sent
 */
bool publish_discovery(void)
{
    const size_t length = sizeof(HA_DISCOVERY_PAYLOAD) - 1; // without the trailing '\0'
    size_t sent = 0;

    if (mqttClient.beginPublish(HA_DISCOVERY_TOPIC, length, true))
    {
        uint8_t chunk[256];
        while (sent < length)
        {
            const size_t n = std::min(sizeof(chunk), length - sent);
            memcpy_P(chunk, HA_DISCOVERY_PAYLOAD + sent, n);
            const size_t written = mqttClient.write(chunk, n);
            sent += written;
            if (written != n)
                break;
        }
    }

    if (sent != length || !mqttClient.endPublish())
    {
        Serial.printf("Failed to publish discovery: %u of %u bytes sent\n", sent, length);
        espClient.stop();
        return false;
    }

    Serial.printf("Discovery published (%u bytes)\n", length);
    forceDataPublish = true;
    return true;
}

/**
 * @brief Publish a float value, skipping NaN/Inf (failed or not ready readings)
 */
static void publish_float(const char *topic, float value, const char *format = "%0.2f")
{
    if (isnan(value) || isinf(value))
        return;
    char buff[20];
    snprintf(buff, sizeof(buff), format, value);
    mqttClient.publish(topic, buff);
}

/**
 * @brief Publish data to server
 */
void publish_data(void)
{
    static uint32_t lastPublishedTimeStamp;
    static char buff[20];

    if (forceDataPublish || millis() - lastPublishedTimeStamp >= MQTT_PUBLISH_INTERVAL)
    {
        forceDataPublish = false;
        mqttClient.publish(MQTT_AVAILABILITY_TOPIC, MQTT_AVAILABILITY_MESSAGE, true);
        // Plain integer seconds, HA "duration" sensor
        snprintf(buff, sizeof(buff), "%lu", millis() / 1000UL);
        mqttClient.publish(MQTT_UPTIME_TOPIC, buff);
        snprintf(buff, sizeof(buff), "%d", (int)WiFi.RSSI());
        mqttClient.publish(MQTT_RSSI_TOPIC, buff);

        // Called only after iaqSensor.run() returned new data, so these are fresh
        publish_float(DEFAULT_TOPIC "temperature", iaqSensor.temperature);
        publish_float(DEFAULT_TOPIC "humidity", iaqSensor.humidity);
        if (iaqSensor.pressure > 0)
            publish_float(DEFAULT_TOPIC "pressure", iaqSensor.pressure / 100.0F);

        snprintf(buff, sizeof(buff), "%u", (unsigned)iaqSensor.staticIaqAccuracy);
        mqttClient.publish(DEFAULT_TOPIC "iaqAccuracy", buff);
        // With accuracy 0 BSEC is still stabilizing (or had a timing violation) and
        // reports default values (IAQ 25/50, CO2 500 ppm...), which are not real readings
        if (iaqSensor.staticIaqAccuracy > 0)
        {
            publish_float(DEFAULT_TOPIC "iaq", iaqSensor.staticIaq);
            publish_float(DEFAULT_TOPIC "co2Equivalent", iaqSensor.co2Equivalent);
            publish_float(DEFAULT_TOPIC "breathVocEquivalent", iaqSensor.breathVocEquivalent);
        }

        // Skip PM2.5 before the first full averaging window and if the sensor went silent
        if (state.valid && millis() - state.lastFrameMs < PM25_STALE_TIMEOUT)
        {
            snprintf(buff, sizeof(buff), "%u", state.avgPM25);
            mqttClient.publish(DEFAULT_TOPIC "pm25", buff);
        }
        // Publish the state of the digital inputs to ensure that the server
        // knows the current state of the inputs
        mqttClient.publish(DEFAULT_TOPIC "dig_input_1", dig_input_1_state ? "1" : "0");
        mqttClient.publish(DEFAULT_TOPIC "dig_input_2", dig_input_2_state ? "1" : "0");

        lastPublishedTimeStamp = millis();
        Serial.println("Data were sent");
    }
}