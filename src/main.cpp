#include <Arduino.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "bsec.h"
#include "def.h"
#include "SerialCom.h"
#include "Types.h"

const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASSWD;

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

particleSensorState_t state;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
uint8_t reconnect(void);
void publish_data(void);

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;

void setup(void)
{
    Serial.begin(115200);
    SerialCom::setup();

    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(LED_BUILTIN, OUTPUT);

    iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." +
             String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) +
             "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();

    iaqSensor.setConfig(bsec_config_iaq);
    checkIaqSensorStatus();

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
    WiFi.begin(ssid, pass);
    WiFi.hostname(HOSTNAME);

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
    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
    Serial.println("Connecting to MQTT server...");
    reconnect();

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

    if (iaqSensor.run())
    { // If new data is available
        output = String(millis());
        output += ", " + String(iaqSensor.rawTemperature);
        output += ", " + String(iaqSensor.pressure);
        output += ", " + String(iaqSensor.rawHumidity);
        output += ", " + String(iaqSensor.gasResistance);
        output += ", " + String(iaqSensor.staticIaq);
        output += ", " + String(iaqSensor.staticIaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        output += ", " + String(iaqSensor.staticIaq);
        output += ", " + String(iaqSensor.co2Equivalent);
        output += ", " + String(iaqSensor.breathVocEquivalent);
        Serial.println(output);
        if (connectedToServer)
        {
            publish_data();
        }
        else
        {
            if (reconnect())
            {
                publish_data();
            }
        }
    }
    else
    {
        checkIaqSensorStatus();
    }

    yield();
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
    if (iaqSensor.bsecStatus != BSEC_OK)
    {
        if (iaqSensor.bsecStatus < BSEC_OK)
        {
            output = "BSEC error code : " + String(iaqSensor.bsecStatus);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        }
        else
        {
            output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
            Serial.println(output);
        }
    }

    if (iaqSensor.bme68xStatus != BME68X_OK)
    {
        if (iaqSensor.bme68xStatus < BME68X_OK)
        {
            output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        }
        else
        {
            output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
            Serial.println(output);
        }
    }
}

void errLeds(void)
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
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
        return 1;
    }
    Serial.println("Can't connect to MQTT server...");
    return 0;
}

/**
 * @brief Publish data to server
 */
void publish_data(void)
{
    static uint32_t lastPublishedTimeStamp;
    static char buff[20];

    if (millis() - lastPublishedTimeStamp >= MQTT_PUBLISH_INTERVAL)
    {
        mqttClient.publish(MQTT_AVAILABILITY_TOPIC, MQTT_AVAILABILITY_MESSAGE);
        sprintf(buff, "%ld sec", millis() / 1000);
        mqttClient.publish(MQTT_UPTIME_TOPIC, buff);
        sprintf(buff, "%0.2f", iaqSensor.temperature);
        mqttClient.publish(DEFAULT_TOPIC "temperature", buff);
        sprintf(buff, "%0.2f", iaqSensor.humidity);
        mqttClient.publish(DEFAULT_TOPIC "humidity", buff);
        sprintf(buff, "%0.2f", iaqSensor.pressure / 100.0F);
        mqttClient.publish(DEFAULT_TOPIC "pressure", buff);
        sprintf(buff, "%0.2f", iaqSensor.staticIaq);
        mqttClient.publish(DEFAULT_TOPIC "iaq", buff);
        sprintf(buff, "%d", iaqSensor.staticIaqAccuracy);
        mqttClient.publish(DEFAULT_TOPIC "iaqAccuracy", buff);
        sprintf(buff, "%0.2f", iaqSensor.co2Equivalent);
        mqttClient.publish(DEFAULT_TOPIC "co2Equivalent", buff);
        sprintf(buff, "%0.2f", iaqSensor.breathVocEquivalent);
        mqttClient.publish(DEFAULT_TOPIC "breathVocEquivalent", buff);
        sprintf(buff, "%d", state.avgPM25);
        mqttClient.publish(DEFAULT_TOPIC "pm25", buff);

        lastPublishedTimeStamp = millis();
        Serial.println("Data were sent");
    }
}