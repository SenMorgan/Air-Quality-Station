#pragma once

#include <Arduino.h>
#include "types.h"
#include "def.h"

#if defined(ESP8266)
#include <SoftwareSerial.h>
#endif

/**
 * Non-blocking reader of the IKEA VINDRIKTNING PM1006 sensor frames.
 *
 * Frame format (20 bytes):
 *   0x16 0x11 0x0B DF1 ... DF16 CS
 *   PM2.5 = DF3 << 8 | DF4 (bytes 5 and 6), sum of all 20 bytes == 0
 */
namespace SerialCom {
#if defined(ESP8266)
    static SoftwareSerial sensorSerial(IKEA_VKG_RX_PIN, IKEA_VKG_TX_PIN);
#else
    static HardwareSerial sensorSerial(1);
#endif

    static constexpr uint8_t FRAME_LEN = 20;
    static constexpr uint8_t HEADER[] = {0x16, 0x11, 0x0B};
    static constexpr uint8_t HEADER_LEN = sizeof(HEADER);
    static constexpr uint8_t AVG_WINDOW = sizeof(particleSensorState_t::measurements) /
                                          sizeof(particleSensorState_t::measurements[0]);

    static uint8_t serialRxBuf[FRAME_LEN];
    static uint8_t rxBufIdx = 0;

    inline void setup() {
#if defined(ESP8266)
        sensorSerial.begin(9600);
#else
        sensorSerial.begin(9600, SERIAL_8N1, IKEA_VKG_RX_PIN, IKEA_VKG_TX_PIN);
#endif
    }

    inline bool isValidChecksum() {
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < FRAME_LEN; i++) {
            checksum += serialRxBuf[i];
        }
        if (checksum != 0) {
            Serial.printf("Received message with invalid checksum: %u\n", checksum);
        }
        return checksum == 0;
    }

    inline void parseState(particleSensorState_t &state) {
        const uint16_t pm25 = (serialRxBuf[5] << 8) | serialRxBuf[6];
        Serial.printf("Received PM 2.5 reading: %u\n", pm25);

        state.measurements[state.measurementIdx] = pm25;
        state.measurementIdx = (state.measurementIdx + 1) % AVG_WINDOW;

        if (state.measurementIdx == 0) {
            uint32_t sum = 0;
            for (uint8_t i = 0; i < AVG_WINDOW; ++i) {
                sum += state.measurements[i];
            }
            state.avgPM25 = (sum + AVG_WINDOW / 2) / AVG_WINDOW;
            state.valid = true;
            Serial.printf("New Avg PM25: %u\n", state.avgPM25);
        }
    }

    /**
     * @brief Consume all available bytes without blocking.
     * Bytes are accumulated across calls until a full frame is received,
     * the header is used to resynchronize on garbage.
     */
    inline void handleUart(particleSensorState_t &state) {
        while (sensorSerial.available()) {
            const uint8_t b = sensorSerial.read();

            if (rxBufIdx < HEADER_LEN && b != HEADER[rxBufIdx]) {
                // Header mismatch: resync, the byte may be a start of a new frame
                rxBufIdx = 0;
                if (b != HEADER[0]) {
                    continue;
                }
            }

            serialRxBuf[rxBufIdx++] = b;

            if (rxBufIdx == FRAME_LEN) {
                if (isValidChecksum()) {
                    parseState(state);
                }
                rxBufIdx = 0;
            }
        }
    }
} // namespace SerialCom
