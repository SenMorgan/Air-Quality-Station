#pragma once

struct particleSensorState_t {
    uint16_t avgPM25 = 0;
    uint16_t measurements[5] = {0, 0, 0, 0, 0};
    uint8_t measurementIdx = 0;
    boolean valid = false;
    uint32_t lastFrameMs = 0; // millis() of the last valid frame, to detect a dead sensor
};
