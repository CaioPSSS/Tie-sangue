#ifndef CRC_H
#define CRC_H

#include <Arduino.h>

class CRC {
public:
    // Calcula um Checksum CRC8 rápido e leve para o ESP32
    static uint8_t calculateCRC8(const uint8_t *data, size_t len) {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
};

#endif