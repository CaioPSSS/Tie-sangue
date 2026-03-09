#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h> // Biblioteca: sandeepmistry/LoRa
#include "../common/SharedTypes.h"
#include "../utils/CRC.h"

// Definição dos Pinos SPI para o RA-02 SX1278 no ESP32
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK  18
#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 26 // Pino de interrupção (Opcional, mas bom conectar)

class LoRaManager {
public:
    // Inicializa o módulo e configura a física (Frequência, SF, BW)
    static bool init();

    // Envia a struct compactada
    static void sendTelemetry(PacketTelemetryLoRa_t &packet);
};

#endif