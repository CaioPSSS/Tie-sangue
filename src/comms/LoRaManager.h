#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h> 
#include "../common/SharedTypes.h"
#include "../utils/CRC.h"

#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK  18
#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 26

// Enumeração para identificar o que chegou pelo rádio
enum LoRaPacketType {
    PACKET_NONE = 0,
    PACKET_RC_UPLINK,
    PACKET_WAYPOINT
};

class LoRaManager {
public:
    static bool init();
    static void sendTelemetry(PacketTelemetryLoRa_t &packet);

    // Agora a função lê o buffer e devolve o tipo exato de pacote validado
    static LoRaPacketType receive(PacketUplinkLoRa_t &rcData, PacketWaypointLoRa_t &wpData, int8_t &rssi);
};

#endif